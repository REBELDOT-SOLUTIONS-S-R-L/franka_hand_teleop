"""Disarmed-by-default FR3 commissioning bridge. No X4 safety input is emulated."""
import json
import math
import os
import socket
import time
import xml.etree.ElementTree as ET
from dataclasses import replace

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SwitchController,
)
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Point, Pose, TwistStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    PlanningScene,
    ServoStatus,
)
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .core import SETTLE_TIME, Teleop, capped, decode, finite
from .deployment import (
    REAL_ARM_CONTROLLER,
    REAL_DOMAIN,
    REAL_NAMESPACE,
    REAL_PLUGIN,
    REAL_PORT,
    MotionPermit,
    bounded_increment,
    load_profile,
    tracking_fresh,
)
from .diagnostics import RobotHistory
from .grasp import GraspGuard, GripperCommands
from .workspace_guard import WorkspaceGuard


def seconds(value):
    return value.sec + value.nanosec*1e-9


def duration(value):
    ns = round(value*1e9)
    return Duration(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


class RealBridge(Node):
    EXPECTED_PLUGIN = REAL_PLUGIN
    EXPECTED_CONTROLLER = REAL_ARM_CONTROLLER

    def __init__(self, **kwargs):
        if os.environ.get("ROS_DOMAIN_ID") != REAL_DOMAIN:
            raise RuntimeError(f"Real bridge requires domain {REAL_DOMAIN}")
        super().__init__("hand_bridge", **kwargs)
        if self.get_namespace() != REAL_NAMESPACE:
            raise RuntimeError(f"Real bridge requires namespace {REAL_NAMESPACE}")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_description_semantic", "")
        self.declare_parameter("profile", "")
        self.declare_parameter("execute", False)
        self.execute = self.get_parameter("execute").value
        self.profile, self.limits = load_profile(self.get_parameter("profile").value, self.execute)
        self.motion = self.profile["motion"]
        self.get_logger().info("Motion settings: "+str(self.profile.get("_source_profile",
            self.get_parameter("profile").value))+f"; max_speed={self.motion['max_speed']:.3f} m/s; "
            f"max_acceleration={self.motion['max_acceleration']:.3f} m/s²; restart to reload")
        xml = self.get_parameter("robot_description").value
        plugins = [p.text for p in ET.fromstring(xml).findall("ros2_control/hardware/plugin")]
        if plugins != [self.EXPECTED_PLUGIN]:
            raise ValueError("Real bridge refuses a mismatched hardware model")
        # Extra geometric reserve for measured/desired deviation and cancellation.
        guarded = replace(self.limits, minimum=tuple(v+.01 for v in self.limits.minimum),
                          maximum=tuple(v-.01 for v in self.limits.maximum),
                          body_minimum=tuple(v+.01 for v in self.limits.body_minimum),
                          body_maximum=tuple(v-.01 for v in self.limits.body_maximum))
        self.guard = WorkspaceGuard(xml, guarded)
        semantic = ET.fromstring(self.get_parameter("robot_description_semantic").value)
        self.allowed_pairs = [(p.get("link1"), p.get("link2")) for p in semantic.findall("disable_collisions")]
        self.allowed_pairs += [("installation_table", n) for n in ("fr3_link0", "fr3_link0_sc")]
        self.logic, self.permit = Teleop(self.limits), MotionPermit()
        self.grasp = GraspGuard() if self.profile["enable_gripper"] else None
        self.gripper_commands = GripperCommands()
        self.motion_fence = -math.inf
        self.last_grasp_phase = "normal"
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", REAL_PORT))
        self.socket.setblocking(False)
        self.camera_peer = None
        self.camera_feedback_sent = -math.inf
        self.operator_result = None
        self.robot = None
        self.robot_history = RobotHistory()
        self.diagnostic_error = None
        self.baseline_diagnostic_logged = False
        self.robot_stamp = self.backend_stamp = self.controllers_stamp = -math.inf
        self.max_robot_feedback_gap = 0.
        self.last_stop_reason = ""
        self.backend_reason = "Hardware verification not received"
        self.controllers_reason = "Controller verification not received"
        self.robot_time = -math.inf
        self.joints, self.speeds, self.desired, self.desired_speeds = {}, {}, {}, {}
        self.finger_width = None
        self.finger_stamp = -math.inf
        self.servo_status = None
        self.servo_stamp = -math.inf
        self.switch_ok = False
        self.hardware_future = self.controllers_future = self.servo_future = None
        self.activation_future = None
        self.activation_deadline = -math.inf
        self.controller_state = "unknown"
        self.rotation_anchor = None
        self.ready = False
        self.reason = "Waiting for verified robot feedback"
        self.guard_rejections = 0
        self.arm_actions_completed = 0
        self.last_segment = None
        self.last_motion_result = None
        self.last_arm_finish = -math.inf
        self.arm_not_before = -math.inf
        self.actions = {}
        self.hardware = self.create_client(ListHardwareComponents, "controller_manager/list_hardware_components")
        self.controllers = self.create_client(ListControllers, "controller_manager/list_controllers")
        self.servo_switch = self.create_client(ServoCommandType, "servo_node/switch_command_type")
        self.activation = (self.create_client(SwitchController, "controller_manager/switch_controller")
                           if self.execute else None)
        self.arm_action = (ActionClient(self, FollowJointTrajectory, "fr3_arm_controller/follow_joint_trajectory")
                           if self.execute else None)
        self.gripper_action = (ActionClient(self, Move, "franka_gripper/move")
                               if self.execute and self.profile["enable_gripper"] else None)
        self.grasp_action = (ActionClient(self, Grasp, "franka_gripper/grasp")
                             if self.execute and self.profile["enable_gripper"] else None)
        self.create_subscription(FrankaRobotState, "franka_robot_state_broadcaster/robot_state",
                                 self.receive_robot, qos_profile_sensor_data)
        self.create_subscription(JointState, "joint_states", self.receive_fingers, qos_profile_sensor_data)
        self.create_subscription(ServoStatus, "servo_node/status", self.receive_servo, 1)
        self.create_subscription(JointTrajectory, "servo_trajectory", self.receive_trajectory, 1)
        self.create_service(SetBool, "~/arm", self.arm_request)
        self.twist = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 1)
        self.status = self.create_publisher(String, "status", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene = self.create_publisher(PlanningScene, "planning_scene", latched)
        self.markers = self.create_publisher(MarkerArray, "markers", latched)
        self.last_tick = self.last_visual = time.monotonic()
        self.create_timer(.02, self.tick)
        self.create_timer(.25, self.check_backend)
        self.get_logger().info("REAL commissioning bridge: " +
                               ("motion available, DISARMED" if self.execute else "PREVIEW; no motion output"))

    def stop(self, reason, report=False):
        capture_diagnostics = self.permit.armed or report
        if "arm" in self.actions:
            # Action cancellation is acknowledged before the controller has
            # finished braking. Do not replace that brake with a new goal.
            self.arm_not_before = max(self.arm_not_before, time.monotonic()+1.6)
        if self.permit.armed or report:
            self.last_stop_reason = reason
            rate = self.robot.control_command_success_rate if self.robot else math.nan
            self.get_logger().warning(
                f"TELEOP STOP: {reason}; last FCI command success={rate:.3f}; "
                f"feedback age={time.monotonic()-self.robot_stamp:.3f}s; "
                f"max feedback gap={self.max_robot_feedback_gap:.3f}s")
        self.permit.stop(reason)
        self.logic.stop(reason)
        self.gripper_commands = GripperCommands()
        if self.grasp:
            self.grasp.interrupt()
        for item in self.actions.values():
            if item.get("handle") and not item.get("cancelled"):
                item["handle"].cancel_goal_async()
                item["cancelled"] = True
        if capture_diagnostics:
            # Diagnostics run only after revoking the permit/cancelling actions.
            # A malformed telemetry field must never interfere with stopping.
            try:
                detail = {"reason": reason, **self.robot_history.summary(time.monotonic()),
                          "controller_rejection": getattr(self, "last_rejection", None),
                          "diagnostic_error": self.diagnostic_error}
                self.get_logger().info("STOP DIAGNOSTIC: "+json.dumps(detail, allow_nan=False))
            except (AttributeError, IndexError, TypeError, ValueError, OverflowError, RuntimeError) as error:
                self.diagnostic_error = str(error)

    def receive_robot(self, message):
        age = self.get_clock().now().nanoseconds*1e-9 - seconds(message.header.stamp)
        if not -.02 <= age < .15 or not finite(message.time) or message.time <= self.robot_time:
            self.stop("Invalid, stale or repeated robot feedback")
            return
        q = dict(zip(message.measured_joint_state.name, message.measured_joint_state.position))
        dq = dict(zip(message.measured_joint_state.name, message.measured_joint_state.velocity))
        desired = dict(zip(message.desired_joint_state.name, message.desired_joint_state.position))
        desired_dq = dict(zip(message.desired_joint_state.name, message.desired_joint_state.velocity))
        if not all(n in q and n in dq and n in desired and n in desired_dq and
                   all(map(finite, (q[n], dq[n], desired[n], desired_dq[n])))
                   for n in self.guard.names):
            self.stop("Incomplete robot joint feedback")
            return
        self.robot, self.robot_time = message, message.time
        self.joints, self.speeds, self.desired, self.desired_speeds = q, dq, desired, desired_dq
        now = time.monotonic()
        if math.isfinite(self.robot_stamp):
            self.max_robot_feedback_gap = max(self.max_robot_feedback_gap, now-self.robot_stamp)
        self.robot_stamp = now
        try:
            previous_load = self.robot_history.load
            self.robot_history.observe(message, now)
            if not self.baseline_diagnostic_logged or previous_load != self.robot_history.load:
                self.get_logger().info("ROBOT BASELINE DIAGNOSTIC: "+
                                       json.dumps(self.robot_history.summary(now), allow_nan=False))
                self.baseline_diagnostic_logged = True
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError) as error:
            self.diagnostic_error = str(error)

    def receive_fingers(self, message):
        values = dict(zip(message.name, message.position))
        names = ("fr3_finger_joint1", "fr3_finger_joint2")
        age = self.get_clock().now().nanoseconds*1e-9 - seconds(message.header.stamp)
        if -.02 <= age < .15 and all(n in values and finite(values[n]) and 0 <= values[n] <= .04
                                    for n in names):
            self.finger_width, self.finger_stamp = sum(values[n] for n in names), time.monotonic()

    def receive_servo(self, message):
        self.servo_status, self.servo_stamp = message, time.monotonic()

    def check_backend(self):
        if self.hardware_future and self.hardware_future.done():
            result = self.hardware_future.result()
            items = result.component if result else []
            valid = len(items) == 1 and items[0].name == "FrankaHardwareInterface" and (
                items[0].plugin_name or items[0].class_type) == self.EXPECTED_PLUGIN and (
                    items[0].state.label == "active")
            self.backend_stamp = time.monotonic() if valid else -math.inf
            details = ", ".join(
                    f"{item.name} plugin={item.plugin_name or item.class_type} state={item.state.label}"
                    for item in items) or "component missing"
            self.backend_reason = "Hardware verified" if valid else "Hardware identity/state rejected: "+details
            self.hardware_future = None
            if not valid:
                self.stop("Hardware identity/state rejected")
        if self.hardware_future is None and self.hardware.service_is_ready():
            self.hardware_future = self.hardware.call_async(ListHardwareComponents.Request())
        if self.controllers_future and self.controllers_future.done():
            result = self.controllers_future.result()
            states = {c.name: c.state for c in result.controller} if result else {}
            active = {n for n, state in states.items() if state == "active"}
            arm_type = next((c.type for c in result.controller if c.name == "fr3_arm_controller"), "") if result else ""
            permitted = {"joint_state_broadcaster", "franka_robot_state_broadcaster", "fr3_arm_controller"}
            valid = ({"joint_state_broadcaster", "franka_robot_state_broadcaster"} <= active <= permitted
                     and arm_type == self.EXPECTED_CONTROLLER)
            self.controllers_stamp = time.monotonic() if valid else -math.inf
            self.controllers_reason = ("Controllers verified" if valid else
                "Controller verification rejected: arm type="+arm_type+"; active="+", ".join(sorted(active)))
            self.controller_state = states.get("fr3_arm_controller", "unknown")
            self.controllers_future = None
        if self.controllers_future is None and self.controllers.service_is_ready():
            self.controllers_future = self.controllers.call_async(ListControllers.Request())
        if self.servo_future and self.servo_future.done():
            result = self.servo_future.result()
            self.switch_ok, self.servo_future = bool(result and result.success), None
        if (not self.switch_ok and self.servo_future is None and self.servo_switch.service_is_ready()
                and time.monotonic()-self.backend_stamp < .75):
            self.servo_future = self.servo_switch.call_async(
                ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST))

    def readiness(self, now):
        if now-self.backend_stamp >= .75:
            return False, (self.backend_reason if not math.isfinite(self.backend_stamp) else
                           "Hardware verification stale (no fresh service response)")
        if now-self.controllers_stamp >= .75:
            return False, (self.controllers_reason if not math.isfinite(self.controllers_stamp) else
                           "Controller verification stale (no fresh service response)")
        if self.controller_state not in ("active", "inactive"):
            return False, "Arm controller not configured"
        if self.permit.armed and self.controller_state != "active":
            return False, "Arm controller is not active"
        if self.robot is None or now-self.robot_stamp >= .15:
            return False, "Robot feedback stale"
        if self.robot.robot_mode not in (FrankaRobotState.ROBOT_MODE_IDLE, FrankaRobotState.ROBOT_MODE_MOVE):
            return False, "Robot is stopped, guiding, recovering or in reflex"
        errors = [k for k in self.robot.current_errors.get_fields_and_field_types()
                  if getattr(self.robot.current_errors, k)]
        if errors:
            return False, "Robot reports current errors: "+", ".join(errors)+"; manual recovery required"
        collisions = self.robot.collision_indicators
        if (any(collisions.is_joint_collision) or any(getattr(v, axis) for v in
                (collisions.is_cartesian_linear_collision, collisions.is_cartesian_angular_collision)
                for axis in ("x", "y", "z"))):
            return False, "Robot reports collision"
        if not self.guard.inside(self.guard.tcp(self.joints)):
            return False, "TCP outside commissioning workspace/clearance"
        if self.grasp and self.grasp.floor_for_path() is not None and (
                self.guard.tcp(self.joints)[2] < self.grasp.floor_for_path()-.001):
            return False, "Measured TCP below grasp Z floor; manual inspection required"
        body_ok, reason = self.guard.body_inside(self.joints)
        if not body_ok:
            return False, reason
        # Compare actual FCI EE and model TCP; do not silently assume the tool transform.
        pose = self.robot.o_t_ee.pose
        actual = (pose.position.x, pose.position.y, pose.position.z)
        model = self.guard.pose(self.joints)
        q = pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        if (not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > .001 or
                not all(map(finite, actual)) or math.dist(actual, model[:3, 3]) > .005):
            return False, "FCI end-effector position disagrees with model TCP"
        x, y, z, w = quat
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        angle = math.acos(float(np.clip((np.trace(rotation.T@model[:3, :3])-1)/2, -1, 1)))
        if angle > .03:
            return False, "FCI end-effector orientation disagrees with model TCP"
        if not self.switch_ok:
            return False, "Servo command interface not verified"
        if self.servo_status is None or now-self.servo_stamp >= .15:
            return False, "Servo status missing or stale"
        if self.servo_status.code in (ServoStatus.INVALID, ServoStatus.HALT_FOR_SINGULARITY,
                                          ServoStatus.HALT_FOR_COLLISION, ServoStatus.JOINT_BOUND):
            return False, f"Servo halted ({self.servo_status.code}): {self.servo_status.message}"
        if self.gripper_action:
            if now-self.finger_stamp >= .15:
                return False, f"Gripper feedback stale ({(now-self.finger_stamp)*1000:.1f} ms; limit 150 ms)"
            if not self.gripper_action.server_is_ready() or not self.grasp_action.server_is_ready():
                return False, "Gripper Move/Grasp action server unavailable"
        return True, "Ready"

    def arm_request(self, request, response):
        response = self.apply_arm_request(request, response)
        self.operator_result = {"monotonic_s": time.monotonic(), "success": response.success,
                                "message": response.message, "action": "arm" if request.data else "disarm"}
        return response

    def apply_arm_request(self, request, response):
        if not request.data:
            self.stop("Disarmed by operator")
            response.success, response.message = True, self.permit.reason
            return response
        if not self.execute:
            response.success, response.message = False, "Preview has no motion output"
            return response
        now = time.monotonic()
        if now < self.arm_not_before:
            response.success, response.message = False, "Previous bounded stop may still be executing; wait before re-arming"
            return response
        ready, reason = self.readiness(now)
        if (not ready or not tracking_fresh(self.logic.tracking, now) or self.actions or self.activation_future
                or not self.activation.service_is_ready() or not self.arm_server_ready()):
            response.success, response.message = False, reason if not ready else "Tracking/previous action not ready"
            return response
        actual = self.guard.tcp(self.joints)
        target = self.motion_target(actual)
        stationary = self.stationary()
        valid, reason = self.permit.arm(now, actual, target, stationary, self.motion["arm_capture_distance"])
        if not valid:
            response.success, response.message = False, reason
            return response
        self.rotation_anchor = self.robot.o_t_ee.pose.orientation
        if self.controller_state == "inactive":
            self.permit.armed = False
            generation = self.permit.generation
            future = self.activation.call_async(SwitchController.Request(
                activate_controllers=["fr3_arm_controller"], strictness=SwitchController.Request.STRICT,
                timeout=Duration(sec=2)))
            self.activation_future = future
            self.activation_deadline = now+2.5

            def activated(done):
                self.activation_future = None
                result = done.result()
                if result and result.ok and self.permit.generation == generation:
                    # Discard a controller-list request issued before activation;
                    # its inactive snapshot must not invalidate the new permit.
                    if self.controllers_future:
                        self.controllers_future.cancel()
                        self.controllers_future = None
                    self.controller_state = "active"
                    ok, why = self.readiness(time.monotonic())
                    if ok and tracking_fresh(self.logic.tracking, time.monotonic()):
                        ok, why = self.permit.arm(time.monotonic(), self.guard.tcp(self.joints),
                            self.motion_target(self.guard.tcp(self.joints)),
                            self.stationary(), self.motion["arm_capture_distance"])
                        if ok:
                            self.logic.valid_since = time.monotonic()-SETTLE_TIME
                            return
                    self.stop(why)
                else:
                    self.stop("Controller activation failed or arming was interrupted")
            future.add_done_callback(activated)
            response.success, response.message = True, "Controller activation requested; inspect armed status"
        else:
            self.logic.valid_since = now-SETTLE_TIME
            response.success, response.message = True, "Armed"
        return response

    def arm_server_ready(self):
        return self.arm_action is not None and self.arm_action.server_is_ready()

    def stationary(self):
        return all(abs(self.speeds[n]) < .002 and abs(self.desired_speeds[n]) < .0001 and
                   abs(self.desired[n]-self.joints[n]) < .002 for n in self.guard.names)

    def dispatch(self, key, client, goal, timeout):
        if not self.execute or not self.permit.armed or key in self.actions or not client.server_is_ready():
            return
        item = {"generation": self.permit.generation, "deadline": time.monotonic()+timeout,
                "opening_move": key == "gripper" and isinstance(goal, Move.Goal)}
        if key == "arm":
            item["tcp_start"] = self.guard.tcp(self.joints)
            item["target"] = self.limits.target(self.logic.tracking["control_uvd"])
        self.actions[key] = item
        future = client.send_goal_async(goal)

        def accepted(done):
            handle = done.result()
            if not handle or not handle.accepted:
                self.actions.pop(key, None)
                self.stop(f"{key} action rejected")
                return
            item["handle"] = handle
            if (not self.permit.armed or item["generation"] != self.permit.generation or
                    item.get("cancel_for_grasp")):
                handle.cancel_goal_async()
                item["cancelled"] = True
            result = handle.get_result_async()

            def finished(done):
                if self.actions.get(key) is item:
                    self.actions.pop(key, None)
                answer = done.result()
                success = bool(answer and answer.status == GoalStatus.STATUS_SUCCEEDED and
                               (answer.result.error_code == 0 if key == "arm" else answer.result.success))
                current = self.permit.armed and item["generation"] == self.permit.generation
                if item["opening_move"] and current:
                    self.gripper_commands.finished(time.monotonic())
                expected_cancel = (item.get("cancel_for_grasp") and answer and
                                   answer.status == GoalStatus.STATUS_CANCELED)
                if success and key == "gripper" and isinstance(goal, Grasp.Goal) and current:
                    if (time.monotonic()-self.finger_stamp >= .15 or
                            not .002 < self.finger_width < .079):
                        self.stop("Grasp result lacks fresh nonempty object width; lift blocked")
                        return
                    actual = self.guard.tcp(self.joints)
                    ready, reason = self.readiness(time.monotonic())
                    if (not ready or not tracking_fresh(self.logic.tracking, time.monotonic()) or
                            math.dist(actual, self.grasp.anchor) > .003):
                        self.stop("Grasp completed without a fresh, valid TCP hold: "+reason)
                        return
                    self.grasp.confirmed(actual, self.limits.target(self.logic.tracking["control_uvd"]),
                                         self.guard.tcp(self.desired)[2])
                    self.motion_fence = self.get_clock().now().nanoseconds*1e-9
                    self.logic.velocity = (0., 0., 0.)
                    self.get_logger().info(f"GRASP CONFIRMED: width={self.finger_width*1000:.1f} mm; "
                        f"grasp Z={self.grasp.anchor[2]*100:.1f} cm; lift=5 cm; floor margin=1 cm; "
                        f"held measured-command Z offset={self.grasp.command_z_offset*1000:.3f} mm")
                if success and key == "arm":
                    self.arm_actions_completed += 1
                if key == "arm":
                    self.last_arm_finish = time.monotonic()
                    actual = self.guard.tcp(self.joints)
                    progress = math.dist(item["tcp_start"], item["target"])-math.dist(actual, item["target"])
                    self.last_motion_result = {"success": success,
                        "tcp_displacement_m": math.dist(item["tcp_start"], actual),
                        "target_progress_m": progress}
                    self.get_logger().info("STEP RESULT: "+json.dumps(self.last_motion_result))
                if not success and current and not expected_cancel:
                    self.stop(f"{key} action did not complete successfully")
            result.add_done_callback(finished)
        future.add_done_callback(accepted)

    def receive_trajectory(self, message):
        now = time.monotonic()
        if (not self.execute or not self.permit.armed or "arm" in self.actions or
                (self.grasp and self.grasp.holding) or seconds(message.header.stamp) <= self.motion_fence):
            return
        ready, reason = self.readiness(now)
        if not ready or not tracking_fresh(self.logic.tracking, now):
            self.stop(reason if not ready else "Tracking expired before trajectory")
            return
        try:
            age = self.get_clock().now().nanoseconds*1e-9 - seconds(message.header.stamp)
            if not -.02 <= age <= .10 or set(message.joint_names) != set(self.guard.names) or (
                    len(message.joint_names) != len(self.guard.names) or len(message.points) != 2 or
                    message.header.frame_id != "fr3_link0" or seconds(message.points[0].time_from_start) != 0):
                raise ValueError("Invalid/stale seeded Servo proposal (two points required)")
            if any(len(p.positions) != len(message.joint_names) for p in message.points):
                raise ValueError("Incomplete Servo positions")
            if not self.stationary():
                if now-self.last_arm_finish > .5 and any(
                        abs(self.desired[n]-self.joints[n]) >= .002 for n in self.guard.names):
                    raise ValueError("Desired/measured joints did not settle within 0.002 rad")
                return  # Wait for the previous bounded step to settle.
            start = [self.desired[n] for n in self.guard.names]
            seed, proposed = [[p.positions[message.joint_names.index(n)] for n in self.guard.names]
                              for p in message.points]
            if any(not self.guard.bounds[n][0] <= q <= self.guard.bounds[n][1]
                   for n, q in zip(self.guard.names, proposed)):
                raise ValueError("Joint limit in Servo proposal")
            horizon = seconds(message.points[1].time_from_start)
            target, total, delta = bounded_increment(seed, proposed, start,
                [self.joints[n] for n in self.guard.names], horizon, self.motion, self.guard.reach)
            if max(map(abs, delta)) < 1e-6:
                return
            valid, reason = self.guard.check(self.joints, [start, target])
            if not valid:
                raise ValueError(reason)
            if self.grasp and not self.grasp.accepts_heights(
                    self.guard.tcp(dict(zip(self.guard.names, start+(np.array(target)-start)*f)))[2]
                    for f in np.linspace(0., 1., 65)):
                return
            self.last_segment = {"seed_q": seed, "desired_start_q": start,
                "desired_minus_measured_q": [self.desired[n]-self.joints[n] for n in self.guard.names],
                "proposal_delta_q": [b-a for a, b in zip(seed, proposed)],
                "command_delta_q": delta, "proposal_horizon_s": horizon, "duration_s": total,
                "expected_tcp_displacement_m": math.dist(self.guard.tcp(self.joints),
                    self.guard.tcp(dict(zip(self.guard.names, target)))),
                "servo_status": self.servo_status.code}
            self.get_logger().info("STEP PLAN: "+json.dumps(self.last_segment))
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.guard.names
            goal.trajectory.points = [
                JointTrajectoryPoint(positions=start, velocities=[0.0]*7, accelerations=[0.0]*7),
                JointTrajectoryPoint(positions=target, velocities=[0.0]*7, accelerations=[0.0]*7,
                                     time_from_start=duration(total)),
            ]
            self.dispatch("arm", self.arm_action, goal, total+.75)
        except (ValueError, IndexError) as error:
            self.guard_rejections += 1
            self.stop(str(error))

    def motion_target(self, actual):
        raw = self.limits.target(self.logic.tracking["control_uvd"])
        return self.grasp.target(raw, actual, self.guard.limits) if self.grasp else raw

    def arm_tail_finished(self):
        return "arm" not in self.actions and time.monotonic() >= self.arm_not_before

    def tick_gripper(self, now, actual):
        if not (self.gripper_action and self.permit.armed and self.ready and self.logic.active):
            return
        policy = self.grasp
        opening = self.logic.tracking["opening"]
        intent = self.gripper_commands
        intent.observe(opening, now)
        if policy.phase == "settling" and intent.open_ready:
            policy.released(actual, self.limits.target(self.logic.tracking["control_uvd"]))
            self.motion_fence = self.get_clock().now().nanoseconds*1e-9
            self.logic.velocity = (0., 0., 0.)
            return
        if policy.phase == "closing" and intent.open_ready:
            self.stop("Grasp cancelled by open gesture; re-arm with an open hand to release")
            return
        if policy.phase == "closing" and math.dist(actual, policy.anchor) > .003:
            self.stop("TCP drifted more than 3 mm during grasp hold")
            return
        if policy.phase in ("lift", "carry", "interrupted") and intent.open_ready:
            policy.opening_requested = True
        # Close intent owns the arm hold immediately, even during an opening Move.
        # Cancel that Move and wait for its final result before sending Grasp.
        if (policy.phase == "normal" and intent.close_ready and
                self.finger_width > .002):
            try:
                policy.begin(actual, now, self.limits.maximum[2]-self.limits.margin)
            except ValueError as error:
                self.stop(str(error))
                return
            intent.reset_moves()
            self.motion_fence = self.get_clock().now().nanoseconds*1e-9
            self.logic.velocity = (0., 0., 0.)
            item = self.actions.get("gripper")
            if item and item["opening_move"]:
                item["cancel_for_grasp"] = True
                if item.get("handle") and not item.get("cancelled"):
                    item["handle"].cancel_goal_async()
                    item["cancelled"] = True
            return
        if "gripper" in self.actions:
            return
        desired = .08*opening
        try:
            if policy.opening_requested:
                if self.finger_width >= .0795:
                    policy.released(actual, self.limits.target(self.logic.tracking["control_uvd"]))
                    self.motion_fence = self.get_clock().now().nanoseconds*1e-9
                    self.logic.velocity = (0., 0., 0.)
                    self.get_logger().info("GRASP RELEASED: Hand open; grasp Z floor cleared")
                    return
                desired = .08
            elif policy.phase == "settling":
                if policy.settled(actual, now, self.stationary(), self.arm_tail_finished()):
                    if actual[2]+policy.LIFT > self.limits.maximum[2]-self.limits.margin:
                        raise ValueError("Settled grasp pose lacks 5 cm lift headroom")
                    goal = Grasp.Goal(width=0., speed=self.profile["gripper_speed_m_s"], force=self.profile["grasp_force_N"])
                    goal.epsilon.inner, goal.epsilon.outer = .001, .08
                    self.get_logger().info(f"GRASP REQUEST: force={goal.force:.1f} N; speed={goal.speed*1000:.1f} mm/s; "
                                           f"held TCP={policy.anchor}")
                    self.dispatch("gripper", self.grasp_action, goal, self.finger_width/self.profile["gripper_speed_m_s"]+2.)
                    if "gripper" not in self.actions:
                        raise ValueError("Grasp server became unavailable before closure")
                return
            elif policy.phase != "normal":
                return
            if policy.phase == "normal" and opening <= policy.CLOSE:
                return  # Pending close debounce must never generate another opening.
            speed = self.profile["gripper_speed_m_s"]
            width = intent.opening_goal(desired, self.finger_width, speed, self.finger_stamp)
            if width is not None:
                self.get_logger().info(f"GRIPPER OPEN: measured={self.finger_width*1000:.2f} mm; "
                                       f"target={width*1000:.2f} mm; speed={speed*1000:.1f} mm/s")
                self.dispatch("gripper", self.gripper_action, Move.Goal(width=width, speed=speed), .75)
                if "gripper" in self.actions:
                    intent.sent(width)
        except ValueError as error:
            self.stop(str(error))

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(.05, now-self.last_tick), now
        for _ in range(64):
            try:
                data, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            packet = decode(data, now) if address[0] == "127.0.0.1" else None
            if packet is None or packet["source"] != "realsense":
                self.stop("Invalid/stale tracking or non-RealSense source")
                continue
            previous = self.logic.tracking
            if previous and (previous["session"] != packet["session"] or (
                    previous["tracked"] and packet["tracked"] and (
                        packet["monotonic_s"]-previous["monotonic_s"] > .25 or
                        math.dist(packet["palm_m"], previous["palm_m"]) > self.limits.max_hand_step or
                        math.dist(packet["palm_m"], previous["palm_m"])/max(
                            1e-6, packet["monotonic_s"]-previous["monotonic_s"]) > self.limits.max_hand_speed))):
                self.stop("Tracking discontinuity; explicit re-arm required")
            self.logic.receive(packet)
            if self.logic.tracking is packet:
                self.camera_peer = address
            if not packet["tracked"]:
                self.stop("Tracking lost, jumped or restarted; explicit re-arm required")
        self.ready, self.reason = self.readiness(now)
        fresh = tracking_fresh(self.logic.tracking, now)
        was_armed = self.permit.armed
        self.permit.observe(now, fresh, self.ready)
        if was_armed and not self.permit.armed:
            self.stop(self.reason if not self.ready else "Tracking unavailable; re-arm required", report=True)
        for item in list(self.actions.values()):
            if now > item["deadline"]:
                self.stop("Action response/execution timeout; explicit re-arm required")
        if self.activation_future and now > self.activation_deadline:
            self.stop("Controller activation timed out; explicit re-arm required")
        actual = self.guard.tcp(self.joints) if self.joints else None
        if fresh and actual is not None:
            self.tick_gripper(now, actual)
        target = self.motion_target(actual) if fresh and actual is not None else None
        velocity = self.logic.step(now, dt, actual, self.ready and self.permit.armed, target)
        if self.grasp:
            velocity = self.logic.velocity = self.grasp.velocity(velocity)
            if self.grasp.phase != self.last_grasp_phase:
                self.get_logger().info("GRASP STATE: "+self.grasp.phase+"; "+self.grasp.status())
                self.last_grasp_phase = self.grasp.phase
        command = TwistStamped()
        command.header.stamp, command.header.frame_id = self.get_clock().now().to_msg(), "fr3_link0"
        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = velocity
        if self.permit.armed and self.logic.active and self.rotation_anchor:
            a, b = self.rotation_anchor, self.robot.o_t_ee.pose.orientation
            error = (a.w*-b.x+a.x*b.w-a.y*b.z+a.z*b.y,
                     a.w*-b.y+a.x*b.z+a.y*b.w-a.z*b.x,
                     a.w*-b.z-a.x*b.y+a.y*b.x+a.z*b.w)
            sign = 1 if a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z >= 0 else -1
            angular = capped(tuple(4*sign*e for e in error), .03)
            command.twist.angular.x, command.twist.angular.y, command.twist.angular.z = angular
        # Preview may drive Servo internally; it has no arm/gripper action client.
        self.twist.publish(command)
        if now-self.last_visual > .1:
            self.visuals(actual, target)
            self.last_visual = now
        state = {"mode": "execute" if self.execute else "preview", "armed": self.permit.armed,
                 "grasp_phase": self.grasp.phase if self.grasp else "disabled",
                 "grasp_z_floor_m": self.grasp.floor_for_path() if self.grasp else None,
                 "motion_mode": getattr(self, "MOTION_MODE", "bounded"),
                 "ready": self.ready, "reason": self.reason if not self.ready else self.permit.reason,
                 "tcp_m": actual, "target_m": target, "velocity_m_s": velocity,
                 "arm_capture_distance_m": self.motion["arm_capture_distance"],
                 "target_distance_m": math.dist(actual, target) if actual and target else None,
                 "control_command_success_rate": self.robot.control_command_success_rate if self.robot and
                     finite(self.robot.control_command_success_rate) else None,
                 "robot_feedback_age_s": now-self.robot_stamp if math.isfinite(self.robot_stamp) else None,
                 "robot_feedback_max_gap_s": self.max_robot_feedback_gap,
                 "last_stop_reason": self.last_stop_reason,
                 "arm_actions_completed": self.arm_actions_completed,
                 "last_segment": self.last_segment, "last_motion_result": self.last_motion_result,
                 "stop_settle_remaining_s": max(0., self.arm_not_before-now),
                 "workspace_inside": self.limits.contains(actual),
                 "whole_arm_inside": self.guard.body_inside(self.joints)[0] if self.joints else False,
                 "guard_rejections": self.guard_rejections, "actions_pending": list(self.actions),
                 "x4_readback_available": False, "gripper_enabled": self.gripper_action is not None}
        self.status.publish(String(data=json.dumps(state, allow_nan=False)))
        self.camera_feedback(now, target)

    def camera_feedback(self, now, target):
        """Display-only reply to the current camera session; never accepts commands."""
        if not self.camera_peer or now-self.camera_feedback_sent < .10:
            return
        self.camera_feedback_sent = now
        valid = self.robot is not None and 0 <= now-self.robot_stamp < .15
        tcp = None
        if valid:
            point = self.robot.o_t_ee.pose.position
            tcp = (point.x, point.y, point.z)
            valid = all(finite(v) for v in tcp)
        packet = {"version": 1, "kind": "tcp_alignment", "session": self.logic.tracking["session"],
                  "monotonic_s": now, "feedback_monotonic_s": self.robot_stamp if valid else now,
                  "valid": valid, "tcp_m": tcp if valid else None,
                  "mapping_offset_m": self.grasp.offset if self.grasp else (0., 0., 0.),
                  "grasp_phase": self.grasp.phase if self.grasp else "disabled",
                  "grasp_z_floor_m": self.grasp.floor_for_path() if self.grasp else None,
                  "tcp_control_uvd": self.limits.camera_control(tuple(a-b for a, b in
                      zip(tcp, self.grasp.offset))) if valid and self.grasp else
                      self.limits.camera_control(tcp) if valid else None,
                  "gap_m": math.dist(tcp, target) if valid and target is not None else None,
                  "capture_distance_m": self.motion["arm_capture_distance"],
                  "ready": self.ready, "armed": self.permit.armed,
                  "mode": "execute" if self.execute else "preview",
                  "reason": self.reason if not self.ready else
                      self.grasp.status() if self.permit.armed and self.grasp and self.grasp.status()
                      else self.permit.reason,
                  "operator_result": self.operator_result}
        try:
            self.socket.sendto(json.dumps(packet, allow_nan=False).encode(), self.camera_peer)
        except OSError:
            pass  # A missing overlay must never stall the motion/stop path.

    def visuals(self, actual, target):
        markers = []
        for ident, (lo, hi, color) in enumerate(((self.limits.minimum, self.limits.maximum, (.2, .7, 1.)),
                                                (self.limits.body_minimum, self.limits.body_maximum, (.2, 1., .3)))):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = "fr3_link0", self.get_clock().now().to_msg()
            marker.id, marker.type, marker.action = ident, Marker.LINE_LIST, Marker.ADD
            marker.pose.orientation.w, marker.scale.x = 1., .003
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, .9
            corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
            for i, p in enumerate(corners):
                for axis in (1, 2, 4):
                    j = i ^ axis
                    if j > i:
                        marker.points.extend([Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
                                              for v in (p, corners[j])])
            markers.append(marker)
        for ident, point in enumerate((actual, target), 2):
            marker = Marker()
            marker.header = markers[0].header
            marker.id, marker.type = ident, Marker.SPHERE
            marker.action = Marker.ADD if point else Marker.DELETE
            marker.pose.orientation.w = 1.
            if point:
                marker.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            marker.scale.x = marker.scale.y = marker.scale.z = .015
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                (.2, .6, 1., 1.) if ident == 2 else (1., .8, .1, 1.))
            markers.append(marker)
            name = Marker()
            name.header = marker.header
            name.id, name.type, name.action = ident+3, Marker.TEXT_VIEW_FACING, marker.action
            name.pose.orientation.w, name.scale.z = 1., .02
            name.color = marker.color
            name.text = "TCP" if ident == 2 else "TARGET"
            if point:
                name.pose.position = Point(x=float(point[0]), y=float(point[1])+(.025 if ident == 2 else -.025),
                                           z=float(point[2])+.03)
            markers.append(name)
        label = Marker()
        label.header = markers[0].header
        label.id, label.type, label.action = 4, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.orientation.w = 1.
        label.pose.position.x = (self.limits.body_minimum[0]+self.limits.body_maximum[0])/2
        label.pose.position.z = self.limits.body_maximum[2]+.08
        label.scale.z = .03
        label.color.r, label.color.g, label.color.a = 1., 1. if self.permit.armed else .3, 1.
        label.text = "REAL:"+("PREVIEW" if not self.execute else "ARMED" if self.permit.armed else "DISARMED")
        markers.append(label)
        link = Marker()
        link.header = markers[0].header
        link.id, link.type = 7, Marker.LINE_LIST
        link.action = Marker.ADD if actual and target else Marker.DELETE
        link.pose.orientation.w, link.scale.x = 1., .002
        link.color.a = 1.
        gap_label = Marker()
        gap_label.header = link.header
        gap_label.id, gap_label.type, gap_label.action = 8, Marker.TEXT_VIEW_FACING, link.action
        gap_label.pose.orientation.w, gap_label.scale.z = 1., .025
        if actual and target:
            gap = math.dist(actual, target)
            within = gap <= self.motion["arm_capture_distance"]
            link.color.r, link.color.g = (0., 1.) if within else (1., .2)
            link.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (actual, target)]
            gap_label.text = f"GAP:{gap*100:.1f}cm"
            gap_label.pose.position = Point(x=(actual[0]+target[0])/2, y=(actual[1]+target[1])/2,
                                            z=max(actual[2], target[2])+.08)
        gap_label.color = link.color
        markers.extend([link, gap_label])
        self.markers.publish(MarkerArray(markers=markers))
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        table = CollisionObject(id="installation_table", operation=CollisionObject.ADD)
        table.header = markers[0].header
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[
            self.motion["table_size"], self.motion["table_size"], self.motion["table_thickness"]])]
        pose = Pose()
        pose.orientation.w, pose.position.z = 1., -self.motion["table_thickness"]/2
        table.primitive_poses = [pose]
        scene.world.collision_objects = [table]
        # The base is bolted to the table. Only that static mounting contact is allowed.
        names = sorted({n for pair in self.allowed_pairs for n in pair})
        pairs = {frozenset(pair) for pair in self.allowed_pairs}
        scene.allowed_collision_matrix.entry_names = names
        scene.allowed_collision_matrix.entry_values = [AllowedCollisionEntry(enabled=[
            a == b or frozenset((a, b)) in pairs for b in names]) for a in names]
        self.scene.publish(scene)

    def close(self):
        self.stop("Application shutting down")
        until = time.monotonic()+.5
        while self.actions and rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.02)
        self.socket.close()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
