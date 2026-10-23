"""Local camera data -> MoveIt Servo, guarded to an isolated mock FR3."""
import json
import math
import os
import socket
import time
from copy import deepcopy

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Point, Pose, TwistStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .core import DOMAIN, NAMESPACE, PORT, Limits, Teleop, capped, decode
from .workspace_guard import WorkspaceGuard


class Bridge(Node):
    def __init__(self):
        if os.environ.get("ROS_DOMAIN_ID") != DOMAIN:
            raise RuntimeError(f"Fake hand bridge requires ROS_DOMAIN_ID={DOMAIN}")
        super().__init__("hand_bridge")
        if self.get_namespace() != NAMESPACE:
            raise RuntimeError(f"Fake hand bridge requires namespace {NAMESPACE}")
        defaults = Limits()
        for name, value in {
            "workspace_min": list(defaults.minimum), "workspace_max": list(defaults.maximum),
            "arm_workspace_min": list(defaults.body_minimum), "arm_workspace_max": list(defaults.body_maximum),
            "workspace_margin": defaults.margin, "max_speed": defaults.speed,
            "max_acceleration": defaults.acceleration, "udp_port": PORT,
            "robot_description": "",
        }.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.logic = Teleop(Limits(tuple(param("workspace_min")), tuple(param("workspace_max")),
                                  param("workspace_margin"), param("max_speed"),
                                  param("max_acceleration"),
                                  body_minimum=tuple(param("arm_workspace_min")),
                                  body_maximum=tuple(param("arm_workspace_max"))))
        self.guard = WorkspaceGuard(param("robot_description"), self.logic.limits)
        self.guard_rejections = 0
        self.guard_reason = "Waiting for trajectories"
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", param("udp_port")))
        self.socket.setblocking(False)
        self.hardware = self.create_client(
            ListHardwareComponents, "controller_manager/list_hardware_components")
        self.controllers = self.create_client(ListControllers, "controller_manager/list_controllers")
        self.switch = self.create_client(ServoCommandType, "servo_node/switch_command_type")
        self.hardware_future = self.switch_future = self.controllers_future = None
        self.backend_stamp = -math.inf
        self.controllers_stamp = self.servo_stamp = -math.inf
        self.switch_ok = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.joints = {}
        self.joint_stamp = -math.inf
        self.create_subscription(JointState, "joint_states", self.receive_joints,
                                 qos_profile_sensor_data)
        self.servo_status = None
        self.create_subscription(ServoStatus, "servo_node/status", self.receive_status, 1)
        self.twist = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 1)
        self.arm_output = self.create_publisher(JointTrajectory, "fr3_arm_controller/joint_trajectory", 1)
        self.create_subscription(JointTrajectory, "servo_trajectory", self.gate_trajectory, 1)
        self.fingers = self.create_publisher(
            JointTrajectory, "fake_gripper_controller/joint_trajectory", 1)
        self.status = self.create_publisher(String, "status", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, "markers", latched)
        self.scene = self.create_publisher(PlanningScene, "planning_scene", latched)
        self.last_tick = time.monotonic()
        self.rotation_anchor = None
        self.last_visual = -math.inf
        self.last_status = ""
        self.create_timer(0.02, self.tick)
        self.create_timer(0.5, self.check_backend)
        self.get_logger().info(
            f"FAKE ONLY: localhost UDP {param('udp_port')}, domain {DOMAIN}, namespace {NAMESPACE}. "
            "Waiting for mock hardware and valid hand tracking; no enable button required.")

    def receive_joints(self, message):
        positions = dict(zip(message.name, message.position))
        names = [f"fr3_joint{i}" for i in range(1, 8)] + ["fr3_finger_joint1"]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (all(name in positions and math.isfinite(positions[name]) for name in names)
                and -0.02 <= age <= 0.25):
            self.joints = positions
            self.joint_stamp = time.monotonic()

    def receive_status(self, message):
        self.servo_status = message
        self.servo_stamp = time.monotonic()

    def gate_trajectory(self, message):
        now = time.monotonic()
        if now-self.backend_stamp >= 1.5 or now-self.joint_stamp >= .25:
            return
        names = message.joint_names
        if len(names) != len(self.guard.names) or set(names) != set(self.guard.names):
            valid, reason = False, "Unexpected trajectory joints"
        else:
            indexes = [names.index(n) for n in self.guard.names]
            waypoints = [[p.positions[i] for i in indexes] for p in message.points
                         if len(p.positions) == len(names)]
            valid, reason = (self.guard.check(self.joints, waypoints) if
                             len(waypoints) == len(message.points) else (False, "Invalid trajectory"))
        self.guard_reason = reason
        if valid:
            output = deepcopy(message)
            for point in output.points:
                point.velocities, point.accelerations, point.effort = [], [], []
            self.arm_output.publish(output)
        else:
            self.guard_rejections += 1
            self.logic.stop(reason)
            hold = JointTrajectory(joint_names=self.guard.names)
            point = JointTrajectoryPoint(positions=[self.joints[n] for n in self.guard.names])
            point.time_from_start.nanosec = 20_000_000
            hold.points = [point]
            self.arm_output.publish(hold)

    def check_backend(self):
        if self.hardware_future and self.hardware_future.done():
            result = self.hardware_future.result()
            self.hardware_future = None
            components = result.component if result else []
            valid = len(components) == 1 and all(
                c.name == "HandTeleopFakeSystem" and
                (c.plugin_name or c.class_type) == "mock_components/GenericSystem" and
                c.state.label == "active" for c in components)
            if valid:
                self.backend_stamp = time.monotonic()
            else:
                self.backend_stamp = -math.inf
                self.logic.stop("Hardware guard rejected backend")
        if self.hardware_future is None and self.hardware.service_is_ready():
            self.hardware_future = self.hardware.call_async(ListHardwareComponents.Request())
        if self.controllers_future and self.controllers_future.done():
            result = self.controllers_future.result()
            self.controllers_future = None
            active = {c.name for c in result.controller if c.state == "active"} if result else set()
            self.controllers_stamp = (time.monotonic() if {
                "joint_state_broadcaster", "fr3_arm_controller", "fake_gripper_controller"
            } <= active else -math.inf)
        if self.controllers_future is None and self.controllers.service_is_ready():
            self.controllers_future = self.controllers.call_async(ListControllers.Request())
        if self.switch_future and self.switch_future.done():
            result = self.switch_future.result()
            self.switch_ok = bool(result and result.success)
            self.switch_future = None
        if (not self.switch_ok and self.switch_future is None and
                self.switch.service_is_ready() and time.monotonic() - self.backend_stamp < 1.5):
            request = ServoCommandType.Request()
            request.command_type = request.TWIST
            self.switch_future = self.switch.call_async(request)

    def receive_packets(self, now):
        # Bound work per tick; do not let a busy sender starve the watchdog.
        for _ in range(64):
            try:
                data, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if address[0] != "127.0.0.1":
                continue
            packet = decode(data, now)
            if packet is not None:
                self.logic.receive(packet)
            else:
                # Invalid data fails closed, rather than retaining an earlier valid target.
                self.logic.stop("Invalid/stale tracking packet; waiting for valid tracking")

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.05), now
        self.receive_packets(now)
        actual = orientation = None
        try:
            transform = self.tf.lookup_transform("fr3_link0", "fr3_hand_tcp", rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if 0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp < 0.25:
                t = transform.transform.translation
                actual = (t.x, t.y, t.z)
                orientation = transform.transform.rotation
        except TransformException:
            pass
        backend_ok = now - self.backend_stamp < 1.5
        ready = (backend_ok and self.switch_ok and now - self.joint_stamp < 0.25 and
                 now - self.controllers_stamp < 1.5 and now - self.servo_stamp < 0.3)
        if self.servo_status and self.servo_status.code in (
                ServoStatus.INVALID,
                ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.HALT_FOR_COLLISION,
                ServoStatus.JOINT_BOUND):
            ready = False
        velocity = self.logic.step(now, dt, actual, ready)
        body_ok, body_reason = self.guard.body_inside(self.joints) if all(
            n in self.joints for n in self.guard.names) else (False, "Waiting for arm feedback")
        if not body_ok:
            self.logic.stop(body_reason)
            velocity = (0.0, 0.0, 0.0)
        if ready and orientation is not None and self.rotation_anchor is None:
            self.rotation_anchor = orientation
        # Publish no motion interface at all until the hardware plugin was verified.
        if backend_ok:
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "fr3_link0"
            command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = velocity
            if self.logic.active and orientation and self.rotation_anchor:
                # Quaternion target * inverse(current), in the base frame.
                a, b = self.rotation_anchor, orientation
                error = (a.w * -b.x + a.x * b.w - a.y * b.z + a.z * b.y,
                         a.w * -b.y + a.x * b.z + a.y * b.w - a.z * b.x,
                         a.w * -b.z - a.x * b.y + a.y * b.x + a.z * b.w)
                sign = 1 if a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z >= 0 else -1
                angular = capped(tuple(4 * sign * e for e in error), 0.15)
                command.twist.angular.x, command.twist.angular.y, command.twist.angular.z = angular
            self.twist.publish(command)
            if "fr3_finger_joint1" in self.joints:
                current = self.joints["fr3_finger_joint1"]
                desired = (0.04 * self.logic.opening if self.logic.active else current)
                target = max(current - 0.03*dt, min(current + 0.03*dt, desired))
                message = JointTrajectory(joint_names=["fr3_finger_joint1"])
                point = JointTrajectoryPoint(positions=[target], velocities=[0.0])
                point.time_from_start.nanosec = 60_000_000
                message.points = [point]
                self.fingers.publish(message)
        if now - self.last_visual > 0.1:
            self.publish_visuals(actual)
            self.last_visual = now
        status = {
            "fake_only": True, "backend_verified": backend_ok, "ready": ready and actual is not None,
            "active": self.logic.active, "reason": self.logic.reason,
            "tcp_m": actual, "target_m": self.logic.target,
            "velocity_m_s": velocity,
            "source": (self.logic.tracking or {}).get("source"),
            "servo": self.servo_status.message if self.servo_status else "Waiting",
            "trajectory_guard": self.guard_reason, "guard_rejections": self.guard_rejections,
            "workspace_inside": self.logic.limits.contains(actual),
            "whole_arm_inside": body_ok, "whole_arm_status": body_reason,
            "fk_tcp_m": self.guard.tcp(self.joints) if all(n in self.joints for n in self.guard.names) else None,
        }
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))
        if self.logic.reason != self.last_status:
            self.get_logger().info(self.logic.reason)
            self.last_status = self.logic.reason

    def publish_visuals(self, actual):
        stamp = self.get_clock().now().to_msg()
        lo, hi = self.logic.limits.minimum, self.logic.limits.maximum
        box = Marker()
        box.header.frame_id, box.header.stamp = "fr3_link0", stamp
        box.ns, box.id, box.type, box.action = "fake_workspace", 0, Marker.LINE_LIST, Marker.ADD
        box.pose.orientation.w = 1.0
        box.scale.x = 0.003
        box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.7, 1.0, 0.9
        corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                   for z in (lo[2], hi[2])]
        for i, p in enumerate(corners):
            for axis in (1, 2, 4):
                j = i ^ axis
                if j > i:
                    box.points += [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                   Point(x=float(corners[j][0]), y=float(corners[j][1]),
                                         z=float(corners[j][2]))]
        label = Marker()
        label.header = box.header
        label.ns, label.id, label.type, label.action = "fake_workspace", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.x = (self.logic.limits.body_minimum[0]+self.logic.limits.body_maximum[0])/2
        label.pose.position.z = self.logic.limits.body_maximum[2] + .13
        label.scale.z = 0.025
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = "FAKE:" + ("TRACKING" if self.logic.active else "WAITING")
        target = Marker()
        target.header = box.header
        target.ns, target.id, target.type = "fake_workspace", 2, Marker.SPHERE
        target.action = Marker.ADD if self.logic.active else Marker.DELETE
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.015
        target.color.r, target.color.g, target.color.a = 1.0, 0.7, 1.0
        if self.logic.target:
            target.pose.position = Point(**dict(zip(("x", "y", "z"), self.logic.target)))
        tcp = deepcopy(target)
        tcp.id, tcp.action = 3, Marker.ADD if actual is not None else Marker.DELETE
        tcp.scale.x = tcp.scale.y = tcp.scale.z = .02
        tcp.color.r, tcp.color.g, tcp.color.b = .2, .6, 1.0
        if actual is not None:
            tcp.pose.position = Point(**dict(zip(("x", "y", "z"), actual)))
        outer = deepcopy(box)
        outer.id, outer.points = 4, []
        outer.color.r, outer.color.g, outer.color.b = .2, 1.0, .3
        lo, hi = self.logic.limits.body_minimum, self.logic.limits.body_maximum
        corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        for i, p in enumerate(corners):
            for axis in (1, 2, 4):
                j = i ^ axis
                if j > i:
                    outer.points += [Point(**dict(zip(("x", "y", "z"), p))),
                                     Point(**dict(zip(("x", "y", "z"), corners[j])))]
        arm_label = deepcopy(label)
        arm_label.id, arm_label.text = 5, "ARM"
        arm_label.pose.position.z = self.logic.limits.body_maximum[2] + .04
        arm_label.color = outer.color
        tool_label = deepcopy(label)
        tool_label.id, tool_label.text = 6, "TOOL"
        tool_label.pose.position.x = (self.logic.limits.minimum[0]+self.logic.limits.maximum[0])/2
        tool_label.pose.position.y = (self.logic.limits.minimum[1]+self.logic.limits.maximum[1])/2
        tool_label.pose.position.z = self.logic.limits.maximum[2] + .03
        tool_label.color = box.color
        self.markers.publish(MarkerArray(markers=[box, label, target, tcp, outer, arm_label, tool_label]))
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        table = CollisionObject()
        table.header = box.header
        table.id, table.operation = "simulation_table", CollisionObject.ADD
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.04])]
        pose = Pose()
        pose.orientation.w, pose.position.z = 1.0, -0.05
        table.primitive_poses = [pose]
        scene.world.collision_objects = [table]
        self.scene.publish(scene)

    def close(self):
        self.socket.close()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
