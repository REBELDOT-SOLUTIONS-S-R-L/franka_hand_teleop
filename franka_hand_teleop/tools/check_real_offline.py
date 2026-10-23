"""Exercise the real bridge against in-process mock services/actions, NEVER a driver."""
import copy
import ctypes
import importlib.util
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SwitchController,
)
from franka_hand_teleop import real_bridge as bridge_module
from franka_hand_teleop.deployment import (
    REAL_ARM_CONTROLLER,
    REAL_NAMESPACE,
    REAL_PLUGIN,
    load_profile,
)
from franka_hand_teleop.real_bridge import RealBridge, duration, seconds
from franka_hand_teleop.workspace_guard import WorkspaceGuard
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose
from lifecycle_msgs.msg import State
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def orientation(matrix):
    # The offline fixture uses SciPy only to generate synthetic robot telemetry.
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(matrix).as_quat()


class Fixture(Node):
    def __init__(self, model, limits, initial, mock_servo=True):
        super().__init__("offline_robot_fixture", namespace=REAL_NAMESPACE)
        self.guard = WorkspaceGuard(model, limits)
        self.q = np.array([initial[n] for n in self.guard.names])
        self.dq = np.zeros(7)
        self.qd, self.dqd = self.q.copy(), self.dq.copy()
        self.joint_bias = np.zeros(7)
        self.plant_lag_s = 0.
        self.encoder_noise = 0.
        self.last_plant_tick = time.monotonic()
        self.mode = FrankaRobotState.ROBOT_MODE_IDLE
        self.plugin = REAL_PLUGIN
        self.hardware_state = "active"
        self.controller = "inactive"
        self.controller_type = REAL_ARM_CONTROLLER
        self.publish_robot = True
        self.job = None
        self.count = self.cancel_count = self.gripper_count = 0
        self.width = .04
        self.grasp_success, self.object_width, self.grasp_delay = True, .02, .3
        self.accept = True
        self.started = time.monotonic()
        self.robot_pub = self.create_publisher(FrankaRobotState, "franka_robot_state_broadcaster/robot_state", 1)
        self.servo_pub = self.create_publisher(ServoStatus, "servo_node/status", 1) if mock_servo else None
        self.finger_pub = self.create_publisher(JointState, "joint_states", 1)
        self.raw = self.create_publisher(JointTrajectory, "servo_trajectory", 1)
        self.create_service(ListHardwareComponents, "controller_manager/list_hardware_components", self.hardware)
        self.create_service(ListControllers, "controller_manager/list_controllers", self.controllers)
        self.create_service(SwitchController, "controller_manager/switch_controller", self.activate)
        if mock_servo:
            self.create_service(ServoCommandType, "servo_node/switch_command_type", self.switch)
        self.server = ActionServer(self, FollowJointTrajectory, "fr3_arm_controller/follow_joint_trajectory",
            self.execute_arm, goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.gripper_server = ActionServer(self, Move, "franka_gripper/move", self.execute_gripper,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.grasp_server = ActionServer(self, Grasp, "franka_gripper/grasp", self.execute_gripper,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.create_timer(.01, self.tick)

    def hardware(self, _, response):
        response.component = [HardwareComponentState(name="FrankaHardwareInterface", plugin_name=self.plugin,
                                                     state=State(id=3 if self.hardware_state == "active" else 2,
                                                                 label=self.hardware_state))]
        return response

    def controllers(self, _, response):
        response.controller = [ControllerState(name=n, state=s,
            type=self.controller_type if n == "fr3_arm_controller" else "offline_broadcaster") for n, s in (
            ("joint_state_broadcaster", "active"), ("franka_robot_state_broadcaster", "active"),
            ("fr3_arm_controller", self.controller))]
        return response

    def activate(self, request, response):
        assert request.activate_controllers == ["fr3_arm_controller"]
        self.controller, response.ok = "active", True
        return response

    def switch(self, _, response):
        response.success = True
        return response

    async def execute_arm(self, handle):
        assert self.job is None
        points = handle.request.trajectory.points
        assert len(points) == 2 and list(points[0].velocities) == [0.]*7 and list(points[1].accelerations) == [0.]*7
        assert math.dist(points[0].positions, self.q) < .003
        valid, reason = self.guard.check(dict(zip(self.guard.names, map(float, self.q))),
                                        [point.positions for point in points])
        assert valid, reason
        self.count += 1
        future = Future()
        self.job = {"start": time.monotonic(), "duration": seconds(points[-1].time_from_start),
                    "a": np.array(points[0].positions), "b": np.array(points[-1].positions),
                    "handle": handle, "done": future}
        await future
        result = FollowJointTrajectory.Result(error_code=0)
        if handle.is_cancel_requested:
            self.cancel_count += 1
            handle.canceled()
        else:
            handle.succeed()
        return result

    async def execute_gripper(self, handle):
        self.gripper_count += 1
        goal = handle.request
        grasping = isinstance(goal, Grasp.Goal)
        assert goal.speed == .04
        if grasping:
            assert goal.width == 0. and goal.force == 30.
        else:
            assert abs(goal.width-self.width) <= .004001
        future = Future()
        timer = self.create_timer(self.grasp_delay if grasping else .3,
            lambda: future.set_result(True) if not future.done() else None)
        await future
        self.destroy_timer(timer)
        if handle.is_cancel_requested:
            self.cancel_count += 1
            handle.canceled()
        else:
            self.width = self.object_width if grasping else goal.width
            if not grasping or self.grasp_success:
                handle.succeed()
            else:
                handle.abort()
        return (Grasp.Result(success=self.grasp_success) if grasping else Move.Result(success=True))

    def tick(self):
        if self.job:
            job = self.job
            s = min(1., (time.monotonic()-job["start"])/job["duration"])
            f = 10*s**3-15*s**4+6*s**5
            self.qd = job["a"]+(job["b"]-job["a"])*f
            self.dqd = (job["b"]-job["a"])*30*s*s*(1-s)**2/job["duration"]
            if s == 1 or job["handle"].is_cancel_requested:
                self.dqd = np.zeros(7)
                self.job = None
                job["done"].set_result(True)
        now = time.monotonic()
        dt, self.last_plant_tick = max(1e-6, now-self.last_plant_tick), now
        previous = self.q.copy()
        alpha = 1-math.exp(-dt/self.plant_lag_s) if self.plant_lag_s else 1.
        self.q += alpha*(self.qd-self.joint_bias-self.q)
        self.q[0] += self.encoder_noise*math.sin(now*19)
        self.dq = (self.q-previous)/dt
        stamp = self.get_clock().now().to_msg()
        if self.publish_robot:
            message = FrankaRobotState(robot_mode=self.mode, time=time.monotonic()-self.started)
            message.header.stamp = stamp
            message.measured_joint_state = JointState(name=self.guard.names, position=self.q.tolist(),
                                                       velocity=self.dq.tolist())
            message.desired_joint_state = JointState(name=self.guard.names,
                position=self.qd.tolist(), velocity=self.dqd.tolist())
            pose = self.guard.pose(dict(zip(self.guard.names, self.q)))
            position, quat = pose[:3, 3], orientation(pose[:3, :3])
            message.o_t_ee.pose = Pose()
            message.o_t_ee.pose.position.x, message.o_t_ee.pose.position.y, message.o_t_ee.pose.position.z = map(float, position)
            q = message.o_t_ee.pose.orientation
            q.x, q.y, q.z, q.w = map(float, quat)
            self.robot_pub.publish(message)
        if self.servo_pub:
            self.servo_pub.publish(ServoStatus(code=0, message="Offline fixture ready"))
        fingers = JointState(name=["fr3_finger_joint1", "fr3_finger_joint2"], position=[self.width/2]*2)
        fingers.header.stamp = stamp
        self.publish_joints(fingers)

    def publish_joints(self, message):
        self.finger_pub.publish(message)

    def trajectory(self, target):
        message = JointTrajectory(joint_names=self.guard.names)
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "fr3_link0"
        message.points = [JointTrajectoryPoint(positions=self.q.tolist()),
            JointTrajectoryPoint(positions=list(map(float, target)), time_from_start=duration(.20))]
        self.raw.publish(message)


def main():
    # This script never constructs ros2_control_node, FrankaHardwareInterface,
    # a gripper driver, libfranka Robot, or a network connection to a robot IP.
    # Test-process globals only: never discover production services/actions.
    global REAL_DOMAIN, REAL_PORT
    REAL_DOMAIN, REAL_PORT = "139", 49303
    bridge_module.REAL_DOMAIN, bridge_module.REAL_PORT = REAL_DOMAIN, REAL_PORT
    os.environ["ROS_DOMAIN_ID"] = REAL_DOMAIN
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", REAL_PORT))
    share = Path(get_package_share_directory("franka_hand_teleop"))
    hardware = Path(get_package_share_directory("franka_hardware")).parents[1]/"lib/libfranka_hardware.so"
    ctypes.CDLL(str(hardware), mode=os.RTLD_NOW)
    print("PASS: real plugin resolves all symbols without constructing a hardware interface or opening a robot connection", flush=True)
    spec = importlib.util.spec_from_file_location("real_launch", share/"launch/real.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    model = launch.model_for("192.0.2.1")  # Documentation-only address, xacro expansion only.
    profile, limits = load_profile(share/"config/real_profile.yaml")
    with tempfile.TemporaryDirectory() as directory:
        unverified = copy.deepcopy(profile)
        unverified["commissioning"] = {key: False for key in unverified["commissioning"]}
        unverified_path = Path(directory)/"unverified-profile.yaml"
        unverified_path.write_text(yaml.safe_dump(unverified))
        try:
            load_profile(unverified_path, execute=True)
        except ValueError:
            print("PASS: unverified profile blocks real execution before hardware startup", flush=True)
        else:
            raise AssertionError("Unverified commissioning profile must block motion")
    semantic = xacro.process_file(str(Path(get_package_share_directory("franka_description"))/"robots/fr3/fr3.srdf.xacro")).toxml()
    initial = yaml.safe_load((share/"config/initial_positions.yaml").read_text())
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    fixture = Fixture(model, limits, initial)
    executor.add_node(fixture)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    bridge = None
    source, tracked, control, opening = "realsense", True, [.5, .5, .5], .5

    def drive(length):
        nonlocal sequence
        until = time.monotonic()+length
        while time.monotonic() < until:
            now = time.monotonic()
            packet = {"version": 2, "kind": "tracking", "session": "offline-fixture", "sequence": sequence,
                      "monotonic_s": now, "source": source, "tracked": tracked, "units": "m",
                      "frame": "camera_color_optical_frame", "palm_m": [0, 0, .65],
                      "opening": opening, "control_uvd": control}
            sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", REAL_PORT))
            sequence += 1
            executor.spin_once(timeout_sec=.005)
            time.sleep(.002)

    def arm(value=True):
        response = bridge.arm_request(SetBool.Request(data=value), SetBool.Response())
        assert bridge.operator_result["success"] == response.success
        assert bridge.operator_result["message"] == response.message
        assert bridge.operator_result["action"] == ("arm" if value else "disarm")
        return response

    def create(path, execute):
        node = RealBridge(namespace=REAL_NAMESPACE, parameter_overrides=[
            Parameter("robot_description", value=model), Parameter("robot_description_semantic", value=semantic),
            Parameter("profile", value=str(path)), Parameter("execute", value=execute)])
        executor.add_node(node)
        return node

    try:
        bridge = create(share/"config/real_profile.yaml", False)
        drive(1.2)
        assert bridge.ready, bridge.reason
        assert (not arm().success and bridge.arm_action is None and bridge.gripper_action is None
                and bridge.grasp_action is None)
        assert fixture.count == 0 and fixture.gripper_count == 0
        print("PASS: preview verifies feedback but has no arm/gripper action client and cannot arm", flush=True)
        executor.remove_node(bridge)
        bridge.close()
        bridge = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"synthetic-only-profile.yaml"
            profile["commissioning"] = {k: True for k in profile["commissioning"]}
            path.write_text(yaml.safe_dump(profile))
            bridge = create(path, True)
            fixture.plugin = "mock_components/GenericSystem"
            drive(1.0)
            assert not bridge.ready and not arm().success
            assert "Hardware identity/state rejected" in bridge.reason
            print("PASS: wrong hardware identity cannot arm", flush=True)
            fixture.plugin = REAL_PLUGIN
            fixture.hardware_state = "inactive"
            drive(.8)
            assert not bridge.ready and not arm().success
            assert "state=inactive" in bridge.reason, bridge.reason
            fixture.hardware_state = "active"
            fixture.controller_type = "joint_trajectory_controller/JointTrajectoryController"
            drive(.8)
            assert not bridge.ready and not arm().success
            assert "Controller verification rejected: arm type=" in bridge.reason
            fixture.controller_type = REAL_ARM_CONTROLLER
            source = "demo"
            drive(1.0)
            assert not arm().success
            source, control = "realsense", [.1, .5, .5]
            drive(.8)
            assert bridge.ready, bridge.reason
            assert not arm().success
            print("PASS: demo input and distant camera targets cannot arm", flush=True)
            control = [.5, .5, .5]
            drive(.6)
            assert arm().success
            drive(.5)
            assert bridge.permit.armed, (bridge.reason, bridge.permit.reason,
                                         bridge.controller_state, fixture.controller)
            legacy = JointTrajectory(joint_names=fixture.guard.names)
            legacy.header.stamp = fixture.get_clock().now().to_msg()
            legacy.points = [JointTrajectoryPoint(positions=fixture.q.tolist(), time_from_start=duration(.01))]
            fixture.raw.publish(legacy)
            drive(.1)
            assert not bridge.permit.armed and "seeded Servo proposal" in bridge.last_stop_reason
            assert fixture.count == 0
            drive(.4)
            assert arm().success
            drive(.1)
            print("PASS: legacy unseeded proposal is rejected without action output", flush=True)
            fixture.trajectory(fixture.q+np.array([.0002, 0, 0, 0, 0, 0, 0]))
            drive(.13)
            assert fixture.count == 1 and bridge.actions
            tracked = False
            drive(.5)
            assert not bridge.permit.armed and fixture.cancel_count >= 1
            assert bridge.last_stop_reason == "Tracking lost, jumped or restarted; explicit re-arm required"
            assert not arm().success
            count = fixture.count
            tracked = True
            drive(.7)
            assert not bridge.permit.armed and fixture.count == count
            print("PASS: bounded quintic action executes; loss cancels it and return never re-arms", flush=True)
            fixture.mode = FrankaRobotState.ROBOT_MODE_USER_STOPPED
            drive(.25)
            assert not arm().success
            fixture.mode = FrankaRobotState.ROBOT_MODE_IDLE
            drive(.6)
            assert arm().success
            drive(.1)
            unsafe = fixture.q.copy()
            unsafe[1] = fixture.guard.bounds["fr3_joint2"][1]+.1
            fixture.trajectory(unsafe)
            drive(.2)
            assert bridge.guard_rejections > 0 and not bridge.permit.armed
            print("PASS: robot stop refuses arming; unsafe Servo path is rejected before action output", flush=True)
            drive(.6)
            assert arm().success
            drive(.1)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = fixture.guard.names
            goal.trajectory.points = [JointTrajectoryPoint(positions=fixture.q.tolist(), velocities=[0.]*7),
                JointTrajectoryPoint(positions=(fixture.q+np.array([.0001, 0, 0, 0, 0, 0, 0])).tolist(),
                                     velocities=[0.]*7, accelerations=[0.]*7, time_from_start=duration(.4))]
            goal.trajectory.points[0].accelerations = [0.]*7
            before = fixture.cancel_count
            bridge.dispatch("arm", bridge.arm_action, goal, 1)
            bridge.stop("Fixture stop before goal acknowledgement")
            drive(1.7)
            assert fixture.cancel_count > before and not bridge.permit.armed
            print("PASS: a goal acknowledged after disarming is cancelled", flush=True)
            drive(.5)
            assert arm().success
            drive(.1)
            fixture.publish_robot = False
            drive(.25)
            assert not bridge.permit.armed and not bridge.ready
            assert bridge.last_stop_reason == "Robot feedback stale", bridge.last_stop_reason
            fixture.publish_robot = True
            drive(.5)
            assert not bridge.permit.armed
            print("PASS: stale robot feedback disarms; fresh feedback does not resume motion", flush=True)
            # Exercise the real supervisor/client with a harmless launch stub.
            session_source = share/"tools/stack_session.py"
            # Remap only the supervisor under test to isolated DDS. Its launch
            # child is replaced with sleep, so no production launch is possible.
            session_code = session_source.read_text().replace('"133"', '"139"')
            stub = "import signal,sys,time; signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); time.sleep(30)"
            harness = ("import subprocess,sys; original=subprocess.Popen; "
                       f"subprocess.Popen=lambda args,**kwargs: original([sys.executable,'-c',{stub!r}],**kwargs); "
                       f"exec(compile({session_code!r},{str(session_source)!r},'exec'),{{'__name__':'__main__'}})")
            with Path("/workspace/logs/real-supervisor-mock-check.log").open("w") as log:
                supervisor = subprocess.Popen([sys.executable, "-c", harness], stdin=subprocess.PIPE,
                                              stdout=log, stderr=subprocess.STDOUT)
                try:
                    drive(1.2)
                    supervisor.stdin.write(b"a\nd\n")
                    supervisor.stdin.flush()
                    drive(.5)
                    assert not bridge.permit.armed
                    supervisor.stdin.write(b"a\n")
                    supervisor.stdin.flush()
                    drive(.5)
                    assert bridge.permit.armed
                    supervisor.stdin.close()
                    drive(.5)
                    assert not bridge.permit.armed
                    assert supervisor.wait(timeout=3) == 0
                finally:
                    if supervisor.poll() is None:
                        supervisor.terminate()
                        supervisor.wait(timeout=3)
            print("PASS: supervisor preserves arm/disarm order and disarms on stdin lease loss; launch child was a local stub", flush=True)
            executor.remove_node(bridge)
            bridge.close()
            bridge = None
            profile["enable_gripper"] = True
            path.write_text(yaml.safe_dump(profile))
            bridge = create(path, True)
            opening = .8
            until = time.monotonic()+4
            while time.monotonic() < until:
                drive(.05)
                if bridge.permit.valid_since and time.monotonic()-bridge.permit.valid_since > .35:
                    break
            response = arm()
            assert response.success, (response.message, bridge.reason, bridge.permit.reason)
            drive(.15)
            assert fixture.gripper_count > 0
            tracked = False
            drive(.5)
            assert not bridge.permit.armed and not bridge.actions
            print("PASS: optional gripper sends bounded Move actions and cancels on loss; no homing/grasp", flush=True)
            # New grasp outcomes use only this in-process fixture; no Hand driver.
            for outcome in ("failed", "empty", "cancelled", "open_cancelled", "hold_drift"):
                fixture.width = .04
                fixture.grasp_success = outcome != "failed"
                fixture.object_width = 0. if outcome == "empty" else .02
                fixture.grasp_delay = 1. if outcome in ("cancelled", "open_cancelled", "hold_drift") else .3
                tracked, opening = True, .5
                if bridge.grasp.phase == "interrupted":
                    # Simulate the operator opening a stopped mock Hand completely.
                    fixture.width = .08
                    opening = 1.
                drive(2.)
                response = arm()
                assert response.success, (outcome, response.message)
                drive(.25)
                if bridge.grasp.phase == "interrupted":
                    fixture.width = .08
                    drive(.35)
                    assert bridge.grasp.phase == "normal"
                fixture.width = .04
                opening = .2
                drive(.25)
                if outcome in ("cancelled", "open_cancelled", "hold_drift"):
                    assert bridge.grasp.phase == "closing", bridge.last_stop_reason
                    if outcome == "cancelled":
                        tracked = False
                    elif outcome == "open_cancelled":
                        opening = 1.
                    else:
                        fixture.qd[1] += .01
                drive(1.4)
                assert not bridge.permit.armed and bridge.grasp.phase == "interrupted", (outcome, bridge.last_stop_reason)
                assert bridge.grasp.floor is None and not bridge.actions
                if outcome == "empty":
                    assert "nonempty" in bridge.last_stop_reason, bridge.last_stop_reason
                if outcome == "hold_drift":
                    assert "drifted" in bridge.last_stop_reason, bridge.last_stop_reason
                print(f"PASS: {outcome} Grasp cannot enter lift or grant downward motion", flush=True)
        print("Real deployment OFFLINE check passed. No real hardware driver was started.", flush=True)
    finally:
        if bridge:
            executor.remove_node(bridge)
            bridge.close()
        sock.close()
        executor.remove_node(fixture)
        fixture.server.destroy()
        fixture.gripper_server.destroy()
        fixture.grasp_server.destroy()
        fixture.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
