"""Actual rolling controller + Servo + bridge on ClockedMockSystem only, DDS 141."""
import argparse
import copy
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_hand_teleop import real_bridge, smooth_bridge
from franka_hand_teleop.deployment import load_profile
from franka_hand_teleop.smooth_bridge import SmoothBridge
from franka_hand_teleop.workspace_guard import WorkspaceGuard
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def mock_hand_feedback():
    """Independent simulated source, matching the separate physical Hand driver."""
    rclpy.init()
    node = Node("rolling_mock_hand", namespace="/hand_teleop_real")
    publisher = node.create_publisher(JointState, "gripper_joint_states", 1)
    status = node.create_publisher(Float64MultiArray, "mock_hand_state", 1)
    width, goals, cancellations, grasps = .04, 0, 0, 0

    def publish():
        message = JointState(name=["fr3_finger_joint1", "fr3_finger_joint2"], position=[width/2, width/2])
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        status.publish(Float64MultiArray(data=[width, float(goals), float(cancellations), float(grasps)]))

    async def execute(handle):
        nonlocal width, goals, cancellations, grasps
        goal = handle.request
        assert 0 <= goal.width <= .08 and goal.speed == .04
        grasping = isinstance(goal, Grasp.Goal)
        target = .04 if grasping else goal.width  # Synthetic 40 mm cylinder, no physical Hand.
        if grasping:
            assert goal.force == 30. and goal.epsilon.outer == .08
            grasps += 1
        goals += 1
        previous = started = time.monotonic()
        finished = Future()

        def advance():
            nonlocal width, previous, cancellations
            if finished.done():
                return
            now = time.monotonic()
            if handle.is_cancel_requested:
                cancellations += 1
                handle.canceled()
                finished.set_result(True)
            else:
                increment = goal.speed*(now-previous)
                width += max(-increment, min(increment, target-width))
                # Model action acknowledgement latency independently of reaching
                # the target, so cancellation tests can identify an in-flight goal.
                if abs(target-width) < 1e-9 and now-started >= .3:
                    handle.succeed()
                    finished.set_result(True)
            previous = now

        timer = node.create_timer(.01, advance)
        await finished
        node.destroy_timer(timer)
        publish()
        return Grasp.Result(success=True) if grasping else Move.Result(success=True)

    server = ActionServer(node, Move, "franka_gripper/move", execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT)
    grasp_server = ActionServer(node, Grasp, "franka_gripper/grasp", execute,
                                cancel_callback=lambda _: CancelResponse.ACCEPT)

    node.create_timer(.125, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy()
        grasp_server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, help="Motion profile to exercise on isolated mock hardware")
    parser.add_argument("--target-distance", type=float, choices=(.02, .06), default=.02,
                        help="Synthetic forward/reverse TCP target displacement in metres")
    parser.add_argument("--diagonal", action="store_true",
                        help="Exercise simultaneous X/Y/Z motion rather than only X")
    parser.add_argument("--step-response", action="store_true",
                        help="Measure 10–90 percent response to a fixed target instead of a ramp")
    parser.add_argument("--mock-hand-only", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    target_fraction = args.target_distance/.16
    if os.environ.get("ROS_DOMAIN_ID") != "141":
        raise SystemExit("Smooth integration requires isolated domain 141")
    if args.mock_hand_only:
        mock_hand_feedback()
        return
    real_bridge.REAL_DOMAIN, real_bridge.REAL_PORT = "141", 49304
    share = Path(get_package_share_directory("franka_hand_teleop"))
    profile_path = args.profile or share/"config/real_profile.yaml"
    model = subprocess.check_output(["xacro", str(share/"urdf/fake_fr3.urdf.xacro")], text=True)
    root = ET.fromstring(model)
    assert root.find("ros2_control/hardware/plugin").text == "mock_components/GenericSystem"
    root.find("ros2_control").set("name", "FrankaHardwareInterface")
    root.find("ros2_control/hardware/plugin").text = "franka_hand_controllers/ClockedMockSystem"
    gpio = ET.SubElement(root.find("ros2_control"), "gpio", name="fr3")
    ET.SubElement(gpio, "state_interface", name="robot_time")
    model = ET.tostring(root, encoding="unicode")
    assert "FrankaHardwareInterface</plugin>" not in model
    description = Path(get_package_share_directory("franka_description"))
    moveit = Path(get_package_share_directory("franka_fr3_moveit_config"))
    semantic = xacro.process_file(str(description/"robots/fr3/fr3.srdf.xacro")).toxml()
    guard = WorkspaceGuard(model, load_profile(profile_path)[1])
    rclpy.init()
    executor = SingleThreadedExecutor()

    class MockBridge(SmoothBridge):
        EXPECTED_PLUGIN = "franka_hand_controllers/ClockedMockSystem"

        inject_stale_seed = False

        def receive_trajectory(self, message):
            if self.inject_stale_seed:
                # Corrupt the actual Servo input, rather than racing a second
                # publisher for a depth-one queue's brief handoff window.
                message = copy.deepcopy(message)
                message.points[0].positions[0] += .01
            super().receive_trajectory(message)

        def dispatch(self, key, client, goal, timeout):
            if key == "gripper":
                assert self.permit.armed and self.logic.active
                if isinstance(goal, Move.Goal):
                    assert abs(goal.width-self.finger_width) <= .004001
                else:
                    assert self.grasp.phase == "closing" and self.stationary() and self.arm_tail_finished()
                assert goal.speed == .04
                self.last_gripper_command_at = time.monotonic()
            super().dispatch(key, client, goal, timeout)

    class Telemetry(Node):
        def __init__(self):
            super().__init__("rolling_mock_telemetry", namespace="/hand_teleop_real")
            self.joints, self.stream, self.samples = {}, None, []
            self.hand_state = [.04, 0., 0.]
            self.publish_robot = True
            self.desired_bias = 0.
            self.started = time.monotonic()
            self.description = self.create_publisher(String, "robot_description",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.description.publish(String(data=model))
            self.publisher = self.create_publisher(FrankaRobotState,
                "franka_robot_state_broadcaster/robot_state", 1)
            self.create_subscription(JointState, "joint_state_broadcaster/joint_states",
                                     self.joint_state, qos_profile_sensor_data)
            self.create_subscription(JointState, "joint_states", self.joint_state, qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray, "fr3_arm_controller/rolling_state",
                                     self.rolling_state, qos_profile_sensor_data)
            self.create_timer(.01, self.tick)
            self.create_subscription(Float64MultiArray, "mock_hand_state",
                                     lambda msg: setattr(self, "hand_state", list(msg.data)), 1)

        def joint_state(self, message):
            self.joints.update(zip(message.name, message.position))

        def rolling_state(self, message):
            self.stream = list(message.data)
            self.samples.append((time.monotonic(), self.stream))

        def tick(self):
            if not self.publish_robot or not all(n in self.joints for n in guard.names):
                return
            position = [self.joints[n] for n in guard.names]
            velocity = self.stream[44:51] if self.stream else [0.]*7
            msg = FrankaRobotState(robot_mode=FrankaRobotState.ROBOT_MODE_MOVE,
                time=time.monotonic()-self.started, control_command_success_rate=1.)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.measured_joint_state = JointState(name=guard.names, position=position, velocity=velocity)
            desired = list(position)
            desired[0] += self.desired_bias
            msg.desired_joint_state = JointState(name=guard.names, position=desired, velocity=velocity)
            pose = guard.pose(self.joints)
            msg.o_t_ee.pose.position.x, msg.o_t_ee.pose.position.y, msg.o_t_ee.pose.position.z = map(float, pose[:3, 3])
            q = msg.o_t_ee.pose.orientation
            q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(pose[:3, :3]).as_quat())
            assert guard.inside(guard.tcp(self.joints)) and guard.body_inside(self.joints)[0]
            self.publisher.publish(msg)

    telemetry = Telemetry()
    executor.add_node(telemetry)
    bridge, processes = None, []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    camera_feedback = None
    sequence, tracked, control, opening = 0, True, [.5, .5, .5], .5

    def drive(length):
        nonlocal sequence, camera_feedback
        end = time.monotonic()+length
        while time.monotonic() < end:
            assert all(p.poll() in (None, 0) for p in processes), "Mock process failed"
            now = time.monotonic()
            data = {"version": 2, "kind": "tracking", "session": "smooth-mock", "sequence": sequence,
                "monotonic_s": now, "source": "realsense", "tracked": tracked, "units": "m",
                "frame": "camera_color_optical_frame", "palm_m": [0, 0, .65], "opening": opening,
                "control_uvd": control}
            sock.sendto(json.dumps(data).encode(), ("127.0.0.1", 49304))
            sequence += 1
            for _ in range(15):
                executor.spin_once(timeout_sec=0)
            for _ in range(16):
                try:
                    data, address = sock.recvfrom(8192)
                except BlockingIOError:
                    break
                packet = json.loads(data)
                assert address == ("127.0.0.1", 49304)
                assert packet["session"] == "smooth-mock" and packet["kind"] == "tcp_alignment"
                if packet["valid"]:
                    assert np.linalg.norm(np.array(guard.limits.target(packet["tcp_control_uvd"]))+
                                          np.array(packet["mapping_offset_m"])-
                                          packet["tcp_m"]) < 1e-10
                camera_feedback = packet
            time.sleep(.001)

    try:
        with tempfile.TemporaryDirectory() as directory, Path("/tmp/smooth-pipeline-nodes.log").open("w") as log:
            params = Path(directory)/"nodes.yaml"
            servo = yaml.safe_load((share/"config/servo.yaml").read_text())
            servo["moveit_servo"] = {k: v.replace("/hand_teleop_fake/", "/hand_teleop_real/")
                if isinstance(v, str) else v for k, v in servo["moveit_servo"].items()}
            servo["moveit_servo"]["use_smoothing"] = False
            all_params = {"robot_description": model, "robot_description_semantic": semantic,
                "robot_description_planning": yaml.safe_load((moveit/"config/fr3_joint_limits.yaml").read_text()),
                "robot_description_kinematics": yaml.safe_load((moveit/"config/kinematics.yaml").read_text()),
                "hardware_synchronization.expect_blocking_read_write": False,
                "motion_profile": str(profile_path), **servo}
            params.write_text(yaml.safe_dump({"/**": {"ros__parameters": all_params},
                "/**/controller_manager": {"ros__parameters": {"franka_robot_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"}}},
                "/**/franka_robot_state_broadcaster": {"ros__parameters": {"use_local_topics": True}}}))

            def launch(args):
                processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                                   start_new_session=True))

            # Planning must not delay the 125 ms source clock. A shared Python
            # executor falsely starved Hand feedback during longer curve searches.
            launch([sys.executable, str(Path(__file__).resolve()), "--mock-hand-only"])
            launch(["ros2", "run", "controller_manager", "ros2_control_node", "--ros-args", "-r",
                "__ns:=/hand_teleop_real", "--params-file", str(share/"config/real_controllers.yaml"),
                "--params-file", str(params)])
            launch(["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
                "franka_robot_state_broadcaster", "--controller-manager", "/hand_teleop_real/controller_manager"])
            launch(["ros2", "run", "controller_manager", "spawner", "fr3_arm_controller", "--inactive",
                "--controller-manager", "/hand_teleop_real/controller_manager"])
            launch(["ros2", "run", "franka_hand_controllers", "feedback_servo", "--ros-args", "-r",
                "__ns:=/hand_teleop_real", "--params-file", str(params)])
            bridge = MockBridge(namespace="/hand_teleop_real", parameter_overrides=[
                Parameter("robot_description", value=model), Parameter("robot_description_semantic", value=semantic),
                Parameter("profile", value=str(profile_path)), Parameter("execute", value=True)])
            executor.add_node(bridge)
            for _ in range(120):
                drive(.2)
                if bridge.ready:
                    break
            assert bridge.ready, bridge.reason
            drive(.5)
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert answer.success, answer.message
            drive(.7)
            assert bridge.permit.armed, bridge.last_stop_reason
            start = guard.tcp(telemetry.joints)
            initial_goals = telemetry.hand_state[1]
            noise_begin = time.monotonic()
            while time.monotonic()-noise_begin < 1.:
                opening = .5+.01*np.sin(100*(time.monotonic()-noise_begin))
                drive(.02)
                assert bridge.permit.armed, bridge.last_stop_reason
            assert telemetry.hand_state[1] == initial_goals, "Opening noise generated Hand actions"
            opening = .5
            print("PASS: stationary noisy gesture produces no finger chatter or repeated Move goals", flush=True)
            begin = time.monotonic()
            rise_start = rise_finish = None
            while time.monotonic()-begin < 6.:
                fraction = 1. if args.step_response else min(1., (time.monotonic()-begin)/5.)
                control = [.5, .5, .5-target_fraction*fraction]
                if args.diagonal:
                    control[:2] = [.5+args.target_distance/.56*fraction,
                                   .5-target_fraction*fraction]
                drive(.03)
                assert bridge.permit.armed, bridge.last_stop_reason
                progress = guard.tcp(telemetry.joints)[0]-start[0]
                if rise_start is None and progress >= .1*args.target_distance:
                    rise_start = time.monotonic()-begin
                if rise_finish is None and progress >= .9*args.target_distance:
                    rise_finish = time.monotonic()-begin
            measured = guard.tcp(telemetry.joints)
            speeds = [np.linalg.norm(d[44:51]) for stamp, d in telemetry.samples
                      if begin+2. < stamp < begin+4.5]
            assert speeds, "No command samples"
            if not args.step_response and args.target_distance == .06:
                assert min(speeds) > 1e-8, "Rolling tail reached rest during steady target motion"
                print("PASS: sustained 60 mm ramp has no stopped command samples", flush=True)
            if not args.step_response:
                # Faster following can reach the existing 1 mm position deadband
                # during this slow ramp. Rest is valid there; reject pauses while
                # the synthetic target is still meaningfully ahead of the TCP.
                for stamp, sample in telemetry.samples:
                    if begin+2. < stamp < begin+4.5 and np.linalg.norm(sample[44:51]) <= 1e-8:
                        fraction = min(1., (stamp-begin)/5.)
                        expected = guard.limits.target((.5, .5, .5-target_fraction*fraction))[0]
                        tcp = guard.tcp(dict(zip(guard.names, sample[37:44])))
                        assert abs(expected-tcp[0]) <= .0015, (expected, tcp, stamp-begin)
            # In six seconds, follow at least 75% of the five-second target ramp.
            # This catches the former unconditional 1.2 s replacement delay in
            # the 60 mm comparison, which only followed about 40 mm (67%).
            assert measured[0]-start[0] > .75*args.target_distance, (start, measured)
            print(f"PASS: real rolling controller follows without resting outside the target deadband: minimum sampled joint speed={min(speeds):.6f} rad/s; TCP progress={(measured[0]-start[0])*1000:.3f} mm; accepted={bridge.arm_actions_completed}", flush=True)
            assert camera_feedback and camera_feedback["valid"] and camera_feedback["armed"]
            assert camera_feedback["operator_result"]["success"]
            assert camera_feedback["tcp_m"][0]-start[0] > .70*args.target_distance
            print("PASS: live measured TCP moves through inverse mapping on the camera's own UDP session", flush=True)
            if args.step_response:
                held = [guard.tcp(dict(zip(guard.names, sample[37:44])))[0]
                        for stamp, sample in telemetry.samples if begin+4. < stamp < begin+6.]
                expected_x = guard.limits.target(control)[0]
                assert held and max(abs(x-expected_x) for x in held) < .002, (expected_x, min(held), max(held))
                assert max(held)-min(held) < .002, "Fixed target causes persistent arm oscillation"
                print(f"PASS: fixed target settles within 2 mm without hunting; final hold ripple={(max(held)-min(held))*1000:.3f} mm", flush=True)
                assert rise_start is not None and rise_finish is not None, (rise_start, rise_finish)
                if args.target_distance == .06:
                    assert rise_finish-rise_start < 1.8, (rise_start, rise_finish)
                print(f"STEP RESPONSE: distance={args.target_distance*1000:.1f} mm; t10={rise_start:.3f}s; "
                      f"t90={rise_finish:.3f}s; rise_10_90={rise_finish-rise_start:.3f}s", flush=True)
            skipped = bridge.stale_proposals
            bridge.inject_stale_seed = True
            for _ in range(120):
                drive(.01)
                assert bridge.permit.armed, bridge.last_stop_reason
                if bridge.stale_proposals > skipped:
                    break
            bridge.inject_stale_seed = False
            assert bridge.stale_proposals > skipped, "Stale IK proposal was not exercised"
            print("PASS: stale IK proposals retain the 2 mrad rejection threshold, grant no new motion, and wait for fresh feedback without disarming", flush=True)
            # Delay complete candidates in transport without blocking feedback or
            # changing the controller's 20 ms admission guard (mock domain only).
            delayed = []
            candidate = bridge.candidate

            def defer_candidate(message):
                delayed.append((time.monotonic()+.15, copy.deepcopy(message)))

            def flush_candidates():
                while delayed and delayed[0][0] <= time.monotonic():
                    candidate.publish(delayed.pop(0)[1])

            delay_timer = telemetry.create_timer(.005, flush_candidates)
            retries, accepted = bridge.deadline_retries, bridge.arm_actions_completed
            with patch.object(bridge, "candidate", SimpleNamespace(publish=defer_candidate)):
                drive(1.2)
                assert bridge.permit.armed, bridge.last_stop_reason
                assert bridge.deadline_retries >= retries+3, bridge.deadline_retries
                assert bridge.arm_actions_completed == accepted
                # Drain the final pending packet through its explicit rejection.
                for _ in range(50):
                    if not delayed and bridge.pending_sequence is None:
                        break
                    drive(.005)
            drive(.7)
            telemetry.destroy_timer(delay_timer)
            assert bridge.permit.armed and bridge.arm_actions_completed > accepted, bridge.last_stop_reason
            print("PASS: repeated transport deadline misses retain the checked finite tail without disarming; fresh validated updates resume and the 20 ms controller guard stays enforced", flush=True)
            original_plan = smooth_bridge.checked_curve
            skips, rejected = bridge.handoff_budget_skips, telemetry.stream[5]

            def delayed_plan(*args):
                time.sleep(.06)
                return original_plan(*args)

            with patch.object(smooth_bridge, "checked_curve", delayed_plan):
                drive(.4)
            assert bridge.permit.armed, bridge.last_stop_reason
            assert bridge.handoff_budget_skips > skips and telemetry.stream[5] == rejected
            drive(.3)
            assert bridge.permit.armed, bridge.last_stop_reason
            print("PASS: injected planning delay retains checked tail without controller rejection; fresh updates resume", flush=True)
            with patch.object(bridge.grasp, "accepts_heights", return_value=False):
                drive(1.2)
                assert bridge.permit.armed, bridge.last_stop_reason
                assert time.monotonic()-bridge.finger_stamp < .15
                assert time.monotonic()-bridge.robot_stamp < .15
            drive(.3)
            assert bridge.permit.armed, bridge.last_stop_reason
            print("PASS: continuously inadmissible height proposals yield within the planning budget; arm/Hand feedback stays fresh and the checked finite tail remains valid", flush=True)
            control = [.5, .5, .5+target_fraction]
            forward = measured[0]
            drive(5.)
            assert bridge.permit.armed and guard.tcp(telemetry.joints)[0] < forward-.002, bridge.last_stop_reason
            control = [.5, .5, .5]
            drive(4.)  # Restore capture alignment before loss/re-arm checks for larger targets.
            assert bridge.permit.armed, bridge.last_stop_reason
            assert bridge.gripper_action is not None
            initial_width = telemetry.hand_state[0]
            opening = .8
            drive(1.2)
            assert bridge.permit.armed and telemetry.hand_state[0] > initial_width+.001, bridge.last_stop_reason
            # Close while an opening is still active, including asynchronous cancellation.
            for _ in range(100):
                drive(.01)
                active_move = bridge.actions.get("gripper")
                if (active_move and active_move.get("opening_move") and active_move.get("handle") and
                        time.monotonic()-bridge.last_gripper_command_at < .04):
                    break
            assert (active_move and active_move.get("opening_move") and active_move.get("handle") and
                    time.monotonic()-bridge.last_gripper_command_at < .04), "No fresh opening to preempt"
            cancellations_before_close = telemetry.hand_state[2]
            opened_width = telemetry.hand_state[0]
            opening = .2
            close_started = time.monotonic()
            for _ in range(20):
                drive(.01)
                if bridge.grasp.phase == "settling":
                    break
            assert bridge.grasp.phase == "settling", "Opening action starved close intent"
            assert time.monotonic()-close_started < .2
            for _ in range(150):
                drive(.02)
                if bridge.grasp.phase == "closing":
                    break
            assert bridge.grasp.phase == "closing", bridge.last_stop_reason
            assert telemetry.hand_state[2] > cancellations_before_close
            assert bridge.permit.armed, bridge.last_stop_reason
            print("PASS: close gesture preempts in-flight opening, immediately holds arm, and waits for cancellation before one Grasp without disarming", flush=True)
            hold = guard.tcp(telemetry.joints)
            control = [.55, .60, .45]  # Move all hand coordinates during closure.
            for _ in range(200):
                drive(.02)
                assert bridge.permit.armed, bridge.last_stop_reason
                assert np.linalg.norm(np.array(guard.tcp(telemetry.joints))-hold) < .0002
                if bridge.grasp.phase == "lift":
                    break
            assert bridge.grasp.phase == "lift", bridge.grasp.phase
            assert telemetry.hand_state[0] < opened_width-.001 and telemetry.hand_state[3] == 1
            target = bridge.motion_target(guard.tcp(telemetry.joints))
            assert np.linalg.norm(np.array(target)-hold) < .0002
            control = [.58, .65, .45]  # X/Y remains live; downward Z is forbidden.
            drive(2.)
            assert bridge.permit.armed and bridge.grasp.phase == "lift", bridge.last_stop_reason
            lifted = guard.tcp(telemetry.joints)
            assert lifted[1] > hold[1]+.003 and lifted[2] >= hold[2]-.0001, (hold, lifted)
            print("PASS: Grasp waits for the finite arm tail, freezes XYZ through confirmation, and re-clutches without chasing the hand; lift keeps X/Y live and blocks downward Z", flush=True)
            control = [.58, .20, .45]
            for _ in range(400):
                drive(.02)
                assert bridge.permit.armed, bridge.last_stop_reason
                if bridge.grasp.phase == "carry":
                    break
            assert bridge.grasp.phase == "carry", (bridge.grasp.status(), guard.tcp(telemetry.joints))
            assert guard.tcp(telemetry.joints)[2] >= hold[2]+.05
            floor = bridge.grasp.floor
            control = [.58, .99, .45]
            drive(4.)
            assert bridge.permit.armed and guard.tcp(telemetry.joints)[2] >= floor-.0001, bridge.last_stop_reason
            print("PASS: measured 5 cm lift unlocks normal Z; validated trajectories retain grasp-height +1 cm floor", flush=True)
            resume_control = list(guard.limits.camera_control(tuple(p-o for p, o in
                zip(guard.tcp(telemetry.joints), bridge.grasp.offset))))
            opening = 1.
            opening_changed_at = time.monotonic()
            cancellations = telemetry.hand_state[2]
            for _ in range(30):
                drive(.01)
                if (getattr(bridge, "last_gripper_command_at", 0) >= opening_changed_at and
                        bridge.actions.get("gripper", {}).get("handle")):
                    break
            assert (getattr(bridge, "last_gripper_command_at", 0) >= opening_changed_at and
                    bridge.actions.get("gripper", {}).get("handle")), "No new active gripper goal to cancel"
            tracked = False
            drive(1.8)
            assert not bridge.permit.armed and telemetry.stream[4] == 0
            assert "Tracking" in bridge.last_stop_reason, bridge.last_stop_reason
            assert max(map(abs, telemetry.stream[44:58])) < 1e-8
            assert not bridge.actions and telemetry.hand_state[2] > cancellations, (bridge.actions, telemetry.hand_state)
            opening = .5
            print("PASS: reversal remains continuous; tracking loss fences session and finite tail reaches rest", flush=True)
            print("PASS: tracking loss cancels an in-flight gripper Move and stops both arm and Hand", flush=True)
            tracked, control = True, resume_control
            drive(.5)
            assert not bridge.permit.armed
            assert camera_feedback is not None and camera_feedback["valid"] and not camera_feedback["armed"]
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert answer.success, answer.message
            drive(.7)
            assert bridge.grasp.floor == floor, "Disarm/re-arm cleared the carry floor"
            opening = 1.
            for _ in range(650):
                drive(.02)
                assert bridge.permit.armed, bridge.last_stop_reason
                if bridge.grasp.phase == "normal":
                    break
                assert bridge.grasp.floor == floor
            assert bridge.grasp.phase == "normal" and bridge.grasp.floor is None
            assert telemetry.hand_state[0] >= .0795
            assert np.linalg.norm(np.array(bridge.motion_target(guard.tcp(telemetry.joints)))-
                                  guard.tcp(telemetry.joints)) < .0003
            opening = .5
            print("PASS: floor survives tracking loss/disarm/re-arm and clears only after confirmed full opening, with no target jump", flush=True)
            telemetry.desired_bias = .0021
            drive(.15)
            assert not bridge.permit.armed and "0.002 rad" in bridge.last_stop_reason
            telemetry.desired_bias = 0.
            drive(1.8)
            print("PASS: desired/measured tracking error retains 0.002 rad rejection during motion", flush=True)
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert answer.success, answer.message
            drive(.7)
            telemetry.publish_robot = False
            drive(.3)
            assert not bridge.permit.armed and "feedback" in bridge.last_stop_reason
            assert camera_feedback is not None and not camera_feedback["valid"]
            telemetry.publish_robot = True
            drive(1.8)
            assert max(map(abs, telemetry.stream[44:58])) < 1e-8
            print("PASS: stale robot telemetry disarms and stops; feedback return does not re-arm", flush=True)
            diagnostic = bridge.robot_history.summary(time.monotonic())
            assert diagnostic["samples"] > 0 and diagnostic["configured_inertia"] is not None
            # Probe the rejection telemetry on the isolated, disarmed mock only.
            rejected = telemetry.create_publisher(JointTrajectory, "fr3_arm_controller/rolling_trajectory", 1)
            drive(.2)
            d = telemetry.stream
            packet = JointTrajectory(joint_names=guard.names)
            packet.header.frame_id = f"rolling1/{int(d[2])}/{int(d[3])+1}/{int(d[3])}"
            stamp = real_bridge.duration(d[1]-.01)
            packet.header.stamp.sec, packet.header.stamp.nanosec = stamp.sec, stamp.nanosec
            hold = list(d[37:44])
            packet.points = [JointTrajectoryPoint(positions=hold, velocities=[0.]*7, accelerations=[0.]*7),
                JointTrajectoryPoint(positions=hold, velocities=[0.]*7, accelerations=[0.]*7,
                                     time_from_start=real_bridge.duration(.1))]
            rejected.publish(packet)
            drive(.3)
            assert bridge.last_rejection and "less than 20 ms lead" in bridge.last_rejection["reasons"]
            assert "stream disabled" in bridge.last_rejection["reasons"]
            assert not bridge.permit.armed and max(abs(a-b) for a, b in zip(hold, telemetry.stream[37:44])) < 1e-12
            print("PASS: controller rejection reasons/lead reach the bridge; diagnostic probe never moves the disarmed mock", flush=True)
            obstacle = CollisionObject(id="smooth_collision_fixture", operation=CollisionObject.ADD)
            obstacle.header.frame_id = "fr3_link0"
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.4]*3)]
            pose = Pose(position=Point(x=.55, y=0., z=.15))
            pose.orientation.w = 1.
            obstacle.primitive_poses = [pose]
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects = [obstacle]
            bridge.scene.publish(scene)
            drive(.5)
            assert not bridge.ready
            print("PASS: scene collision refuses arming; all measured TCP/whole-arm states stayed inside workspace", flush=True)
    finally:
        if bridge:
            executor.remove_node(bridge)
            bridge.close()
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=3)
        telemetry.destroy_node()
        sock.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
