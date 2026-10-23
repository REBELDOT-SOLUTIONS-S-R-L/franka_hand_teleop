"""Real bridge + installed Servo + synthetic arm; NO physical drivers or robot IP."""
import argparse
import importlib.util
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_hand_teleop import real_bridge
from franka_hand_teleop.deployment import load_profile
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feedback-servo", action="store_true", help="Compatibility flag; seeded feedback Servo is always used")
    parser.add_argument("--split-feedback", action="store_true", help="Separate 1 kHz arm and 30 Hz gripper writers")
    parser.add_argument("--gripper-period", type=float, default=1/30)
    parser.add_argument("--expect-readiness-flicker", action="store_true")
    parser.add_argument("--joint-bias", type=float, default=0., help="Synthetic measured/desired offset, rad")
    parser.add_argument("--plant-lag", type=float, default=0., help="Synthetic plant time constant, seconds")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "138":
        raise SystemExit("Servo pipeline fixture requires isolated domain 138")
    share = Path(get_package_share_directory("franka_hand_teleop"))
    spec = importlib.util.spec_from_file_location("offline_fixture", share/"tools/check_real_offline.py")
    fixture_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture_module)
    # Test-process globals only. Production files and their hardware gate are
    # unchanged. DDS domain and UDP port are separate from all operator sessions.
    real_bridge.REAL_DOMAIN, real_bridge.REAL_PORT = "138", 49302

    class MockBridge(real_bridge.RealBridge):
        EXPECTED_PLUGIN = "mock_components/GenericSystem"
        max_forecast_gap = 0.0

        def receive_trajectory(self, message):
            if self.joints and message.points:
                forecast = dict(zip(message.joint_names, message.points[-1].positions))
                self.max_forecast_gap = max(self.max_forecast_gap,
                    math.dist(self.guard.tcp(self.joints), self.guard.tcp(forecast)))
            super().receive_trajectory(message)

    class ServoFixture(fixture_module.Fixture):
        publish_servo_joints = True
        publish_fingers = True
        publish_arm_states = True
        def tick(self):
            super().tick()
            measured = dict(zip(self.guard.names, self.q))
            assert self.guard.inside(self.guard.tcp(measured), pad=0), "Synthetic TCP left workspace"
            valid, reason = self.guard.body_inside(measured)
            assert valid, reason
        def publish_joints(self, message):
            if not self.publish_servo_joints or not self.publish_fingers:
                return
            if args.split_feedback:
                if time.monotonic()-getattr(self, "last_fingers", 0) >= args.gripper_period:
                    self.finger_pub.publish(message)
                    self.last_fingers = time.monotonic()
                return
            self.finger_pub.publish(message)

    model = subprocess.check_output(["xacro", str(share/"urdf/fake_fr3.urdf.xacro")], text=True)
    assert "mock_components/GenericSystem" in model and "FrankaHardwareInterface" not in model
    description = Path(get_package_share_directory("franka_description"))
    moveit = Path(get_package_share_directory("franka_fr3_moveit_config"))
    semantic = xacro.process_file(str(description/"robots/fr3/fr3.srdf.xacro")).toxml()
    profile, limits = load_profile(share/"config/real_profile.yaml")
    initial = yaml.safe_load((share/"config/initial_positions.yaml").read_text())
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    fixture = ServoFixture(model, limits, initial, mock_servo=False)
    fixture.destroy_publisher(fixture.finger_pub)
    fixture.finger_pub = fixture.create_publisher(JointState, "gripper_joint_states", 1)
    fixture.joint_bias = np.array([args.joint_bias*(-1 if i % 2 else 1) for i in range(7)])
    fixture.qd = fixture.q+fixture.joint_bias
    fixture.plant_lag_s = args.plant_lag
    fixture.encoder_noise = 1e-7
    arm_pub = fixture.create_publisher(JointState, "joint_state_broadcaster/joint_states", 1)

    def publish_arm():
        if fixture.publish_servo_joints and fixture.publish_arm_states:
            joints = JointState(name=fixture.guard.names, position=fixture.q.tolist(), velocity=fixture.dq.tolist())
            joints.header.stamp = fixture.get_clock().now().to_msg()
            arm_pub.publish(joints)
    fixture.create_timer(.001 if args.split_feedback else .01, publish_arm)
    fixture.plugin = "mock_components/GenericSystem"
    executor.add_node(fixture)
    bridge = None
    processes = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence, control, tracked = 0, [.5, .5, .5], True

    def drive(length):
        nonlocal sequence
        deadline = time.monotonic()+length
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), "Pipeline node exited; inspect log"
            now = time.monotonic()
            packet = {"version": 2, "kind": "tracking", "session": "servo-pipeline-fixture", "sequence": sequence,
                "monotonic_s": now, "source": "realsense", "tracked": tracked, "units": "m",
                "frame": "camera_color_optical_frame", "palm_m": [0, 0, .65], "opening": .5, "control_uvd": control}
            sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", 49302))
            sequence += 1
            for _ in range(12):
                executor.spin_once(timeout_sec=0)
            time.sleep(.001)

    log_path = Path("/workspace/logs/real-servo-pipeline-check.log")
    try:
        with tempfile.TemporaryDirectory() as directory, log_path.open("w") as log:
            path = Path(directory)/"profile.yaml"
            profile["commissioning"] = {k: True for k in profile["commissioning"]}
            path.write_text(yaml.safe_dump(profile))
            bridge = MockBridge(namespace="/hand_teleop_real", parameter_overrides=[
                Parameter("robot_description", value=model), Parameter("robot_description_semantic", value=semantic),
                Parameter("profile", value=str(path)), Parameter("execute", value=True)])
            executor.add_node(bridge)
            servo = yaml.safe_load((share/"config/servo.yaml").read_text())
            servo["moveit_servo"] = {k: v.replace("/hand_teleop_fake/", "/hand_teleop_real/")
                if isinstance(v, str) else v for k, v in servo["moveit_servo"].items()}
            servo["moveit_servo"]["use_smoothing"] = False
            parameters = {"robot_description": model, "robot_description_semantic": semantic,
                "robot_description_planning": yaml.safe_load((moveit/"config/fr3_joint_limits.yaml").read_text()),
                "motion_profile": str(share/"config/real_profile.yaml"),
                "robot_description_kinematics": yaml.safe_load((moveit/"config/kinematics.yaml").read_text()), **servo}
            params = Path(directory)/"nodes.yaml"
            params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
            servo_command = ("franka_hand_controllers", "feedback_servo")
            for package, executable in (("robot_state_publisher", "robot_state_publisher"), servo_command):
                processes.append(subprocess.Popen(["ros2", "run", package, executable, "--ros-args",
                    "-r", "__ns:=/hand_teleop_real", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
                    "--params-file", str(params)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            for _ in range(100):
                drive(.2)
                if bridge.ready:
                    break
            assert bridge.ready, bridge.reason
            if args.expect_readiness_flicker:
                states, reasons = set(), set()
                for _ in range(40):
                    drive(.05)
                    states.add(bridge.ready)
                    if not bridge.ready:
                        reasons.add(bridge.reason)
                    answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
                    assert not answer.success, "Flickering readiness unexpectedly allowed arming"
                assert states == {False, True}, states
                assert any("fr3_finger_joint1" in r for r in reasons), reasons
                print(f"PASS: slow gripper feedback reproduces flickering readiness and refuses arming: {reasons}", flush=True)
                return
            drive(.5)
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert answer.success, answer.message
            drive(.5)
            assert bridge.permit.armed, bridge.permit.reason
            start = fixture.guard.tcp(dict(zip(fixture.guard.names, fixture.q)))
            control = [.5, .5, .375]  # 2 cm towards +X, well inside the operating box.
            for _ in range(90):
                drive(.2)
                if not bridge.permit.armed:
                    break
            end = fixture.guard.tcp(dict(zip(fixture.guard.names, fixture.q)))
            print(f"Pipeline displacement mm: {[(b-a)*1000 for a, b in zip(start, end)]}; "
                  f"actions={fixture.count}; armed={bridge.permit.armed}; stop={bridge.last_stop_reason}; "
                  f"maximum proposal gap={bridge.max_forecast_gap*1000:.1f} mm", flush=True)
            assert bridge.permit.armed and bridge.guard_rejections == 0, bridge.last_stop_reason
            assert end[0]-start[0] > .006, (start, end, bridge.last_motion_result)
            # The first target is 20 mm away; a 200 ms IK proposal may extend
            # gain*error*0.2 beyond its measured seed. Keep a 5 mm allowance for
            # FK/nonlinear IK and callback timing. Executed actions retain their
            # independent travel/path limits; this is not an output acceptance gate.
            proposal_limit = min(limits.speed, limits.gain*.02)*.20+.005
            assert bridge.max_forecast_gap < proposal_limit, (bridge.max_forecast_gap, proposal_limit)
            assert bridge.last_segment and bridge.last_motion_result
            assert bridge.last_segment["proposal_horizon_s"] == .20
            if args.joint_bias or args.plant_lag:
                assert np.max(np.abs(fixture.qd-fixture.q)) > 1e-5
                print(f"PASS: target progress with persistent joint bias={args.joint_bias} rad "
                      f"and plant lag={args.plant_lag} s", flush=True)
            forward = end[0]
            control = [.5, .5, .625]  # Reverse target to 2 cm towards -X.
            drive(8.)
            reverse = fixture.guard.tcp(dict(zip(fixture.guard.names, fixture.q)))[0]
            assert bridge.permit.armed and reverse < forward-.002, (forward, reverse, bridge.last_stop_reason)
            control = [.5, .5, .5]  # Restore capture near the actual TCP for subsequent arm tests.
            tracked = False
            drive(.4)
            assert not bridge.permit.armed
            print("PASS: closed-loop target progress without forecast drift; hand loss disarms", flush=True)
            tracked = True
            drive(1.7)
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert answer.success, answer.message
            drive(.2)
            assert bridge.permit.armed
            if args.split_feedback:
                fixture.publish_fingers = False
                drive(.25)
                assert not bridge.permit.armed and "fr3_finger_joint1" in bridge.last_stop_reason
                fixture.publish_fingers = True
                drive(1.7)
                assert bridge.ready and not bridge.permit.armed
                answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
                assert answer.success, answer.message
                drive(.2)
                assert bridge.permit.armed
                print("PASS: gripper feedback loss disarms with fresh arm state; restoring feedback does not re-arm", flush=True)
                fixture.publish_arm_states = False
                drive(.145)
                assert not bridge.permit.armed and "Arm joint feedback" in bridge.last_stop_reason
                fixture.publish_arm_states = True
                drive(1.7)
                assert bridge.ready and not bridge.permit.armed
                answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
                assert answer.success, answer.message
                drive(.2)
                assert bridge.permit.armed
                print("PASS: arm-only feedback loss still disarms on the fast deadline with fresh gripper/Franka state", flush=True)
            fixture.publish_servo_joints = False
            drive(.3)
            assert not bridge.permit.armed and "Servo" in bridge.last_stop_reason
            fixture.publish_servo_joints = True
            drive(.5)
            assert bridge.ready and not bridge.permit.armed
            print("PASS: stale IK joint feedback disarms despite fresh synthetic Franka state; no auto re-arm", flush=True)
            obstacle = CollisionObject(id="offline_collision_block", operation=CollisionObject.ADD)
            obstacle.header.frame_id = "fr3_link0"
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.4, .4, .4])]
            pose = Pose(position=Point(x=.55, y=0., z=.15))
            pose.orientation.w = 1.
            obstacle.primitive_poses = [pose]
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects = [obstacle]
            bridge.scene.publish(scene)
            drive(.6)
            assert not bridge.ready and bridge.servo_status.code == 5
            answer = bridge.arm_request(SetBool.Request(data=True), SetBool.Response())
            assert not answer.success
            print("PASS: live Servo collision checking blocks arming", flush=True)
    finally:
        if bridge:
            executor.remove_node(bridge)
            bridge.close()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        fixture.destroy_node()
        executor.shutdown()
        sock.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
