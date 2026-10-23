"""End-to-end check using only the package's dedicated mock model and domain."""
import json
import math
import os
import signal
import socket
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_hand_teleop.core import DOMAIN, NAMESPACE, PORT, Limits
from franka_hand_teleop.workspace_guard import WorkspaceGuard
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN:
        raise SystemExit(f"Check requires ROS_DOMAIN_ID={DOMAIN}")
    share = Path(get_package_share_directory("franka_hand_teleop"))
    config = yaml.safe_load((share/"config/teleop.yaml").read_text())["/**/hand_bridge"]["ros__parameters"]
    limits = Limits(minimum=tuple(config["workspace_min"]), maximum=tuple(config["workspace_max"]),
                    margin=config["workspace_margin"], speed=config["max_speed"],
                    body_minimum=tuple(config["arm_workspace_min"]),
                    body_maximum=tuple(config["arm_workspace_max"]))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", PORT))
    log_path = Path("/workspace/logs/hand-teleop-check.log")
    rclpy.init()
    node = rclpy.create_node("hand_fake_integration_check")
    state, joints = {}, {}

    def status(message):
        state.update(json.loads(message.data))

    def feedback(message):
        joints.update(zip(message.name, message.position))

    node.create_subscription(String, f"{NAMESPACE}/status", status, 1)
    node.create_subscription(JointState, f"{NAMESPACE}/joint_states", feedback,
                             qos_profile_sensor_data)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session, sequence = uuid.uuid4().hex, 0
    process = None
    default_control = [.5, .5, .5]
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                "ros2", "launch", "franka_hand_teleop", "fake.launch.py", "use_rviz:=false",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

            def drive(duration, palm=None, opening=.8, tracking=True, tracked=True, control=None):
                nonlocal sequence
                started = time.monotonic()
                while time.monotonic() - started < duration:
                    if process.poll() is not None:
                        raise RuntimeError(f"Fake launch exited; inspect {log_path}")
                    now = time.monotonic()
                    if tracking:
                        point = palm(now-started) if callable(palm) else palm or [0, 0, .65]
                        uvdepth = control(now-started) if callable(control) else control or default_control
                        packet = {"version": 2, "kind": "tracking", "monotonic_s": now,
                                  "session": session, "sequence": sequence, "source": "demo",
                                  "frame": "camera_color_optical_frame", "units": "m",
                                  "tracked": tracked, "palm_m": point, "opening": opening,
                                  "control_uvd": uvdepth}
                        sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", PORT))
                        sequence += 1
                    rclpy.spin_once(node, timeout_sec=.01)
                    if state.get("tcp_m"):
                        assert limits.contains(state["tcp_m"]), state
                        assert math.sqrt(sum(v*v for v in state["velocity_m_s"])) <= limits.speed+1e-6
                        assert math.dist(state["tcp_m"], state["fk_tcp_m"]) < .005, state
                        assert state["whole_arm_inside"], state
                    time.sleep(.01)

            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                drive(.2, tracking=False)
                if state.get("ready"):
                    break
            else:
                raise RuntimeError(f"Fake stack not ready: {state}; inspect {log_path}")
            assert state["backend_verified"] and state["fake_only"]
            print("PASS: mock backend verified; Servo and tool feedback ready", flush=True)
            initial = tuple(state["tcp_m"])
            assert math.dist(initial, (.55, 0, .15)) < .001, initial
            print("PASS: mock initializes inside the measured tool box at [0.55, 0, 0.15] m", flush=True)
            fractions = [(p-a-limits.margin)/(b-a-2*limits.margin)
                         for p, a, b in zip(initial, limits.minimum, limits.maximum)]
            default_control = [fractions[1], 1-fractions[2], 1-fractions[0]]
            drive(.4, tracked=False)
            assert not state["active"] and math.dist(initial, state["tcp_m"]) < .002
            print("PASS: missing tracking cannot move the robot", flush=True)
            drive(.6)
            assert state["active"], state
            anchor = tuple(state["tcp_m"])
            assert math.dist(initial, anchor) < .002
            print("PASS: absolute camera coordinate maps to the expected stationary target", flush=True)
            drive(2., control=[default_control[0]+.04, default_control[1], default_control[2]])
            assert state["tcp_m"][1] > anchor[1]+.005, state
            drive(2., control=[default_control[0]-.04, default_control[1], default_control[2]])
            assert state["tcp_m"][1] < anchor[1]-.005, state
            drive(2.)
            print("PASS: reversed horizontal mapping moves raw camera +U toward +Y and -U toward -Y", flush=True)
            drive(2.0, control=lambda t: [default_control[0], default_control[1]-.08*min(t, 1), default_control[2]])
            assert state["active"] and state["tcp_m"][2] - anchor[2] > .005, state
            assert joints["fr3_finger_joint1"] > .005, joints
            assert abs(joints["fr3_finger_joint1"] - joints["fr3_finger_joint2"]) < .001
            print("PASS: absolute hand movement and both simulated fingers move", flush=True)
            drive(.5, tracked=False)
            stopped, fingers = tuple(state["tcp_m"]), joints["fr3_finger_joint1"]
            drive(.4, tracked=False, opening=0)
            assert not state["active"] and math.dist(stopped, state["tcp_m"]) < .002
            assert abs(joints["fr3_finger_joint1"] - fingers) < .001
            print("PASS: tracking loss stops arm and fingers", flush=True)
            drive(.6, palm=[.04, 0, .65], control=[default_control[0], 1, default_control[2]])
            assert state["active"] and abs(state["target_m"][2] -
                                           (limits.minimum[2]+limits.margin)) < 1e-6, state
            assert state["tcp_m"][2] < stopped[2]-.001, state
            print("PASS: re-entry at image bottom targets workspace bottom, without rebasing", flush=True)
            drive(.6, tracking=False)
            assert not state["active"], state
            drive(.15, palm=[.02, 0, .65], control=[default_control[0], 0, default_control[2]])
            assert not state["active"], state
            drive(.4, palm=[.02, 0, .65], control=[default_control[0], 0, default_control[2]])
            assert state["active"] and abs(state["target_m"][2] -
                                           (limits.maximum[2]-limits.margin)) < 1e-6, state
            drive(4.0, control=[default_control[0], -10, default_control[2]])
            assert abs(state["target_m"][2] - (limits.maximum[2]-limits.margin)) < .0001, state
            print("PASS: image top always targets workspace top; out-of-range input saturates with 20 mm clearance", flush=True)
            urdf = subprocess.check_output([
                "xacro", str(share/"urdf/fake_fr3.urdf.xacro")
            ], text=True)
            guard = WorkspaceGuard(urdf, limits)
            rejected = state["guard_rejections"]
            unsafe = None
            for name in guard.names:
                for bound in guard.bounds[name]:
                    candidate = dict(joints)
                    candidate[name] = bound
                    if not guard.inside(guard.tcp(candidate), pad=0):
                        unsafe = [candidate[n] for n in guard.names]
                        break
                if unsafe:
                    break
            assert unsafe is not None
            raw = node.create_publisher(JointTrajectory, f"{NAMESPACE}/servo_trajectory", 1)
            drive(.2, tracking=False)
            message = JointTrajectory(joint_names=guard.names)
            point = JointTrajectoryPoint(positions=unsafe)
            point.time_from_start.nanosec = 100_000_000
            message.points = [point]
            raw.publish(message)
            drive(.3, tracking=False)
            assert state["guard_rejections"] > rejected, state
            print("PASS: an injected out-of-workspace trajectory is rejected before the fake controller", flush=True)
            # Pinned FR3 model: TCP is inside the small box, while link5 is
            # outside the whole-arm box. This reproduces the user's distinction.
            elbow_outside = [.749397687872353, 1.7222340194433825, -.9673354117156465,
                             -1.8960300269866828, -1.7030880892607971, .6608201372321354,
                             -1.7459775214379774]
            candidate = dict(zip(guard.names, elbow_outside))
            assert guard.inside(guard.tcp(candidate))
            assert not guard.body_inside(candidate)[0]
            rejected = state["guard_rejections"]
            point.positions = elbow_outside
            raw.publish(message)
            drive(.3, tracking=False)
            assert state["guard_rejections"] > rejected, state
            print("PASS: whole-arm violation is rejected even when TCP stays inside its workspace", flush=True)
            drive(.4, tracking=False)
            print(f"Fake hand teleop check passed. Log: {log_path}", flush=True)
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)


if __name__ == "__main__":
    main()
