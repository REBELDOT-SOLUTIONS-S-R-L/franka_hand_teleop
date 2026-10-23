"""Run the real controller settings on GenericSystem only, isolated in domain 134."""
import os
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from franka_hand_teleop.deployment import load_profile
from franka_hand_teleop.real_bridge import duration
from franka_hand_teleop.workspace_guard import WorkspaceGuard
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "134":
        raise SystemExit("Mock controller check requires ROS_DOMAIN_ID=134")
    share = Path(get_package_share_directory("franka_hand_teleop"))
    model = subprocess.check_output(["xacro", str(share/"urdf/fake_fr3.urdf.xacro")], text=True)
    plugins = [p.text for p in ET.fromstring(model).findall("ros2_control/hardware/plugin")]
    assert plugins == ["mock_components/GenericSystem"], plugins
    root = ET.fromstring(model)
    root.find("ros2_control/hardware/plugin").text = "franka_hand_controllers/ClockedMockSystem"
    clock = ET.SubElement(root.find("ros2_control"), "gpio", name="fr3")
    ET.SubElement(clock, "state_interface", name="robot_time")
    model = ET.tostring(root, encoding="unicode")
    guard = WorkspaceGuard(model, load_profile(share/"config/real_profile.yaml")[1])
    log_path = Path("/workspace/logs/real-controller-mock-check.log")
    processes, joints = [], {}
    rclpy.init()
    node = rclpy.create_node("real_controller_mock_check")
    namespace = "/hand_real_controller_test"
    description = node.create_publisher(String, namespace+"/robot_description",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    description.publish(String(data=model))
    node.create_subscription(JointState, namespace+"/joint_state_broadcaster/joint_states",
                             lambda msg: joints.update(zip(msg.name, msg.position)), qos_profile_sensor_data)
    client = ActionClient(node, FollowJointTrajectory, namespace+"/fr3_arm_controller/follow_joint_trajectory")
    hardware = node.create_client(ListHardwareComponents, namespace+"/controller_manager/list_hardware_components")

    def spin_until(predicate, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if joints:
                assert guard.inside(guard.tcp(joints)) and guard.body_inside(joints)[0]
            if predicate():
                return
        raise RuntimeError(f"Mock controller check timed out; see {log_path}")

    try:
        with tempfile.TemporaryDirectory() as directory, log_path.open("w") as log:
            params = Path(directory)/"model.yaml"
            params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": model}}}))
            # Legacy action-controller regression remains separate from new rolling integration.
            legacy = Path(directory)/"legacy.yaml"
            legacy.write_text((share/"config/real_controllers.yaml").read_text().replace(
                "GuardedRollingController", "RobotClockTrajectoryController"))
            # GenericSystem does not block on hardware packets; its test must
            # use the normal periodic loop rather than the real FCI clock.
            timing = Path(directory)/"mock-timing.yaml"
            timing.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
                "hardware_synchronization.expect_blocking_read_write": False}}}))
            processes.append(subprocess.Popen(["ros2", "run", "controller_manager", "ros2_control_node",
                "--ros-args", "-r", f"__ns:={namespace}", "--params-file", str(legacy),
                "--params-file", str(params), "--params-file", str(timing)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            spin_until(hardware.service_is_ready)
            future = hardware.call_async(ListHardwareComponents.Request())
            spin_until(future.done)
            items = future.result().component
            assert len(items) == 1 and items[0].plugin_name == "franka_hand_controllers/ClockedMockSystem"
            processes.append(subprocess.Popen(["ros2", "run", "controller_manager", "spawner",
                "joint_state_broadcaster", "fr3_arm_controller", "--controller-manager", namespace+"/controller_manager"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            spin_until(lambda: client.server_is_ready() and all(n in joints for n in guard.names))
            # JTC creates its action server during configuration, before the
            # spawner finishes activating it. Do not confuse discovery with readiness.
            spin_until(lambda: processes[-1].poll() is not None)
            assert processes[-1].returncode == 0, f"Controller spawner failed; see {log_path}"

            def goal(offset, total):
                start = [joints[n] for n in guard.names]
                target = start.copy()
                target[0] += offset
                assert guard.check(joints, [target])[0]
                request = FollowJointTrajectory.Goal()
                request.trajectory.joint_names = guard.names
                request.trajectory.points = [
                    JointTrajectoryPoint(positions=start, velocities=[0.]*7, accelerations=[0.]*7),
                    JointTrajectoryPoint(positions=target, velocities=[0.]*7, accelerations=[0.]*7,
                                         time_from_start=duration(total))]
                accepted = client.send_goal_async(request)
                spin_until(accepted.done)
                assert accepted.result().accepted
                return accepted.result(), target

            handle, target = goal(.001, .6)
            result = handle.get_result_async()
            spin_until(result.done, 3)
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED, result.result()
            # Result RPC and best-effort joint feedback are separate streams.
            # Wait for fresh endpoint feedback instead of asserting on an older sample.
            try:
                spin_until(lambda: abs(joints[guard.names[0]]-target[0]) < .0001, .5)
            except RuntimeError as error:
                raise AssertionError(f"Mock endpoint mismatch: actual={joints[guard.names[0]]}, target={target[0]}") from error
            print("PASS: real 1 kHz controller settings accept and execute a zero-end-velocity/acceleration quintic on mock hardware", flush=True)
            handle, target = goal(.005, 1.)
            start = time.monotonic()
            spin_until(lambda: time.monotonic()-start > .2)
            cancel = handle.cancel_goal_async()
            spin_until(cancel.done)
            result = handle.get_result_async()
            spin_until(result.done, 3)
            assert result.result().status == GoalStatus.STATUS_CANCELED, result.result()
            assert joints[guard.names[0]] <= target[0]+.0001
            print("PASS: installed controller supports configured cancellation; sampled mock states stay in both boxes", flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
