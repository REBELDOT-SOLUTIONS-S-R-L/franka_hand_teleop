"""Check FCI loop scheduling on an EMPTY controller manager; never load hardware."""
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "137":
        raise SystemExit("Empty-manager timing check requires ROS_DOMAIN_ID=137")
    share = Path(get_package_share_directory("franka_hand_teleop"))
    log_path = Path("/workspace/logs/real-timing-empty-manager-check.log")
    rclpy.init(args=[])
    node = rclpy.create_node("empty_manager_timing_check")
    namespace = "/hand_teleop_timing_check/controller_manager/"
    parameters = node.create_client(GetParameters, namespace+"get_parameters")

    def wait(predicate):
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise RuntimeError(f"Empty-manager timing check timed out; see {log_path}")

    with log_path.open("w") as log:
        # No robot_description publisher or parameter: there is no model/plugin
        # to instantiate and no robot address, command controller or action goal.
        process = subprocess.Popen(["ros2", "run", "controller_manager", "ros2_control_node",
            "--ros-args", "-r", "__ns:=/hand_teleop_timing_check", "--params-file",
            str(share/"config/real_controllers.yaml")], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            # Hardware services are deliberately absent until a description is
            # received. Only the node's ordinary parameter service is needed.
            wait(parameters.service_is_ready)
            future = parameters.call_async(GetParameters.Request(names=[
                "hardware_synchronization.expect_blocking_read_write",
                "hardware_synchronization.minimum_cycle_time", "lock_memory", "update_rate"]))
            wait(future.done)
            values = future.result().values
            assert values[0].bool_value and values[1].double_value == .0001
            assert values[2].bool_value and values[3].integer_value == 1000
            assert "Synchronizing control loop with hardware" in log_path.read_text()
            assert "Received robot description" not in log_path.read_text()
            print("PASS: installed controller manager uses blocking-hardware synchronization, "
                  "memory locking and 1 kHz configuration; NO hardware was loaded", flush=True)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
