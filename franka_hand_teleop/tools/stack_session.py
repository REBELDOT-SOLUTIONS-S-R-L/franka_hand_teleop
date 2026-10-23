"""Stop the chosen ROS launch when its host launcher closes the stdin lease."""
import json
import os
import select
import signal
import subprocess
import sys
import time


def main():
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain not in ("132", "133"):
        raise SystemExit("The hand-teleop session requires ROS_DOMAIN_ID=132 or 133")
    launch = "fake.launch.py" if domain == "132" else "real.launch.py"
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    control_node = control = None
    if domain == "133":
        import rclpy
        from franka_hand_teleop.deployment import alignment_text
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        control_node = rclpy.create_node("hand_teleop_session_control")
        control = control_node.create_client(SetBool, "/hand_teleop_real/hand_bridge/arm")
        last_status_key, last_status_time = None, -float("inf")

        def status(message):
            nonlocal last_status_key, last_status_time
            state = json.loads(message.data)
            key = (state.get("armed"), state.get("ready"), state.get("reason"))
            now = time.monotonic()
            if key != last_status_key or now-last_status_time >= 2:
                print(alignment_text(state), flush=True)
                last_status_key, last_status_time = key, now

        control_node.create_subscription(String, "/hand_teleop_real/status", status, 1)
    process = subprocess.Popen([
        "ros2", "launch", "franka_hand_teleop", launch, *sys.argv[1:],
    ], start_new_session=True)
    result = 0
    pending = b""
    try:
        while not stopped:
            if control_node:
                rclpy.spin_once(control_node, timeout_sec=0)
            if process.poll() is not None:
                result = process.returncode
                break
            readable, _, _ = select.select([sys.stdin], [], [], .1)
            if readable:
                data = os.read(sys.stdin.fileno(), 1024)
                if not data:
                    break
                if domain == "133":
                    pending += data
                    while b"\n" in pending:
                        line, pending = pending.split(b"\n", 1)
                        if line in (b"a", b"d") and control.service_is_ready():
                            # One reliable client preserves arm/disarm request order;
                            # disarm is never dropped because an arm reply is pending.
                            future = control.call_async(SetBool.Request(data=line == b"a"))
                            future.add_done_callback(lambda f: print(f"Arm/disarm: {f.result()}", flush=True))
                        else:
                            print("Arm/disarm service not ready; request not queued.", flush=True)
    finally:
        if control_node and control.service_is_ready():
            control.call_async(SetBool.Request(data=False))
            rclpy.spin_once(control_node, timeout_sec=.1)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        if control_node:
            control_node.destroy_node()
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
