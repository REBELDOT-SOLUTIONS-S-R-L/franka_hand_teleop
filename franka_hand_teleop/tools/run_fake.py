"""Run host camera + container fake robot together, and stop both on exit."""
import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.paths import project_root


def main():
    root = project_root(__file__)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("realsense", "demo"), default="realsense")
    parser.add_argument("--tracker-dir", type=Path, default=root.parent / "hand-tracking")
    parser.add_argument("--serial", help="Select a particular RealSense camera")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--headless", action="store_true", help="Camera without a window; needs --frames")
    parser.add_argument("--frames", type=int, default=0, help="Stop everything after N camera frames")
    args = parser.parse_args()
    if args.frames < 0 or (args.headless and not args.frames):
        parser.error("Use a positive --frames value with --headless")
    tracker = args.tracker_dir.expanduser().resolve() / "run.sh"
    if not tracker.is_file():
        parser.error(f"Camera launcher not found: {tracker}")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 49300))
    except OSError:
        parser.error("Fake teleop is already running on port 49300; close that instance first")
    camera_command = ["bash", str(tracker), "--camera", args.camera, "--udp-port", "49300"]
    if args.serial:
        camera_command += ["--serial", args.serial]
    if args.headless:
        camera_command.append("--headless")
    if args.frames:
        camera_command += ["--frames", str(args.frames)]
    stack_command = [
        str(root / "franka"), "exec", "env", "ROS_DOMAIN_ID=132", "python",
        str(Path("/workspace")/Path(__file__).resolve().with_name("stack_session.py").relative_to(root)),
        f"use_rviz:={'false' if args.no_rviz else 'true'}",
    ]
    stack = camera = None
    interrupted = False
    log_path = root / "logs/hand-fake.log"
    log_path.parent.mkdir(exist_ok=True)
    print(f"Starting {args.camera} tracking + fake FR3 + {'no RViz' if args.no_rviz else 'RViz'}.", flush=True)
    print("No enable button: tracking starts automatically once XYZ and calibrated opening are valid.", flush=True)
    print("Fixed camera mapping: B starts/saves a control-range sweep; F restores defaults. "
          "Same camera position always maps to the same robot target.", flush=True)
    print(f"C=closed / O=open calibration in the camera window. Ctrl+C stops everything.\nRobot log: {log_path}", flush=True)
    result = 0
    with log_path.open("w") as log:
        try:
            # EOF on this pipe tells the container supervisor to stop ROS, even
            # when the host process disappears or the Docker CLI is interrupted.
            stack = subprocess.Popen(stack_command, stdin=subprocess.PIPE, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            camera = subprocess.Popen(camera_command, start_new_session=True)
            while True:
                if stack.poll() is not None:
                    result = stack.returncode
                    print(f"Fake robot stopped (exit {result}); closing camera. See {log_path}.", flush=True)
                    break
                if camera.poll() is not None:
                    result = camera.returncode
                    print(f"Camera stopped (exit {result}); closing fake robot.", flush=True)
                    break
                time.sleep(.1)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            # A second Ctrl+C must not interrupt cleanup midway through.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            if camera and camera.poll() is None:
                os.killpg(camera.pid, signal.SIGINT)
                try:
                    camera.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(camera.pid, signal.SIGTERM)
                    camera.wait(timeout=3)
            if stack:
                stack.stdin.close()
                try:
                    stack.wait(timeout=18)
                except subprocess.TimeoutExpired:
                    os.killpg(stack.pid, signal.SIGTERM)
                    stack.wait(timeout=3)
    if interrupted:
        print("Camera and fake robot stopped.", flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
