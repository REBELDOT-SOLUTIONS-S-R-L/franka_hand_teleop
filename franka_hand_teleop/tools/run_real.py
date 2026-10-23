"""One-command real FR3 camera/ROS session, preview unless --execute is explicit."""
import argparse
import json
import math
import os
import select
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.deployment import REAL_DOMAIN, REAL_PORT, endpoint, load_profile
from franka_hand_teleop.paths import project_root


def stack_exit_report(code, log_text):
    """ROS launch may exit zero after a required node crashes; report the cause."""
    startup = log_text.split("sending signal 'SIGINT'", 1)[0]
    if "Connection to FCI refused" in startup:
        return code or 1, ("Robot refused the FCI connection. Check the installed FCI feature and "
                           "FCI activation in Desk before retrying. RViz was closed because the "
                           "required robot connection failed.")
    for line in startup.splitlines():
        if ("[FATAL]" in line or "Caught exception in launch" in line or
                ("[ERROR]" in line and "process has died" in line)):
            return code or 1, f"Real robot session failed: {line.strip()[:500]}"
    if code:
        return code, f"Real robot session exited with code {code}; inspect the robot log."
    return 0, "Robot/RViz session closed; stopping the camera."


def console_updates(text, pending=""):
    """Relay operator feedback while retaining incomplete writes from the log."""
    lines = (pending+text).split("\n")
    messages = []
    for line in lines[:-1]:
        if line.startswith(("Arm/disarm:", "ALIGNMENT:", "TELEOP:")):
            messages.append(line)
        elif ("[FrankaHardwareInterface]" in line or ".fr3_arm_controller]" in line) and (
                "[ERROR]" in line or "[FATAL]" in line):
            messages.append("ROBOT ERROR: "+line.strip())
        elif "TELEOP STOP: " in line:
            messages.append("TELEOP STOP: "+line.split("TELEOP STOP: ", 1)[1])
    return messages, lines[-1]


def camera_updates(text, pending="", *, execute=False, now=None):
    """Accept fresh, explicit UI intent only from our camera child's stdout pipe."""
    lines = (pending+text).split("\n")
    commands, messages = [], []
    now = time.monotonic() if now is None else now
    for line in lines[:-1]:
        if not line.startswith("HAND_TELEOP_OPERATOR "):
            messages.append(line)
            continue
        try:
            packet = json.loads(line[len("HAND_TELEOP_OPERATOR "):])
            stamp = packet.get("monotonic_s")
            if (execute and type(packet.get("version")) is int and packet["version"] == 1 and
                    type(stamp) in (int, float) and math.isfinite(stamp) and
                    packet.get("action") in ("arm", "disarm") and
                    (packet["action"] == "disarm" or -.02 <= now-stamp <= .5)):
                commands.append("a" if packet["action"] == "arm" else "d")
            else:
                messages.append("Camera operator request ignored: unavailable or expired; click again if needed.")
        except (ValueError, TypeError, AttributeError):
            messages.append("Invalid camera operator request ignored.")
    # Bound a malformed unterminated line rather than accumulating indefinitely.
    return commands, messages, lines[-1] if len(lines[-1]) <= 8192 else ""


def main():
    root = project_root(__file__)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, type=endpoint, help="Actual C2 IPv4 address")
    parser.add_argument("--profile", type=Path, default=Path(__file__).resolve().parents[1]/"config/real_profile.yaml")
    parser.add_argument("--execute", action="store_true", help="Allow explicit arming after installation verification")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print commands without starting any process")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--fullscreen", action="store_true", help="Start the camera app fullscreen")
    parser.add_argument("--serial")
    parser.add_argument("--tracker-dir", type=Path, default=root.parent/"hand-tracking")
    args = parser.parse_args()
    try:
        profile = args.profile.expanduser().resolve(strict=True)
        relative = profile.relative_to(root)
        configuration, _ = load_profile(profile, execute=args.execute)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    tracker = args.tracker_dir.expanduser().resolve()/"run.sh"
    if not tracker.is_file():
        parser.error(f"Camera launcher not found: {tracker}")
    camera_command = ["bash", str(tracker), "--camera", "realsense", "--udp-port", str(REAL_PORT)]
    if args.serial:
        camera_command += ["--serial", args.serial]
    if args.fullscreen:
        camera_command += ["--fullscreen"]
    if args.execute:
        camera_command += ["--operator-controls"]
    stack_command = [str(root/"franka"), "exec", "env", f"ROS_DOMAIN_ID={REAL_DOMAIN}", "python",
                     str(Path("/workspace")/Path(__file__).resolve().with_name("stack_session.py").relative_to(root)),
                     f"robot_ip:={args.robot_ip}", f"profile:=/workspace/{relative.as_posix()}",
                     f"execute:={str(args.execute).lower()}", f"use_rviz:={str(not args.no_rviz).lower()}"]
    if args.dry_run:
        print(json.dumps({"mode": "execute/disarmed" if args.execute else "preview",
                          "camera": camera_command, "stack": stack_command}, indent=2))
        return 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("127.0.0.1", REAL_PORT))
        except OSError:
            parser.error("A real teleop session already owns the observation port")
    terminal_controls = args.execute and sys.stdin.isatty()
    print("Connecting to REAL hardware. " + ("DISARMED; use ARM / DISARM in the camera window."
          if args.execute else "PREVIEW; no arm/gripper motion output."), flush=True)
    if terminal_controls:
        print("Terminal fallback: a + Enter arms, d + Enter disarms.", flush=True)
    if args.execute:
        print("Gripper: " + ("enabled after arming; calibrated thumb/pinky opening controls width (5 mm/s)."
                            if configuration["enable_gripper"] else "disabled by profile."), flush=True)
    print("The commissioned robot-side safety configuration must independently enforce stopping/protection.", flush=True)
    print("Ctrl+C or closing the camera/RViz stops the session.", flush=True)
    stack = camera = None
    log_path = root/"logs/hand-real.log"
    log_path.parent.mkdir(exist_ok=True)
    print(f"Robot log: {log_path}", flush=True)
    result = 0
    with log_path.open("w") as log, log_path.open(errors="replace") as console:
        pending = camera_pending = ""

        def show_updates():
            nonlocal pending
            messages, pending = console_updates(console.read(), pending)
            for message in messages:
                print(message, flush=True)

        def request(key):
            stack.stdin.write((key+"\n").encode())
            stack.stdin.flush()
            print("Arm requested; waiting for response." if key == "a" else "Disarm requested.", flush=True)

        try:
            stack = subprocess.Popen(stack_command, stdin=subprocess.PIPE, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            # Dedicated ordered pipe: the camera has no robot/ROS command client.
            # Keep the separate host->supervisor stdin lease alive for GUI-only use.
            camera = subprocess.Popen(camera_command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                      start_new_session=True)
            while True:
                show_updates()
                if stack.poll() is not None:
                    result, message = stack_exit_report(stack.returncode, log_path.read_text(errors="replace"))
                    print(message, flush=True)
                    break
                if camera.poll() is not None:
                    result = camera.returncode
                    print(f"Camera exited with code {result}; stopping the robot session.", flush=True)
                    break
                inputs = [camera.stdout] + ([sys.stdin] if terminal_controls else [])
                readable, _, _ = select.select(inputs, [], [], .1)
                if camera.stdout in readable:
                    data = os.read(camera.stdout.fileno(), 65536)
                    if not data:
                        print("Camera control pipe closed; stopping the session.", flush=True)
                        break
                    commands, messages, camera_pending = camera_updates(
                        data.decode(errors="replace"), camera_pending, execute=args.execute)
                    for message in messages:
                        print(message, flush=True)
                    for key in commands:
                        request(key)
                if terminal_controls and sys.stdin in readable:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    key = line.strip().lower()
                    if key in ("a", "d"):
                        request(key)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            if camera and camera.poll() is None:
                os.killpg(camera.pid, signal.SIGINT)
                try:
                    camera.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(camera.pid, signal.SIGTERM)
                    camera.wait(timeout=3)
            if camera and camera.stdout:
                camera.stdout.close()
            if stack:
                stack.stdin.close()
                try:
                    stack.wait(timeout=18)
                except subprocess.TimeoutExpired:
                    os.killpg(stack.pid, signal.SIGTERM)
                    stack.wait(timeout=3)
            show_updates()
    print("Real teleop session stopped.", flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
