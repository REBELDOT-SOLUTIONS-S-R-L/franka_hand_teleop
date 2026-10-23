"""Regression for the FCI failure that closed RViz before its window appeared."""
import contextlib
import importlib.util
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("run_real", Path(__file__).resolve().parents[1]/"tools/run_real.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_gui_only_launcher_relays_clicks_and_keeps_supervisor_lease(self):
        original_popen = subprocess.Popen
        original_profile = Path(launcher.__file__).resolve().parents[1]/"config/real_profile.yaml"
        commands = []
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/"franka").touch()
            (root/"compose.yaml").touch()
            source = root/"ws/src/franka_hand_teleop/franka_hand_teleop/tools/run_real.py"
            profile = source.parent.parent/"config/real_profile.yaml"
            profile.parent.mkdir(parents=True)
            profile.write_text(original_profile.read_text())
            tracker = root/"tracker"
            tracker.mkdir()
            (tracker/"run.sh").touch()
            received = root/"requests.txt"
            stack_stub = ("import sys; from pathlib import Path; "
                          f"p=Path({str(received)!r}); "
                          "p.write_text(sys.stdin.read()+'lease closed')")
            camera_stub = ("import json,time; time.sleep(.15); "
                           "print('Camera: local UI stub',flush=True); "
                           "print('HAND_TELEOP_OPERATOR '+json.dumps(dict(version=1,action='arm',"
                           "monotonic_s=time.monotonic())),flush=True); "
                           "print('HAND_TELEOP_OPERATOR '+json.dumps(dict(version=1,action='disarm',"
                           "monotonic_s=time.monotonic())),flush=True); time.sleep(.2)")

            def stub(command, **kwargs):
                commands.append(command)
                code = camera_stub if "--camera" in command else stack_stub
                return original_popen([sys.executable, "-c", code], **kwargs)

            # No camera, ROS launch, physical driver or production observation port.
            with patch.object(launcher, "__file__", str(source)), \
                 patch.object(launcher.subprocess, "Popen", side_effect=stub), \
                 patch.object(launcher, "REAL_PORT", 0), \
                 patch.object(launcher.signal, "signal"), \
                 patch.object(launcher.sys, "stdin", io.StringIO()), \
                 patch.object(launcher.sys, "argv", ["run_real.py", "--robot-ip", "10.42.0.55",
                              "--execute", "--fullscreen", "--no-rviz", "--tracker-dir", str(tracker)]), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(launcher.main(), 0)
            self.assertEqual(received.read_text(), "a\nd\nlease closed")
            self.assertIn("--operator-controls", commands[1])
            self.assertIn("--fullscreen", commands[1])

    def test_camera_intent_uses_ordered_pipe_and_rejects_stale_or_invalid_arm(self):
        def event(action, stamp=10., **changes):
            return "HAND_TELEOP_OPERATOR " + json.dumps(
                {"version": 1, "action": action, "monotonic_s": stamp} | changes) + "\n"
        commands, messages, pending = launcher.camera_updates(
            "Camera: RealSense\n"+event("arm")+event("disarm"), execute=True, now=10.1)
        self.assertEqual(commands, ["a", "d"])
        self.assertEqual(messages, ["Camera: RealSense"])
        self.assertEqual(pending, "")
        self.assertEqual(launcher.camera_updates(event("arm"), execute=False, now=10.1)[0], [])
        for message in [event("arm", 9.), event("arm", 11.), event("arm", float("nan")),
                        event("home"), event("arm", version=True), "HAND_TELEOP_OPERATOR []\n"]:
            self.assertEqual(launcher.camera_updates(message, execute=True, now=10.1)[0], [])
        # A delayed disarm is still delivered; it can only reduce authority.
        self.assertEqual(launcher.camera_updates(event("disarm", 1.), execute=True, now=10.1)[0], ["d"])
        line = event("arm")
        first, _, pending = launcher.camera_updates(line[:25], execute=True, now=10.1)
        second, _, pending = launcher.camera_updates(line[25:], pending, execute=True, now=10.1)
        self.assertEqual(first, [])
        self.assertEqual(second, ["a"])
        self.assertEqual(pending, "")

    def test_operator_feedback_survives_partial_log_writes(self):
        messages, pending = launcher.console_updates("[INFO] ROS startup\nArm/dis")
        self.assertEqual(messages, [])
        messages, pending = launcher.console_updates(
            "arm: refused: align target\nALIGNMENT: gap=5.0 cm\n[INFO] noise\nTELEOP: ARM", pending)
        self.assertEqual(messages, ["Arm/disarm: refused: align target", "ALIGNMENT: gap=5.0 cm"])
        messages, pending = launcher.console_updates("ED\n", pending)
        self.assertEqual(messages, ["TELEOP: ARMED"])
        self.assertEqual(pending, "")
        self.assertEqual(launcher.console_updates("", pending), ([], ""))

    def test_first_stop_reason_and_communication_telemetry_are_relayed(self):
        message = "Tracking expired; last FCI command success=0.990; feedback age=0.010s"
        messages, _ = launcher.console_updates("[real_bridge-7] [WARN] TELEOP STOP: "+message+"\n")
        self.assertEqual(messages, ["TELEOP STOP: "+message])

    def test_robot_reflex_is_reported_without_waiting_for_session_exit(self):
        fault = ('[ros2_control_node-2] [ERROR] [123] [FrankaHardwareInterface]: '
                 'motion aborted by reflex! ["joint_position_motion_generator_start_pose_invalid"]')
        messages, pending = launcher.console_updates(fault+"\n")
        self.assertEqual(messages, ["ROBOT ERROR: "+fault])
        self.assertEqual(pending, "")

    def test_controller_clock_rejection_is_reported(self):
        fault = "[ros2_control_node-2] [ERROR] [hand_teleop_real.fr3_arm_controller]: FR3 robot clock stale"
        self.assertEqual(launcher.console_updates(fault+"\n")[0], ["ROBOT ERROR: "+fault])

    def test_fci_refusal_is_reported_even_if_launch_returns_zero(self):
        log = """[INFO] [rviz2-8]: process started
[franka_gripper_node-5] [FATAL] libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.
[INFO] [rviz2-8]: sending signal 'SIGINT' to process[rviz2-8]
[rviz2-8] [FATAL] context is invalid
"""
        code, message = launcher.stack_exit_report(0, log)
        self.assertEqual(code, 1)
        self.assertIn("FCI", message)
        self.assertIn("Desk", message)
        self.assertNotIn("context is invalid", message)

    def test_launch_configuration_error_is_exposed(self):
        code, message = launcher.stack_exit_report(1, "[ERROR] [launch]: Caught exception in launch: bad profile")
        self.assertEqual(code, 1)
        self.assertIn("bad profile", message)

    def test_normal_close_ignores_secondary_shutdown_errors(self):
        log = "[INFO] sending signal 'SIGINT'\n[ERROR] [servo]: process has died: shutdown"
        code, message = launcher.stack_exit_report(0, log)
        self.assertEqual(code, 0)
        self.assertIn("closed", message)

    def test_unknown_nonzero_exit_is_not_reported_as_success(self):
        code, message = launcher.stack_exit_report(2, "")
        self.assertEqual(code, 2)
        self.assertIn("code 2", message)


if __name__ == "__main__":
    unittest.main()
