"""Inspect launch wiring with Node construction mocked: no process or hardware I/O."""
import importlib.util
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml
from launch import LaunchContext

spec = importlib.util.spec_from_file_location("motion_launch", Path(__file__).parents[1]/"launch/real.launch.py")
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
PROFILE = Path(__file__).parent/"fixtures/rolling_profile.yaml"


class MotionLaunchTests(unittest.TestCase):
    def context(self, path):
        context = LaunchContext()
        context.launch_configurations.update(profile=str(path), robot_ip="10.42.0.55",
                                             execute="false", use_rviz="false")
        return context

    def test_all_motion_components_share_one_immutable_three_mps_snapshot(self):
        profile = yaml.safe_load(PROFILE.read_text())
        profile["motion"]["max_speed"] = 3.
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"profile.yaml"
            source.write_text(yaml.safe_dump(profile))
            with patch.dict(os.environ, ROS_DOMAIN_ID="133"), \
                 patch.object(launch, "model_for", return_value="<robot/>"), \
                 patch.object(launch, "Node", side_effect=lambda **kwargs: kwargs):
                actions = launch.launch_real(self.context(source))
            nodes = [a for a in actions if isinstance(a, dict)]
            bridge = next(n for n in nodes if n["executable"] == "smooth_bridge")
            servo = next(n for n in nodes if n["executable"] == "feedback_servo")
            snapshot = Path(bridge["parameters"][-1]["profile"])
            try:
                self.assertEqual(servo["parameters"][-1]["motion_profile"], str(snapshot))
                manager = next(n for n in nodes if n["executable"] == "ros2_control_node")
                controller_file = Path(manager["parameters"][1])
                controllers = yaml.safe_load(controller_file.read_text())
                self.assertEqual(controllers["/**/fr3_arm_controller"]["ros__parameters"]["motion_profile"],
                                 str(snapshot))
                spawner = next(n for n in nodes if n.get("arguments", [None])[0] == "fr3_arm_controller")
                self.assertIn(str(controller_file), spawner["arguments"])
                profile["motion"]["max_speed"] = .5
                source.write_text(yaml.safe_dump(profile))
                self.assertEqual(json.loads(snapshot.read_text())["motion"]["max_speed"], 3.)
            finally:
                shutil.rmtree(snapshot.parent)

    def test_excessive_profile_is_rejected_before_model_or_nodes_are_created(self):
        profile = yaml.safe_load(PROFILE.read_text())
        profile["motion"]["max_speed"] = 3.001
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"profile.yaml"
            source.write_text(yaml.safe_dump(profile))
            with patch.dict(os.environ, ROS_DOMAIN_ID="133"), \
                 patch.object(launch, "model_for") as model, patch.object(launch, "Node") as node:
                with self.assertRaisesRegex(ValueError, "FR3 envelope"):
                    launch.launch_real(self.context(source))
                model.assert_not_called()
                node.assert_not_called()


if __name__ == "__main__":
    unittest.main()
