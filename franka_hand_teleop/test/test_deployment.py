import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.deployment import (
    MotionPermit,
    alignment_text,
    bounded_increment,
    endpoint,
    load_profile,
    segment_duration,
    tracking_fresh,
)
from franka_hand_teleop.rolling import Limits as RollingLimits

ACTIVE_PROFILE = Path(__file__).resolve().parents[1]/"config/real_profile.yaml"
PROFILE = Path(__file__).parent/"fixtures/rolling_profile.yaml"


class DeploymentTests(unittest.TestCase):
    def test_gripper_speed_is_doubled_without_changing_force_or_lift_settings(self):
        profile, _ = load_profile(PROFILE)
        self.assertEqual(profile["gripper_speed_m_s"], .040)
        self.assertEqual(profile["grasp_force_N"], 30.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            for value in (0., -.01, .2001, True, math.nan):
                changed = copy.deepcopy(profile)
                changed["gripper_speed_m_s"] = value
                path.write_text(yaml.safe_dump(changed))
                with self.assertRaisesRegex(ValueError, "Gripper speed"):
                    load_profile(path)

    def test_grasp_force_stays_within_documented_original_hand_range(self):
        profile = yaml.safe_load(PROFILE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            for value in (0., 5., 29.9, 70.1, True, math.nan):
                profile["grasp_force_N"] = value
                path.write_text(yaml.safe_dump(profile))
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_seeded_increment_preserves_direction_with_measured_desired_bias(self):
        motion = load_profile(PROFILE)[0]["motion"]
        seed = [.1]*7
        proposed = [q+.00008 for q in seed]
        for bias in (-.00016, 0., .00008, .00016):
            desired = [q+bias for q in seed]
            endpoint, total, delta = bounded_increment(seed, proposed, desired, seed, .20, motion, 2.)
            for a, b, d in zip(desired, endpoint, delta):
                self.assertGreater(b-a, 0.)
                self.assertAlmostEqual(d, .00008)
            self.assertGreaterEqual(total, .20)

    def test_only_executed_increment_is_bounded_and_excessive_discrepancy_refused(self):
        motion = load_profile(PROFILE)[0]["motion"]
        seed, desired = [0.]*7, [.0001]*7
        target, total, delta = bounded_increment(seed, [.01]*7, desired, seed, .20, motion, 2.)
        self.assertLessEqual(2*sum(map(abs, delta)), motion["max_tool_step"]+1e-12)
        self.assertAlmostEqual(2*sum(map(abs, delta)), .100)
        peak, length = max(map(abs, delta)), 2*sum(map(abs, delta))
        self.assertLessEqual(1.875*length/total, motion["max_speed"]+1e-12)
        self.assertLessEqual(((10/math.sqrt(3))*length+1.875**2*length*sum(map(abs, delta)))/total**2,
                             motion["max_acceleration"]+1e-12)
        self.assertLessEqual(60*peak/total**3, motion["joint_jerk"]+1e-12)
        self.assertGreaterEqual(total, .20)
        self.assertLessEqual(total, 1.5)
        self.assertEqual(target, [a+b for a, b in zip(desired, delta)])
        for values in ((seed, [math.nan]*7, desired, seed, .2),
                       (seed, [.01]*7, [.0021]*7, seed, .2),
                       ([.003]*7, [.01]*7, desired, seed, .2),
                       (seed, [.01]*7, desired, seed, .01)):
            with self.assertRaises(ValueError):
                bounded_increment(*values, motion, 2.)

    def test_step_limits_are_profile_values_and_invalid_values_are_refused(self):
        profile = yaml.safe_load(PROFILE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            profile["motion"]["max_tool_step"] = .2
            profile["motion"]["max_joint_step"] = math.radians(30)
            path.write_text(yaml.safe_dump(profile))
            loaded, _ = load_profile(path, execute=True)
            self.assertEqual(loaded["motion"]["max_tool_step"], .2)
            for value in (0., -1., math.nan, True):
                profile["motion"]["max_tool_step"] = value
                path.write_text(yaml.safe_dump(profile))
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_three_metres_per_second_and_manufacturer_envelope_need_only_profile_edits(self):
        profile = yaml.safe_load(PROFILE.read_text())
        profile["motion"].update(max_speed=3., max_acceleration=9., joint_speed=2.62,
            joint_acceleration=10., joint_jerk=5000., tracking_gain=40.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            path.write_text(yaml.safe_dump(profile))
            loaded, limits = load_profile(path, execute=True)
            self.assertEqual(limits.speed, 3.)
            self.assertEqual(limits.gain, 40.)
            planner = RollingLimits.from_motion(loaded["motion"], 1.722834753163608)
            self.assertEqual(planner.tool_speed, 3.)
            self.assertEqual(planner.joint_acceleration, 10.)
            for key, value in (("max_speed", 3.0001), ("max_acceleration", 9.0001)):
                excessive = copy.deepcopy(profile)
                excessive["motion"][key] = value
                path.write_text(yaml.safe_dump(excessive))
                with self.assertRaisesRegex(ValueError, "FR3 envelope"):
                    load_profile(path)

    def test_larger_joint_increment_keeps_all_derivative_and_travel_bounds(self):
        motion = load_profile(PROFILE)[0]["motion"]
        # Reach bounds >=1 m are supported. At the installed 1.723 m reach,
        # 100 mm travel limits every joint below 0.059 rad, even with a 25 degree ceiling.
        for reach in (1., 1.722834753163608, 2.5):
            for sign in (-1., 1.):
                proposed = [sign*.04]+[0.]*6
                target, total, delta = bounded_increment([0.]*7, proposed,
                    [0.]*7, [0.]*7, .20, motion, reach)
                peak, summed = max(map(abs, delta)), sum(map(abs, delta))
                length = reach*summed
                self.assertGreater(peak, .01)
                self.assertLessEqual(peak, motion["max_joint_step"])
                self.assertLessEqual(length, .10+1e-12)
                self.assertLessEqual(total, 1.5)
                self.assertGreater(delta[0]*sign, 0.)
                self.assertEqual(target, delta)
                for tick in range(1001):
                    x = tick/1000
                    d1 = 30*x*x*(1-x)**2/total
                    d2 = (60*x-180*x*x+120*x**3)/total**2
                    d3 = (60-360*x+360*x*x)/total**3
                    self.assertLessEqual(peak*abs(d1), motion["joint_speed"]+1e-9)
                    self.assertLessEqual(peak*abs(d2), motion["joint_acceleration"]+1e-9)
                    self.assertLessEqual(peak*abs(d3), motion["joint_jerk"]+1e-9)
                    self.assertLessEqual(length*abs(d1), motion["max_speed"]+1e-9)
                    self.assertLessEqual(length*(abs(d2)+d1*d1*summed),
                                         motion["max_acceleration"]+1e-9)

    def test_25_degree_joint_ceiling_does_not_change_installed_travel_clipping(self):
        motion = load_profile(PROFILE)[0]["motion"]
        previous = dict(motion, max_joint_step=math.radians(10))
        reach = 1.722834753163608
        proposals = [[sign*.04 if i == axis else 0. for i in range(7)]
                     for sign in (-1., 1.) for axis in range(7)]
        proposals.append([.12, -.08, .01, -.15, .03, .04, -.02])
        for proposed in proposals:
            old = bounded_increment([0.]*7, proposed, [0.]*7, [0.]*7, .2, previous, reach)
            new = bounded_increment([0.]*7, proposed, [0.]*7, [0.]*7, .2, motion, reach)
            self.assertEqual(old, new)
            self.assertLessEqual(max(map(abs, new[2])), .1/reach+1e-12)

    def test_manufacturer_joint_envelope_rejects_excessive_or_nonfinite_values(self):
        original = yaml.safe_load(PROFILE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            for key, value in (("joint_speed", 2.6201), ("joint_acceleration", 10.0001),
                               ("joint_jerk", 5000.0001)):
                profile = copy.deepcopy(original)
                profile["motion"][key] = value
                path.write_text(yaml.safe_dump(profile))
                with self.assertRaisesRegex(ValueError, "FR3 envelope"):
                    load_profile(path)

    def test_active_profile_is_the_planner_source_without_implicit_defaults(self):
        motion = load_profile(ACTIVE_PROFILE)[0]["motion"]
        limits = RollingLimits.from_motion(motion, 1.722834753163608)
        for key, attribute in (("joint_speed", "joint_speed"),
                ("joint_acceleration", "joint_acceleration"), ("joint_jerk", "joint_jerk"),
                ("max_speed", "tool_speed"), ("max_acceleration", "tool_acceleration"),
                ("max_tool_step", "tool_travel"), ("max_joint_step", "joint_step"),
                ("max_curve_duration_s", "maximum_horizon")):
            self.assertEqual(getattr(limits, attribute), motion[key])
        with self.assertRaises(TypeError):
            RollingLimits()

    def test_bridge_timing_requires_overlap_and_planning_reserve(self):
        original = yaml.safe_load(PROFILE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            profile = copy.deepcopy(original)
            profile["bridge"].update(handoff_lead_s=.10, min_publish_lead_s=.055,
                                     planning_budget_s=.02, min_curve_duration_s=.14)
            path.write_text(yaml.safe_dump(profile))
            self.assertEqual(load_profile(path)[0]["bridge"]["min_curve_duration_s"], .14)
            for section, key, value in (("bridge", "min_curve_duration_s", .15),
                    ("bridge", "handoff_lead_s", .31), ("bridge", "min_publish_lead_s", .02),
                    ("bridge", "planning_budget_s", .1), ("bridge", "proposal_horizon_s", .501),
                    ("motion", "max_curve_duration_s", 1.501)):
                profile = copy.deepcopy(original)
                profile[section][key] = value
                path.write_text(yaml.safe_dump(profile))
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_alignment_feedback_uses_robot_coordinates_and_centimetres(self):
        state = {"tcp_m": [.55, 0, .15], "target_m": [.60, 0, .15],
                 "ready": True, "arm_capture_distance_m": .03}
        message = alignment_text(state)
        self.assertIn("gap=5.0 cm (need <= 3.0 cm)", message)
        self.assertIn("TCP=(55.0, 0.0, 15.0) cm", message)
        self.assertIn("TARGET=(60.0, 0.0, 15.0) cm", message)
        self.assertIn("ready=yes", message)
        self.assertIn("DISARMED; explicit arm required", message)
        self.assertIn("completed steps=12", alignment_text(dict(state, armed=True, arm_actions_completed=12)))
        self.assertIn("TCP=(55.00, 0.00, 15.00) cm", alignment_text(dict(state, armed=True)))
        state.update(ready=False, reason="Robot feedback stale")
        self.assertIn("ready=no | Robot feedback stale", alignment_text(state))
        state["target_m"] = None
        self.assertIn("waiting for actual TCP and valid hand target", alignment_text(state))
        self.assertIn("ARMED", alignment_text({"armed": True}))
        self.assertIn("FCI command success=99.5%", alignment_text({
            "armed": True, "control_command_success_rate": .995}))

    def test_uncommissioned_profile_allows_preview_but_blocks_motion(self):
        profile = yaml.safe_load(PROFILE.read_text())
        profile["commissioning"] = {key: False for key in profile["commissioning"]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"unverified-profile.yaml"
            path.write_text(yaml.safe_dump(profile))
            load_profile(path)
            with self.assertRaisesRegex(ValueError, "Motion blocked"):
                load_profile(path, execute=True)

    def test_checks_require_booleans_and_commissioning_limits_cannot_be_raised(self):
        original = yaml.safe_load(PROFILE.read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.yaml"
            for section, key, value in (("commissioning", "robot_safety_verified", "true"),
                                        ("motion", "joint_jerk", 5001.),
                                        ("motion", "tracking_gain", -1.),
                                        ("motion", "max_speed", 3.001),
                                        ("motion", "workspace_min", [float("nan"), 0, 0])):
                profile = copy.deepcopy(original)
                profile[section][key] = value
                path.write_text(yaml.safe_dump(profile))
                with self.assertRaises(ValueError):
                    load_profile(path)

    def test_explicit_endpoint(self):
        self.assertEqual(endpoint("192.0.2.1"), "192.0.2.1")
        for value in ("", "dont-care", "0.0.0.0", "127.0.0.1", "224.0.0.1"):
            with self.assertRaises(ValueError):
                endpoint(value)

    def test_loss_never_automatically_rearms_and_nearby_capture_required(self):
        permit = MotionPermit()
        permit.observe(10, True, True)
        self.assertFalse(permit.arm(10.1, (.55, 0, .15), (.55, 0, .15), True, .03)[0])
        self.assertFalse(permit.arm(10.4, (.55, 0, .15), (.65, 0, .15), True, .03)[0])
        self.assertFalse(permit.arm(10.4, (.55, 0, .15), (.55, 0, .15), False, .03)[0])
        self.assertTrue(permit.arm(10.4, (.55, 0, .15), (.55, 0, .15), True, .03)[0])
        generation = permit.generation
        permit.observe(10.5, False, True)
        self.assertFalse(permit.armed)
        self.assertGreater(permit.generation, generation)
        permit.observe(10.6, True, True)
        permit.observe(11, True, True)
        self.assertFalse(permit.armed)
        self.assertTrue(permit.arm(11, (.55, 0, .15), (.55, 0, .15), True, .03)[0])
        permit.observe(11.1, True, False)
        self.assertFalse(permit.armed)

    def test_only_fresh_metric_camera_tracking_can_arm(self):
        packet = {"tracked": True, "source": "realsense", "monotonic_s": 10}
        self.assertTrue(tracking_fresh(packet, 10.1))
        self.assertFalse(tracking_fresh(packet, 10.3))
        self.assertFalse(tracking_fresh(dict(packet, source="demo"), 10.1))
        self.assertFalse(tracking_fresh(dict(packet, tracked=False), 10.1))

    def test_quintic_derivatives_obey_joint_and_cartesian_limits(self):
        motion = load_profile(PROFILE)[0]["motion"]
        reach = 2.5
        delta = [.00015, -.0002, .00008, -.0003, .00005, .0002, -.0001]
        total = segment_duration(delta, motion, reach)
        length = reach*sum(map(abs, delta))
        for i in range(1001):
            s = i/1000
            d1 = 30*s*s*(1-s)**2/total
            d2 = (60*s-180*s*s+120*s**3)/total**2
            d3 = (60-360*s+360*s*s)/total**3
            for distance in delta:
                self.assertLessEqual(abs(distance*d1), motion["joint_speed"]+1e-9)
                self.assertLessEqual(abs(distance*d2), motion["joint_acceleration"]+1e-9)
                self.assertLessEqual(abs(distance*d3), motion["joint_jerk"]+1e-9)
            self.assertLessEqual(length*abs(d1), motion["max_speed"]+1e-9)
            acceleration = length*(abs(d2)+d1*d1*sum(map(abs, delta)))
            self.assertLessEqual(acceleration, motion["max_acceleration"]+1e-9)
        for value in (math.radians(25)+1e-6, math.nan):
            with self.assertRaises(ValueError):
                segment_duration([value]*7, motion, reach)


if __name__ == "__main__":
    unittest.main()
