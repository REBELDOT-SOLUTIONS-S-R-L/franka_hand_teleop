"""Regression tests for absolute FOV mapping, loss/re-entry, and boundary braking."""
import itertools
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.core import Limits, Teleop, decode


def tracking(t, control=(.5, .5, .5), tracked=True, session="camera", palm=(0, 0, .65)):
    return {"version": 2, "kind": "tracking", "monotonic_s": t, "source": "demo",
            "frame": "camera_color_optical_frame", "units": "m", "tracked": tracked,
            "session": session, "sequence": int(t*1000), "palm_m": palm, "opening": .5,
            "control_uvd": control}


class ProtocolTests(unittest.TestCase):
    def test_age_version_units_and_finite_values(self):
        self.assertIsNotNone(decode(json.dumps(tracking(10)), 10.1))
        self.assertIsNone(decode(json.dumps(tracking(10)), 10.3))
        self.assertIsNone(decode(json.dumps(tracking(11)), 10))
        for changes in ({"version": 1}, {"units": "mm"}, {"control_uvd": None},
                        {"control_uvd": [0, float("nan"), 1]}, {"opening": True},
                        {"tracked": 1}, {"source": "webcam"}, {"kind": "enable"}):
            packet = tracking(10)
            packet.update(changes)
            self.assertIsNone(decode(json.dumps(packet), 10))
        for data in (b"null", b"[]", b"garbage", b"\xff"):
            self.assertIsNone(decode(data, 10))


class MappingTests(unittest.TestCase):
    position = (.55, 0, .15)

    def active(self, control=(.5, .5, .5), start=10, logic=None):
        logic = logic or Teleop()
        for offset in (0, .1, .2, .32):
            at = start+offset
            logic.receive(tracking(at, control))
            logic.step(at, .02, self.position)
        self.assertTrue(logic.active)
        return logic

    def test_fov_endpoints_span_usable_workspace(self):
        limits = Limits()
        for value, expected in zip(limits.target((.5, .5, .5)), self.position):
            self.assertAlmostEqual(value, expected)
        for control in itertools.product((0, 1), repeat=3):
            target = limits.target(control)
            fractions = (1-control[2], control[0], 1-control[1])
            expected = tuple(a+.02+f*(b-a-.04) for f, a, b in
                             zip(fractions, limits.minimum, limits.maximum))
            for value, wanted in zip(target, expected):
                self.assertAlmostEqual(value, wanted)

    def test_horizontal_direction_is_reversed_without_changing_depth_or_height(self):
        limits = Limits()
        left = limits.target((.4, .3, .7))
        right = limits.target((.6, .3, .7))
        self.assertLess(left[1], right[1])
        self.assertEqual(left[0], right[0])
        self.assertEqual(left[2], right[2])
        self.assertLess(limits.target((.5, .5, .6))[0], self.position[0])
        self.assertGreater(limits.target((.5, .4, .5))[2], self.position[2])

    def test_live_tcp_inverse_round_trip_preserves_axis_directions_and_inset(self):
        limits = Limits()
        for control in itertools.product((0., .17, .5, .83, 1.), repeat=3):
            back = limits.camera_control(limits.target(control))
            for actual, expected in zip(back, control):
                self.assertAlmostEqual(actual, expected, places=12)
        self.assertLess(limits.camera_control((.7, 0, .15))[2], 0)
        self.assertGreater(limits.camera_control((.55, .4, .15))[0], 1)
        with self.assertRaises(ValueError):
            limits.camera_control((float("nan"), 0., .1))

    def test_loss_reentry_and_robot_pose_cannot_shift_mapping(self):
        logic = self.active(control=(.5, .5, .5))
        center_target = logic.target
        logic.receive(tracking(10.4, tracked=False))
        self.assertFalse(logic.active)
        self.active(control=(.5, 1, .5), start=10.5, logic=logic)
        self.assertAlmostEqual(logic.target[2], .07)  # bottom always means bottom
        self.assertNotEqual(logic.target, center_target)
        logic.receive(tracking(10.9, (.5, 0, .5)))
        logic.step(10.9, .02, (.5, .1, .2))
        self.assertAlmostEqual(logic.target[2], .23)  # top always means top
        logic.receive(tracking(11, (.5, .5, .5)))
        logic.step(11, .02, (.5, .1, .2))
        self.assertEqual(logic.target, center_target)

    def test_outside_camera_region_saturates_with_clearance(self):
        limits = Limits()
        for control in itertools.product((-10, 10), repeat=3):
            target = limits.target(control)
            self.assertTrue(all(a+.0199 <= p <= b-.0199 for a, p, b in
                                zip(limits.minimum, target, limits.maximum)))

    def test_feedback_watchdog_and_speed_limits(self):
        logic = self.active()
        logic.receive(tracking(10.4, (0, 0, 0)))
        velocity = logic.step(10.4, .02, self.position)
        self.assertLessEqual(sum(v*v for v in velocity)**.5, .3*.02 + 1e-10)
        self.assertEqual(logic.step(10.8, .02, self.position), (0, 0, 0))
        self.assertFalse(logic.active)
        self.assertEqual(logic.step(10.9, .02, (.651, 0, .15)), (0, 0, 0))

    def test_outward_velocity_is_removed_before_boundary(self):
        logic = self.active()
        logic.receive(tracking(10.4, (.5, .5, 0)))
        logic.velocity = (.08, 0, 0)
        velocity = logic.step(10.4, .02, (.649, 0, .15))
        self.assertLessEqual(velocity[0], 0)

    def test_jump_gap_and_restart_stop_without_changing_map(self):
        for packet in (tracking(10.4, palm=(1, 0, .65)),
                       tracking(10.4, session="new"), tracking(10.7)):
            logic = self.active()
            logic.receive(packet)
            self.assertFalse(logic.active)
            self.assertIsNone(logic.valid_since)
        for kwargs in ({"speed": float("nan")}, {"margin": 0}, {"margin": .3}):
            with self.assertRaises(ValueError):
                Limits(**kwargs)


if __name__ == "__main__":
    unittest.main()
