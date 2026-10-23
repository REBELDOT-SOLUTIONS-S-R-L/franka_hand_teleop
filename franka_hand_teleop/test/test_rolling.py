import math
import random
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.deployment import load_profile
from franka_hand_teleop.grasp import GraspGuard
from franka_hand_teleop.rolling import (
    Limits,
    Quintic,
    State,
    bernstein,
    checked_curve,
    curve_inside,
    derivative,
    evaluate,
    range_bound,
    split,
    tracking_endpoint,
)


def numeric_limits():
    motion = load_profile(Path(__file__).parent/"fixtures/rolling_profile.yaml")[0]["motion"]
    return Limits.from_motion(motion, 1.722834753163608)


class RollingTests(unittest.TestCase):
    def test_batched_hulls_match_original_subdivision_for_mixed_motion(self):
        randomizer = random.Random(73)
        for degree in range(2, 6):
            for _ in range(30):
                coefficients = tuple(randomizer.uniform(-3., 3.) for _ in range(degree+1))
                pieces = [bernstein(coefficients)]
                for _ in range(5):
                    pieces = [part for piece in pieces for part in split(piece)]
                reference = min(min(p) for p in pieces), max(max(p) for p in pieces)
                for old, fast in zip(reference, range_bound(coefficients)):
                    self.assertAlmostEqual(old, fast, places=12)

    def test_batched_curve_samples_match_scalar_positions_during_splice_and_reversal(self):
        curve = Quintic(State((.1, -.3), (.03, -.07), (-.5, .2)), (.09, -.27), .3)
        for count in (1, 19, 100, 256):
            for tick, positions in enumerate(curve.positions(count)):
                for batched, scalar in zip(positions, curve.at(curve.duration*tick/count).q):
                    self.assertAlmostEqual(batched, scalar, places=12)

    def test_high_gain_ik_projection_stops_at_target_and_rejects_wrong_direction(self):
        guard = self.open_guard()
        seed = (0.,)*7
        desired = (.0001,)+(0.,)*6
        forward = (.0108,)+(0.,)*6  # gain=27, 2 mm error, 200 ms horizon
        target = (.0021, 0., 0.)
        end = tracking_endpoint(seed, forward, desired, target, guard)
        self.assertAlmostEqual(end[0], target[0])
        end = tracking_endpoint(seed, (-.01,)+(0.,)*6, desired, target, guard)
        self.assertEqual(end, desired)
        end = tracking_endpoint(seed, (.0002,)+(0.,)*6, desired, target, guard)
        self.assertAlmostEqual(end[0], .0003)

    def test_exhausted_planning_budget_returns_without_relaxing_validation(self):
        initial = State((0.,)*7, (0.,)*7, (0.,)*7)
        with patch("franka_hand_teleop.rolling.time.monotonic", side_effect=(0., .021)):
            self.assertIsNone(checked_curve(initial, (.001,)+(0.,)*6, self.motion(),
                                           self.open_guard(), budget_s=.02))

    def test_live_minimum_duration_keeps_tail_overlapping_150ms_handoff(self):
        initial = State((0.,)*7, (0.,)*7, (0.,)*7)
        curve = checked_curve(initial, (.0001,)+(0.,)*6, self.motion(), self.open_guard(),
                              minimum_duration_s=.2)
        self.assertEqual(curve.duration, .2)
        curve.validate(numeric_limits())

    def test_grasp_floor_checks_curve_interior_not_only_endpoints(self):
        guard = self.open_guard()
        guard.tcp = lambda q: (0., 0., q["0"])
        policy = GraspGuard()
        policy.phase, policy.floor = "carry", .13
        curve = Quintic(State((.14,)+(0.,)*6, (-.2,)+(0.,)*6, (0.,)*7),
                        (.14,)+(0.,)*6, .5)
        self.assertTrue(curve_inside(curve, guard))
        self.assertFalse(curve_inside(curve, guard, policy))

    @staticmethod
    def open_guard():
        return SimpleNamespace(names=[str(i) for i in range(7)],
            bounds={str(i): (-3., 3.) for i in range(7)}, reach=numeric_limits().reach,
            tcp=lambda q: (q["0"], 0., 0.), inside=lambda _: True,
            body_inside=lambda _: (True, "synthetic body inside"))

    @staticmethod
    def motion():
        limits = numeric_limits()
        return {"joint_speed": limits.joint_speed, "joint_acceleration": limits.joint_acceleration,
                "joint_jerk": limits.joint_jerk, "max_joint_step": limits.joint_step,
                "max_tool_step": limits.tool_travel, "max_speed": limits.tool_speed,
                "max_acceleration": limits.tool_acceleration, "max_curve_duration_s": limits.maximum_horizon}

    def test_shortest_valid_horizon_preserves_endpoint_before_shrinking(self):
        initial = State((0.,)*7, (0.,)*7, (0.,)*7)
        for distance, duration in ((.0001, .1), (.001, .1), (.01, .2), (.02, .3), (.03, .3)):
            target = (distance,)+(0.,)*6
            curve = checked_curve(initial, target, self.motion(), self.open_guard())
            self.assertIsNotNone(curve)
            self.assertEqual(curve.duration, duration)
            curve.validate(numeric_limits())
            self.assertAlmostEqual(curve.at(duration).q[0], target[0])
            for shorter in (.1, .2, .3, .4, .5, .6, .9, 1.2):
                if shorter < duration:
                    with self.assertRaises(ValueError):
                        Quintic(initial, target, shorter).validate(numeric_limits())

    def test_fifty_percent_faster_curve_preserves_path_and_scales_derivatives(self):
        old_limits = replace(numeric_limits(), joint_speed=1., joint_acceleration=2.,
                             joint_jerk=27., tool_speed=.48, tool_acceleration=3.6)
        initial = State((0.,)*7, (.002,)+(0.,)*6, (.001,)+(0.,)*6)
        target = (.01,)+(0.,)*6
        old = Quintic(initial, target, .3)
        old.validate(old_limits)
        fast = Quintic(State(initial.q, tuple(1.5*v for v in initial.v),
                             tuple(2.25*a for a in initial.a)), target, .2)
        fast.validate(numeric_limits())
        with self.assertRaises(ValueError):
            fast.validate(old_limits)
        for tick in range(101):
            for a, b in zip(old.at(.3*tick/100).q, fast.at(.2*tick/100).q):
                self.assertAlmostEqual(a, b, places=12)
        for order, (before, after) in enumerate(zip(old.bounds()[:3], fast.bounds()[:3]), 1):
            for a, b in zip(before, after):
                self.assertAlmostEqual(b, a*1.5**order, places=10)

    def test_three_times_faster_curve_scales_derivatives_and_preserves_path(self):
        initial = State((0.,)*7, (.002,)+(0.,)*6, (.001,)+(0.,)*6)
        target = (.01,)+(0.,)*6
        slow = Quintic(initial, target, .9)
        slow.validate(replace(numeric_limits(), joint_jerk=1., tool_speed=.16, tool_acceleration=.4))
        fast = Quintic(State(initial.q, tuple(3*v for v in initial.v),
                             tuple(9*a for a in initial.a)), target, .3)
        fast.validate(numeric_limits())
        for tick in range(101):
            before, after = slow.at(.9*tick/100), fast.at(.3*tick/100)
            for a, b in zip(before.q, after.q):
                self.assertAlmostEqual(a, b, places=12)
        for order, (old, new) in enumerate(zip(slow.bounds()[:3], fast.bounds()[:3]), 1):
            for a, b in zip(old, new):
                self.assertAlmostEqual(b, a*3**order, places=10)

    def test_shorter_moving_replacements_and_reversals_keep_c2_and_rest_tail(self):
        initial = Quintic(State((0.,)*7, (0.,)*7, (0.,)*7),
                          (.01,)+(0.,)*6, 1.2).at(.4)
        for distance in (.015, -.005):
            curve = checked_curve(initial, (distance,)+(0.,)*6, self.motion(), self.open_guard())
            self.assertIsNotNone(curve)
            curve.validate(numeric_limits())
            for old, new in zip((initial.q, initial.v, initial.a),
                                (curve.at(0).q, curve.at(0).v, curve.at(0).a)):
                for a, b in zip(old, new):
                    self.assertAlmostEqual(a, b, places=12)
            end = curve.at(curve.duration)
            self.assertLess(max(map(abs, end.v+end.a)), 1e-12)

    def test_moving_splice_preserves_position_velocity_and_acceleration(self):
        old = Quintic(State((0.,)*7, (0.,)*7, (0.,)*7), (.01,)+(0.,)*6, 1.2)
        initial = old.at(.4)
        new = Quintic(initial, (.015,)+(0.,)*6, 1.2)
        new.validate(numeric_limits())
        for before, after in zip((initial.q, initial.v, initial.a),
                                 (new.at(0.).q, new.at(0.).v, new.at(0.).a)):
            for a, b in zip(before, after):
                self.assertAlmostEqual(a, b, places=12)
        self.assertGreater(new.at(0.).v[0], .001)

    def test_whole_polynomial_derivative_bounds_enclose_dense_samples(self):
        curve = Quintic(State((.1,), (.003,), (-.002,)), (.108,), 1.2)
        velocity, acceleration, jerk, _ = curve.bounds()
        for tick in range(1001):
            coefficients = curve.coefficients[0]
            for order, bound in enumerate((velocity[0], acceleration[0], jerk[0]), 1):
                coefficients = derivative(coefficients)
                self.assertLessEqual(abs(evaluate(coefficients, tick/1000))/curve.duration**order,
                                     bound+1e-12)

    def test_curve_overshoot_is_rejected_even_when_both_endpoints_are_inside(self):
        guard = SimpleNamespace(names=[str(i) for i in range(7)],
            bounds={str(i): (-3., 3.) for i in range(7)}, reach=1.722834753163608,
            tcp=lambda q: (q["0"], 0., 0.),
            inside=lambda p: .549 <= p[0] <= .553,
            body_inside=lambda _: (True, "synthetic body inside"))
        curve = Quintic(State((.550,)+(0.,)*6, (.03,)+(0.,)*6, (0.,)*7),
                        (.551,)+(0.,)*6, 1.2)
        self.assertTrue(guard.inside((curve.at(0.).q[0], 0., 0.)))
        self.assertTrue(guard.inside((curve.at(1.2).q[0], 0., 0.)))
        self.assertFalse(curve_inside(curve, guard))

    def test_larger_curve_fits_new_limits_and_is_rejected_by_previous_profile(self):
        initial = State((0.,)*7, (0.,)*7, (0.,)*7)
        curve = Quintic(initial, (.02, -.015, .01, -.005, .003, -.002, .001), 1.4)
        curve.validate(numeric_limits())
        previous = replace(numeric_limits(), tool_travel=.05, tool_speed=.08, tool_acceleration=.2,
                           joint_speed=.1, joint_acceleration=.3, joint_step=math.radians(10))
        with self.assertRaises(ValueError):
            curve.validate(previous)
        velocity, acceleration, jerk, travel = curve.bounds()
        self.assertGreater(numeric_limits().reach*sum(travel), .05)
        self.assertLessEqual(numeric_limits().reach*sum(travel), .1)
        self.assertLessEqual(max(jerk), 1.)
        self.assertLessEqual(numeric_limits().reach*sum(velocity), .16)
        self.assertLessEqual(numeric_limits().reach*(sum(acceleration)+sum(velocity)**2), .4)

    def test_excessive_jerk_and_travel_fail_closed(self):
        initial = State((0.,)*7, (0.,)*7, (0.,)*7)
        for target, duration in (((.1,)*7, 1.2), ((.01,)+(0.,)*6, .1)):
            with self.assertRaises(ValueError):
                Quintic(initial, target, duration).validate(numeric_limits())
        self.assertEqual(numeric_limits().joint_step, math.radians(25))


if __name__ == "__main__":
    unittest.main()
