import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.core import Limits, Teleop
from franka_hand_teleop.grasp import GraspGuard, GripperCommands


class GraspTests(unittest.TestCase):
    def setUp(self):
        self.guard = GraspGuard()
        self.limits = Limits()
        self.start = (.55, 0., .12)

    def close(self):
        g = self.guard
        g.begin(self.start, 1., .23)
        self.assertFalse(g.settled(self.start, 1.1, False, False))
        self.assertFalse(g.settled(self.start, 1.2, True, True))
        self.assertTrue(g.settled(self.start, 1.31, True, True))

    def test_gripper_intent_debounces_and_latches_close_until_deliberate_open(self):
        commands = GripperCommands()
        for opening, now in ((.24, 1.), (.26, 1.02), (.24, 1.04)):
            commands.observe(opening, now)
            self.assertFalse(commands.close_ready)
        commands.observe(.20, 1.11)
        self.assertTrue(commands.close_ready)
        commands.observe(.5, 1.2)
        self.assertTrue(commands.close_ready)
        commands.observe(.81, 1.3)
        commands.observe(.79, 1.32)
        self.assertFalse(commands.open_ready)
        commands.observe(.95, 1.4)
        commands.observe(.95, 1.53)
        self.assertTrue(commands.open_ready)
        self.assertFalse(commands.close_ready)

    def test_stale_width_cannot_repeat_or_reverse_opening_commands(self):
        commands = GripperCommands()
        self.assertAlmostEqual(commands.opening_goal(.08, .04, .04, 1.), .044)
        commands.sent(.044)
        commands.finished(1.1)
        self.assertIsNone(commands.opening_goal(.08, .04, .04, 1.09))
        self.assertIsNone(commands.opening_goal(.08, .04, .04, 1.2))
        self.assertIsNone(commands.opening_goal(.043, .0438, .04, 1.2))
        self.assertAlmostEqual(commands.opening_goal(.08, .044, .04, 1.2), .048)
        commands.reset_moves()
        self.assertAlmostEqual(commands.opening_goal(.08, .04, .04, 1.2), .044)

    def test_stationary_noisy_opening_does_not_chatter(self):
        commands = GripperCommands()
        width = .04
        goals = []
        for tick in range(200):
            desired = .04 + .0008*math.sin(tick)
            goal = commands.opening_goal(desired, width, .04, tick*.02)
            if goal is not None:
                goals.append(goal)
                commands.sent(goal)
        self.assertEqual(goals, [])
        commands.sent(.04)
        for tick in range(200):
            self.assertIsNone(commands.opening_goal(.04, .0395+.0003*math.sin(tick), .04, tick*.02))

    def test_hold_waits_for_tail_and_stationary_then_ignores_xyz(self):
        self.close()
        self.assertEqual(self.guard.phase, "closing")
        self.assertEqual(self.guard.target((.60, .2, .08), self.start, self.limits), self.start)
        self.assertEqual(self.guard.velocity((.1, .1, -.1)), (0., 0., 0.))

    def test_confirm_reclutches_then_lift_keeps_xy_and_blocks_filtered_down_velocity(self):
        self.close()
        raw = (.60, .10, .08)  # Hand moved during closure; do not chase it.
        self.guard.confirmed(self.start, raw)
        self.assertEqual(self.guard.target(raw, self.start, self.limits), self.start)
        moved = self.guard.target((.61, .11, .07), self.start, self.limits)
        self.assertAlmostEqual(moved[0], .56)
        self.assertAlmostEqual(moved[1], .01)
        self.assertAlmostEqual(moved[2], .12)
        self.assertEqual(self.guard.velocity((.01, -.02, -.1)), (.01, -.02, 0.))
        self.guard.target(raw, (.55, 0., .14), self.limits)
        self.assertAlmostEqual(self.guard.target(raw, self.start, self.limits)[2], .14)

    def test_only_measured_5cm_lift_unlocks_normal_z_and_floor_is_1cm(self):
        self.close()
        self.guard.confirmed(self.start, self.start)
        self.guard.target((.55, 0., .22), self.start, self.limits)
        self.assertEqual(self.guard.phase, "lift")  # High hand target is insufficient.
        self.guard.target(self.start, (.55, 0., .169), self.limits)
        self.assertEqual(self.guard.phase, "lift")
        target = self.guard.target((.56, .01, .08), (.55, 0., .171), self.limits)
        self.assertEqual(self.guard.phase, "carry")
        self.assertAlmostEqual(target[2], .13)
        self.assertAlmostEqual(self.guard.floor, .13)
        self.assertEqual(self.guard.velocity((0., 0., -.01)), (0., 0., -.01))

    def test_trajectory_height_check_refuses_lift_dips_and_carry_floor_crossing(self):
        self.close()
        self.guard.confirmed(self.start, self.start)
        self.assertTrue(self.guard.accepts_heights([.12, .13, .14]))
        self.assertTrue(self.guard.accepts_heights([.12]*5))
        self.assertFalse(self.guard.accepts_heights([.12, .14, .13]))
        self.assertFalse(self.guard.accepts_heights([.12, .119]))
        self.guard.target(self.start, (.55, 0., .171), self.limits)
        self.assertTrue(self.guard.accepts_heights([.17, .15, .131]))
        self.assertFalse(self.guard.accepts_heights([.17, .129]))

    def test_interrupt_does_not_confirm_grasp_or_clear_carry_floor(self):
        self.close()
        self.guard.interrupt()
        self.assertTrue(self.guard.holding)
        with self.assertRaises(ValueError):
            self.guard.confirmed(self.start, self.start)
        self.guard.released(self.start, (.60, .2, .20))
        self.assertEqual(self.guard.target((.60, .2, .20), self.start, self.limits), self.start)
        self.assertIsNone(self.guard.floor)
        self.close()
        self.guard.confirmed(self.start, self.start)
        self.guard.interrupt()
        self.assertEqual(self.guard.floor, .13)

    def test_insufficient_headroom_and_settle_timeout_block_closure(self):
        with self.assertRaises(ValueError):
            self.guard.begin((.55, 0., .20), 1., .23)
        self.guard.begin(self.start, 1., .23)
        with self.assertRaises(ValueError):
            self.guard.settled(self.start, 4.1, False, False)

    def test_target_override_changes_controller_error_without_changing_raw_mapping(self):
        teleop = Teleop(self.limits)
        teleop.active = True
        teleop.tracking = {"tracked": True, "monotonic_s": 1., "opening": .2,
                          "control_uvd": (0., 0., 0.)}
        self.assertEqual(teleop.step(1., .02, self.start, True, self.start), (0., 0., 0.))
        self.assertEqual(teleop.target, self.start)

    def test_recorded_67_micrometre_command_offset_does_not_block_upward_lift(self):
        self.close()
        command_z = self.start[2]-.000067230424043
        self.guard.confirmed(self.start, self.start, command_z)
        self.assertTrue(self.guard.accepts_heights([command_z, command_z+.0005]))
        self.assertFalse(self.guard.accepts_heights([command_z, command_z-.0001]))
        self.assertAlmostEqual(self.guard.floor, self.start[2]+.01)
        self.guard.target(self.start, (.55, 0., .171), self.limits)
        self.assertEqual(self.guard.phase, "carry")
        self.assertFalse(self.guard.accepts_heights([self.guard.floor-.001-self.guard.command_z_offset]))

    def test_small_hold_deflection_uses_confirmed_measured_lift_reference(self):
        self.close()
        actual = (.55, 0., .1198)
        self.guard.confirmed(actual, self.start, .1197)
        self.assertAlmostEqual(self.guard.floor_for_path(), actual[2])
        self.assertTrue(self.guard.accepts_heights([.1197, .12]))
        self.assertFalse(self.guard.accepts_heights([.1197, .1195]))
        self.assertEqual(self.guard.target(self.start, actual, self.limits)[2], self.start[2])


if __name__ == "__main__":
    unittest.main()
