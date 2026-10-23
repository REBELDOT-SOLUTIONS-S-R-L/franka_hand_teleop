import json
import math
import unittest
from types import SimpleNamespace as NS

from franka_hand_teleop.diagnostics import RobotHistory, rejection_detail


class ErrorFixture:
    def get_fields_and_field_types(self):
        return {"cartesian_reflex": "boolean"}
    cartesian_reflex = False


def robot(force=3., mass=.73):
    v = lambda x: NS(x=x, y=0., z=0.)
    joints = NS(position=[.1]*7, velocity=[.01]*7, effort=[.2]*7)
    return NS(time=50., robot_mode=2, control_command_success_rate=1., measured_joint_state=joints,
              desired_joint_state=joints, tau_ext_hat_filtered=joints, ddq_d=[.02]*7,
              o_f_ext_hat_k=NS(wrench=NS(force=v(force), torque=v(.1))),
              collision_indicators=NS(), inertia_ee=NS(inertia=NS(m=mass, com=v(.02))),
              inertia_load=NS(inertia=NS(m=0., com=v(0.))),
              inertia_total=NS(inertia=NS(m=mass)),
              current_errors=ErrorFixture(), last_motion_errors=ErrorFixture())


class DiagnosticsTests(unittest.TestCase):
    def test_rejection_decodes_multiple_actual_conditions_and_mismatches(self):
        detail = rejection_detail([1., 3., 12., 160., 10., 24., 23., 10., 23., .009,
                                   1e-8, 2e-7, 3e-6, 0., 1., 12.009])
        self.assertEqual(detail["reasons"], ["less than 20 ms lead", "q/v/a continuity mismatch"])
        self.assertEqual(detail["remaining_lead_ms"], 9.)
        self.assertEqual(detail["packet_sequence"], 24)
        self.assertIsNone(rejection_detail([0.]*16))
        self.assertIsNone(rejection_detail([math.nan]*16))
        self.assertIsNone(rejection_detail([]))

    def test_bounded_history_captures_forces_mass_and_pre_stop_dynamics(self):
        history = RobotHistory()
        for i in range(201):
            history.observe(robot(force=-12. if i == 195 else 2.), 10.+i*.02)
        report = history.summary(14.12)
        self.assertLessEqual(report["window_s"], 2.001)
        self.assertLessEqual(report["samples"], 101)
        self.assertEqual(report["peak_abs_external_wrench_N_Nm"][0], 12.)
        self.assertEqual(report["configured_inertia"]["ee_mass_kg"], .73)
        self.assertEqual(report["configured_inertia"]["load_mass_kg"], 0.)
        self.assertAlmostEqual(report["last_received_age_s"], .12)
        self.assertEqual(len(report["last_samples"]), 8)
        json.dumps(report, allow_nan=False)

    def test_empty_or_nonfinite_telemetry_cannot_break_stop_logging(self):
        history = RobotHistory()
        json.dumps(history.summary(10.), allow_nan=False)
        history.observe(robot(force=math.nan, mass=math.nan), 10.)
        report = history.summary(10.2)
        self.assertIsNone(report["configured_inertia"]["ee_mass_kg"])
        self.assertIsNone(report["peak_abs_external_wrench_N_Nm"][0])
        json.dumps(report, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
