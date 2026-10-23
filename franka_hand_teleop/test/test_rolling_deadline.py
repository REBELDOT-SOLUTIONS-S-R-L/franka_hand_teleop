import copy
import unittest

from franka_hand_teleop.diagnostics import rejection_detail
from franka_hand_teleop.rolling import retryable_deadline


class DeadlineTests(unittest.TestCase):
    def setUp(self):
        self.detail = rejection_detail([1., 3., 12., 32., 10., 24., 23., 10., 23., .009,
                                        0., 0., 0., 0., 1., 12.009])
        self.stream = [0.]*60
        self.stream[1:6] = [12.02, 10., 23., 1., 3.]

    def allowed(self, detail=None, stream=None, **kwargs):
        params = {"epoch": 10, "sequence": 24, "handoff": 12.009}
        params.update(kwargs)
        return retryable_deadline(self.detail if detail is None else detail,
                                  stream=self.stream if stream is None else stream, **params)

    def test_confirmed_timing_only_rejection_can_wait_for_a_fresh_proposal(self):
        self.assertTrue(self.allowed())
        detail = self.detail.copy()
        detail["max_v_mismatch_rad_s"] = 2.5e-16
        detail["max_a_mismatch_rad_s2"] = 3e-15
        self.assertTrue(self.allowed(detail=detail))

    def test_old_or_unrelated_nack_cannot_clear_an_outstanding_packet(self):
        for key, value in [('epoch', 12), ('sequence', 25), ('handoff', 12.2),
                           ('epoch', None), ('sequence', None), ('handoff', None)]:
            with self.subTest(key=key, value=value):
                self.assertFalse(self.allowed(**{key: value}))
        for index, value in [(1, 11.99), (2, 8.), (3, 24.), (4, 0.), (5, 2.), (58, 1.)]:
            d = self.stream.copy()
            d[index] = value
            self.assertFalse(self.allowed(stream=d))

    def test_protocol_continuity_pending_or_disabled_rejections_are_never_retried(self):
        for key, value in [('reasons', ['less than 20 ms lead', 'q/v/a continuity mismatch']),
                           ('reasons', ['epoch mismatch']), ('packet_parent', 22),
                           ('accepted_sequence', 22), ('stream_enabled', False),
                           ('handoff_pending', True), ('max_q_mismatch_rad', 2e-9),
                           ('max_v_mismatch_rad_s', 2e-8), ('max_a_mismatch_rad_s2', 2e-7)]:
            d = copy.deepcopy(self.detail)
            d[key] = value
            self.assertFalse(self.allowed(detail=d), (key, value))


if __name__ == '__main__':
    unittest.main()
