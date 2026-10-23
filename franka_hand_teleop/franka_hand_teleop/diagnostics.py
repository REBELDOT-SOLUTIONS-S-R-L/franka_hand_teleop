"""Bounded, read-only telemetry for motion-stop diagnosis; no control decisions."""
import math
from collections import deque

REJECTION_REASONS = {1: "stream disabled", 2: "handoff already pending", 4: "epoch mismatch",
                     8: "parent mismatch", 16: "sequence mismatch", 32: "less than 20 ms lead",
                     64: "more than 300 ms lead", 128: "q/v/a continuity mismatch"}


def rejection_detail(values):
    if (len(values) != 16 or not all(math.isfinite(v) for v in values) or values[0] != 1 or
            values[3] != int(values[3]) or not 0 < values[3] < 256):
        return None
    return {"count": int(values[1]), "robot_time_s": values[2],
            "reasons": [text for bit, text in REJECTION_REASONS.items() if int(values[3]) & bit],
            "packet_epoch": int(values[4]), "packet_sequence": int(values[5]),
            "packet_parent": int(values[6]), "controller_epoch": int(values[7]),
            "accepted_sequence": int(values[8]), "remaining_lead_ms": values[9]*1000,
            "max_q_mismatch_rad": values[10], "max_v_mismatch_rad_s": values[11],
            "max_a_mismatch_rad_s2": values[12], "handoff_pending": bool(values[13]),
            "stream_enabled": bool(values[14]), "handoff_robot_time_s": values[15]}


def member(value, path):
    for name in path.split("."):
        value = getattr(value, name, None)
    return value


def scalar(value):
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def vector(value):
    return [scalar(v) for v in value] if value is not None else []


def xyz(value):
    return [scalar(getattr(value, key, None)) for key in ("x", "y", "z")]


def errors(message):
    fields = message.get_fields_and_field_types() if message is not None else {}
    return [name for name in fields if getattr(message, name, False) is True]


class RobotHistory:
    def __init__(self):
        self.samples = deque(maxlen=120)
        self.load = None

    def observe(self, message, now):
        if self.samples and now-self.samples[-1]["received_monotonic_s"] < .019:
            return
        q = vector(member(message, "measured_joint_state.position"))
        desired = vector(member(message, "desired_joint_state.position"))
        mismatch = [abs(a-b) for a, b in zip(q, desired) if a is not None and b is not None]
        wrench = member(message, "o_f_ext_hat_k.wrench")
        indicators = member(message, "collision_indicators")
        self.samples.append({"received_monotonic_s": now, "robot_time_s": scalar(message.time),
            "mode": int(message.robot_mode), "fci_success": scalar(message.control_command_success_rate),
            "external_wrench_N_Nm": xyz(member(wrench, "force"))+xyz(member(wrench, "torque")),
            "external_joint_torque_Nm": vector(member(message, "tau_ext_hat_filtered.effort")),
            "dq_rad_s": vector(member(message, "measured_joint_state.velocity")),
            "desired_dq_rad_s": vector(member(message, "desired_joint_state.velocity")),
            "desired_ddq_rad_s2": vector(member(message, "ddq_d")),
            "max_tracking_error_rad": max(mismatch, default=None),
            "cartesian_collision": xyz(member(indicators, "is_cartesian_linear_collision"))+
                                   xyz(member(indicators, "is_cartesian_angular_collision")),
            "joint_collision": vector(member(indicators, "is_joint_collision")),
            "errors": errors(message.current_errors), "last_motion_errors": errors(message.last_motion_errors)})
        self.load = {"ee_mass_kg": scalar(member(message, "inertia_ee.inertia.m")),
                     "ee_com_m": xyz(member(message, "inertia_ee.inertia.com")),
                     "load_mass_kg": scalar(member(message, "inertia_load.inertia.m")),
                     "load_com_m": xyz(member(message, "inertia_load.inertia.com")),
                     "total_mass_kg": scalar(member(message, "inertia_total.inertia.m"))}
        while self.samples and now-self.samples[0]["received_monotonic_s"] > 2.:
            self.samples.popleft()

    def summary(self, now):
        samples = list(self.samples)
        peaks = []
        for axis in range(6):
            values = [abs(s["external_wrench_N_Nm"][axis]) for s in samples
                      if s["external_wrench_N_Nm"][axis] is not None]
            peaks.append(max(values, default=None))
        return {"samples": len(samples), "window_s": samples[-1]["received_monotonic_s"]-
                samples[0]["received_monotonic_s"] if samples else 0.,
                "last_received_age_s": now-samples[-1]["received_monotonic_s"] if samples else None,
                "peak_abs_external_wrench_N_Nm": peaks, "configured_inertia": self.load,
                "last_samples": samples[-8:]}
