"""Offline deployment validation and conservative real-arm commissioning policy."""
import ipaddress
import math
from pathlib import Path

import yaml

from .core import SETTLE_TIME, TIMEOUT, Limits, finite, vector

REAL_DOMAIN = "133"
REAL_NAMESPACE = "/hand_teleop_real"
REAL_PORT = 49301
REAL_PLUGIN = "franka_hardware/FrankaHardwareInterface"
REAL_ARM_CONTROLLER = "franka_hand_controllers/RobotClockTrajectoryController"
REQUIRED_CHECKS = ("robot_safety_verified", "workspace_verified", "tool_and_load_verified")


def endpoint(value):
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_loopback or address.is_multicast or int(address) == 0xffffffff:
        raise ValueError("Specify the robot's C2 IPv4 address")
    return str(address)


def load_profile(path, execute=False):
    profile = yaml.safe_load(Path(path).read_text())
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise ValueError("Expected deployment profile schema_version: 1")
    p = profile["motion"]
    limits = Limits(minimum=tuple(p["workspace_min"]), maximum=tuple(p["workspace_max"]),
                    body_minimum=tuple(p["arm_workspace_min"]),
                    body_maximum=tuple(p["arm_workspace_max"]), margin=p["workspace_margin"],
                    speed=p["max_speed"], acceleration=p["max_acceleration"],
                    gain=p["tracking_gain"])
    for name in ("joint_speed", "joint_acceleration", "joint_jerk", "max_joint_step",
                 "max_tool_step", "arm_capture_distance", "table_size", "table_thickness"):
        if not finite(p[name]) or p[name] <= 0:
            raise ValueError(f"{name} must be finite and positive")
    # Fixed manufacturer/protocol envelopes, not copies of the selected tuning.
    # https://frankarobotics.github.io/docs/robot_specifications.html
    if limits.speed > 3. or limits.acceleration > 9.:
        raise ValueError("FR3 envelope: max_speed <=3 m/s and max_acceleration <=9 m/s²")
    if p["joint_speed"] > 2.62 or p["joint_acceleration"] > 10. or p["joint_jerk"] > 5000.:
        raise ValueError("FR3 envelope: joint_speed <=2.62 rad/s, acceleration <=10 rad/s², jerk <=5000 rad/s³")
    maximum = p.get("max_curve_duration_s")
    if not finite(maximum) or not .05 <= maximum <= 1.5:
        raise ValueError("max_curve_duration_s must be 0.05–1.5 s for the bounded-stop protocol")
    timing = profile.get("bridge")
    keys = ("update_period_s", "proposal_horizon_s", "handoff_lead_s", "min_publish_lead_s",
            "planning_budget_s", "min_curve_duration_s")
    if not isinstance(timing, dict) or any(not finite(timing.get(k)) or timing[k] <= 0 for k in keys):
        raise ValueError("bridge timing fields must be explicit finite positive seconds")
    if not .02 <= timing["proposal_horizon_s"] <= .5:
        raise ValueError("bridge.proposal_horizon_s must be 0.02–0.5 s")
    if (not .02 < timing["min_publish_lead_s"] < timing["handoff_lead_s"] <= .30 or
            timing["planning_budget_s"]+timing["min_publish_lead_s"]+.02 > timing["handoff_lead_s"]+1e-12):
        raise ValueError("bridge handoff must cover planning budget + publish lead + 20 ms state-report reserve")
    if (not timing["handoff_lead_s"]+.02 <= timing["min_curve_duration_s"] <= maximum or
            timing["update_period_s"] >= timing["min_curve_duration_s"]-.02):
        raise ValueError("bridge minimum curve duration must overlap handoff/update timing by at least 20 ms")
    if type(profile.get("enable_gripper")) is not bool:
        raise ValueError("enable_gripper must be a boolean")
    force = profile.get("grasp_force_N", 30.)
    if not finite(force) or not 30. <= force <= 70.:
        raise ValueError("Franka Hand grasp force must be 30–70 N")
    profile["grasp_force_N"] = force
    gripper_speed = profile.get("gripper_speed_m_s")
    if not finite(gripper_speed) or not 0 < gripper_speed <= .2:
        raise ValueError("Gripper speed must be positive and <=200 mm/s")
    profile["gripper_speed_m_s"] = gripper_speed
    checks = profile.get("commissioning", {})
    if any(type(checks.get(k)) is not bool for k in REQUIRED_CHECKS):
        raise ValueError("All commissioning fields must be explicit booleans")
    if execute and not all(checks[k] for k in REQUIRED_CHECKS):
        missing = ", ".join(k for k in REQUIRED_CHECKS if not checks[k])
        raise ValueError(f"Motion blocked: unverified installation requirements: {missing}")
    return profile, limits


def segment_duration(delta, motion, reach):
    """Quintic common progress: max f'=1.875, |f''|=10/sqrt(3), |f'''|=60.

    All joints share the same f(t), so the geometric path remains the straight
    joint segment checked by WorkspaceGuard. Reach also bounds TCP speed/accel.
    """
    delta = tuple(delta)
    if not delta or not all(map(finite, delta)):
        raise ValueError("Invalid segment displacement")
    peak, total = max(map(abs, delta)), sum(map(abs, delta))
    if peak > motion["max_joint_step"]:
        raise ValueError("Servo step exceeds joint displacement limit")
    length = reach * total
    duration = max(.20, 1.875*peak/motion["joint_speed"],
                   math.sqrt((10/math.sqrt(3))*peak/motion["joint_acceleration"]),
                   (60*peak/motion["joint_jerk"])**(1/3),
                   1.875*length/motion["max_speed"],
                   math.sqrt(((10/math.sqrt(3))*length + 1.875**2*length*total)/
                             motion["max_acceleration"]))
    if duration > 1.5:
        raise ValueError("Servo step requires more than the 1.5 s bounded segment")
    return duration


def bounded_increment(seed, proposed, desired, measured, horizon, motion, reach):
    """Rebase a seeded IK increment onto continuous FCI desired commands.

    Clip the executed segment, not the extrapolated IK proposal. All outgoing
    geometry must still be checked by WorkspaceGuard. No forecast is accumulated.
    """
    values = (seed, proposed, desired, measured)
    if any(len(v) != 7 or not all(map(finite, v)) for v in values):
        raise ValueError("Seeded proposal requires seven finite joint positions")
    if not finite(horizon) or not .02 <= horizon <= .5:
        raise ValueError("Seeded proposal horizon must be 20-500 ms")
    if not finite(reach) or reach < 1.:
        raise ValueError("Invalid reach bound")
    if max(abs(a-b) for a, b in zip(seed, measured)) > .002:
        raise ValueError("Servo proposal seed differs from current measured joints")
    if max(abs(a-b) for a, b in zip(desired, measured)) >= .002:
        raise ValueError("Desired/measured joint discrepancy exceeds 0.002 rad")
    delta = [b-a for a, b in zip(seed, proposed)]
    peak, length = max(map(abs, delta)), reach*sum(map(abs, delta))
    factor = min(1., motion["max_joint_step"]/max(peak, 1e-12),
                 motion["max_tool_step"]/max(length, 1e-12))
    delta = [v*factor for v in delta]
    total = max(horizon, segment_duration(delta, motion, reach))
    return [a+d for a, d in zip(desired, delta)], total, delta


class MotionPermit:
    """A software latch; it is not X4 readback or a safety-rated enable."""
    def __init__(self):
        self.armed = False
        self.valid_since = None
        self.reason = "Disarmed; explicit arm required"
        self.generation = 0

    def stop(self, reason):
        self.armed = False
        self.valid_since = None
        self.reason = reason
        self.generation += 1

    def observe(self, now, tracking_ok, ready):
        if not tracking_ok or not ready:
            self.stop("Tracking or robot readiness lost; explicit re-arm required")
        elif self.valid_since is None:
            self.valid_since = now

    def arm(self, now, actual, target, stationary, capture_distance):
        if self.valid_since is None or now-self.valid_since < SETTLE_TIME:
            return False, "Need 0.3 seconds of fresh, valid tracking and robot readiness"
        if not stationary:
            return False, "Robot must be stationary before arming"
        if actual is None or target is None or math.dist(actual, target) > capture_distance:
            return False, "Align the camera target within the arming capture distance of the actual TCP"
        self.armed = True
        self.reason = "Armed"
        self.generation += 1
        return True, self.reason


def tracking_fresh(packet, now):
    return bool(packet and packet["tracked"] and packet["source"] == "realsense" and
                -.02 <= now-packet["monotonic_s"] <= TIMEOUT)


def alignment_text(state):
    """Operator-facing explanation of the fixed hand-to-robot target, in cm."""
    actual, target = state.get("tcp_m"), state.get("target_m")
    if state.get("armed"):
        message = "TELEOP: ARMED - physical motion enabled"
        if vector(actual):
            message += " | TCP=("+", ".join(f"{v*100:.2f}" for v in actual)+") cm"
        if vector(actual) and vector(target):
            message += f" | gap={math.dist(actual, target)*100:.1f} cm"
        if "arm_actions_completed" in state:
            label = "accepted updates" if state.get("motion_mode") == "rolling" else "completed steps"
            message += f" | {label}={state['arm_actions_completed']}"
        result = state.get("last_motion_result")
        if result:
            message += f" | last TCP step={result['tcp_displacement_m']*1000:.3f} mm"
            message += f" | target progress={result['target_progress_m']*1000:+.3f} mm"
        rate = state.get("control_command_success_rate")
        if finite(rate):
            message += f" | FCI command success={rate:.1%}"
        return message
    if not vector(actual) or not vector(target):
        return "TELEOP: DISARMED - waiting for actual TCP and valid hand target; " + state.get("reason", "")
    gap = math.dist(actual, target)*100
    capture = state.get("arm_capture_distance_m", .03)*100
    def point(p):
        return "("+", ".join(f"{v*100:.1f}" for v in p)+")"
    ready = "yes" if state.get("ready") else "no"
    message = (f"ALIGNMENT: gap={gap:.1f} cm (need <= {capture:.1f} cm) | "
            f"TCP={point(actual)} cm | TARGET={point(target)} cm | ready={ready}" +
            ("" if state.get("ready") else " | "+state.get("reason", "")))
    message += " | DISARMED; explicit arm required"
    if state.get("last_stop_reason"):
        message += " | last stop="+state["last_stop_reason"]
    return message


def main():
    """Validate and display settings without importing ROS or connecting to hardware."""
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Check motion settings without starting the robot")
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    try:
        profile, _ = load_profile(args.profile)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))
    print("Profile valid; no hardware connection")
    print(json.dumps({"motion": profile["motion"], "bridge": profile["bridge"],
                      "gripper_speed_m_s": profile["gripper_speed_m_s"]}, indent=2))


if __name__ == "__main__":
    main()
