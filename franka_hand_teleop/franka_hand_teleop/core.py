"""Transport validation and fixed camera-to-workspace mapping. No ROS dependencies."""

import json
import math
from dataclasses import dataclass

DOMAIN = "132"
NAMESPACE = "/hand_teleop_fake"
PORT = 49300
TIMEOUT = 0.25
SETTLE_TIME = 0.30


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def vector(value):
    return isinstance(value, (list, tuple)) and len(value) == 3 and all(map(finite, value))


def decode(data, now):
    """Reject stale source frames, NaNs, wrong units, and malformed local datagrams."""
    try:
        packet = json.loads(data)
        if (not isinstance(packet, dict) or type(packet.get("version")) is not int or
                packet["version"] != 2):
            return None
        if packet.get("kind") != "tracking":
            return None
        if not isinstance(packet.get("session"), str) or not 1 <= len(packet["session"]) <= 64:
            return None
        if type(packet.get("sequence")) is not int or packet["sequence"] < 0:
            return None
        stamp = packet.get("monotonic_s")
        if not finite(stamp) or not -0.02 <= now - stamp <= TIMEOUT:
            return None
        if packet.get("frame") != "camera_color_optical_frame" or packet.get("units") != "m":
            return None
        if packet.get("source") not in ("realsense", "demo", "webcam"):
            return None
        if type(packet.get("tracked")) is not bool:
            return None
        if packet["tracked"]:
            if not vector(packet.get("palm_m")) or not 0.15 <= packet["palm_m"][2] <= 3.0:
                return None
            if not finite(packet.get("opening")) or not 0 <= packet["opening"] <= 1:
                return None
            if not vector(packet.get("control_uvd")):
                return None
            if packet["source"] == "webcam":
                return None
        return packet
    except (ValueError, TypeError, UnicodeError):
        return None


@dataclass
class Limits:
    minimum: tuple = (0.45, -0.30, 0.05)
    maximum: tuple = (0.65, 0.30, 0.25)
    margin: float = 0.02
    speed: float = 0.08
    acceleration: float = 0.30
    gain: float = 3.0
    max_hand_step: float = 0.10
    max_hand_speed: float = 1.5
    body_minimum: tuple = (-.45, -.55, -.05)
    body_maximum: tuple = (.85, .55, 1.10)

    def __post_init__(self):
        if not vector(self.minimum) or not vector(self.maximum):
            raise ValueError("Workspace requires three finite minimum/maximum values")
        if any(a >= b for a, b in zip(self.minimum, self.maximum)):
            raise ValueError("Workspace minimum must be below maximum")
        if (not vector(self.body_minimum) or not vector(self.body_maximum) or
                any(a >= b for a, b in zip(self.body_minimum, self.body_maximum))):
            raise ValueError("Whole-arm workspace requires finite increasing XYZ bounds")
        if not finite(self.margin) or self.margin < .005 or any(
                b-a <= 2*self.margin for a, b in zip(self.minimum, self.maximum)):
            raise ValueError("Workspace margin must be at least 5 mm and fit inside the box")
        for value in (self.speed, self.acceleration, self.gain,
                      self.max_hand_step, self.max_hand_speed):
            if not finite(value) or value <= 0:
                raise ValueError("Motion limits must be finite and positive")

    def contains(self, point):
        return vector(point) and all(a <= p <= b for a, p, b in
                                     zip(self.minimum, point, self.maximum))

    def clamp(self, point):
        return tuple(max(a, min(b, p)) for a, p, b in
                     zip(self.minimum, point, self.maximum))

    def target(self, control_uvd):
        # Absolute mapping: near -> +X, raw camera right -> +Y, camera top -> +Z.
        # The camera preview is mirrored; reverse only horizontal robot mapping.
        u, v, d = (max(0.0, min(1.0, n)) for n in control_uvd)
        fractions = (1-d, u, 1-v)
        return tuple(a+self.margin + f*(b-a-2*self.margin)
                     for a, b, f in zip(self.minimum, self.maximum, fractions))

    def camera_control(self, tcp):
        """Inverse absolute mapping, intentionally unclipped for off-range guides."""
        if not vector(tcp):
            raise ValueError("TCP must contain three finite coordinates")
        x, y, z = ((p-a-self.margin)/(b-a-2*self.margin)
                   for p, a, b in zip(tcp, self.minimum, self.maximum))
        return (y, 1-z, 1-x)


def capped(values, maximum):
    norm = math.sqrt(sum(v * v for v in values))
    return tuple(v * min(1.0, maximum / max(norm, 1e-12)) for v in values)


class Teleop:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.tracking = None
        self.last = {}
        self.valid_since = None
        self.active = False
        self.reason = "Waiting for hand tracking"
        self.target = None
        self.velocity = (0.0, 0.0, 0.0)
        self.opening = None

    def stop(self, reason):
        self.active = False
        self.velocity = (0.0, 0.0, 0.0)
        self.reason = reason
        self.valid_since = None

    def receive(self, packet):
        kind = packet["kind"]
        previous = self.last.get(kind)
        identity = (packet["session"], packet["sequence"], packet["monotonic_s"])
        if previous and (identity[2] <= previous[2] or
                         (identity[0] == previous[0] and identity[1] <= previous[1])):
            return
        if previous and identity[0] != previous[0]:
            self.stop("Tracking sender restarted; waiting for valid tracking")
        self.last[kind] = identity
        old = self.tracking
        self.tracking = packet
        if not packet["tracked"]:
            self.stop("Waiting for hand tracking, depth, and gesture calibration")
        elif old and old["tracked"]:
            distance = math.dist(old["palm_m"], packet["palm_m"])
            dt = packet["monotonic_s"] - old["monotonic_s"]
            if (dt > TIMEOUT or old["source"] != packet["source"] or
                distance > self.limits.max_hand_step or
                    distance / max(dt, 1e-6) > self.limits.max_hand_speed):
                self.stop("Tracking gap/jump or source change; waiting for valid tracking")

    def step(self, now, dt, actual, ready=True, target_override=None):
        if not ready or actual is None:
            self.stop("Fake backend/feedback/Servo not ready")
            return self.velocity
        if not self.limits.contains(actual):
            self.stop(f"TCP outside workspace: {actual}")
            return self.velocity
        if (not self.tracking or not self.tracking["tracked"] or
                now - self.tracking["monotonic_s"] > TIMEOUT):
            self.stop("Tracking expired/unavailable; waiting for hand")
            return self.velocity
        if not self.active:
            if self.valid_since is None:
                self.valid_since = now
            if now - self.valid_since < SETTLE_TIME:
                self.reason = "Waiting for valid tracking"
                return self.velocity
            self.active = True
        self.target = (self.limits.target(self.tracking["control_uvd"])
                       if target_override is None else target_override)
        error = tuple(t - p for t, p in zip(self.target, actual))
        desired = capped(tuple(self.limits.gain * e if abs(e) > 0.001 else 0.0
                               for e in error), self.limits.speed)
        change = capped(tuple(v - old for v, old in zip(desired, self.velocity)),
                        self.limits.acceleration * min(max(dt, 0), 0.05))
        self.velocity = tuple(old + d for old, d in zip(self.velocity, change))
        # Brake before the boundary, allowing for Servo/controller latency.
        horizon = .25
        self.velocity = tuple(max(-(max(0, p-a-.005))/horizon,
                                  min(max(0, b-.005-p)/horizon, v))
                              for a, p, b, v in zip(self.limits.minimum, actual,
                                                   self.limits.maximum, self.velocity))
        self.opening = self.tracking["opening"]
        self.reason = "Tracking active (FAKE ONLY)"
        return self.velocity
