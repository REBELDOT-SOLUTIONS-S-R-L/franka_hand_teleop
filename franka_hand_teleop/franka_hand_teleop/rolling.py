"""Finite rolling quintics and conservative whole-curve bounds; no ROS I/O."""
import math
import time
from dataclasses import dataclass
from functools import cached_property
from itertools import pairwise

import numpy as np


@dataclass(frozen=True)
class State:
    q: tuple
    v: tuple
    a: tuple


def derivative(coefficients):
    return tuple(i*c for i, c in enumerate(coefficients) if i)


def evaluate(coefficients, x):
    value = 0.
    for coefficient in reversed(coefficients):
        value = value*x+coefficient
    return value


def bernstein(coefficients):
    degree = len(coefficients)-1
    return tuple(sum(coefficients[j]*math.comb(k, j)/math.comb(degree, j)
                     for j in range(k+1)) for k in range(degree+1))


def split(control):
    levels = [tuple(control)]
    while len(levels[-1]) > 1:
        levels.append(tuple((a+b)/2 for a, b in zip(levels[-1], levels[-1][1:])))
    return tuple(level[0] for level in levels), tuple(level[-1] for level in levels[::-1])


def bound_transform(degree, depth=5):
    """Power coefficients -> the same subdivided Bernstein control hulls.

    Precompute the linear operation once, rather than repeating 31 scalar
    subdivisions for every joint, derivative, and candidate horizon.
    """
    conversion = np.zeros((degree+1, degree+1))
    for k in range(degree+1):
        for j in range(k+1):
            conversion[k, j] = math.comb(k, j)/math.comb(degree, j)
    pieces = conversion[None, :, :]
    for _ in range(depth):
        levels = [pieces]
        while levels[-1].shape[1] > 1:
            levels.append((levels[-1][:, :-1]+levels[-1][:, 1:])/2)
        left = np.stack([level[:, 0] for level in levels], axis=1)
        right = np.stack([level[:, -1] for level in reversed(levels)], axis=1)
        pieces = np.stack((left, right), axis=1).reshape(-1, degree+1, degree+1)
    return pieces.reshape(-1, degree+1)


_BOUND_TRANSFORMS = {degree: bound_transform(degree) for degree in range(6)}


def range_bounds(coefficients):
    """Whole-interval bounds for a batch of equal-degree polynomials."""
    coefficients = np.asarray(coefficients, dtype=float)
    controls = coefficients @ _BOUND_TRANSFORMS[coefficients.shape[1]-1].T
    return controls.min(axis=1), controls.max(axis=1)


def range_bound(coefficients, depth=5):
    # Same depth-five convex hulls; no derivative/path sampling is substituted.
    if depth == 5 and len(coefficients) <= 6:
        low, high = range_bounds([coefficients])
        return float(low[0]), float(high[0])
    pieces = [bernstein(coefficients)]
    for _ in range(depth):
        pieces = [part for piece in pieces for part in split(piece)]
    return min(min(p) for p in pieces), max(max(p) for p in pieces)


@dataclass(frozen=True)
class Limits:
    reach: float
    joint_speed: float
    joint_acceleration: float
    joint_jerk: float
    joint_step: float
    tool_travel: float
    tool_speed: float
    tool_acceleration: float
    maximum_horizon: float

    @classmethod
    def from_motion(cls, motion, reach):
        return cls(reach=reach, joint_speed=motion["joint_speed"],
                   joint_acceleration=motion["joint_acceleration"], joint_jerk=motion["joint_jerk"],
                   joint_step=motion["max_joint_step"], tool_travel=motion["max_tool_step"],
                   tool_speed=motion["max_speed"], tool_acceleration=motion["max_acceleration"],
                   maximum_horizon=motion["max_curve_duration_s"])


class Quintic:
    def __init__(self, initial, target, duration):
        if (not 0. < duration <= 1.5 or not len(target) == len(initial.q)
                or len(initial.v) != len(target) or len(initial.a) != len(target)
                or not all(math.isfinite(x) for values in (initial.q, initial.v, initial.a, target)
                           for x in values)):
            raise ValueError('Invalid finite quintic state/duration')
        self.duration = duration
        self.coefficients = []
        for q, v, a, end in zip(initial.q, initial.v, initial.a, target):
            c0, c1, c2 = q, v*duration, .5*a*duration**2
            d, dv, da = end-c0-c1-c2, -c1-2*c2, -2*c2
            self.coefficients.append((c0, c1, c2, 10*d-4*dv+.5*da,
                                      -15*d+7*dv-da, 6*d-3*dv+.5*da))

    def at(self, elapsed):
        x = min(1., max(0., elapsed/self.duration))
        result = []
        for order in range(3):
            values = []
            for coefficients in self.coefficients:
                for _ in range(order):
                    coefficients = derivative(coefficients)
                values.append(evaluate(coefficients, x)/self.duration**order)
            result.append(tuple(values))
        return State(*result)

    @cached_property
    def _bounds(self):
        peaks = []
        derivatives = self.coefficients
        for order in (1, 2, 3):
            derivatives = [derivative(c) for c in derivatives]
            low, high = range_bounds(derivatives)
            peaks.append(tuple(np.maximum(np.abs(low), np.abs(high))/self.duration**order))
        # Integral of abs derivative bounded by control-polygon variation.
        travel = []
        for coefficients in self.coefficients:
            controls = bernstein(coefficients)
            travel.append(sum(abs(b-a) for a, b in pairwise(controls)))
        return (*peaks, tuple(travel))

    def bounds(self):
        return self._bounds

    def positions(self, count):
        """Evaluate the original polynomial at all unchanged guard sample times."""
        return np.polynomial.polynomial.polyval(
            np.linspace(0., 1., count+1), np.asarray(self.coefficients).T).T

    def validate(self, limits):
        velocity, acceleration, jerk, travel = self.bounds()
        if (self.duration > limits.maximum_horizon
                or max(velocity) > limits.joint_speed+1e-12
                or max(acceleration) > limits.joint_acceleration+1e-12
                or max(jerk) > limits.joint_jerk+1e-12
                or max(travel) > limits.joint_step+1e-12
                or limits.reach*sum(travel) > limits.tool_travel+1e-12
                or limits.reach*sum(velocity) > limits.tool_speed+1e-12
                or limits.reach*(sum(acceleration)+sum(velocity)**2) > limits.tool_acceleration+1e-12):
            raise ValueError('Candidate exceeds derivative, travel or horizon bound')



def tracking_endpoint(seed, proposed, desired, target, guard):
    """Cap the extrapolated IK step at the live TCP target instead of chasing past it.

    High following gains can request >100% of the remaining error during the
    200 ms IK horizon. Rebase only on the current command, never an accumulated
    forecast. Full polynomial admission still checks the resulting moving splice.
    """
    delta = tuple(b-a for a, b in zip(seed, proposed))
    endpoint = tuple(q+d for q, d in zip(desired, delta))
    start_tcp = guard.tcp(dict(zip(guard.names, desired)))
    end_tcp = guard.tcp(dict(zip(guard.names, endpoint)))
    step = tuple(b-a for a, b in zip(start_tcp, end_tcp))
    error = tuple(b-a for a, b in zip(start_tcp, target))
    square = sum(d*d for d in step)
    factor = min(1., max(0., sum(e*d for e, d in zip(error, step))/square)) if square > 1e-16 else 1.
    return tuple(q+factor*d for q, d in zip(desired, delta))


def checked_curve(initial, target, motion, guard, grasp=None, budget_s=None, minimum_duration_s=.1):
    deadline = time.monotonic()+budget_s if budget_s is not None else math.inf
    limits = Limits.from_motion(motion, guard.reach)
    difference = [b-a for a, b in zip(initial.q, target)]
    factor = min(1., .8*limits.tool_travel/max(guard.reach*sum(map(abs, difference)), 1e-12))
    # Preserve the requested endpoint when a longer admissible curve can reach
    # it. Within each endpoint size, use the shortest fully checked horizon;
    # always preferring 1.2 s needlessly slowed every replacement while moving.
    for shrink in (1., .75, .5, .25):
        for duration in sorted({minimum_duration_s, .1, .2, .3, .4, .5, .6, .9, 1.2, 1.4,
                                limits.maximum_horizon}):
            if duration < minimum_duration_s or duration > limits.maximum_horizon:
                continue
            if time.monotonic() >= deadline:
                return None  # Yield to feedback callbacks; keep the previous finite tail.
            endpoint = tuple(q+d*factor*shrink for q, d in zip(initial.q, difference))
            curve = Quintic(initial, endpoint, duration)
            try:
                curve.validate(limits)
            except ValueError:
                continue
            if time.monotonic() >= deadline:
                return None
            if curve_inside(curve, guard, grasp):
                return curve if time.monotonic() < deadline else None
    return None


def curve_inside(curve, guard, grasp=None):
    # Check actual curved polynomial; straight endpoint interpolation is insufficient.
    lows, highs = range_bounds(curve.coefficients)
    for name, low, high in zip(guard.names, lows, highs):
        if low < guard.bounds[name][0] or high > guard.bounds[name][1]:
            return False
    velocity = curve.bounds()[0]
    count = max(1, math.ceil(guard.reach*sum(velocity)*curve.duration/.0018))
    if count > 256:
        return False
    samples = curve.positions(count)
    if hasattr(guard, "samples_inside"):
        if grasp is not None:
            return guard.samples_inside(samples, height_check=grasp.accepts_heights)
        return guard.samples_inside(samples)
    for sample in samples:
        state = dict(zip(guard.names, sample))
        if not guard.inside(guard.tcp(state)) or not guard.body_inside(state)[0]:
            return False
    return grasp is None or grasp.accepts_heights(
        guard.tcp(dict(zip(guard.names, sample)))[2] for sample in samples)


def retryable_deadline(detail, epoch, sequence, handoff, stream):
    """Only a matching, timing-only NACK plus a confirming controller snapshot.

    No packet is retimed/replayed. The old finite checked tail remains in force;
    a later fresh IK proposal must pass the complete validation pipeline again.
    """
    return bool(detail and epoch is not None and sequence is not None and handoff is not None and
        stream and len(stream) == 60 and
        detail["reasons"] == ["less than 20 ms lead"] and
        detail["packet_epoch"] == detail["controller_epoch"] == epoch == stream[2] and
        detail["packet_sequence"] == sequence and
        detail["packet_parent"] == detail["accepted_sequence"] == sequence-1 == stream[3] and
        abs(detail["handoff_robot_time_s"]-handoff) < 1e-7 and
        stream[1] >= detail["robot_time_s"] and stream[5] == detail["count"] and
        detail["stream_enabled"] and stream[4] and
        not detail["handoff_pending"] and not stream[58] and
        0. <= detail["max_q_mismatch_rad"] <= 1e-9 and
        0. <= detail["max_v_mismatch_rad_s"] <= 1e-8 and
        0. <= detail["max_a_mismatch_rad_s2"] <= 1e-7)
