"""Grasp/lift sequencing, independent of ROS and physical hardware."""
import math

from .core import finite, vector


class GraspGuard:
    LIFT = .05
    MARGIN = .01
    CLOSE = .25
    OPEN = .8

    def __init__(self):
        self.phase = "normal"
        self.anchor = None
        self.offset = (0., 0., 0.)
        self.floor = None
        self.peak_z = None
        self.started = None
        self.settled_since = None
        self.opening_requested = False
        self.command_z_offset = 0.
        self.lift_base_z = None
        self.last_height_rejection = None

    @property
    def holding(self):
        return self.phase in ("settling", "closing", "interrupted")

    def begin(self, actual, now, ceiling):
        if not vector(actual) or actual[2]+self.LIFT > ceiling:
            raise ValueError("Grasp needs 5 cm of lift headroom inside the workspace")
        self.anchor = tuple(actual)
        self.phase, self.started, self.settled_since = "settling", now, None

    def settled(self, actual, now, stationary, tail_finished):
        if self.phase != "settling":
            return False
        if now-self.started > 3.:
            raise ValueError("Arm did not settle before grasp; closure blocked")
        if not stationary or not tail_finished:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = now
        if now-self.settled_since < .10:
            return False
        # Capture immediately before physical closure, after the finite tail.
        self.anchor, self.phase = tuple(actual), "closing"
        return True

    def confirmed(self, actual, raw_target, command_z=None):
        if self.phase != "closing" or not vector(actual) or not vector(raw_target):
            raise ValueError("Unexpected grasp confirmation")
        self.offset = tuple(a-b for a, b in zip(actual, raw_target))
        command_z = actual[2] if command_z is None else command_z
        if not finite(command_z):
            raise ValueError("Grasp confirmation needs a finite held command height")
        # Stationary impedance control need not have q_measured=q_desired.
        # Compare future command displacement with the measured hold reference.
        self.command_z_offset = actual[2]-command_z
        self.lift_base_z = actual[2]
        self.floor = self.anchor[2]+self.MARGIN
        self.peak_z = max(actual[2], self.anchor[2])
        self.phase = "lift"

    def interrupt(self):
        self.opening_requested = False
        if self.phase in ("settling", "closing"):
            self.phase = "interrupted"

    def target(self, raw, actual, limits):
        if self.holding:
            return self.anchor
        if self.phase == "lift":
            self.peak_z = max(self.peak_z, actual[2])
            if actual[2] >= self.anchor[2]+self.LIFT:
                self.phase = "carry"
        target = tuple(max(low+.001, min(high-.001, a+b)) for a, b, low, high in
                       zip(raw, self.offset, limits.minimum, limits.maximum))
        if self.phase == "lift":
            target = (*target[:2], max(self.peak_z, target[2]))
        elif self.floor is not None:
            target = (*target[:2], max(self.floor, target[2]))
        return target

    def velocity(self, velocity):
        if self.holding:
            return (0., 0., 0.)
        if self.phase == "lift":
            return (*velocity[:2], max(0., velocity[2]))
        return velocity

    def released(self, actual, raw_target):
        # Re-clutch on opening too: changing the floor cannot cause a jump.
        self.offset = tuple(a-b for a, b in zip(actual, raw_target))
        self.phase, self.floor, self.peak_z = "normal", None, None
        self.anchor, self.opening_requested = None, False
        self.command_z_offset, self.lift_base_z = 0., None

    def floor_for_path(self):
        if self.phase == "lift":
            return self.lift_base_z
        return self.floor

    def accepts_heights(self, heights):
        """Reject descending lift paths as well as paths below the carry floor.

        50 micrometres accommodates IK/FK rounding; this is an application
        constraint, not a certified Cartesian safety function.
        """
        self.last_height_rejection = None
        values = [v+self.command_z_offset for v in heights]
        if not values or not all(math.isfinite(v) for v in values):
            self.last_height_rejection = "Nonfinite/empty commanded TCP heights"
            return False
        floor = self.floor_for_path()
        if floor is not None and min(values) < floor-.00005:
            self.last_height_rejection = (f"Aligned command Z {min(values)*1000:.3f} mm below "
                                          f"grasp floor {floor*1000:.3f} mm")
            return False
        if self.phase == "lift":
            peak = values[0]
            for value in values:
                if value < peak-.00005:
                    self.last_height_rejection = "Commanded lift curve descends by more than 0.05 mm"
                    return False
                peak = max(peak, value)
        return True

    def status(self):
        if self.phase == "settling":
            return "GRASP: waiting for arm to settle before closure"
        if self.phase == "closing":
            return "GRASP: holding XYZ; waiting for grasp confirmation"
        if self.phase == "interrupted":
            return "GRASP interrupted: open hand fully to release the hold"
        if self.phase == "lift":
            remaining = max(0., self.anchor[2]+self.LIFT-self.peak_z)
            return f"LIFT: up only; {remaining*100:.1f} cm remaining; X/Y active"
        if self.phase == "carry":
            return f"CARRY: Z floor {self.floor*100:.1f} cm; open hand fully to release"
        return ""


class GripperCommands:
    """Debounced intent and monotonic opening goals across delayed Hand feedback."""
    CLOSE_CONFIRM = .06
    OPEN_CONFIRM = .12
    WIDTH_DEADBAND = .002
    STEP_TIME = .10

    def __init__(self):
        self.close_since = self.open_since = None
        self.close_ready = self.open_ready = False
        self.reset_moves()

    def reset_moves(self):
        self.last_width = None
        self.feedback_after = -math.inf

    def observe(self, opening, now):
        if opening <= GraspGuard.CLOSE:
            if self.close_since is None:
                self.close_since = now
            if now-self.close_since >= self.CLOSE_CONFIRM:
                self.close_ready, self.open_ready = True, False
        else:
            self.close_since = None
        if opening >= GraspGuard.OPEN:
            if self.open_since is None:
                self.open_since = now
            if now-self.open_since >= self.OPEN_CONFIRM:
                self.open_ready, self.close_ready = True, False
        else:
            self.open_since = None

    def opening_goal(self, desired, measured, speed, feedback_stamp):
        if feedback_stamp <= self.feedback_after:
            return None  # Do not reuse a snapshot predating the last action result.
        baseline = max(measured, self.last_width if self.last_width is not None else measured)
        if desired-baseline < self.WIDTH_DEADBAND and desired < .0795:
            return None
        width = min(.08, desired, measured+speed*self.STEP_TIME)
        # A stale width must never reissue the same goal or reverse an opening.
        if width <= measured+.0005 or (self.last_width is not None and width <= self.last_width+.0005):
            return None
        return width

    def sent(self, width):
        self.last_width = width

    def finished(self, now):
        self.feedback_after = now
