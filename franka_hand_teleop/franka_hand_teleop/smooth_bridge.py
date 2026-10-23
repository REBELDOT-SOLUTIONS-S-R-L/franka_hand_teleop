"""Session-fenced rolling references; physical transport only through checked Servo paths."""
import json
import math
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .deployment import tracking_fresh
from .diagnostics import rejection_detail
from .real_bridge import RealBridge, duration, seconds
from .rolling import (
    Quintic,
    State,
    checked_curve,
    retryable_deadline,
    tracking_endpoint,
)


class SmoothBridge(RealBridge):
    MOTION_MODE = "rolling"
    EXPECTED_CONTROLLER = "franka_hand_controllers/GuardedRollingController"
    def __init__(self, **kwargs):
        self.stream = None
        self.stream_stamp = -math.inf
        self.stream_epoch = None
        self.enable_future = None
        self.pending_sequence = None
        self.pending_handoff = None
        self.sent_at = self.last_update = -math.inf
        self.stream_rejections = 0
        self.stale_proposals = 0
        self.handoff_budget_skips = 0
        self.deadline_retries = 0
        self.stream_start_tcp = None
        self.last_rejection = None
        super().__init__(**kwargs)
        timing = self.profile["bridge"]
        for attribute, key in (("UPDATE_PERIOD_S", "update_period_s"),
                ("HANDOFF_LEAD_S", "handoff_lead_s"), ("MIN_PUBLISH_LEAD_S", "min_publish_lead_s"),
                ("PLANNING_BUDGET_S", "planning_budget_s"), ("MIN_CURVE_DURATION_S", "min_curve_duration_s")):
            setattr(self, attribute, timing[key])
        self.stream_enable = self.create_client(SetBool, "fr3_arm_controller/stream_enabled")
        self.candidate = self.create_publisher(JointTrajectory, "servo_node/rolling_candidate", 1)
        self.create_subscription(Float64MultiArray, "fr3_arm_controller/rolling_state",
                                 self.receive_stream, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "fr3_arm_controller/rolling_rejections",
                                 self.receive_rejection, 10)

    def receive_rejection(self, message):
        detail = rejection_detail(list(message.data))
        if detail is not None and detail != self.last_rejection:
            self.last_rejection = detail
            self.get_logger().warning("ROLLING REJECTION: "+json.dumps(detail, allow_nan=False))
            self.retry_deadline()

    def retry_deadline(self):
        now = time.monotonic()
        if (self.permit.armed and now-self.stream_stamp < .10 and
                retryable_deadline(self.last_rejection, self.stream_epoch, self.pending_sequence,
                                   self.pending_handoff, self.stream)):
            self.pending_sequence = self.pending_handoff = None
            self.deadline_retries += 1
            self.get_logger().warning(
                "ROLLING WAIT: controller confirmed timing-only rejection; checked finite tail retained; "
                f"waiting for fresh validated proposal; deadline retries={self.deadline_retries}")

    def readiness(self, now):
        ready, reason = super().readiness(now)
        if ready and self.permit.armed and any(abs(self.desired[n]-self.joints[n]) >= .002
                                              for n in self.guard.names):
            return False, "Desired/measured rolling joints differ by >=0.002 rad"
        return ready, reason

    def arm_server_ready(self):
        return self.stream_enable.service_is_ready()

    def arm_tail_finished(self):
        d = self.stream
        return bool(d and time.monotonic()-self.stream_stamp < .1 and
                    self.pending_sequence is None and not d[58] and d[1] >= d[59] and
                    max(map(abs, d[44:58])) < 1e-8)

    def receive_stream(self, message):
        d = list(message.data)
        if len(d) != 60 or not all(math.isfinite(v) for v in d) or d[0] != 1:
            self.stop("Invalid rolling controller state")
            return
        if self.stream and d[1] <= self.stream[1]:
            return
        self.stream, self.stream_stamp = d, time.monotonic()
        if abs(d[8]-self.guard.reach) > 1e-8:
            self.stop("Rolling controller and URDF reach bounds disagree")
        if self.pending_sequence is not None and d[2] == self.stream_epoch and d[3] == self.pending_sequence:
            self.pending_sequence = self.pending_handoff = None
            self.arm_actions_completed += 1
            if self.stream_start_tcp:
                actual = self.guard.tcp(self.joints)
                self.last_motion_result = {"success": True,
                    "tcp_displacement_m": math.dist(actual, self.stream_start_tcp), "target_progress_m": 0.}
            self.stream_start_tcp = self.guard.tcp(self.joints)
        if self.stream_epoch is not None and ((d[2] > self.stream_epoch or
                (d[2] < self.stream_epoch and time.monotonic()-self.sent_at > .3)) or
                (d[2] == self.stream_epoch and not d[4])):
            self.stop("Rolling session disabled or changed")
        self.retry_deadline()

    def stop(self, reason, report=False):
        if self.stream_epoch is not None or self.enable_future is not None:
            self.arm_not_before = max(getattr(self, "arm_not_before", -math.inf), time.monotonic()+1.6)
            if hasattr(self, "stream_enable") and self.stream_enable.service_is_ready():
                self.stream_enable.call_async(SetBool.Request(data=False))
        self.stream_epoch = None
        self.enable_future = None
        self.pending_sequence = self.pending_handoff = None
        super().stop(reason, report)

    def tick(self):
        super().tick()
        now = time.monotonic()
        if not self.permit.armed:
            return
        if self.enable_future is not None:
            if now-self.sent_at > .5:
                self.stop("Rolling session enable timeout")
            elif self.enable_future.done():
                answer = self.enable_future.result()
                self.enable_future = None
                if answer and answer.success:
                    self.stream_epoch = int(answer.message)
                    self.sent_at = now
                else:
                    self.stop("Rolling session enable rejected")
            return
        if self.stream_epoch is None:
            self.enable_future = self.stream_enable.call_async(SetBool.Request(data=True))
            self.sent_at = now
            return
        if not self.stream or now-self.stream_stamp > .15:
            if now-self.sent_at > .5:
                self.stop("Rolling command feedback stale")
            return
        if self.pending_sequence is not None and now-self.sent_at > .4:
            detail = self.last_rejection
            rejection = (", ".join(detail["reasons"])+f"; lead={detail['remaining_lead_ms']:.1f} ms"
                         if detail and detail["controller_epoch"] == self.stream_epoch else "not recorded")
            self.stop("Rolling path validation/acknowledgement timeout; "
                      f"controller rejections={int(self.stream[5])}; reason={rejection}")

    def receive_trajectory(self, message):
        now = time.monotonic()
        d = self.stream
        if (not self.execute or not self.permit.armed or self.stream_epoch is None or
                self.enable_future is not None or self.pending_sequence is not None or
                (self.grasp and self.grasp.holding) or seconds(message.header.stamp) <= self.motion_fence or
                not d or d[2] != self.stream_epoch or not d[4] or d[58] or
                now-self.stream_stamp > .10 or now-self.last_update < self.UPDATE_PERIOD_S):
            return
        ready, reason = self.readiness(now)
        if not ready or not tracking_fresh(self.logic.tracking, now):
            self.stop(reason if not ready else "Tracking expired before rolling update")
            return
        try:
            age = self.get_clock().now().nanoseconds*1e-9-seconds(message.header.stamp)
            if (not -.02 <= age <= .10 or message.header.frame_id != "fr3_link0" or
                    message.joint_names != self.guard.names or len(message.points) != 2 or
                    any(len(p.positions) != 7 for p in message.points) or
                    seconds(message.points[0].time_from_start) != 0 or
                    not .02 <= seconds(message.points[1].time_from_start) <= .5):
                raise ValueError("Invalid/stale seeded rolling proposal")
            seed, proposed = [list(p.positions) for p in message.points]
            if not all(math.isfinite(v) for values in (seed, proposed) for v in values):
                raise ValueError("Nonfinite rolling proposal seed/endpoint")
            seed_error = max(abs(q-self.joints[n]) for n, q in zip(self.guard.names, seed))
            if seed_error > .002:
                # IK and measured feedback are asynchronous. Never execute an
                # out-of-date proposal; the previous checked finite tail remains.
                self.stale_proposals += 1
                self.last_update = now
                if self.stale_proposals % 5 == 1:
                    self.get_logger().warning(
                        f"ROLLING WAIT: stale IK seed ({seed_error*1000:.2f} mrad > 2 mrad); "
                        f"checked tail retained; skipped={self.stale_proposals}")
                return
            if any(not self.guard.bounds[n][0] <= q <= self.guard.bounds[n][1]
                   for n, q in zip(self.guard.names, proposed)):
                raise ValueError("Joint limit in rolling proposal")
            handoff = round((d[1]+self.HANDOFF_LEAD_S)*1000)/1000
            if d[3] > 0:
                previous = Quintic(State(tuple(d[9:16]), tuple(d[16:23]), tuple(d[23:30])),
                                   tuple(d[30:37]), d[7])
                state = previous.at(handoff-d[6])
            else:
                state = State(tuple(d[37:44]), (0.,)*7, (0.,)*7)
            target = self.motion_target(self.guard.tcp(self.joints))
            if self.grasp and self.grasp.floor_for_path() is not None:
                target = (*target[:2], target[2]-self.grasp.command_z_offset)
            endpoint = tracking_endpoint(seed, proposed, [self.desired[n] for n in self.guard.names],
                                         target, self.guard)
            if self.grasp:
                self.grasp.last_height_rejection = None
            curve = checked_curve(state, endpoint, self.motion, self.guard, self.grasp,
                                  self.PLANNING_BUDGET_S, self.MIN_CURVE_DURATION_S)
            if curve is None:
                self.stream_rejections += 1
                if self.stream_rejections % 5 == 1:
                    self.get_logger().warning("ROLLING WAIT: no checked update within planning budget; existing tail retained; "
                                              f"rejected updates={self.stream_rejections}; "
                                              f"planning={(time.monotonic()-now)*1000:.1f} ms; "
                                              f"height check={self.grasp.last_height_rejection if self.grasp else None}")
                self.last_update = time.monotonic()
                return  # Existing checked finite tail remains valid; no command discontinuity.
            remaining_lead = handoff-d[1]-(time.monotonic()-self.stream_stamp)
            if remaining_lead < self.MIN_PUBLISH_LEAD_S:
                self.stream_rejections += 1
                self.handoff_budget_skips += 1
                self.last_update = time.monotonic()
                self.get_logger().warning(
                    f"ROLLING WAIT: planning left only {remaining_lead*1000:.1f} ms handoff lead; "
                    "existing checked tail retained; fresh update required")
                return
            request = JointTrajectory(joint_names=self.guard.names)
            request.header.frame_id = f"rolling1/{self.stream_epoch}/{int(d[3])+1}/{int(d[3])}"
            stamp = duration(handoff)
            request.header.stamp = Time(sec=stamp.sec, nanosec=stamp.nanosec)  # Absolute robot time.
            request.points = [
                JointTrajectoryPoint(positions=list(state.q), velocities=list(state.v), accelerations=list(state.a)),
                JointTrajectoryPoint(positions=list(curve.at(curve.duration).q), velocities=[0.]*7,
                                     accelerations=[0.]*7, time_from_start=duration(curve.duration))]
            self.pending_sequence = int(d[3])+1
            self.pending_handoff = handoff
            self.sent_at = self.last_update = time.monotonic()
            if self.stream_start_tcp is None:
                self.stream_start_tcp = self.guard.tcp(self.joints)
            self.last_segment = {"mode": "rolling", "sequence": self.pending_sequence,
                "duration_s": curve.duration, "proposal_horizon_s": self.profile["bridge"]["proposal_horizon_s"],
                "planning_ms": (self.sent_at-now)*1000,
                "command_delta_q": [b-a for a, b in zip(state.q, curve.at(curve.duration).q)]}
            self.get_logger().info("ROLLING PLAN: "+json.dumps({**self.last_segment,
                "handoff_robot_time_s": handoff, "initial_q": list(state.q),
                "initial_v": list(state.v), "initial_a": list(state.a),
                "target_q": list(curve.at(curve.duration).q)}))
            self.candidate.publish(request)
        except (ValueError, IndexError, TypeError) as error:
            self.guard_rejections += 1
            self.stop(str(error))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SmoothBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
