# FR3 hand teleop controllers

Normal real launch now uses `GuardedRollingController`: robot-clock position commands
at 1 kHz with finite rolling quintics. It inherits activation/interface checks from
the legacy controller, then disables the legacy action/topic inputs. Only the
session-fenced rolling protocol can supply commands.

`~/stream_enabled` is SetBool. Every request creates a new epoch, reported on
`~/rolling_state`. Enabling only succeeds at commanded rest. Commands arrive on
`~/rolling_trajectory` after full-curve collision validation by `feedback_servo`.
JointTrajectory header timestamp is explicitly **absolute robot time**, not ROS wall
time, and frame ID is `rolling1/epoch/sequence/parent`. Both points carry seven q/v/a
values; first point starts at zero relative time and terminal derivatives are zero.
Controller validates complete-curve derivative/travel bounds outside the update loop,
then checks epoch, sequence, future timing and exact command-state splice inside it.
Only one future replacement is pending; the existing curve continues until handoff.

Disarming discards pending extensions. The active already checked finite tail ends
at rest. Old-session, reordered, late and mismatched updates cannot extend it. Invalid
robot clocks refuse updates. The reference tail plus lead is at most 1.49 s under
current bridge planner; physical stopping is unverified and remains independent.

`feedback_servo` still emits fresh measured-seed IK proposals with a 200 ms horizon.
It now additionally validates complete rolling candidates for joint bounds and both
scene/self-collision clearance before forwarding. Python bridge checks the actual
curved path against TCP and whole-arm workspace bounds, including stopping tail.
All curve derivatives and conservative travel retain configured caps. No filter
changes the checked path. Fake streaming application remains unchanged.

`RobotClockTrajectoryController` remains available for legacy bounded-action tests.
Its two-point rest-to-rest acceptance and continuous cancellation are unchanged.
`ClockedMockSystem` remains test-only GenericSystem with synthetic robot clock;
it contains no network or physical Franka driver.

Offline suite now includes seven rolling-controller tests (domain 140), eight legacy
command-continuity tests (136), startup mocks and actual rolling/Servo/bridge integration
on ClockedMockSystem (141). Use `./franka hand-real-check`. Tests cover repeated moving
handoffs, host jitter, biased/lagged measurements, stale/reordered packets, disarm
fencing, full-curve bounds, reversal, tracking loss, collision refusal and final rest.

Current maximum profile: 25° joint variation, 100 mm conservative whole-arm curve
travel, 480 mm/s and 3600 mm/s² Cartesian caps, joint speed 1.0 rad/s and acceleration
2.0 rad/s², and joint jerk 27 rad/s³. The faster profile scales velocity by 3,
acceleration by 9 and jerk by 27, with tracking gain 9/s. C++ collision sampling
uses the complete polynomial's conservative whole-arm speed bound, preserving
at most 1 mm travel per interval. Python planner/validator defaults and actual YAML
match these caps. Workspaces, 0.002 rad feedback discrepancy gate and stopping/epoch
rules remain unchanged. Full 100 mm seven-joint curve has sampled controller coverage.

For each endpoint size the planner tries horizons of 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.9, 1.2 and 1.4 s
in ascending order. It only shrinks the endpoint if every horizon fails. Full
derivative, travel, joint-position, TCP/whole-arm workspace and collision checks
apply to the shorter curves too. Replacements preserve commanded position,
velocity and acceleration, including during reversal. The shorter checked horizon
addresses latency that raising joint caps alone cannot remove. The bridge offers
updates at up to 15 Hz with 90 ms future handoff lead; the controller still requires
at least 20 ms remaining lead after validation. Workspace FK evaluates exactly
the same samples in a batch to reduce validation time. Late replacements are
rejected. Camera U now maps toward +Y, reversing only horizontal travel; camera
depth and height mappings, target scale and the workspace are unchanged.

Arm state now comes from `joint_state_broadcaster/joint_states`, with
`use_local_topics: true`. The Hand driver publishes separately on
`gripper_joint_states`. A dedicated Servo callback group receives both and
publishes complete nine-joint snapshots on `joint_states` at 100 Hz, independently
of full-curve collision validation. A fast arm message can no longer overwrite a
finger-only message in MoveIt's latest-message queue. Each snapshot keeps the
oldest original source timestamp; cached samples never acquire a new timestamp.
Raw arm and Hand inputs still expire at 100 ms and 150 ms, respectively. At a
sample transition, Servo may wait up to 20 ms for MoveIt's asynchronous state
monitor to consume a newly received complete snapshot, then rechecks both source
deadlines before using it. Failed/stale inputs still block/disarm the bridge.

The pinned Hand driver also returns without publishing if readOnce throws; it
does not publish an old width with a new timestamp. `joint_feedback_tests` checks
source freshness, complete snapshots and invalid samples without ROS hardware;
`gripper_feedback_tests` compiles the actual publisher callback with an explicitly
renamed fake Gripper implementation and verifies read-failure behavior with zero
motion calls. Gripper motion remains a separate, disabled profile option.

### Rejection diagnostics

The rolling controller retains all admission predicates and publishes a separate
fixed 16-value `~/rolling_rejections` diagnostic through a realtime publisher.
Fields are schema=1, cumulative rejection count, robot time, reason bitmask,
packet epoch/sequence/parent, controller epoch/accepted sequence, remaining lead
in seconds, maximum q/v/a mismatches, pending/enabled flags and handoff robot time.
Reason bits: 1 disabled, 2 pending, 4 epoch, 8 parent, 16 sequence, 32 late
(less than 20 ms lead), 64 too far ahead (over 300 ms), 128 continuity mismatch.
The main 60-value rolling state is unchanged. Normal RT control does not format
diagnostic strings; the Python bridge records the additional topic. Input-schema
and derivative/travel/horizon refusals are logged from the non-RT receive callback.
Fatal robot-clock messages now include current/previous robot time and the gap;
the existing invalid/repeated/20 ms rejection remains unchanged.

### Timing-only rejection recovery

Full-curve validation keeps the same <=1 mm sampling and >=1 mm world/self
clearance. Distance queries are restricted to the required 1 mm clearance instead
of calculating exact distances to distant geometry; all links, attached bodies,
world objects and existing allowed-collision pairs are retained. Servo logs
`ROLLING VALIDATION` with sequence, sample count and elapsed milliseconds.

A controller NACK containing only the existing <20 ms lead condition can release
the bridge's pending update after a fresh controller snapshot confirms the same
epoch, unchanged parent sequence, enabled stream and no pending handoff. The
rejected handoff timestamp must match the outstanding candidate. The checked
finite tail remains active; only a new, fresh proposal passing all checks can
extend it. The rejected packet is never retimed or replayed. Other rejections,
missing acknowledgements, stale feedback and tracking loss retain their stop
behavior. This does not automatically arm a disarmed session.
