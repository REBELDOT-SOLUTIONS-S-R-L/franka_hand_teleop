# Real FR3 commissioning

The real deployment path is prepared and tested offline. The local profile now
records all three installation checks as completed and approved, as explicitly
confirmed by the user and Safety Operator on 2026-10-20. This is a record of their
confirmation, not independent physical verification by the agent. Motion still
requires `--execute` and explicit arming. Calibrated thumb/pinky gestures now
sequence a held grasp, upward lift, and a grasp-relative Z floor while armed.
No brakes, FCI setting, Watchman rule, tool/load parameter or physical robot
command was changed by the agent. The camera stays in `/home/roboticslab/hand-tracking`.

## Current motion settings: one file

On the migrated laptop, edit only
`/home/rebelfranka/workspace/franka/ws/src/franka_hand_teleop/franka_hand_teleop/config/real_profile.yaml`.
This is the source for following gain, TCP and joint speed/acceleration/jerk,
travel bounds, maximum curve duration, gripper speed and bridge timing. Historical
sections below describe earlier implementations; their duplicated C++ tuning
constants and current-value validation caps have been superseded.

For a 3 m/s TCP ceiling, change the existing `motion.max_speed` line to:

```yaml
  max_speed: 3.0
```

Keep the other fields in the file. Validate without starting hardware:

```bash
./franka exec python3 -m franka_hand_teleop.deployment /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/config/real_profile.yaml
```

Restart the app normally to apply changes. After the one-time refactor build,
configuration edits require no source edits or recompilation. The active profile
still has speed 1.44 m/s, acceleration 8.1 m/s² and gripper speed 0.040 m/s; the
3 m/s / shorter-timing profile under `logs/` is a separate mock-only test artifact.

Launch validates the file before constructing hardware nodes and freezes one
session snapshot. The bridge, Servo and controller all consume that same snapshot.
Editing the source during a session has no live effect. Missing C++ motion
settings fail closed instead of falling back to hidden defaults. Python and C++
retain the fixed FR3 envelope (TCP 3 m/s, 9 m/s²; scalar joint speed <=2.62 rad/s,
acceleration <=10 rad/s² and jerk <=5000 rad/s³). Installed-model, position-dependent,
collision, workspace and robot-side protection can constrain actual motion sooner.
[Manufacturer limits](https://frankarobotics.github.io/docs/robot_specifications.html).

`smooth_bridge` generates finite quintic replacements with matching position,
velocity and acceleration at each handoff. The compiled controller evaluates them
on the robot clock at 1 kHz. When new targets stop, the accepted finite tail ends
at rest. This continuity/stopping function remains; it no longer owns tuning values.

The 200 ms minimum is an application timing choice, not a Franka requirement.
With the current 150 ms future handoff it leaves 50 ms of overlap. A curve may be
replaced before it finishes; the minimum does not insert a 200 ms pause between
movements. These related settings are now in the profile's `bridge` section:

```yaml
bridge:
  update_period_s: 0.06666666666666667
  proposal_horizon_s: 0.20
  handoff_lead_s: 0.15
  min_publish_lead_s: 0.10
  planning_budget_s: 0.02
  min_curve_duration_s: 0.20
```

Validation requires minimum duration >= handoff lead + 20 ms, handoff lead >=
planning budget + publish lead + 20 ms, and an update period shorter than the
minimum duration minus 20 ms. The stream retains its 20–300 ms packet-lead window,
50 ms minimum polynomial duration and <=1.5 s finite-tail protocol bound. These
are protocol constraints rather than extra copies of user tuning. Raising a
speed ceiling does not guarantee that speed within the current workspace or
under the other active derivative/travel constraints.

The refactor passed 73 Python tests, 10 compiled tests and full isolated mock
pipelines using both the default profile and a separate 3 m/s ceiling / 180 ms
minimum-curve profile. The shipped timing remains unchanged. A 100 ms handoff /
55 ms publish-reserve experiment was stopped for insufficient controller lead;
changing timing requires full mock validation, not just passing the profile check.
No physical robot connection or commands were made for this work.

## X4 and the installation

The originally requested X4 behavior is: released → stop, middle → permission,
fully pressed → stop. X4 is now deferred; the application does not mandate that
particular device. Physical motion still requires a validated robot-side safety
configuration suitable for the actual test setup, including independent stopping
and any required enabling/protective devices. A Python enable latch does not
provide that safety function. Deferring X4 changes no robot-side safety setting.
The installed libfranka state message does not expose a dedicated X4 state field;
the application reports `x4_readback_available: false`.

The user initially reported no safety scenario configured, and later confirmed
that the installation checks were completed and approved with the Safety Operator.
For initial setup or any revision, leave the robot locked while a
Safety Operator checks the installed configuration. Export the committed Watchman
settings before editing. FCI operates in Execution mode, so verify the behavior
in **Work**, rather than relying on the Programming/Test & Jog behavior.
The 5.10 manual says X4 activates SMSS on release/full press, with the Execution
reaction dependent on the Work scenario. However, its editable rule activation
choices list Always, X3.2 and X3.3, without X4. An exact X4 setup must therefore
be confirmed in the installed UI or with Franka support; do not substitute an X3
input or remove existing rules to make motion work.

The Safety Operator must configure and validate the applicable safety functions,
including the actual stop response, stopping distances, emergency stop and
end-effector behavior. Configuration validation includes controlled physical tests;
an input indicator alone is insufficient. See sections 11 and 12 of the
[system-image 5.10 operating manual](https://franka.de/hubfs/Operating%20Manual%20Franka%20Research%203_5.10_R02216_1.3_EN.pdf?hsLang=en).
The manual's restriction on FCI with Cartesian safety monitoring applies to
system images below 5.10. Verify the actual installed version and functions.

## Preview and offline inspection

X4 commissioning can remain pending while testing connection, feedback and camera
mapping in preview. It does not enable motion. A smaller software box alone does
not establish the robot-side stop/enabling arrangement needed for physical motion;
any unverified motion prerequisites must remain false. The current local profile
records the user's subsequent confirmation of completed installation checks.

These commands do not connect to the robot:

```bash
./franka hand-real --robot-ip 192.0.2.1 --dry-run
./franka hand-real-check
```

`192.0.2.1` is a documentation-only address for the dry run. The dry run validates
the profile and prints commands without starting the camera, ROS or hardware.
The offline check uses synthetic services/actions and GenericSystem, never a
physical hardware driver. It does not validate physical stopping performance.

After confirming the current C2 address and FCI availability, preview is:

```bash
./franka hand-real --robot-ip YOUR_C2_IP
```

This command **connects to real hardware** for state feedback and starts the D435,
Servo and RViz. The arm controller is configured **inactive**. The bridge creates
no arm or gripper action clients in preview; requests to arm are refused. Servo
can process internal zero commands, but its output has no route to the controller
through the preview bridge. RViz displays `REAL:PREVIEW`, actual TCP and the
mapped camera target. The real gripper driver reads state without homing or moving.

FCI connection may require preparation in Desk even for feedback. Do not unlock
or change operating mode merely to make this command succeed. Only one arm-driver
stack may own the physical robot at a time; stop generic bringup/MoveIt first.

For the first connected preview:

1. Confirm the actual C2 address in Desk. The last recorded value was
   `10.42.0.55`; it is not assumed to be current. Keep the direct C2 Ethernet link.
2. Check that the FCI feature/mode is available. First try the read-only probe:
   `./franka exec /workspace/examples/libfranka/build/read_state YOUR_C2_IP`.
   It reads one state without motion, homing, recovery or configuration calls.
   If FCI is refused, resolve that through the documented Desk preparation; do
   not bypass a robot-side safety restriction to obtain a connection.
3. Start the preview command above. Confirm RViz says `REAL:PREVIEW`, displays the
   actual arm pose and shows the reduced tool box. Check reported TCP coordinates
   against the installation and verify that camera motion moves only the target.
   Use C/O for finger calibration and B for camera-range calibration if needed.
4. A parked TCP outside the new box may still be viewed in preview; arming will
   remain unavailable. Positioning near [0.55, 0.0, 0.15] m is a later, separate
   commissioned positioning procedure, not an automatic motion by this app.
5. Ctrl+C ends the session. Review `logs/hand-real.log` for any readiness error.

If RViz disappears immediately and the log reports `Connection to FCI refused`,
the required hardware connection failed and ROS launch shut down RViz. This is
not an EGL/DRI3 rendering failure. The host launcher reports the connection error
in the terminal and exits unsuccessfully, even when ROS launch itself exits zero.
The 5.10 documented activation path is Desk → System menu → Execution mode →
Activate FCI. Its general FCI operating prerequisites also include open brakes
and an error-free safety system. Therefore a live preview cannot be promised
with brakes locked in every robot state. If Desk requires further preparation,
complete the approved commissioning procedure rather than unlocking solely to
obtain a window. The software preview still sends no arm/gripper motion commands.

## Commissioning profile and motion

Make an installation-specific copy of `config/real_profile.yaml` inside this
workspace. The `commissioning` booleans record verification of the robot's overall
safety configuration (`robot_safety_verified`), workspace, and tool/load. X4 itself
is not a required app acknowledgement. They are administrative startup checks, not
sensors or safety functions. Keep them false until those installation checks
have actually been completed by the responsible people.

Verify the X/Y origin at the centre of joint 1 and Z=0 at the mounting plane.
The measured tool box is X 0.40–0.70 m, Y −0.35–0.35 m, Z 0–0.30 m. The operating
box is 50 mm inward from every face: X 0.45–0.65 m, Y −0.30–0.30 m, Z 0.05–0.25 m.
Camera targets are a further 20 mm inward: X 0.47–0.63 m, Y −0.28–0.28 m,
Z 0.07–0.23 m. Extra clearance is not a commissioned stopping-distance guarantee.
The box constrains `fr3_hand_tcp`; it does not describe the outer
surface of the gripper. The separate whole-arm box remains an example and must
be checked against the actual cell. Robot-side monitored points/spheres do not
necessarily represent the same geometry as the app's collision boxes.

This version models the standard Franka Hand without a held object and a flat
2 m square table whose top is Z=0. Confirm the actual mounting/tool/TCP, configured
mass, centre of mass and inertia. Other tools, payload geometry, fixtures or
obstacles need matching model/scene changes before use. Camera depth is used for
hand tracking; it does not provide a collision map of the cell.

The bridge compares the FCI-reported EE pose with the URDF TCP (5 mm position,
0.03 rad orientation tolerance). A mismatch blocks arming; it does not silently
change the robot's TCP setting. No application call changes `setEE`, `setLoad`,
collision thresholds, brakes or Watchman.

Only after installation verification, the motion-capable command is:

```bash
./franka hand-real --robot-ip YOUR_C2_IP --profile PATH_INSIDE_WORKSPACE --execute
```

This starts **disarmed**. Align the camera target within 30 mm of the actual TCP,
obtain at least 0.3 seconds of valid D435 tracking and stationary robot feedback,
then click **ARM** in the camera window. **DISARM** or **D** in that window
requests a stop. Terminal **a + Enter** / **d + Enter** remain available as a
fallback. Arming activates the arm
controller from its measured state; there is no automatic move to the mock start
pose. The current robot pose must already satisfy the workspace and clearance.
Initial positioning is a separate commissioned procedure.

The real arm controller type is now
`franka_hand_controllers/GuardedRollingController`, built from the sibling
`ws/src/franka_hand_teleop/franka_hand_controllers` package. It extends the installed JTC and uses
the hardware's `fr3/robot_time` to advance interpolation. Incoming bridge goals
use a zero header stamp and relative waypoint times. Missing/invalid/repeated or
reversed robot timestamps, gaps of 20 ms or more, and nonzero goal header stamps
refuse controller updates. The bridge verifies the exact controller type before
arming, so a previously loaded stock controller does not silently qualify.

This addresses a reproduced command-sampling failure mode: host-period jitter can
fast-forward an otherwise smooth trajectory and exceed Franka acceleration/jerk
limits even when packets are delivered. A 5 ms injected host-clock step on a small
quintic generated approximately 15 rad/s² acceleration and 30,000 rad/s³ jerk in
the stock JTC; the robot-clock version remained below the app limits across twenty
consecutive segment reversals. The saved physical log did not include per-cycle
commands, so this does not establish the precise cause of that physical reflex.
The robot-clock controller preserves the geometric path and existing cancellation
behavior; no independent joint smoothing or raised limit masks discontinuities.
Offline tests do not validate physical performance. See the sibling
[controller README](../franka_hand_controllers/README.md).

Real mode uses `franka_hand_controllers/feedback_servo`, a MoveIt Servo C++ API
node that starts each IK proposal from fresh measured joints. The standard
streaming ServoNode accumulates a forecast assuming its output is executed; the
bounded-action bridge instead shortens/discards proposals while an action runs.
Combining those behaviors let predictions run far ahead of the actual tool and
trigger the workspace guard. Real mode now publishes one measured-state proposal
per cycle without a rolling forecast. Each proposal carries its exact measured
joint seed and a 200 ms endpoint. Real mode rebases that IK increment onto current
FCI desired joints to form a target; its new rolling bridge builds a future curve
from the last accepted commanded position, velocity and acceleration. It does not
integrate IK from a forecast that may never execute. Stationary verification remains
required for initial arming. Position filtering stays disabled in real Servo because
rolling quintics supply command smoothing; fake streaming keeps its existing filter.

The real launcher uses `smooth_bridge` and `GuardedRollingController`. Rolling targets
update approximately every 200 ms; commands interpolate from robot time at 1 kHz.
Status labels accepted rolling updates separately from completed legacy actions.
`ROLLING PLAN` log records include command state, target, splice time and sequence.
`ready=yes` only means prerequisites are available; explicit arming remains required.

Real-mode readiness checks use separate feedback deadlines: 100 ms for arm joints
and 150 ms for Hand joints. The Hand driver's configured 30 Hz timer calls a
blocking hardware read, so it does not guarantee a sample every 33 ms. A read-only
probe of this installation measured approximately 116 ms intervals. Stale Hand
feedback still blocks/disarms the arm, and collision checking remains active.
If readiness fails, the console now prints the Servo status code and reason,
including stale joint names, instead of only “Servo unavailable or halted”.

Hand feedback is always required, including when `enable_gripper` is set to false; enabling movement
does not fix missing measurements. Real launch now keeps arm and Hand inputs on
separate topics and combines them into complete joint-state snapshots for MoveIt
and RViz. Input reception runs independently of curve collision validation. The
snapshot timestamp is the oldest actual source timestamp, never the publication
time. The 100/150 ms source deadlines are unchanged; actual input loss still
disarms. The Hand driver does not publish after a failed hardware read. Retest in
preview before a physical motion test after rebuilding these packages.

The camera target is the robot position calculated from the hand's image position
and depth; the hand itself stays in the camera's view, away from the robot.
In RViz, **blue TCP** is the measured tool position and **yellow TARGET** is the
mapped hand position. Move the hand while disarmed to bring TARGET near TCP.
Their connecting line turns green when the straight-line gap is at most 3 cm;
the gap label and terminal `ALIGNMENT` messages show the distance. For example,
TCP (55, 0, 15) cm and TARGET (60, 0, 15) cm are 5 cm apart, so arming is refused.
Wait for `ready=yes` and gap <= 3 cm, then request arming again. Reaching that
distance never automatically arms. Arm/disarm responses now appear in the
launching terminal as well as the log.

RViz loads Move Camera, Select and Focus Camera tools plus the Views panel.
Select Move Camera (shortcut M with RViz focused): left-drag rotates, middle-drag
or Shift+left-drag pans, and the mouse wheel zooms. Apparent marker overlap from
one viewpoint does not establish a small three-dimensional gap.

With the default camera region (U/V 5–95%, hand depth 0.35–1.00 m), a hand at
0.50 m maps to robot X about 0.593 m. Screen position controls Y and Z, not X;
moving the hand away from the camera decreases robot X. At the measured parked
TCP (0.546, -0.013, 0.075) m, the exact inverse default mapping is approximately
48% from the mirrored preview's left edge, 92% from its top, and hand depth
0.691 m. Holding the hand slightly higher (for example 85% down) leaves the target
within the capture tolerance if X/Y match. Keep the whole hand visible and use
the numerical gap, not apparent overlap, to judge alignment. These numbers depend
on the actual TCP and camera calibration and must not be treated as universal.
The tracked palm depth, rather than the person's body distance, sets X.

For a different comfortable hand range, explicitly disarm, press B in the camera
window, sweep the desired left/right/top/bottom and near/far extent, then B again
to save. Include at least 10 cm of depth and keep the whole hand visible. Target
X/Y/Z midpoints correspond to the midpoints of the calibrated depth/image ranges.
C/O only calibrates the finger opening and does not change this spatial mapping.

If alignment is within range but hardware verification fails, inspect the first
`ROBOT ERROR` rather than repeatedly requesting arm. In the recorded first
physical attempt the controller activated and accepted a goal, then FCI rejected
startup with `joint_position_motion_generator_start_pose_invalid`; ROS deactivated
the hardware. The later verification rejection was a consequence of that fault.
The bridge now reports the returned hardware identity/state and distinguishes it
from a stale service response. Driver ERROR/FATAL messages appear in the terminal.

The local `patches/teleop-position-startup.patch` changes joint position startup:
the hardware reads the newly started control session and seeds command interfaces
from FCI `q_d` before returning from the mode switch, so the trajectory controller
captures the correct initial hold. Measured `q` may differ slightly and is not used
as that initial command. A failed/non-finite initial read refuses the switch and
stops the just-started interface; it does not recover or retry automatically.
The real controller explicitly preserves these command values on activation.
Rebuild `franka_hardware` after restoring this patch. The behavior is verified
offline against the installed trajectory controller with measured/desired offsets;
physical startup and stopping behavior are not established by that test.

An existing robot reflex/error still requires operator recovery through the
approved Desk procedure before another physical attempt. Restart the application
after recovery; the software does not clear the fault or reactivate the hardware
on its own. Recheck readiness/alignment before an explicit arm request.

If FCI reports `communication_constraints_violation`, arming may have succeeded
and physical steps may already have run before the stop. Do not interpret the
subsequent inactive hardware state as another alignment failure. Inspect the
first robot error and `TELEOP STOP` event. While armed the terminal now reports
FCI's `control_command_success_rate`; stop events include the last received rate,
feedback age and maximum callback gap for that session. These are diagnostic
measurements, not extra safety permissions or a complete packet-loss trace.

The real manager uses `hardware_synchronization.expect_blocking_read_write: true`
with a 0.0001 s nonblocking-loop guard and explicit memory locking. FCI `readOnce`
already waits for robot packets; ROS must not add a separate periodic sleep or
skip cycles to follow another clock. This is the installed controller manager's
[documented blocking-hardware mode](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#launching-controller-manager).
The nominal rate remains 1 kHz. GenericSystem checks override that mode to false,
because mock hardware does not wait for packets. Overrun reporting and robot-side
communication protection have not been disabled. Blocking-mode warnings can
occur while hardware is not active; after a fault, stop the session and follow
the approved recovery procedure rather than trying to re-arm inactive hardware.

The recorded 2026-10-20 attempt completed its first bounded action before the
communication reflex. A short read-only ping test on the user-confirmed direct
C2 cable had 0% loss but 0.046/0.115/2.972 ms min/avg/max RTT. A separate 10 s host
cyclictest had 14 microseconds maximum scheduler wake-up latency. These short
tests do not establish FCI reliability under camera/Servo/RViz load or prove
the Ethernet adapter is defective. The link used enp101s0, a Realtek RTL8125 with
r8169, at 2.5 Gb/s full duplex; interface counters had no errors or drops. No NIC,
IRQ, CPU isolation, host scheduler or power-management settings were changed.
See [Franka's communication troubleshooting](https://frankarobotics.github.io/docs/troubleshooting.html#motion-stopped-due-to-discontinuities-or-communication-constraints-violation).
For a supervised physical retest after operator recovery, `--no-rviz` can reduce
software-rendering load while keeping the camera and terminal feedback. Physical
communication reliability remains unverified after the timing change.

### Align in the camera window without RViz

```bash
./franka hand-real --robot-ip 10.42.0.55 --execute --no-rviz
```

The camera now shows a live TCP ring and a gold palm-center marker. Move the palm
center onto the ring. The depth rail in the side panel shows the current hand depth
as a gold dot and the equivalent TCP depth as a tick, with a toward/away instruction.
The gap is the full 3D robot-space distance; green indicates the 3 cm capture range.
Click **ARM** in the camera window once aligned and ready. The current state and
arming acceptance/refusal are shown in the app. Preview disables motion buttons;
the execute session starts disarmed and retains every existing readiness check.
Clicks are forwarded through a dedicated child stdout pipe to the launcher's
existing ordered supervisor client, not through the tracking UDP stream. A
fresh explicit click is required for each arm attempt, with no automatic retry.
Disarm is delivered even while an arm response is pending. GUI-only launching is
supported; camera exit or loss of the host-to-supervisor lease still ends the session.

Use **FULLSCREEN**, **Tab** or **F11** to toggle fullscreen; **Esc** returns to
a window and **Q** quits. The window can be resized/maximized. Add `--fullscreen`
to the command above to start fullscreen. **F** keeps its range-reset function.
The dashboard now renders at the actual client-area resolution rather than
enlarging a fixed screenshot. The video preserves its aspect ratio; overlays,
text and buttons are drawn after resizing only the RGB frame. This leaves
camera calibration and observation coordinates unchanged. The source image
remains 640×480, shown in the header; fullscreen cannot add captured detail.

Reflex/rejection diagnosis now records `ROBOT BASELINE DIAGNOSTIC` on the first
valid robot state and when its reported tool/load inertia changes. This works in
preview without arming and includes the configured end-effector/payload masses
and centers of mass, reported external wrench and joint dynamics. It does not
read back or change collision thresholds. A bounded two-second sample history
adds `STOP DIAGNOSTIC` after a stop: force/torque peaks, the last eight received
samples, tracking errors, collision/error flags and inertia settings. This is
received ROS telemetry, not a 1 kHz recording of the final reflex instant.

`ROLLING REJECTION` identifies stream/epoch/sequence/pending, handoff lead or
q/v/a continuity failures, with the packet IDs, remaining lead in milliseconds
and mismatch magnitudes. Malformed or out-of-bound packets have separate
`ROLLING INPUT REJECTION` messages. The existing 60-element rolling-state protocol
is unchanged; rejection details use an additional fixed-size diagnostic topic.
Diagnostic records do not alter arming, collision behavior, workspace, speed,
acceleration, jerk, feedback timeouts or handoff admission limits.

TCP feedback comes from measured FCI `o_t_ee`, including while disarmed, not from
the hand target or planned command. The bridge inverts the same inset workspace
mapping, and the camera then inverts its active image/depth calibration and mirrors
the image coordinate. This shows the hand position corresponding to the TCP in
teleop coordinates; it is not an extrinsic camera/robot scene projection. An
out-of-range TCP is labelled. Stale feedback hides the guide, and range calibration
pauses it. The hand reference is the average of wrist and four knuckle landmarks
(0, 5, 9, 13, 17), shared by tracking, depth sampling and the visible center marker.

Display-only UDP feedback returns to the current camera observation socket on
localhost; no extra port or ROS dependency is needed in the camera environment.
Packets carry the camera session, freshness timestamps and readiness state.

The latest captured physical run had two acknowledgement timeouts, with approximately
34 ms Python planning on both last candidates. This is consistent with an exhausted
90 ms handoff window under load; the rejection counter does not identify the exact
failed condition. Before publishing a curve, the bridge now reserves at least 100 ms
estimated remaining lead for collision checking/delivery and the controller's 20 ms
minimum. If insufficient, it retains the already checked finite tail and tries a
fresh update. Controller deadlines, collision checks and stop conditions are not
relaxed. This reduces avoidable late submissions but does not establish physical
communication reliability or guarantee that RViz caused the observed rejections.

Type **d + Enter** to disarm. Tracking loss, source restart/jump, stale robot
feedback, controller/Servo stops, invalid trajectories and action failures disarm
the bridge and fence pending rolling updates. Returning the hand never automatically
re-arms. Legacy/gripper action acknowledgements are cancelled after disarming. Status
and rolling/action diagnostics are written to `logs/hand-real.log`; RViz shows
`REAL:DISARMED` or `REAL:ARMED`.

Real execution now replaces finite quintic tails while moving. Each new curve starts
at a future robot-clock instant with exactly matching position, velocity and
acceleration. The controller retains the old curve until that handoff; it does not
wait for zero velocity between updates. Initial handoff lead is 150 ms, and updates
are offered at up to 15 Hz. The controller still requires at least 20 ms lead after
validation. The live planner tries 0.2, 0.3, 0.4, 0.5, 0.6, 0.9, 1.2 and 1.4 s
in ascending order for each endpoint size,
using the shortest fully admissible curve before considering a smaller endpoint.
The 0.2 s minimum overlaps the longer handoff lead. A cooperative 20 ms search
budget yields to tracking/robot/Hand callbacks; unfinished validation never
publishes a candidate. The maximum selected horizon remains 1.4 s. Late or
mismatched replacements are rejected and leave the existing checked tail intact.

Nominal caps are 480 mm/s and 3600 mm/s²; joint speed and acceleration caps are
1.0 rad/s and 2.0 rad/s², with joint jerk 27 rad/s³. Relative to the previous
profile, threefold following scales velocity by 3, acceleration by 9 and jerk by
27, with tracking gain 9/s. Conservative whole-arm
travel is at most 100 mm per curve, with a 25° (0.43633231 rad) per-joint ceiling.
At this installation’s 1.723 m reach bound, 100 mm still limits an individual joint
below approximately 0.058 rad (3.33°). Increasing the joint ceiling alone therefore changes no step
while this travel cap dominates. Planner reserves endpoint travel budget for the
moving initial state; velocity, acceleration, jerk and total curved travel are
bounded over each entire polynomial.

These joint ceilings are application settings, not a replacement for FR3's
position-dependent hardware limits. See the [official FR3 limits](https://frankarobotics.github.io/docs/robot_specifications.html#limits-for-franka-research-3-fr3).
The previous Cartesian checks already implied per-joint speed below 0.093 rad/s
and acceleration below 0.233 rad/s², so raising joint caps alone could not speed
that profile. Selecting shorter fully checked curves removes the former 1.2 s preference;
actual speed remains constrained by jerk, travel, geometry and Cartesian bounds.

Workspace validation now follows the actual curved path, including its finite
stopping tail, retaining the additional 10 mm workspace reserve. Servo independently
checks joint limits, scene distance and self-collision distance along that full curve
before forwarding it. Sampling spacing and clearance use the conservative whole-arm
speed bound. Python evaluates the same sample points together using batched FK;
C++ chooses sample spacing from the curve's conservative speed bound rather than
the configured maximum, retaining at most 1 mm travel per interval. Existing
Servo collision/singularity status gates remain enforced.

Only the horizontal mapping has been reversed: increasing raw camera U now
increases base Y. Depth and height directions and camera-to-workspace scale are
unchanged. The camera preview remains mirrored.

Every streaming enable/disable changes a controller session epoch. Replacements must
name the enabled epoch, next sequence and accepted parent, and match command state
at a future handoff. Disarm clears pending future extensions; delayed messages from
older sessions cannot restart motion. An active curve finishes its already checked
stop tail without jumping to measured position. Loss of bridge/tracking updates also
leaves a finite rest endpoint. Maximum planned tail including handoff lead is 1.49 s;
re-arm cooldown remains 1.6 s. These are reference bounds, not measured physical
stopping times or a certified stop. Independent robot stopping remains required.

The previous `RealBridge` and `RobotClockTrajectoryController` remain available for
legacy regressions, but normal `hand-real` launch now selects rolling execution.
Physical performance and stopping of this new mode have not been tested by agent.
Ctrl+C, camera exit or RViz exit still shuts down the supervised session.

The real model sets the local driver option `automatic_error_recovery_on_start`
to false. A failed control start propagates the error instead of automatically
recovering and retrying. Explicit operator recovery remains a separate robot
operation. The source change is recorded in
`/workspace/patches/teleop-startup-recovery.patch`; rebuild `franka_hardware` as
well as this application after restoring sources.

Gripper motion is enabled in the real profile (`enable_gripper: true`). C/O in the
camera calibrates closed/open gestures; it does not home the physical Hand.
A close gesture at or below 25% opening requests a grasp. Opening fully (80% or
higher) requests release. Closure is now an explicit force-controlled `Grasp`,
rather than repeated width moves into an object. `grasp_force_N: 30.0` is the
lowest documented continuous setting for the original Franka Hand (30–70 N),
not a force claimed safe for every printed part. Closing/opening speed stays
10 mm/s; opening remains bounded to 1 mm increments from fresh feedback.

The sequence is:

1. Stop accepting new arm paths and allow the already checked finite trajectory
   to finish. Require fresh measured/desired stationary joints and a finished,
   acknowledged rolling tail for 100 ms. Closure is blocked if this takes over
   three seconds or if 5 cm of upward travel does not fit inside the existing
   workspace/mapping ceiling. There is no instantaneous stop or force-limit bypass.
2. Capture the measured TCP immediately before closure. Hold all XYZ by sending
   no new arm paths while the gripper closes. A >3 mm measured hold drift stops
   the app. Wait for a successful `Grasp` result and fresh nonempty jaw width
   (>2 mm and <79 mm). Failed/rejected/timed-out/empty/cancelled grasps cannot
   unlock the lift. Fully open the hand to clear an interrupted grasp hold.
3. Re-clutch the camera target to the measured TCP at grasp confirmation, discarding
   hand travel accumulated during the hold. X/Y follows immediately. Block negative
   Z velocity and reject descending lift curves. Only a measured TCP rise of 5 cm
   from the captured grasp position changes the state to `carry`; a high hand
   target or desired position cannot unlock it.
4. Restore normal Z following with a lower bound 1 cm above the captured grasp Z.
   Validate outgoing curved paths against this floor as well as the original
   workspace, whole-arm, collision and derivative checks. The floor survives
   disarm/tracking loss/re-arm within the process. It clears only after an explicit
   open gesture has completed opening (within 0.5 mm of 80 mm), then re-clutches
   again to avoid a target jump.

Re-clutching adds a fixed camera-to-TCP translation for the remaining session;
it does not enlarge the workspace. The camera's live TCP guide uses the inverse
translation too. The camera status line shows `GRASP`, remaining `LIFT` distance,
or the `CARRY` floor height. A process restart resets this session state; it does
not establish that an object has been released. Confirm tool/payload state before
starting a new session. Grasp confirmation is not automatic payload identification.

Tracking, hardware and feedback guards remain active in every phase. Disarm or
tracking loss cancels in-flight Move/Grasp actions and fences the rolling stream.
An open gesture during closure cancels the grasp and disarms; re-arm with the
hand open to complete the release while retaining the arm hold.
There is no automatic re-arm, recovery, load change, or homing. Preview creates
neither Move nor Grasp action clients. The trajectory geometry checks are sampled
application checks (50 micrometre numerical tolerance for the grasp height test),
not safety-rated contact/table protection or a guarantee of zero physical drift.

### Measured versus commanded grasp height

At grasp confirmation the bridge captures both measured TCP Z and the held
desired-command TCP Z. An impedance-controlled arm can have a small difference
between these at rest. The recorded 2026-10-21 stalled grasp had a 0.067 mm
difference, greater than the height validator's 0.05 mm numerical tolerance, so
the old check rejected every curve at its initial sample, including upward ones.
The bridge now maps command-height displacement onto the confirmed measured hold
reference. It retains the same numerical tolerance, 5 cm measured lift threshold,
1 cm carry floor and measured-state stop checks. The 1 cm floor remains relative
to the pre-closure measured grasp height; minor hold deflection cannot block an
upward recovery. The height offset is logged at confirmation and used only by
the command-path height check, not to modify the measured robot state.

## Repeatable offline checks

```bash
./franka exec python -m unittest discover -s /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/test -v
./franka hand-real-check
./franka hand-check
./franka exec cmake -S /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/tools/startup_policy_tests -B /tmp/franka-teleop-policy-build
./franka exec cmake --build /tmp/franka-teleop-policy-build --parallel 2
./franka exec /tmp/franka-teleop-policy-build/startup_policy_tests
```

The C++ target compiles the actual driver Robot implementation against mocks,
covering no-retry startup for all five control interfaces. It avoids requiring
the upstream package's optional, currently uninstalled lint suite. Real modes use
domain 133, namespace `/hand_teleop_real` and UDP 49301. The separate actual
controller/mock check uses domain 134. Fake development remains in domain 132.
The bridge-only fixture uses isolated domain 139 and UDP 49303; it never shares
the production observation socket or DDS services/actions.
`hand-real-check` also builds/runs the focused C++ tests. Its position startup
tests link the installed real hardware library to a mock Robot and the installed
trajectory controller, with no physical connection, on domain 136. They verify
desired-position seeding before activation, fresh seeding after restart, refusal
on startup read/non-finite failures, and continuous initial hold commands when
measured positions differ from desired positions.
An empty-manager check on domain 137 verifies the installed binary selects
blocking-hardware synchronization and the intended parameters; it provides no
robot description, hardware plugin or command controller.
The command-continuity suite compares sampled stock and robot-clock controller
outputs under injected host timing jitter, consecutive segment reversals and bad
clock/timestamp inputs. The action integration check uses ClockedMockSystem,
derived from GenericSystem, to supply the new controller's robot-clock interface.
It is isolated on domain 134 and cannot connect to physical hardware.

The workspace realtime_tools overlay also contains a small Linux thread-name
compatibility backport required by the installed ROS hardware interface. The
patch is recorded in `patches/realtime-tools-thread-name.patch` at the workspace
root, with its hash in `config/versions.json`. It changes no scheduler priority
or affinity. The offline check loads the real plugin with immediate symbol
resolution, without constructing the hardware interface or connecting to a robot;
the C++ test checks thread-name truncation and restores its original name.


## Rolling execution offline checks

`./franka hand-real-check` also runs five rolling-controller tests on domain 140 and
`tools/check_smooth_pipeline.py` on domain 141 / UDP 49304. The latter loads the
actual rolling plugin on ClockedMockSystem, actual feedback Servo and smooth bridge,
with synthetic Franka telemetry. No physical hardware driver or camera starts.
It checks nonzero velocity across successive updates, reversal, tracking/feedback loss,
finite-tail stopping, no automatic re-arm and collision refusal. C++ tests include
late/reordered/old-session packets, disarm before handoff, host jitter, measured bias,
plant lag and sampled command derivatives at 1 ms. Python checks include curve
overshoot that endpoints alone would miss. Legacy action checks retain their own
controller selection and isolated domains.

The 100 mm travel limit and workspace geometry remain fixed in the faster profile.
Actual curves may use less travel when jerk, geometry or duration limits require it.
Feedback deadlines and stopping/epoch checks remain enforced; faster following
does not provide safety-rated stopping or guarantee a physical speed factor.
A larger-target comparison can run offline with
`./franka exec env ROS_DOMAIN_ID=141 python /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/tools/check_smooth_pipeline.py --target-distance .06`.
Add `--step-response` to measure 10–90% following time for the same 60 mm target.


## Speed adjustment on the migrated laptop — 2026-10-22

The active development copy is `/home/rebelfranka/workspace/franka` on
`10.40.20.162`; subsequent development/builds use that copy.
The user requested 50% faster arm following and 100% faster gripper motion.

The nominal following gain is now 13.5/s (previously 9/s), Cartesian speed ceiling
0.72 m/s (0.48), and Cartesian acceleration ceiling 8.1 m/s² (3.6). Consistent
1.5x time scaling requires velocity x1.5, acceleration x2.25 and jerk x3.375;
joint ceilings are now 1.5 rad/s, 4.5 rad/s² and 91.125 rad/s³. Python validation,
planner defaults and compiled C++ rolling-curve admission use matching ceilings.
The 100 mm conservative travel cap, 25° step cap, collision/workspace checks,
feedback deadlines, handoff lead and minimum curve duration remain unchanged.
These constraints can still limit achieved speed; the change specifies nominal
following and ceilings, not a guaranteed 1.5x response for every trajectory.

`gripper_speed_m_s: 0.010` now controls both Grasp closure and bounded Move opening
(previously 0.005 m/s). The grasp timeout derives from that speed. The width
increment stays <=1 mm, force stays 30 N, and hold/5 cm upward lift/1 cm carry
floor/release behavior is unchanged. Jaw feedback/action latency can limit the
achieved opening speed of the small Move increments.

Rebuild `franka_hand_controllers` and `franka_hand_teleop` before restarting the app;
the compiled controller must match the new planner ceilings. Prior source files
are backed up under `.migration/speed-update-_e7febii`. Validation is mock-only;
no physical robot connection or motion was requested by the agent.

At the faster response rate, stale asynchronous IK snapshots are skipped while
the last checked finite tail remains in force. The 2 mrad seed-error threshold
is unchanged: an out-of-date proposal never reaches the controller. Nonfinite
or malformed proposals and stale/missing actual robot/Hand feedback still stop
the app. The isolated pipeline explicitly injects stale seeds to verify this.


## Smooth-following repair and second gripper doubling — 2026-10-22

The next physical session exposed planning-budget waits and 20 handoffs after the
previous curve had already finished, with 40–60 ms gaps. The faster profile made
the scalar polynomial checks too costly for the existing 20 ms planning budget.
The planner now precomputes the same depth-five Bernstein subdivision transforms,
batches joint/derivative bounds, caches each candidate's derivative bounds, and
evaluates only positions for the unchanged path sample grid. The candidate search,
curve geometry, 1.8 mm sampling bound, admission limits, C2 splice conditions,
20 ms budget, finite stopping tail and feedback deadlines are retained.

Gripper command speed is now `gripper_speed_m_s: 0.020` (20 mm/s), twice the prior
10 mm/s, for both closing Grasp and opening Move. Opening increments remain <=1 mm;
action and feedback latency can constrain achieved opening rate. Force remains
30 N and the grasp/lift/carry-floor behavior is retained. Restart the application
to load these changes; the Python package was rebuilt with symlink installation.

Model-only replay of all 49 logged admissible plans reduced median computation
from 9.193 ms to 1.136 ms, with all repaired plans completing within 20 ms
(maximum 2.763 ms). Unit tests compare batched hulls against the original scalar
subdivision and compare batch positions with scalar curve evaluations. The mock
pipeline's 60 mm diagonal ramp requires nonzero command velocity throughout its
steady-motion interval and also exercises reversal, grasp/lift/release and stops.
Backup: `.migration/smooth-repair-t6nvgkml`. No physical robot was commanded.


## Gripper command chatter and close-priority repair — 2026-10-22

The next recorded physical run issued 266 opening Move requests but only one
Grasp, which succeeded. The old opening loop sent repeated 1 mm targets from
asynchronous finger snapshots and checked close intent only after the current
action ended. This allowed opening traffic to delay the arm hold/Grasp transition
and could reissue a goal below a more recent opening target.

`GripperCommands` debounces close for 60 ms and deliberate opening for 120 ms.
Close intent starts the existing arm-settling hold before checking for a current
Move, cancels an in-flight opening (including late goal acknowledgement), and
waits for that action's final result before dispatching one Grasp. An expected
Move cancellation for grasp does not disarm; failed/rejected actions, unexpected
cancellations, drift and timeouts retain their stopping behavior. Debouncing also
prevents one noisy open frame from cancelling a valid grasp.

Opening now coalesces width changes below 2 mm, waits for a finger snapshot
received after the previous result, and requires strictly increasing targets.
It never reissues or reverses its last target from delayed feedback. Each opening
goal is bounded to 100 ms of configured motion (4 mm at the new 40 mm/s speed)
and still has a 750 ms timeout and tracking-loss cancellation. The former 1 mm
increment is superseded by this speed-derived bound. A deliberate full release
can still complete the final sub-2 mm increment. New GRIPPER OPEN logs record
measured width, target and speed. Grasp force remains 30 N.

Requested nominal following gain and speed ceiling doubled: 13.5 -> 27 /s and
0.72 -> 1.44 m/s, with Python and C++ admission caps synchronized. Gripper closing
and opening command speed doubled: 0.020 -> 0.040 m/s. Acceleration stays 8.1 m/s²
and joint derivative ceilings stay 1.5 / 4.5 / 91.125; multiplying all accelerations
by four would exceed the published FR3 maximum Cartesian acceleration of 9 m/s²
and joint acceleration of 10 rad/s². Source:
https://frankarobotics.github.io/docs/robot_specifications.html

The 200 ms IK extrapolation can exceed the remaining hand-target error at high
gain. Its endpoint is now projected toward, and capped at, the live TCP target
before the unchanged whole-curve derivative/collision/workspace validation.
Grasp command-height alignment is applied when using a lift/carry floor. It does
not accumulate a forecast or alter C2 handoff continuity. Nominal doubling does
not guarantee twice the physical response: acceleration, 100 mm conservative
whole-arm travel, minimum duration, collision/singularity scaling and feedback
still constrain each curve.

Both packages were rebuilt and installed. Mock regressions verify zero Move
goals for small stationary opening noise, monotonic goals with old snapshots,
close priority during a freshly started opening, Grasp/lift/floor/release,
continuous diagonal motion and fixed-target settling. The step test's final TCP
hold varied by 0.052 mm. All tests use isolated mock domains; no physical robot
was connected to, armed or commanded. Backup: `.migration/gripper-chatter-qhv7nwmc`.
Restart the app with the usual command to load the installed changes.
