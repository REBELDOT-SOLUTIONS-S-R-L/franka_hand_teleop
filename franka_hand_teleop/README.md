# FR3 hand teleoperation

The `hand-fake` development path is **fake hardware only**. It does not read X4, configure
Watchman, unlock brakes, activate FCI, or connect to a physical arm or gripper.
The real robot can remain locked throughout development.

A separate real-hardware commissioning path is documented in
[REAL_DEPLOYMENT.md](REAL_DEPLOYMENT.md). It defaults to preview with no bridge
motion output. The local real profile records installation checks reported
completed and approved by the user and Safety Operator on 2026-10-20. Motion
still requires `--execute` and manual arming. Physical app motion has not been
tested by the agent. The fake launcher and bridge retain their mock-only checks.
The real path uses the sibling `franka_hand_controllers` package to time joint
trajectory interpolation from the robot's own clock; the bridge verifies that
controller type before arming. `hand-real-check` includes per-command continuity
tests, rolling integration and legacy action cancellation on a clocked mock derived from GenericSystem.
Real mode also uses feedback-anchored Servo proposals from that package, so its
rolling references preserve commanded position, velocity and acceleration at future
handoffs without waiting for each target to stop. Full curves and finite stopping
tails pass workspace and collision checks before execution. Offline checks use
actual rolling control on ClockedMockSystem and synthetic robot telemetry.

The camera project stays at `/home/roboticslab/hand-tracking` and runs on the host
with its existing Python environment. Its optional localhost UDP stream carries
observations into the host-network ROS Docker container; no camera USB forwarding
or ROS installation in the camera environment is required.

## Run

From `/home/roboticslab/franka`, after building with `./franka build`:

```bash
./franka hand-fake
```

This one command starts the D435 camera app on the host and the mock FR3, Servo,
bridge, and RViz in Docker. **There is no enable button.** Once metric palm XYZ
and calibrated finger opening are available for 0.3 seconds, the fake robot starts
following the fixed camera-to-workspace mapping. There is no relative hand origin.

Calibrate thumb/pinky opening with **C** (closed) then **O** (open) in the camera
window if needed. A webcam cannot substitute for the D435's measured XYZ.
Hide your hand to stop. Showing it again selects the position corresponding to
its location in the camera; it does not reset the mapping. The robot approaches
that target at the configured speed, rather than teleporting. Tool orientation
is held at its initial orientation throughout the run.

Press **B** to start camera-range calibration. Sweep your hand through comfortable
left/right, top/bottom, and near/far limits, then press **B** again to save. Collect
at least 30 valid depth frames spanning 15% of image width/height and 10 cm of
depth. Robot tracking holds while calibration is in progress. The preview rectangle
shows the calibrated image region. **F** restores defaults. Calibration is saved
separately from finger opening in `~/.config/thumb-pinky-tracker/control_region.json`
(with the existing XDG configuration override support).

Ctrl+C stops everything. Quitting the camera window with Q/Esc or closing RViz
also stops the other processes. Robot logs go to `logs/hand-fake.log`.
No changes are required on the real robot.

To try the entire application without a camera:

```bash
./franka hand-fake --camera demo
```

For an automated, windowless demo that exits after 240 frames:

```bash
./franka hand-fake --camera demo --no-rviz --headless --frames 240
```

`--no-rviz` omits RViz; `--serial NUMBER` selects a RealSense; `--tracker-dir PATH`
selects a different location for the existing camera project. Other options are
listed by `./franka hand-fake --help`.

## Mapping and workspace

Normalized palm image coordinates and measured palm depth map linearly to the
usable tool workspace. The same camera location always produces the same target,
including after tracking loss or a camera restart. The calibrated camera range
spans the entire tool region, with a 20 mm inset from its hard boundaries:

| Position in the unmirrored camera control region | Fake robot target |
| --- | --- |
| Near → far depth | Maximum → minimum base X |
| Raw camera image left → right | Minimum → maximum base Y |
| Image top → bottom | Maximum → minimum base Z |

The camera preview is mirrored; check the direction visually before practising.
Defaults use the central 90% of the RGB image and depth 0.35–1.00 m. These are
starting values; use B to calibrate your comfortable control range. Moving outside
the calibrated region saturates the target at its corresponding edge.
The existing calibrated
thumb-to-pinky separation maps to 0–80 mm simulated jaw width, with a finger
speed limit of 30 mm/s. Both fingers are provided by mock hardware, including
the model's mimic joint; no real gripper actions are used.

`config/teleop.yaml` defines two independent volumes relative to `fr3_link0`.
The tool operating bounds are inset 50 mm from every face of the user's measured
box, in both fake and real profiles. These are software limits; physical safety
commissioning is incomplete.
X/Y are measured from the centre of the first-joint axis, with positive directions
given by the base markings. Z=0 is the mounting plane/table surface, as confirmed
by the user. The supplied reversed Y min/max are ordered as −0.35 to +0.35 m.

- **Green, ARM (example bounds):** X −0.45–0.85 m, Y −0.55–0.55 m, Z −0.05–1.10 m.
  This includes the base, elbow, wrist, hand, and full finger travel. It must contain
  the mounted base and initial robot pose. It is deliberately larger than the tool region.
- **Blue, TOOL (inset operating bounds):** X 0.45–0.65 m, Y −0.30–0.30 m, Z 0.05–0.25 m.
  The measured box is X 0.40–0.70 m, Y −0.35–0.35 m, Z 0.00–0.30 m.
  Targets stay a further 20 mm inside: X 0.47–0.63 m, Y −0.28–0.28 m, Z 0.07–0.23 m.
  The bounds constrain `fr3_hand_tcp`, rather than every surface of the gripper.
  The yellow dot is the target;
  the blue dot is the measured TCP.

`config/initial_positions.yaml` initializes mock joint states with the TCP at
X=0.55 m, Y=0, Z=0.15 m, pointing down. This replaces the standard mock start
above the new tool box; no approach from outside the box is commanded.
The initial joint values apply only to `mock_components/GenericSystem`.

Translation commands are limited to 0.08 m/s and acceleration to 0.30 m/s², with
extra boundary braking. Servo output goes through a trajectory guard before it
reaches the fake controller. The guard uses URDF forward kinematics, conservative
boxes around all collision meshes/primitives, and checks interpolated joint paths
against both volumes. Finger envelopes cover the entire opening range. Unsafe
trajectories are replaced with a joint hold. Collision envelopes may stop movement
earlier than the visible robot surface because their bounds are conservative.

Servo continues to check self-collision, a simple simulated table, singularities,
and joint limits. These software checks are for this mock application, not certified
physical-robot safety functions. Collision geometry does not include an external
payload or objects held in the gripper.
Some mapped targets may be unreachable because of joint, collision, or whole-arm
limits; the robot holds rather than bypassing those limits.

## Interlocks and isolation

- Dedicated ROS domain **132**, namespace `/hand_teleop_fake`, namespaced TF.
- Launch file rejects other domains. No robot IP or real-hardware switch exists.
- The URDF contains only `mock_components/GenericSystem`.
- Before publishing commands, the bridge checks the controller manager for the
  exact mock plugin and expected system name. Controller activation and fresh
  Servo status are also required to arm. These checks refresh continuously.
- Camera packets must carry fresh source monotonic timestamps (250 ms),
  sessions, and increasing sequence numbers. This protocol is for the same Linux
  host and host-network Docker only. The socket listens on `127.0.0.1:49300`.
- Tracking/depth/calibration loss, stale feedback, hand jumps, sender restart,
  malformed input, and Servo hard stops stop tracking. After 0.3 seconds of valid
  data, tracking resumes toward the same absolute mapped position. The camera
  mapping and tool orientation do not change on reacquisition.
- The host launcher supervises the camera and ROS processes together. Closing
  its stdin pipe stops the container-side session, preventing an abandoned ROS
  stack when the host launcher exits.
- Servo has its own 150 ms command watchdog if the bridge stops. Stops involve
  controller/smoothing latency; fake hardware does not validate physical stopping.

## Verification

```bash
./franka exec python -m unittest discover -s /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/test -v
./franka hand-check
```

The integration check launches its own headless mock stack in domain 132 and
exercises synthetic observations, missing tracking, automatic start, movement,
tracking timeout, fixed mapping across re-entry, camera-region saturation, speed
limits, simulated fingers, and rejection of unsafe tool/whole-arm trajectories.
Every sampled feedback state is checked for TCP and whole-arm containment.
Close an existing `hand-fake` instance
first. Logs are in `logs/hand-teleop-check.log`.

Status inspection (while `hand-fake` runs):

```bash
./franka exec env ROS_DOMAIN_ID=132 ros2 topic echo /hand_teleop_fake/status
```

The MoveIt Servo integration follows the installed Jazzy interfaces and the
[official Servo documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html).
The guard forwards position-only paths, matching the controller's
[linear interpolation behavior](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html).

Do not reuse this launch as a real-robot safety solution. Real operation
requires a separate implementation and commissioning of X4/enabling behavior,
verification of the measured workspace and coordinate origin, tool geometry,
and robot-side safety limits.
