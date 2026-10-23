# Franka hand teleoperation

This repository contains the two ROS 2 packages for our FR3 hand-teleoperation
application. Colcon discovers both packages beneath this repository automatically.

```text
franka_hand_teleop/
├── franka_hand_teleop/       # Python application, configuration, launchers and tests
└── franka_hand_controllers/  # C++ Servo, rolling controller and clocked mock hardware
```

## Existing workspace

On the development laptop this repository lives at
`~/workspace/franka/ws/src/franka_hand_teleop`. The workspace wrapper is
`~/workspace/franka/franka`. Existing `./franka hand-real`, `hand-fake` and
`hand-real-check` commands are updated for this layout; ROS package names and
interfaces are unchanged.

The host camera/UI remains a separate project at
`~/workspace/hand-tracking`. The workspace wrapper/container, SDK dependencies,
and robot-specific configuration outside these two packages are not part of this
repository. Host launchers find the workspace's `franka` + `compose.yaml` markers
instead of assuming a fixed parent-directory depth.

## Configuration

Edit `franka_hand_teleop/config/real_profile.yaml` for motion, gripper and timing
settings. Restart the app to load a new session snapshot. See
[real deployment and tuning](franka_hand_teleop/REAL_DEPLOYMENT.md) and
[package documentation](franka_hand_teleop/README.md).

## Build and inspect

From `~/workspace/franka`:

```bash
./franka exec bash -c 'cd /workspace/ws && colcon build --packages-select franka_hand_controllers franka_hand_teleop --symlink-install'
./franka hand-real --robot-ip 192.0.2.1 --dry-run
./franka exec python3 -m unittest discover -s /workspace/ws/src/franka_hand_teleop/franka_hand_teleop/test -v
```

The dry run does not start the camera or connect to hardware. Normal real launch
uses preview unless execution is explicitly requested and the app is armed.

## Dependencies

This application is developed against ROS 2 Jazzy and the existing Franka
workspace. Current source revisions are:

| Dependency | Revision |
| --- | --- |
| franka_ros2 | `4acc05c01ebfcdf916ee6b3df966df0ad7b2f9ec` |
| franka_description | `7aeeddc449edf8d62b594f9e36a81da53e7796f9` |
| libfranka | `3cc708cafa1cdba49ceaab536a14216dfb7bf48a` |

The local franka_ros2 checkout also contains driver/feedback/real-time fixes;
those changes remain outside this repository and must be preserved when
reproducing the current installation. Hardware-policy tests expect its source
at `../franka_ros2/franka_hardware`, relative to this repository. An alternative
source location can be supplied through CMake's `FRANKA_HARDWARE_SOURCE` option.

The repository folder is `franka_hand_teleop`; its two ROS package names remain
`franka_hand_teleop` and `franka_hand_controllers`.
