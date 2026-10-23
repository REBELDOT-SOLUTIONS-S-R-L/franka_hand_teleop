"""Real FR3 connection, defaulting to preview with the arm controller inactive."""
import json
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_hand_teleop.deployment import REAL_DOMAIN, endpoint, load_profile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def model_for(ip):
    bringup = Path(get_package_share_directory("franka_bringup"))
    model = xacro.process_file(str(bringup/"urdf/franka_arm.urdf.xacro"), mappings={
        "robot_type": "fr3", "arm_prefix": "", "robot_ip": endpoint(ip),
        "hand": "true", "use_fake_hardware": "false", "fake_sensor_commands": "false",
    }).toxml()
    root = ET.fromstring(model)
    plugins = [p.text for p in root.findall("ros2_control/hardware/plugin")]
    if plugins != ["franka_hardware/FrankaHardwareInterface"]:
        raise ValueError(f"Unexpected real model plugins: {plugins}")
    ET.SubElement(root.find("ros2_control/hardware"), "param",
                  name="automatic_error_recovery_on_start").text = "false"
    return ET.tostring(root, encoding="unicode")


def launch_real(context):
    if os.environ.get("ROS_DOMAIN_ID") != REAL_DOMAIN:
        raise RuntimeError(f"Real hand teleop requires ROS_DOMAIN_ID={REAL_DOMAIN}")
    execute = LaunchConfiguration("execute").perform(context).lower() == "true"
    profile_path = LaunchConfiguration("profile").perform(context)
    profile, _ = load_profile(profile_path, execute=execute)  # Validate before constructing any hardware node.
    ip = endpoint(LaunchConfiguration("robot_ip").perform(context))
    share = Path(get_package_share_directory("franka_hand_teleop"))
    description = Path(get_package_share_directory("franka_description"))
    moveit = Path(get_package_share_directory("franka_fr3_moveit_config"))
    model = {"robot_description": model_for(ip)}
    semantic = {"robot_description_semantic": xacro.process_file(
        str(description/"robots/fr3/fr3.srdf.xacro")).toxml()}
    planning = {"robot_description_planning": yaml.safe_load(
        (moveit/"config/fr3_joint_limits.yaml").read_text())}
    kinematics = {"robot_description_kinematics": yaml.safe_load(
        (moveit/"config/kinematics.yaml").read_text())}
    servo = yaml.safe_load((share/"config/servo.yaml").read_text())
    servo["moveit_servo"] = {k: v.replace("/hand_teleop_fake/", "/hand_teleop_real/")
                            if isinstance(v, str) else v for k, v in servo["moveit_servo"].items()}
    # Bounded actions provide quintic smoothing. A filter reset on every IK
    # proposal attenuates every increment; fake streaming retains its filter.
    servo["moveit_servo"]["use_smoothing"] = False
    # One immutable startup snapshot is consumed by every motion component.
    session = tempfile.TemporaryDirectory(prefix="hand-motion-")
    snapshot = Path(session.name)/"motion-profile.json"
    profile["_source_profile"] = profile_path
    snapshot.write_text(json.dumps(profile, allow_nan=False))
    controllers = yaml.safe_load((share/"config/real_controllers.yaml").read_text())
    controllers["/**/fr3_arm_controller"]["ros__parameters"]["motion_profile"] = str(snapshot)
    controller_file = Path(session.name)/"controllers.yaml"
    controller_file.write_text(yaml.safe_dump(controllers))

    def cleanup(_):
        session.cleanup()
        return []

    common = {"namespace": "hand_teleop_real", "output": "screen",
              "remappings": [("/tf", "tf"), ("/tf_static", "tf_static")]}
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[model], **common),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[model, str(controller_file),
                         {"robot_type": "fr3", "load_gripper": True, "arm_prefix": ""}],
             on_exit=Shutdown(), **common),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "franka_robot_state_broadcaster",
                        "--controller-manager", "/hand_teleop_real/controller_manager"], **common),
        Node(package="controller_manager", executable="spawner",
             arguments=["fr3_arm_controller", "--inactive", "--param-file", str(controller_file),
                        "--controller-manager", "/hand_teleop_real/controller_manager"], **common),
        # State reads only at startup. No homing, grasp, setLoad or setEE call.
        Node(package="franka_gripper", executable="franka_gripper_node", name="franka_gripper",
             namespace="hand_teleop_real", output="screen", on_exit=Shutdown(),
             parameters=[{"robot_ip": ip, "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
                          "state_publish_rate": 30, "feedback_publish_rate": 30}],
             remappings=[("~/joint_states", "gripper_joint_states")]),
        Node(package="franka_hand_controllers", executable="feedback_servo", name="servo_node",
             parameters=[model, semantic, planning, kinematics, servo, {"motion_profile": str(snapshot)}], on_exit=Shutdown(), **common),
        Node(package="franka_hand_teleop", executable="smooth_bridge", name="hand_bridge",
             parameters=[model, semantic, {"profile": str(snapshot), "execute": execute}],
             on_exit=Shutdown(), **common),
        Node(package="rviz2", executable="rviz2", name="real_hand_rviz",
             arguments=["-d", str(share/"config/real.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")), on_exit=Shutdown(), **common),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("franka_hand_teleop"))
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", description="Actual C2 IPv4 address; no default"),
        DeclareLaunchArgument("profile", default_value=str(share/"config/real_profile.yaml")),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_real),
    ])
