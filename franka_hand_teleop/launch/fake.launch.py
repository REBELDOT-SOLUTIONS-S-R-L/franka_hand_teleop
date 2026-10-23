"""Dedicated fake-only stack. No robot IP or real-hardware launch argument exists."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_fake(context):
    if os.environ.get("ROS_DOMAIN_ID") != "132":
        raise RuntimeError("Fake hand teleop requires isolated ROS_DOMAIN_ID=132")
    share = Path(get_package_share_directory("franka_hand_teleop"))
    description = Path(get_package_share_directory("franka_description"))
    moveit = Path(get_package_share_directory("franka_fr3_moveit_config"))
    model = {"robot_description": ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(share / "urdf/fake_fr3.urdf.xacro")]),
        value_type=str)}
    semantic = {"robot_description_semantic": ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(description / "robots/fr3/fr3.srdf.xacro")]),
        value_type=str)}
    limits = {"robot_description_planning": yaml.safe_load(
        (moveit / "config/fr3_joint_limits.yaml").read_text())}
    kinematics = {"robot_description_kinematics": yaml.safe_load(
        (moveit / "config/kinematics.yaml").read_text())}
    common = {"namespace": "hand_teleop_fake", "output": "screen",
                  "remappings": [("/tf", "tf"), ("/tf_static", "tf_static")]}
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[model], **common),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[model, str(share / "config/controllers.yaml")],
             on_exit=Shutdown(), **common),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "fr3_arm_controller", "fake_gripper_controller",
                        "--controller-manager", "/hand_teleop_fake/controller_manager"], **common),
        Node(package="moveit_servo", executable="servo_node", name="servo_node",
             parameters=[model, semantic, limits, kinematics,
                         yaml.safe_load((share / "config/servo.yaml").read_text())],
             on_exit=Shutdown(), **common),
        Node(package="franka_hand_teleop", executable="bridge", name="hand_bridge",
             parameters=[model, str(share / "config/teleop.yaml")], on_exit=Shutdown(), **common),
        Node(package="rviz2", executable="rviz2", name="fake_hand_rviz",
             arguments=["-d", str(share / "config/fake.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")), on_exit=Shutdown(), **common),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_fake),
    ])
