#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "assign1"
    pkg_share = get_package_share_directory(pkg_name)

    default_model_path = os.path.join(pkg_share, "urdf", "myfirst.urdf.xacro")
    default_rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    declare_model = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to robot URDF/Xacro file",
    )

    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui",
    )

    declare_rvizconfig = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz_config,
        description="Absolute path to RViz config file",
    )

    # Expand xacro -> URDF (also fine if you later switch the file to .urdf, just update default_model_path)
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # If you don't have an RViz config yet, RViz will still open,
    # but may warn if the file doesn't exist. Create rviz/display.rviz or change rvizconfig.
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            declare_model,
            declare_use_gui,
            declare_rvizconfig,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )