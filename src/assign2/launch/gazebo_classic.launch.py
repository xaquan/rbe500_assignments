#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'assign2'
    pkg_share_path = get_package_share_directory(package_name)
    gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')

    default_xacro_path = os.path.join(pkg_share_path, 'urdf', 'scara_fixed_joints_classic.urdf.xacro')
    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to robot Xacro file',
    )

    model = LaunchConfiguration('model')

    robot_description = ParameterValue(
        Command(['xacro', ' ', model]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'pause': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'scara_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
    )

    return LaunchDescription([
        declare_model,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
