#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'assign2'
    pkg_share_path = get_package_share_directory(package_name)
    gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')

    default_xacro_path = os.path.join(pkg_share_path, 'urdf', 'scara_fixed_joints_classic.urdf.xacro')
    default_controllers_path = os.path.join(pkg_share_path, 'config', 'joint_state_broadcaster.yaml')
    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to robot Xacro file',
    )
    declare_controllers = DeclareLaunchArgument(
        'controllers_file',
        default_value=default_controllers_path,
        description='Absolute path to controller configuration file',
    )

    model = LaunchConfiguration('model')
    controllers_file = LaunchConfiguration('controllers_file')

    robot_description = ParameterValue(
        Command(['xacro', ' ', model, ' ', 'controllers_file:=', controllers_file]),
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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )

    # load_joint_state_broadcaster = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=spawn_robot,
    #         on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
    #     )
    # )

    return LaunchDescription([
        declare_model,
        declare_controllers,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        # load_joint_state_broadcaster,
        # joint_state_broadcaster_spawner
    ])
