#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'assign3'
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    pkg_share_path = get_package_share_directory(package_name)

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    world_path = os.path.join(pkg_share_path, 'worlds', 'empty.sdf')
    gui_config_path = os.path.join(pkg_share_path, 'config', 'gui.config')
    xacro_path = os.path.join(pkg_share_path, 'urdf', 'scara_fixed_joints.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_path]),
        value_type=str,
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'scara_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
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

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
        ],
        output='screen',
    )

    apply_joint_force_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/scara_robot/joint/joint3/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        output='screen',
    )

    joint_control_service = Node(
        package=package_name,
        executable='joint_control_service',
        name='joint_control_service_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    joints_to_ee_velocities_service = Node(
        package=package_name,
        executable='joints_to_ee_velocities_service',
        name='joints_to_ee_velocities_service_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share_path, 'models'),
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            os.path.join(pkg_share_path, 'plugins'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': f'-r {world_path} --gui-config {gui_config_path}',
                'on_exit_shutdown': 'True',
            }.items(),
        ),
        robot_state_publisher,
        spawn_robot,
        joint_state_bridge,
        apply_joint_force_bridge,
        joint_control_service,
        joints_to_ee_velocities_service
    ])
