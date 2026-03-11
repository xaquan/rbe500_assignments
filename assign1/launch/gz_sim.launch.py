#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'assign1'
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    # pkg_share = FindPackageShare(package_name)
    pkg_share_path = get_package_share_directory(package_name)
    
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    world_path = os.path.join(pkg_share_path, 'worlds', 'empty.sdf')
    gui_config_path = os.path.join(pkg_share_path, 'config', 'gui.config')
    # xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'scara.urdf.xacro'])
    xacro_path = os.path.join(pkg_share_path, 'urdf', 'scara.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_path]),
        value_type=str
    )

    # Spawn the robot in Gazebo (replace with your own model and topic names)
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

    # Publish the robot state (replace with your own robot description topic)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Bridge for joint states and trajectories
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
        ],
        output='screen'
    )

    # Bridge for joint trajectory commands
    # only use to send trajectory commands to Gazebo, not for receiving joint states
    joint_trajectory_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/scara_robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
        ],
        output='screen'
    )

    # Bridge for robot pose
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/scara_robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            # PathJoinSubstitution([pkg_share, 'models'])
            os.path.join(pkg_share_path, 'models')
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            # PathJoinSubstitution([pkg_share, 'plugins'])
            os.path.join(pkg_share_path, 'plugins')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': f"-r {world_path} --gui-config {gui_config_path}",
                'on_exit_shutdown': 'True',
            }.items(),
        ),
        robot_state_publisher,
        joint_trajectory_bridge,
        spawn_robot,
        joint_state_bridge,
        pose_bridge,
    ])