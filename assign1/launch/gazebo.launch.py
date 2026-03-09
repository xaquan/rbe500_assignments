#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_name = "assign1"
    pkg_share = get_package_share_directory(pkg_name)

    default_xacro = os.path.join(pkg_share, "urdf", "myfirst.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_xacro,
        description="Absolute path to robot URDF.xacro",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo GUI",
    )

    pause_arg = DeclareLaunchArgument(
        "pause",
        default_value="false",
        description="Start Gazebo paused",
    )

    robot_description = Command(["xacro", " ", LaunchConfiguration("model")])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 
                'launch', 
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            # 'gz_args': '-r empty.sdf'  # -r runs on start, empty.sdf is the world
            # To handle the GUI toggle similar to your old code:
            'gz_args': LaunchConfiguration('gz_args'), 
        }.items(),
    )
    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "myfirst_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 'controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    joint1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint1_controller',
            '--controller-manager', 'controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    delay_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_joint1_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint1_controller_spawner],
        )
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        gui_arg,
        pause_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        joint1_controller_spawner,
    ])