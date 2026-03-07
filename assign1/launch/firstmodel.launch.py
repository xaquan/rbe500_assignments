#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_name = "assign1"
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'myfirst.urdf.xacro']),
        description='Absolute path to URDF/Xacro model file',
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

    # rvizconfig_arg = DeclareLaunchArgument(
    #     name='rvizconfig',
    #     default_value=PathJoinSubstitution([FindPackageShare(pkg_name), 'rviz', 'display.rviz']),
    # )

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
    

    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'myfirst_robot',
    #         '-topic', 'robot_description',
    #         '-x', '0', '-y', '0', '-z', '0.5'
    #     ],
    #     output='screen',
    # )

    # Spawn controllers so joint1_controller subscribes to command topic.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    joint1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint1_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    # delay_jsb_after_spawn = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    # delay_joint1_after_jsb = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[joint1_controller_spawner],
    #     )
    # )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gui_arg,
        use_sim_time_arg,
        # spawn_entity,
        gazebo_launch,
        joint_state_broadcaster_spawner,
        joint1_controller_spawner,
        # delay_jsb_after_spawn,
        # delay_joint1_after_jsb,
    ])