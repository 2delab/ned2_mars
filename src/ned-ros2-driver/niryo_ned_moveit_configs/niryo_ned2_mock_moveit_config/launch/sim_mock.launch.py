#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Launch file for niryo_ned2 mock MoveIt simulation with ros2_control.

    This provides:
    - Mock hardware simulation via ros2_control
    - Joint State Publisher (for manual joint control)
    - Robot State Publisher (TF transforms)
    - ROS2 control node with mock components
    - MoveIt move_group (planning and execution)
    - RViz with interactive motion planning visualization
    - Optional warehouse database for motion planning history

    Usage: ros2 launch niryo_ned2_mock_moveit_config sim_mock.launch.py
    """
    # Launch arguments
    db_arg = DeclareLaunchArgument("db", default_value="false")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value="moveit.rviz")
    publish_frequency_arg = DeclareLaunchArgument(
        "publish_frequency", default_value="15.0"
    )

    # MoveIt configuration - for single-arm mock simulation
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()

    # Robot state publisher - broadcasts TF transforms
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )

    # ROS2 control node - manages mock hardware and controllers
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                str(moveit_config.package_path), "config", "ros2_controllers.yaml"
            ),
        ],
    )

    # Joint state broadcaster spawner - publishes joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Arm controller spawner - loads trajectory controller for arm
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # MoveIt move_group node - handles planning and execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    # RViz visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("niryo_ned2_mock_moveit_config"),
                    "config",
                    LaunchConfiguration("rviz_config"),
                ]
            ),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Warehouse database (optional) - stores motion planning history
    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("niryo_ned2_mock_moveit_config"),
                    "launch",
                    "warehouse_db.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("db")),
    )

    return LaunchDescription(
        [
            db_arg,
            use_rviz_arg,
            rviz_config_arg,
            publish_frequency_arg,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_node,
            rviz_node,
            warehouse_db_launch,
        ]
    )
