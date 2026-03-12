#!/usr/bin/env python3

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    db_arg = DeclareLaunchArgument("db", default_value="false")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value="moveit.rviz")
    publish_frequency_arg = DeclareLaunchArgument(
        "publish_frequency", default_value="15.0"
    )
    load_scene_arg = DeclareLaunchArgument(
        "load_scene",
        default_value="true",
        description="Load workspace planning scene on startup",
    )

    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_dual", package_name="niryo_ned2_dual_arm_moveit_config"
    ).to_moveit_configs()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )

    # ROS2 control node
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

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_1"],
        output="screen",
    )

    arm_2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_2"],
        output="screen",
    )

    # MoveIt move_group node
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

    # Workspace scene loader (optional) - loads planning scene from YAML file
    # Uses ROS 2 planning scene API to add collision objects
    pkg_share_dir = get_package_share_directory("niryo_ned2_dual_arm_moveit_config")
    install_dir = str(
        Path(pkg_share_dir).parent.parent
    )  # Go up: share/pkg -> . -> install
    scene_script = os.path.join(
        install_dir,
        "lib",
        "niryo_ned2_dual_arm_moveit_config",
        "load_workspace_scene.py",
    )

    load_scene_node = ExecuteProcess(
        cmd=["python3", scene_script],
        output="screen",
        condition=IfCondition(LaunchConfiguration("load_scene")),
    )

    # Async Executor Node for asynchronous dual-arm trajectory execution
    # This node implements the paper: "A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"
    async_executor_node = Node(
        package="niryo_ned2_dual_arm_moveit_config",
        executable="async_executor_node.py",
        output="screen",
        parameters=[
            os.path.join(
                str(moveit_config.package_path), "config", "async_executor.yaml"
            ),
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("niryo_ned2_dual_arm_moveit_config"),
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

    # Warehouse database (optional)
    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("niryo_ned2_dual_arm_moveit_config"),
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
            load_scene_arg,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_1_controller_spawner,
            arm_2_controller_spawner,
            move_group_node,
            load_scene_node,
            # async_executor_node,  # Add async executor for asynchronous trajectory execution
            rviz_node,
            warehouse_db_launch,
        ]
    )
