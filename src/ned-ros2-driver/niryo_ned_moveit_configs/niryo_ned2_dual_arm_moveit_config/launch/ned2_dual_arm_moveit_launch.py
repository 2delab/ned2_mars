import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from rclpy import logging

logger = logging.get_logger("ned2_dual_arm_moveit.launch")


def generate_launch_description():
    """
    Simplified dual-arm MoveIt2 launch file following ROS2 best practices.

    Architecture:
    - Drivers publish prefixed joint states (/arm_1/joint_states, /arm_2/joint_states)
    - RSP (Robot State Publisher) converts joint states to TF transforms
    - ROS2 TF system automatically merges /arm_1/tf and /arm_2/tf into unified /tf
    - MoveIt move_group plans and executes with unified robot model
    - RViz visualizes the dual-arm system

    No custom aggregators needed - drivers already publish prefixed data matching URDF.
    """
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    # Path to unified URDF with both robots (arm_1 and arm_2 prefixes)
    urdf_file = os.path.join(
        get_package_share_directory("niryo_ned_description"),
        "urdf/ned2",
        "niryo_ned2_dual.urdf.xacro",
    )

    # Single MoveIt config with both move groups defined
    moveit_config = (
        MoveItConfigsBuilder("niryo_ned2_dual_arm")
        .robot_description(
            file_path=urdf_file,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned2_dual.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Joint State Prefixer - bridges unprefixed driver data to prefixed URDF
    # Subscribes to: /arm_1/joint_states, /arm_2/joint_states (unprefixed names)
    # Publishes to: /joint_states (with prefixed names: arm_1_joint_1, arm_2_joint_1, etc.)
    # This enables RSP to match joint names in the URDF
    joint_state_prefixer_node = Node(
        package="niryo_ned2_dual_arm_moveit_config",
        executable="joint_state_prefixer.py",
        output="screen",
        parameters=[
            {"robot_namespaces": ["arm_1", "arm_2"]},
            {"publish_frequency": 15.0},
        ],
    )

    # Robot State Publisher - publishes TF from joint states
    # Subscribes to /joint_states (with prefixed names from prefixer)
    # Publishes corresponding TF frames to /tf (unified tree)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 15.0},
        ],
    )

    # MoveIt move_group - handles planning and execution for both arms
    # Routes trajectories to hardware via SimpleControllerManager
    # SimpleControllerManager uses action_ns paths defined in moveit_controllers.yaml
    # Actions are provided by Niryo driver nodes (arm_1 and arm_2 namespaces)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz with dual robot visualization
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_ned2_dual_arm_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            joint_state_prefixer_node,
            rsp_node,
            move_group_node,
            rviz_node,
        ]
    )
