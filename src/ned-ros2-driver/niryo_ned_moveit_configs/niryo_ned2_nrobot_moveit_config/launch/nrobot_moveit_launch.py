import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from rclpy import logging

logger = logging.get_logger("ned2_nrobot_moveit.launch")


def generate_launch_description():
    """
    Launch file for N-robot Niryo Ned2 MoveIt2 system

    Architecture:
    - Drivers publish prefixed joint states (/arm_X/joint_states)
    - JointStatePrefixer aggregates states and provides proxy trajectories
    - RSP (Robot State Publisher) converts joint states to TF transforms
    - MoveIt move_group plans and executes with unified robot model
    - RViz visualizes all robots
    """

    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    robot_config_arg = DeclareLaunchArgument(
        "robot_config_file",
        default_value="config/robot_config.yaml",
        description="Robot configuration YAML file",
    )

    # Get package share directory
    package_dir = get_package_share_directory("niryo_ned2_nrobot_moveit_config")

    # Load robot configuration to extract robot names dynamically
    config_file = os.path.join(package_dir, "config", "robot_config.yaml")
    if os.path.exists(config_file):
        with open(config_file, "r") as f:
            robot_config = yaml.safe_load(f)
        robot_names = [r["name"] for r in robot_config.get("robots", [])]
    else:
        logger.warning(f"robot_config.yaml not found at {config_file}")
        robot_names = ["arm_1", "arm_2"]

    # Path to parametric URDF with N robots
    urdf_file = os.path.join(
        package_dir,
        "config",
        "niryo_ned2_nrobot.urdf.xacro",
    )

    # MoveIt configuration builder - loads all generated configs
    moveit_config = (
        MoveItConfigsBuilder(
            "niryo_ned2_nrobot", package_name="niryo_ned2_nrobot_moveit_config"
        )
        .robot_description(
            file_path=urdf_file,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned2_nrobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Joint State Prefixer - aggregates unprefixed driver states to prefixed joint states
    # Subscribes to: /arm_X/joint_states (unprefixed joint names from drivers)
    # Publishes to: /joint_states (with prefixed names: arm_1_joint_1, arm_2_joint_1, etc.)
    joint_state_prefixer_node = Node(
        package="niryo_ned2_nrobot_moveit_config",
        executable="joint_state_prefixer.py",
        output="screen",
        parameters=[
            {
                "robot_namespaces": robot_names
            },  # Dynamically extracted from robot_config.yaml
            {"publish_frequency": 15.0},
        ],
    )

    # Robot State Publisher - publishes TF from joint states
    # Subscribes to: /joint_states (with prefixed names)
    # Publishes to: /tf (TF frames for all robots)
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

    # MoveIt move_group - handles planning and execution for all robots
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz for visualization
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_ned2_nrobot_moveit_config"), "config", rviz_base]
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
            robot_config_arg,
            joint_state_prefixer_node,
            rsp_node,
            move_group_node,
            rviz_node,
        ]
    )
