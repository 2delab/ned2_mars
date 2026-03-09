#!/usr/bin/env python3


import yaml
from pathlib import Path


def generate_ros2_controllers(config):
    """Generate ros2_controllers.yaml for ROS2 control framework"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        "# ROS2 Controllers Configuration",
        "# Generated from robot_config.yaml",
        "# DO NOT EDIT MANUALLY - regenerate with generate_all.py",
        "",
        "controller_manager:",
        "  ros__parameters:",
        "    update_rate: 100  # Hz",
        "",
    ]

    # List all controllers
    controller_names = []
    for robot in robots:
        controller_names.append(f"{robot['name']}_controller")
    controller_names.append("all_controller")
    controller_names.append("joint_state_broadcaster")

    for name in controller_names:
        lines.append(f"    {name}:")
        if name == "joint_state_broadcaster":
            lines.append("      type: joint_state_broadcaster/JointStateBroadcaster")
        elif name == "all_controller":
            lines.append(
                "      type: joint_trajectory_controller/JointTrajectoryController"
            )
        else:
            lines.append(
                "      type: joint_trajectory_controller/JointTrajectoryController"
            )
        lines.append("")

    lines.append("")

    # Per-robot controller configuration
    for robot in robots:
        robot_name = robot["name"]
        controller_name = f"{robot_name}_controller"

        lines.append(f"{controller_name}:")
        lines.append("  ros__parameters:")
        lines.append("    joints:")
        for joint_idx in range(1, 7):
            lines.append(f"      - {robot_name}_joint_{joint_idx}")

        lines.append("    command_interfaces:")
        lines.append("      - position")
        lines.append("    state_interfaces:")
        lines.append("      - position")
        lines.append("    allow_nonzero_velocity_at_trajectory_end: true")
        lines.append("")

    # Combined all_controller
    lines.append("all_controller:")
    lines.append("  ros__parameters:")
    lines.append("    joints:")
    for robot in robots:
        for joint_idx in range(1, 7):
            lines.append(f"      - {robot['name']}_joint_{joint_idx}")

    lines.append("    command_interfaces:")
    lines.append("      - position")
    lines.append("    state_interfaces:")
    lines.append("      - position")
    lines.append("    allow_nonzero_velocity_at_trajectory_end: true")

    return "\n".join(lines)


def generate_moveit_controllers(config):
    """Generate moveit_controllers.yaml for MoveIt trajectory execution"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        "# MoveIt Controller Configuration",
        "# Routes trajectories through joint_state_prefixer proxy servers",
        "# Generated from robot_config.yaml",
        "# DO NOT EDIT MANUALLY - regenerate with generate_all.py",
        "",
        "moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "",
        "moveit_simple_controller_manager:",
        "  controller_names:",
    ]

    # List controller names
    for robot in robots:
        lines.append(f"    - {robot['name']}")
    lines.append("")

    # Per-robot controller configuration
    for robot in robots:
        robot_name = robot["name"]

        lines.append(f"  {robot_name}:")
        lines.append("    type: FollowJointTrajectory")
        lines.append("    joints:")
        for joint_idx in range(1, 7):
            lines.append(f"      - {robot_name}_joint_{joint_idx}")
        lines.append("    action_ns: follow_joint_trajectory_prefixed")
        lines.append("    default: true")
        lines.append("")

    return "\n".join(lines)


def main():
    """Main entry point"""
    script_dir = Path(__file__).parent
    config_dir = script_dir.parent / "config"

    # Load robot config
    config_file = config_dir / "robot_config.yaml"
    if not config_file.exists():
        raise FileNotFoundError(f"robot_config.yaml not found at {config_file}")

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    # Generate ros2_controllers.yaml
    ros2_controllers_content = generate_ros2_controllers(config)
    ros2_controllers_file = config_dir / "ros2_controllers.yaml"
    with open(ros2_controllers_file, "w") as f:
        f.write(ros2_controllers_content)
    print(f"✓ Generated: {ros2_controllers_file}")

    # Generate moveit_controllers.yaml
    moveit_controllers_content = generate_moveit_controllers(config)
    moveit_controllers_file = config_dir / "moveit_controllers.yaml"
    with open(moveit_controllers_file, "w") as f:
        f.write(moveit_controllers_content)
    print(f"✓ Generated: {moveit_controllers_file}")

    print(f"  Total robots: {len(config.get('robots', []))}")


if __name__ == "__main__":
    main()
