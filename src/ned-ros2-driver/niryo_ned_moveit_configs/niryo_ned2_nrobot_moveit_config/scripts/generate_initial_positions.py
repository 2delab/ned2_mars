#!/usr/bin/env python3


import yaml
from pathlib import Path


def generate_initial_positions(config):
    """Generate initial_positions.yaml for ROS2 control"""

    robots = config.get("robots", [])
    default_home_pose = config.get("default_home_pose", {})

    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        "# Initial Positions for ROS2 Control Fake System",
        "# Applied when system starts for testing/simulation",
        "# Generated from robot_config.yaml",
        "# DO NOT EDIT MANUALLY - regenerate with generate_all.py",
        "",
        "initial_positions:",
    ]

    # Add initial position for each joint of each robot
    for robot in robots:
        robot_name = robot["name"]
        for joint_idx in range(1, 7):
            joint_key = f"joint_{joint_idx}"
            value = default_home_pose.get(joint_key, 0.0)
            lines.append(f"  {robot_name}_joint_{joint_idx}: {value}")

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

    # Generate initial_positions.yaml
    initial_positions_content = generate_initial_positions(config)
    output_file = config_dir / "initial_positions.yaml"
    with open(output_file, "w") as f:
        f.write(initial_positions_content)

    num_robots = len(config.get("robots", []))
    print(f"✓ Generated: {output_file}")
    print(f"  Initial positions for {num_robots} robots ({num_robots * 6} joints)")


if __name__ == "__main__":
    main()
