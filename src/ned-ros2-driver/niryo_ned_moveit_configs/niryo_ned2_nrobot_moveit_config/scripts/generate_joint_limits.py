#!/usr/bin/env python3


import yaml
from pathlib import Path


def generate_joint_limits(config):
    """Generate joint_limits.yaml with motion constraints"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        "# Joint Limits Configuration",
        "# Velocity, acceleration, and jerk limits for each joint",
        "# Generated from robot_config.yaml",
        "# DO NOT EDIT MANUALLY - regenerate with generate_all.py",
        "",
    ]

    # Standard limits for Niryo Ned2 (based on existing config)
    joint_limits = {
        "has_velocity_limits": True,
        "max_velocity": 10.0,
        "has_acceleration_limits": True,
        "max_acceleration": 50.0,
        "has_jerk_limits": False,
    }

    # Add limits for each joint of each robot
    for robot in robots:
        robot_name = robot["name"]
        for joint_idx in range(1, 7):
            joint_name = f"{robot_name}_joint_{joint_idx}"
            lines.append(f"{joint_name}:")
            for key, value in joint_limits.items():
                if isinstance(value, bool):
                    lines.append(f"  {key}: {str(value).lower()}")
                else:
                    lines.append(f"  {key}: {value}")
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

    # Generate joint_limits.yaml
    joint_limits_content = generate_joint_limits(config)
    output_file = config_dir / "joint_limits.yaml"
    with open(output_file, "w") as f:
        f.write(joint_limits_content)

    num_robots = len(config.get("robots", []))
    num_joints = num_robots * 6
    print(f"✓ Generated: {output_file}")
    print(f"  Total joints: {num_joints} ({num_robots} robots × 6 joints)")


if __name__ == "__main__":
    main()
