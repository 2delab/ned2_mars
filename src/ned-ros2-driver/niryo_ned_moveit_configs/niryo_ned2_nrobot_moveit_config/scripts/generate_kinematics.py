#!/usr/bin/env python3

import yaml
from pathlib import Path


def generate_kinematics(config):
    """Generate kinematics.yaml with KDL solvers"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        "# Kinematics Configuration",
        "# KDL solver for each robot group",
        "# Generated from robot_config.yaml",
        "# DO NOT EDIT MANUALLY - regenerate with generate_all.py",
        "",
    ]

    # Add kinematics config for each robot
    for robot in robots:
        robot_name = robot["name"]
        lines.append(f"{robot_name}:")
        lines.append("  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin")
        lines.append("  kinematics_solver_search_resolution: 0.005")
        lines.append("  kinematics_solver_timeout: 0.005")
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

    # Generate kinematics.yaml
    kinematics_content = generate_kinematics(config)
    output_file = config_dir / "kinematics.yaml"
    with open(output_file, "w") as f:
        f.write(kinematics_content)

    print(f"✓ Generated: {output_file}")
    print(f"  Robots with KDL solver: {len(config.get('robots', []))}")


if __name__ == "__main__":
    main()
