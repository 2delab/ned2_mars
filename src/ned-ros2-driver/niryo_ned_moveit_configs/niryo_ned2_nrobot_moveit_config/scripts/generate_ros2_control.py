#!/usr/bin/env python3

import yaml
from pathlib import Path


def generate_ros2_control(config):
    """Generate ROS2 control xacro without foreach loops"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        "    <!--",
        "    ROS2 Control configuration for N-robot system",
        "    Generated from robot_config.yaml",
        "    DO NOT EDIT MANUALLY - regenerate with scripts/generate_all.py",
        "    -->",
        "",
        "    <!-- Macro for ROS2 control fake system -->",
        '    <xacro:macro name="niryo_ned2_nrobot_ros2_control" params="name initial_positions_file">',
        '        <ros2_control name="${name}" type="system">',
        "            <!-- Fake hardware system for testing/simulation -->",
        "            <hardware>",
        "                <plugin>mock_components/GenericSystem</plugin>",
        '                <param name="fake_sensor_commands">true</param>',
        "            </hardware>",
        "",
        "            <!-- Joint interfaces for all robots -->",
    ]

    # Generate joint interfaces for each robot
    for robot in robots:
        robot_name = robot["name"]
        lines.append(f"            <!-- {robot_name}: 6 joint interfaces -->")

        for joint_idx in range(1, 7):
            joint_name = f"{robot_name}_joint_{joint_idx}"
            lines.append(f'            <joint name="{joint_name}">')
            lines.append(f'                <command_interface name="position"/>')
            lines.append(f'                <state_interface name="position">')
            lines.append(f'                    <param name="initial_value">0.0</param>')
            lines.append(f"                </state_interface>")
            lines.append(f"            </joint>")

        lines.append("")

    lines.append("        </ros2_control>")
    lines.append("    </xacro:macro>")
    lines.append("")
    lines.append("</robot>")

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

    # Generate ROS2 control xacro
    ros2_control_content = generate_ros2_control(config)

    # Write output
    output_file = config_dir / "niryo_ned2_nrobot.ros2_control.xacro"
    with open(output_file, "w") as f:
        f.write(ros2_control_content)

    num_robots = len(config.get("robots", []))
    num_joints = num_robots * 6
    print(f"✓ Generated ROS2 control xacro: {output_file}")
    print(f"  Robots: {num_robots}")
    print(f"  Total joints: {num_joints}")


if __name__ == "__main__":
    main()
