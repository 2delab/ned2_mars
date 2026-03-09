#!/usr/bin/env python3


import yaml
from pathlib import Path


def generate_srdf(config):
    """Generate SRDF XML from robot configuration"""

    robots = config.get("robots", [])
    groups = config.get("groups", {})
    default_home_pose = config.get("default_home_pose", {})

    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    # Start SRDF document
    srdf_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<robot name="niryo_ned2_nrobot">',
        "    <!-- SRDF Generated from robot_config.yaml -->",
        "    <!-- DO NOT EDIT MANUALLY - regenerate with generate_all.py -->",
        "",
        "    <!-- GROUPS: Named sets of joints for planning -->",
    ]

    # Create per-robot groups (including world joint for base_link)
    for robot in robots:
        robot_name = robot["name"]
        srdf_lines.append(f'    <group name="{robot_name}">')
        # Include world joint to attach base_link to world frame
        srdf_lines.append(f'        <joint name="{robot_name}_joint_world"/>')
        # Include all 6 arm joints
        for joint_idx in range(1, 7):
            srdf_lines.append(f'        <joint name="{robot_name}_joint_{joint_idx}"/>')
        srdf_lines.append("    </group>")

    srdf_lines.append("")
    srdf_lines.append("    <!-- Named groups for coordinated motion -->")

    # Create named groups from config
    for group_name, group_robots in groups.items():
        if not isinstance(group_robots, list):
            group_robots = [group_robots]

        srdf_lines.append(f'    <group name="{group_name}">')
        for robot_name in group_robots:
            # Check if this robot exists
            if any(r["name"] == robot_name for r in robots):
                srdf_lines.append(f'        <group name="{robot_name}"/>')
        srdf_lines.append("    </group>")

    srdf_lines.append("")
    srdf_lines.append("    <!-- GROUP STATES: Named poses for planning -->")

    # Create home pose for each robot
    for robot in robots:
        robot_name = robot["name"]
        srdf_lines.append(f'    <group_state name="home" group="{robot_name}">')
        for joint_idx in range(1, 7):
            joint_name = f"{robot_name}_joint_{joint_idx}"
            # Get value from default_home_pose using joint_X key
            joint_key = f"joint_{joint_idx}"
            value = default_home_pose.get(joint_key, 0.0)
            srdf_lines.append(f'        <joint name="{joint_name}" value="{value}"/>')
        srdf_lines.append("    </group_state>")

    srdf_lines.append("")
    srdf_lines.append(
        "    <!-- DISABLE COLLISIONS: Rules for robot-robot collision exclusion -->"
    )

    # Disable collisions between different robots (all link pairs)
    robot_links = [
        "base_link",
        "shoulder_link",
        "arm_link",
        "elbow_link",
        "forearm_link",
        "wrist_link",
        "hand_link",
    ]

    for i, robot1 in enumerate(robots):
        for robot2 in robots[i + 1 :]:
            robot1_name = robot1["name"]
            robot2_name = robot2["name"]

            # Disable collision for all link pairs between the two robots
            for link1 in robot_links:
                for link2 in robot_links:
                    link1_full = f"{robot1_name}_{link1}"
                    link2_full = f"{robot2_name}_{link2}"
                    reason = "Different robots"
                    srdf_lines.append(
                        f'    <disable_collisions link1="{link1_full}" link2="{link2_full}" reason="{reason}"/>'
                    )

    # Standard disable collisions within each robot (adjacent links)
    # These are added to prevent false collisions on the same arm
    adjacent_pairs = [
        ("base_link", "shoulder_link"),
        ("shoulder_link", "arm_link"),
        ("arm_link", "elbow_link"),
        ("elbow_link", "forearm_link"),
        ("forearm_link", "wrist_link"),
        ("wrist_link", "hand_link"),
    ]

    srdf_lines.append("    <!-- Adjacent link pairs within each robot -->")
    for robot in robots:
        robot_name = robot["name"]
        for link1, link2 in adjacent_pairs:
            link1_full = f"{robot_name}_{link1}"
            link2_full = f"{robot_name}_{link2}"
            srdf_lines.append(
                f'    <disable_collisions link1="{link1_full}" link2="{link2_full}" reason="Adjacent"/>'
            )

    # Add some non-adjacent but safe pairs
    safe_pairs = [
        ("base_link", "arm_link"),
        ("base_link", "elbow_link"),
        ("base_link", "forearm_link"),
        ("shoulder_link", "elbow_link"),
        ("shoulder_link", "forearm_link"),
        ("shoulder_link", "wrist_link"),
        ("arm_link", "wrist_link"),
        ("arm_link", "hand_link"),
    ]

    srdf_lines.append("    <!-- Non-adjacent safe pairs within each robot -->")
    for robot in robots:
        robot_name = robot["name"]
        for link1, link2 in safe_pairs:
            link1_full = f"{robot_name}_{link1}"
            link2_full = f"{robot_name}_{link2}"
            srdf_lines.append(
                f'    <disable_collisions link1="{link1_full}" link2="{link2_full}" reason="Never"/>'
            )

    srdf_lines.append("</robot>")

    return "\n".join(srdf_lines)


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

    # Generate SRDF
    srdf_content = generate_srdf(config)

    # Write output
    output_file = config_dir / "niryo_ned2_nrobot.srdf"
    with open(output_file, "w") as f:
        f.write(srdf_content)

    print(f"✓ Generated SRDF: {output_file}")
    print(f"  Robots: {len(config.get('robots', []))}")
    print(f"  Groups: {len(config.get('groups', {}))}")


if __name__ == "__main__":
    main()
