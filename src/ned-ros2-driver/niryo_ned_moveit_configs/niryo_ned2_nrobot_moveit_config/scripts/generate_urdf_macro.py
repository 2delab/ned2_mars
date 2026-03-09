#!/usr/bin/env python3


import yaml
import math
from pathlib import Path


def load_arm_assembly_macro():
    """Load the ned2_arm_assembly_nrobot macro from existing URDF"""
    # This is the macro definition from the original URDF
    macro = """    <!-- ==================== EXTENDED ARM ASSEMBLY MACRO ==================== -->
    <!-- Enhanced version of ned2_arm_assembly that supports x_offset, y_offset, and yaw_rotation -->
    <xacro:macro name="ned2_arm_assembly_nrobot" params="prefix x_offset y_offset yaw_rotation">
        <!-- ========== WORLD TO ARM BASE JOINT ========== -->
        <!-- Fixed joint positioning this arm relative to world origin -->
        <joint name="${prefix}joint_world" type="fixed">
            <parent link="world"/>
            <child link="${prefix}base_link"/>
            <origin xyz="${x_offset} ${y_offset} 0" rpy="0 0 ${yaw_rotation}"/>
        </joint>

        <!-- ========== ARM KINEMATICS ========== -->
        <!-- LED Ring Link -->
        <link name="${prefix}led_ring_link"/>

        <!-- Base Link -->
        <link name="${prefix}base_link">
            <inertial>
                <origin xyz="-0.008924 0.0001357 0.052392" rpy="0 0 0"/>
                <mass value="0.71142"/>
                <inertia ixx="0.0017" ixy="0.0" ixz="0.0" iyy="0.0017" iyz="0.0" izz="0.0032"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/base_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="-${PI/2} 0 0"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/base_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Shoulder Link -->
        <link name="${prefix}shoulder_link">
            <inertial>
                <origin xyz="-0.031951 0.0080419 0.030675" rpy="0 0 0"/>
                <mass value="0.35056"/>
                <inertia ixx="0.00023875" ixy="2.3853E-08" ixz="2.0596E-06" iyy="0.00032638" iyz="-8.9319E-07"
                         izz="0.00030089"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/shoulder_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/shoulder_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Arm Link -->
        <link name="${prefix}arm_link">
            <inertial>
                <origin xyz="-0.00096976 0.086432 0.0038832" rpy="0 ${PI} 0"/>
                <mass value="1.0838"/>
                <inertia ixx="0.008194" ixy="0.00015602" ixz="-3.434E-06" iyy="0.0011945" iyz="-0.00031298" izz="0.007915"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 ${PI} 0"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/arm_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 ${PI} 0"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/arm_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Elbow Link -->
        <link name="${prefix}elbow_link">
            <inertial>
                <origin xyz="-0.019703 0.037336 -1.7431E-09" rpy="0 ${PI} 0"/>
                <mass value="0.22126"/>
                <inertia ixx="0.00011754" ixy="-1.2314E-05" ixz="-6.2064E-11" iyy="0.00020851" iyz="9.2393E-11"
                         izz="0.00022753"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 ${PI} ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/elbow_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 ${PI} ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/elbow_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Forearm Link -->
        <link name="${prefix}forearm_link">
            <inertial>
                <origin xyz="-0.0049532 7.8351E-06 0.08106" rpy="0 0 0"/>
                <mass value="0.35686"/>
                <inertia ixx="0.0013664" ixy="-9.6367E-08" ixz="0.00013594" iyy="0.0014781" iyz="-1.4596E-07"
                         izz="0.00023715"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/forearm_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/forearm_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Wrist Link -->
        <link name="${prefix}wrist_link">
            <inertial>
                <origin xyz="-0.019666 0.037312 0.0" rpy="0 0 0"/>
                <mass value="0.22126"/>
                <inertia ixx="0.0015" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.0015"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/wrist_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/wrist_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Hand Link -->
        <link name="${prefix}hand_link">
            <inertial>
                <origin xyz="0 0 0.009" rpy="0 0 0"/>
                <mass value="0.0070027"/>
                <inertia ixx="0.0015" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.0015"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/collada/hand_link.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 ${-PI/2}"/>
                <geometry>
                    <mesh filename="package://niryo_ned_description/meshes/ned2/stl/hand_link.stl"/>
                </geometry>
            </collision>
        </link>

        <!-- Tool Link (End Effector) -->
        <link name="${prefix}tool_link"/>

        <!-- ========== ARM JOINTS ========== -->

        <!-- Joint 1 - Shoulder Rotation -->
        <joint name="${prefix}joint_1" type="revolute">
            <origin xyz="0 0 0.1013" rpy="0 0 0"/>
            <parent link="${prefix}base_link"/>
            <child link="${prefix}shoulder_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="10.0" velocity="10.0" lower="${(limit_low_shoulder_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_shoulder_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Joint 2 - Arm Rotation -->
        <joint name="${prefix}joint_2" type="revolute">
            <origin xyz="0 0 0.065" rpy="${PI/2} 0 0"/>
            <parent link="${prefix}shoulder_link"/>
            <child link="${prefix}arm_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="10.0" velocity="8.0" lower="${(limit_low_arm_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_arm_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Joint 3 - Elbow Rotation -->
        <joint name="${prefix}joint_3" type="revolute">
            <origin xyz="0.012 0.221 0" rpy="0 0 ${PI/2}"/>
            <parent link="${prefix}arm_link"/>
            <child link="${prefix}elbow_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="7.0" velocity="8.0" lower="${(limit_low_elbow_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_elbow_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Joint 4 - Forearm Rotation -->
        <joint name="${prefix}joint_4" type="revolute">
            <origin xyz="0.0325 -0.065 0" rpy="${PI/2} 0 0"/>
            <parent link="${prefix}elbow_link"/>
            <child link="${prefix}forearm_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="7.0" velocity="2.0" lower="${(limit_low_forearm_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_forearm_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Joint 5 - Wrist Rotation -->
        <joint name="${prefix}joint_5" type="revolute">
            <origin xyz="0 0 0.17" rpy="${-PI/2} 0 0"/>
            <parent link="${prefix}forearm_link"/>
            <child link="${prefix}wrist_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="6.0" velocity="2.0" lower="${(limit_low_wrist_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_wrist_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Joint 6 - Hand Rotation -->
        <joint name="${prefix}joint_6" type="revolute">
            <origin xyz="0.00925 -0.0197 0" rpy="${PI/2} 0 0"/>
            <parent link="${prefix}wrist_link"/>
            <child link="${prefix}hand_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="5.0" velocity="2.0" lower="${(limit_low_hand_rotation + safety_pos_margin) * deg_to_rad}"
                   upper="${(limit_up_hand_rotation - safety_pos_margin) * deg_to_rad}"/>
        </joint>

        <!-- Tool Joint - End Effector -->
        <joint name="${prefix}joint_tool" type="fixed">
            <origin xyz="0 0 ${distance_hand_tool}" rpy="${PI} -${PI/2} 0"/>
            <parent link="${prefix}hand_link"/>
            <child link="${prefix}tool_link"/>
        </joint>

        <!-- LED Ring Joint -->
        <joint name="${prefix}joint_led_ring" type="fixed">
            <parent link="world"/>
            <child link="${prefix}led_ring_link"/>
            <origin xyz="${x_offset} ${y_offset} 0.0923" rpy="0 0 ${yaw_rotation}"/>
        </joint>

    </xacro:macro>
"""
    return macro


def generate_urdf(config):
    """Generate parametric URDF with conditional robot instantiation"""

    robots = config.get("robots", [])
    if not robots:
        raise ValueError("No robots defined in robot_config.yaml")

    num_robots = len(robots)
    if num_robots > 16:
        raise ValueError(f"Too many robots ({num_robots}). Maximum supported is 16.")

    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="niryo_ned2_nrobot">',
        "    <!-- ",
        "    Parametric URDF for N-robot Niryo Ned2 system",
        "    Generated from robot_config.yaml",
        "    DO NOT EDIT MANUALLY - regenerate with scripts/generate_all.py",
        "    -->",
        "",
        "    <!-- ==================== INCLUDES ==================== -->",
        "    <!-- Import common parameters and constants -->",
        '    <xacro:include filename="$(find niryo_ned_description)/urdf/ned2/niryo_ned2_param.urdf.xacro"/>',
        "    ",
        "    <!-- Import ROS2 control macro -->",
        '    <xacro:include filename="niryo_ned2_nrobot.ros2_control.xacro"/>',
        "",
        "",
        "    <!-- ==================== WORLD LINK ==================== -->",
        "    <!-- Common reference frame for all robots -->",
        '    <link name="world"/>',
        "",
        "",
    ]

    # Add the arm assembly macro
    lines.append(load_arm_assembly_macro())
    lines.append("")
    lines.append("")

    # Add instantiation section
    lines.append(
        "    <!-- ==================== INSTANTIATE N ROBOTS ==================== -->"
    )
    lines.append(
        "    <!-- Robots are instantiated conditionally based on robot_config.yaml -->"
    )
    lines.append("")

    # Generate instantiation code for each robot
    for i, robot in enumerate(robots, 1):
        robot_name = robot["name"]
        angle_deg = robot["angle_deg"]
        distance_m = robot["distance_m"]

        # Calculate offsets
        angle_rad = math.radians(angle_deg)
        x_offset = distance_m * math.cos(angle_rad)
        y_offset = distance_m * math.sin(angle_rad)

        lines.append(f"    <!-- Robot {i}: {robot_name} @ {angle_deg}° -->")
        lines.append(f"    <xacro:ned2_arm_assembly_nrobot")
        lines.append(f'        prefix="{robot_name}_"')
        lines.append(f'        x_offset="{x_offset:.6f}"')
        lines.append(f'        y_offset="{y_offset:.6f}"')
        lines.append(f'        yaw_rotation="{angle_rad:.6f}"')
        lines.append(f"    />")
        lines.append("")

    lines.append("")
    lines.append(
        "    <!-- ==================== ROS2 CONTROL INTEGRATION ==================== -->"
    )
    lines.append("    <!-- Fake system for simulation/testing -->")
    lines.append("    <xacro:niryo_ned2_nrobot_ros2_control")
    lines.append('        name="FakeSystem"')
    lines.append(
        '        initial_positions_file="$(find niryo_ned2_nrobot_moveit_config)/config/initial_positions.yaml"'
    )
    lines.append("    />")
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

    # Generate URDF
    urdf_content = generate_urdf(config)

    # Write output
    output_file = config_dir / "niryo_ned2_nrobot.urdf.xacro"
    with open(output_file, "w") as f:
        f.write(urdf_content)

    num_robots = len(config.get("robots", []))
    print(f"✓ Generated URDF: {output_file}")
    print(f"  Robots: {num_robots}")
    for robot in config.get("robots", []):
        print(f"    - {robot['name']}: {robot['angle_deg']}° @ {robot['distance_m']}m")


if __name__ == "__main__":
    main()
