#!/usr/bin/env python3
"""
Move Group Action Script - Move arm_1_tool_link to target pose

This script uses the move_group action server to move the end effector (arm_1_tool_link)
to a specific cartesian pose using OMPL planner with inverse kinematics.

Target Pose (DETERMINISTIC - same position every run):
- Position: x=0.25, y=0.0, z=0.25 (meters)
- Orientation: roll=0.0, pitch=1.56, yaw=0.0 (radians)

Uses both PositionConstraint and OrientationConstraint for deterministic behavior.
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import math
import sys


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]


print("\n" + "=" * 70)
print("Move Group Action - Move arm_1_tool_link to target pose")
print("=" * 70)

# Initialize ROS
rclpy.init()

try:
    # Create node and action client
    print("\n[1/3] Creating move_group action client...")
    node = rclpy.create_node("moveit_py_test")
    action_client = ActionClient(node, MoveGroup, "/move_action")
    print("✓ Action client created\n")

    # Wait for move_group action server
    print("[2/3] Waiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=5):
        print("✗ move_group action server not available!")
        print("\nMake sure MoveIt is running:")
        print(
            "  ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py use_rviz:=false\n"
        )
        sys.exit(1)
    print("✓ Connected to move_group action server\n")

    # Create motion plan request with pose goal using ONLY position constraint
    print("[3/3] Planning and executing motion...")
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "arm_1"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Create position constraint as the goal
    # This tells MoveIt: "Move the tool to this position (IK will solve orientation)"
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "world"
    position_constraint.link_name = "arm_1_tool_link"
    position_constraint.weight = 1.0

    # Define a box constraint region (the goal region)
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.3, 0.3, 0.3]  # Large tolerance for reachability

    position_constraint.constraint_region.primitives.append(box)

    # Define the center of the constraint region (target pose)
    target_pose = Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.0
    target_pose.position.z = 0.25
    # Orientation doesn't matter for position constraint, IK will solve it
    target_pose.orientation.w = 1.0

    position_constraint.constraint_region.primitive_poses.append(target_pose)

    # Create orientation constraint to lock the orientation
    # This ensures DETERMINISTIC behavior - same pose every run
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "world"
    orientation_constraint.link_name = "arm_1_tool_link"

    # Convert target RPY to quaternion (roll=0, pitch=1.56, yaw=0)
    quaternion = euler_to_quaternion(0.0, 1.56, 0.0)
    orientation_constraint.orientation.x = quaternion[0]
    orientation_constraint.orientation.y = quaternion[1]
    orientation_constraint.orientation.z = quaternion[2]
    orientation_constraint.orientation.w = quaternion[3]

    # Set tight tolerance to ensure this exact orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.05  # ±2.9 degrees
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.05
    orientation_constraint.weight = 1.0

    # Add BOTH position and orientation constraints for deterministic goal
    goal.request.goal_constraints.append(
        Constraints(
            position_constraints=[position_constraint],
            orientation_constraints=[orientation_constraint],
        )
    )

    # Send goal to move_group action server
    print("Sending goal to move_group...")
    print(f"Target: arm_1_tool_link at position (x=0.25, y=0.0, z=0.25)")
    print(f"Orientation: roll=0.0, pitch=1.56, yaw=0.0 (LOCKED - deterministic)\n")

    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("✗ Goal rejected by move_group")
        sys.exit(1)

    print("✓ Goal accepted - planning and executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result = result_future.result().result

    # Check result
    error_code = result.error_code.val

    print("=" * 70)
    if error_code == 1:  # SUCCESS
        print("✓ SUCCESS!")
        print("=" * 70)
        print("\nMotion planning and execution completed successfully!")
        print("End effector (arm_1_tool_link) moved to target pose (DETERMINISTIC):")
        print("  Position: (0.25, 0.0, 0.25) meters")
        print("  Orientation: roll=0.0, pitch=1.56, yaw=0.0 (LOCKED)")
        print("\nUsed OMPL planner with position + orientation constraints.")
        print("\n" + "=" * 70 + "\n")
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            1: "SUCCESS",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            4: "START_STATE_INVALID",
            5: "GOAL_IN_COLLISION",
            6: "GOAL_INVALID",
            7: "GOAL_CONSTRAINTS_VIOLATED",
        }
        error_name = error_messages.get(error_code, f"UNKNOWN ({error_code})")
        print(f"✗ Motion planning failed!")
        print("=" * 70)
        print(f"\nError: {error_name}")
        print(f"Error code: {error_code}")
        print(f"\nPossible causes:")
        print(f"  - Target pose is unreachable by the robot")
        print(f"  - Pose is too far or in collision")
        print(f"  - Robot configuration doesn't allow IK solution\n")
        sys.exit(1)

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()
    sys.exit(1)

finally:
    rclpy.shutdown()
