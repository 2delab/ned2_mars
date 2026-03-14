#!/usr/bin/env python3
"""
Minimal Move Group Action Script - Move arm_1_joint_1 to 50 degrees

This script uses the move_group action server to move arm_1_joint_1 to 50 degrees.
Uses OMPL planner which is already configured in move_group.

Prerequisites:
- MoveIt must be running with:
  ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py use_rviz:=false
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians
import sys

print("\n" + "=" * 70)
print("Move Group Action - Move arm_1_joint_1 to 50 degrees")
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

    # Create motion plan request with joint goal
    print("[3/3] Planning and executing motion...")
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "arm_1"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Set joint goals: arm_1_joint_1 = 50 degrees, all others locked at 0
    constraints_list = []

    for i in range(1, 7):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = f"arm_1_joint_{i}"
        joint_constraint.weight = 1.0

        if i == 1:
            # arm_1_joint_1 moves to 50 degrees
            joint_constraint.position = radians(50)
            joint_constraint.tolerance_above = radians(2)
            joint_constraint.tolerance_below = radians(2)
        else:
            # All other joints locked at 0 degrees
            joint_constraint.position = 0.0
            joint_constraint.tolerance_above = radians(1)
            joint_constraint.tolerance_below = radians(1)

        constraints_list.append(joint_constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    # Send goal to move_group action server
    print("Sending goal to move_group...")
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
        print("Robot arm_1_joint_1 moved to 50 degrees using OMPL planner.")
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
        print(f"Error code: {error_code}\n")
        sys.exit(1)

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    sys.exit(1)

finally:
    rclpy.shutdown()
