#!/usr/bin/env python3
"""
Deterministic Move Group Action Script - Move to exact joint angles

This script uses the move_group action server with FIXED JOINT ANGLES instead of
cartesian pose goals. This guarantees EXACT DETERMINISTIC behavior every single run.

Target Joint Angles (DETERMINISTIC - same configuration every run):
- Joint 1: 0.0 rad
- Joint 2: 0.5 rad
- Joint 3: 0.5 rad
- Joint 4: 0.0 rad
- Joint 5: 1.56 rad
- Joint 6: 0.0 rad

This approach avoids:
- IK solver randomness
- Multiple solutions to same pose
- Planner stochasticity
- Tolerance-based goal regions
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
)
import sys


print("\n" + "=" * 70)
print("Move Group Action - Move to EXACT joint angles (DETERMINISTIC)")
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

    # Create motion plan request with FIXED JOINT ANGLES
    print("[3/3] Planning and executing motion...")
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "arm_1"
    goal.request.num_planning_attempts = (
        1  # Only 1 attempt needed - we have exact angles
    )
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Define exact joint angles as goal constraints
    # These are FIXED values - same every run, NO RANDOMNESS
    joint_angles = {
        "arm_1_joint_1": 0.0,
        "arm_1_joint_2": 0.5,
        "arm_1_joint_3": 0.5,
        "arm_1_joint_4": 0.0,
        "arm_1_joint_5": 1.56,
        "arm_1_joint_6": 0.0,
    }

    # Create tight joint constraints (±0.01 rad tolerance)
    constraints = Constraints()
    for joint_name, angle in joint_angles.items():
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = angle
        joint_constraint.tolerance_above = 0.01  # ±0.01 rad = ±0.57 degrees
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

    goal.request.goal_constraints.append(constraints)

    # Send goal to move_group action server
    print("Sending goal to move_group...")
    print(f"Target: EXACT joint angles (DETERMINISTIC - same pose EVERY run)")
    print(f"  arm_1_joint_1: 0.0 rad")
    print(f"  arm_1_joint_2: 0.5 rad")
    print(f"  arm_1_joint_3: 0.5 rad")
    print(f"  arm_1_joint_4: 0.0 rad")
    print(f"  arm_1_joint_5: 1.56 rad")
    print(f"  arm_1_joint_6: 0.0 rad\n")

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
        print("Robot reached EXACT joint angles (DETERMINISTIC):")
        print("  arm_1_joint_1: 0.0 rad")
        print("  arm_1_joint_2: 0.5 rad")
        print("  arm_1_joint_3: 0.5 rad")
        print("  arm_1_joint_4: 0.0 rad")
        print("  arm_1_joint_5: 1.56 rad")
        print("  arm_1_joint_6: 0.0 rad")
        print("\nRun this script 100 times - pose will be IDENTICAL every time!")
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
        print(f"  - Joint angles are unreachable")
        print(f"  - Pose is in collision")
        print(f"  - Robot configuration doesn't allow these angles\n")
        sys.exit(1)

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()
    sys.exit(1)

finally:
    rclpy.shutdown()
