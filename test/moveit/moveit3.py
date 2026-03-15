#!/usr/bin/env python3
"""
Dual-Arm Async Movement - Move both arm_1_joint_1 and arm_2_joint_1 to 50 degrees simultaneously

This script demonstrates asynchronous dual-arm trajectory execution using ROS 2 action client callbacks.
Both arms move in parallel without waiting for each other to complete.

Prerequisites:
- MoveIt must be running with:
  ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py use_rviz:=false
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians
import sys
import time
from typing import Dict, Any

print("\n" + "=" * 70)
print("Dual-Arm Async Movement - Move both joints to 50 degrees simultaneously")
print("=" * 70)

# Shared state for tracking arm execution
start_time = time.time()
arm_status = {
    "arm_1": {
        "accepted": False,
        "completed": False,
        "error": None,
        "error_code": None,
        "accepted_time": None,
        "completed_time": None,
    },
    "arm_2": {
        "accepted": False,
        "completed": False,
        "error": None,
        "error_code": None,
        "accepted_time": None,
        "completed_time": None,
    },
}


def create_motion_plan_goal(group_name: str, target_angle: float) -> MoveGroup.Goal:
    """Create a motion plan goal for a single arm group"""
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = group_name
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Create joint constraints
    constraints_list = []

    if group_name == "arm_1":
        joint_names = [f"arm_1_joint_{i}" for i in range(1, 7)]
    else:  # arm_2
        joint_names = [f"arm_2_joint_{i}" for i in range(1, 7)]

    for i, joint_name in enumerate(joint_names):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.weight = 1.0

        if i == 0:  # First joint moves to target angle
            joint_constraint.position = radians(target_angle)
            joint_constraint.tolerance_above = radians(2)
            joint_constraint.tolerance_below = radians(2)
        else:  # All other joints locked at 0
            joint_constraint.position = 0.0
            joint_constraint.tolerance_above = radians(1)
            joint_constraint.tolerance_below = radians(1)

        constraints_list.append(joint_constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    return goal


def goal_response_callback(arm_id: str, future):
    """Called when action server accepts or rejects the goal"""
    goal_handle = future.result()
    elapsed = time.time() - start_time
    if not goal_handle.accepted:
        print(f"✗ [{elapsed:.3f}s] Goal rejected by {arm_id}")
        arm_status[arm_id]["accepted"] = False
    else:
        print(
            f"✓ [{elapsed:.3f}s] Goal accepted by {arm_id} - planning and executing..."
        )
        arm_status[arm_id]["accepted"] = True
        arm_status[arm_id]["accepted_time"] = elapsed


def result_callback(arm_id: str, future):
    """Called when trajectory execution completes"""
    result = future.result().result
    error_code = result.error_code.val
    elapsed = time.time() - start_time
    arm_status[arm_id]["completed"] = True
    arm_status[arm_id]["error_code"] = error_code
    arm_status[arm_id]["completed_time"] = elapsed

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

    if error_code == 1:
        print(f"✓ [{elapsed:.3f}s] {arm_id}: SUCCESS - moved to 50 degrees")
    else:
        print(f"✗ [{elapsed:.3f}s] {arm_id}: {error_name}")
        arm_status[arm_id]["error"] = error_name


# Initialize ROS
rclpy.init()

try:
    # Create node and action client
    print("\n[1/4] Creating move_group action client...")
    node = rclpy.create_node("moveit_dual_arm_async_test")
    action_client = ActionClient(node, MoveGroup, "/move_action")
    print("✓ Action client created\n")

    # Wait for move_group action server
    print("[2/4] Waiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=5):
        print("✗ move_group action server not available!")
        print("\nMake sure MoveIt is running:")
        print(
            "  ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py use_rviz:=false\n"
        )
        sys.exit(1)
    print("✓ Connected to move_group action server\n")

    # Create and send goals for both arms
    print("[3/4] Sending goals to both arms (async)...")

    # Create goals
    goal_arm_1 = create_motion_plan_goal("arm_1", 50)
    goal_arm_2 = create_motion_plan_goal("arm_2", 50)

    # Send goals asynchronously - BOTH sent immediately, without waiting
    print("  Sending arm_1 goal...")
    future_arm_1 = action_client.send_goal_async(goal_arm_1)
    future_arm_1.add_done_callback(lambda f: goal_response_callback("arm_1", f))

    print("  Sending arm_2 goal...")
    future_arm_2 = action_client.send_goal_async(goal_arm_2)
    future_arm_2.add_done_callback(lambda f: goal_response_callback("arm_2", f))

    print("✓ Both goals sent simultaneously\n")

    print("[4/4] Waiting for both arms to complete (async)...")
    print("  Note: Both arms are executing in parallel\n")

    # Let the event loop process both goals simultaneously
    # Spin until both goal responses are received
    while not (arm_status["arm_1"]["accepted"] and arm_status["arm_2"]["accepted"]):
        rclpy.spin_once(node, timeout_sec=0.1)

    goal_handle_1 = future_arm_1.result()
    goal_handle_2 = future_arm_2.result()

    if not goal_handle_1.accepted or not goal_handle_2.accepted:
        print("✗ One or both goals were rejected")
        sys.exit(1)

    # Get result futures and attach result callbacks
    result_future_1 = goal_handle_1.get_result_async()
    result_future_2 = goal_handle_2.get_result_async()

    result_future_1.add_done_callback(lambda future: result_callback("arm_1", future))
    result_future_2.add_done_callback(lambda future: result_callback("arm_2", future))

    # Spin until both arms complete execution
    while not (arm_status["arm_1"]["completed"] and arm_status["arm_2"]["completed"]):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Print results
    print("\n" + "=" * 70)
    print("EXECUTION SUMMARY")
    print("=" * 70)

    all_success = True
    for arm_id in ["arm_1", "arm_2"]:
        status = arm_status[arm_id]
        if status["error_code"] == 1:
            print(f"\n✓ {arm_id}: SUCCESS")
            print(f"  - Goal accepted at: {status['accepted_time']:.3f}s")
            print(f"  - Execution completed at: {status['completed_time']:.3f}s")
            print(
                f"  - Total execution time: {status['completed_time'] - status['accepted_time']:.3f}s"
            )
        else:
            print(f"\n✗ {arm_id}: FAILED")
            print(f"  - Error: {status['error']}")
            all_success = False

    # Show timing analysis
    print("\n" + "=" * 70)
    print("TIMING ANALYSIS (Parallel Execution Verification)")
    print("=" * 70)
    arm1_accept = arm_status["arm_1"]["accepted_time"]
    arm2_accept = arm_status["arm_2"]["accepted_time"]
    time_diff = abs(arm1_accept - arm2_accept)
    print(f"Arm 1 accepted at: {arm1_accept:.3f}s")
    print(f"Arm 2 accepted at: {arm2_accept:.3f}s")
    print(f"Time difference: {time_diff:.3f}s")
    if time_diff < 0.1:
        print("✓ PARALLEL EXECUTION CONFIRMED - Both arms started within 100ms!")
    else:
        print("⚠ Sequential execution detected - arms started with delay")

    print("\n" + "=" * 70)

    if all_success:
        print("✓ Both arms executed successfully!")
        print("Both arms moved to 50 degrees simultaneously (in parallel)")
        print("\n" + "=" * 70 + "\n")
    else:
        print("✗ One or both arms failed")
        print("\n" + "=" * 70 + "\n")
        sys.exit(1)

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()
    sys.exit(1)

finally:
    rclpy.shutdown()
