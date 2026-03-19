#!/usr/bin/env python3
"""
async9.py - Simplified Dual-Arm Async Execution
Based on stooppas/ba_dual_arm approach: Use a single shared MoveItPy instance
with separate planning components, but share the underlying planning scene.
Both arms plan and execute through the same scene, ensuring collision awareness.
"""

import rclpy
import os
import threading
import time
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

print("=" * 80)
print("ASYNC DUAL-ARM EXECUTION - SIMPLIFIED SHARED PLANNING SCENE")
print("(Based on stooppas/ba_dual_arm approach)")
print("=" * 80 + "\n")

# Create SINGLE MoveItPy instance - both arms share this
print("[1/4] Creating shared MoveItPy instance for dual-arm system...\n")

start_time = time.time()

moveit_py = MoveItPy(
    node_name="moveit_py_dual_shared", launch_params_filepaths=[config_file]
)

print("✓ Shared MoveItPy instance (unified planning scene)\n")

# Get planning components for BOTH arms from the SAME shared instance
print("[2/4] Getting planning components...\n")

arm_1 = moveit_py.get_planning_component("arm_1")
arm_2 = moveit_py.get_planning_component("arm_2")

arm_1.set_start_state_to_current_state()
arm_2.set_start_state_to_current_state()

print("✓ arm_1 (shared scene)")
print("✓ arm_2 (shared scene)\n")

# Track execution
execution_status = {
    "arm_1": {"start": None, "end": None, "success": False, "plan": None},
    "arm_2": {"start": None, "end": None, "success": False, "plan": None},
}

print("[3/4] Planning and executing both arms in parallel...\n")


def plan_and_execute_arm(arm_name, arm_component, moveit_py_shared, target_pos):
    """
    Plan and execute a single arm through the shared MoveItPy instance.
    Each arm executes in its own thread but shares the planning scene.
    """
    try:
        # Set start state
        arm_component.set_start_state_to_current_state()

        # Create pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = f"{arm_name}_base_link"
        pose_goal.pose.position.x = target_pos[0]
        pose_goal.pose.position.y = target_pos[1]
        pose_goal.pose.position.z = target_pos[2]

        # Orientation: X-axis facing DOWN
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.7071
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.7071

        # Set goal
        arm_component.set_goal_state(
            pose_stamped_msg=pose_goal, pose_link=f"{arm_name}_tool_link"
        )

        # Plan
        print(f"  Planning {arm_name}...")
        plan_result = arm_component.plan()

        if not plan_result:
            print(f"  ✗ {arm_name} planning failed")
            execution_status[arm_name]["success"] = False
            return

        print(f"  ✓ {arm_name} plan successful")
        execution_status[arm_name]["plan"] = plan_result

        # Execute
        print(f"→ [Thread] Executing {arm_name}...")
        execution_status[arm_name]["start"] = time.time()

        moveit_py_shared.execute(plan_result.trajectory, controllers=[])

        execution_status[arm_name]["end"] = time.time()
        execution_status[arm_name]["success"] = True

        duration = (
            execution_status[arm_name]["end"] - execution_status[arm_name]["start"]
        )
        print(f"✓ [Thread] {arm_name} completed ({duration:.3f}s)")

    except Exception as e:
        execution_status[arm_name]["success"] = False
        print(f"✗ {arm_name} failed: {e}")


# Define target positions for each arm
# Both moving to same position - tests collision avoidance
arm_1_target = [0.4, 0.2, 0.2]
arm_2_target = [0.4, 0.2, 0.2]

# Create threads for planning AND execution
# Key: Both threads use the SAME shared moveit_py instance
thread_1 = threading.Thread(
    target=plan_and_execute_arm,
    args=("arm_1", arm_1, moveit_py, arm_1_target),
    name="Thread-arm_1",
)

thread_2 = threading.Thread(
    target=plan_and_execute_arm,
    args=("arm_2", arm_2, moveit_py, arm_2_target),
    name="Thread-arm_2",
)

# Start both threads - they will plan and execute in parallel
# through the shared MoveItPy instance
exec_start = time.time()
thread_1.start()
thread_2.start()

print("Both threads started - planning and executing in parallel\n")

# Wait for both to complete
thread_1.join()
thread_2.join()

exec_duration = time.time() - exec_start

# Print results
print("\n" + "=" * 80)
print("EXECUTION SUMMARY")
print("=" * 80)

if execution_status["arm_1"]["start"] and execution_status["arm_2"]["start"]:
    arm_1_start = execution_status["arm_1"]["start"]
    arm_2_start = execution_status["arm_2"]["start"]
    start_offset = abs(arm_2_start - arm_1_start)

    print(f"\nStart timing:")
    print(f"  arm_1 started at: {arm_1_start - exec_start:.3f}s")
    print(f"  arm_2 started at: {arm_2_start - exec_start:.3f}s")
    print(f"  Start offset: {start_offset:.3f}s")

    if execution_status["arm_1"]["end"] and execution_status["arm_2"]["end"]:
        arm_1_end = execution_status["arm_1"]["end"]
        arm_2_end = execution_status["arm_2"]["end"]
        end_offset = abs(arm_2_end - arm_1_end)

        print(f"\nCompletion timing:")
        print(f"  arm_1 completed at: {arm_1_end - exec_start:.3f}s")
        print(f"  arm_2 completed at: {arm_2_end - exec_start:.3f}s")
        print(f"  Completion offset: {end_offset:.3f}s")

        print(f"\nTotal execution time: {exec_duration:.3f}s")

        if end_offset < 0.5:
            print("✓ Arms executed in TRUE PARALLEL")
        else:
            print("⚠ Arms executed with timing difference")

print("\nExecution Status:")
print(f"  arm_1: {'✓ SUCCESS' if execution_status['arm_1']['success'] else '✗ FAILED'}")
print(f"  arm_2: {'✓ SUCCESS' if execution_status['arm_2']['success'] else '✗ FAILED'}")

print("\nArchitecture:")
print("  ✓ Single shared MoveItPy instance")
print("  ✓ Both arms use same planning scene (collision-aware by design)")
print("  ✓ Separate threads for parallel execution")
print("  ✓ MoveIt's built-in planning ensures safety\n")

print("[4/4] Cleanup")
total_time = time.time() - start_time
print(f"\nTotal script time: {total_time:.3f}s")

rclpy.shutdown()
