#!/usr/bin/env python3

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
print("ASYNC DUAL-ARM EXECUTION - TWO SEPARATE MOVEITPY INSTANCES")
print("=" * 80 + "\n")

# Create TWO separate MoveItPy instances (one per arm)
print("[1/4] Creating two separate MoveItPy instances...\n")

start_time = time.time()

moveit_py_1 = MoveItPy(
    node_name="moveit_py_arm1", launch_params_filepaths=[config_file]
)
moveit_py_2 = MoveItPy(
    node_name="moveit_py_arm2", launch_params_filepaths=[config_file]
)

print("✓ moveit_py_1 (arm_1)")
print("✓ moveit_py_2 (arm_2)\n")

# Get planning components for each arm from its own instance
print("[2/4] Getting planning components...\n")

arm_1 = moveit_py_1.get_planning_component("arm_1")
arm_2 = moveit_py_2.get_planning_component("arm_2")

arm_1.set_start_state_to_current_state()
arm_2.set_start_state_to_current_state()

print("✓ arm_1 (from moveit_py_1)")
print("✓ arm_2 (from moveit_py_2)\n")

# Create identical pose goal
print("[3/4] Planning both arms to identical pose...\n")

pose_goal = PoseStamped()
pose_goal.pose.position.x = 0.4
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2

# X-axis facing DOWN (pointing toward ground)
qx = 0.0
qy = 0.7071
qz = 0.0
qw = 0.7071

pose_goal.pose.orientation.x = qx
pose_goal.pose.orientation.y = qy
pose_goal.pose.orientation.z = qz
pose_goal.pose.orientation.w = qw

# Set goal state for arm_1
pose_goal_1 = PoseStamped()
pose_goal_1.header.frame_id = "arm_1_base_link"
pose_goal_1.pose = pose_goal.pose
arm_1.set_goal_state(pose_stamped_msg=pose_goal_1, pose_link="arm_1_tool_link")

# Set goal state for arm_2
pose_goal_2 = PoseStamped()
pose_goal_2.header.frame_id = "arm_2_base_link"
pose_goal_2.pose = pose_goal.pose
arm_2.set_goal_state(pose_stamped_msg=pose_goal_2, pose_link="arm_2_tool_link")

# Plan for both arms
print("  Planning arm_1...")
plan_time_1_start = time.time()
plan_result_1 = arm_1.plan()
plan_time_1 = time.time() - plan_time_1_start

print("  Planning arm_2...")
plan_time_2_start = time.time()
plan_result_2 = arm_2.plan()
plan_time_2 = time.time() - plan_time_2_start

if plan_result_1 and plan_result_2:
    print(f"\n✓ arm_1: Plan successful ({plan_time_1:.3f}s)")
    print(f"✓ arm_2: Plan successful ({plan_time_2:.3f}s)\n")

    # Track execution status and timing
    execution_status = {
        "arm_1": {"start": None, "end": None, "success": False},
        "arm_2": {"start": None, "end": None, "success": False},
    }

    print("[4/4] Executing both arms in parallel threads...\n")

    def execute_arm_1():
        """Execute arm_1 in its own thread"""
        execution_status["arm_1"]["start"] = time.time()
        print("→ [Thread-1] Executing arm_1...")
        try:
            moveit_py_1.execute(plan_result_1.trajectory, controllers=[])
            execution_status["arm_1"]["success"] = True
            execution_status["arm_1"]["end"] = time.time()
            duration = (
                execution_status["arm_1"]["end"] - execution_status["arm_1"]["start"]
            )
            print(f"✓ [Thread-1] arm_1 completed ({duration:.3f}s)")
        except Exception as e:
            execution_status["arm_1"]["end"] = time.time()
            print(f"✗ [Thread-1] arm_1 failed: {e}")

    def execute_arm_2():
        """Execute arm_2 in its own thread"""
        execution_status["arm_2"]["start"] = time.time()
        print("→ [Thread-2] Executing arm_2...")
        try:
            moveit_py_2.execute(plan_result_2.trajectory, controllers=[])
            execution_status["arm_2"]["success"] = True
            execution_status["arm_2"]["end"] = time.time()
            duration = (
                execution_status["arm_2"]["end"] - execution_status["arm_2"]["start"]
            )
            print(f"✓ [Thread-2] arm_2 completed ({duration:.3f}s)")
        except Exception as e:
            execution_status["arm_2"]["end"] = time.time()
            print(f"✗ [Thread-2] arm_2 failed: {e}")

    # Start both threads simultaneously
    thread_1 = threading.Thread(target=execute_arm_1, name="Thread-1")
    thread_2 = threading.Thread(target=execute_arm_2, name="Thread-2")

    exec_start = time.time()
    thread_1.start()
    thread_2.start()

    print("Both threads started - executing in parallel\n")

    # Wait for both to complete
    thread_1.join()
    thread_2.join()

    exec_duration = time.time() - exec_start

    # Print summary
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
        print(f"  Start offset: {start_offset:.3f}s (nearly simultaneous)")

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
                print("✓ Arms executed in TRUE PARALLEL (within 500ms of each other)")
            else:
                print("⚠ Arms executed with significant timing difference")

    print("\nStatus:")
    print(
        f"  arm_1: {'✓ SUCCESS' if execution_status['arm_1']['success'] else '✗ FAILED'}"
    )
    print(
        f"  arm_2: {'✓ SUCCESS' if execution_status['arm_2']['success'] else '✗ FAILED'}"
    )

    total_time = time.time() - start_time
    print(f"\nTotal script time: {total_time:.3f}s")

else:
    print("Planning failed for one or both arms")

rclpy.shutdown()
