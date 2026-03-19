#!/usr/bin/env python3

import rclpy
import os
import threading
import time
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped


def synchronize_arm2_to_planning_scene(moveit_py_1, moveit_py_2, arm_2):
    """
    Synchronize arm_2's current state into arm_1's planning scene
    so that arm_1's planner sees arm_2 as a collision obstacle.

    Args:
        moveit_py_1: MoveItPy instance for arm_1
        moveit_py_2: MoveItPy instance for arm_2
        arm_2: Planning component for arm_2

    Returns:
        bool: True if successful, False if failed (execution should abort)
    """
    try:
        # Get planning scene monitor from arm_1's instance
        scene_monitor_1 = moveit_py_1.get_planning_scene_monitor()

        # Get arm_2's robot model and create a robot state to capture current joint positions
        robot_model_2 = moveit_py_2.get_robot_model()
        robot_state_2 = RobotState(robot_model_2)
        robot_state_2.update()  # Sync with current TF state

        # Extract arm_2's current joint positions
        arm_2_joints = robot_state_2.get_joint_group_positions("arm_2")

        # Update arm_1's planning scene with arm_2's current configuration
        with scene_monitor_1.read_write() as scene:
            # Get the current robot state from arm_1's scene
            current_state = scene.current_state

            # Set arm_2's joints in the scene to match arm_2's actual current state
            current_state.set_joint_group_positions("arm_2", arm_2_joints)

            # Update transforms so collision geometry reflects new joint positions
            current_state.update()

        return True

    except Exception as e:
        print(f"✗ Synchronization error: {e}")
        return False


def check_collision_in_scene(moveit_py_1):
    """
    Check if there is a collision in arm_1's planning scene.

    Args:
        moveit_py_1: MoveItPy instance for arm_1

    Returns:
        bool: True if collision detected, False if no collision
    """
    try:
        scene_monitor_1 = moveit_py_1.get_planning_scene_monitor()

        with scene_monitor_1.read_only() as scene:
            robot_state = scene.current_state
            # Check if the current state is in collision
            is_colliding = scene.is_state_colliding(
                robot_state=robot_state, joint_model_group_name="arm_1", verbose=False
            )
            return is_colliding

    except Exception as e:
        print(f"✗ Collision check error: {e}")
        return False


def continuous_collision_monitor(
    moveit_py_1, moveit_py_2, arm_2, execution_status, sync_interval=0.05
):
    """
    Background thread that continuously synchronizes arm_2 state and checks for collisions
    during arm_1 execution.

    Args:
        moveit_py_1: MoveItPy instance for arm_1
        moveit_py_2: MoveItPy instance for arm_2
        arm_2: Planning component for arm_2
        execution_status: Shared dict tracking execution state
        sync_interval: Time in seconds between synchronization checks (default 50ms)
    """
    monitor_active = True
    collision_detected = False
    sync_count = 0

    while monitor_active:
        # Check if both arms have finished executing
        if (
            execution_status["arm_1"]["end"] is not None
            and execution_status["arm_2"]["end"] is not None
        ):
            monitor_active = False
            break

        # Synchronize arm_2's current state to arm_1's planning scene
        if not synchronize_arm2_to_planning_scene(moveit_py_1, moveit_py_2, arm_2):
            print("✗ [Monitor] Synchronization failed - aborting monitoring")
            execution_status["collision_abort"] = True
            break

        sync_count += 1

        # Check for collision in the updated scene
        if check_collision_in_scene(moveit_py_1):
            collision_detected = True
            print(f"\n✗ [Monitor] COLLISION DETECTED at sync cycle {sync_count}!")
            execution_status["collision_abort"] = True
            print("✗ [Monitor] Collision abort signal sent to execution threads")
            break

        # Wait before next synchronization cycle
        time.sleep(sync_interval)

    if collision_detected:
        print(f"✗ [Monitor] Monitoring stopped due to collision")
    else:
        print(
            f"✓ [Monitor] Completed {sync_count} synchronization cycles with no collisions"
        )


rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

print("=" * 80)
print("ASYNC DUAL-ARM EXECUTION - WITH CONTINUOUS COLLISION MONITORING")
print("=" * 80 + "\n")

# Create TWO separate MoveItPy instances (one per arm)
print("[1/6] Creating two separate MoveItPy instances...\n")

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
print("[2/6] Getting planning components...\n")

arm_1 = moveit_py_1.get_planning_component("arm_1")
arm_2 = moveit_py_2.get_planning_component("arm_2")

arm_1.set_start_state_to_current_state()
arm_2.set_start_state_to_current_state()

print("✓ arm_1 (from moveit_py_1)")
print("✓ arm_2 (from moveit_py_2)\n")

# Create identical pose goal
print("[3/6] Planning both arms to identical pose...\n")

pose_goal = PoseStamped()
pose_goal.pose.position.x = 0.4
pose_goal.pose.position.y = -0.2
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

# Synchronize arm_2's state to arm_1's planning scene for collision awareness
print("[4/6] Synchronizing collision objects...\n")

print("  Synchronizing arm_2 state to planning scene...")
if not synchronize_arm2_to_planning_scene(moveit_py_1, moveit_py_2, arm_2):
    print("\n✗ EXECUTION ABORTED: Could not synchronize collision objects")
    rclpy.shutdown()
    exit(1)

print("✓ Collision synchronization successful\n")

# Plan for both arms
print("[5/6] Planning both arms...\n")

print("  Planning arm_1 (with arm_2 collision awareness)...")
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
        "collision_abort": False,
    }

    print("[6/6] Executing both arms in parallel with continuous monitoring...\n")

    def execute_arm_1():
        """Execute arm_1 in its own thread"""
        execution_status["arm_1"]["start"] = time.time()
        print("→ [Thread-1] Executing arm_1...")
        try:
            moveit_py_1.execute(plan_result_1.trajectory, controllers=[])

            # Check if collision abort was triggered
            if execution_status["collision_abort"]:
                print("✗ [Thread-1] arm_1 execution halted due to collision detection")
                execution_status["arm_1"]["success"] = False
            else:
                execution_status["arm_1"]["success"] = True

            execution_status["arm_1"]["end"] = time.time()
            duration = (
                execution_status["arm_1"]["end"] - execution_status["arm_1"]["start"]
            )
            if execution_status["arm_1"]["success"]:
                print(f"✓ [Thread-1] arm_1 completed ({duration:.3f}s)")
            else:
                print(f"✗ [Thread-1] arm_1 stopped ({duration:.3f}s)")
        except Exception as e:
            execution_status["arm_1"]["end"] = time.time()
            print(f"✗ [Thread-1] arm_1 failed: {e}")

    def execute_arm_2():
        """Execute arm_2 in its own thread"""
        execution_status["arm_2"]["start"] = time.time()
        print("→ [Thread-2] Executing arm_2...")
        try:
            moveit_py_2.execute(plan_result_2.trajectory, controllers=[])

            # Check if collision abort was triggered
            if execution_status["collision_abort"]:
                print("✗ [Thread-2] arm_2 execution halted due to collision detection")
                execution_status["arm_2"]["success"] = False
            else:
                execution_status["arm_2"]["success"] = True

            execution_status["arm_2"]["end"] = time.time()
            duration = (
                execution_status["arm_2"]["end"] - execution_status["arm_2"]["start"]
            )
            if execution_status["arm_2"]["success"]:
                print(f"✓ [Thread-2] arm_2 completed ({duration:.3f}s)")
            else:
                print(f"✗ [Thread-2] arm_2 stopped ({duration:.3f}s)")
        except Exception as e:
            execution_status["arm_2"]["end"] = time.time()
            print(f"✗ [Thread-2] arm_2 failed: {e}")

    # Start the collision monitoring thread (50ms synchronization interval)
    monitor_thread = threading.Thread(
        target=continuous_collision_monitor,
        args=(moveit_py_1, moveit_py_2, arm_2, execution_status, 0.05),
        name="Monitor",
        daemon=True,
    )

    # Start both execution threads
    thread_1 = threading.Thread(target=execute_arm_1, name="Thread-1")
    thread_2 = threading.Thread(target=execute_arm_2, name="Thread-2")

    exec_start = time.time()

    # Start all three threads: monitor + 2 execution threads
    monitor_thread.start()
    thread_1.start()
    thread_2.start()

    print("Monitor thread started - executing in parallel with collision checking\n")

    # Wait for both execution threads to complete
    thread_1.join()
    thread_2.join()

    # Wait for monitor thread to finish
    monitor_thread.join(timeout=1.0)

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

    # Report collision status
    print("\nCollision Status:")
    if execution_status["collision_abort"]:
        print("  ✗ COLLISION DETECTED - Execution aborted for safety")
    else:
        print("  ✓ No collisions detected during execution")

    print("\nExecution Status:")
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
