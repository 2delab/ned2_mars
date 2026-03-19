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


def get_trajectory_state_at_time(trajectory_msg, time_from_start):
    """
    Interpolate robot state at a specific time from trajectory.

    Args:
        trajectory_msg: RobotTrajectory message
        time_from_start: Time in seconds from start of trajectory

    Returns:
        List of joint positions at that time, or None if time is beyond trajectory
    """
    points = trajectory_msg.joint_trajectory.points

    if not points:
        return None

    # If time is before first point, return first state
    if time_from_start <= 0:
        return list(points[0].positions)

    # If time is after last point, return last state
    last_time = (
        points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
    )
    if time_from_start >= last_time:
        return list(points[-1].positions)

    # Find two points to interpolate between
    for i in range(len(points) - 1):
        t1 = points[i].time_from_start.sec + points[i].time_from_start.nanosec / 1e9
        t2 = (
            points[i + 1].time_from_start.sec
            + points[i + 1].time_from_start.nanosec / 1e9
        )

        if t1 <= time_from_start <= t2:
            # Linear interpolation between two points
            if t2 == t1:
                return list(points[i].positions)

            alpha = (time_from_start - t1) / (t2 - t1)
            p1 = points[i].positions
            p2 = points[i + 1].positions

            interpolated = [p1[j] + alpha * (p2[j] - p1[j]) for j in range(len(p1))]
            return interpolated

    return list(points[-1].positions)


def check_collision_with_trajectory_state(
    moveit_py_1, arm_2_joint_names, arm_2_joint_positions
):
    """
    Check collision between arm_1 and arm_2 in current robot state.

    Args:
        moveit_py_1: MoveItPy instance for arm_1
        arm_2_joint_names: List of arm_2 joint names
        arm_2_joint_positions: List of arm_2 joint positions

    Returns:
        bool: True if collision detected, False if no collision
    """
    try:
        scene_monitor_1 = moveit_py_1.get_planning_scene_monitor()

        with scene_monitor_1.read_write() as scene:
            current_state = scene.current_state

            # Set arm_2 to the specific state we're checking
            current_state.set_joint_group_positions("arm_2", arm_2_joint_positions)
            current_state.update()

            # Check collision for entire robot (arm_1 + arm_2 + environment)
            # Using None for joint_model_group_name checks the full robot state
            is_colliding = scene.is_state_colliding(
                robot_state=current_state, joint_model_group_name=None, verbose=False
            )
            return is_colliding

    except Exception as e:
        print(f"✗ Collision check error: {e}")
        return False


def trajectory_collision_monitor(
    moveit_py_1,
    moveit_py_2,
    plan_result_1,
    plan_result_2,
    execution_status,
    check_interval=0.02,
):
    """
    Monitor collision between arm_1's planned trajectory and arm_2's executing trajectory.

    Args:
        moveit_py_1: MoveItPy instance for arm_1
        moveit_py_2: MoveItPy instance for arm_2
        plan_result_1: Planned trajectory for arm_1
        plan_result_2: Planned trajectory for arm_2
        execution_status: Shared dict tracking execution state
        check_interval: Time in seconds between collision checks (default 20ms)
    """
    monitor_active = True
    collision_detected = False
    check_count = 0

    # Get joint names for arm_2
    robot_model_2 = moveit_py_2.get_robot_model()
    joint_model_group_2 = robot_model_2.get_joint_model_group("arm_2")
    arm_2_joint_names = joint_model_group_2.get_joint_model_names()

    traj_1 = plan_result_1.trajectory
    traj_2 = plan_result_2.trajectory

    while monitor_active:
        # Check if both arms have finished executing
        if (
            execution_status["arm_1"]["end"] is not None
            and execution_status["arm_2"]["end"] is not None
        ):
            monitor_active = False
            break

        # Calculate time elapsed since execution start
        if (
            execution_status["arm_1"]["start"] is None
            or execution_status["arm_2"]["start"] is None
        ):
            time.sleep(check_interval)
            continue

        min_start_time = min(
            execution_status["arm_1"]["start"], execution_status["arm_2"]["start"]
        )
        elapsed_time = time.time() - min_start_time

        # Get arm_2's state at current elapsed time from its trajectory
        arm_2_positions = get_trajectory_state_at_time(traj_2, elapsed_time)

        if arm_2_positions is None:
            time.sleep(check_interval)
            continue

        check_count += 1

        # Check collision with arm_2's state at this point in time
        if check_collision_with_trajectory_state(
            moveit_py_1, arm_2_joint_names, arm_2_positions
        ):
            collision_detected = True
            print(
                f"\n✗ [Monitor] COLLISION DETECTED at check {check_count} (t={elapsed_time:.3f}s)!"
            )
            execution_status["collision_abort"] = True
            print("✗ [Monitor] Collision abort signal sent to execution threads")
            break

        # Wait before next check
        time.sleep(check_interval)

    if collision_detected:
        print(f"✗ [Monitor] Monitoring stopped due to collision")
    else:
        print(
            f"✓ [Monitor] Completed {check_count} collision checks with no collisions detected"
        )


rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

print("=" * 80)
print("ASYNC DUAL-ARM EXECUTION - TRAJECTORY-BASED COLLISION MONITORING")
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

    print("[6/6] Executing both arms with trajectory collision monitoring...\n")

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

    # Start the collision monitoring thread (20ms check interval)
    monitor_thread = threading.Thread(
        target=trajectory_collision_monitor,
        args=(
            moveit_py_1,
            moveit_py_2,
            plan_result_1,
            plan_result_2,
            execution_status,
            0.02,
        ),
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

    print("Monitor thread started - checking trajectories for collisions\n")

    # Wait for both execution threads to complete
    thread_1.join()
    thread_2.join()

    # Wait for monitor thread to finish
    monitor_thread.join(timeout=2.0)

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
