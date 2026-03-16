#!/usr/bin/env python3
"""
Multi-trajectory async executor test - Execute 3 trajectories on both arms in parallel.

Trajectories:
    1. Move joint_1 to 50 degrees (both arms)
    2. Move joint_2 to -35 degrees (both arms)
    3. Sweep joint_1 from 50 to -50 degrees (both arms, joint_2 stays at -35)

All trajectories sent simultaneously without blocking - true parallel execution.

Prerequisites:
    Terminal 1: ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py use_rviz:=false
    Terminal 2: python3 test_async1.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import radians
import time
import threading
from functools import partial

# Import the async executor library
from async_executor_lib import AsyncDualArmExecutor

print("\n" + "=" * 80)
print("Multi-Trajectory Async Executor Test - 3 steps on both arms in PARALLEL")
print("=" * 80)

# Track results for all 6 parallel executions
start_time = time.time()
results = {
    "arm_1_step_1": {"completed": False, "success": False, "error": None, "time": None},
    "arm_1_step_2": {"completed": False, "success": False, "error": None, "time": None},
    "arm_1_step_3": {"completed": False, "success": False, "error": None, "time": None},
    "arm_2_step_1": {"completed": False, "success": False, "error": None, "time": None},
    "arm_2_step_2": {"completed": False, "success": False, "error": None, "time": None},
    "arm_2_step_3": {"completed": False, "success": False, "error": None, "time": None},
}


def on_execution_complete(
    execution_id: str, arm_id: str, success: bool, error_msg: str
):
    """Callback when execution completes"""
    elapsed = time.time() - start_time
    results[execution_id]["completed"] = True
    results[execution_id]["success"] = success
    results[execution_id]["error"] = error_msg
    results[execution_id]["time"] = elapsed

    if success:
        print(f"✓ [{elapsed:.3f}s] {execution_id}: SUCCESS")
    else:
        print(f"✗ [{elapsed:.3f}s] {execution_id}: FAILED - {error_msg}")


# Initialize ROS
rclpy.init()

try:
    print("\n[1/4] Creating node and async executor...")
    node = rclpy.create_node("test_async1")
    executor = AsyncDualArmExecutor(node)

    # Start spinning in background immediately
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)
    executor_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    executor_thread.start()
    print("✓ Executor created and spinning\n")

    print("[2/4] Waiting for action servers to become available...")
    # Wait for servers to be ready
    timeout = 5.0
    start_wait = time.time()
    arm_1_ready = executor.arm_1_client.wait_for_server(timeout_sec=timeout)
    arm_2_ready = executor.arm_2_client.wait_for_server(timeout_sec=timeout)

    if not arm_1_ready or not arm_2_ready:
        print("✗ Action servers not available!")
        raise RuntimeError("Action servers not ready")
    print("✓ Action servers available\n")

    print("[3/4] Creating 3 trajectory steps for both arms...")

    # ==================== STEP 1: joint_1 to 50 degrees ====================
    print("\n  Step 1: joint_1 → 50°")

    # Arm 1 - Step 1
    traj_1_step_1 = JointTrajectory()
    traj_1_step_1.joint_names = [
        "arm_1_joint_1",
        "arm_1_joint_2",
        "arm_1_joint_3",
        "arm_1_joint_4",
        "arm_1_joint_5",
        "arm_1_joint_6",
    ]
    p1_s1 = JointTrajectoryPoint()
    p1_s1.positions = [float(radians(50)), 0.0, 0.0, 0.0, 0.0, 0.0]
    p1_s1.time_from_start = Duration(sec=1)
    traj_1_step_1.points.append(p1_s1)

    # Arm 2 - Step 1
    traj_2_step_1 = JointTrajectory()
    traj_2_step_1.joint_names = [
        "arm_2_joint_1",
        "arm_2_joint_2",
        "arm_2_joint_3",
        "arm_2_joint_4",
        "arm_2_joint_5",
        "arm_2_joint_6",
    ]
    p2_s1 = JointTrajectoryPoint()
    p2_s1.positions = [float(radians(50)), 0.0, 0.0, 0.0, 0.0, 0.0]
    p2_s1.time_from_start = Duration(sec=1)
    traj_2_step_1.points.append(p2_s1)

    # ==================== STEP 2: joint_2 to -35 degrees ====================
    print("  Step 2: joint_2 → -35°")

    # Arm 1 - Step 2
    traj_1_step_2 = JointTrajectory()
    traj_1_step_2.joint_names = [
        "arm_1_joint_1",
        "arm_1_joint_2",
        "arm_1_joint_3",
        "arm_1_joint_4",
        "arm_1_joint_5",
        "arm_1_joint_6",
    ]
    p1_s2 = JointTrajectoryPoint()
    p1_s2.positions = [float(radians(50)), float(radians(-35)), 0.0, 0.0, 0.0, 0.0]
    p1_s2.time_from_start = Duration(sec=1)
    traj_1_step_2.points.append(p1_s2)

    # Arm 2 - Step 2
    traj_2_step_2 = JointTrajectory()
    traj_2_step_2.joint_names = [
        "arm_2_joint_1",
        "arm_2_joint_2",
        "arm_2_joint_3",
        "arm_2_joint_4",
        "arm_2_joint_5",
        "arm_2_joint_6",
    ]
    p2_s2 = JointTrajectoryPoint()
    p2_s2.positions = [float(radians(50)), float(radians(-35)), 0.0, 0.0, 0.0, 0.0]
    p2_s2.time_from_start = Duration(sec=1)
    traj_2_step_2.points.append(p2_s2)

    # ==================== STEP 3: Sweep joint_1 from 50 to -50 ====================
    print("  Step 3: Sweep joint_1 50° → -50° (joint_2 stays at -35°)\n")

    # Arm 1 - Step 3
    traj_1_step_3 = JointTrajectory()
    traj_1_step_3.joint_names = [
        "arm_1_joint_1",
        "arm_1_joint_2",
        "arm_1_joint_3",
        "arm_1_joint_4",
        "arm_1_joint_5",
        "arm_1_joint_6",
    ]
    p1_s3 = JointTrajectoryPoint()
    p1_s3.positions = [float(radians(-50)), float(radians(-35)), 0.0, 0.0, 0.0, 0.0]
    p1_s3.time_from_start = Duration(sec=1)
    traj_1_step_3.points.append(p1_s3)

    # Arm 2 - Step 3
    traj_2_step_3 = JointTrajectory()
    traj_2_step_3.joint_names = [
        "arm_2_joint_1",
        "arm_2_joint_2",
        "arm_2_joint_3",
        "arm_2_joint_4",
        "arm_2_joint_5",
        "arm_2_joint_6",
    ]
    p2_s3 = JointTrajectoryPoint()
    p2_s3.positions = [float(radians(-50)), float(radians(-35)), 0.0, 0.0, 0.0, 0.0]
    p2_s3.time_from_start = Duration(sec=1)
    traj_2_step_3.points.append(p2_s3)

    print("✓ All trajectories created\n")

    print("[4/4] Executing all 6 trajectories asynchronously (in PARALLEL)...\n")

    # Send all 6 trajectories immediately without blocking
    print("  Sending arm_1 step_1 trajectory...")
    executor.execute_async(
        "arm_1",
        traj_1_step_1,
        partial(on_execution_complete, "arm_1_step_1"),
    )

    print("  Sending arm_1 step_2 trajectory...")
    executor.execute_async(
        "arm_1",
        traj_1_step_2,
        partial(on_execution_complete, "arm_1_step_2"),
    )

    print("  Sending arm_1 step_3 trajectory...")
    executor.execute_async(
        "arm_1",
        traj_1_step_3,
        partial(on_execution_complete, "arm_1_step_3"),
    )

    print("  Sending arm_2 step_1 trajectory...")
    executor.execute_async(
        "arm_2",
        traj_2_step_1,
        partial(on_execution_complete, "arm_2_step_1"),
    )

    print("  Sending arm_2 step_2 trajectory...")
    executor.execute_async(
        "arm_2",
        traj_2_step_2,
        partial(on_execution_complete, "arm_2_step_2"),
    )

    print("  Sending arm_2 step_3 trajectory...")
    executor.execute_async(
        "arm_2",
        traj_2_step_3,
        partial(on_execution_complete, "arm_2_step_3"),
    )

    print("  Sending arm_1 step_2 trajectory...")
    executor.execute_async(
        "arm_1", traj_1_step_2, lambda s, e: on_execution_complete("arm_1_step_2", s, e)
    )

    print("  Sending arm_1 step_3 trajectory...")
    executor.execute_async(
        "arm_1", traj_1_step_3, lambda s, e: on_execution_complete("arm_1_step_3", s, e)
    )

    print("  Sending arm_2 step_1 trajectory...")
    executor.execute_async(
        "arm_2", traj_2_step_1, lambda s, e: on_execution_complete("arm_2_step_1", s, e)
    )

    print("  Sending arm_2 step_2 trajectory...")
    executor.execute_async(
        "arm_2", traj_2_step_2, lambda s, e: on_execution_complete("arm_2_step_2", s, e)
    )

    print("  Sending arm_2 step_3 trajectory...")
    executor.execute_async(
        "arm_2", traj_2_step_3, lambda s, e: on_execution_complete("arm_2_step_3", s, e)
    )

    print("\n✓ All 6 trajectories sent simultaneously\n")

    print("Waiting for execution to complete...\n")

    # Wait for all to complete
    max_wait = 60
    start_wait = time.time()

    all_completed = False
    while not all_completed:
        elapsed = time.time() - start_wait
        if elapsed > max_wait:
            print(f"✗ TIMEOUT: Waited {max_wait}s for completion")
            break

        all_completed = all(r["completed"] for r in results.values())
        time.sleep(0.1)

    # Shutdown
    ros_executor.shutdown()

    # Print results
    print("\n" + "=" * 80)
    print("EXECUTION SUMMARY")
    print("=" * 80)

    print("\nArm 1 Trajectories:")
    for i in range(1, 4):
        key = f"arm_1_step_{i}"
        result = results[key]
        status = "SUCCESS" if result["success"] else f"FAILED ({result['error']})"
        time_str = f"{result['time']:.3f}s" if result["time"] is not None else "N/A"
        print(f"  Step {i}: {status} @ {time_str}")

    print("\nArm 2 Trajectories:")
    for i in range(1, 4):
        key = f"arm_2_step_{i}"
        result = results[key]
        status = "SUCCESS" if result["success"] else f"FAILED ({result['error']})"
        time_str = f"{result['time']:.3f}s" if result["time"] is not None else "N/A"
        print(f"  Step {i}: {status} @ {time_str}")

    # Analyze parallelism
    print("\n" + "-" * 80)
    print("PARALLELISM ANALYSIS:")
    print("-" * 80)

    arm_1_times = [
        results[f"arm_1_step_{i}"]["time"]
        for i in range(1, 4)
        if results[f"arm_1_step_{i}"]["time"]
    ]
    arm_2_times = [
        results[f"arm_2_step_{i}"]["time"]
        for i in range(1, 4)
        if results[f"arm_2_step_{i}"]["time"]
    ]

    if arm_1_times and arm_2_times:
        arm_1_start = min(arm_1_times)
        arm_1_end = max(arm_1_times)
        arm_2_start = min(arm_2_times)
        arm_2_end = max(arm_2_times)

        print(
            f"\nArm 1 execution window: {arm_1_start:.3f}s - {arm_1_end:.3f}s (span: {arm_1_end - arm_1_start:.3f}s)"
        )
        print(
            f"Arm 2 execution window: {arm_2_start:.3f}s - {arm_2_end:.3f}s (span: {arm_2_end - arm_2_start:.3f}s)"
        )

        # Check if trajectories overlapped
        overlap_start = max(arm_1_start, arm_2_start)
        overlap_end = min(arm_1_end, arm_2_end)

        if overlap_end > overlap_start:
            overlap_duration = overlap_end - overlap_start
            print(f"\n✓ PARALLEL EXECUTION CONFIRMED!")
            print(
                f"  Execution overlap: {overlap_duration:.3f}s ({overlap_start:.3f}s - {overlap_end:.3f}s)"
            )
        else:
            print(f"\n⚠ Sequential execution detected (no overlap)")

    print("\n" + "=" * 80)

    all_success = all(r["success"] for r in results.values())

    if all_success:
        print("✓ TEST PASSED - All 6 trajectories executed successfully!")
        print("=" * 80 + "\n")
    else:
        print("✗ TEST FAILED - One or more trajectories failed:")
        for key, result in results.items():
            if not result["success"]:
                print(f"  {key}: {result['error']}")
        print("=" * 80 + "\n")

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()

finally:
    rclpy.shutdown()
