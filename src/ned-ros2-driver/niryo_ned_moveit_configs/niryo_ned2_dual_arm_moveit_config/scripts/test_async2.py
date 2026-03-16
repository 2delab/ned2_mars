#!/usr/bin/env python3
"""
Advanced async executor test - 7 concurrent movements demonstrating true parallelism.

Arm 1: 4 movements alternating between 0° and 50° (0→50→0→50)
Arm 2: 3 movements alternating between 0° and 50° (0→50→0)

All 7 movements sent simultaneously to demonstrate true async/parallel execution.

Prerequisites:
    Terminal 1: ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py use_rviz:=false
    Terminal 2: python3 test_async2.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import radians
import time
import threading

# Import the async executor library
from async_executor_lib import AsyncDualArmExecutor

print("\n" + "=" * 80)
print("Advanced Async Executor Test - 7 Concurrent Movements")
print("Arm 1: 4 movements (0°→50°→0°→50°)")
print("Arm 2: 3 movements (0°→50°→0°)")
print("=" * 80)

# Track results for all 7 movements
start_time = time.time()
movements = {
    # Arm 1: 4 movements
    "arm_1_mov_1": {
        "arm": "arm_1",
        "sequence": 1,
        "target_pos": 50,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    "arm_1_mov_2": {
        "arm": "arm_1",
        "sequence": 2,
        "target_pos": 0,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    "arm_1_mov_3": {
        "arm": "arm_1",
        "sequence": 3,
        "target_pos": 50,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    "arm_1_mov_4": {
        "arm": "arm_1",
        "sequence": 4,
        "target_pos": 0,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    # Arm 2: 3 movements
    "arm_2_mov_1": {
        "arm": "arm_2",
        "sequence": 1,
        "target_pos": 50,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    "arm_2_mov_2": {
        "arm": "arm_2",
        "sequence": 2,
        "target_pos": 0,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
    "arm_2_mov_3": {
        "arm": "arm_2",
        "sequence": 3,
        "target_pos": 50,
        "sent_time": None,
        "completed": False,
        "success": False,
        "error": None,
        "completion_time": None,
        "duration": None,
    },
}


def on_execution_complete(movement_id: str, arm_id: str, success: bool, error_msg: str):
    """Callback when a movement completes"""
    elapsed = time.time() - start_time
    movements[movement_id]["completed"] = True
    movements[movement_id]["success"] = success
    movements[movement_id]["error"] = error_msg
    movements[movement_id]["completion_time"] = elapsed

    # Calculate duration from sent_time
    if movements[movement_id]["sent_time"] is not None:
        movements[movement_id]["duration"] = (
            elapsed - movements[movement_id]["sent_time"]
        )

    target_pos = movements[movement_id]["target_pos"]

    if success:
        print(
            f"[T={elapsed:8.3f}s] ✓ {movement_id:12s} → {target_pos}° COMPLETED (duration: {movements[movement_id]['duration']:.3f}s)"
        )
    else:
        print(
            f"[T={elapsed:8.3f}s] ✗ {movement_id:12s} → {target_pos}° FAILED - {error_msg}"
        )


# Initialize ROS
rclpy.init()

try:
    print("\n[PHASE 1/4] Creating node and async executor...")
    node = rclpy.create_node("test_async_advanced")
    executor = AsyncDualArmExecutor(
        node,
        max_velocity_scaling_factor=0.2,
        max_acceleration_scaling_factor=0.2,
    )

    # Start spinning in background immediately
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)
    executor_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    executor_thread.start()
    print("✓ Executor created and spinning\n")

    print("[PHASE 2/4] Waiting for action servers to become available...")
    # Wait for servers to be ready
    timeout = 5.0
    arm_1_ready = executor.arm_1_client.wait_for_server(timeout_sec=timeout)
    arm_2_ready = executor.arm_2_client.wait_for_server(timeout_sec=timeout)

    if not arm_1_ready or not arm_2_ready:
        print("✗ Action servers not available!")
        raise RuntimeError("Action servers not ready")
    print("✓ Action servers available\n")

    print("[PHASE 3/4] Creating 7 trajectories...")
    print("-" * 80)

    # Dictionary to store all trajectories
    trajectories = {}

    # ARM 1: 4 movements (0→50→0→50)
    arm1_positions = [50, 0, 50, 0]
    for i, target_pos in enumerate(arm1_positions, 1):
        mov_id = f"arm_1_mov_{i}"
        traj = JointTrajectory()
        traj.joint_names = [
            "arm_1_joint_1",
            "arm_1_joint_2",
            "arm_1_joint_3",
            "arm_1_joint_4",
            "arm_1_joint_5",
            "arm_1_joint_6",
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(radians(target_pos)), 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1)
        traj.points.append(point)
        trajectories[mov_id] = traj
        print(f"  ✓ {mov_id}: joint_1 → {target_pos}°")

    # ARM 2: 3 movements (0→50→0)
    arm2_positions = [50, 0, 50]
    for i, target_pos in enumerate(arm2_positions, 1):
        mov_id = f"arm_2_mov_{i}"
        traj = JointTrajectory()
        traj.joint_names = [
            "arm_2_joint_1",
            "arm_2_joint_2",
            "arm_2_joint_3",
            "arm_2_joint_4",
            "arm_2_joint_5",
            "arm_2_joint_6",
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(radians(target_pos)), 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1)
        traj.points.append(point)
        trajectories[mov_id] = traj
        print(f"  ✓ {mov_id}: joint_1 → {target_pos}°")

    print("✓ All 7 trajectories created\n")

    print("[PHASE 4/4] Executing ALL 7 movements simultaneously...\n")
    print("-" * 80)
    print("EXECUTION LOG:")
    print("-" * 80)

    # Send all 7 trajectories immediately in sequence
    # They will all be in-flight at the same time (parallel execution)
    for mov_id in [
        "arm_1_mov_1",
        "arm_1_mov_2",
        "arm_1_mov_3",
        "arm_1_mov_4",
        "arm_2_mov_1",
        "arm_2_mov_2",
        "arm_2_mov_3",
    ]:
        elapsed = time.time() - start_time
        arm_id = movements[mov_id]["arm"]
        target_pos = movements[mov_id]["target_pos"]

        # Record when we send this movement
        movements[mov_id]["sent_time"] = elapsed

        # Create callback closure for this specific movement
        def make_callback(mid, aid):
            def callback(_, success, error):
                on_execution_complete(mid, aid, success, error)

            return callback

        # Send the trajectory
        print(
            f"[T={elapsed:8.3f}s] Sending {mov_id:12s} → {target_pos}° (sent via {arm_id})"
        )
        executor.execute_async(
            arm_id, trajectories[mov_id], make_callback(mov_id, arm_id)
        )

        # Small delay between sends to keep them all in flight
        time.sleep(0.01)

    print("-" * 80)
    print("✓ All 7 trajectories sent simultaneously\n")
    print("Waiting for all movements to complete...\n")
    print("-" * 80)

    # Wait for all to complete
    max_wait = 60
    start_wait = time.time()
    all_completed = False

    while True:
        elapsed = time.time() - start_wait
        completed_count = sum(1 for m in movements.values() if m["completed"])

        if completed_count == 7:
            all_completed = True
            break

        if elapsed > max_wait:
            print(f"✗ TIMEOUT: Waited {max_wait}s for completion")
            print(f"  {completed_count}/7 movements completed")
            break

        time.sleep(0.1)

    # Shutdown
    ros_executor.shutdown()

    # Print detailed results
    print("-" * 80)
    print("\n" + "=" * 80)
    print("EXECUTION SUMMARY")
    print("=" * 80)

    # Sort movements by completion time for timeline view
    sorted_movements = sorted(
        movements.items(),
        key=lambda x: (
            x[1]["completion_time"]
            if x[1]["completion_time"] is not None
            else float("inf")
        ),
    )

    print("\nTIMELINE OF MOVEMENTS (in completion order):")
    print("-" * 80)

    for mov_id, data in sorted_movements:
        if data["completed"]:
            status = "✓ SUCCESS" if data["success"] else "✗ FAILED"
            print(
                f"  {mov_id:12s}: sent={data['sent_time']:8.3f}s, "
                f"completed={data['completion_time']:8.3f}s, "
                f"duration={data['duration']:8.3f}s | {status}"
            )
        else:
            print(f"  {mov_id:12s}: NOT COMPLETED")

    print("\nMOVEMENT DETAILS BY ARM:")
    print("-" * 80)

    # Arm 1 movements
    print("\nArm 1 (4 movements):")
    arm1_movements = [m for m in sorted_movements if m[0].startswith("arm_1")]
    for mov_id, data in arm1_movements:
        status = "✓ SUCCESS" if data["success"] else "✗ FAILED"
        target = data["target_pos"]
        print(f"  {mov_id}: → {target}° | {status} | Duration: {data['duration']:.3f}s")

    # Arm 2 movements
    print("\nArm 2 (3 movements):")
    arm2_movements = [m for m in sorted_movements if m[0].startswith("arm_2")]
    for mov_id, data in arm2_movements:
        status = "✓ SUCCESS" if data["success"] else "✗ FAILED"
        target = data["target_pos"]
        print(f"  {mov_id}: → {target}° | {status} | Duration: {data['duration']:.3f}s")

    # Parallelism analysis
    print("\nPARALLELISM ANALYSIS:")
    print("-" * 80)

    arm1_sent_times = sorted(
        [m[1]["sent_time"] for m in arm1_movements if m[1]["sent_time"] is not None]
    )
    arm2_sent_times = sorted(
        [m[1]["sent_time"] for m in arm2_movements if m[1]["sent_time"] is not None]
    )
    arm1_completed_times = sorted(
        [
            m[1]["completion_time"]
            for m in arm1_movements
            if m[1]["completion_time"] is not None
        ]
    )
    arm2_completed_times = sorted(
        [
            m[1]["completion_time"]
            for m in arm2_movements
            if m[1]["completion_time"] is not None
        ]
    )

    if arm1_sent_times and arm2_sent_times:
        # Check for overlap in execution windows
        arm1_execution_start = min(arm1_sent_times)
        arm1_execution_end = max(arm1_completed_times) if arm1_completed_times else 0
        arm2_execution_start = min(arm2_sent_times)
        arm2_execution_end = max(arm2_completed_times) if arm2_completed_times else 0

        print(
            f"\nArm 1 execution window: {arm1_execution_start:.3f}s → {arm1_execution_end:.3f}s"
        )
        print(
            f"Arm 2 execution window: {arm2_execution_start:.3f}s → {arm2_execution_end:.3f}s"
        )

        # Check overlap
        overlap_start = max(arm1_execution_start, arm2_execution_start)
        overlap_end = min(arm1_execution_end, arm2_execution_end)

        if overlap_end > overlap_start:
            overlap_duration = overlap_end - overlap_start
            print(f"\n✓ PARALLEL EXECUTION CONFIRMED!")
            print(f"  Overlap period: {overlap_start:.3f}s → {overlap_end:.3f}s")
            print(f"  Overlap duration: {overlap_duration:.3f}s")
        else:
            print("\n⚠ No overlap detected - movements may be sequential")

    # Overall success
    print("\n" + "=" * 80)
    total_movements = len(movements)
    successful_movements = sum(1 for m in movements.values() if m["success"])

    print(f"RESULTS: {successful_movements}/{total_movements} movements successful")

    if successful_movements == total_movements:
        print("✓ TEST PASSED - All 7 movements executed successfully in parallel!")
    else:
        print("✗ TEST FAILED - Some movements did not complete successfully")
        for mov_id, data in movements.items():
            if not data["success"]:
                print(f"  {mov_id}: {data['error']}")

    print("=" * 80 + "\n")

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()

finally:
    rclpy.shutdown()
