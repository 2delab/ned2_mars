#!/usr/bin/env python3
"""
Simple async executor test - both arms move to 50 degrees in parallel.

No separate ROS processes, no context issues - just pure async execution.

Prerequisites:
    Terminal 1: ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py use_rviz:=false
    Terminal 2: python3 test_async_simple.py
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

print("\n" + "=" * 70)
print("Async Executor Test - Move both arms to 50 degrees in PARALLEL")
print("=" * 70)

# Track results
start_time = time.time()
results = {
    "arm_1": {"completed": False, "success": False, "error": None, "time": None},
    "arm_2": {"completed": False, "success": False, "error": None, "time": None},
}


def on_execution_complete(arm_id: str, success: bool, error_msg: str):
    """Callback when execution completes"""
    elapsed = time.time() - start_time
    results[arm_id]["completed"] = True
    results[arm_id]["success"] = success
    results[arm_id]["error"] = error_msg
    results[arm_id]["time"] = elapsed

    if success:
        print(f"✓ [{elapsed:.3f}s] {arm_id}: SUCCESS")
    else:
        print(f"✗ [{elapsed:.3f}s] {arm_id}: FAILED - {error_msg}")


# Initialize ROS
rclpy.init()

try:
    print("\n[1/4] Creating node and async executor...")
    node = rclpy.create_node("test_async_simple")
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

    print("[3/3] Creating trajectories...")
    # Arm 1 trajectory
    traj_1 = JointTrajectory()
    traj_1.joint_names = [
        "arm_1_joint_1",
        "arm_1_joint_2",
        "arm_1_joint_3",
        "arm_1_joint_4",
        "arm_1_joint_5",
        "arm_1_joint_6",
    ]
    p1 = JointTrajectoryPoint()
    p1.positions = [float(radians(50)), 0.0, 0.0, 0.0, 0.0, 0.0]
    p1.time_from_start = Duration(sec=1)
    traj_1.points.append(p1)

    # Arm 2 trajectory
    traj_2 = JointTrajectory()
    traj_2.joint_names = [
        "arm_2_joint_1",
        "arm_2_joint_2",
        "arm_2_joint_3",
        "arm_2_joint_4",
        "arm_2_joint_5",
        "arm_2_joint_6",
    ]
    p2 = JointTrajectoryPoint()
    p2.positions = [float(radians(50)), 0.0, 0.0, 0.0, 0.0, 0.0]
    p2.time_from_start = Duration(sec=1)
    traj_2.points.append(p2)
    print("✓ Trajectories created\n")

    print("[4/4] Executing both arms asynchronously...\n")

    # Execute both arms asynchronously
    print("  Sending arm_1 trajectory...")
    executor.execute_async("arm_1", traj_1, on_execution_complete)

    print("  Sending arm_2 trajectory...")
    executor.execute_async("arm_2", traj_2, on_execution_complete)

    print("✓ Both trajectories sent simultaneously\n")

    print("Waiting for execution to complete...\n")

    # Wait for both to complete
    max_wait = 30
    start_wait = time.time()

    while not (results["arm_1"]["completed"] and results["arm_2"]["completed"]):
        elapsed = time.time() - start_wait
        if elapsed > max_wait:
            print(f"✗ TIMEOUT: Waited {max_wait}s for completion")
            break
        time.sleep(0.1)

    # Shutdown
    ros_executor.shutdown()

    # Print results
    print("\n" + "=" * 70)
    print("EXECUTION SUMMARY")
    print("=" * 70)

    arm_1_time = results["arm_1"]["time"]
    arm_2_time = results["arm_2"]["time"]

    print(f"\nArm 1 completed at: {arm_1_time:.3f}s - {results['arm_1']['success']}")
    print(f"Arm 2 completed at: {arm_2_time:.3f}s - {results['arm_2']['success']}")

    if arm_1_time and arm_2_time:
        time_diff = abs(arm_1_time - arm_2_time)
        print(f"Time difference: {time_diff:.3f}s")

        if time_diff < 0.5:
            print("✓ PARALLEL EXECUTION CONFIRMED - Both arms completed within 500ms!")
        else:
            print("⚠ Sequential execution detected")

    print("\n" + "=" * 70)

    all_success = all(r["success"] for r in results.values())

    if all_success:
        print("✓ TEST PASSED - Both arms executed successfully in parallel!")
        print("=" * 70 + "\n")
    else:
        print("✗ TEST FAILED - One or both arms failed")
        for arm_id, result in results.items():
            if not result["success"]:
                print(f"  {arm_id}: {result['error']}")
        print("=" * 70 + "\n")

except Exception as e:
    print(f"\n✗ ERROR: {e}\n")
    import traceback

    traceback.print_exc()

finally:
    rclpy.shutdown()
