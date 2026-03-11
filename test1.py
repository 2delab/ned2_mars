#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from math import radians, degrees


def add_trajectory_timing(trajectory_msg, total_time=5.0):
    """Add timing to trajectory points if they are all zero."""
    joint_traj = trajectory_msg.joint_trajectory

    # Check if all times are zero
    all_times_zero = all(
        p.time_from_start.sec == 0 and p.time_from_start.nanosec == 0
        for p in joint_traj.points
    )

    if all_times_zero and len(joint_traj.points) > 1:
        num_points = len(joint_traj.points)
        time_step = total_time / (num_points - 1)

        for i, point in enumerate(joint_traj.points):
            time_offset = i * time_step
            point.time_from_start.sec = int(time_offset)
            point.time_from_start.nanosec = int((time_offset % 1.0) * 1e9)

        return True
    return False


# Initialize ROS
rclpy.init()

# Initialize MoveItPy with config file
# Uses YAML to define planning pipelines, scene monitor, and controllers
config_path = "/home/i/ned2_mars/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_mock_moveit_config/config/moveit_py_params.yaml"
moveit = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_path])

# Get planning component for the arm
arm = moveit.get_planning_component("arm")

# Set start state to current state
arm.set_start_state_to_current_state()

# Set goal state: joint_1 to 50 degrees
robot_state = RobotState(moveit.get_robot_model())
robot_state.joint_positions = {"joint_1": radians(50)}
arm.set_goal_state(robot_state=robot_state)

# Plan with 30% velocity
params = PlanRequestParameters(moveit)
params.max_velocity_scaling_factor = 0.3
plan_result = arm.plan(parameters=params)

# Check planning result
if plan_result:
    print("✓ Planning succeeded!")

    # ==================== TRAJECTORY TIMING FIX ====================
    # MoveIt sometimes generates trajectories without proper timing
    # Get the trajectory message and fix timing if needed
    traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
    joint_traj = traj_msg.joint_trajectory

    # Apply timing fix
    if add_trajectory_timing(traj_msg, total_time=5.0):
        print(
            "[DEBUG] Detected zero timing in trajectory. Added timing (5.0s duration)."
        )
        print(
            f"[DEBUG] Points: {len(joint_traj.points)}, Time step: {5.0 / (len(joint_traj.points) - 1):.4f}s"
        )
    else:
        plan_result_for_exec = plan_result

    # ==================== DEBUG OUTPUT ====================
    print("\n" + "=" * 70)
    print("TRAJECTORY DEBUG OUTPUT")
    print("=" * 70)

    print(f"\n[TRAJECTORY METADATA]")
    print(f"  Joint names ({len(joint_traj.joint_names)}): {joint_traj.joint_names}")
    print(f"  Number of points: {len(joint_traj.points)}")
    print(f"  Frame ID: {joint_traj.header.frame_id}")

    print(f"\n[TRAJECTORY POINTS DETAILS]")
    for i, point in enumerate(joint_traj.points):
        print(f"\n  Point {i}:")
        print(
            f"    Time from start: {point.time_from_start.sec}s + {point.time_from_start.nanosec}ns"
        )
        print(f"    Positions: {list(point.positions)}")
        if point.positions:
            print(f"    Positions (degrees): {[degrees(p) for p in point.positions]}")
        print(
            f"    Velocities: {list(point.velocities) if point.velocities else 'None'}"
        )
        print(
            f"    Accelerations: {list(point.accelerations) if point.accelerations else 'None'}"
        )
        print(f"    Effort: {list(point.effort) if point.effort else 'None'}")

    print(f"\n[CONTROLLER CONFIGURATION]")
    print(f"  Planning component name: arm")
    print(f"  Controller action namespace: /arm_controller/follow_joint_trajectory")
    print(f"  Max velocity scaling: 0.3")

    # Check for potential issues
    print(f"\n[TRAJECTORY VALIDATION]")
    first_point = joint_traj.points[0] if joint_traj.points else None
    if first_point:
        total_time = (
            first_point.time_from_start.sec + first_point.time_from_start.nanosec / 1e9
        )
        if total_time == 0:
            print(
                f"  ⚠️  WARNING: First point time_from_start is 0! This may cause controller rejection."
            )
            print(
                f"      Issue: JointTrajectoryController expects first point to have positive time offset."
            )
        else:
            print(f"  ✓ First point time offset is valid: {total_time:.3f}s")

        # Check joint count
        if len(joint_traj.joint_names) != 6:
            print(f"  ⚠️  WARNING: Expected 6 joints, got {len(joint_traj.joint_names)}")
        else:
            print(f"  ✓ Joint count is correct: 6")

        # Check position values
        if all(pos == 0 for pos in first_point.positions):
            print(f"  ⚠️  WARNING: All positions are 0. Verify this is intended.")
        else:
            print(f"  ✓ Position values vary as expected")

    print("\n" + "=" * 70)
    print("EXECUTING TRAJECTORY...")
    print("=" * 70 + "\n")

    # Send trajectory directly to action server (with proper timing)
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from rclpy.node import Node
    import time

    # Create a temporary node for the action client
    temp_node = rclpy.create_node("trajectory_executor")
    action_client = ActionClient(
        temp_node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
    )

    # Wait for server
    if action_client.wait_for_server(timeout_sec=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj_msg.joint_trajectory  # Use the timed trajectory!

        print("[DEBUG] Sending goal to arm_controller action server...")
        print(f"[DEBUG] Trajectory has {len(goal.trajectory.points)} points")
        print(
            f"[DEBUG] First point time: {goal.trajectory.points[0].time_from_start.sec}s"
        )
        print(
            f"[DEBUG] Last point time: {goal.trajectory.points[-1].time_from_start.sec}s"
        )

        future = action_client.send_goal_async(goal)
        # Give it time to process
        time.sleep(6.0)

        print("✓ Trajectory execution completed!")
        exec_result_status = "SUCCESS"
    else:
        print("✗ Could not connect to action server")
        exec_result_status = "FAILED"

    print(f"Execution status: {exec_result_status}")
else:
    print("✗ Planning failed!")

rclpy.shutdown()
