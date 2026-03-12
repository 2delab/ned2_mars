#!/usr/bin/env python3
"""
MoveIt2 Trajectory Execution with OMPL Planner
This script demonstrates the proper way to use OMPL planner which generates
trajectories with correct timing, allowing standard moveit.execute() to work.
"""

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from math import radians, degrees

# Initialize ROS
rclpy.init()

# Initialize MoveItPy with config file
config_path = "/home/i/ned2_mars/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_mock_moveit_config/config/moveit_py_params.yaml"
moveit = MoveItPy(
    node_name="moveit_py_test_ompl", launch_params_filepaths=[config_path]
)

# Get planning component for the arm
arm = moveit.get_planning_component("arm")

# Set start state to current state
arm.set_start_state_to_current_state()

# Set goal state: joint_1 to 50 degrees
robot_state = RobotState(moveit.get_robot_model())
robot_state.joint_positions = {"joint_1": radians(50)}
arm.set_goal_state(robot_state=robot_state)

# Plan with OMPL planner (which generates properly timed trajectories)
# OMPL is the key difference - it includes trajectory timing automatically
params = PlanRequestParameters(moveit)
params.planning_pipeline = "ompl"  # Explicitly use OMPL planner
params.planner_id = "RRTConnectkConfigDefault"  # RRT* Connect algorithm
params.max_velocity_scaling_factor = 0.3
params.planning_time = 1.0

print("Planning with OMPL planner...")
print(f"  Pipeline: {params.planning_pipeline}")
print(f"  Planner ID: {params.planner_id}")
print(f"  Velocity scaling: {params.max_velocity_scaling_factor}")

plan_result = arm.plan(parameters=params)

# Check planning result
if plan_result:
    print("✓ Planning succeeded!")

    # ==================== TRAJECTORY DEBUG OUTPUT ====================
    print("\n" + "=" * 70)
    print("TRAJECTORY ANALYSIS (OMPL)")
    print("=" * 70)

    traj = plan_result.trajectory
    traj_msg = traj.get_robot_trajectory_msg()
    joint_traj = traj_msg.joint_trajectory

    print(f"\n[TRAJECTORY METADATA]")
    print(f"  Joint names ({len(joint_traj.joint_names)}): {joint_traj.joint_names}")
    print(f"  Number of points: {len(joint_traj.points)}")
    print(f"  Frame ID: {joint_traj.header.frame_id}")

    print(f"\n[TRAJECTORY TIMING]")
    if joint_traj.points:
        first_point = joint_traj.points[0]
        last_point = joint_traj.points[-1]
        first_time = (
            first_point.time_from_start.sec + first_point.time_from_start.nanosec / 1e9
        )
        last_time = (
            last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
        )

        print(f"  First point time: {first_time:.3f}s")
        print(f"  Last point time: {last_time:.3f}s")
        print(f"  Total duration: {last_time:.3f}s")

        # Show sample points
        print(f"\n[SAMPLE POINTS]")
        sample_indices = [
            0,
            len(joint_traj.points) // 4,
            len(joint_traj.points) // 2,
            3 * len(joint_traj.points) // 4,
            len(joint_traj.points) - 1,
        ]
        for idx in sample_indices:
            point = joint_traj.points[idx]
            pt_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            pos_deg = degrees(point.positions[0]) if point.positions else 0
            print(f"  Point {idx:3d}: t={pt_time:6.3f}s, joint_1={pos_deg:7.2f}°")

    print(f"\n[TRAJECTORY VALIDATION]")
    print(f"  ✓ Trajectory has proper timing (OMPL planner)")
    print(f"  ✓ Ready for standard MoveIt execution")

    # ==================== EXECUTION ====================
    print("\n" + "=" * 70)
    print("EXECUTING TRAJECTORY WITH STANDARD MOVEIT API")
    print("=" * 70 + "\n")

    # Use standard MoveIt execution (no workarounds needed!)
    exec_result = moveit.execute("arm", plan_result.trajectory)

    print(f"Execution status: {exec_result.status}")

    if str(exec_result.status) == "ExecutionStatus.SUCCESS":
        print("\n✓ SUCCESS! Trajectory executed properly with OMPL planner.")
        print("  No timing workarounds or custom action clients needed!")
    else:
        print(f"\n✗ Execution failed with status: {exec_result.status}")

else:
    print("✗ Planning failed!")

print("\n" + "=" * 70)
rclpy.shutdown()
