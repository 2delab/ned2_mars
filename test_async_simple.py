#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * math.pi / 180.0


def create_home_trajectory(arm_id, duration_sec=3.0):
    """Create trajectory to move arm to [0,0,0,0,0,0]"""
    traj = JointTrajectory()
    traj.joint_names = [f"{arm_id}_joint_{i}" for i in range(1, 7)]

    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
    traj.points.append(point)

    return traj


def create_compound_swing_trajectory(arm_id, swings):
    """
    Create single compound trajectory with multiple swings.

    Args:
        arm_id: "arm_1" or "arm_2"
        swings: List of (start_deg, end_deg, duration_sec) tuples
                e.g., [(0, 50, 2.0), (50, -50, 2.0), ...]

    Returns:
        JointTrajectory with waypoints at each swing endpoint
    """
    traj = JointTrajectory()
    traj.joint_names = [f"{arm_id}_joint_{i}" for i in range(1, 7)]

    cumulative_time = 0.0

    # Add starting point (0 degrees at t=0)
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    point.time_from_start = Duration(sec=0, nanosec=0)
    traj.points.append(point)

    # Add endpoint for each swing
    for start_deg, end_deg, duration_sec in swings:
        cumulative_time += duration_sec
        end_rad = degrees_to_radians(end_deg)

        point = JointTrajectoryPoint()
        positions = [0.0] * 6
        positions[0] = end_rad  # Only joint_1 moves
        point.positions = positions

        # Convert float time to Duration
        point.time_from_start = Duration(
            sec=int(cumulative_time),
            nanosec=int((cumulative_time - int(cumulative_time)) * 1e9),
        )
        traj.points.append(point)

    return traj


def main():
    rclpy.init()
    node = rclpy.create_node("dual_arm_swinger")

    # Create action clients for both arms
    arm1_client = ActionClient(
        node, FollowJointTrajectory, "/arm_1/follow_joint_trajectory"
    )
    arm2_client = ActionClient(
        node, FollowJointTrajectory, "/arm_2/follow_joint_trajectory"
    )

    # Wait for servers
    print("Waiting for action servers...")
    arm1_client.wait_for_server(timeout_sec=10.0)
    arm2_client.wait_for_server(timeout_sec=10.0)
    print("Servers ready")

    # ========== PHASE 1: Move both to home ==========
    print("\nPHASE 1: Moving both arms to home [0,0,0,0,0,0]...")
    home1 = create_home_trajectory("arm_1", duration_sec=3.0)
    home2 = create_home_trajectory("arm_2", duration_sec=3.0)

    goal1 = FollowJointTrajectory.Goal(trajectory=home1)
    goal2 = FollowJointTrajectory.Goal(trajectory=home2)

    arm1_client.send_goal_async(goal1)
    arm2_client.send_goal_async(goal2)
    time.sleep(4)
    print("✓ Both arms at home")

    # ========== PHASE 2: Send compound trajectories (one goal per arm) ==========
    print("\nPHASE 2: Sending compound trajectories (parallel execution)...")
    print("  ARM_1: 4 continuous swings (8 seconds total)")
    print("    0° → +50° → -50° → +50° → -50°")
    print("  ARM_2: 2 continuous swings (4 seconds total)")
    print("    0° → +50° → -50°")

    # Define ARM_1 swings: (start_deg, end_deg, duration_sec)
    arm1_swings = [
        (0.0, 50.0, 2.0),  # Swing 1: 0° → +50° (2 sec)
        (50.0, -50.0, 2.0),  # Swing 2: +50° → -50° (2 sec)
        (-50.0, 50.0, 2.0),  # Swing 3: -50° → +50° (2 sec)
        (50.0, -50.0, 2.0),  # Swing 4: +50° → -50° (2 sec)
    ]

    # Define ARM_2 swings: (start_deg, end_deg, duration_sec)
    arm2_swings = [
        (0.0, 50.0, 2.0),  # Swing 1: 0° → +50° (2 sec)
        (50.0, -50.0, 2.0),  # Swing 2: +50° → -50° (2 sec)
    ]

    # Create compound trajectories (one per arm)
    traj1 = create_compound_swing_trajectory("arm_1", arm1_swings)
    traj2 = create_compound_swing_trajectory("arm_2", arm2_swings)

    # Send both goals asynchronously (true parallel execution)
    goal1 = FollowJointTrajectory.Goal(trajectory=traj1)
    goal2 = FollowJointTrajectory.Goal(trajectory=traj2)

    arm1_client.send_goal_async(goal1)
    arm2_client.send_goal_async(goal2)

    print("  ✓ Both compound trajectories sent asynchronously")
    print("  Waiting for execution (~9 seconds)...")
    time.sleep(9)
    print("  ✓ Execution complete")

    # ========== PHASE 3: Return both to home ==========
    print("\nPHASE 3: Returning both arms to home...")
    home1 = create_home_trajectory("arm_1", duration_sec=2.0)
    home2 = create_home_trajectory("arm_2", duration_sec=2.0)

    goal1 = FollowJointTrajectory.Goal(trajectory=home1)
    goal2 = FollowJointTrajectory.Goal(trajectory=home2)

    arm1_client.send_goal_async(goal1)
    arm2_client.send_goal_async(goal2)
    time.sleep(3)
    print("✓ Both arms returned home")

    print("\nTest complete!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
