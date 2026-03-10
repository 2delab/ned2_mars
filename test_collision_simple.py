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


def create_multi_joint_trajectory(arm_id, joint_moves, duration_sec=3.0):
    """
    Create trajectory with multiple joints moving.

    Args:
        arm_id: "arm_1" or "arm_2"
        joint_moves: Dict of {joint_index: angle_deg}
                     e.g., {0: 50.0, 1: -35.0} means joint_1=50°, joint_2=-35°
        duration_sec: Time to reach target position

    Returns:
        JointTrajectory with interpolated points
    """
    traj = JointTrajectory()
    traj.joint_names = [f"{arm_id}_joint_{i}" for i in range(1, 7)]

    # Convert all angles to radians
    target_positions = [0.0] * 6
    for joint_idx, angle_deg in joint_moves.items():
        target_positions[joint_idx] = degrees_to_radians(angle_deg)

    # Create 3 interpolated points for smooth motion
    num_points = 3
    for i in range(num_points):
        point = JointTrajectoryPoint()
        alpha = i / (num_points - 1)

        # Interpolate from [0,0,0,0,0,0] to target
        positions = [0.0] * 6
        for joint_idx, target_rad in enumerate(target_positions):
            positions[joint_idx] = target_rad * alpha

        point.positions = positions
        time_sec = duration_sec * alpha
        point.time_from_start = Duration(
            sec=int(time_sec), nanosec=int((time_sec - int(time_sec)) * 1e9)
        )
        traj.points.append(point)

    return traj


def create_compound_position_trajectory(arm_id, positions_list, duration_per_move=3.0):
    """
    Create compound trajectory with multiple sequential position targets.

    Args:
        arm_id: "arm_1" or "arm_2"
        positions_list: List of dicts, each dict is {joint_index: angle_deg}
                        e.g., [{0: 50, 1: -35}, {0: -50, 1: -35}]
        duration_per_move: Time for each move (3 seconds)

    Returns:
        JointTrajectory with waypoints for each position
    """
    traj = JointTrajectory()
    traj.joint_names = [f"{arm_id}_joint_{i}" for i in range(1, 7)]

    cumulative_time = 0.0

    # Add starting point (all zeros at t=0)
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 6
    point.time_from_start = Duration(sec=0, nanosec=0)
    traj.points.append(point)

    # Add waypoint for each position target
    for position_dict in positions_list:
        cumulative_time += duration_per_move

        point = JointTrajectoryPoint()
        positions = [0.0] * 6

        # Set specified joints, others stay at 0
        for joint_idx, angle_deg in position_dict.items():
            positions[joint_idx] = degrees_to_radians(angle_deg)

        point.positions = positions
        point.time_from_start = Duration(
            sec=int(cumulative_time),
            nanosec=int((cumulative_time - int(cumulative_time)) * 1e9),
        )
        traj.points.append(point)

    return traj


def create_smooth_rotation_trajectory(
    arm_id,
    start_angle_deg,
    end_angle_deg,
    fixed_joints,
    duration_sec=3.0,
    num_points=20,
):
    """
    Create a smooth rotation trajectory for one joint while keeping others fixed.

    Args:
        arm_id: "arm_1" or "arm_2"
        start_angle_deg: Starting angle in degrees
        end_angle_deg: Ending angle in degrees
        fixed_joints: Dict of {joint_index: angle_deg} for joints that stay fixed
                      e.g., {1: -35} means joint_2 stays at -35°
        duration_sec: Total duration of the movement
        num_points: Number of interpolated points for smooth motion

    Returns:
        JointTrajectory with smooth interpolation
    """
    traj = JointTrajectory()
    traj.joint_names = [f"{arm_id}_joint_{i}" for i in range(1, 7)]

    # Create smooth interpolation between start and end angles
    for i in range(num_points):
        point = JointTrajectoryPoint()
        alpha = i / (num_points - 1)  # 0 to 1

        # Interpolate the rotating joint
        current_angle_deg = start_angle_deg + alpha * (end_angle_deg - start_angle_deg)

        # Build position array
        positions = [0.0] * 6
        positions[0] = degrees_to_radians(current_angle_deg)  # rotating joint

        # Set fixed joints
        for joint_idx, angle_deg in fixed_joints.items():
            positions[joint_idx] = degrees_to_radians(angle_deg)

        point.positions = positions
        time_sec = duration_sec * alpha
        point.time_from_start = Duration(
            sec=int(time_sec), nanosec=int((time_sec - int(time_sec)) * 1e9)
        )
        traj.points.append(point)

    return traj


def main():
    rclpy.init()
    node = rclpy.create_node("collision_tester")

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

    # ========== PHASE 2: Move to collision test position ==========
    print("\nPHASE 2: Moving to collision test position...")
    print("  ARM_1: joint_1=+50°, joint_2=-35°")
    print("  ARM_2: joint_1=+50°, joint_2=-35°")

    # Compound trajectory: home -> position 1
    # Position 1: {0: 50, 1: -35} (joint_1=+50°, joint_2=-35°)
    traj1 = create_compound_position_trajectory(
        "arm_1", [{0: 50, 1: -35}], duration_per_move=3.0
    )
    traj2 = create_compound_position_trajectory(
        "arm_2", [{0: 50, 1: -35}], duration_per_move=3.0
    )

    goal1 = FollowJointTrajectory.Goal(trajectory=traj1)
    goal2 = FollowJointTrajectory.Goal(trajectory=traj2)

    arm1_client.send_goal_async(goal1)
    arm2_client.send_goal_async(goal2)

    print("  ✓ Both moving to position 1 (parallel)")
    print("  Waiting for execution (~4 seconds)...")
    time.sleep(4)
    print("  ✓ Position 1 reached")

    # ========== PHASE 3: Smooth rotation to collision zone ==========
    print(
        "\nPHASE 3: Rotating joint_1 smoothly from +50° to -50° (joint_2 fixed at -35°)..."
    )
    print("  Watch RViz for potential collision...")

    # Smooth rotation: joint_1 from +50° to -50°, while joint_2 stays at -35°
    traj1 = create_smooth_rotation_trajectory(
        "arm_1",
        start_angle_deg=50.0,
        end_angle_deg=-50.0,
        fixed_joints={1: -35.0},
        duration_sec=3.0,
        num_points=20,
    )
    traj2 = create_smooth_rotation_trajectory(
        "arm_2",
        start_angle_deg=50.0,
        end_angle_deg=-50.0,
        fixed_joints={1: -35.0},
        duration_sec=3.0,
        num_points=20,
    )

    goal1 = FollowJointTrajectory.Goal(trajectory=traj1)
    goal2 = FollowJointTrajectory.Goal(trajectory=traj2)

    arm1_client.send_goal_async(goal1)
    arm2_client.send_goal_async(goal2)

    print("  ✓ Both rotating smoothly (parallel)")
    print("  Waiting for execution (~4 seconds)...")
    time.sleep(4)
    print("  ✓ Rotation complete - collision zone reached")

    print("\nCollision test complete!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
