"""
Trajectory utility functions for conversion, splitting, and manipulation.

This module provides helper functions to:
- Convert between ROS2 standard and Niryo trajectory formats
- Split combined trajectories into per-arm trajectories
- Interpolate robot states at specific times
- Build trajectory data structures
"""

from typing import List, Tuple, Optional
import numpy as np
from copy import deepcopy

# ROS2 standard trajectory message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Niryo custom trajectory messages
try:
    from niryo_ned_ros2_interfaces.msg import (
        JointTrajectory as NiryoJointTrajectory,
        JointTrajectoryPoint as NiryoJointTrajectoryPoint,
    )
except ImportError:
    # Fallback if Niryo interfaces not available (shouldn't happen)
    NiryoJointTrajectory = JointTrajectory
    NiryoJointTrajectoryPoint = JointTrajectoryPoint


def convert_trajectory_msgs_to_niryo(
    traj_msgs: JointTrajectory,
) -> NiryoJointTrajectory:
    """
    Convert ROS2 standard trajectory_msgs/JointTrajectory to Niryo format.

    The Niryo format is a subset of trajectory_msgs:
    - Keeps: joint_names, positions, time_from_start
    - Drops: velocities, accelerations, effort (not used by Niryo)

    Args:
        traj_msgs: Standard ROS2 JointTrajectory

    Returns:
        Niryo-format JointTrajectory
    """
    niryo_traj = NiryoJointTrajectory()

    # Copy joint names as-is
    niryo_traj.joint_names = list(traj_msgs.joint_names)

    # Convert each point
    for point_msgs in traj_msgs.points:
        niryo_point = NiryoJointTrajectoryPoint()

        # Copy positions
        niryo_point.positions = list(point_msgs.positions)

        # Copy time_from_start
        niryo_point.time_from_start = deepcopy(point_msgs.time_from_start)

        niryo_traj.points.append(niryo_point)

    return niryo_traj


def split_trajectory_by_arm(
    trajectory: JointTrajectory,
) -> Tuple[JointTrajectory, JointTrajectory]:
    """
    Split a combined dual-arm trajectory into separate arm trajectories.

    Input trajectory has joint names like:
        [arm_1_joint_1, arm_1_joint_2, ..., arm_2_joint_1, arm_2_joint_2, ...]

    Output:
        - arm_1_traj: only arm_1_joint_* names and corresponding positions
        - arm_2_traj: only arm_2_joint_* names and corresponding positions

    Args:
        trajectory: Combined trajectory from MoveIt

    Returns:
        Tuple of (arm_1_trajectory, arm_2_trajectory)
    """
    # Find indices for each arm
    arm_1_indices = [
        i for i, name in enumerate(trajectory.joint_names) if name.startswith("arm_1_")
    ]
    arm_2_indices = [
        i for i, name in enumerate(trajectory.joint_names) if name.startswith("arm_2_")
    ]

    # Create arm_1 trajectory
    arm_1_traj = JointTrajectory()
    arm_1_traj.joint_names = [trajectory.joint_names[i] for i in arm_1_indices]

    # Create arm_2 trajectory
    arm_2_traj = JointTrajectory()
    arm_2_traj.joint_names = [trajectory.joint_names[i] for i in arm_2_indices]

    # Copy points, extracting only relevant positions for each arm
    for point_combined in trajectory.points:
        # arm_1 point
        point_1 = JointTrajectoryPoint()
        point_1.positions = [point_combined.positions[i] for i in arm_1_indices]
        point_1.time_from_start = deepcopy(point_combined.time_from_start)
        if point_combined.velocities:
            point_1.velocities = [point_combined.velocities[i] for i in arm_1_indices]
        if point_combined.accelerations:
            point_1.accelerations = [
                point_combined.accelerations[i] for i in arm_1_indices
            ]
        if point_combined.effort:
            point_1.effort = [point_combined.effort[i] for i in arm_1_indices]
        arm_1_traj.points.append(point_1)

        # arm_2 point
        point_2 = JointTrajectoryPoint()
        point_2.positions = [point_combined.positions[i] for i in arm_2_indices]
        point_2.time_from_start = deepcopy(point_combined.time_from_start)
        if point_combined.velocities:
            point_2.velocities = [point_combined.velocities[i] for i in arm_2_indices]
        if point_combined.accelerations:
            point_2.accelerations = [
                point_combined.accelerations[i] for i in arm_2_indices
            ]
        if point_combined.effort:
            point_2.effort = [point_combined.effort[i] for i in arm_2_indices]
        arm_2_traj.points.append(point_2)

    return arm_1_traj, arm_2_traj


def duration_to_seconds(duration) -> float:
    """
    Convert builtin_interfaces/Duration to float seconds.

    Args:
        duration: ROS2 Duration message

    Returns:
        Seconds as float
    """
    return duration.sec + duration.nanosec / 1e9


def seconds_to_duration(seconds: float):
    """
    Convert float seconds to builtin_interfaces/Duration.

    Args:
        seconds: Time in seconds

    Returns:
        ROS2 Duration message
    """
    from builtin_interfaces.msg import Duration

    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1e9)
    return duration


def get_robot_state_at_time(
    trajectory: JointTrajectory,
    time_target: float,
) -> Tuple[List[str], List[float]]:
    """
    Interpolate robot joint state at a specific time in trajectory.

    Uses linear interpolation between the two closest trajectory points.

    Args:
        trajectory: JointTrajectory message
        time_target: Time in seconds from trajectory start

    Returns:
        Tuple of (joint_names, joint_positions)
    """
    if not trajectory.points:
        return trajectory.joint_names, [0.0] * len(trajectory.joint_names)

    # Find two points surrounding time_target
    times = [duration_to_seconds(p.time_from_start) for p in trajectory.points]

    # If before first point
    if time_target <= times[0]:
        return trajectory.joint_names, list(trajectory.points[0].positions)

    # If after last point
    if time_target >= times[-1]:
        return trajectory.joint_names, list(trajectory.points[-1].positions)

    # Find interpolation window
    idx = 0
    for i in range(len(times) - 1):
        if times[i] <= time_target <= times[i + 1]:
            idx = i
            break

    t0, t1 = times[idx], times[idx + 1]
    p0, p1 = trajectory.points[idx], trajectory.points[idx + 1]

    # Linear interpolation coefficient
    if t1 == t0:
        alpha = 0.0
    else:
        alpha = (time_target - t0) / (t1 - t0)

    # Interpolate positions
    positions = []
    for j in range(len(trajectory.joint_names)):
        pos = (1.0 - alpha) * p0.positions[j] + alpha * p1.positions[j]
        positions.append(pos)

    return trajectory.joint_names, positions


def discretize_trajectory(
    trajectory: JointTrajectory,
    timestep: float = 0.02,
) -> List[Tuple[float, List[str], List[float]]]:
    """
    Discretize trajectory into fixed timestep intervals.

    Returns states at regular time intervals, interpolating as needed.

    Args:
        trajectory: JointTrajectory message
        timestep: Time step for discretization in seconds (default 0.02s = 50Hz)

    Returns:
        List of (time, joint_names, joint_positions) tuples
    """
    if not trajectory.points:
        return []

    # Get trajectory duration
    duration = duration_to_seconds(trajectory.points[-1].time_from_start)

    # Generate timesteps
    times = np.arange(0.0, duration + timestep, timestep)

    # Sample trajectory at each timestep
    discretized = []
    for t in times:
        joint_names, positions = get_robot_state_at_time(trajectory, float(t))
        discretized.append((float(t), joint_names, positions))

    return discretized


def remove_joint_prefix(joint_name: str, prefix: str = "arm_") -> str:
    """
    Remove arm prefix from joint name.

    Example: "arm_1_joint_1" -> "joint_1"

    Args:
        joint_name: Full joint name with prefix
        prefix: Prefix to remove (default "arm_")

    Returns:
        Joint name without prefix
    """
    for i in range(1, 3):  # Try arm_1, arm_2
        full_prefix = f"arm_{i}_"
        if joint_name.startswith(full_prefix):
            return joint_name[len(full_prefix) :]
    return joint_name


def get_arm_id_from_joint_name(joint_name: str) -> Optional[str]:
    """
    Extract arm ID from prefixed joint name.

    Example: "arm_1_joint_1" -> "arm_1"

    Args:
        joint_name: Full joint name with prefix

    Returns:
        Arm ID string ("arm_1" or "arm_2"), or None if no prefix found
    """
    if joint_name.startswith("arm_1_"):
        return "arm_1"
    elif joint_name.startswith("arm_2_"):
        return "arm_2"
    return None


def trajectory_duration(trajectory: JointTrajectory) -> float:
    """
    Get total duration of a trajectory.

    Args:
        trajectory: JointTrajectory message

    Returns:
        Duration in seconds
    """
    if not trajectory.points:
        return 0.0
    return duration_to_seconds(trajectory.points[-1].time_from_start)
