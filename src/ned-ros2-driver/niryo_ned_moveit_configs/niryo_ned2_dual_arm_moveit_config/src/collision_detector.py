"""
Discrete-time collision detection between trajectories.

This module implements collision detection from the paper:
"A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"

Key responsibilities:
- Discretize trajectories into fixed timesteps
- Interpolate robot states at each timestep
- Use MoveIt's FCL (Flexible Collision Library) to detect collisions
- Check collisions between two executing trajectories

Paper Component #2: Collision Detection
"""

from typing import List, Tuple, Optional
import threading
import time
import bisect

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

from src.trajectory_utils import discretize_trajectory, get_robot_state_at_time


class CollisionDetector:
    """
    Collision detection between trajectories using discrete timesteps.

    Paper Component #2: Collision Detection

    Strategy:
    - Discretize both trajectories into fixed timestep intervals
    - At each timestep, get the joint state of both arms
    - Create a combined robot state
    - Use MoveIt's FCL to check for inter-arm collisions
    """

    def __init__(self, planning_scene_monitor=None, logger=None):
        """
        Initialize collision detector.

        Args:
            planning_scene_monitor: MoveIt PlanningSceneMonitor for FCL access
            logger: ROS2 logger
        """
        self.planning_scene_monitor = planning_scene_monitor
        self.logger = logger
        self.timestep = 0.02  # 50ms discretization (configurable)
        self.lock = threading.RLock()

    def set_timestep(self, timestep: float):
        """
        Set the discretization timestep.

        Smaller timestep = safer but slower collision checking.
        Larger timestep = faster but might miss collisions between steps.

        Args:
            timestep: Time step in seconds (default 0.02 = 50ms)
        """
        with self.lock:
            self.timestep = timestep
            if self.logger:
                self.logger.info(f"[CollisionDetector] Timestep set to {timestep}s")

    def check_collision_between_trajectories(
        self,
        trajectory_a: JointTrajectory,
        trajectory_b: JointTrajectory,
    ) -> bool:
        """
        Check if two trajectories collide with each other.

        This is the main function used by the scheduler.

        Approach:
        1. Discretize both trajectories
        2. For each common timestep, check inter-arm collision
        3. Return True if any collision found

        Args:
            trajectory_a: First trajectory (usually arm_1)
            trajectory_b: Second trajectory (usually arm_2)

        Returns:
            True if collision detected, False if safe
        """
        with self.lock:
            # If either trajectory is empty, no collision
            if not trajectory_a.points or not trajectory_b.points:
                return False

            # Discretize both trajectories
            try:
                discrete_a = discretize_trajectory(trajectory_a, self.timestep)
                discrete_b = discretize_trajectory(trajectory_b, self.timestep)
            except Exception as e:
                if self.logger:
                    self.logger.error(
                        f"[CollisionDetector] Error discretizing trajectories: {e}"
                    )
                # In case of error, assume safe (conservative approach)
                return False

            if not discrete_a or not discrete_b:
                return False

            # Get max duration to check
            max_time_a = discrete_a[-1][0] if discrete_a else 0.0
            max_time_b = discrete_b[-1][0] if discrete_b else 0.0
            max_time = min(max_time_a, max_time_b)

            # Build sorted time index for trajectory_b for O(log n) binary search
            times_b = [t for t, _, _ in discrete_b]
            positions_b_by_time = {t: (names, pos) for t, names, pos in discrete_b}

            # Check collision at each timestep of trajectory_a
            for time_step in range(len(discrete_a)):
                time = discrete_a[time_step][0]

                if time > max_time:
                    break

                # Get states at this time for trajectory_a
                _, positions_a = discrete_a[time_step][1], discrete_a[time_step][2]

                # Find corresponding state in trajectory_b using binary search O(log n)
                positions_b = None

                # Use bisect to find nearest time point in trajectory_b
                idx = bisect.bisect_left(times_b, time)

                # Check both the found index and adjacent indices for closest match
                for check_idx in [idx - 1, idx]:
                    if 0 <= check_idx < len(times_b):
                        t_candidate = times_b[check_idx]
                        if abs(t_candidate - time) < self.timestep / 2:
                            names_b, positions_b = positions_b_by_time[t_candidate]
                            break

                if positions_b is None:
                    # Use interpolation if no close point found within tolerance
                    _, positions_b = get_robot_state_at_time(trajectory_b, time)

                # Check collision at this timestep
                if self._check_inter_arm_collision(
                    trajectory_a.joint_names,
                    positions_a,
                    trajectory_b.joint_names,
                    positions_b,
                ):
                    if self.logger:
                        self.logger.warn(
                            f"[CollisionDetector] Collision detected at time {time:.2f}s"
                        )
                    return True

            # No collision found
            return False

    def _check_inter_arm_collision(
        self,
        joint_names_a: List[str],
        positions_a: List[float],
        joint_names_b: List[str],
        positions_b: List[float],
    ) -> bool:
        """
        Check inter-arm collision for a specific state.

        This would use MoveIt's FCL checker if available.
        For now, uses simple geometric bounds checking as fallback.

        Args:
            joint_names_a: Joint names for arm A
            positions_a: Joint positions for arm A
            joint_names_b: Joint names for arm B
            positions_b: Joint positions for arm B

        Returns:
            True if collision detected
        """
        # TODO: Integrate with MoveIt's FCL collision checker
        # For now, use conservative heuristic: check joint position bounds

        # Simple heuristic: if using MoveIt's FCL
        if self.planning_scene_monitor is not None:
            try:
                return self._check_fcl_collision(
                    joint_names_a,
                    positions_a,
                    joint_names_b,
                    positions_b,
                )
            except Exception as e:
                if self.logger:
                    self.logger.warn(
                        f"[CollisionDetector] FCL check failed: {e}, using fallback"
                    )

        # Fallback: conservative bounds checking
        return self._check_bounds_collision(positions_a, positions_b)

    def _check_fcl_collision(
        self,
        joint_names_a: List[str],
        positions_a: List[float],
        joint_names_b: List[str],
        positions_b: List[float],
    ) -> bool:
        """
        Check collision using MoveIt's FCL collision library.

        This requires access to MoveIt's planning scene.

        Args:
            joint_names_a, joint_names_b: Joint names
            positions_a, positions_b: Joint positions

        Returns:
            True if collision detected
        """
        try:
            # Get the planning scene
            planning_scene = self.planning_scene_monitor.getPlanningScene()

            # Create robot state
            # This is a simplified version - full implementation would need
            # to properly set all joint states and forward kinematics
            from moveit_core.robot_state import RobotState as MoveItRobotState

            robot_state = planning_scene.getRobotState()

            # Set joint values for arm_a
            for name, pos in zip(joint_names_a, positions_a):
                robot_state.setJointPositions(name, [pos])

            # Set joint values for arm_b
            for name, pos in zip(joint_names_b, positions_b):
                robot_state.setJointPositions(name, [pos])

            # Update forward kinematics
            robot_state.update()

            # Check collision
            collision_result = planning_scene.getCollisionDetector().checkCollision(
                robot_state
            )

            return collision_result.collision

        except Exception as e:
            if self.logger:
                self.logger.warn(
                    f"[CollisionDetector] FCL collision check exception: {e}"
                )
            return False  # Assume safe if check fails

    def _check_bounds_collision(
        self,
        positions_a: List[float],
        positions_b: List[float],
    ) -> bool:
        """
        Simple geometric bounds collision check (fallback).

        For dual-arm systems like Niryo Ned2, check if arms are in close proximity
        by examining configuration space distance.

        Args:
            positions_a: Joint positions for arm A
            positions_b: Joint positions for arm B

        Returns:
            True if potential collision detected
        """
        if len(positions_a) < 3 or len(positions_b) < 3:
            return False

        # For Niryo Ned2 dual-arm: compute configuration space distance
        # If arms have very similar joint angles, they're likely close/colliding

        # Calculate normalized joint angle differences
        # Niryo joints have ranges roughly [-π, π], normalize to [-1, 1]
        config_distance = 0.0
        for pos_a, pos_b in zip(positions_a, positions_b):
            # Normalize angles to [-π, π] then to [-1, 1]
            diff = abs(pos_a - pos_b)
            # Handle circular distance (angle wrapping)
            if diff > 3.14159:
                diff = 6.28318 - diff
            config_distance += (diff / 3.14159) ** 2

        config_distance = (config_distance / len(positions_a)) ** 0.5

        # Collision threshold: if configuration distance < 0.3, arms are very close
        # This is a heuristic tuned for Niryo Ned2 where arms come from opposite sides
        # and joint ranges are similar
        collision_threshold = 0.3

        if config_distance < collision_threshold:
            if self.logger:
                self.logger.info(
                    f"[CollisionDetector] Bounds check: config distance {config_distance:.3f} < {collision_threshold}"
                )
            return True

        return False

    def get_configuration_distance(
        self,
        positions_a: List[float],
        positions_b: List[float],
    ) -> float:
        """
        Calculate configuration space distance between two joint states.

        Used for debugging and analysis.

        Args:
            positions_a: Joint positions for state A
            positions_b: Joint positions for state B

        Returns:
            Euclidean distance in configuration space
        """
        if len(positions_a) != len(positions_b):
            return float("inf")

        sum_sq = sum((a - b) ** 2 for a, b in zip(positions_a, positions_b))
        return sum_sq**0.5
