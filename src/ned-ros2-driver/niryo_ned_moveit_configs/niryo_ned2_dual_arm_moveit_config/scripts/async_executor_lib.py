#!/usr/bin/env python3
"""
Async Executor Library - Embedded executor for dual-arm async trajectory execution.

"""

import threading
import copy
from typing import Optional, Callable, Dict
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class AsyncDualArmExecutor:
    """
    Embedded async executor for dual-arm trajectories.

    Usage:
        executor = AsyncDualArmExecutor(node)
        executor.execute_async("arm_1", trajectory1, callback)
        executor.execute_async("arm_2", trajectory2, callback)
    """

    def __init__(
        self,
        node: Node,
        max_velocity_scaling_factor: float = 0.2,
        max_acceleration_scaling_factor: float = 0.2,
    ):
        """
        Initialize executor.

        Args:
            node: ROS2 Node for creating action clients
            max_velocity_scaling_factor: Speed scaling factor (0.0-1.0, default 0.2)
            max_acceleration_scaling_factor: Acceleration scaling factor (0.0-1.0, default 0.2)
        """
        self.node = node
        self.action_callback_group = ReentrantCallbackGroup()
        self.max_velocity_scaling_factor = max_velocity_scaling_factor
        self.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        # Action clients - send to MoveIt trajectory executor action servers
        self.arm_1_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/arm_1/follow_joint_trajectory_prefixed",
            callback_group=self.action_callback_group,
        )

        self.arm_2_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/arm_2/follow_joint_trajectory_prefixed",
            callback_group=self.action_callback_group,
        )

        self.clients = {
            "arm_1": self.arm_1_client,
            "arm_2": self.arm_2_client,
        }

        # Track active executions
        self.active_executions: Dict[str, Dict] = {}
        self.lock = threading.RLock()

    def _split_trajectory_for_arm(
        self, trajectory: JointTrajectory, arm_id: str
    ) -> JointTrajectory:
        """Split combined trajectory to only include this arm's joints."""
        arm_indices = [
            i
            for i, name in enumerate(trajectory.joint_names)
            if name.startswith(arm_id)
        ]

        if not arm_indices:
            return JointTrajectory()

        split_trajectory = JointTrajectory()
        split_trajectory.joint_names = [trajectory.joint_names[i] for i in arm_indices]

        for point in trajectory.points:
            new_point = JointTrajectoryPoint()
            new_point.positions = [point.positions[i] for i in arm_indices]
            new_point.time_from_start = point.time_from_start
            split_trajectory.points.append(new_point)

        return split_trajectory

    def _apply_trajectory_scaling(
        self,
        trajectory: JointTrajectory,
        velocity_scaling: float,
        acceleration_scaling: float,
    ) -> JointTrajectory:
        """
        Apply velocity and acceleration scaling to trajectory.

        Args:
            trajectory: Original trajectory
            velocity_scaling: Velocity scaling factor (0.0-1.0)
            acceleration_scaling: Acceleration scaling factor (0.0-1.0)

        Returns:
            Scaled trajectory with adjusted time_from_start values
        """
        # Create a deep copy to avoid modifying original
        scaled_trajectory = copy.deepcopy(trajectory)

        # Scale time_from_start for each point
        # Scaling formula: scaled_time = original_time / velocity_scaling_factor
        # Example: 1 sec with 0.2 scaling = 5 sec execution (slower motion)
        if velocity_scaling > 0:
            for point in scaled_trajectory.points:
                # Get current time in seconds
                original_time_sec = (
                    point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                )

                # Scale the time
                scaled_time_sec = original_time_sec / velocity_scaling

                # Convert back to seconds and nanoseconds
                sec = int(scaled_time_sec)
                nanosec = int((scaled_time_sec - sec) * 1e9)

                # Update the time_from_start
                point.time_from_start.sec = sec
                point.time_from_start.nanosec = nanosec

        return scaled_trajectory

    def execute_async(
        self,
        arm_id: str,
        trajectory: JointTrajectory,
        callback: Optional[Callable[[str, bool, str], None]] = None,
        timeout_sec: float = 30.0,
        max_velocity_scaling_factor: Optional[float] = None,
        max_acceleration_scaling_factor: Optional[float] = None,
    ):
        """
        Execute trajectory asynchronously with callback.

        Args:
            arm_id: "arm_1" or "arm_2"
            trajectory: JointTrajectory to execute
            callback: Function(arm_id, success, error_msg) called when complete
            timeout_sec: Timeout for execution
            max_velocity_scaling_factor: Optional override for velocity scaling (0.0-1.0)
            max_acceleration_scaling_factor: Optional override for acceleration scaling (0.0-1.0)
        """
        # Use provided scaling factors or fall back to instance defaults
        velocity_scaling = (
            max_velocity_scaling_factor
            if max_velocity_scaling_factor is not None
            else self.max_velocity_scaling_factor
        )
        acceleration_scaling = (
            max_acceleration_scaling_factor
            if max_acceleration_scaling_factor is not None
            else self.max_acceleration_scaling_factor
        )

        # Split trajectory for this arm
        split_traj = self._split_trajectory_for_arm(trajectory, arm_id)

        if not split_traj.joint_names:
            if callback:
                callback(arm_id, False, "No joints found for arm")
            return

        # Apply trajectory scaling
        scaled_traj = self._apply_trajectory_scaling(
            split_traj, velocity_scaling, acceleration_scaling
        )

        # Get controller client
        client = self.clients[arm_id]

        # Create goal
        goal = FollowJointTrajectory.Goal(trajectory=scaled_traj)

        # Send goal asynchronously with callbacks
        def on_accepted(future):
            try:
                goal_handle = future.result()

                if not goal_handle.accepted:
                    if callback:
                        callback(arm_id, False, "Goal rejected by controller")
                    return

                # Goal accepted - wait for result
                result_future = goal_handle.get_result_async()

                def on_result(future):
                    try:
                        result = future.result().result
                        success = (
                            result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
                        )
                        error_msg = (
                            f"error_code={result.error_code}" if not success else ""
                        )

                        if callback:
                            callback(arm_id, success, error_msg)
                    except Exception as e:
                        if callback:
                            callback(arm_id, False, str(e))

                result_future.add_done_callback(on_result)

            except Exception as e:
                if callback:
                    callback(arm_id, False, str(e))

        # Send goal
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(on_accepted)
