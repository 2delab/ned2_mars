"""
Async Executor for Dual-Arm MoveIt2 Asynchronous Trajectory Execution

This package implements asynchronous trajectory execution for multi-robot/multi-arm
robotic setups based on the paper:
"A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"
(Stoop et al., IROS 2023 Workshop)

Components:
- scheduler: Central trajectory queue and dispatch logic
- collision_detector: Discrete-time collision detection between trajectories
- online_collision_monitor: Real-time safety monitoring during execution
- trajectory_utils: Helper functions for trajectory conversion and manipulation
- async_executor_node: Main ROS2 node integrating all components

Usage:
    ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py

For simulation: Uses standard ROS2 JointTrajectoryControllers
For real hardware: Can be adapted to use Niryo ManageTrajectory.srv
"""

__version__ = "0.1.0"
__author__ = "Bachelor's Project"

from .scheduler import Scheduler
from .collision_detector import CollisionDetector
from .online_collision_monitor import OnlineCollisionMonitor
from .trajectory_utils import (
    convert_trajectory_msgs_to_niryo,
    split_trajectory_by_arm,
    get_robot_state_at_time,
)

__all__ = [
    "Scheduler",
    "CollisionDetector",
    "OnlineCollisionMonitor",
    "convert_trajectory_msgs_to_niryo",
    "split_trajectory_by_arm",
    "get_robot_state_at_time",
]
