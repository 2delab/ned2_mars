"""
Central scheduler for asynchronous trajectory execution.

This module implements the scheduler component from the paper:
"A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"

Key responsibilities:
- Manage execution queue (trajectories currently executing)
- Manage backlog queue (trajectories waiting for collision-free slot)
- Collision check new trajectories against running ones
- Dispatch collision-free trajectories to controllers
- Handle backlog timeouts to prevent deadlocks
"""

from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import threading
import time
from copy import deepcopy

from trajectory_msgs.msg import JointTrajectory
from src.trajectory_utils import trajectory_duration, get_robot_state_at_time


class TrajectoryState(Enum):
    """State of a trajectory in the scheduler"""

    PENDING = "pending"  # Just received
    QUEUED = "queued"  # In execution queue
    IN_BACKLOG = "in_backlog"  # Waiting due to collision
    EXECUTING = "executing"  # Currently being executed
    COMPLETED = "completed"  # Finished successfully
    ABORTED = "aborted"  # Stopped due to timeout or collision
    TIMEOUT = "timeout"  # Backlog timeout expired


@dataclass
class ScheduledTrajectory:
    """Data structure for a trajectory in the scheduler"""

    arm_id: str  # "arm_1" or "arm_2"
    trajectory: JointTrajectory  # The actual trajectory
    goal_handle: Optional[object] = None  # Action server goal handle (type varies)
    state: TrajectoryState = TrajectoryState.PENDING
    queued_time: float = 0.0  # When added to queue
    backlog_time: Optional[float] = None  # When added to backlog
    start_time: Optional[float] = None  # When execution started
    end_time: Optional[float] = None  # When execution completed
    backlog_timeout: float = 5.0  # Max time in backlog (seconds)


class Scheduler:
    """
    Central scheduler managing asynchronous trajectory execution.

    Paper Component #1: Central Scheduler
    """

    def __init__(self, collision_detector=None, logger=None):
        """
        Initialize scheduler.

        Args:
            collision_detector: CollisionDetector instance for checking collisions
            logger: ROS2 logger for output
        """
        self.collision_detector = collision_detector
        self.logger = logger

        # Execution queue: trajectories currently executing
        # Key: arm_id, Value: ScheduledTrajectory
        self.execution_queue: Dict[str, Optional[ScheduledTrajectory]] = {
            "arm_1": None,
            "arm_2": None,
        }

        # Backlog queue: trajectories waiting for execution
        # Key: arm_id, Value: List of ScheduledTrajectories
        self.backlog_queue: Dict[str, List[ScheduledTrajectory]] = {
            "arm_1": [],
            "arm_2": [],
        }

        # Thread safety
        self.lock = threading.RLock()

        # Statistics
        self.stats = {
            "total_requests": 0,
            "executed_immediately": 0,
            "executed_from_backlog": 0,
            "backlog_timeouts": 0,
            "collisions_detected": 0,
        }

    def on_trajectory_request(
        self,
        arm_id: str,
        trajectory: JointTrajectory,
        goal_handle: object,
        backlog_timeout: float = 5.0,
    ) -> bool:
        """
        Handle a new trajectory request.

        Checks collision with currently executing trajectories.
        If safe: adds to execution queue
        If collision: adds to backlog queue

        Args:
            arm_id: "arm_1" or "arm_2"
            trajectory: JointTrajectory to execute
            goal_handle: Action server goal handle for this trajectory
            backlog_timeout: Max time to wait in backlog before aborting

        Returns:
            True if trajectory was queued or added to backlog
            False if immediate rejection
        """
        with self.lock:
            self.stats["total_requests"] += 1

            scheduled_traj = ScheduledTrajectory(
                arm_id=arm_id,
                trajectory=deepcopy(trajectory),
                goal_handle=goal_handle,
                state=TrajectoryState.PENDING,
                queued_time=time.time(),
                backlog_timeout=backlog_timeout,
            )

            # Log
            if self.logger:
                self.logger.info(f"[Scheduler] New trajectory request for {arm_id}")

            # Check collision with the OTHER arm's executing trajectory
            other_arm = "arm_2" if arm_id == "arm_1" else "arm_1"
            running_traj = self.execution_queue[other_arm]

            collision_detected = False
            if running_traj is not None:
                # Check collision between new trajectory and running trajectory
                if self.collision_detector:
                    collision_detected = (
                        self.collision_detector.check_collision_between_trajectories(
                            running_traj.trajectory,
                            trajectory,
                        )
                    )

            if collision_detected:
                # Add to backlog
                self.backlog_queue[arm_id].append(scheduled_traj)
                scheduled_traj.state = TrajectoryState.IN_BACKLOG
                scheduled_traj.backlog_time = time.time()
                self.stats["collisions_detected"] += 1

                if self.logger:
                    self.logger.warn(
                        f"[Scheduler] Collision detected for {arm_id}, "
                        f"adding to backlog (timeout: {backlog_timeout}s)"
                    )
            else:
                # Add to execution queue
                self.execution_queue[arm_id] = scheduled_traj
                scheduled_traj.state = TrajectoryState.QUEUED
                self.stats["executed_immediately"] += 1

                if self.logger:
                    self.logger.info(f"[Scheduler] {arm_id} added to execution queue")

            return True

    def on_trajectory_completion(self, arm_id: str) -> bool:
        """
        Called when a trajectory finishes executing.

        Removes trajectory from execution queue.
        Promotes waiting trajectories from backlog if collision-free.

        Args:
            arm_id: "arm_1" or "arm_2"

        Returns:
            True if trajectory was in execution queue
        """
        with self.lock:
            if self.execution_queue[arm_id] is None:
                return False

            # Mark as completed
            self.execution_queue[arm_id].state = TrajectoryState.COMPLETED
            self.execution_queue[arm_id].end_time = time.time()

            if self.logger:
                self.logger.info(f"[Scheduler] {arm_id} trajectory completed")

            # Clear execution queue for this arm
            self.execution_queue[arm_id] = None

            # Try to promote trajectories from backlog
            self._promote_from_backlog(arm_id)

            return True

    def _promote_from_backlog(self, arm_id: str):
        """
        Try to promote trajectories from backlog to execution queue.

        Checks each backlog trajectory for collisions with running trajectories.
        If collision-free, moves to execution queue.
        If timeout expired, aborts trajectory.

        Args:
            arm_id: "arm_1" or "arm_2"
        """
        backlog = self.backlog_queue[arm_id]
        current_time = time.time()

        # Check each trajectory in backlog
        promoted = []
        timed_out = []

        for i, backlog_traj in enumerate(backlog):
            # Check timeout
            if backlog_traj.backlog_time is None:
                continue

            time_in_backlog = current_time - backlog_traj.backlog_time

            if time_in_backlog > backlog_traj.backlog_timeout:
                # Timeout expired
                backlog_traj.state = TrajectoryState.TIMEOUT
                timed_out.append(i)
                self.stats["backlog_timeouts"] += 1

                if self.logger:
                    self.logger.error(
                        f"[Scheduler] {arm_id} trajectory backlog timeout "
                        f"({time_in_backlog:.1f}s > {backlog_traj.backlog_timeout}s)"
                    )
            else:
                # Check collision with other arm
                other_arm = "arm_2" if arm_id == "arm_1" else "arm_1"
                running_traj = self.execution_queue[other_arm]

                collision_detected = False
                if running_traj is not None and self.collision_detector:
                    collision_detected = (
                        self.collision_detector.check_collision_between_trajectories(
                            running_traj.trajectory,
                            backlog_traj.trajectory,
                        )
                    )

                if not collision_detected:
                    # Promote to execution queue
                    self.execution_queue[arm_id] = backlog_traj
                    backlog_traj.state = TrajectoryState.QUEUED
                    promoted.append(i)
                    self.stats["executed_from_backlog"] += 1

                    if self.logger:
                        self.logger.info(
                            f"[Scheduler] {arm_id} promoted from backlog "
                            f"to execution queue"
                        )
                    # Only promote one at a time
                    break

        # Remove promoted and timed-out from backlog (reverse order to maintain indices)
        for i in sorted(set(promoted + timed_out), reverse=True):
            backlog.pop(i)

    def get_execution_queue_status(self) -> Dict:
        """
        Get current status of execution and backlog queues.

        Returns:
            Dictionary with queue status information
        """
        with self.lock:
            status = {
                "execution_queue": {},
                "backlog_queue": {},
                "stats": deepcopy(self.stats),
            }

            for arm_id in ["arm_1", "arm_2"]:
                if self.execution_queue[arm_id] is not None:
                    traj = self.execution_queue[arm_id]
                    status["execution_queue"][arm_id] = {
                        "state": traj.state.value,
                        "duration": trajectory_duration(traj.trajectory),
                        "queued_time": traj.queued_time,
                    }
                else:
                    status["execution_queue"][arm_id] = None

                status["backlog_queue"][arm_id] = [
                    {
                        "state": t.state.value,
                        "duration": trajectory_duration(t.trajectory),
                        "backlog_time": t.backlog_time,
                    }
                    for t in self.backlog_queue[arm_id]
                ]

            return status

    def get_next_for_execution(self) -> Optional[Tuple[str, ScheduledTrajectory]]:
        """
        Get the next trajectory ready for execution.

        Called by execution loop to dispatch trajectories.

        Returns:
            Tuple of (arm_id, ScheduledTrajectory) or None if nothing to execute
        """
        with self.lock:
            for arm_id in ["arm_1", "arm_2"]:
                traj = self.execution_queue[arm_id]
                if traj is not None and traj.state in [
                    TrajectoryState.QUEUED,
                    TrajectoryState.PENDING,
                ]:
                    return (arm_id, traj)
            return None

    def get_arm_trajectory(self, arm_id: str) -> Optional[ScheduledTrajectory]:
        """Get the currently executing trajectory for an arm"""
        with self.lock:
            return self.execution_queue[arm_id]

    def get_all_executing_trajectories(self) -> Dict[str, ScheduledTrajectory]:
        """Get all currently executing trajectories"""
        with self.lock:
            return {
                arm: traj
                for arm, traj in self.execution_queue.items()
                if traj is not None
            }
