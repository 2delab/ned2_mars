#!/usr/bin/env python3
"""
Asynchronous Trajectory Execution Node for Dual-Arm MoveIt2 System.

This node implements async execution for the Niryo Ned2 dual-arm system based on:
"A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"
(Stoop et al., IROS 2023 Workshop)

Main responsibilities:
- Accept trajectory requests from MoveIt
- Use Scheduler to manage execution and backlog
- Use CollisionDetector for inter-arm collision checking
- Use OnlineCollisionMonitor for real-time safety
- Route trajectories to ROS2 controllers for execution
- Publish status feedback

For simulation: Uses ROS2 JointTrajectoryControllers
For real hardware: Can be adapted to use Niryo ManageTrajectory.srv
"""

import threading
import time
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32, Bool

from src.scheduler import Scheduler, TrajectoryState
from src.collision_detector import CollisionDetector
from src.online_collision_monitor import OnlineCollisionMonitor, SafetyState
from src.trajectory_utils import (
    split_trajectory_by_arm,
    trajectory_duration,
    convert_trajectory_msgs_to_niryo,
)


class AsyncExecutorStatus:
    """Status message for async executor (could be ROS message)"""

    def __init__(self):
        self.state = "IDLE"
        self.arm_1_state = "idle"
        self.arm_2_state = "idle"
        self.execution_queue_size = 0
        self.backlog_queue_size_1 = 0
        self.backlog_queue_size_2 = 0
        self.safety_state = "SAFE"
        self.collisions_detected = 0


class AsyncExecutorNode(Node):
    """
    Main ROS2 node for asynchronous trajectory execution.

    Integrates all components:
    - Scheduler (Component #1)
    - CollisionDetector (Component #2)
    - OnlineCollisionMonitor (Component #3)
    """

    def __init__(self):
        super().__init__("async_executor_node")

        # Get parameters
        self.declare_parameter("backlog_timeout", 5.0)
        self.declare_parameter("collision_timestep", 0.02)
        self.declare_parameter("monitoring_frequency", 20.0)

        backlog_timeout = self.get_parameter("backlog_timeout").value
        collision_timestep = self.get_parameter("collision_timestep").value
        monitoring_freq = self.get_parameter("monitoring_frequency").value

        self.get_logger().info("=" * 70)
        self.get_logger().info("ASYNC EXECUTOR NODE STARTING")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"Backlog timeout: {backlog_timeout}s, "
            f"Collision timestep: {collision_timestep}s, "
            f"Monitoring freq: {monitoring_freq} Hz"
        )

        # Create components
        self.collision_detector = CollisionDetector(logger=self.get_logger())
        self.collision_detector.set_timestep(collision_timestep)

        self.scheduler = Scheduler(
            collision_detector=self.collision_detector,
            logger=self.get_logger(),
        )

        self.collision_monitor = OnlineCollisionMonitor(
            logger=self.get_logger(),
            emergency_stop_callback=self._on_emergency_stop,
        )
        self.collision_monitor.set_check_frequency(monitoring_freq)

        # Callback groups for thread isolation
        self.action_callback_group = ReentrantCallbackGroup()
        self.joint_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.execution_callback_group = ReentrantCallbackGroup()

        # QoS for real-time data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos,
            callback_group=self.joint_state_callback_group,
        )

        # Action servers (accept trajectories from MoveIt)
        self.arm_1_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_1/follow_joint_trajectory",
            execute_callback=self._execute_arm_1_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_callback_group,
        )

        self.arm_2_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_2/follow_joint_trajectory",
            execute_callback=self._execute_arm_2_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_callback_group,
        )

        # Action clients (send trajectories to actual controllers at different namespace)
        # MoveIt sends to /arm_1/follow_joint_trajectory (async executor)
        # Async executor forwards to /arm_1_controller/follow_joint_trajectory (actual controller)
        self.arm_1_controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_1_controller/follow_joint_trajectory",
            callback_group=self.action_callback_group,
        )

        self.arm_2_controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_2_controller/follow_joint_trajectory",
            callback_group=self.action_callback_group,
        )

        # Publishers for status
        self.status_pub = self.create_publisher(
            String,
            "/async_executor/status",
            qos,
        )

        self.collisions_pub = self.create_publisher(
            Float32,
            "/async_executor/collisions_detected",
            qos,
        )

        self.safety_state_pub = self.create_publisher(
            String,
            "/async_executor/safety_state",
            qos,
        )

        # Execution loop timer
        self.execution_timer = self.create_timer(
            0.01,  # 100 Hz execution loop
            self._execution_loop,
            callback_group=self.execution_callback_group,
        )

        # Status publishing timer
        self.status_timer = self.create_timer(
            0.5,  # 2 Hz status publishing
            self._publish_status,
            callback_group=self.execution_callback_group,
        )

        # Configuration
        self.backlog_timeout = backlog_timeout
        self.emergency_stop_active = False
        self.emergency_stop_lock = threading.RLock()

        # Start monitoring
        self.collision_monitor.start_monitoring()

        self.get_logger().info("Async executor node ready")
        self.get_logger().info("=" * 70)

    def _goal_callback(self, goal_request):
        """Accept all trajectory goals"""
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Allow canceling goals"""
        return CancelResponse.ACCEPT

    def _on_joint_state(self, msg: JointState):
        """Callback for joint state updates"""
        self.collision_monitor.on_joint_state(msg)

    def _execute_arm_1_callback(self, goal_handle):
        """Handle trajectory request for arm 1"""
        return self._execute_arm_callback(goal_handle, "arm_1")

    def _execute_arm_2_callback(self, goal_handle):
        """Handle trajectory request for arm 2"""
        return self._execute_arm_callback(goal_handle, "arm_2")

    def _execute_arm_callback(self, goal_handle, arm_id: str):
        """
        Handle trajectory execution request.

        Returns immediately (non-blocking).
        Actual execution happens in execution loop.
        """
        trajectory = goal_handle.request.trajectory

        self.get_logger().info(
            f"[{arm_id}] Trajectory request received "
            f"({len(trajectory.points)} points, "
            f"{trajectory_duration(trajectory):.2f}s)"
        )

        # Add to scheduler
        self.scheduler.on_trajectory_request(
            arm_id=arm_id,
            trajectory=trajectory,
            goal_handle=goal_handle,
            backlog_timeout=self.backlog_timeout,
        )

        # Return immediately (non-blocking action)
        # The trajectory will be executed in the execution loop
        # We don't return a result here - it's handled by _execution_loop

    def _execution_loop(self):
        """
        Main execution loop - runs at 100 Hz.

        Dispatches queued trajectories to controllers.
        """
        with self.emergency_stop_lock:
            if self.emergency_stop_active:
                return

        # Get next trajectory to execute
        next_traj = self.scheduler.get_next_for_execution()
        if next_traj is None:
            return

        arm_id, scheduled_traj = next_traj

        # Skip if already executing (state was set by controller acceptance callback)
        if scheduled_traj.state == TrajectoryState.EXECUTING:
            return

        self.get_logger().info(f"[{arm_id}] Sending to controller")

        # Send to controller
        # Note: state is updated to EXECUTING only after controller accepts the goal
        self._send_trajectory_to_controller(arm_id, scheduled_traj)

    def _send_trajectory_to_controller(self, arm_id: str, scheduled_traj):
        """Send trajectory to the appropriate controller"""
        trajectory = scheduled_traj.trajectory
        goal_handle = scheduled_traj.goal_handle

        # Choose the appropriate controller client
        if arm_id == "arm_1":
            controller_client = self.arm_1_controller_client
        else:
            controller_client = self.arm_2_controller_client

        # Wait for controller to be ready
        if not controller_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"[{arm_id}] Controller server not available")
            goal_handle.abort()
            self.scheduler.on_trajectory_completion(arm_id)
            return

        # Create goal
        controller_goal = FollowJointTrajectory.Goal(trajectory=trajectory)

        # Send goal to controller with feedback callback
        def feedback_callback(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        send_goal_future = controller_client.send_goal_async(
            controller_goal,
            feedback_callback=feedback_callback,
        )

        # Wait for goal acceptance and result in background
        def result_callback(future):
            try:
                goal_handle_controller = future.result()
                if not goal_handle_controller.accepted:
                    self.get_logger().error(f"[{arm_id}] Goal rejected by controller")
                    goal_handle.abort()
                    self.scheduler.on_trajectory_completion(arm_id)
                    return

                # Controller accepted - now mark as executing
                scheduled_traj.state = TrajectoryState.EXECUTING
                scheduled_traj.start_time = time.time()
                self.get_logger().info(
                    f"[{arm_id}] Controller accepted, trajectory executing"
                )

                # Wait for result
                result_future = goal_handle_controller.get_result_async()

                def result_ready_callback(result_future):
                    try:
                        result = result_future.result().result

                        # Handle result
                        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                            self.get_logger().info(
                                f"[{arm_id}] Trajectory completed successfully"
                            )
                            goal_handle.succeed()
                        else:
                            self.get_logger().error(
                                f"[{arm_id}] Trajectory failed with code {result.error_code}"
                            )
                            goal_handle.abort()

                        # Notify scheduler
                        self.scheduler.on_trajectory_completion(arm_id)

                    except Exception as e:
                        self.get_logger().error(
                            f"[{arm_id}] Error getting trajectory result: {e}"
                        )
                        goal_handle.abort()
                        self.scheduler.on_trajectory_completion(arm_id)

                result_future.add_done_callback(result_ready_callback)

            except Exception as e:
                self.get_logger().error(f"[{arm_id}] Error sending goal: {e}")
                goal_handle.abort()
                self.scheduler.on_trajectory_completion(arm_id)

        send_goal_future.add_done_callback(result_callback)

    def _on_emergency_stop(self):
        """Called by collision monitor when collision is detected"""
        with self.emergency_stop_lock:
            self.emergency_stop_active = True

        self.get_logger().error("!!! EMERGENCY STOP TRIGGERED - Collision detected !!!")

        # Abort all executing trajectories
        for arm_id in ["arm_1", "arm_2"]:
            traj = self.scheduler.get_arm_trajectory(arm_id)
            if traj and traj.goal_handle:
                traj.goal_handle.abort()
            self.scheduler.on_trajectory_completion(arm_id)

    def _publish_status(self):
        """Publish executor status (2 Hz)"""
        queue_status = self.scheduler.get_execution_queue_status()
        monitor_stats = self.collision_monitor.get_statistics()
        safety_state = self.collision_monitor.get_safety_state()

        # Build status string
        exec_q = queue_status["execution_queue"]
        backlog_q = queue_status["backlog_queue"]

        status_str = (
            f"EXEC: arm_1={exec_q['arm_1'] is not None}, "
            f"arm_2={exec_q['arm_2'] is not None} | "
            f"BACKLOG: arm_1={len(backlog_q['arm_1'])}, "
            f"arm_2={len(backlog_q['arm_2'])} | "
            f"SAFETY: {safety_state.value} | "
            f"Collisions: {monitor_stats['collisions_detected']}"
        )

        self.status_pub.publish(String(data=status_str))
        self.safety_state_pub.publish(String(data=safety_state.value))
        self.collisions_pub.publish(
            Float32(data=float(monitor_stats["collisions_detected"]))
        )

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.collision_monitor.stop_monitoring()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = AsyncExecutorNode()

    # Use MultiThreadedExecutor for proper threading
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
