#!/usr/bin/env python3


import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class JointStatePrefixer(Node):
    """Aggregates and prefixes joint states from multiple robot namespaces."""

    def __init__(self):
        super().__init__("joint_state_prefixer")

        # Parameters
        self.declare_parameter("robot_namespaces", ["arm_1", "arm_2"])
        self.declare_parameter("publish_frequency", 40.0)
        self.declare_parameter("trajectory_timeout_sec", 120.0)
        self.declare_parameter("server_timeout_sec", 30.0)

        self.namespaces = self.get_parameter("robot_namespaces").value
        self.pub_freq = self.get_parameter("publish_frequency").value
        self.traj_timeout = self.get_parameter("trajectory_timeout_sec").value
        self.srv_timeout = self.get_parameter("server_timeout_sec").value

        if not self.namespaces:
            raise ValueError("robot_namespaces cannot be empty")

        # Separate callback groups to prevent blocking
        self.joint_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Joint state aggregation
        self.joint_states: Dict[str, Optional[JointState]] = {
            ns: None for ns in self.namespaces
        }
        self.state_lock = threading.Lock()

        # QoS for real-time sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", qos)

        for ns in self.namespaces:
            self.create_subscription(
                JointState,
                f"/{ns}/joint_states",
                lambda msg, namespace=ns: self._on_joint_state(msg, namespace),
                qos,
                callback_group=self.joint_state_callback_group,
            )

        # Trajectory proxying
        self.traj_servers: Dict[str, ActionServer] = {}
        self.hw_clients: Dict[str, ActionClient] = {}
        self._setup_trajectory_proxy()

        # Publishing timer
        self.create_timer(
            1.0 / self.pub_freq,
            self._publish_combined_states,
            callback_group=self.joint_state_callback_group,
        )

        self.get_logger().info(
            f"Started with namespaces: {self.namespaces}, "
            f"freq: {self.pub_freq}Hz, "
            f"timeouts: server={self.srv_timeout}s, traj={self.traj_timeout}s"
        )

    def _on_joint_state(self, msg: JointState, namespace: str):
        """Thread-safe update of joint state for a namespace."""
        with self.state_lock:
            self.joint_states[namespace] = msg

    def _publish_combined_states(self):
        """Combine and publish joint states with prefixed names."""
        with self.state_lock:
            if any(state is None for state in self.joint_states.values()):
                return

            combined = JointState()
            combined.header.stamp = self.get_clock().now().to_msg()

            for ns in sorted(self.joint_states.keys()):
                arm_state = self.joint_states[ns]
                num_joints = len(arm_state.name)

                combined.name.extend([f"{ns}_{joint}" for joint in arm_state.name])
                combined.position.extend(arm_state.position)
                combined.velocity.extend(
                    arm_state.velocity if arm_state.velocity else [0.0] * num_joints
                )
                combined.effort.extend(
                    arm_state.effort if arm_state.effort else [0.0] * num_joints
                )

            if combined.name:
                self.joint_state_pub.publish(combined)

    def _setup_trajectory_proxy(self):
        """Setup action servers and clients for trajectory forwarding."""
        for ns in self.namespaces:
            # Proxy server (accepts prefixed trajectories from MoveIt)
            self.traj_servers[ns] = ActionServer(
                self,
                FollowJointTrajectory,
                f"/{ns}/follow_joint_trajectory_prefixed",
                execute_callback=lambda gh, namespace=ns: self._execute_trajectory(
                    gh, namespace
                ),
                goal_callback=lambda _: GoalResponse.ACCEPT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self.action_callback_group,
            )

            # Hardware client (sends unprefixed trajectories)
            self.hw_clients[ns] = ActionClient(
                self,
                FollowJointTrajectory,
                f"/{ns}/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory",
                callback_group=self.action_callback_group,
            )

    def _strip_prefix(
        self, trajectory: JointTrajectory, namespace: str
    ) -> JointTrajectory:
        """Remove namespace prefix from joint names."""
        unprefixed = JointTrajectory()
        unprefixed.header = trajectory.header
        unprefixed.points = trajectory.points

        prefix = f"{namespace}_"
        unprefixed.joint_names = [
            name[len(prefix) :] if name.startswith(prefix) else name
            for name in trajectory.joint_names
        ]

        return unprefixed

    def _execute_trajectory(self, goal_handle, namespace: str):
        """
        Forward trajectory to hardware with unprefixed joint names.
        SYNCHRONOUS version that works reliably in ROS 2 Humble.
        """
        try:
            self.get_logger().info(f"[{namespace}] Executing trajectory")

            # Unprefix joint names
            hw_trajectory = self._strip_prefix(
                goal_handle.request.trajectory, namespace
            )
            hw_client = self.hw_clients[namespace]

            # Wait for hardware server
            if not hw_client.wait_for_server(timeout_sec=self.srv_timeout):
                self.get_logger().error(f"[{namespace}] Hardware server not available")
                goal_handle.abort()
                return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

            # Send goal with feedback forwarding
            hw_goal = FollowJointTrajectory.Goal(trajectory=hw_trajectory)

            def feedback_callback(feedback):
                """Forward hardware feedback to MoveIt"""
                goal_handle.publish_feedback(feedback.feedback)

            send_future = hw_client.send_goal_async(
                hw_goal, feedback_callback=feedback_callback
            )

            # Wait for goal acceptance (blocking with timeout)
            event = threading.Event()
            hw_goal_handle = None
            error_msg = None

            def on_goal_response(future):
                nonlocal hw_goal_handle, error_msg
                try:
                    hw_goal_handle = future.result()
                except Exception as e:
                    error_msg = str(e)
                finally:
                    event.set()

            send_future.add_done_callback(on_goal_response)

            if not event.wait(timeout=self.srv_timeout):
                self.get_logger().error(f"[{namespace}] Goal acceptance timeout")
                goal_handle.abort()
                return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

            if error_msg or not hw_goal_handle:
                self.get_logger().error(
                    f"[{namespace}] Failed to send goal: {error_msg}"
                )
                goal_handle.abort()
                return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

            if not hw_goal_handle.accepted:
                self.get_logger().warn(f"[{namespace}] Goal rejected by hardware")
                goal_handle.abort()
                return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

            self.get_logger().info(
                f"[{namespace}] Goal accepted, waiting for result..."
            )

            # Wait for execution result
            result_future = hw_goal_handle.get_result_async()
            result_event = threading.Event()
            hw_result = None
            result_error = None

            def on_result(future):
                nonlocal hw_result, result_error
                try:
                    hw_result = future.result().result
                except Exception as e:
                    result_error = str(e)
                finally:
                    result_event.set()

            result_future.add_done_callback(on_result)

            # Wait with cancellation support
            start_time = self.get_clock().now()
            while not result_event.is_set():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f"[{namespace}] Cancelling trajectory")
                    hw_goal_handle.cancel_goal_async()
                    result_event.wait(timeout=1.0)  # Wait briefly for cancellation
                    goal_handle.canceled()
                    return self._create_result(
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )

                # Check for timeout
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > self.traj_timeout:
                    self.get_logger().error(
                        f"[{namespace}] Execution timeout ({elapsed:.1f}s)"
                    )
                    hw_goal_handle.cancel_goal_async()
                    goal_handle.abort()
                    return self._create_result(
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )

                # Small sleep to yield to other threads
                result_event.wait(timeout=0.01)

            # Check result
            if result_error or not hw_result:
                self.get_logger().error(
                    f"[{namespace}] Failed to get result: {result_error}"
                )
                goal_handle.abort()
                return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

            # Set goal state and return
            if hw_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info(
                    f"[{namespace}] Trajectory completed successfully"
                )
                goal_handle.succeed()
            else:
                self.get_logger().warn(
                    f"[{namespace}] Trajectory failed with code {hw_result.error_code}"
                )
                goal_handle.abort()

            return hw_result

        except Exception as e:
            self.get_logger().error(f"[{namespace}] Exception: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            return self._create_result(FollowJointTrajectory.Result.INVALID_GOAL)

    @staticmethod
    def _create_result(error_code: int) -> FollowJointTrajectory.Result:
        """Create a properly formatted result."""
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        return result


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrefixer()

    # Use MultiThreadedExecutor with enough threads for N robots
    # Per robot: 1 joint state sub + 2 action servers (1 in, 1 out)
    # Plus: 1 timer + overhead buffer
    num_robots = len(node.namespaces)
    num_threads = max(8, 2 + num_robots * 2)
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
