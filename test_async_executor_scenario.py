#!/usr/bin/env python3
"""
Test Script for Async Executor - ARM_1 TWICE + ARM_2 ONCE

This script tests the async executor with the following scenario:
1. Move both arms to home position [0,0,0,0,0,0]
2. ARM_1: Execute 2 iterations
   - Iteration 1: Joint 1 from 0° to +50° (2 seconds)
   - Iteration 2: Joint 1 from +50° to -50° (2 seconds)
3. ARM_2: Execute 1 iteration
   - Iteration 1: Joint 1 from 0° to +50° (2 seconds)

Expected behavior:
- ARM_1 iter1 and ARM_2 iter1 execute SIMULTANEOUSLY (async)
- ARM_1 iter2 executes after iter1 completes
- Total time: ~4 seconds (async) vs ~6 seconds (sequential)

Usage:
  Terminal 1: ros2 launch niryo_ned2_dual_arm_moveit_config sim_dualarm.launch.py
  Terminal 2: python3 test_async_executor_scenario.py
  Watch in RViz for visual confirmation!
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * math.pi / 180.0


class AsyncExecutorTester(Node):
    """Test harness for async executor - ARM_1 TWICE + ARM_2 ONCE scenario"""

    def __init__(self):
        super().__init__("async_executor_tester")

        self.get_logger().info("=" * 70)
        self.get_logger().info("ASYNC EXECUTOR TEST: ARM_1 TWICE + ARM_2 ONCE")
        self.get_logger().info("=" * 70)

        # Callback group for parallel requests
        callback_group = ReentrantCallbackGroup()

        # Action clients for both arms
        self.arm_1_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_1/follow_joint_trajectory",
            callback_group=callback_group,
        )

        self.arm_2_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_2/follow_joint_trajectory",
            callback_group=callback_group,
        )

        self.get_logger().info("Waiting for action servers...")

        # Wait for servers
        if not self.arm_1_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("ARM_1 action server not available!")
            raise RuntimeError("ARM_1 action server timeout")

        if not self.arm_2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("ARM_2 action server not available!")
            raise RuntimeError("ARM_2 action server timeout")

        self.get_logger().info("✓ Both action servers ready")

    def create_single_joint_trajectory(
        self,
        arm_id: str,
        joint_index: int,
        start_angle_deg: float,
        end_angle_deg: float,
        duration_sec: float = 2.0,
    ) -> JointTrajectory:
        """
        Create a trajectory where only one joint moves.

        Args:
            arm_id: "arm_1" or "arm_2"
            joint_index: 0-5 (which joint to move, 0 = joint_1)
            start_angle_deg: Starting angle in degrees
            end_angle_deg: Ending angle in degrees
            duration_sec: Trajectory duration in seconds

        Returns:
            JointTrajectory message
        """
        traj = JointTrajectory()

        # Joint names for 6-DOF arm
        traj.joint_names = [
            f"{arm_id}_joint_1",
            f"{arm_id}_joint_2",
            f"{arm_id}_joint_3",
            f"{arm_id}_joint_4",
            f"{arm_id}_joint_5",
            f"{arm_id}_joint_6",
        ]

        # Convert degrees to radians
        start_rad = degrees_to_radians(start_angle_deg)
        end_rad = degrees_to_radians(end_angle_deg)

        # Create 3 trajectory points for smooth motion
        num_points = 3
        for i in range(num_points):
            point = JointTrajectoryPoint()

            # Interpolation coefficient (0.0 to 1.0)
            alpha = i / (num_points - 1)

            # Calculate positions (only the specified joint moves)
            positions = [0.0] * 6  # All joints start at 0
            positions[joint_index] = start_rad + (end_rad - start_rad) * alpha

            point.positions = positions

            # Set time
            time_sec = duration_sec * alpha
            point.time_from_start = Duration(
                sec=int(time_sec), nanosec=int((time_sec - int(time_sec)) * 1e9)
            )

            traj.points.append(point)

        return traj

    def create_home_trajectory(
        self, arm_id: str, duration_sec: float = 3.0
    ) -> JointTrajectory:
        """
        Create trajectory to move arm to home position [0,0,0,0,0,0].

        Args:
            arm_id: "arm_1" or "arm_2"
            duration_sec: Time to reach home

        Returns:
            JointTrajectory message
        """
        traj = JointTrajectory()

        traj.joint_names = [
            f"{arm_id}_joint_1",
            f"{arm_id}_joint_2",
            f"{arm_id}_joint_3",
            f"{arm_id}_joint_4",
            f"{arm_id}_joint_5",
            f"{arm_id}_joint_6",
        ]

        # Single point: move to all zeros
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec - int(duration_sec)) * 1e9)
        )

        traj.points.append(point)

        return traj

    def send_trajectory_and_wait(self, client, trajectory, timeout_sec=10.0):
        """
        Send trajectory and wait for completion.

        Args:
            client: ActionClient instance
            trajectory: JointTrajectory to execute
            timeout_sec: Maximum time to wait
        """
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)

        # Send goal
        send_future = client.send_goal_async(goal)

        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)

        if not send_future.done():
            self.get_logger().error("Failed to send goal - timeout")
            return

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

    def send_trajectory_async(self, client, trajectory):
        """
        Send trajectory without waiting (async).

        Args:
            client: ActionClient instance
            trajectory: JointTrajectory to execute
        """
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        client.send_goal_async(goal)

    def test_arm1_twice_arm2_once(self):
        """
        Main test: ARM_1 executes 2 iterations, ARM_2 executes 1 iteration.

        Expected timeline:
        t=0-2s:   ARM_1 iter1 + ARM_2 execute in parallel (ASYNC)
        t=2-4s:   ARM_1 iter2 executes
        Total:    ~4 seconds (async) vs ~6 seconds (sequential)
        """
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("STARTING TEST")
        self.get_logger().info("=" * 70)

        # PHASE 1: Move to home position
        self.get_logger().info("\n[PHASE 1] Moving both arms to home position...")
        self.get_logger().info("  ARM_1: [0, 0, 0, 0, 0, 0]")
        self.get_logger().info("  ARM_2: [0, 0, 0, 0, 0, 0]")

        home_arm_1 = self.create_home_trajectory("arm_1", duration_sec=3.0)
        home_arm_2 = self.create_home_trajectory("arm_2", duration_sec=3.0)

        self.send_trajectory_and_wait(self.arm_1_client, home_arm_1)
        self.send_trajectory_and_wait(self.arm_2_client, home_arm_2)

        self.get_logger().info("✓ Both arms at home position")
        time.sleep(1.0)  # Pause before next phase

        # PHASE 2: Async execution test
        self.get_logger().info("\n[PHASE 2] Testing async execution...")
        self.get_logger().info(
            "  ARM_1 Iteration 1: Joint 1 from 0° to +50° (2 seconds)"
        )
        self.get_logger().info(
            "  ARM_2 Iteration 1: Joint 1 from 0° to +50° (2 seconds)"
        )
        self.get_logger().info(
            "  ARM_1 Iteration 2: Joint 1 from +50° to -50° (2 seconds)"
        )
        self.get_logger().info("")
        self.get_logger().info("Expected: ARM_1_iter1 and ARM_2 execute SIMULTANEOUSLY")
        self.get_logger().info("")

        # Create trajectories
        arm_1_iter1 = self.create_single_joint_trajectory(
            arm_id="arm_1",
            joint_index=0,  # Joint 1
            start_angle_deg=0.0,
            end_angle_deg=50.0,
            duration_sec=2.0,
        )

        arm_2_iter1 = self.create_single_joint_trajectory(
            arm_id="arm_2",
            joint_index=0,  # Joint 1
            start_angle_deg=0.0,
            end_angle_deg=50.0,
            duration_sec=2.0,
        )

        arm_1_iter2 = self.create_single_joint_trajectory(
            arm_id="arm_1",
            joint_index=0,  # Joint 1
            start_angle_deg=50.0,
            end_angle_deg=-50.0,
            duration_sec=2.0,
        )

        # Send trajectories and measure timing
        start_time = time.time()

        self.get_logger().info("[t=0.00s] Sending ARM_1 iteration 1...")
        self.send_trajectory_async(self.arm_1_client, arm_1_iter1)

        time.sleep(0.05)

        self.get_logger().info("[t=0.05s] Sending ARM_2 iteration 1...")
        self.send_trajectory_async(self.arm_2_client, arm_2_iter1)

        time.sleep(0.05)

        self.get_logger().info("[t=0.10s] Sending ARM_1 iteration 2...")
        self.send_trajectory_async(self.arm_1_client, arm_1_iter2)

        self.get_logger().info("\nWaiting for execution to complete...")
        self.get_logger().info("Watch RViz to see parallel motion!")

        # Wait for all trajectories to complete
        time.sleep(5.0)

        total_time = time.time() - start_time

        # Print results
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("TEST RESULTS")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Total execution time: {total_time:.2f} seconds")
        self.get_logger().info("")
        self.get_logger().info("Expected timing (if ASYNC WORKING):")
        self.get_logger().info("  t=0.0-2.0s: ARM_1_iter1 + ARM_2 execute TOGETHER")
        self.get_logger().info("  t=2.0-4.0s: ARM_1_iter2 executes")
        self.get_logger().info("  Total:      ~4 seconds")
        self.get_logger().info("")
        self.get_logger().info("Expected timing (if SEQUENTIAL - BAD):")
        self.get_logger().info("  t=0.0-2.0s: ARM_1_iter1 executes ALONE")
        self.get_logger().info("  t=2.0-4.0s: ARM_2_iter1 executes ALONE")
        self.get_logger().info("  t=4.0-6.0s: ARM_1_iter2 executes ALONE")
        self.get_logger().info("  Total:      ~6 seconds")
        self.get_logger().info("")

        # Verdict
        if total_time < 4.5:
            self.get_logger().info("✓✓✓ VERDICT: PASS - ASYNC EXECUTION WORKING! ✓✓✓")
        else:
            self.get_logger().info(
                "✗✗✗ VERDICT: FAIL - Sequential execution detected ✗✗✗"
            )

        self.get_logger().info("=" * 70)
        self.get_logger().info("\nVisual confirmation:")
        self.get_logger().info("  - In RViz, you should have seen:")
        self.get_logger().info(
            "    1. Both arms swing joint_1 to +50° AT THE SAME TIME"
        )
        self.get_logger().info("    2. ARM_1 swings back to -50° (while ARM_2 stays)")
        self.get_logger().info("=" * 70)


def main(args=None):
    rclpy.init(args=args)

    try:
        tester = AsyncExecutorTester()
        tester.test_arm1_twice_arm2_once()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        try:
            tester.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
