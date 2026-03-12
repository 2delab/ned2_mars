#!/usr/bin/env python3
"""
MoveIt2 Proper Trajectory Execution with OMPL Planner (Option A: MoveAction)

This demonstrates the CORRECT way to use OMPL planner with full safety:
- Uses move_group's MoveAction (combines planning + execution)
- Includes collision checking and safety constraints
- Automatically applies AddTimeOptimalParameterization timing
- Works with sim_mock_ompl.launch.py configuration

Key Advantage over test1.py:
- Full collision checking during planning
- Uses MoveIt's safety framework
- Proper trajectory timing via planning pipeline
- No manual workarounds needed

Requirements:
  ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py use_rviz:=false

Then run:
  python3 test2.py
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians, degrees


def main():
    rclpy.init()

    # Create node and action client
    node = rclpy.create_node("moveit_test_ompl")
    action_client = ActionClient(node, MoveGroup, "/move_action")

    print("\n" + "=" * 70)
    print("MOVEIT2 WITH OMPL PLANNER - PROPER APPROACH (MoveAction)")
    print("=" * 70)

    print("\nWaiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        print("\nMake sure sim_mock_ompl.launch.py is running:")
        print(
            "  ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py use_rviz:=false"
        )
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server")

    # Create goal request
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "arm"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Set joint goal: joint_1 = 50 degrees
    constraint = JointConstraint()
    constraint.joint_name = "joint_1"
    constraint.position = radians(50)
    constraint.tolerance_above = radians(5)
    constraint.tolerance_below = radians(5)
    constraint.weight = 1.0

    goal.request.goal_constraints.append(Constraints(joint_constraints=[constraint]))

    print("\n" + "=" * 70)
    print("MOTION PLANNING REQUEST")
    print("=" * 70)
    print(f"\nPlanning parameters:")
    print(f"  Group: {goal.request.group_name}")
    print(f"  Planning pipeline: {goal.request.pipeline_id}")
    print(f"  Planner algorithm: {goal.request.planner_id}")
    print(f"  Max velocity scaling: {goal.request.max_velocity_scaling_factor}")
    print(f"  Planning attempts: {goal.request.num_planning_attempts}")
    print(f"  Planning time limit: {goal.request.allowed_planning_time}s")

    print(f"\nGoal state:")
    print(f"  joint_1 = 50°")
    print(f"  Tolerance: ±5°")

    print("\n" + "=" * 70)
    print("EXECUTING MOTION PLANNING")
    print("=" * 70)
    print("\nThis action will:")
    print("  1. Plan trajectory with OMPL algorithm")
    print("  2. Check for collisions during planning")
    print("  3. Apply AddTimeOptimalParameterization for trajectory timing")
    print("  4. Execute trajectory on arm_controller")
    print()

    # Send goal
    future = action_client.send_goal_async(goal)

    # Wait for goal to be accepted
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None:
        print("✗ Goal failed to send")
        rclpy.shutdown()
        return False

    if not goal_handle.accepted:
        print("✗ Goal rejected by move_group")
        rclpy.shutdown()
        return False

    print("✓ Goal accepted - planning and executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None:
        print("✗ No result returned from move_group")
        rclpy.shutdown()
        return False

    # Extract the actual result from the response
    result = result_response.result

    if result is None:
        print("✗ No result data in response")
        rclpy.shutdown()
        return False

    # Check result
    error_code = result.error_code.val

    print("=" * 70)
    if error_code == 1:  # SUCCESS
        print("✓ SUCCESS!")
        print("=" * 70)
        print("\nMotion planning and execution completed successfully!")
        print("\nWhat happened:")
        print("  1. OMPL planner generated collision-free trajectory")
        print("  2. AddTimeOptimalParameterization added trajectory timing")
        print("  3. Trajectory was executed on arm_controller")
        print("  4. Robot moved from 0° to 50° on joint_1")
        print("\nThis is the proper MoveIt2 workflow:")
        print("  [Planning with collision checking] → [Parameterization] → [Execution]")
        print("\nAdvantages over direct action client (test1.py):")
        print("  ✓ Full collision checking during planning")
        print("  ✓ Automatic trajectory timing via planning pipeline")
        print("  ✓ Complete safety framework")
        print("  ✓ Uses configured OMPL planner settings")
        print("=" * 70)
        rclpy.shutdown()
        return True
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            1: "SUCCESS",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            4: "START_STATE_INVALID",
            5: "GOAL_IN_COLLISION",
            6: "GOAL_INVALID",
            7: "GOAL_CONSTRAINTS_VIOLATED",
        }

        error_name = error_messages.get(error_code, f"UNKNOWN ({error_code})")
        print(f"✗ Motion planning failed!")
        print("=" * 70)
        print(f"\nError: {error_name}")
        print(f"Error code: {error_code}")

        print("\nTroubleshooting:")
        print("  1. Verify sim_mock_ompl.launch.py is running:")
        print(
            "       ros2 launch niryo_ned2_mock_moveit_config sim_mock_ompl.launch.py use_rviz:=false"
        )
        print("  2. Check that move_group is using OMPL planner:")
        print("       ros2 topic echo /move_group/status")
        print("  3. Check for collision issues:")
        print("       ros2 service call /planning_scene 'std_srvs/Empty'")
        print("=" * 70)
        rclpy.shutdown()
        return False


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
