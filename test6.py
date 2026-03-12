#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Multi-Phase Control Sequence
Phase 1: Move all joints to 0°
Phase 2: Move joint_1 to 50° and joint_2 to -35° (lock others at 0°)
Phase 3: Move joint_1 to -50° while keeping joint_2 at -35° (lock others at 0°)
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians


def send_phase_goal(
    action_client, node, phase_num, phase_description, target_positions
):
    """
    Send and execute a phase goal.

    Args:
        action_client: MoveGroup action client
        node: ROS node
        phase_num: Phase number (1, 2, 3)
        phase_description: Description of phase
        target_positions: Dict of {joint_name: target_angle_deg}

    Returns:
        True if successful, False otherwise
    """

    print(f"\n{'=' * 70}")
    print(f"PHASE {phase_num}: {phase_description}")
    print(f"{'=' * 70}")

    # Create goal request
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "dual"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraints
    constraints_list = []

    # Process all 12 joints (6 per arm)
    for arm_prefix in ["arm_1_", "arm_2_"]:
        for i in range(1, 7):
            joint_name = f"{arm_prefix}joint_{i}"
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.weight = 1.0

            if joint_name in target_positions:
                # This joint is part of the target
                angle_deg = target_positions[joint_name]
                constraint.position = radians(angle_deg)
                constraint.tolerance_above = radians(5)
                constraint.tolerance_below = radians(5)
            else:
                # This joint should stay locked
                constraint.position = 0.0
                constraint.tolerance_above = radians(1)  # Very tight
                constraint.tolerance_below = radians(1)  # Lock in place

            constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    # Print phase details
    print(f"Target positions:")
    for joint, angle in sorted(target_positions.items()):
        print(f"  {joint} → {angle}°")

    locked_joints = []
    for arm_prefix in ["arm_1_", "arm_2_"]:
        for i in range(1, 7):
            joint_name = f"{arm_prefix}joint_{i}"
            if joint_name not in target_positions:
                locked_joints.append(joint_name)

    if locked_joints:
        print(f"Locked joints: {locked_joints}")
    print()

    # Send goal
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None or not goal_handle.accepted:
        print(f"✗ Phase {phase_num} goal rejected by move_group")
        return False

    print(f"✓ Phase {phase_num} goal accepted - executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        print(f"✗ No result returned for Phase {phase_num}")
        return False

    error_code = result_response.result.error_code.val

    if error_code == 1:  # SUCCESS
        print(f"✓ Phase {phase_num} SUCCESS!")
        return True
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            5: "GOAL_IN_COLLISION",
        }
        print(
            f"✗ Phase {phase_num} Failed: {error_messages.get(error_code, f'ERROR {error_code}')}"
        )
        return False


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_phases")
    action_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server")

    print("\n" + "=" * 70)
    print("DUAL-ARM 3-PHASE MOTION SEQUENCE")
    print("=" * 70)
    print("Phase 1: Home position (all joints to 0°)")
    print("Phase 2: Move joint_1 to 50° and joint_2 to -35°")
    print("Phase 3: Move joint_1 to -50° (keep joint_2 at -35°)")

    # Phase 1: Move all joints to 0
    phase1_targets = {
        "arm_1_joint_1": 0,
        "arm_1_joint_2": 0,
        "arm_1_joint_3": 0,
        "arm_1_joint_4": 0,
        "arm_1_joint_5": 0,
        "arm_1_joint_6": 0,
        "arm_2_joint_1": 0,
        "arm_2_joint_2": 0,
        "arm_2_joint_3": 0,
        "arm_2_joint_4": 0,
        "arm_2_joint_5": 0,
        "arm_2_joint_6": 0,
    }
    result1 = send_phase_goal(
        action_client, node, 1, "Home Position (All joints to 0°)", phase1_targets
    )

    if not result1:
        print("\n✗ Phase 1 failed - stopping")
        rclpy.shutdown()
        return False

    # Phase 2: Move joint_1 to 50° and joint_2 to -35°
    phase2_targets = {
        "arm_1_joint_1": 50,
        "arm_1_joint_2": -35,
        "arm_2_joint_1": 50,
        "arm_2_joint_2": -35,
    }
    result2 = send_phase_goal(
        action_client,
        node,
        2,
        "Move joint_1 to 50° and joint_2 to -35°",
        phase2_targets,
    )

    if not result2:
        print("\n✗ Phase 2 failed - stopping")
        rclpy.shutdown()
        return False

    # Phase 3: Move joint_1 to -50° (keep joint_2 at -35°)
    phase3_targets = {
        "arm_1_joint_1": -50,
        "arm_1_joint_2": -35,
        "arm_2_joint_1": -50,
        "arm_2_joint_2": -35,
    }
    result3 = send_phase_goal(
        action_client,
        node,
        3,
        "Move joint_1 to -50° (keep joint_2 at -35°)",
        phase3_targets,
    )

    # Summary
    print("\n" + "=" * 70)
    print("SEQUENCE SUMMARY")
    print("=" * 70)
    if result1 and result2 and result3:
        print("✓ ALL PHASES COMPLETED SUCCESSFULLY!")
        success = True
    else:
        print("✗ SEQUENCE FAILED")
        print(f"  Phase 1: {'✓' if result1 else '✗'}")
        print(f"  Phase 2: {'✓' if result2 else '✗'}")
        print(f"  Phase 3: {'✓' if result3 else '✗'}")
        success = False

    rclpy.shutdown()
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
