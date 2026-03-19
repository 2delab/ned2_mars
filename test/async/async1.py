#!/usr/bin/env python3

import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance with config file
moveit_py = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_file])

# Get planning components for both arms
arm_1 = moveit_py.get_planning_component("arm_1")
arm_2 = moveit_py.get_planning_component("arm_2")

# Set start state to current state
arm_1.set_start_state_to_current_state()
arm_2.set_start_state_to_current_state()

# Create pose goal
pose_goal = PoseStamped()
pose_goal.pose.position.x = 0.4
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.2

# X-axis facing DOWN (pointing toward ground)
qx = 0.0
qy = 0.7071
qz = 0.0
qw = 0.7071

pose_goal.pose.orientation.x = qx
pose_goal.pose.orientation.y = qy
pose_goal.pose.orientation.z = qz
pose_goal.pose.orientation.w = qw

# Set goal state for arm_1
pose_goal_1 = PoseStamped()
pose_goal_1.header.frame_id = "arm_1_base_link"
pose_goal_1.pose = pose_goal.pose
arm_1.set_goal_state(pose_stamped_msg=pose_goal_1, pose_link="arm_1_tool_link")

# Set goal state for arm_2
pose_goal_2 = PoseStamped()
pose_goal_2.header.frame_id = "arm_2_base_link"
pose_goal_2.pose = pose_goal.pose
arm_2.set_goal_state(pose_stamped_msg=pose_goal_2, pose_link="arm_2_tool_link")

# Plan for both arms
plan_result_1 = arm_1.plan()
plan_result_2 = arm_2.plan()

# Execute
if plan_result_1 and plan_result_2:
    print("Both arms planned successfully")
    print("→ Executing arm_1...")
    moveit_py.execute(plan_result_1.trajectory, controllers=[])
    print("✓ Successfully moved arm_1 to target pose")

    print("→ Executing arm_2...")
    moveit_py.execute(plan_result_2.trajectory, controllers=[])
    print("✓ Successfully moved arm_2 to target pose")
else:
    print("Planning failed")

rclpy.shutdown()
