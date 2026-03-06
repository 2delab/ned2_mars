#!/usr/bin/env python3
"""
Marker Calibration Script

Standalone script to calibrate the world reference frame using marker 40.
Prompts user to position TCP at marker 40, then saves the TCP pose as the world reference.

Usage:
    python3 calibrate_marker.py
    OR
    ros2 run niryo_ned_vision marker_calibration
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from niryo_ned_ros2_interfaces.msg import RobotState
import os
from pathlib import Path
import time


class MarkerCalibrationNode(Node):
    """Node for calibrating the marker world reference frame."""

    def __init__(self):
        super().__init__("marker_calibration")

        # Subscribe to robot state topic
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            "/niryo_robot/robot_state",
            self.robot_state_callback,
            qos_profile=QoSProfile(depth=10),
        )

        self.current_robot_state = None
        self.reference_file = Path("/tmp/marker_reference.json")

        self.get_logger().info("MarkerCalibrationNode initialized")

    def robot_state_callback(self, msg):
        """Callback to store current robot state."""
        self.current_robot_state = msg

    def calibrate(self):
        """Calibrate the marker reference position."""
        self.get_logger().info(
            "=== Marker 40 Calibration ===\n"
            "Position the robot TCP at marker 40 location.\n"
            "Press Enter when ready to save the reference pose..."
        )

        # Wait for user input
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            self.get_logger().info("Calibration cancelled")
            return False

        # Query current TCP pose via FK service
        self.get_logger().info("Querying current TCP pose...")
        tcp_pose = self.get_tcp_pose()

        if tcp_pose is None:
            self.get_logger().error("Failed to get TCP pose")
            return False

        # Save to file
        if self.save_reference(tcp_pose):
            self.get_logger().info(
                f"Calibration successful!\n"
                f"Reference pose saved to {self.reference_file}\n"
                f"Position: x={tcp_pose['x']:.3f}, y={tcp_pose['y']:.3f}, z={tcp_pose['z']:.3f}\n"
                f"Orientation: roll={tcp_pose['roll']:.3f}, pitch={tcp_pose['pitch']:.3f}, yaw={tcp_pose['yaw']:.3f}"
            )
            return True
        else:
            self.get_logger().error("Failed to save reference pose")
            return False

    def get_tcp_pose(self):
        """Get current TCP pose from robot state topic."""
        try:
            if self.current_robot_state is None:
                self.get_logger().error("No robot state received yet. Waiting...")
                # Give it time to receive a message
                for i in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.current_robot_state is not None:
                        break

                if self.current_robot_state is None:
                    self.get_logger().error("Still waiting for robot state...")
                    return None

            # Extract TCP pose from RobotState
            pose = self.current_robot_state

            return {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "roll": float(pose.rpy.roll),
                "pitch": float(pose.rpy.pitch),
                "yaw": float(pose.rpy.yaw),
            }

        except Exception as e:
            self.get_logger().error(f"Error getting TCP pose: {e}")
            return None

            response = future.result()

            if response.status != 0:
                self.get_logger().error(
                    f"FK service returned status {response.status}: {response.message}"
                )
                return None

            # Extract TCP pose from RobotState
            pose = response.pose

            return {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "roll": float(pose.rpy.roll),
                "pitch": float(pose.rpy.pitch),
                "yaw": float(pose.rpy.yaw),
            }

        except Exception as e:
            self.get_logger().error(f"Error getting TCP pose: {e}")
            return None

    def save_reference(self, tcp_pose):
        """Save the reference pose to a JSON file."""
        try:
            # Ensure /tmp directory exists
            os.makedirs("/tmp", exist_ok=True)

            # Save to JSON file
            with open(self.reference_file, "w") as f:
                json.dump(tcp_pose, f, indent=2)

            return True
        except Exception as e:
            self.get_logger().error(f"Error saving reference: {e}")
            return False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MarkerCalibrationNode()

    try:
        # Run calibration
        success = node.calibrate()

        # Shutdown ROS
        rclpy.shutdown()

        # Exit with status
        exit(0 if success else 1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        rclpy.shutdown()
        exit(1)
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
        rclpy.shutdown()
        exit(1)


if __name__ == "__main__":
    main()
