#!/usr/bin/env python3
"""
Marker Translation Node

Subscribes to detected ArUco markers from the camera, calculates their positions
relative to the world reference frame (marker 40), and publishes target TCP poses.

The node:
1. Loads the reference pose from /tmp/marker_reference.json (calibrated position)
2. Subscribes to TF transforms for all detected markers
3. For each detected marker (except marker 40):
   - Calculates offset from marker 40
   - Converts to absolute world position
   - Publishes as target TCP pose

Usage:
    Launched as part of vision.launch.py
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
from pathlib import Path
from tf2_geometry_msgs import PointStamped
import math


class MarkerTranslationNode(Node):
    """Node for converting detected markers to world TCP poses."""

    def __init__(self):
        super().__init__("marker_translation")

        # Load reference pose from calibration
        self.reference_pose = self._load_reference()
        if self.reference_pose is None:
            self.get_logger().warn(
                "No calibration found. Run calibration first: "
                "ros2 run niryo_ned_vision marker_calibration"
            )

        # TF2 setup for reading transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for target poses
        self.target_poses_publisher = self.create_publisher(
            PoseArray,
            "/marker_translation/target_poses",
            qos_profile=rclpy.qos.QoSProfile(depth=10),
        )

        # Create a timer to periodically check for markers
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.on_timer)

        # Track detected marker IDs for logging
        self.last_detected_markers = set()

        self.get_logger().info("MarkerTranslationNode initialized")
        if self.reference_pose:
            self.get_logger().info(
                f"Reference pose loaded: x={self.reference_pose['x']:.3f}, "
                f"y={self.reference_pose['y']:.3f}, z={self.reference_pose['z']:.3f}"
            )

    def _load_reference(self):
        """Load reference pose from calibration file."""
        try:
            reference_file = Path("/tmp/marker_reference.json")
            if not reference_file.exists():
                self.get_logger().error(f"Reference file not found: {reference_file}")
                return None

            with open(reference_file, "r") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading reference: {e}")
            return None

    def on_timer(self):
        """Periodically check for detected markers and publish target poses."""
        if self.reference_pose is None:
            return

        try:
            # Get all frames in the TF tree
            all_frames = self.tf_buffer.all_frames_as_string()

            # Find all marker frames (mono_aruco_marker_*)
            target_poses = []
            detected_markers = set()

            # Parse frame names to find markers
            # Format: "Frame mono_aruco_marker_XX exists with parent mono_camera."
            for line in all_frames.split("\n"):
                if "mono_aruco_marker_" not in line:
                    continue

                try:
                    # Extract frame name from the line
                    # Expected format: "Frame mono_aruco_marker_XX exists with parent ..."
                    parts = line.split()
                    if len(parts) < 2 or parts[0] != "Frame":
                        continue

                    frame_name = parts[1]

                    # Extract marker ID from frame name (e.g., "mono_aruco_marker_40" -> 40)
                    marker_id = int(frame_name.split("_")[-1])
                    detected_markers.add(marker_id)

                    # Skip the reference marker (marker 40)
                    if marker_id == 40:
                        continue

                    # Get marker pose in camera frame
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            "mono_camera", frame_name, rclpy.time.Time()
                        )

                        # Convert to target TCP pose
                        target_pose = self._calculate_target_pose(marker_id, transform)

                        if target_pose:
                            target_poses.append(target_pose)

                    except tf2_ros.TransformException as e:
                        self.get_logger().debug(
                            f"Transform lookup failed for {frame_name}: {e}"
                        )
                        continue

                except (ValueError, IndexError):
                    # Skip malformed marker frame names
                    continue

            # Check if we have marker 40 (reference)
            if 40 not in detected_markers:
                self.get_logger().debug("Reference marker (40) not detected")
                # Don't publish empty poses
                return

            # Publish target poses if any detected
            if target_poses:
                self._publish_target_poses(target_poses)

                # Log detected markers if changed
                if detected_markers != self.last_detected_markers:
                    marker_ids = sorted([m for m in detected_markers if m != 40])
                    self.get_logger().info(f"Detected markers: {marker_ids}")
                    self.last_detected_markers = detected_markers

        except Exception as e:
            self.get_logger().debug(f"Error in marker processing: {e}")

    def _calculate_target_pose(self, marker_id, marker_transform):
        """
        Calculate target TCP pose from marker position in camera frame.

        Args:
            marker_id: ID of the detected marker
            marker_transform: TransformStamped from camera to marker

        Returns:
            Pose object with the target TCP position, or None if calculation failed
        """
        try:
            # Get marker position in camera frame
            marker_x = marker_transform.transform.translation.x
            marker_y = marker_transform.transform.translation.y
            marker_z = marker_transform.transform.translation.z

            # Get reference marker position in camera frame
            # (We need this to calculate offset)
            try:
                ref_transform = self.tf_buffer.lookup_transform(
                    "mono_camera", "mono_aruco_marker_40", rclpy.time.Time()
                )

                ref_marker_x = ref_transform.transform.translation.x
                ref_marker_y = ref_transform.transform.translation.y
                ref_marker_z = ref_transform.transform.translation.z

            except tf2_ros.TransformException:
                self.get_logger().debug("Could not get reference marker transform")
                return None

            # Calculate offset in camera frame
            offset_x = marker_x - ref_marker_x
            offset_y = marker_y - ref_marker_y
            offset_z = marker_z - ref_marker_z

            # Convert to world frame: world_pos = reference + offset
            target_x = self.reference_pose["x"] + offset_x
            target_y = self.reference_pose["y"] + offset_y
            target_z = self.reference_pose["z"] + offset_z

            # Create Pose object
            pose = Pose()
            pose.position.x = target_x
            pose.position.y = target_y
            pose.position.z = target_z

            # Use reference orientation (all targets have same approach angle)
            # Convert RPY to quaternion
            roll = self.reference_pose["roll"]
            pitch = self.reference_pose["pitch"]
            yaw = self.reference_pose["yaw"]

            qx, qy, qz, qw = self._rpy_to_quaternion(roll, pitch, yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            return pose

        except Exception as e:
            self.get_logger().debug(
                f"Error calculating target pose for marker {marker_id}: {e}"
            )
            return None

    def _rpy_to_quaternion(self, roll, pitch, yaw):
        """
        Convert RPY (roll, pitch, yaw) to quaternion.

        Args:
            roll, pitch, yaw: Rotation angles in radians

        Returns:
            Tuple of (x, y, z, w) quaternion components
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def _publish_target_poses(self, poses):
        """Publish target poses as PoseArray."""
        try:
            pose_array = PoseArray()
            pose_array.header.frame_id = "base_link"
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.poses = poses

            self.target_poses_publisher.publish(pose_array)
        except Exception as e:
            self.get_logger().error(f"Error publishing target poses: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MarkerTranslationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
