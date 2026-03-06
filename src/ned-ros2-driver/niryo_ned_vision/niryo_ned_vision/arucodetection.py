#!/usr/bin/env python3
"""
ArUco Marker Detection and Pose Publishing Node with Camera Integration

Captures video from camera, detects ArUco markers and publishes their poses as TF transforms.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from pathlib import Path
import math
from cv_bridge import CvBridge
import threading
import time


class ArucoPosePublisher(Node):
    """Node for capturing camera feed, detecting ArUco markers and publishing their poses."""

    def __init__(self):
        super().__init__("aruco_pose_publisher")

        # Load calibration data
        self._load_calibration()

        # Initialize ArUco detector
        self._init_detector()

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Bridge for ROS Image messages
        self.cv_bridge = CvBridge()

        # Declare parameters
        self.declare_parameter("camera_id", 6)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("marker_size", 0.03)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("publish_image", False)

        # Get parameters
        self.camera_id = self.get_parameter("camera_id").value
        self.camera_fps = self.get_parameter("camera_fps").value
        self.marker_size = self.get_parameter("marker_size").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        self.enable_viz = self.get_parameter("enable_visualization").value
        self.publish_image = self.get_parameter("publish_image").value

        # Initialize camera
        self.cap = None
        self._init_camera()

        # Image publisher (optional)
        if self.publish_image:
            self.image_publisher = self.create_publisher(
                Image, "/aruco/image_marked", qos_profile=QoSProfile(depth=1)
            )

        # Control flags
        self.running = True
        self.processing_lock = threading.Lock()

        # Create timer for camera capture at specified rate
        self.frame_count = 0
        self.start_time = time.time()

        timer_period = 1.0 / self.camera_fps
        self.timer = self.create_timer(timer_period, self.camera_timer_callback)

        self.get_logger().info(
            f"ArucoPosePublisher initialized. "
            f"Camera: {self.camera_id}, FPS: {self.camera_fps}, "
            f"Marker size: {self.marker_size}m"
        )

    def _load_calibration(self) -> None:
        """Load camera calibration from file."""
        try:
            calib_file = Path(__file__).parent / "camera_calibration.npz"
            if not calib_file.exists():
                self.get_logger().warn(
                    f"Calibration file not found: {calib_file}. "
                    "Using identity matrix as fallback."
                )
                # Fallback calibration (identity matrix)
                self.camera_matrix = np.array(
                    [[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32
                )
                self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
            else:
                calib_data = np.load(calib_file)
                self.camera_matrix = calib_data["camera_matrix"]
                self.dist_coeffs = calib_data["dist_coeffs"]
                self.get_logger().info("Camera calibration loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise

    def _init_detector(self) -> None:
        """Initialize ArUco detector."""
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.get_logger().info("ArUco detector initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize detector: {e}")
            raise

    def _init_camera(self) -> None:
        """Initialize camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_id}")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            self.cap.set(
                cv2.CAP_PROP_BUFFERSIZE, 1
            )  # Reduce buffer size for lower latency

            # Get actual properties
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(
                f"Camera initialized successfully. "
                f"Resolution: {actual_width}x{actual_height}, FPS: {actual_fps}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            raise

    def camera_timer_callback(self) -> None:
        """Capture frame from camera and process it."""
        if not self.running or self.cap is None:
            return

        try:
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("Failed to read frame from camera")
                return

            self.frame_count += 1
            frame = cv2.flip(
                frame, -1
            )  # Flip both horizontally and vertically (180 degrees)

            # Create ROS Image message with current timestamp
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.frame_id = "camera"

            # Process frame (detect markers and publish transforms)
            marked_frame = self._process_frame(frame, ros_image)

            # Publish marked image if enabled
            if self.publish_image and marked_frame is not None:
                ros_marked = self.cv_bridge.cv2_to_imgmsg(marked_frame, encoding="bgr8")
                ros_marked.header = ros_image.header
                self.image_publisher.publish(ros_marked)

            # Display visualization if enabled
            if self.enable_viz:
                self._display_frame(marked_frame if marked_frame is not None else frame)

        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {e}")

    def _process_frame(self, frame: np.ndarray, ros_msg: Image) -> np.ndarray:
        """
        Process frame to detect markers and publish transforms.

        Args:
            frame: OpenCV image frame
            ros_msg: ROS Image message

        Returns:
            Marked frame for visualization
        """
        with self.processing_lock:
            try:
                marked_frame = frame.copy()

                # Convert to grayscale for detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect markers
                corners, ids, rejected = self.detector.detectMarkers(gray)

                # Draw detected markers
                marked_frame = cv2.aruco.drawDetectedMarkers(marked_frame, corners, ids)

                # Process detected markers
                if ids is not None and len(ids) > 0:
                    self._process_markers(marked_frame, ros_msg, corners, ids)

                # Draw FPS
                self._draw_fps(marked_frame)

                return marked_frame

            except Exception as e:
                self.get_logger().error(f"Error processing frame: {e}")
                return frame

    def _process_markers(
        self, frame: np.ndarray, ros_msg: Image, corners: list, ids: np.ndarray
    ) -> None:
        """
        Process detected ArUco markers and publish transforms.

        Args:
            frame: OpenCV image frame
            ros_msg: ROS Image message
            corners: Detected marker corners
            ids: Detected marker IDs
        """
        try:
            # Estimate poses
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Calculate distance
                distance = np.linalg.norm(tvec)

                # Publish TF transform
                self._publish_transform(ros_msg, marker_id, rvec, tvec)

                # Draw visualization
                if self.enable_viz:
                    self._draw_marker_axes(frame, rvec, tvec)
                    self._draw_marker_info(frame, marker_id, distance, i)

        except Exception as e:
            self.get_logger().error(f"Error processing markers: {e}")

    def _publish_transform(
        self, ros_msg: Image, marker_id: int, rvec: np.ndarray, tvec: np.ndarray
    ) -> None:
        """
        Publish marker pose as TF transform.

        Args:
            ros_msg: ROS Image message (for timestamp)
            marker_id: ID of the marker
            rvec: Rotation vector
            tvec: Translation vector
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "mono_camera"
        transform.child_frame_id = f"mono_aruco_marker_{int(marker_id)}"

        # Set translation
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])

        # Convert rotation vector to quaternion
        quat = self._rvec_to_quat(rvec)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    def _draw_marker_axes(
        self, frame: np.ndarray, rvec: np.ndarray, tvec: np.ndarray
    ) -> None:
        """
        Draw coordinate axes on the marker.

        Args:
            frame: OpenCV image frame
            rvec: Rotation vector
            tvec: Translation vector
        """
        try:
            axis_length = 0.05
            axis_pts = np.float32(
                [
                    [0, 0, 0],
                    [axis_length, 0, 0],
                    [0, axis_length, 0],
                    [0, 0, axis_length],
                ]
            )

            img_pts, _ = cv2.projectPoints(
                axis_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            img_pts = np.int32(img_pts).reshape(-1, 2)

            origin = tuple(img_pts[0])

            # Draw axes (X=Red, Y=Green, Z=Blue)
            cv2.line(frame, origin, tuple(img_pts[1]), (0, 0, 255), 3)  # X
            cv2.line(frame, origin, tuple(img_pts[2]), (0, 255, 0), 3)  # Y
            cv2.line(frame, origin, tuple(img_pts[3]), (255, 0, 0), 3)  # Z
        except Exception as e:
            self.get_logger().debug(f"Error drawing axes: {e}")

    def _draw_marker_info(
        self, frame: np.ndarray, marker_id: int, distance: float, marker_index: int
    ) -> None:
        """
        Draw marker information on frame.

        Args:
            frame: OpenCV image frame
            marker_id: ID of the marker
            distance: Distance to marker
            marker_index: Index of marker (for text positioning)
        """
        text = f"ID: {int(marker_id)} | Dist: {distance:.2f}m"
        position = (10, 60 + marker_index * 30)
        cv2.putText(
            frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )

    def _draw_fps(self, frame: np.ndarray) -> None:
        """
        Draw FPS counter on frame.

        Args:
            frame: OpenCV image frame
        """
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frame_count / elapsed
            text = f"FPS: {fps:.1f}"
            cv2.putText(
                frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

    def _display_frame(self, frame: np.ndarray) -> None:
        """
        Display frame with OpenCV.

        Args:
            frame: OpenCV image frame
        """
        cv2.imshow("ArUco Detection", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.running = False

    @staticmethod
    def _rvec_to_quat(rvec: np.ndarray) -> list:
        """
        Convert rotation vector to quaternion.

        Args:
            rvec: Rotation vector (Rodrigues format)

        Returns:
            Quaternion as [x, y, z, w]
        """
        theta = np.linalg.norm(rvec)

        if theta < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]

        axis = rvec / theta
        s = math.sin(theta / 2)
        c = math.cos(theta / 2)

        return [axis[0] * s, axis[1] * s, axis[2] * s, c]

    def destroy_node(self):
        """Cleanup resources."""
        self.running = False

        if self.cap is not None:
            self.cap.release()
            self.get_logger().info("Camera released")

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ArucoPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
