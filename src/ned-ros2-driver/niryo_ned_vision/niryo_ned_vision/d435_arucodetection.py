#!/usr/bin/env python3
"""
RealSense D435 ArUco Marker Detection and Pose Publishing Node

Captures video from RealSense D435 camera with on-the-fly calibration loading,
detects ArUco markers and publishes their poses as TF transforms.
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

try:
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False


class D435ArucoPosePublisher(Node):
    """Node for capturing RealSense D435 feed, detecting ArUco markers and publishing their poses."""

    def __init__(self):
        super().__init__("d435_aruco_pose_publisher")

        # Check RealSense availability
        if not REALSENSE_AVAILABLE:
            self.get_logger().error(
                "pyrealsense2 not available. Install with: pip install pyrealsense2"
            )
            raise RuntimeError("pyrealsense2 is required but not installed")

        # Declare parameters FIRST (before using them in _init_realsense)
        self.declare_parameter("camera_serial", "")  # Empty = first available
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("marker_size", 0.03)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("publish_image", False)
        self.declare_parameter(
            "use_depth_stream", False
        )  # Use depth stream in addition to color

        # Get parameters
        self.camera_serial = self.get_parameter("camera_serial").value
        self.camera_fps = self.get_parameter("camera_fps").value
        self.marker_size = self.get_parameter("marker_size").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        self.enable_viz = self.get_parameter("enable_visualization").value
        self.publish_image = self.get_parameter("publish_image").value
        self.use_depth_stream = self.get_parameter("use_depth_stream").value

        # Initialize RealSense pipeline and calibration
        self._init_realsense()

        # Initialize ArUco detector
        self._init_detector()

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Bridge for ROS Image messages
        self.cv_bridge = CvBridge()

        # Image publisher (optional)
        if self.publish_image:
            self.image_publisher = self.create_publisher(
                Image, "/d435/aruco/image_marked", qos_profile=QoSProfile(depth=1)
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
            f"D435ArucoPosePublisher initialized. "
            f"Serial: {self.camera_serial if self.camera_serial else 'auto-detect'}, "
            f"FPS: {self.camera_fps}, "
            f"Marker size: {self.marker_size}m"
        )

    def _init_realsense(self) -> None:
        """Initialize RealSense D435 pipeline and load calibration on-the-fly."""
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # Find device
            context = rs.context()
            devices = context.query_devices()

            if len(devices) == 0:
                raise RuntimeError("No RealSense device found")

            # Select device by serial or use first available
            selected_device = None
            if self.camera_serial:
                for device in devices:
                    if (
                        device.get_info(rs.camera_info.serial_number)
                        == self.camera_serial
                    ):
                        selected_device = device
                        break
                if selected_device is None:
                    raise RuntimeError(
                        f"Device with serial {self.camera_serial} not found"
                    )
            else:
                selected_device = devices[0]

            device_serial = selected_device.get_info(rs.camera_info.serial_number)
            device_name = selected_device.get_info(rs.camera_info.name)

            self.get_logger().info(
                f"Found RealSense device: {device_name} (Serial: {device_serial})"
            )

            # Configure streams
            self.config.enable_device(device_serial)
            self.config.enable_stream(
                rs.stream.color,
                self.frame_width,
                self.frame_height,
                rs.format.bgr8,
                self.camera_fps,
            )

            if self.use_depth_stream:
                self.config.enable_stream(
                    rs.stream.depth,
                    self.frame_width,
                    self.frame_height,
                    rs.format.z16,
                    self.camera_fps,
                )

            # Start pipeline
            pipeline_profile = self.pipeline.start(self.config)

            # Load calibration from RealSense on-the-fly
            self._load_realsense_calibration(pipeline_profile)

            self.get_logger().info("RealSense pipeline started with calibration loaded")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize RealSense: {e}")
            raise

    def _load_realsense_calibration(self, pipeline_profile) -> None:
        """
        Load camera calibration from RealSense device on-the-fly.

        Args:
            pipeline_profile: Pipeline profile from started pipeline
        """
        try:
            # Get color stream profile
            color_profile = pipeline_profile.get_stream(rs.stream.color)
            color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

            # Extract intrinsic parameters
            self.camera_matrix = np.array(
                [
                    [color_intrinsics.fx, 0, color_intrinsics.ppx],
                    [0, color_intrinsics.fy, color_intrinsics.ppy],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )

            # Extract distortion coefficients (Brown-Conrady model)
            # Order: [k1, k2, p1, p2, k3]
            self.dist_coeffs = np.array(color_intrinsics.coeffs, dtype=np.float32)

            self.get_logger().info(
                f"RealSense calibration loaded:\n"
                f"  Camera Matrix:\n{self.camera_matrix}\n"
                f"  Distortion Coefficients: {self.dist_coeffs}"
            )

            # Optional: Get depth intrinsics if depth stream is enabled
            if self.use_depth_stream:
                try:
                    depth_profile = pipeline_profile.get_stream(rs.stream.depth)
                    depth_intrinsics = (
                        depth_profile.as_video_stream_profile().get_intrinsics()
                    )
                    self.depth_scale = (
                        pipeline_profile.get_device()
                        .first_depth_sensor()
                        .get_depth_scale()
                    )

                    self.get_logger().info(
                        f"Depth stream calibration loaded:\n"
                        f"  Focal length: {depth_intrinsics.fx}, {depth_intrinsics.fy}\n"
                        f"  Principal point: {depth_intrinsics.ppx}, {depth_intrinsics.ppy}\n"
                        f"  Depth scale: {self.depth_scale}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"Could not load depth calibration: {e}")

        except Exception as e:
            self.get_logger().error(f"Failed to load RealSense calibration: {e}")
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

    def camera_timer_callback(self) -> None:
        """Capture frame from RealSense and process it."""
        if not self.running:
            return

        try:
            # Wait for frames with timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)

            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warn("No color frame received")
                return

            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())

            self.frame_count += 1
            frame = cv2.flip(
                frame, -1
            )  # Flip both horizontally and vertically (180 degrees)

            # Create ROS Image message with current timestamp
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.frame_id = "camera"
            ros_image.header.stamp = self.get_clock().now().to_msg()

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
        transform.header.frame_id = "d435_camera"
        transform.child_frame_id = f"d435_aruco_marker_{int(marker_id)}"

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
        cv2.imshow("D435 ArUco Detection", frame)
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

        if hasattr(self, "pipeline"):
            self.pipeline.stop()
            self.get_logger().info("RealSense pipeline stopped")

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = D435ArucoPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
