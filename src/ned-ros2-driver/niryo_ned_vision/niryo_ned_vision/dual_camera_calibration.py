
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from collections import defaultdict


class CalibrationCorrector(Node):
    """Corrects d435_camera transform by comparing marker detections."""

    def __init__(self):
        super().__init__("calibration_corrector")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data collection
        self.measurements = defaultdict(list)
        self.running = True
        self.collection_duration = 60.0  # seconds
        self.start_time = time.time()

        self.get_logger().info("Calibration Corrector started")
        self.get_logger().info(
            f"Collecting data for {self.collection_duration} seconds..."
        )
        self.get_logger().info("Move the ArUco tag around to get diverse measurements")

        # Timer to collect data
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.collect_data_callback)

    def collect_data_callback(self):
        """Collect TF data from both cameras."""
        elapsed = time.time() - self.start_time

        if elapsed > self.collection_duration:
            self.finish_calibration()
            self.running = False
            return

        try:
            # Try to find any mono_aruco_marker_* and corresponding d435_aruco_marker_*
            # range of marker IDs (0-20)
            for marker_id in range(21):
                mono_frame = f"mono_aruco_marker_{marker_id}"
                d435_frame = f"d435_aruco_marker_{marker_id}"

                try:
                    # Get transform from mono_camera to marker
                    mono_tf = self.tf_buffer.lookup_transform(
                        "mono_camera", mono_frame, rclpy.time.Time()
                    )

                    # Get transform from d435_camera to marker
                    d435_tf = self.tf_buffer.lookup_transform(
                        "d435_camera", d435_frame, rclpy.time.Time()
                    )

                    # Record the measurement
                    measurement = {
                        "marker_id": marker_id,
                        "mono_pos": [
                            mono_tf.transform.translation.x,
                            mono_tf.transform.translation.y,
                            mono_tf.transform.translation.z,
                        ],
                        "mono_quat": [
                            mono_tf.transform.rotation.x,
                            mono_tf.transform.rotation.y,
                            mono_tf.transform.rotation.z,
                            mono_tf.transform.rotation.w,
                        ],
                        "d435_pos": [
                            d435_tf.transform.translation.x,
                            d435_tf.transform.translation.y,
                            d435_tf.transform.translation.z,
                        ],
                        "d435_quat": [
                            d435_tf.transform.rotation.x,
                            d435_tf.transform.rotation.y,
                            d435_tf.transform.rotation.z,
                            d435_tf.transform.rotation.w,
                        ],
                    }
                    self.measurements[marker_id].append(measurement)

                except TransformException:
                    # Marker not found or not yet published
                    pass

        except Exception as e:
            self.get_logger().debug(f"Error collecting data: {e}")

        # Print progress
        total_samples = sum(len(v) for v in self.measurements.values())
        if total_samples > 0:
            self.get_logger().info(
                f"[{elapsed:.1f}s] Collected {total_samples} measurements from "
                f"{len(self.measurements)} unique markers"
            )

    def finish_calibration(self):
        """Calculate and output calibration results."""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("CALIBRATION CORRECTION RESULTS")
        self.get_logger().info("=" * 70)

        total_samples = sum(len(v) for v in self.measurements.values())

        if total_samples == 0:
            self.get_logger().error(
                "No measurements collected! Make sure both cameras are detecting markers."
            )
            return

        # Collect all position and rotation differences
        position_diffs = []
        rotation_diffs = []

        for marker_id, measurements in self.measurements.items():
            for meas in measurements:
                # Position difference (d435 - mono)
                mono_pos = np.array(meas["mono_pos"])
                d435_pos = np.array(meas["d435_pos"])
                pos_diff = d435_pos - mono_pos
                position_diffs.append(pos_diff)

                # Rotation difference
                mono_quat = np.array(meas["mono_quat"])
                d435_quat = np.array(meas["d435_quat"])

                # Convert to rotation matrices
                r_mono = R.from_quat(mono_quat)
                r_d435 = R.from_quat(d435_quat)

                # Relative rotation: how much d435 is rotated from mono
                r_diff = r_d435 * r_mono.inv()
                euler_diff = r_diff.as_euler("xyz", degrees=False)
                rotation_diffs.append(euler_diff)

        position_diffs = np.array(position_diffs)
        rotation_diffs = np.array(rotation_diffs)

        # Calculate statistics
        avg_pos_diff = np.mean(position_diffs, axis=0)
        std_pos_diff = np.std(position_diffs, axis=0)
        avg_rot_diff = np.mean(rotation_diffs, axis=0)
        std_rot_diff = np.std(rotation_diffs, axis=0)

        # Current TF values
        current_x = 0.0385
        current_y = 0.0
        current_z = 0.0
        current_yaw = 1.7453  # radians

        # Corrected values
        corrected_x = current_x + avg_pos_diff[0]
        corrected_y = current_y + avg_pos_diff[1]
        corrected_z = current_z + avg_pos_diff[2]
        corrected_roll = avg_rot_diff[0]
        corrected_pitch = avg_rot_diff[1]
        corrected_yaw = current_yaw + avg_rot_diff[2]

        # OUTPUT 1: TF Distance in Euler Angles
        self.get_logger().info("\n[OUTPUT 1] TF Distance (Euler Angles):")
        self.get_logger().info("-" * 70)
        self.get_logger().info(f"Position Difference:")
        self.get_logger().info(
            f"  X: {avg_pos_diff[0]:+.6f} m (±{std_pos_diff[0]:.6f} m)"
        )
        self.get_logger().info(
            f"  Y: {avg_pos_diff[1]:+.6f} m (±{std_pos_diff[1]:.6f} m)"
        )
        self.get_logger().info(
            f"  Z: {avg_pos_diff[2]:+.6f} m (±{std_pos_diff[2]:.6f} m)"
        )
        self.get_logger().info(f"Rotation Difference (Euler - radians):")
        self.get_logger().info(
            f"  Roll:  {avg_rot_diff[0]:+.6f} rad (±{std_rot_diff[0]:.6f} rad)"
        )
        self.get_logger().info(
            f"  Pitch: {avg_rot_diff[1]:+.6f} rad (±{std_rot_diff[1]:.6f} rad)"
        )
        self.get_logger().info(
            f"  Yaw:   {avg_rot_diff[2]:+.6f} rad (±{std_rot_diff[2]:.6f} rad)"
        )

        # Convert radians to degrees for user reference
        deg_roll = np.degrees(avg_rot_diff[0])
        deg_pitch = np.degrees(avg_rot_diff[1])
        deg_yaw = np.degrees(avg_rot_diff[2])
        self.get_logger().info(f"Rotation Difference (Euler - degrees):")
        self.get_logger().info(f"  Roll:  {deg_roll:+.4f}°")
        self.get_logger().info(f"  Pitch: {deg_pitch:+.4f}°")
        self.get_logger().info(f"  Yaw:   {deg_yaw:+.4f}°")

        # OUTPUT 2: Corrected Launch File Arguments
        self.get_logger().info(
            "\n[OUTPUT 2] Corrected d435_camera_tf_broadcaster Arguments:"
        )
        self.get_logger().info("-" * 70)
        self.get_logger().info("Replace the arguments list in vision.launch.py with:")
        self.get_logger().info("[")
        self.get_logger().info(
            f'    "{corrected_x:.6f}",    # X translation (was {current_x})'
        )
        self.get_logger().info(
            f'    "{corrected_y:.6f}",    # Y translation (was {current_y})'
        )
        self.get_logger().info(
            f'    "{corrected_z:.6f}",    # Z translation (was {current_z})'
        )
        self.get_logger().info(
            f'    "{corrected_roll:.6f}",   # Roll rotation in radians'
        )
        self.get_logger().info(
            f'    "{corrected_pitch:.6f}",  # Pitch rotation in radians'
        )
        self.get_logger().info(
            f'    "{corrected_yaw:.6f}",    # Yaw rotation in radians (was {current_yaw})'
        )
        self.get_logger().info('    "arm_1_wrist_link",')
        self.get_logger().info('    "d435_camera",')
        self.get_logger().info("]")

        # Summary
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info(f"Statistics: {total_samples} measurements collected")
        self.get_logger().info(f"From {len(self.measurements)} unique marker IDs")
        magnitude = np.linalg.norm(avg_pos_diff)
        self.get_logger().info(f"Average position error magnitude: {magnitude:.6f} m")
        self.get_logger().info("=" * 70 + "\n")

        # Clean shutdown
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCorrector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted by user")
        node.finish_calibration()
    finally:
        if node.running:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
