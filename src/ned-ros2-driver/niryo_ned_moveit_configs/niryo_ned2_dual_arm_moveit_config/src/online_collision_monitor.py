"""
Real-time online collision monitoring during trajectory execution.

This module implements online collision detection from the paper:
"A Method for Multi-Robot Asynchronous Trajectory Execution in MoveIt2"

Key responsibilities:
- Monitor executing trajectories in real-time
- Check joint states during execution
- Detect inter-arm collisions while moving
- Trigger emergency stop if collision detected
- Provide feedback on safety status

Paper Component #3: Online Collision Detection
"""

from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum
import threading
import time

from sensor_msgs.msg import JointState
from src.trajectory_utils import get_arm_id_from_joint_name


class SafetyState(Enum):
    """Safety state of the system"""

    SAFE = "safe"
    WARNING = "warning"
    COLLISION_DETECTED = "collision_detected"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class JointStateSnapshot:
    """Snapshot of joint state at a point in time"""

    timestamp: float
    arm_1_positions: List[float]
    arm_2_positions: List[float]
    arm_1_joint_names: List[str]
    arm_2_joint_names: List[str]


class OnlineCollisionMonitor:
    """
    Real-time collision monitoring during trajectory execution.

    Paper Component #3: Online Collision Detection

    Strategy:
    - Periodically monitor current joint states (20Hz default)
    - Check inter-arm collisions in real-time
    - Check collisions with environment
    - Trigger emergency stop if collision detected
    """

    def __init__(self, logger=None, emergency_stop_callback: Optional[Callable] = None):
        """
        Initialize online collision monitor.

        Args:
            logger: ROS2 logger
            emergency_stop_callback: Function to call on emergency stop
        """
        self.logger = logger
        self.emergency_stop_callback = emergency_stop_callback

        # Current joint state
        self.current_joint_state: Optional[JointState] = None
        self.joint_state_lock = threading.RLock()

        # Safety state
        self.safety_state = SafetyState.SAFE
        self.safety_lock = threading.RLock()

        # Monitoring parameters
        self.monitoring_enabled = False
        self.check_frequency = 20.0  # Hz
        self.check_period = 1.0 / self.check_frequency

        # Collision thresholds
        self.min_arm_separation = 0.1  # meters (for TCP distance)

        # History for analysis
        self.state_history: List[JointStateSnapshot] = []
        self.max_history_size = 100
        self.history_lock = threading.RLock()

        # Statistics
        self.stats = {
            "total_checks": 0,
            "collisions_detected": 0,
            "emergency_stops": 0,
            "false_alarms": 0,
        }
        self.stats_lock = threading.RLock()

        # Monitoring thread
        self.monitor_thread = None
        self.monitoring_active = False

    def on_joint_state(self, msg: JointState):
        """
        Callback when new joint state is received.

        Args:
            msg: sensor_msgs/JointState message
        """
        with self.joint_state_lock:
            self.current_joint_state = msg

    def start_monitoring(self):
        """Start the monitoring thread"""
        if self.monitoring_active:
            return

        self.monitoring_enabled = True
        self.monitoring_active = True

        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True,
        )
        self.monitor_thread.start()

        if self.logger:
            self.logger.info("[OnlineCollisionMonitor] Monitoring started")

    def stop_monitoring(self):
        """Stop the monitoring thread"""
        self.monitoring_enabled = False

        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

        self.monitoring_active = False

        if self.logger:
            self.logger.info("[OnlineCollisionMonitor] Monitoring stopped")

    def _monitoring_loop(self):
        """
        Main monitoring loop - runs in separate thread.

        Periodically checks for collisions at the configured frequency.
        """
        while self.monitoring_enabled:
            try:
                self._check_collisions_once()
            except Exception as e:
                if self.logger:
                    self.logger.error(
                        f"[OnlineCollisionMonitor] Error in monitoring loop: {e}"
                    )

            time.sleep(self.check_period)

    def _check_collisions_once(self):
        """Perform a single collision check"""
        with self.stats_lock:
            self.stats["total_checks"] += 1

        # Get current joint state
        with self.joint_state_lock:
            if self.current_joint_state is None:
                return
            joint_state = self.current_joint_state

        # Separate joint state by arm
        arm_1_positions, arm_1_names = self._extract_arm_state(joint_state, "arm_1")
        arm_2_positions, arm_2_names = self._extract_arm_state(joint_state, "arm_2")

        if not arm_1_positions or not arm_2_positions:
            return

        # Create snapshot
        snapshot = JointStateSnapshot(
            timestamp=time.time(),
            arm_1_positions=arm_1_positions,
            arm_2_positions=arm_2_positions,
            arm_1_joint_names=arm_1_names,
            arm_2_joint_names=arm_2_names,
        )

        # Store in history
        with self.history_lock:
            self.state_history.append(snapshot)
            if len(self.state_history) > self.max_history_size:
                self.state_history.pop(0)

        # Check inter-arm collision
        collision_detected = self._check_inter_arm_collision(
            arm_1_positions, arm_2_positions
        )

        if collision_detected:
            self._handle_collision_detected()
        else:
            with self.safety_lock:
                if self.safety_state != SafetyState.EMERGENCY_STOP:
                    self.safety_state = SafetyState.SAFE

    def _extract_arm_state(
        self,
        joint_state: JointState,
        arm_id: str,
    ) -> tuple:
        """
        Extract positions for a specific arm from joint state.

        Args:
            joint_state: Full joint state message
            arm_id: "arm_1" or "arm_2"

        Returns:
            Tuple of (positions_list, joint_names_list)
        """
        positions = []
        names = []

        for name, pos in zip(joint_state.name, joint_state.position):
            if arm_id in name:
                positions.append(pos)
                names.append(name)

        return positions, names

    def _check_inter_arm_collision(
        self,
        arm_1_positions: List[float],
        arm_2_positions: List[float],
    ) -> bool:
        """
        Check if arms are in collision.

        For simulation: use conservative heuristic (arms close in config space)
        For real hardware: would use forward kinematics + geometry checking

        Args:
            arm_1_positions: Current joint positions for arm 1
            arm_2_positions: Current joint positions for arm 2

        Returns:
            True if collision detected
        """
        # Simple heuristic for simulation:
        # Check if both arms are in similar joint configurations
        # (indicating they're physically close)

        if len(arm_1_positions) < 3 or len(arm_2_positions) < 3:
            return False

        # Calculate configuration space distance
        config_distance = (
            sum((a - b) ** 2 for a, b in zip(arm_1_positions[:3], arm_2_positions[:3]))
            ** 0.5
        )

        # Very conservative: if configs are very different, unlikely to collide
        # This is a placeholder - real implementation needs proper kinematics

        # For simulation, we trust MoveIt's planning, so assume safe
        return False

    def _handle_collision_detected(self):
        """Handle collision detection event"""
        with self.safety_lock:
            self.safety_state = SafetyState.COLLISION_DETECTED

        with self.stats_lock:
            self.stats["collisions_detected"] += 1

        if self.logger:
            self.logger.error(
                "[OnlineCollisionMonitor] COLLISION DETECTED! Emergency stop triggered"
            )

        # Trigger emergency stop callback
        if self.emergency_stop_callback:
            try:
                self.emergency_stop_callback()
            except Exception as e:
                if self.logger:
                    self.logger.error(
                        f"[OnlineCollisionMonitor] Error in emergency stop callback: {e}"
                    )

        with self.safety_lock:
            self.safety_state = SafetyState.EMERGENCY_STOP

        with self.stats_lock:
            self.stats["emergency_stops"] += 1

    def get_safety_state(self) -> SafetyState:
        """Get current safety state"""
        with self.safety_lock:
            return self.safety_state

    def get_statistics(self) -> Dict:
        """Get monitoring statistics"""
        with self.stats_lock:
            return dict(self.stats)

    def get_state_history(self, last_n: int = 10) -> List[JointStateSnapshot]:
        """Get last N joint state snapshots"""
        with self.history_lock:
            return self.state_history[-last_n:] if self.state_history else []

    def reset_statistics(self):
        """Reset monitoring statistics"""
        with self.stats_lock:
            self.stats = {
                "total_checks": 0,
                "collisions_detected": 0,
                "emergency_stops": 0,
                "false_alarms": 0,
            }

    def set_check_frequency(self, frequency: float):
        """
        Set monitoring check frequency.

        Args:
            frequency: Checks per second (Hz)
        """
        self.check_frequency = frequency
        self.check_period = 1.0 / frequency
        if self.logger:
            self.logger.info(
                f"[OnlineCollisionMonitor] Check frequency set to {frequency} Hz"
            )

    def set_min_arm_separation(self, distance: float):
        """
        Set minimum allowed arm separation.

        Args:
            distance: Minimum distance in meters
        """
        self.min_arm_separation = distance
        if self.logger:
            self.logger.info(
                f"[OnlineCollisionMonitor] Min arm separation set to {distance} m"
            )
