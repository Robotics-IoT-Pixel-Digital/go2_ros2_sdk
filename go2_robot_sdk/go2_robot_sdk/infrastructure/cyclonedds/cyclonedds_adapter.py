# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
CycloneDDS Adapter for Go2 robot communication over Ethernet.

This adapter implements the IRobotDataReceiver and IRobotController interfaces
for communication with the Go2 robot via CycloneDDS (Ethernet connection).

When using CycloneDDS, the Go2 robot publishes standard ROS2 topics directly,
and we subscribe to them. Commands are sent via ROS2 topic publishers.
"""

import logging
from typing import Callable, Dict, Any, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from go2_interfaces.msg import LowState, SportModeState, SportModeCmd

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotConfig

logger = logging.getLogger(__name__)


# CycloneDDS topic names as published by Go2 robot over Ethernet
CYCLONEDDS_TOPICS = {
    # Subscriber topics (from robot)
    "LOW_STATE": "lowstate",
    "SPORT_MODE_STATE": "sportmodestate",
    "LIDAR_CLOUD": "/utlidar/cloud",
    "ROBOT_POSE": "/utlidar/robot_pose",
    "ODOMETRY": "/utlidar/robot_odom",
    "IMU": "imu",
    # Publisher topics (to robot)
    "SPORT_MODE_CMD": "sportmodecmd",
    "CMD_VEL": "cmd_vel",
}


class CycloneDDSAdapter(IRobotDataReceiver, IRobotController):
    """
    CycloneDDS adapter for Go2 robot communication over Ethernet.

    This adapter creates ROS2 subscriptions to the topics published by
    the Go2 robot when connected via Ethernet (CycloneDDS).

    Note: Unlike WebRTC which requires explicit connection, CycloneDDS
    communication is automatically established when the robot and computer
    are on the same DDS domain.
    """

    def __init__(
        self,
        node: Node,
        config: RobotConfig,
        on_validated_callback: Callable = None,
        event_loop=None,
    ):
        """
        Initialize the CycloneDDS adapter.

        Args:
            node: ROS2 node instance for creating publishers/subscribers
            config: Robot configuration parameters
            on_validated_callback: Callback when connection is validated
            event_loop: Event loop (not used for CycloneDDS but kept for interface compatibility)
        """
        self.node = node
        self.config = config
        self.on_validated_callback = on_validated_callback
        self.data_callback: Optional[Callable[[Dict[str, Any], str], None]] = None

        # Track connected robots
        self.connected_robots: Dict[str, bool] = {}

        # QoS profiles
        self.qos_reliable = QoSProfile(depth=10)
        self.qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers (will be created per robot)
        self.subscribers: Dict[str, list] = {}

        # Publishers for sending commands
        self.publishers: Dict[str, Dict[str, Any]] = {}

        logger.info("CycloneDDS adapter initialized")

    async def connect(self, robot_id: str) -> None:
        """
        Connect to robot via CycloneDDS.

        For CycloneDDS, "connection" means setting up ROS2 subscriptions
        to the robot's topics. The actual DDS connection is automatic.

        Args:
            robot_id: Robot identifier (index in robot list)
        """
        try:
            robot_idx = int(robot_id)

            # Create topic prefixes based on connection mode
            if self.config.conn_mode == "single":
                prefix = ""
            else:
                prefix = f"robot{robot_idx}/"

            # Initialize subscriber and publisher storage
            self.subscribers[robot_id] = []
            self.publishers[robot_id] = {}

            # Create subscribers for robot data
            self._create_subscribers(robot_id, prefix)

            # Create publishers for commands
            self._create_publishers(robot_id, prefix)

            self.connected_robots[robot_id] = True
            logger.info(f"CycloneDDS subscriptions created for robot {robot_id}")

            # Call validation callback (connection is immediate for CycloneDDS)
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)

        except Exception as e:
            logger.error(f"Failed to setup CycloneDDS for robot {robot_id}: {e}")
            raise

    def _create_subscribers(self, robot_id: str, prefix: str) -> None:
        """Create ROS2 subscribers for robot data topics."""

        # LowState subscriber (motor states, IMU, foot force)
        low_state_sub = self.node.create_subscription(
            LowState,
            f"{prefix}{CYCLONEDDS_TOPICS['LOW_STATE']}",
            lambda msg, rid=robot_id: self._on_low_state(msg, rid),
            self.qos_reliable,
        )
        self.subscribers[robot_id].append(low_state_sub)

        # SportModeState subscriber (robot state, position, gait)
        sport_state_sub = self.node.create_subscription(
            SportModeState,
            f"{prefix}{CYCLONEDDS_TOPICS['SPORT_MODE_STATE']}",
            lambda msg, rid=robot_id: self._on_sport_mode_state(msg, rid),
            self.qos_reliable,
        )
        self.subscribers[robot_id].append(sport_state_sub)

        # LiDAR point cloud subscriber
        # Note: LiDAR topics typically don't use robot prefix on Go2
        lidar_topic = CYCLONEDDS_TOPICS["LIDAR_CLOUD"]
        lidar_sub = self.node.create_subscription(
            PointCloud2,
            lidar_topic,
            lambda msg, rid=robot_id: self._on_lidar_cloud(msg, rid),
            self.qos_best_effort,
        )
        self.subscribers[robot_id].append(lidar_sub)

        # Robot pose subscriber
        pose_topic = CYCLONEDDS_TOPICS["ROBOT_POSE"]
        pose_sub = self.node.create_subscription(
            PoseStamped,
            pose_topic,
            lambda msg, rid=robot_id: self._on_robot_pose(msg, rid),
            self.qos_reliable,
        )
        self.subscribers[robot_id].append(pose_sub)

        # Odometry subscriber
        odom_topic = CYCLONEDDS_TOPICS["ODOMETRY"]
        odom_sub = self.node.create_subscription(
            Odometry,
            odom_topic,
            lambda msg, rid=robot_id: self._on_odometry(msg, rid),
            self.qos_reliable,
        )
        self.subscribers[robot_id].append(odom_sub)

        logger.debug(
            f"Created {len(self.subscribers[robot_id])} subscribers for robot {robot_id}"
        )

    def _create_publishers(self, robot_id: str, prefix: str) -> None:
        """Create ROS2 publishers for robot command topics."""

        # SportModeCmd publisher for movement commands
        self.publishers[robot_id]["sport_cmd"] = self.node.create_publisher(
            SportModeCmd,
            f"{prefix}{CYCLONEDDS_TOPICS['SPORT_MODE_CMD']}",
            self.qos_reliable,
        )

        # Twist publisher for velocity commands (alternative interface)
        self.publishers[robot_id]["cmd_vel"] = self.node.create_publisher(
            Twist,
            f"{prefix}{CYCLONEDDS_TOPICS['CMD_VEL']}",
            self.qos_reliable,
        )

        logger.debug(f"Created publishers for robot {robot_id}")

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot (cleanup subscriptions)."""
        if robot_id in self.subscribers:
            for sub in self.subscribers[robot_id]:
                self.node.destroy_subscription(sub)
            del self.subscribers[robot_id]

        if robot_id in self.publishers:
            for pub in self.publishers[robot_id].values():
                self.node.destroy_publisher(pub)
            del self.publishers[robot_id]

        if robot_id in self.connected_robots:
            del self.connected_robots[robot_id]

        logger.info(f"Disconnected from robot {robot_id}")

    def set_data_callback(
        self, callback: Callable[[Dict[str, Any], str], None]
    ) -> None:
        """Set callback for data reception."""
        self.data_callback = callback

    def send_command(self, robot_id: str, command: str) -> None:
        """
        Send raw command to robot.

        Note: For CycloneDDS, this is a no-op as commands are sent
        via typed ROS2 messages, not raw strings.
        """
        logger.debug(
            "send_command() called in CycloneDDS mode; raw command strings are ignored "
            "because only typed ROS2 messages are supported"
        )

    def send_movement_command(
        self, robot_id: str, x: float, y: float, z: float
    ) -> None:
        """Send movement command to robot via CycloneDDS."""
        try:
            if robot_id not in self.publishers:
                logger.warning(f"No publishers for robot {robot_id}")
                return

            # Create SportModeCmd message
            cmd = SportModeCmd()
            cmd.mode = 2  # Walk mode
            cmd.gait_type = 1  # Trot gait
            cmd.speed_level = 1
            cmd.foot_raise_height = 0.1
            cmd.body_height = 0.0
            cmd.velocity = [float(x), float(y)]
            cmd.yaw_speed = float(z)

            self.publishers[robot_id]["sport_cmd"].publish(cmd)

            # Also publish as Twist for compatibility
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.angular.z = float(z)
            self.publishers[robot_id]["cmd_vel"].publish(twist)

            logger.debug(
                f"Movement command sent to robot {robot_id}: x={x}, y={y}, z={z}"
            )

        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command via CycloneDDS."""
        try:
            if robot_id not in self.publishers:
                return

            cmd = SportModeCmd()
            cmd.mode = 1  # Stand up mode
            cmd.gait_type = 0
            cmd.speed_level = 0
            cmd.foot_raise_height = 0.0
            cmd.body_height = 0.0
            cmd.velocity = [0.0, 0.0]
            cmd.yaw_speed = 0.0

            self.publishers[robot_id]["sport_cmd"].publish(cmd)
            logger.info(f"Stand up command sent to robot {robot_id}")

        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command via CycloneDDS."""
        try:
            if robot_id not in self.publishers:
                return

            cmd = SportModeCmd()
            cmd.mode = 5  # Stand down mode
            cmd.gait_type = 0
            cmd.speed_level = 0
            cmd.foot_raise_height = 0.0
            cmd.body_height = 0.0
            cmd.velocity = [0.0, 0.0]
            cmd.yaw_speed = 0.0

            self.publishers[robot_id]["sport_cmd"].publish(cmd)
            logger.info(f"Stand down command sent to robot {robot_id}")

        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    def send_webrtc_request(
        self, robot_id: str, api_id: int, parameter: Any, topic: str
    ) -> None:
        """
        Send WebRTC-style request.

        Note: In CycloneDDS mode, WebRTC requests are mapped to appropriate
        ROS2 topic publishes where possible.
        """
        logger.debug(
            f"WebRTC request in CycloneDDS mode - api_id: {api_id}, topic: {topic}"
        )
        # Most WebRTC requests don't have direct CycloneDDS equivalents
        # Handle specific cases as needed

    def process_webrtc_commands(self, robot_id: str) -> None:
        """Process queued commands (no-op for CycloneDDS)."""
        pass

    # === Data Reception Callbacks ===

    def _on_low_state(self, msg: LowState, robot_id: str) -> None:
        """Handle LowState message from robot."""
        try:
            if not self.data_callback:
                return

            # Convert LowState to format expected by RobotDataService
            motor_states = []
            for motor in msg.motor_state[:12]:  # 12 joints for quadruped
                motor_states.append(
                    {
                        "mode": motor.mode,
                        "q": motor.q,
                        "dq": motor.dq,
                        "ddq": motor.ddq,
                        "tau_est": motor.tau_est,
                        "q_raw": motor.q_raw,
                        "dq_raw": motor.dq_raw,
                        "ddq_raw": motor.ddq_raw,
                        "temperature": motor.temperature,
                        "lost": motor.lost,
                    }
                )

            # Build message dict matching WebRTC format
            data = {
                "topic": "rt/lf/lowstate",  # Match WebRTC topic
                "data": {
                    "head": list(msg.head),
                    "level_flag": msg.level_flag,
                    "frame_reserve": msg.frame_reserve,
                    "motor_state": motor_states,
                    "imu_state": {
                        "quaternion": list(msg.imu_state.quaternion),
                        "accelerometer": list(msg.imu_state.accelerometer),
                        "gyroscope": list(msg.imu_state.gyroscope),
                        "rpy": list(msg.imu_state.rpy),
                        "temperature": msg.imu_state.temperature,
                    },
                    "bms_state": {
                        "version_high": msg.bms_state.version_high if hasattr(msg.bms_state, 'version_high') else 0,
                        "version_low": msg.bms_state.version_low if hasattr(msg.bms_state, 'version_low') else 0,
                        "soc": msg.bms_state.soc if hasattr(msg.bms_state, 'soc') else 0,
                        "current": msg.bms_state.current if hasattr(msg.bms_state, 'current') else 0,
                        "cycle": msg.bms_state.cycle if hasattr(msg.bms_state, 'cycle') else 0,
                    },
                    "foot_force": list(msg.foot_force),
                    "foot_force_est": list(msg.foot_force_est),
                    "tick": msg.tick,
                    "wireless_remote": list(msg.wireless_remote),
                    "bit_flag": msg.bit_flag,
                    "adc_reel": msg.adc_reel,
                    "temperature_ntc1": msg.temperature_ntc1,
                    "temperature_ntc2": msg.temperature_ntc2,
                    "power_v": msg.power_v,
                    "power_a": msg.power_a,
                    "fan_frequency": list(msg.fan_frequency),
                    "reserve": msg.reserve,
                },
            }

            self.data_callback(data, robot_id)

        except Exception as e:
            logger.error(f"Error processing LowState: {e}")

    def _on_sport_mode_state(self, msg: SportModeState, robot_id: str) -> None:
        """Handle SportModeState message from robot."""
        try:
            if not self.data_callback:
                return

            # Build message dict matching WebRTC format
            data = {
                "topic": "rt/lf/sportmodestate",  # Match WebRTC topic
                "data": {
                    "stamp": {
                        "sec": msg.stamp.sec if hasattr(msg.stamp, 'sec') else 0,
                        "nanosec": msg.stamp.nanosec if hasattr(msg.stamp, 'nanosec') else 0,
                    },
                    "error_code": msg.error_code,
                    "mode": msg.mode,
                    "progress": msg.progress,
                    "gait_type": msg.gait_type,
                    "foot_raise_height": msg.foot_raise_height,
                    "position": list(msg.position),
                    "body_height": msg.body_height,
                    "velocity": list(msg.velocity),
                    "yaw_speed": msg.yaw_speed,
                    "range_obstacle": list(msg.range_obstacle),
                    "foot_force": list(msg.foot_force),
                    "foot_position_body": list(msg.foot_position_body),
                    "foot_speed_body": list(msg.foot_speed_body),
                    "imu_state": {
                        "quaternion": list(msg.imu_state.quaternion),
                        "accelerometer": list(msg.imu_state.accelerometer),
                        "gyroscope": list(msg.imu_state.gyroscope),
                        "rpy": list(msg.imu_state.rpy),
                        "temperature": msg.imu_state.temperature,
                    },
                },
            }

            self.data_callback(data, robot_id)

        except Exception as e:
            logger.error(f"Error processing SportModeState: {e}")

    def _on_lidar_cloud(self, msg: PointCloud2, robot_id: str) -> None:
        """Handle LiDAR PointCloud2 message from robot."""
        try:
            if not self.data_callback:
                return

            # For CycloneDDS, the PointCloud2 data is already in standard format
            # We pass it through to be republished
            data = {
                "topic": "cyclonedds/lidar_cloud",
                "pointcloud2_msg": msg,
                "robot_id": robot_id,
            }

            self.data_callback(data, robot_id)

        except Exception as e:
            logger.error(f"Error processing LiDAR cloud: {e}")

    def _on_robot_pose(self, msg: PoseStamped, robot_id: str) -> None:
        """Handle robot pose message."""
        try:
            if not self.data_callback:
                return

            data = {
                "topic": "rt/utlidar/robot_pose",  # Match WebRTC topic format
                "data": {
                    "pose": {
                        "position": {
                            "x": msg.pose.position.x,
                            "y": msg.pose.position.y,
                            "z": msg.pose.position.z,
                        },
                        "orientation": {
                            "x": msg.pose.orientation.x,
                            "y": msg.pose.orientation.y,
                            "z": msg.pose.orientation.z,
                            "w": msg.pose.orientation.w,
                        },
                    }
                },
            }

            self.data_callback(data, robot_id)

        except Exception as e:
            logger.error(f"Error processing robot pose: {e}")

    def _on_odometry(self, msg: Odometry, robot_id: str) -> None:
        """Handle odometry message."""
        try:
            if not self.data_callback:
                return

            data = {
                "topic": "rt/utlidar/robot_pose",
                "data": {
                    "pose": {
                        "position": {
                            "x": msg.pose.pose.position.x,
                            "y": msg.pose.pose.position.y,
                            "z": msg.pose.pose.position.z,
                        },
                        "orientation": {
                            "x": msg.pose.pose.orientation.x,
                            "y": msg.pose.pose.orientation.y,
                            "z": msg.pose.pose.orientation.z,
                            "w": msg.pose.pose.orientation.w,
                        },
                    }
                },
            }

            self.data_callback(data, robot_id)

        except Exception as e:
            logger.error(f"Error processing odometry: {e}")
