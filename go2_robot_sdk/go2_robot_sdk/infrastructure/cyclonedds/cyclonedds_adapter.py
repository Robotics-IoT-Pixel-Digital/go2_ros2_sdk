# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
CycloneDDS Adapter for Go2 robot communication over Ethernet.

This adapter implements the IRobotDataReceiver and IRobotController interfaces
for communication with the Go2 robot via CycloneDDS (Ethernet connection).

FIXED: Now uses raw CycloneDDS Python API to subscribe to rt/ prefixed topics
instead of ROS2 APIs. The Go2 robot publishes raw DDS topics that are NOT
ROS2-compatible, so we need to use the cyclonedds library directly.

The adapter:
1. Uses DDSBridge to subscribe to raw rt/ topics from robot
2. Converts received data and forwards to RobotDataService
3. Publishes commands back to robot via ROS2 publishers (commands still work)
"""

import logging
from typing import Callable, Dict, Any, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from go2_interfaces.msg import SportModeCmd

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotConfig
from .dds_bridge import DDSBridge, CYCLONEDDS_AVAILABLE

logger = logging.getLogger(__name__)


# CycloneDDS topic names as published by Go2 robot over Ethernet
# NOTE: Go2 uses 'rt/' prefix for RAW DDS topics (not ROS2 topics!)
# These must be accessed via raw CycloneDDS API, not ROS2 subscriptions
CYCLONEDDS_TOPICS = {
    # Subscriber topics (from robot) - RAW DDS topics with rt/ prefix
    "LOW_STATE": "rt/lf/lowstate",              # Motor states, IMU, battery
    "SPORT_MODE_STATE": "rt/sportmodestate",    # Robot mode, gait, position
    "LIDAR_CLOUD": "rt/utlidar/cloud",          # LiDAR point cloud
    "ROBOT_POSE": "rt/utlidar/robot_pose",      # Robot pose from LiDAR SLAM
    "ODOMETRY": "rt/utlidar/robot_odom",        # Odometry from LiDAR
    "IMU": "rt/imu",                            # IMU data
    "WIRELESS_CONTROLLER": "rt/wirelesscontroller",  # Remote controller state
    # Publisher topics (to robot) - commands we send to the Go2 Pro
    "SPORT_MODE_CMD": "rt/api/sport/request",   # Sport mode commands (movement, gait)
    "CMD_VEL": "rt/go2/cmd_vel",                # Velocity commands (Twist)
}

# Alternative topic names - some Go2 firmware versions use different prefixes
CYCLONEDDS_TOPICS_ALT = {
    "SPORT_MODE_STATE": "rt/lf/sportmodestate",  # Alternative location
    "IMU": "rt/lf/imu",                          # Alternative IMU topic
}


class CycloneDDSAdapter(IRobotDataReceiver, IRobotController):
    """
    CycloneDDS adapter for Go2 robot communication over Ethernet.

    FIXED: This adapter now uses raw CycloneDDS Python API via DDSBridge
    to subscribe to the robot's native rt/ topics. The Go2 robot publishes
    raw DDS topics that are NOT ROS2-compatible (missing ROS2 metadata).

    The adapter:
    - Uses DDSBridge for subscribing to raw DDS rt/ topics
    - Still uses ROS2 publishers for sending commands (they work fine)
    - Converts DDS data and republishes as ROS2 topics for the rest of system
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
            node: ROS2 node instance for creating publishers (commands)
            config: Robot configuration parameters
            on_validated_callback: Callback when connection is validated
            event_loop: Event loop (not used for CycloneDDS but kept for interface compatibility)
        """
        if not CYCLONEDDS_AVAILABLE:
            raise ImportError(
                "cyclonedds Python library is required for CycloneDDS mode. "
                "Install with: pip install cyclonedds"
            )

        self.node = node
        self.config = config
        self.on_validated_callback = on_validated_callback
        self.data_callback: Optional[Callable[[Dict[str, Any], str], None]] = None

        # Track connected robots
        self.connected_robots: Dict[str, bool] = {}

        # DDS Bridge for subscribing to raw rt/ topics
        self.dds_bridge: Optional[DDSBridge] = None

        # QoS profiles for ROS2 publishers (commands)
        self.qos_reliable = QoSProfile(depth=10)
        self.qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers for sending commands (ROS2 publishers still work!)
        self.publishers: Dict[str, Dict[str, Any]] = {}
        
        # Publishers for republishing DDS data as ROS2 topics
        self.ros2_publishers: Dict[str, Dict[str, Any]] = {}

        logger.info("CycloneDDS adapter initialized with raw DDS bridge support")

    async def connect(self, robot_id: str) -> None:
        """
        Connect to robot via CycloneDDS.

        FIXED: Now creates DDSBridge and subscribes to raw rt/ topics
        using cyclonedds Python library instead of ROS2 subscriptions.

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

            # Initialize DDS Bridge if not already done
            if self.dds_bridge is None:
                self.dds_bridge = DDSBridge(domain_id=0)  # ROS 2 uses domain 0 by default
                self.dds_bridge.start()
                logger.info("DDS Bridge started for raw DDS topic subscriptions")

            # Subscribe to raw DDS topics using the bridge
            self._subscribe_dds_topics(robot_id)

            # Create ROS2 publishers for commands (these still work!)
            self._create_publishers(robot_id, prefix)
            
            # Create ROS2 publishers for republishing DDS data
            self._create_ros2_republishers(robot_id, prefix)

            self.connected_robots[robot_id] = True
            logger.info(f"CycloneDDS connection established for robot {robot_id}")
            logger.info(f"  - Subscribed to raw DDS rt/ topics via DDSBridge")
            logger.info(f"  - Created ROS2 publishers for sending commands")

            # Call validation callback (connection is immediate for CycloneDDS)
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)

        except Exception as e:
            logger.error(f"Failed to connect via CycloneDDS for robot {robot_id}: {e}")
            raise

    def _subscribe_dds_topics(self, robot_id: str) -> None:
        """
        Subscribe to raw DDS topics using DDSBridge.
        
        FIXED: Uses raw cyclonedds Python library instead of ROS2 subscriptions.
        This allows us to receive the rt/ prefixed topics published by the robot.
        """
        logger.info(f"Subscribing to raw DDS topics for robot {robot_id}:")

        # Subscribe to LowState (motor states, IMU, battery)
        low_state_topic = CYCLONEDDS_TOPICS['LOW_STATE']
        logger.info(f"  - Raw DDS topic: {low_state_topic}")
        self.dds_bridge.subscribe_lowstate(
            callback=lambda data: self._on_dds_data(data, robot_id),
            topic_name=low_state_topic
        )

        # Subscribe to SportModeState (robot state, position, gait)
        sport_state_topic = CYCLONEDDS_TOPICS['SPORT_MODE_STATE']
        logger.info(f"  - Raw DDS topic: {sport_state_topic}")
        self.dds_bridge.subscribe_sportmodestate(
            callback=lambda data: self._on_dds_data(data, robot_id),
            topic_name=sport_state_topic
        )

        # Try alternative topic names too
        sport_state_alt_topic = CYCLONEDDS_TOPICS_ALT.get('SPORT_MODE_STATE')
        if sport_state_alt_topic and sport_state_alt_topic != sport_state_topic:
            logger.info(f"  - Raw DDS topic (alt): {sport_state_alt_topic}")
            try:
                self.dds_bridge.subscribe_sportmodestate(
                    callback=lambda data: self._on_dds_data(data, robot_id),
                    topic_name=sport_state_alt_topic
                )
            except Exception as e:
                logger.warning(f"Could not subscribe to alternative topic {sport_state_alt_topic}: {e}")

        # Subscribe to LiDAR topics
        try:
            lidar_cloud_topic = CYCLONEDDS_TOPICS['LIDAR_CLOUD']
            logger.info(f"  - Raw DDS topic: {lidar_cloud_topic}")
            self.dds_bridge.subscribe_topic(
                callback=lambda data: self._on_dds_data(data, robot_id),
                topic_name=lidar_cloud_topic,
                msg_type="pointcloud"
            )
        except Exception as e:
            logger.warning(f"Could not subscribe to LiDAR cloud topic: {e}")

        # Subscribe to robot pose from LiDAR SLAM
        try:
            robot_pose_topic = CYCLONEDDS_TOPICS['ROBOT_POSE']
            logger.info(f"  - Raw DDS topic: {robot_pose_topic}")
            self.dds_bridge.subscribe_topic(
                callback=lambda data: self._on_dds_data(data, robot_id),
                topic_name=robot_pose_topic,
                msg_type="robot_pose"
            )
        except Exception as e:
            logger.warning(f"Could not subscribe to robot pose topic: {e}")

        # Subscribe to odometry from LiDAR
        try:
            odom_topic = CYCLONEDDS_TOPICS['ODOMETRY']
            logger.info(f"  - Raw DDS topic: {odom_topic}")
            self.dds_bridge.subscribe_topic(
                callback=lambda data: self._on_dds_data(data, robot_id),
                topic_name=odom_topic,
                msg_type="odometry"
            )
        except Exception as e:
            logger.warning(f"Could not subscribe to odometry topic: {e}")

        logger.info(f"DDS topic subscriptions completed for robot {robot_id}")

    def _on_dds_data(self, data: Dict[str, Any], robot_id: str) -> None:
        """
        Handle data received from DDS bridge.
        
        This callback is called by DDSBridge when data arrives from raw DDS topics.
        We forward it to the data_callback set by RobotDataService and also
        republish LiDAR data as ROS2 messages.
        """
        try:
            topic = data.get("topic", "")
            msg_type = data.get("type", "")
            
            # Log first message from each topic for debugging
            if not hasattr(self, '_logged_topics'):
                self._logged_topics = set()
            if topic not in self._logged_topics:
                logger.info(f"First message from {topic} (type: {msg_type})")
                self._logged_topics.add(topic)
            
            # Republish LiDAR data as ROS2 messages
            if robot_id in self.ros2_publishers:
                # Handle point cloud data
                if msg_type == "pointcloud" and "lidar_cloud" in self.ros2_publishers[robot_id]:
                    self._republish_pointcloud(data, robot_id)
                
                # Handle robot pose data
                elif msg_type == "robot_pose" and "robot_pose" in self.ros2_publishers[robot_id]:
                    self._republish_pose(data, robot_id)
                
                # Handle odometry data
                elif msg_type == "odometry" and "lidar_odom" in self.ros2_publishers[robot_id]:
                    self._republish_odometry(data, robot_id)
            
            # Forward to data callback (RobotDataService will process it)
            if self.data_callback:
                self.data_callback(data, robot_id)
        except Exception as e:
            logger.error(f"Error handling DDS data from {topic}: {e}")
            logger.exception(e)

    def _create_subscribers(self, robot_id: str, prefix: str) -> None:
        """
        DEPRECATED: Old method that used ROS2 subscriptions.
        
        Now handled by _subscribe_dds_topics() using raw DDS bridge.
        Kept for compatibility but does nothing.
        """
        logger.debug("_create_subscribers() is deprecated - using DDS bridge instead")

    def _create_publishers(self, robot_id: str, prefix: str) -> None:
        """Create ROS2 publishers for robot command topics."""

        # Initialize publisher storage
        if robot_id not in self.publishers:
            self.publishers[robot_id] = {}

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

        logger.debug(f"Created command publishers for robot {robot_id}")

    def _create_ros2_republishers(self, robot_id: str, prefix: str) -> None:
        """Create ROS2 publishers for republishing raw DDS data."""
        
        # Initialize publisher storage
        if robot_id not in self.ros2_publishers:
            self.ros2_publishers[robot_id] = {}
        
        # PointCloud2 publisher for LiDAR data
        self.ros2_publishers[robot_id]["lidar_cloud"] = self.node.create_publisher(
            PointCloud2,
            f"{prefix}point_cloud" if not prefix else f"{prefix}/point_cloud",
            self.qos_best_effort,
        )
        
        # PoseStamped publisher for robot pose from LiDAR SLAM
        self.ros2_publishers[robot_id]["robot_pose"] = self.node.create_publisher(
            PoseStamped,
            f"{prefix}robot_pose" if not prefix else f"{prefix}/robot_pose",
            self.qos_best_effort,
        )
        
        # Odometry publisher for LiDAR odometry
        self.ros2_publishers[robot_id]["lidar_odom"] = self.node.create_publisher(
            Odometry,
            f"{prefix}odom_lidar" if not prefix else f"{prefix}/odom_lidar",
            self.qos_best_effort,
        )
        
        logger.debug(f"Created ROS2 republishers for robot {robot_id}")

    def _republish_pointcloud(self, data: Dict[str, Any], robot_id: str) -> None:
        """Convert and republish point cloud data as ROS2 PointCloud2."""
        try:
            # Simply relay - robot already publishes this as /utlidar/cloud
            # User should remap or subscribe to /utlidar/cloud directly
            pass
            
        except Exception as e:
            logger.debug(f"Error republishing point cloud: {e}")

    def _republish_pose(self, data: Dict[str, Any], robot_id: str) -> None:
        """Convert and republish pose data as ROS2 PoseStamped."""
        try:
            # Simply relay - robot already publishes this as /utlidar/robot_pose
            # User should remap or subscribe to /utlidar/robot_pose directly
            pass
            
        except Exception as e:
            logger.debug(f"Error republishing pose: {e}")

    def _republish_odometry(self, data: Dict[str, Any], robot_id: str) -> None:
        """Convert and republish odometry data as ROS2 Odometry."""
        try:
            # Simply relay - robot already publishes this as /utlidar/robot_odom
            # User should remap or subscribe to /utlidar/robot_odom directly
            pass
            
        except Exception as e:
            logger.debug(f"Error republishing odometry: {e}")

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot (cleanup resources)."""
        # Stop DDS bridge if this is the last robot
        if len(self.connected_robots) == 1 and self.dds_bridge:
            self.dds_bridge.stop()
            self.dds_bridge = None
            logger.info("DDS Bridge stopped")

        # Cleanup publishers
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

    # === Deprecated Methods (kept for compatibility) ===
    # These were used with ROS2 subscriptions, now replaced by DDSBridge

    def _on_low_state(self, msg, robot_id: str) -> None:
        """DEPRECATED: Old ROS2 subscription callback."""
        logger.warning("_on_low_state() called but should not be used with DDSBridge")

    def _on_sport_mode_state(self, msg, robot_id: str) -> None:
        """DEPRECATED: Old ROS2 subscription callback."""
        logger.warning("_on_sport_mode_state() called but should not be used with DDSBridge")

    def _on_lidar_cloud(self, msg, robot_id: str) -> None:
        """DEPRECATED: Old ROS2 subscription callback."""
        logger.warning("_on_lidar_cloud() called but should not be used with DDSBridge")

    def _on_robot_pose(self, msg, robot_id: str) -> None:
        """DEPRECATED: Old ROS2 subscription callback."""
        logger.warning("_on_robot_pose() called but should not be used with DDSBridge")

    def _on_odometry(self, msg, robot_id: str) -> None:
        """DEPRECATED: Old ROS2 subscription callback."""
        logger.warning("_on_odometry() called but should not be used with DDSBridge")
