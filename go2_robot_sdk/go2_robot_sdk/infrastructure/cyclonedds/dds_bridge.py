# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Raw DDS Bridge for Go2 Robot Communication.

This module uses the cyclonedds Python library to subscribe directly to
the robot's raw DDS topics (rt/ prefix) without ROS2 middleware.

The bridge:
1. Uses XTypes dynamic type discovery to find topic types
2. Subscribes to raw DDS topics using discovered types
3. Converts raw DDS data to Python dictionaries
4. Publishes commands to robot using RPC Request messages (unitree_api format)
"""

import logging
import time
import threading
import json
from dataclasses import dataclass, field
from typing import Callable, Dict, Any, Optional, List
from datetime import timedelta

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.pub import DataWriter
    from cyclonedds.util import duration
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    from cyclonedds.core import ReadCondition, SampleState, ViewState, InstanceState, Qos, Policy
    from cyclonedds.dynamic import get_types_for_typeid
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.annotations import keylist, final, autoid
    from cyclonedds.idl.types import sequence, array, int64, int32, uint8
    CYCLONEDDS_AVAILABLE = True
except ImportError:
    CYCLONEDDS_AVAILABLE = False
    logging.warning("cyclonedds Python library not available. Install with: pip install cyclonedds")

logger = logging.getLogger(__name__)


# ============================================================================
# Sport API IDs (from unitree_sdk2_python)
# ============================================================================
SPORT_API_ID_DAMP = 1001
SPORT_API_ID_BALANCESTAND = 1002
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_STANDUP = 1004
SPORT_API_ID_STANDDOWN = 1005
SPORT_API_ID_RECOVERYSTAND = 1006
SPORT_API_ID_EULER = 1007
SPORT_API_ID_MOVE = 1008
SPORT_API_ID_SIT = 1009
SPORT_API_ID_RISESIT = 1010
SPORT_API_ID_SPEEDLEVEL = 1015


# ============================================================================
# DDS Request/Response Types for Go2 Robot RPC Communication
# ============================================================================
# These dataclasses define the RPC message structures for communicating with 
# the Go2 robot via raw CycloneDDS. The robot uses a request/response RPC
# pattern over DDS topics.
#
# Topics:
#   - rt/api/sport/request - Send Request_ messages to control the robot
#   - rt/api/sport/response - Receive Response_ messages with results
# ============================================================================

@dataclass
class RequestIdentity(IdlStruct, typename="unitree_api.msg.dds_.RequestIdentity_"):
    """Identity information for RPC request."""
    id: int64 = 0        # int64 - unique request ID (usually timestamp in ns)
    api_id: int64 = 0    # int64 - API function ID (e.g., 1008 for Move)


@dataclass
class RequestLease(IdlStruct, typename="unitree_api.msg.dds_.RequestLease_"):
    """Lease information for RPC request."""
    id: int64 = 0  # int64 - lease ID (usually 0)


@dataclass
class RequestPolicy(IdlStruct, typename="unitree_api.msg.dds_.RequestPolicy_"):
    """Policy settings for RPC request."""
    priority: int32 = 0   # int32 - request priority
    noreply: bool = True  # bool - if true, don't wait for response


@dataclass
class RequestHeader(IdlStruct, typename="unitree_api.msg.dds_.RequestHeader_"):
    """Header for RPC request message."""
    identity: RequestIdentity = field(default_factory=RequestIdentity)
    lease: RequestLease = field(default_factory=RequestLease)
    policy: RequestPolicy = field(default_factory=RequestPolicy)


@dataclass
class Request(IdlStruct, typename="unitree_api.msg.dds_.Request_"):
    """
    RPC Request message for Go2 robot API calls.
    
    This is the main message type for sending commands to the robot.
    Published to "rt/api/sport/request" topic for sport/movement commands.
    
    The parameter field contains JSON-encoded command parameters.
    For Move command (API ID 1008): {"x": vx, "y": vy, "z": vyaw}
    """
    header: RequestHeader = field(default_factory=RequestHeader)
    parameter: str = ""   # JSON-encoded parameters
    binary: sequence[uint8] = field(default_factory=list)  # Binary data (usually empty)


# ============================================================================
# DDS Message Types for Go2 Robot Commands
# ============================================================================
# These dataclasses define the message structures for communicating with the
# Go2 robot via raw CycloneDDS (without ROS2 middleware).
# 
# The robot expects messages on topic "rt/api/sport/request" with type
# "unitree_go::msg::dds_::SportModeCmd_"
# ============================================================================

@dataclass
class BmsCmd(IdlStruct, typename="unitree_go::msg::dds_::BmsCmd_"):
    """Battery Management System command."""
    off: int = 0          # uint8 - turn off battery
    reserve: array[int, 3] = None  # uint8[3] - reserved bytes
    
    def __post_init__(self):
        if self.reserve is None:
            self.reserve = [0, 0, 0]


@dataclass
class PathPoint(IdlStruct, typename="unitree_go::msg::dds_::PathPoint_"):
    """Path point for trajectory planning."""
    t_from_start: float = 0.0  # float32 - time from start (t_from_start in C++)
    x: float = 0.0             # float32 - x position
    y: float = 0.0             # float32 - y position
    yaw: float = 0.0           # float32 - yaw angle
    vx: float = 0.0            # float32 - x velocity
    vy: float = 0.0            # float32 - y velocity
    vyaw: float = 0.0          # float32 - yaw velocity


@dataclass
class SportModeCmd(IdlStruct, typename="unitree_go::msg::dds_::SportModeCmd_"):
    """
    Sport mode command message for Go2 robot.
    
    This is the main command message for controlling the robot's movement,
    gait, and behavior. Published to "rt/api/sport/request" topic.
    
    Mode values:
        0 - Idle
        1 - Stand up (balance stand)
        2 - Walking/moving
        5 - Stand down (sit/lie down)
        6 - Damping mode
        
    Gait types (when mode=2):
        0 - Idle
        1 - Trot
        2 - Trot running
        3 - Forward climbing
        4 - Reverse climbing
    """
    mode: int = 0                    # uint8 - robot mode
    gait_type: int = 0               # uint8 - gait type
    speed_level: int = 0             # uint8 - speed level (0-2)
    foot_raise_height: float = 0.0   # float32 - foot raise height
    body_height: float = 0.0         # float32 - body height
    position: array[float, 2] = None  # float32[2] - target position [x, y]
    euler: array[float, 3] = None     # float32[3] - euler angles [roll, pitch, yaw]
    velocity: array[float, 2] = None  # float32[2] - velocity [vx, vy]
    yaw_speed: float = 0.0           # float32 - yaw angular velocity
    bms_cmd: BmsCmd = None           # BmsCmd - battery management command
    path_point: array[PathPoint, 30] = None  # PathPoint[30] - fixed array of path points
    
    def __post_init__(self):
        if self.position is None:
            self.position = [0.0, 0.0]
        if self.euler is None:
            self.euler = [0.0, 0.0, 0.0]
        if self.velocity is None:
            self.velocity = [0.0, 0.0]
        if self.bms_cmd is None:
            self.bms_cmd = BmsCmd()
        if self.path_point is None:
            self.path_point = [PathPoint() for _ in range(30)]


class DDSBridge:
    """
    DDS Bridge that subscribes to raw DDS topics using dynamic type discovery.
    
    This bridge uses the cyclonedds Python library with XTypes to dynamically
    discover and subscribe to the robot's native DDS topics.
    """

    def __init__(self, domain_id: int = 0, discovery_timeout: float = 2.0):
        """
        Initialize DDS bridge.
        
        Args:
            domain_id: DDS domain ID (default 0, same as ROS 2)
            discovery_timeout: Timeout for type discovery in seconds
        """
        if not CYCLONEDDS_AVAILABLE:
            raise ImportError(
                "cyclonedds Python library not available. "
                "Install with: pip install cyclonedds"
            )

        self.domain_id = domain_id
        self.discovery_timeout = discovery_timeout
        self.participant: Optional[DomainParticipant] = None
        self.readers: Dict[str, DataReader] = {}
        self.writers: Dict[str, DataWriter] = {}  # For publishing commands
        self.topics: Dict[str, Topic] = {}  # Cache topics for reuse
        self.discovered_types: Dict[str, Any] = {}  # topic_name -> datatype
        self.callbacks: Dict[str, Callable] = {}
        self.polling_threads: Dict[str, threading.Thread] = {}
        self.active = False

        logger.info(f"DDS Bridge initialized for domain {domain_id}")

    def start(self):
        """Start the DDS bridge (create participant)."""
        if self.participant is None:
            self.participant = DomainParticipant(self.domain_id)
            self.active = True
            logger.info("DDS Participant created")

    def stop(self):
        """Stop the DDS bridge and cleanup resources."""
        self.active = False

        # Stop all polling threads
        for thread in self.polling_threads.values():
            if thread.is_alive():
                thread.join(timeout=1.0)

        # Cleanup writers
        for writer in self.writers.values():
            del writer
        self.writers.clear()

        # Cleanup readers
        for reader in self.readers.values():
            del reader
        self.readers.clear()

        # Cleanup topics
        self.topics.clear()

        # Cleanup participant
        if self.participant:
            del self.participant
            self.participant = None

        logger.info("DDS Bridge stopped")

    def _discover_type_for_topic(self, topic_name: str) -> Optional[Any]:
        """
        Dynamically discover the type for a given topic using XTypes.
        
        Args:
            topic_name: The DDS topic name
            
        Returns:
            The discovered datatype class, or None if not found
        """
        if not self.participant:
            raise RuntimeError("DDS Bridge not started. Call start() first.")

        logger.info(f"Discovering type for topic: {topic_name}")

        # Create builtin reader for publication discovery
        rdw = BuiltinDataReader(self.participant, BuiltinTopicDcpsPublication)
        rcw = ReadCondition(
            rdw, SampleState.NotRead | ViewState.Any | InstanceState.Alive
        )

        type_ids = set()
        start = time.time()
        end = start + self.discovery_timeout

        # Discovery phase
        while time.time() < end:
            for pub in rdw.take(N=20, condition=rcw):
                if pub.topic_name == topic_name and pub.type_id is not None:
                    logger.info(f"Found type_id for topic {topic_name}")
                    type_ids.add(pub.type_id)
            time.sleep(0.01)

        if not type_ids:
            logger.error(f"No type IDs discovered for topic {topic_name}")
            return None

        # Get the dynamic type from the first type_id
        for type_id in type_ids:
            logger.info(f"Fetching type definition for type_id: {type_id}")
            try:
                datatype, _ = get_types_for_typeid(
                    self.participant, type_id, duration(seconds=self.discovery_timeout)
                )
                logger.info(f"Successfully discovered type: {datatype}")
                return datatype
            except Exception as e:
                logger.error(f"Failed to get type for type_id {type_id}: {e}")
                continue

        return None

    def subscribe_lowstate(
        self, 
        callback: Callable[[Dict[str, Any]], None],
        topic_name: str = "rt/lf/lowstate"
    ):
        """
        Subscribe to LowState topic using dynamic type discovery.
        
        Args:
            callback: Function called when data is received
            topic_name: DDS topic name (default: rt/lf/lowstate)
        """
        self.subscribe_topic(callback, topic_name, "lowstate")

    def subscribe_sportmodestate(
        self,
        callback: Callable[[Dict[str, Any]], None],
        topic_name: str = "rt/sportmodestate"
    ):
        """
        Subscribe to SportModeState topic using dynamic type discovery.
        
        Args:
            callback: Function called when data is received
            topic_name: DDS topic name (default: rt/sportmodestate)
        """
        self.subscribe_topic(callback, topic_name, "sportmodestate")

    def subscribe_topic(
        self,
        callback: Callable[[Dict[str, Any]], None],
        topic_name: str,
        msg_type: str = "generic"
    ):
        """
        Generic method to subscribe to any DDS topic using dynamic type discovery.
        
        Args:
            callback: Function called when data is received
            topic_name: DDS topic name (e.g., 'rt/utlidar/cloud')
            msg_type: Message type name for logging (e.g., 'pointcloud')
        """
        if not self.participant:
            raise RuntimeError("DDS Bridge not started. Call start() first.")

        try:
            # Discover type if not already cached
            if topic_name not in self.discovered_types:
                datatype = self._discover_type_for_topic(topic_name)
                if datatype is None:
                    raise RuntimeError(f"Failed to discover type for topic {topic_name}")
                self.discovered_types[topic_name] = datatype
            else:
                datatype = self.discovered_types[topic_name]

            # Create topic and reader with discovered type
            topic = Topic(self.participant, topic_name, datatype)
            reader = DataReader(self.participant, topic)

            self.readers[topic_name] = reader
            self.callbacks[topic_name] = callback

            # Start polling thread
            thread = threading.Thread(
                target=self._poll_generic,
                args=(reader, callback, topic_name, msg_type),
                daemon=True
            )
            thread.start()
            self.polling_threads[topic_name] = thread

            logger.info(f"Subscribed to DDS topic: {topic_name} (type: {msg_type})")

        except Exception as e:
            logger.error(f"Failed to subscribe to {topic_name}: {e}")
            raise

    def _poll_generic(
        self, 
        reader: DataReader, 
        callback: Callable, 
        topic_name: str,
        msg_type: str
    ):
        """Generic polling thread for any dynamically discovered type."""
        logger.info(f"Started polling thread for {topic_name}")
        
        while self.active:
            try:
                # Read with timeout
                samples = reader.take(N=10)
                
                for sample in samples:
                    if sample is None:
                        continue

                    # Convert DDS message to dictionary
                    data = self._dynamic_to_dict(sample, topic_name, msg_type)
                    
                    # Call the callback
                    callback(data)

            except Exception as e:
                logger.error(f"Error polling {topic_name}: {e}")
                
            # Small sleep to avoid busy-waiting
            threading.Event().wait(0.001)  # 1ms

        logger.info(f"Stopped polling thread for {topic_name}")

    def _dynamic_to_dict(self, msg: Any, topic_name: str, msg_type: str) -> Dict[str, Any]:
        """
        Convert dynamically discovered DDS message to dictionary format.
        
        This uses Python's object introspection to handle any structure.
        """
        try:
            def to_python_types(obj):
                """Recursively convert DDS objects to Python types."""
                # Handle None
                if obj is None:
                    return None
                    
                # Handle bytes
                if isinstance(obj, (bytes, bytearray)):
                    return obj
                    
                # Handle primitives
                if isinstance(obj, (int, float, str, bool)):
                    return obj
                    
                # Handle lists/sequences
                if isinstance(obj, (list, tuple)):
                    return [to_python_types(item) for item in obj]
                    
                # Handle DDS structures (have __dataclass_fields__)
                if hasattr(obj, '__dataclass_fields__'):
                    result = {}
                    for field_name in obj.__dataclass_fields__:
                        value = getattr(obj, field_name)
                        result[field_name] = to_python_types(value)
                    return result
                    
                # Fallback - try to convert to string
                return str(obj)

            data_dict = to_python_types(msg)
            
            return {
                "topic": topic_name,
                "type": msg_type,
                "data": data_dict
            }

        except Exception as e:
            logger.error(f"Error converting {msg_type} to dict: {e}")
            logger.exception(e)

    # ========================================================================
    # Publishing Methods - Send commands to robot via RPC Request messages
    # ========================================================================
    # The Go2 robot uses an RPC pattern where commands are sent as Request_
    # messages with a header containing the API ID and JSON parameters.
    # ========================================================================

    def _get_request_id(self) -> int:
        """Generate a unique request ID using nanosecond timestamp."""
        return time.monotonic_ns()

    def create_request_writer(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Create a DataWriter for publishing RPC Request messages.
        
        Args:
            topic_name: DDS topic name (default: rt/api/sport/request)
            
        Returns:
            True if writer was created successfully
        """
        if not self.participant:
            raise RuntimeError("DDS Bridge not started. Call start() first.")
        
        if topic_name in self.writers:
            logger.debug(f"Writer for {topic_name} already exists")
            return True
        
        try:
            # Create QoS for reliable publishing
            qos = Qos(
                Policy.Reliability.Reliable(duration(seconds=1)),
                Policy.History.KeepLast(10),
                Policy.Durability.Volatile,
            )
            
            # Create topic with RPC Request type
            topic = Topic(self.participant, topic_name, Request, qos=qos)
            self.topics[topic_name] = topic
            
            # Create writer
            writer = DataWriter(self.participant, topic, qos=qos)
            self.writers[topic_name] = writer
            
            logger.info(f"Created DDS RPC writer for topic: {topic_name}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to create writer for {topic_name}: {e}")
            return False

    def _send_rpc_request(
        self,
        api_id: int,
        parameter: dict,
        topic_name: str = "rt/api/sport/request",
        noreply: bool = True
    ) -> bool:
        """
        Send an RPC request to the robot.
        
        Args:
            api_id: API function ID (e.g., SPORT_API_ID_MOVE = 1008)
            parameter: Dictionary of parameters (will be JSON-encoded)
            topic_name: DDS topic to publish to
            noreply: If True, don't wait for response
            
        Returns:
            True if request was sent successfully
        """
        # Ensure writer exists
        if topic_name not in self.writers:
            if not self.create_request_writer(topic_name):
                return False
        
        try:
            # Create request header
            identity = RequestIdentity(
                id=self._get_request_id(),
                api_id=api_id
            )
            lease = RequestLease(id=0)
            policy = RequestPolicy(priority=0, noreply=noreply)
            header = RequestHeader(
                identity=identity,
                lease=lease,
                policy=policy
            )
            
            # Create request message with JSON-encoded parameters
            request = Request(
                header=header,
                parameter=json.dumps(parameter),
                binary=[]
            )
            
            # Publish the request
            self.writers[topic_name].write(request)
            
            logger.info(
                f"Sent RPC request to {topic_name}: api_id={api_id}, params={parameter}"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to send RPC request: {e}")
            return False

    def publish_movement(
        self, 
        x: float, 
        y: float, 
        yaw: float,
        topic_name: str = "rt/api/sport/request"
    ) -> bool:
        """
        Publish a movement command using RPC API.
        
        Args:
            x: Forward velocity (m/s), range [-2.5, 3.8]
            y: Lateral velocity (m/s), range [-1.0, 1.0]
            yaw: Yaw angular velocity (rad/s), range [-4, 4]
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        # Move command uses API ID 1008 with {"x": vx, "y": vy, "z": vyaw}
        return self._send_rpc_request(
            api_id=SPORT_API_ID_MOVE,
            parameter={"x": x, "y": y, "z": yaw},
            topic_name=topic_name,
            noreply=True  # Move commands don't need reply
        )

    def publish_stand_up(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a stand up command using RPC API.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_STANDUP,
            parameter={},
            topic_name=topic_name,
            noreply=False  # Wait for acknowledgment
        )

    def publish_stand_down(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a stand down command using RPC API.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_STANDDOWN,
            parameter={},
            topic_name=topic_name,
            noreply=False  # Wait for acknowledgment
        )

    def publish_recovery_stand(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a recovery stand command using RPC API.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_RECOVERYSTAND,
            parameter={},
            topic_name=topic_name,
            noreply=False
        )

    def publish_balance_stand(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a balance stand command using RPC API.
        Switches to balanced standing mode where the robot maintains balance.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_BALANCESTAND,
            parameter={},
            topic_name=topic_name,
            noreply=False
        )

    def publish_stop_move(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a stop move command using RPC API.
        Stops current motion and resets internal parameters to defaults.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_STOPMOVE,
            parameter={},
            topic_name=topic_name,
            noreply=False
        )

    def publish_damp(self, topic_name: str = "rt/api/sport/request") -> bool:
        """
        Publish a damping mode command using RPC API.
        All motors enter damping state - use for emergency stops.
        
        Args:
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_DAMP,
            parameter={},
            topic_name=topic_name,
            noreply=False
        )

    def publish_euler(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        topic_name: str = "rt/api/sport/request"
    ) -> bool:
        """
        Publish an euler angle (posture) command using RPC API.
        
        Args:
            roll: Roll angle (rad), range [-0.75, 0.75]
            pitch: Pitch angle (rad), range [-0.75, 0.75]
            yaw: Yaw angle (rad), range [-0.6, 0.6]
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_EULER,
            parameter={"x": roll, "y": pitch, "z": yaw},
            topic_name=topic_name,
            noreply=False
        )

    def publish_speed_level(
        self,
        level: int,
        topic_name: str = "rt/api/sport/request"
    ) -> bool:
        """
        Publish a speed level command using RPC API.
        
        Args:
            level: Speed level (-1=slow, 0=normal, 1=fast)
            topic_name: DDS topic to publish to
            
        Returns:
            True if message was published successfully
        """
        return self._send_rpc_request(
            api_id=SPORT_API_ID_SPEEDLEVEL,
            parameter={"data": level},
            topic_name=topic_name,
            noreply=False
        )

    # Legacy method for backward compatibility - redirects to RPC format
    def create_sport_cmd_writer(self, topic_name: str = "rt/api/sport/request") -> bool:
        """Legacy method - now uses RPC Request format."""
        return self.create_request_writer(topic_name)

    def publish_sport_cmd(
        self,
        mode: int = 0,
        gait_type: int = 0,
        speed_level: int = 0,
        velocity_x: float = 0.0,
        velocity_y: float = 0.0,
        yaw_speed: float = 0.0,
        foot_raise_height: float = 0.0,
        body_height: float = 0.0,
        topic_name: str = "rt/api/sport/request"
    ) -> bool:
        """
        Legacy method for publishing sport commands.
        Now uses RPC API calls based on mode.
        
        For movement, use publish_movement() directly for better results.
        """
        if mode == 2 and (velocity_x != 0.0 or velocity_y != 0.0 or yaw_speed != 0.0):
            # Walk mode with velocity - use Move API
            return self.publish_movement(velocity_x, velocity_y, yaw_speed, topic_name)
        elif mode == 1:
            # Stand up mode
            return self.publish_stand_up(topic_name)
        elif mode == 5:
            # Stand down mode
            return self.publish_stand_down(topic_name)
        elif mode == 6:
            # Damping mode
            return self.publish_damp(topic_name)
        else:
            # Default to stop move
            return self.publish_stop_move(topic_name)