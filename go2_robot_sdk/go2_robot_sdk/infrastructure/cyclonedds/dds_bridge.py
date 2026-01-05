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
4. Republishes as ROS2 topics for the rest of the system
"""

import logging
import time
import threading
from typing import Callable, Dict, Any, Optional
from datetime import timedelta

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.util import duration
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    from cyclonedds.core import ReadCondition, SampleState, ViewState, InstanceState
    from cyclonedds.dynamic import get_types_for_typeid
    CYCLONEDDS_AVAILABLE = True
except ImportError:
    CYCLONEDDS_AVAILABLE = False
    logging.warning("cyclonedds Python library not available. Install with: pip install cyclonedds")

logger = logging.getLogger(__name__)


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

        # Cleanup readers
        for reader in self.readers.values():
            del reader

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
