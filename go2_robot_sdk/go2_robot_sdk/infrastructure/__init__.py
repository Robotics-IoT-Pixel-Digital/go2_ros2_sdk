"""
Infrastructure layer - adapters for external systems
"""
from .ros2 import ROS2Publisher
from .cyclonedds import CycloneDDSAdapter
from .sensors import load_camera_info, decode_lidar_data

# Lazy import for WebRTCAdapter to avoid requiring aiortc when using CycloneDDS
WebRTCAdapter = None

def get_webrtc_adapter():
    """Lazily import WebRTCAdapter when needed."""
    global WebRTCAdapter
    if WebRTCAdapter is None:
        from .webrtc import WebRTCAdapter as _WebRTCAdapter
        WebRTCAdapter = _WebRTCAdapter
    return WebRTCAdapter

__all__ = ['ROS2Publisher', 'WebRTCAdapter', 'CycloneDDSAdapter', 'load_camera_info', 'decode_lidar_data', 'get_webrtc_adapter'] 