# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
CycloneDDS Launch File for Go2 Pro Robot (Ethernet Connection)

This launch file sets up the ROS2 environment to use CycloneDDS as the
RMW (ROS Middleware) implementation for communicating with the Go2 Pro
robot over Ethernet.

Prerequisites:
1. Your PC must be connected to the Go2 Pro via Ethernet
2. Your PC needs an IP address in the 192.168.123.x subnet
   Example: sudo ip addr add 192.168.123.100/24 dev eth0
3. The Go2 Pro robot should be at 192.168.123.161 (default)

Usage:
    # Set environment variables first
    export ROBOT_IP="192.168.123.161"
    export CONN_TYPE="cyclonedds"
    
    # Launch the driver
    ros2 launch go2_robot_sdk cyclonedds.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def setup_cyclonedds_env(context, *args, **kwargs):
    """Setup CycloneDDS environment variables and return nodes"""
    
    package_dir = get_package_share_directory('go2_robot_sdk')
    cyclonedds_config = os.path.join(package_dir, 'config', 'cyclonedds.xml')
    
    robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    
    # Get launch configurations
    enable_video = LaunchConfiguration('enable_video').perform(context) == 'true'
    decode_lidar = LaunchConfiguration('decode_lidar').perform(context) == 'true'
    publish_raw_voxel = LaunchConfiguration('publish_raw_voxel').perform(context) == 'true'
    
    print("=" * 60)
    print("🤖 Go2 Pro CycloneDDS Configuration")
    print("=" * 60)
    print(f"   Robot IP          : {robot_ip}")
    print(f"   Connection Type   : cyclonedds (Ethernet)")
    print(f"   CycloneDDS Config : {cyclonedds_config}")
    print(f"   Enable Video      : {enable_video}")
    print(f"   Decode LiDAR      : {decode_lidar}")
    print("=" * 60)
    print("")
    print("📋 Network Checklist:")
    print("   [ ] PC Ethernet connected to Go2 Pro")
    print("   [ ] PC has IP in 192.168.123.x subnet")
    print("   [ ] Can ping 192.168.123.161")
    print("=" * 60)
    
    return [
        # Main Go2 driver node with CycloneDDS
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': 'cyclonedds',
                'enable_video': enable_video,
                'decode_lidar': decode_lidar,
                'publish_raw_voxel': publish_raw_voxel,
            }],
        ),
    ]


def generate_launch_description():
    """Generate the CycloneDDS launch description for Go2 Pro robot"""
    
    package_dir = get_package_share_directory('go2_robot_sdk')
    cyclonedds_config = os.path.join(package_dir, 'config', 'cyclonedds.xml')
    
    return LaunchDescription([
        # Set CycloneDDS as the RMW implementation
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        
        # Set the CycloneDDS configuration file
        SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclonedds_config}'),
        
        # Force CycloneDDS to use the correct network interface
        # This helps when multiple network interfaces are present
        SetEnvironmentVariable('CONN_TYPE', 'cyclonedds'),
        
        # Launch arguments
        DeclareLaunchArgument(
            'enable_video',
            default_value='false',
            description='Enable video streaming (not supported over CycloneDDS)'
        ),
        DeclareLaunchArgument(
            'decode_lidar',
            default_value='true',
            description='Enable LiDAR point cloud decoding'
        ),
        DeclareLaunchArgument(
            'publish_raw_voxel',
            default_value='false',
            description='Publish raw voxel map data'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        
        # Setup and launch nodes
        OpaqueFunction(function=setup_cyclonedds_env),
        
        # Optional RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='go2_rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'config', 'cyclonedds_config.rviz')],
        ),
    ])
