import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    robot_ip = os.getenv('ROBOT_IP', '')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    
    package_dir = get_package_share_directory('go2_robot_sdk')
    cyclonedds_config = os.path.join(package_dir, 'config', 'cyclonedds.xml')

    print("🎮 Go2 Driver Launch Configuration 🎮")
    print(f"Robot IP     : {robot_ip}")
    print(f"Conn Type    : {conn_type}")
    
    # Build list of launch actions
    launch_actions = []
    
    # If using CycloneDDS, set the required environment variables
    if conn_type == 'cyclonedds':
        print(f"CycloneDDS   : {cyclonedds_config}")
        launch_actions.extend([
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
            SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclonedds_config}'),
        ])

    # Add the driver node
    launch_actions.append(
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': conn_type,
                'enable_video': False,
                'decode_lidar': True,
            }],
        )
    )

    return LaunchDescription(launch_actions)