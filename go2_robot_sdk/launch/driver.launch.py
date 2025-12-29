import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_ip = os.getenv('ROBOT_IP', '')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    print("🎮 Go2 Driver Launch Configuration 🎮")
    print(f"Robot IP     : {robot_ip}")
    print(f"Conn Type    : {conn_type}")

    return LaunchDescription([

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
        ),

    ])