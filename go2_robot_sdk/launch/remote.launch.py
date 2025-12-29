import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


class Go2RemoteLaunchConfig:

    def __init__(self):

        self.pkg_share = get_package_share_directory('go2_robot_sdk')
        self.joystick_yaml = os.path.join(self.pkg_share, 'config', 'joystick.yaml')
        self.twist_mux_yaml = os.path.join(self.pkg_share, 'config', 'twist_mux.yaml')
        
        print("🎮 Go2 Remote Launch Configuration 🎮")

class Go2RemoteNodeFactory:

    def __init__(self, config: Go2RemoteLaunchConfig):
        self.config = config

    def create_joystick_node(self) -> Node:
        return Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[self.config.joystick_yaml],
        )

    def create_teleop_node(self) -> Node:
        return Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='go2_teleop_node',
            output='screen',
            parameters=[self.config.twist_mux_yaml],
        )

    def create_twist_mux_node(self) -> Node:
        return Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[self.config.twist_mux_yaml],
        )

    def create_all_nodes(self):
        return [
            self.create_joystick_node(),
            self.create_teleop_node(),
            self.create_twist_mux_node(),
        ]


def generate_launch_description():

    config = Go2RemoteLaunchConfig()
    factory = Go2RemoteNodeFactory(config)

    return LaunchDescription(
        factory.create_all_nodes()
    )
