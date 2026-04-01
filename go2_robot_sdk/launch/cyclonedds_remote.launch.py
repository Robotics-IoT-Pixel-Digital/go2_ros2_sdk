import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


class Go2LaunchConfig:
    
    def __init__(self):
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
        self.conn_type = os.getenv('CONN_TYPE', 'cyclonedds')
        self.conn_mode = "single"
        
        self.go2_package_dir = get_package_share_directory('go2_robot_sdk')
        self.aggregator_package_dir = get_package_share_directory('pointcloud2_aggregator')
        self.config_paths = self._get_config_paths()
        
        print(f"        Go2 Launch Configuration") 
        print(f"        Robot IP    : {self.robot_ip}")
        print(f"        Connection  : {self.conn_type} ({self.conn_mode})")
    
    def _get_config_paths(self) -> dict:
        return {
            'joystick': os.path.join(self.go2_package_dir, 'config', 'joystick.yaml'),
            'twistmux': os.path.join(self.go2_package_dir, 'config', 'twist_mux.yaml'),
            'cyclonedds': os.path.join(self.go2_package_dir, 'config', 'cyclonedds.xml'),
        }
    

class Go2NodeFactory:
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        with open(urdf_path, 'r') as file:
            return file.read()

    def create_core_nodes(self) -> List[Node]:       
        return [
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='screen',
                parameters=[{
                    'token': self.config.robot_token,
                    'robot_ip': self.config.robot_ip,
                    'conn_type': self.config.conn_type,
                    'enable_video': False,
                    'decode_lidar': False,
                }]
            ),
        ]

    def create_teleop_nodes(self) -> List[Node]:
        return [
            Node(
                package='joy',
                executable='joy_node',
                parameters=[self.config.config_paths['joystick']],
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='go2_teleop_node',
                parameters=[self.config.config_paths['twistmux']],
                remappings=[('cmd_vel', 'cmd_vel_joy')], 
            ),
            Node(
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                parameters=[self.config.config_paths['twistmux']],
            ),
        ]
    

def generate_launch_description():

    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)

    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()

    print(f"🔧 Setting up CycloneDDS environment")
    print(f"   Config file: {config.config_paths['cyclonedds']}")
    env_setup = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', f"file://{config.config_paths['cyclonedds']}"),
    ]
    
    launch_entities = (
        env_setup +
        core_nodes +
        teleop_nodes 
    )
    
    return LaunchDescription(launch_entities)