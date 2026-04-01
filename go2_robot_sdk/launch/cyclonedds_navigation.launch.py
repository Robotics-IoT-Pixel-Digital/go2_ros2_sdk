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
            'rviz': os.path.join(self.go2_package_dir, 'config', 'navigation.rviz'),
            'urdf': os.path.join(self.go2_package_dir, 'urdf', 'go2.urdf'),
            'cyclonedds': os.path.join(self.go2_package_dir, 'config', 'cyclonedds.xml'),
            'aggregator': os.path.join(self.aggregator_package_dir, 'config', 'aggregator.yaml'),
            'nav2': os.path.join(self.go2_package_dir, 'config', 'params_navigation.yaml'),
            'keepout': os.path.join(self.go2_package_dir, 'config', 'params_keepout.yaml'),
        }
    

class Go2NodeFactory:
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        with open(urdf_path, 'r') as file:
            return file.read()
        
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        return [
            DeclareLaunchArgument(
                'map', 
                default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/maps/Studio.yaml',
                description='Absolute path to the map yaml formatted file'
            ),
            DeclareLaunchArgument(
                'keepout_mask', 
                default_value='false',
                description='Enable/disable keepout mask[boolean]'
            ),
            DeclareLaunchArgument(
                'keepout_map', 
                default_value='/home/ubuntu/Downloads/LOBBY1_keepout(1).yaml',
                description='Absolute path to the keepout mask map yaml file'
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Enable/disable RViz Visualization [boolean]'
            ),
            DeclareLaunchArgument(
                'remote',
                default_value='true',
                description='Enable/disable remote control [boolean]'
            ),
        ]
        
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
        
    def create_state_nodes(self) -> List[Node]:
        robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='go2_robot_state_publisher',
                # output='screen',
                parameters=[{'robot_description': robot_desc}],
                arguments=[self.config.config_paths['urdf']]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='utlidar_lidar_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'radar', 'utlidar_lidar']
            ),
        ]
    
    def create_laserscan_nodes(self) -> List[Node]:
        cloud_topic = 'utlidar/cloud_deskewed_aggregated'

        return [
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', cloud_topic),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'max_height': 0.8,
                    'angle_increment': 0.0087,
                    'scan_time': 0.1,
                    'range_min': 0.1,
                    'range_max': 15.0,
                    'use_inf': True,
                    # 'qos_overrides./cloud_in.subscription.reliability': 'best_effort',
                    # 'qos_overrides./scan.publisher.reliability': 'reliable',
                }],
                output='screen',
            ),
        ]
    
    def create_aggregator_nodes(self) -> List[Node]:
        return [
            Node(
                package='pointcloud2_aggregator',
                namespace="aggregator",
                executable="aggregator",
                name='go2_pointcloud2_aggregator',
                parameters=[self.config.config_paths['aggregator']],
                # output='screen',
            ),
        ]

    def create_teleop_nodes(self) -> List[Node]:
        return [
            Node(
                package='joy',
                executable='joy_node',
                parameters=[self.config.config_paths['joystick']],
                condition=IfCondition(LaunchConfiguration('remote')),
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='go2_teleop_node',
                parameters=[self.config.config_paths['twistmux']],
                condition=IfCondition(LaunchConfiguration('remote')),
                remappings=[('cmd_vel', 'cmd_vel_joy')], 
            ),
            Node(
                package='twist_mux',
                executable='twist_mux',
                # output='screen',
                parameters=[self.config.config_paths['twistmux']],
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                name='go2_rviz2',
                output='screen',
                arguments=['-d', self.config.config_paths['rviz']],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': False}]
            ),
        ]
    
    def create_camera_nodes(self) -> List[Node]:
        return [
            Node(
                package='go2_robot_sdk',
                executable='go2_gstreamer_jetson_node',
                name='go2_gstreamer_jetson_node'
            ),
        ]
    
    def create_keepout_nodes(self) -> List[Node]:
        return [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                condition=IfCondition(LaunchConfiguration('keepout_mask')),
                parameters=[
                    {'yaml_filename': LaunchConfiguration('keepout_map')},
                    self.config.config_paths['keepout'],
                ],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                condition=IfCondition(LaunchConfiguration('keepout_mask')),
                parameters=[self.config.config_paths['keepout']],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                condition=IfCondition(LaunchConfiguration('keepout_mask')),
                parameters=[
                    {'use_sim_time': False}, {'autostart': True},
                    {'node_names': ['keepout_filter_mask_server', 
                                    'keepout_costmap_filter_info_server']}
                ],
            ),
        ]

    def create_nav2_launches(self) -> List[IncludeLaunchDescription]:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ]),
                launch_arguments={
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': 'false',
                }.items(),
            ),
        ]
    
    def create_localization_launches(self) -> List[IncludeLaunchDescription]:
        map_file = LaunchConfiguration('map')

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'localization_launch.py')
                ]),
                launch_arguments={
                    'map': map_file,
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': 'false',
                }.items(),
            ),
        ]
    

def generate_launch_description():

    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)

    launch_args = factory.create_launch_arguments()
    core_nodes = factory.create_core_nodes()
    state_nodes = factory.create_state_nodes()
    aggregate_nodes = factory.create_aggregator_nodes()
    laserscan_nodes = factory.create_laserscan_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    camera_nodes = factory.create_camera_nodes()
    keepout_nodes = factory.create_keepout_nodes()
    nav2_launches = factory.create_nav2_launches()
    localization_launches = factory.create_localization_launches()  

    print(f"🔧 Setting up CycloneDDS environment 🔧")
    print(f"   Config file: {config.config_paths['cyclonedds']}")

    env_setup = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', f"file://{config.config_paths['cyclonedds']}"),
    ]
    
    launch_entities = (
        env_setup +
        launch_args +
        core_nodes +
        state_nodes +
        aggregate_nodes+
        laserscan_nodes +
        teleop_nodes +
        visualization_nodes + 
        camera_nodes +
        keepout_nodes +
        nav2_launches + 
        localization_launches
    )
    
    return LaunchDescription(launch_entities)