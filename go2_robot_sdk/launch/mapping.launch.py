import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


class Go2MappingLaunchConfig:
    
    def __init__(self):
        self.map_name = os.getenv('MAP_NAME', '3d_map')
        self.save_map = os.getenv('MAP_SAVE', 'true')
        self.rviz_config = "single_robot_conf.rviz"
        self.urdf_file = "go2.urdf"
        
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"🎮 Go2 Mapping Launch Configuration 🎮")
        print(f"   URDF: {self.urdf_file}")
    
    def _get_config_paths(self) -> dict:
        return {
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
            'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file),
        }


class Go2MappingNodeFactory:
    
    def __init__(self, config: Go2MappingLaunchConfig):
        self.config = config
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        with open(urdf_path, 'r') as file:
            return file.read()

    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        return [
            DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('slam', default_value='true', description='Launch SLAM'),
        ]
    
    def create_robot_state_nodes(self) -> List[Node]:
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])
            
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='go2_robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_desc
                }],
                arguments=[self.config.config_paths['urdf']]
            ),
        ]
    
    def create_pointcloud_to_laserscan_node(self) -> List[Node]:
        return [    
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='go2_pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'max_height': 0.5
                }],
                output='screen',
            ),
        ]
    
    def create_lidar_nodes(self) -> List[Node]:
        return [
            Node(
                package='lidar_processor',
                executable='lidar_to_pointcloud',
                name='lidar_to_pointcloud',
                parameters=[{
                    'map_name': self.config.map_name,
                    'map_save': self.config.save_map
                }],
            ),
            Node(
                package='lidar_processor',
                executable='pointcloud_aggregator',
                name='pointcloud_aggregator',
                parameters=[{
                    'max_range': 20.0,
                    'min_range': 0.1,
                    'height_filter_min': -2.0,
                    'height_filter_max': 3.0,
                    'downsample_rate': 5,
                    'publish_rate': 10.0
                }],
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(with_rviz2),
                name='go2_rviz2',
                output='screen',
                arguments=['-d', self.config.config_paths['rviz']],
                parameters=[{'use_sim_time': False}]
            ),
        ]
    
    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_slam = LaunchConfiguration('slam', default='true')
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=IfCondition(with_slam),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]

def generate_launch_description():
    config = Go2MappingLaunchConfig()
    factory = Go2MappingNodeFactory(config)
    
    launch_args = factory.create_launch_arguments()
    robot_state_nodes = factory.create_robot_state_nodes()
    laserscan_nodes = factory.create_pointcloud_to_laserscan_node()
    lidar_nodes = factory.create_lidar_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    include_launches = factory.create_include_launches()
    
    launch_entities = (
        launch_args +
        robot_state_nodes +
        laserscan_nodes +
        lidar_nodes +
        visualization_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)