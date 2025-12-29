import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('go2_robot_sdk')

    map_name = os.getenv('MAP_NAME', '3d_map')
    save_map = os.getenv('MAP_SAVE', 'true')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'navigation.rviz')
    urdf_config = os.path.join(pkg_share, 'urdf', 'go2.urdf')
    twist_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    joy_config = os.path.join(pkg_share, 'config', 'joystick.yaml')

    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file (REQUIRED)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    robot_ip = os.getenv('ROBOT_IP', '')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    print("🎮 Go2 Driver Launch Configuration 🎮")
    print(f"Robot IP     : {robot_ip}")
    print(f"Conn Type    : {conn_type}")

    robot_state = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                }],
            arguments=[urdf_config],
        )
    
    point_to_cloud = Node(
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
            )
    
    lidar_scan = Node(
            package='lidar_processor',
            executable='lidar_to_pointcloud',
            name='lidar_to_pointcloud',
            parameters=[{
                'map_name': map_name,
                'map_save': save_map
            }],
        )
            
    lidar_process = Node(
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
            )

    driver = Node(
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
            # 'obstacle_avoidance': True,
        }],
    )

    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_config],
        )

    joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config],
        )

    teleop = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='go2_teleop_node',
            output='screen',
            parameters=[twist_config],
        )
    
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'yaml_filename': map_yaml,
    #         'use_sim_time': use_sim_time
    #     }]
    # )

    # map_lifecycle = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'node_names': ['map_server']
    #     }]
    # )

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'localization_launch.py')]),
            launch_arguments={
                'map': map_yaml,
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items(),
        )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py')]),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items(),
        )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        driver,
        joy,
        teleop,
        twist_mux,
        robot_state,
        point_to_cloud,
        lidar_scan,
        lidar_process,
        # map_server,
        # map_lifecycle,
        nav2,
        nav2_localization,
        rviz
    ])
