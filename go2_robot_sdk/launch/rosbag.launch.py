from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rosbag_dir_arg = DeclareLaunchArgument(
        'rosbag_dir',
        default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/rosbag2_record_1/',
        description='Directory of rosbag to play'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/config/mapper_params_online_async_cyclonedds.yaml',
        description='SLAM Toolbox parameter file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/config/rosbag.rviz',
        description='RViz config file (empty = default RViz)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rosbag_dir = LaunchConfiguration('rosbag_dir')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            rosbag_dir,
            '--topics',
            '/tf',
            '/tf_static',
            '/odom',
            '/scan'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # slam_toolbox_launch = os.path.join(
    #     get_package_share_directory('slam_toolbox'),
    #     'launch',
    #     'offline_launch.py'
    # )

    # slam_toolbox = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_toolbox_launch)
    # )


    return LaunchDescription([
        rosbag_dir_arg,
        slam_params_arg,
        rviz_config_arg,
        use_sim_time_arg,

        rosbag_play,
        rviz_node,
        slam_toolbox
    ])
