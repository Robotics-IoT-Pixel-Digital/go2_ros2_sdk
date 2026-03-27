from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


class Go2Rosbag:

    def __init__(self):
        self.rosbag_dir = LaunchConfiguration('rosbag_dir')
        self.play_topics = LaunchConfiguration('play_topics')
        self.slam_params = LaunchConfiguration('slam_params')
        self.use_sim_time = LaunchConfiguration('use_sim_time')


    def arguments(self):
        return [
            DeclareLaunchArgument(
                'rosbag_dir',
                default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/record/lobby_test',
                description='Directory of rosbag file to play'
            ),
            DeclareLaunchArgument(
                'play_topics',
                default_value='',
                description='Topics to play (space-separated). Default play all'
            ),
            DeclareLaunchArgument(
                'slam_params',
                default_value='/home/ubuntu/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/config/params_mapping.yaml',
                description='SLAM Toolbox parameter file'
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation time'
            ),
        ]

    def rosbag_play_all(self):
        return ExecuteProcess(
            condition=UnlessCondition(
                PythonExpression(["'", self.play_topics, "' != ''"])
            ),
            cmd=[
                'ros2', 'bag', 'play',
                self.rosbag_dir,
                '--clock'
            ],
            output='screen'
        )

    def rosbag_play_selected(self):
        return ExecuteProcess(
            condition=IfCondition(
                PythonExpression(["'", self.play_topics, "' != ''"])
            ),
            cmd=[
                'ros2', 'bag', 'play',
                self.rosbag_dir,
                '--topics',
                self.play_topics,
                '--clock'
            ],
            output='screen'
        )

    def rviz(self):
        return Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': self.use_sim_time}],
            output='screen'
        )

    def slam_toolbox(self):
        return IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                launch_arguments={
                    'slam_params_file': self.slam_params,
                    'use_sim_time': self.use_sim_time
                }.items()
            )
    
    def slam_toolbox_offline(self):
        return IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'offline_launch.py')
                ])
            )
        


def generate_launch_description():

    launcher = Go2Rosbag()
    launch = LaunchDescription()

    for arg in launcher.arguments():
        launch.add_action(arg)

    launch.add_action(launcher.rosbag_play_all())
    launch.add_action(launcher.rosbag_play_selected())
    launch.add_action(launcher.rviz())
    launch.add_action(launcher.slam_toolbox())
    # launch.add_action(launcher.slam_toolbox_offline())

    return launch
