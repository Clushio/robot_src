from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('lio_lite')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='lio_lite',
            executable='run_location_online',
            name='laserMapping',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'mid360_ros2.yaml'),
                {
                    'preprocess.blind2': 0.04,
                    'pcd_save.pcd_save_en': False,
                    'ivox_nearby_type': 6,
                    'location_mode': True,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_loc',
            prefix='nice',
            arguments=['-d', os.path.join(share, 'launch', 'include', 'local_rviz.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
