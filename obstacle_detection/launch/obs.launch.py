from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('obstacle_detection'))
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='obstacle_detection',
            name='obstacle_detection',
            output='screen',
            parameters=[str(package_share / 'config' / 'params.yaml')],
        ),
    ])
