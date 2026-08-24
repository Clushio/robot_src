import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            'launch',
            launch_file)))


def generate_launch_description():
    return LaunchDescription([
        include_launch('ranger_base', 'ranger_mini_v1.launch.py'),
        include_launch('cmd_vel_arbiter', 'cmd_vel_arbiter.launch.py'),
        include_launch('collision_monitor', 'collision_monitor.launch.py'),
    ])
