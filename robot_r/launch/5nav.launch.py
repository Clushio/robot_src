import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name), "launch", launch_file
            )
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            include_launch("livx2pcl", "repub_ang.launch.py"),
            include_launch("mxb_move_base", "navigation.launch.py"),
        ]
    )
