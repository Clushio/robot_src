import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_r")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_enable", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "pcd2pgm.launch.py")
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_rviz",
                arguments=["-d", os.path.join(share, "rviz", "mapgen_rviz.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz_enable")),
                output="screen",
            ),
        ]
    )
