import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_r")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="lio_lite",
                executable="run_mapping_online",
                name="laserMapping",
                output="screen",
                parameters=[
                    os.path.join(share, "lio", "config", "mid360_ros2.yaml")
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                prefix="nice",
                arguments=[
                    "-d",
                    os.path.join(share, "lio", "launch", "include", "rviz.rviz"),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
