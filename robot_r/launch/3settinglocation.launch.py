from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="x2bot_teleop",
                executable="saveLocation",
                name="saveLocation",
                output="screen",
            )
        ]
    )
