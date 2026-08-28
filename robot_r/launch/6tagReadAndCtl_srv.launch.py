from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("tag_abort_timeout", default_value="10.0"),
            Node(
                package="x2bot_teleop",
                executable="TagCtl",
                name="TagCtl_service",
                output="screen",
                parameters=[
                    {
                        "tag_abort_timeout": ParameterValue(
                            LaunchConfiguration("tag_abort_timeout"), value_type=float
                        )
                    }
                ],
            ),
        ]
    )
