from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("file_directory", default_value="/home/nav/maps/"),
            DeclareLaunchArgument("file_name", default_value="GlobalMap"),
            Node(
                package="pcd2pgm",
                executable="pcd2topic",
                name="pcd2topic",
                output="screen",
                parameters=[
                    {
                        "file_directory": LaunchConfiguration("file_directory"),
                        "file_name": LaunchConfiguration("file_name"),
                        "thre_z_min": 0.04,
                        "thre_z_max": 1.6,
                        "flag_pass_through": 0,
                        "map_resolution": 0.01,
                        "map_topic_name": "map",
                        "thre_radius": 0.08,
                        "radius_filter_enable": True,
                        "radius_min_neighbors": 3,
                        "grid_x": 0.1,
                        "grid_y": 0.1,
                        "grid_z": 0.1,
                    }
                ],
            ),
        ]
    )
