import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("robot_r")
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "livox_mid360.launch.py")
        )
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "lio", "launch", "location_360.launch.py")
        )
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_file,
                "topic_name": "map",
                "frame_id": "map",
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_file", default_value="/home/nav/maps/map.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            livox,
            map_server,
            lifecycle_manager,
            localization,
        ]
    )
