import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(path, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    share = get_package_share_directory("robot_r")
    map_file = LaunchConfiguration("map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
    mavros_launch = PathJoinSubstitution(
        [FindPackageShare("mavros"), "launch", "px4.launch"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_enable", default_value="true"),
            DeclareLaunchArgument("mavros_enable", default_value="false"),
            DeclareLaunchArgument("map_file", default_value="/home/nav/maps/map.yaml"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            include(os.path.join(share, "launch", "robottf.launch.py")),
            include(os.path.join(share, "launch", "livox_mid360.launch.py")),
            map_server,
            lifecycle_manager,
            include(
                os.path.join(
                    share, "lio", "launch", "location_360_nrviz.launch.py"
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_rviz",
                arguments=["-d", os.path.join(share, "rviz", "nav_rviz.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz_enable")),
                output="screen",
            ),
            include(
                mavros_launch,
                condition=IfCondition(LaunchConfiguration("mavros_enable")),
            ),
        ]
    )
