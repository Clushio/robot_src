from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("file_directory", default_value="/home/nav/maps/"),
        DeclareLaunchArgument("file_name", default_value="GlobalMap"),
        DeclareLaunchArgument("thre_z_min", default_value="-0.1"),
        DeclareLaunchArgument("thre_z_max", default_value="0.3"),
        DeclareLaunchArgument("flag_pass_through", default_value="0"),
        DeclareLaunchArgument("map_resolution", default_value="0.01"),
        DeclareLaunchArgument("map_topic_name", default_value="map"),
        DeclareLaunchArgument("thre_radius", default_value="0.1"),
        DeclareLaunchArgument("radius_filter_enable", default_value="false"),
        DeclareLaunchArgument("radius_min_neighbors", default_value="10"),
        DeclareLaunchArgument("grid_x", default_value="0.1"),
        DeclareLaunchArgument("grid_y", default_value="0.1"),
        DeclareLaunchArgument("grid_z", default_value="0.1"),
        DeclareLaunchArgument("savemap", default_value="true"),
    ]

    node = Node(
        package="pcd2pgm",
        executable="pcd2topic",
        name="pcd2topic",
        output="screen",
        parameters=[
            {
                "file_directory": LaunchConfiguration("file_directory"),
                "file_name": LaunchConfiguration("file_name"),
                "thre_z_min": ParameterValue(
                    LaunchConfiguration("thre_z_min"), value_type=float
                ),
                "thre_z_max": ParameterValue(
                    LaunchConfiguration("thre_z_max"), value_type=float
                ),
                "flag_pass_through": ParameterValue(
                    LaunchConfiguration("flag_pass_through"), value_type=int
                ),
                "map_resolution": ParameterValue(
                    LaunchConfiguration("map_resolution"), value_type=float
                ),
                "map_topic_name": LaunchConfiguration("map_topic_name"),
                "thre_radius": ParameterValue(
                    LaunchConfiguration("thre_radius"), value_type=float
                ),
                "radius_filter_enable": ParameterValue(
                    LaunchConfiguration("radius_filter_enable"), value_type=bool
                ),
                "radius_min_neighbors": ParameterValue(
                    LaunchConfiguration("radius_min_neighbors"), value_type=int
                ),
                "grid_x": ParameterValue(
                    LaunchConfiguration("grid_x"), value_type=float
                ),
                "grid_y": ParameterValue(
                    LaunchConfiguration("grid_y"), value_type=float
                ),
                "grid_z": ParameterValue(
                    LaunchConfiguration("grid_z"), value_type=float
                ),
                "savemap": ParameterValue(
                    LaunchConfiguration("savemap"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
