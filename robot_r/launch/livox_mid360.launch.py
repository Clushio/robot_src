import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("robot_r")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(share, "config", "MID360_config.json"),
            ),
            DeclareLaunchArgument("xfer_format", default_value="1"),
            DeclareLaunchArgument("multi_topic", default_value="0"),
            DeclareLaunchArgument("data_src", default_value="0"),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("output_data_type", default_value="0"),
            DeclareLaunchArgument("frame_id", default_value="livox_frame"),
            DeclareLaunchArgument(
                "lvx_file_path", default_value="/home/livox/livox_test.lvx"
            ),
            DeclareLaunchArgument(
                "cmdline_input_bd_code", default_value="100000000000000"
            ),
            DeclareLaunchArgument("rosbag_enable", default_value="false"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher2",
                output="screen",
                parameters=[
                    {
                        "xfer_format": ParameterValue(
                            LaunchConfiguration("xfer_format"), value_type=int
                        ),
                        "multi_topic": ParameterValue(
                            LaunchConfiguration("multi_topic"), value_type=int
                        ),
                        "data_src": ParameterValue(
                            LaunchConfiguration("data_src"), value_type=int
                        ),
                        "publish_freq": ParameterValue(
                            LaunchConfiguration("publish_freq"), value_type=float
                        ),
                        "output_data_type": ParameterValue(
                            LaunchConfiguration("output_data_type"), value_type=int
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "user_config_path": LaunchConfiguration("config_file"),
                        "lvx_file_path": LaunchConfiguration("lvx_file_path"),
                        "cmdline_input_bd_code": ParameterValue(
                            LaunchConfiguration("cmdline_input_bd_code"),
                            value_type=str,
                        ),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rosbag_enable")),
            ),
        ]
    )
