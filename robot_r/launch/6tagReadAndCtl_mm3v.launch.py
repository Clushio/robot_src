from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="38400"),
        DeclareLaunchArgument("x_sign", default_value="1.0"),
        DeclareLaunchArgument("y_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="-1.0"),
        DeclareLaunchArgument("xp_bias", default_value="0.0"),
        DeclareLaunchArgument("yp_bias", default_value="0.0"),
        DeclareLaunchArgument("angle_bias", default_value="90.0"),
        DeclareLaunchArgument("udp_feedback_enable", default_value="true"),
        DeclareLaunchArgument("udp_feedback_host", default_value="192.168.3.17"),
        DeclareLaunchArgument("udp_feedback_port", default_value="22222"),
        DeclareLaunchArgument("udp_feedback_rate", default_value="2.0"),
        DeclareLaunchArgument("tag_abort_timeout", default_value="10.0"),
    ]

    tag_controller = Node(
        package="x2bot_teleop",
        executable="TagCtl",
        name="TagCtl_service",
        output="screen",
        parameters=[{"tag_abort_timeout": typed("tag_abort_timeout", float)}],
    )
    serial_reader = Node(
        package="robot_r",
        executable="mm3v_serial_reader.py",
        name="mm3v_serial_reader",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": typed("baudrate", int),
                "frame_id": "map",
                "output_topic": "/tag_position",
                "output_meter": False,
                "publish_no_tag": True,
                "tag_lost_timeout": 0.5,
                "x_sign": typed("x_sign", float),
                "y_sign": typed("y_sign", float),
                "yaw_sign": typed("yaw_sign", float),
                "xp_bias": typed("xp_bias", float),
                "yp_bias": typed("yp_bias", float),
                "angle_bias": typed("angle_bias", float),
                "udp_feedback_enable": typed("udp_feedback_enable", bool),
                "udp_feedback_host": LaunchConfiguration("udp_feedback_host"),
                "udp_feedback_port": typed("udp_feedback_port", int),
                "udp_feedback_rate": typed("udp_feedback_rate", float),
            }
        ],
    )
    return LaunchDescription(arguments + [tag_controller, serial_reader])
