from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        Node(
            package='x2bot_teleop',
            executable='x2bot_joy_PXN',
            name='x2bot_joy_PXN',
            output='screen',
            parameters=[{
                'scale_angular': 1.0,
                'scale_linear': 0.1,
                'cmd_vel_topic': '/cmd_vel/teleop',
            }],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joystick',
            output='screen',
            parameters=[{
                'device_id': ParameterValue(
                    LaunchConfiguration('device_id'), value_type=int),
            }],
        ),
    ])
