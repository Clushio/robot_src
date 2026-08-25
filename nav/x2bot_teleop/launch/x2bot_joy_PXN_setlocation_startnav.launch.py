from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument(
            'maps_dir',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'maps'])),
        DeclareLaunchArgument('navigate_action', default_value='/navigate_to_pose'),
        Node(
            package='x2bot_teleop',
            executable='x2bot_joy_PXN',
            name='x2bot_joy_PXN',
            output='screen',
            parameters=[{
                'scale_angular': 0.4,
                'scale_linear': 0.25,
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
        Node(
            package='x2bot_teleop',
            executable='runnav',
            name='runnav',
            output='screen',
            parameters=[{
                'maps_dir': LaunchConfiguration('maps_dir'),
                'navigate_action': LaunchConfiguration('navigate_action'),
            }],
        ),
    ])
