"""Optional ROS 2 replacement for the disabled ROS 1 yocs smoother include."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/cmd_vel/teleop_raw'),
        DeclareLaunchArgument('output_topic', default_value='/cmd_vel/teleop'),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='teleop_velocity_smoother',
            output='screen',
            parameters=[{
                'smoothing_frequency': 20.0,
                'scale_velocities': False,
                'feedback': 'OPEN_LOOP',
                'max_velocity': [0.85, 0.0, 8.2],
                'min_velocity': [-0.85, 0.0, -8.2],
                'max_accel': [2.5, 0.0, 3.2],
                'max_decel': [-2.5, 0.0, -3.2],
                'odom_topic': '/odom',
                'odom_duration': 0.1,
                'deadband_velocity': [0.0, 0.0, 0.0],
                'velocity_timeout': 1.0,
            }],
            remappings=[
                ('cmd_vel', input_topic),
                ('cmd_vel_smoothed', output_topic),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_teleop_velocity_smoother',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['teleop_velocity_smoother'],
            }],
        ),
    ])
