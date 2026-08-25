from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livx2pcl',
            executable='livox_repub_ang',
            name='livox_repub_ang',
            output='screen',
            parameters=[{
                'min_angle_deg': -110.0,
                'max_angle_deg': 110.0,
            }],
            remappings=[('/livox/lidar', '/livox/lidar')]),
    ])
