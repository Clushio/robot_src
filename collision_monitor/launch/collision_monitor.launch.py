import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('collision_monitor')
    footprint_config = os.path.join(
        package_share, 'config', 'robot_footprint.yaml')
    monitor_config = os.path.join(
        package_share, 'config', 'collision_monitor.yaml')

    monitor = Node(
        package='collision_monitor',
        executable='collision_monitor_node',
        name='collision_monitor',
        output='screen',
        parameters=[footprint_config, monitor_config])

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(
                reason='collision_monitor exited'))]))

    return LaunchDescription([monitor, shutdown_on_exit])
