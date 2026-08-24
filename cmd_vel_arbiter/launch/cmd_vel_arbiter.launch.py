import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('cmd_vel_arbiter'),
        'config',
        'cmd_vel_arbiter.yaml')

    arbiter = Node(
        package='cmd_vel_arbiter',
        executable='cmd_vel_arbiter_node',
        name='cmd_vel_arbiter',
        output='screen',
        parameters=[config_file])

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=arbiter,
            on_exit=[EmitEvent(event=Shutdown(
                reason='cmd_vel_arbiter exited'))]))

    return LaunchDescription([arbiter, shutdown_on_exit])
