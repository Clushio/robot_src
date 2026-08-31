#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    results_root = LaunchConfiguration('results_root')
    positions_file = LaunchConfiguration('positions_file')
    topology_file = LaunchConfiguration('topology_file')
    repo_path = LaunchConfiguration('repo_path')
    home = EnvironmentVariable('HOME')
    config = PathJoinSubstitution(
        [FindPackageShare('nav_benchmark'), 'config', 'benchmark.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'results_root',
            default_value=PathJoinSubstitution(
                [home, 'maps', 'nav_benchmark_results']
            ),
        ),
        DeclareLaunchArgument(
            'positions_file',
            default_value=PathJoinSubstitution(
                [home, 'maps', 'robot_positions.txt']
            ),
        ),
        DeclareLaunchArgument(
            'topology_file',
            default_value=PathJoinSubstitution(
                [home, 'maps', 'topology.yaml']
            ),
        ),
        DeclareLaunchArgument('repo_path', default_value=''),
        Node(
            package='nav_benchmark',
            executable='navigation_benchmark.py',
            name='navigation_benchmark',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                config,
                {
                    'results_root': results_root,
                    'positions_file': positions_file,
                    'topology_file': topology_file,
                    'repo_path': repo_path,
                },
            ],
        ),
    ])
