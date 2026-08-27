"""Launch the MXB Nav2 servers and ROS1 compatibility endpoints."""

# Copyright 2026 MXB navigation maintainers
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the project navigation stack."""
    package_share = get_package_share_directory('mxb_move_base')
    default_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    replanning_tree = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_replanning.xml')
    fixed_route_tree = os.path.join(
        package_share, 'behavior_trees', 'navigate_to_pose_fixed_route.xml')
    through_poses_tree = os.path.join(
        package_share, 'behavior_trees', 'navigate_through_poses.xml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel/nav')])

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': replanning_tree,
                'default_nav_through_poses_bt_xml': through_poses_tree,
            },
        ])

    compatibility_node = Node(
        package='mxb_move_base',
        executable='mxb_move_base',
        name='mxb_move_base',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'fixed_route_behavior_tree': fixed_route_tree,
            },
        ])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server', 'controller_server', 'bt_navigator'],
            },
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Absolute path to the Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate the navigation lifecycle nodes'),
        planner_server,
        controller_server,
        bt_navigator,
        compatibility_node,
        lifecycle_manager,
    ])
