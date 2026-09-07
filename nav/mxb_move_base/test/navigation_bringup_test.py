#!/usr/bin/env python3

"""Exercise the real navigation bringup and its costmap wiring."""

# Copyright 2026 MXB navigation maintainers
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_testing.actions
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Start the production navigation launch with a minimal static TF."""
    navigation_launch = os.path.join(
        get_package_share_directory('mxb_move_base'),
        'launch',
        'navigation.launch.py')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch))
    map_to_base = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='navigation_test_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen')

    return launch.LaunchDescription([
        map_to_base,
        navigation,
        launch_testing.actions.ReadyToTest(),
    ])


class NavigationBringupTest(unittest.TestCase):
    """Verify lifecycle activation and ROS2 costmap topic contracts."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_bringup_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_until(self, predicate, description, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail('timed out waiting for ' + description)

    def _wait_for_lifecycle_active(self, node_name, timeout=30.0):
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=20.0):
                self.fail(f'{node_name} lifecycle service was not available')
            deadline = time.monotonic() + timeout
            last_state = State.PRIMARY_STATE_UNKNOWN
            while time.monotonic() < deadline and rclpy.ok():
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=1.0)
                if future.done() and future.result() is not None:
                    last_state = future.result().current_state.id
                    if last_state == State.PRIMARY_STATE_ACTIVE:
                        return
                time.sleep(0.05)
            self.fail(
                f'{node_name} did not reach active; last state={last_state}')
        finally:
            self.node.destroy_client(client)

    def test_navigation_nodes_become_active(self):
        """All lifecycle-managed navigation servers must activate."""
        for node_name in ('planner_server', 'controller_server',
                          'bt_navigator'):
            self._wait_for_lifecycle_active(node_name)

    def test_costmap_topic_wiring(self):
        """Plugins must consume OccupancyGrid, never Nav2 raw costmaps."""
        def wiring_is_ready():
            topics = dict(self.node.get_topic_names_and_types())
            subscriptions = self.node.get_subscriptions_info_by_topic(
                '/local_costmap/costmap')
            return (
                topics.get('/local_costmap/costmap') ==
                ['nav_msgs/msg/OccupancyGrid'] and
                topics.get('/local_costmap/costmap_raw') ==
                ['nav2_msgs/msg/Costmap'] and
                any(endpoint.node_name == 'controller_server'
                    and endpoint.topic_type == 'nav_msgs/msg/OccupancyGrid'
                    for endpoint in subscriptions))

        self._wait_until(wiring_is_ready, 'Nav2 local costmap wiring')
        raw_subscriptions = self.node.get_subscriptions_info_by_topic(
            '/local_costmap/costmap_raw')
        self.assertFalse(any(
            endpoint.node_name == 'controller_server' and
            endpoint.topic_type == 'nav_msgs/msg/OccupancyGrid'
            for endpoint in raw_subscriptions))


if __name__ == '__main__':
    unittest.main()
