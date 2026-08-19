#!/usr/bin/env python3

import copy
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from cmd_vel_arbiter.msg import ArbitratedCommand
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import UInt8
from visualization_msgs.msg import Marker, MarkerArray


class CollisionMonitorNodeTest(unittest.TestCase):

    def setUp(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._publish_odom = False
        self._publish_local = False
        self._publish_tf = False
        self._publish_phase = True
        self._phase = 0
        self._candidate = None
        self._odom_linear_x = 0.0
        self._odom_linear_y = 0.0
        self._odom_angular_z = 0.0
        self._local_map = self._make_map()
        self._last_output = None
        self._last_status = None
        self._last_markers = {}

        self._candidate_pub = rospy.Publisher(
            '/collision_monitor_test/candidate', ArbitratedCommand,
            queue_size=1)
        self._odom_pub = rospy.Publisher(
            '/collision_monitor_test/odom', Odometry, queue_size=1)
        self._static_pub = rospy.Publisher(
            '/collision_monitor_test/static_map', OccupancyGrid,
            queue_size=1, latch=True)
        self._local_pub = rospy.Publisher(
            '/collision_monitor_test/local_map', OccupancyGrid, queue_size=1)
        self._phase_pub = rospy.Publisher(
            '/collision_monitor_test/topology_phase', UInt8, queue_size=1)
        self._output_sub = rospy.Subscriber(
            '/collision_monitor_test/output', Twist, self._output_callback,
            queue_size=1)
        self._status_sub = rospy.Subscriber(
            '/collision_monitor_test/status', DiagnosticArray,
            self._status_callback, queue_size=1)
        self._marker_sub = rospy.Subscriber(
            '/collision_monitor_test/markers', MarkerArray,
            self._marker_callback, queue_size=1)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._thread = threading.Thread(target=self._publish_loop)
        self._thread.daemon = True
        self._thread.start()
        self._wait_for_connections()

    def tearDown(self):
        self._stop.set()
        self._thread.join(1.0)

    @staticmethod
    def _make_map():
        grid = OccupancyGrid()
        grid.header.frame_id = 'collision_test_map'
        grid.info.resolution = 0.05
        grid.info.width = 200
        grid.info.height = 200
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        return grid

    @staticmethod
    def _set_cell(grid, x, y, value):
        mx = int((x - grid.info.origin.position.x) / grid.info.resolution)
        my = int((y - grid.info.origin.position.y) / grid.info.resolution)
        grid.data[my * grid.info.width + mx] = value

    def _wait_for_connections(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (self._candidate_pub.get_num_connections() and
                    self._odom_pub.get_num_connections() and
                    self._static_pub.get_num_connections() and
                    self._local_pub.get_num_connections() and
                    self._phase_pub.get_num_connections()):
                return
            time.sleep(0.02)
        self.fail('collision monitor subscriptions did not connect')

    def _output_callback(self, message):
        with self._lock:
            self._last_output = message

    def _status_callback(self, message):
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        with self._lock:
            self._last_status = values

    def _marker_callback(self, message):
        markers = {
            marker.ns: marker for marker in message.markers
            if marker.action == Marker.ADD
        }
        with self._lock:
            self._last_markers = markers

    def _publish_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self._stop.is_set():
            stamp = rospy.Time.now()
            with self._lock:
                candidate = copy.deepcopy(self._candidate)
                publish_odom = self._publish_odom
                publish_local = self._publish_local
                publish_tf = self._publish_tf
                local_map = copy.deepcopy(self._local_map)
                odom_linear_x = self._odom_linear_x
                odom_linear_y = self._odom_linear_y
                odom_angular_z = self._odom_angular_z
                publish_phase = self._publish_phase
                phase = self._phase

            if candidate is not None:
                candidate.header.stamp = stamp
                self._candidate_pub.publish(candidate)
            if publish_odom:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'collision_test_map'
                odom.child_frame_id = 'collision_test_base'
                odom.twist.twist.linear.x = odom_linear_x
                odom.twist.twist.linear.y = odom_linear_y
                odom.twist.twist.angular.z = odom_angular_z
                self._odom_pub.publish(odom)
            if publish_local:
                local_map.header.stamp = stamp
                self._local_pub.publish(local_map)
            if publish_tf:
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = 'collision_test_map'
                transform.child_frame_id = 'collision_test_base'
                transform.transform.rotation.w = 1.0
                self._tf_broadcaster.sendTransform(transform)
            if publish_phase:
                self._phase_pub.publish(UInt8(data=phase))
            rate.sleep()

    def _set_phase(self, phase, publish=True):
        with self._lock:
            self._phase = phase
            self._publish_phase = publish
            self._last_status = None

    def _set_command(self, source, linear_x=0.0, linear_y=0.0,
                     angular_z=0.0):
        command = ArbitratedCommand()
        command.source = source
        command.command.linear.x = linear_x
        command.command.linear.y = linear_y
        command.command.angular.z = angular_z
        with self._lock:
            self._candidate = command
            self._last_output = None
            self._last_status = None

    def _wait_for_state(self, expected, expected_reason=None, timeout=3.0):
        zero_output_states = {
            'STOPPED', 'STOP_COMMAND', 'DATA_NOT_READY',
            'UNSUPPORTED_MOTION', 'PROFILE_TRANSITION',
            'EMERGENCY_STOP', 'COLLISION_STOP'
        }
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                status = copy.deepcopy(self._last_status)
                output = copy.deepcopy(self._last_output)
            output_matches = output is not None
            if output_matches and expected in zero_output_states:
                output_matches = (
                    abs(output.linear.x) < 0.001 and
                    abs(output.linear.y) < 0.001 and
                    abs(output.angular.z) < 0.001)
            if (status is not None and status.get('state') == expected and
                    output_matches and
                    (expected_reason is None or
                     status.get('reason') == expected_reason)):
                return status, output
            time.sleep(0.02)
        with self._lock:
            current = copy.deepcopy(self._last_status)
        self.fail('expected state {} reason {}, got {}'.format(
            expected, expected_reason, current))

    def _wait_for_ok_output(self, linear_x, linear_y, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                status = copy.deepcopy(self._last_status)
                output = copy.deepcopy(self._last_output)
            if (status is not None and status.get('state') == 'OK' and
                    output is not None and
                    abs(output.linear.x - linear_x) < 0.001 and
                    abs(output.linear.y - linear_y) < 0.001):
                return status, output
            time.sleep(0.02)
        self.fail('expected OK output ({}, {}), got status={} output={}'.format(
            linear_x, linear_y, status, output))

    def _wait_for_marker(self, namespace, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                marker = copy.deepcopy(self._last_markers.get(namespace))
            if marker is not None:
                return marker
            time.sleep(0.02)
        self.fail('expected marker namespace {}, got {}'.format(
            namespace, sorted(self._last_markers.keys())))

    def test_readiness_bypass_limits_speed_levels_and_timeout(self):
        self._set_command('nav', linear_x=0.10)
        status, output = self._wait_for_state(
            'DATA_NOT_READY', 'ODOM_STALE')
        self.assertEqual('ODOM_STALE', status['reason'])
        self.assertAlmostEqual(0.0, output.linear.x)

        self._set_command('teleop', linear_x=0.25, angular_z=0.80)
        status, output = self._wait_for_state('MANUAL_BYPASS')
        self.assertAlmostEqual(0.25, output.linear.x)
        self.assertAlmostEqual(0.80, output.angular.z)

        static_map = self._make_map()
        static_map.header.stamp = rospy.Time.now()
        self._static_pub.publish(static_map)
        with self._lock:
            self._publish_odom = True
            self._publish_local = True
            self._publish_tf = True

        self._set_command('nav', linear_x=0.25)
        status, output = self._wait_for_state(
            'UNSUPPORTED_MOTION', 'COMMAND_LIMIT_EXCEEDED')
        self.assertEqual('COMMAND_LIMIT_EXCEEDED', status['reason'])
        self.assertAlmostEqual(0.0, output.linear.x)

        self._set_command('nav', linear_x=0.20)
        self._wait_for_ok_output(0.20, 0.0, timeout=3.0)

        obstacle_map = self._make_map()
        self._set_cell(obstacle_map, 0.76, 0.0, 100)
        with self._lock:
            self._local_map = obstacle_map
        status, output = self._wait_for_state('SLOWDOWN')
        self.assertAlmostEqual(0.50, float(status['scale']), places=2)
        self.assertAlmostEqual(0.10, output.linear.x, places=2)

        with self._lock:
            self._local_map = self._make_map()
        status, output = self._wait_for_ok_output(0.20, 0.0, timeout=3.0)
        self.assertAlmostEqual(1.0, float(status['scale']), places=2)
        self.assertAlmostEqual(0.20, output.linear.x, places=2)

        # START_SEGMENT and FINAL_SEGMENT use the 0.08 m nav_terminal
        # footprint, but shrinking from normal navigation must wait for real
        # odometry to stop. Navigation limits and source remain unchanged.
        with self._lock:
            self._odom_linear_x = 0.03
        self._set_phase(1)
        self._set_command('nav', linear_x=0.10)
        status, output = self._wait_for_state(
            'PROFILE_TRANSITION',
            'WAITING_FOR_NAVIGATION_TERMINAL_STOP')
        self.assertEqual('transition', status['profile'])
        self.assertAlmostEqual(0.15, float(status['footprint_padding']),
                               places=2)
        self.assertAlmostEqual(0.0, output.linear.x)
        with self._lock:
            self._odom_linear_x = 0.0
        status, output = self._wait_for_ok_output(0.10, 0.0)
        self.assertEqual('nav_terminal', status['profile'])
        self.assertEqual('START_SEGMENT', status['topology_phase'])
        self.assertEqual('true', status['topology_phase_fresh'])
        self.assertAlmostEqual(0.08, float(status['footprint_padding']),
                               places=2)

        # Expanding back to normal navigation is conservative and immediate.
        self._set_phase(0)
        status, output = self._wait_for_ok_output(0.10, 0.0)
        self.assertEqual('nav', status['profile'])
        self.assertAlmostEqual(0.15, float(status['footprint_padding']),
                               places=2)

        # A lost AutoNAV heartbeat must fail back to 0.15 m rather than leave
        # the terminal padding active.
        self._set_phase(2)
        status, output = self._wait_for_ok_output(0.10, 0.0)
        self.assertEqual('nav_terminal', status['profile'])
        self.assertEqual('FINAL_SEGMENT', status['topology_phase'])
        self._set_phase(2, publish=False)
        status, output = self._wait_for_ok_output(0.10, 0.0)
        deadline = time.monotonic() + 2.0
        while (status.get('profile') != 'nav' and
               time.monotonic() < deadline):
            time.sleep(0.03)
            with self._lock:
                status = copy.deepcopy(self._last_status)
        self.assertIsNotNone(status)
        self.assertEqual('nav', status['profile'])
        self.assertEqual('false', status['topology_phase_fresh'])
        self.assertAlmostEqual(0.15, float(status['footprint_padding']),
                               places=2)
        self._set_phase(0)

        with self._lock:
            self._odom_linear_x = 0.03
        self._set_command('tag', linear_y=0.05)
        status, output = self._wait_for_state(
            'PROFILE_TRANSITION', 'WAITING_FOR_ROBOT_TO_STOP')
        self.assertAlmostEqual(0.0, output.linear.y)
        self.assertEqual('transition', status['profile'])
        self.assertAlmostEqual(0.15, float(status['footprint_padding']),
                               places=2)
        with self._lock:
            self._odom_linear_x = 0.0
        status, output = self._wait_for_ok_output(0.0, 0.05, timeout=3.0)
        self.assertEqual('tag', status['profile'])
        self.assertAlmostEqual(0.08, float(status['footprint_padding']),
                               places=2)
        self.assertAlmostEqual(0.05, output.linear.y, places=3)

        self._set_command('tag', linear_x=-0.05)
        status, output = self._wait_for_ok_output(-0.05, 0.0)
        self.assertAlmostEqual(-0.05, output.linear.x, places=3)
        self.assertAlmostEqual(0.0, output.linear.y, places=3)

        self._set_command('nav', linear_y=0.01)
        status, output = self._wait_for_state(
            'UNSUPPORTED_MOTION', 'UNSUPPORTED_COMMAND_DOF')
        self.assertAlmostEqual(0.0, output.linear.y)

        # A small residual lateral odom value must not deadlock the return
        # from tag to navigation after the stationary transition threshold.
        with self._lock:
            self._odom_linear_y = 0.005
        self._set_command('nav', linear_x=0.10)
        status, output = self._wait_for_state('PROFILE_TRANSITION')
        self.assertAlmostEqual(0.0, output.linear.x)
        status, output = self._wait_for_ok_output(0.10, 0.0)
        with self._lock:
            self._odom_linear_y = 0.0

        with self._lock:
            self._candidate = None
        status, output = self._wait_for_state(
            'STOPPED', 'CANDIDATE_STALE')
        self.assertAlmostEqual(0.0, output.linear.x)

        self._set_command('nav', linear_x=0.20)
        with self._lock:
            self._publish_odom = False
        status, output = self._wait_for_state(
            'DATA_NOT_READY', 'ODOM_STALE')
        self.assertAlmostEqual(0.0, output.linear.x)

        with self._lock:
            self._publish_odom = True
            self._publish_tf = False
        status, output = self._wait_for_state(
            'DATA_NOT_READY', 'TF_STALE')
        self.assertAlmostEqual(0.0, output.linear.x)

        with self._lock:
            self._publish_tf = True
        invalid_static_map = self._make_map()
        invalid_static_map.header.frame_id = 'wrong_frame'
        self._static_pub.publish(invalid_static_map)
        status, output = self._wait_for_state(
            'DATA_NOT_READY', 'STATIC_MAP_NOT_READY')
        self.assertAlmostEqual(0.0, output.linear.x)
        static_map.header.stamp = rospy.Time.now()
        self._static_pub.publish(static_map)

        self._set_command('tag', linear_y=0.03)
        with self._lock:
            self._publish_local = False
        status, output = self._wait_for_state(
            'DATA_NOT_READY', 'LOCAL_MAP_NOT_READY')
        self.assertEqual('LOCAL_MAP_NOT_READY', status['reason'])
        self.assertAlmostEqual(0.0, output.linear.x)

        self._set_command('nav')
        status, output = self._wait_for_state('STOP_COMMAND')
        self.assertEqual('ZERO_COMMAND_PASSED', status['reason'])
        self.assertAlmostEqual(0.0, output.linear.x)

        current = self._wait_for_marker('current_padded_footprint')
        self.assertEqual(Marker.LINE_STRIP, current.type)
        self.assertEqual(5, len(current.points))
        self.assertAlmostEqual(-0.51, min(p.x for p in current.points),
                               places=3)
        self.assertAlmostEqual(0.51, max(p.x for p in current.points),
                               places=3)
        self.assertAlmostEqual(-0.40, min(p.y for p in current.points),
                               places=3)
        self.assertAlmostEqual(0.40, max(p.y for p in current.points),
                               places=3)


if __name__ == '__main__':
    rospy.init_node('collision_monitor_node_test')
    rostest.rosrun('collision_monitor', 'collision_monitor_node_test',
                   CollisionMonitorNodeTest)
