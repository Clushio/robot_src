#!/usr/bin/env python3

import csv
import datetime
import json
import math
import os
import shutil
import statistics
import subprocess
import threading
import time

import rospy
import tf2_ros
from cmd_vel_arbiter.msg import ArbitratedCommand
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String, UInt8

from nav_benchmark.metrics import (
    circular_mean,
    closest_point_on_polyline,
    file_fingerprint,
    finite,
    nearest_pose,
    parse_robot_positions,
    path_length,
    percentile,
    quaternion_to_yaw,
    resolve_requested_pose,
    root_mean_square,
    summarize,
    wrap_angle,
)
from nav_benchmark.storage import DurableCsv, atomic_write_json


SAMPLE_FIELDS = [
    'wall_time', 'ros_time', 'session_id', 'run_id', 'task_state',
    'start_id', 'start_name', 'goal_id', 'goal_name', 'run_mode',
    'x', 'y', 'yaw_deg', 'tf_age_ms', 'localization_age_ms',
    'odom_vx', 'odom_vy', 'odom_wz',
    'cmd_nav_vx', 'cmd_nav_wz', 'cmd_source',
    'cmd_candidate_vx', 'cmd_candidate_vy', 'cmd_candidate_wz',
    'cmd_output_vx', 'cmd_output_vy', 'cmd_output_wz',
    'topology_phase', 'bspline_status', 'reference_version',
    'reference_segment', 'reference_x', 'reference_y',
    'cte_signed_m', 'cte_abs_m',
    'goal_x', 'goal_y', 'goal_yaw_deg',
    'goal_position_error_m', 'goal_yaw_error_deg',
    'collision_state', 'collision_reason', 'collision_scale',
    'sample_valid',
]

EVENT_FIELDS = [
    'wall_time', 'ros_time', 'session_id', 'run_id', 'event_type',
    'state', 'reason', 'source', 'detail', 'path_version',
]

PATH_FIELDS = [
    'wall_time', 'ros_time', 'session_id', 'run_id', 'path_type',
    'path_version', 'frame_id', 'point_index', 'x', 'y', 'yaw_deg',
]

TASK_FIELDS = [
    'session_id', 'run_id', 'direction', 'start_id', 'start_name',
    'goal_id', 'goal_name', 'run_mode', 'start_time', 'finish_time',
    'duration_s', 'result', 'failure_reason', 'manual_intervention',
    'recovered_after_logger_restart', 'data_complete',
    'start_x', 'start_y', 'start_yaw_deg',
    'goal_x', 'goal_y', 'goal_yaw_deg',
    'arrival_x', 'arrival_y', 'arrival_yaw_deg',
    'arrival_position_error_m', 'arrival_yaw_error_deg',
    'settled_x', 'settled_y', 'settled_yaw_deg',
    'settled_position_error_m', 'settled_yaw_error_deg', 'settle_window_s',
    'topology_path_length_m', 'reference_path_length_m',
    'actual_path_length_m', 'extra_path_ratio',
    'cte_sample_count', 'cte_sum_m', 'cte_sum_sq_m2',
    'mean_cte_m', 'rmse_cte_m', 'p95_cte_m', 'max_cte_m',
    'max_abs_angular_velocity', 'rms_angular_acceleration',
    'sharp_turn_count', 'unexpected_stop_count', 'observed_replan_count',
    'collision_slowdown_count', 'collision_stop_count', 'estop_count',
    'localization_gap_count', 'max_localization_gap_s',
]


def iso_time(timestamp=None):
    timestamp = time.time() if timestamp is None else float(timestamp)
    return datetime.datetime.fromtimestamp(
        timestamp, datetime.timezone.utc
    ).astimezone().isoformat(timespec='milliseconds')


def number(value, digits=6):
    if not finite(value):
        return ''
    return round(float(value), digits)


def bool_text(value):
    return 'true' if value else 'false'


def process_is_alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, TypeError, ValueError):
        return False


class ActiveTask:
    MAX_DISTRIBUTION_SAMPLES = 5000

    def __init__(self, run_id, requested_id, goal, target_name, start_wall,
                 start_pose=None, start_node=None, run_mode=1, recovered=False):
        self.run_id = int(run_id)
        self.requested_id = int(requested_id)
        self.goal = dict(goal) if goal else None
        self.target_name = target_name
        self.start_wall = float(start_wall)
        self.start_pose = tuple(start_pose) if start_pose else None
        self.start_node = dict(start_node) if start_node else None
        self.run_mode = int(run_mode)
        self.recovered = bool(recovered)
        self.manual_intervention = False
        self.actual_length = 0.0
        self.last_pose = None
        self.cte_values = []
        self.cte_sum = 0.0
        self.cte_sum_sq = 0.0
        self.cte_count = 0
        self.angular_accelerations = []
        self.max_abs_omega = 0.0
        self.last_omega = None
        self.last_omega_wall = None
        self.sharp_since = None
        self.sharp_latched = False
        self.stop_since = None
        self.stop_latched = False
        self.sharp_turn_count = 0
        self.unexpected_stop_count = 0
        self.topology_path_length = float('nan')
        self.reference_path_length = float('nan')
        self.topology_updates = 0
        self.last_topology_fingerprint = ''
        self.observed_replan_count = 0
        self.collision_slowdown_count = 0
        self.collision_stop_count = 0
        self.estop_count = 0
        self.localization_gap_count = 0
        self.max_localization_gap = 0.0
        self.localization_gap_started = None

    @staticmethod
    def _bounded_append(values, value):
        values.append(float(value))
        if len(values) > ActiveTask.MAX_DISTRIBUTION_SAMPLES:
            del values[1::2]

    def update_pose(self, pose, max_pose_jump):
        if self.start_pose is None:
            self.start_pose = tuple(pose)
        if self.last_pose is not None:
            distance = math.hypot(pose[0] - self.last_pose[0],
                                  pose[1] - self.last_pose[1])
            if distance <= max_pose_jump:
                self.actual_length += distance
            else:
                self.localization_gap_count += 1
        self.last_pose = tuple(pose)

    def update_tracking(self, cte, omega, speed, wall_time, collision_state,
                        stop_speed, stop_duration, sharp_omega,
                        sharp_duration):
        cte = abs(float(cte))
        self.cte_count += 1
        self.cte_sum += cte
        self.cte_sum_sq += cte * cte
        self._bounded_append(self.cte_values, cte)

        omega = float(omega)
        self.max_abs_omega = max(self.max_abs_omega, abs(omega))
        if self.last_omega is not None and self.last_omega_wall is not None:
            dt = wall_time - self.last_omega_wall
            if 0.02 <= dt <= 0.5:
                self._bounded_append(
                    self.angular_accelerations, (omega - self.last_omega) / dt
                )
        self.last_omega = omega
        self.last_omega_wall = wall_time

        if abs(omega) > sharp_omega:
            if self.sharp_since is None:
                self.sharp_since = wall_time
            elif (not self.sharp_latched and
                  wall_time - self.sharp_since >= sharp_duration):
                self.sharp_turn_count += 1
                self.sharp_latched = True
        else:
            self.sharp_since = None
            self.sharp_latched = False

        if abs(speed) < stop_speed and collision_state == 'OK':
            if self.stop_since is None:
                self.stop_since = wall_time
            elif (not self.stop_latched and
                  wall_time - self.stop_since >= stop_duration):
                self.unexpected_stop_count += 1
                self.stop_latched = True
        else:
            self.stop_since = None
            self.stop_latched = False

    def stop_tracking_window(self):
        self.last_omega = None
        self.last_omega_wall = None
        self.sharp_since = None
        self.sharp_latched = False
        self.stop_since = None
        self.stop_latched = False

    def checkpoint(self):
        return {
            'run_id': self.run_id,
            'requested_id': self.requested_id,
            'goal': self.goal,
            'target_name': self.target_name,
            'start_wall': self.start_wall,
            'start_pose': self.start_pose,
            'start_node': self.start_node,
            'run_mode': self.run_mode,
            'manual_intervention': self.manual_intervention,
            'actual_length': self.actual_length,
            'last_pose': self.last_pose,
            'cte_values': self.cte_values,
            'cte_sum': self.cte_sum,
            'cte_sum_sq': self.cte_sum_sq,
            'cte_count': self.cte_count,
            'angular_accelerations': self.angular_accelerations,
            'max_abs_omega': self.max_abs_omega,
            'sharp_turn_count': self.sharp_turn_count,
            'unexpected_stop_count': self.unexpected_stop_count,
            'topology_path_length': self.topology_path_length,
            'reference_path_length': self.reference_path_length,
            'topology_updates': self.topology_updates,
            'last_topology_fingerprint': self.last_topology_fingerprint,
            'observed_replan_count': self.observed_replan_count,
            'collision_slowdown_count': self.collision_slowdown_count,
            'collision_stop_count': self.collision_stop_count,
            'estop_count': self.estop_count,
            'localization_gap_count': self.localization_gap_count,
            'max_localization_gap': self.max_localization_gap,
        }

    @classmethod
    def restore(cls, checkpoint):
        task = cls(
            checkpoint['run_id'], checkpoint['requested_id'],
            checkpoint.get('goal'), checkpoint.get('target_name', ''),
            checkpoint.get('start_wall', time.time()),
            checkpoint.get('start_pose'), checkpoint.get('start_node'),
            checkpoint.get('run_mode', 1), recovered=True,
        )
        for key in (
            'manual_intervention', 'actual_length', 'last_pose', 'cte_values',
            'cte_sum', 'cte_sum_sq', 'cte_count', 'angular_accelerations',
            'max_abs_omega', 'sharp_turn_count', 'unexpected_stop_count',
            'topology_path_length', 'reference_path_length',
            'topology_updates', 'last_topology_fingerprint',
            'observed_replan_count', 'collision_slowdown_count',
            'collision_stop_count', 'estop_count', 'localization_gap_count',
            'max_localization_gap',
        ):
            if key in checkpoint:
                setattr(task, key, checkpoint[key])
        return task


class NavigationBenchmark:
    def __init__(self):
        self.lock = threading.RLock()
        self.shutdown_started = False
        self.sample_rate = max(1.0, float(rospy.get_param('~sample_rate', 10.0)))
        self.sync_interval = max(0.1, float(rospy.get_param('~sync_interval', 1.0)))
        self.settle_window = max(0.0, float(rospy.get_param('~settle_window', 0.5)))
        self.bspline_status_timeout = max(
            0.1, float(rospy.get_param('~bspline_status_timeout', 0.5))
        )
        self.tracking_min_speed = max(
            0.0, float(rospy.get_param('~tracking_min_speed', 0.02))
        )
        self.stop_speed = max(0.0, float(rospy.get_param('~stop_speed', 0.02)))
        self.stop_duration = max(0.1, float(rospy.get_param('~stop_duration', 0.5)))
        self.sharp_omega = max(
            0.0, float(rospy.get_param('~sharp_turn_omega', 0.3))
        )
        self.sharp_duration = max(
            0.1, float(rospy.get_param('~sharp_turn_duration', 0.3))
        )
        self.max_pose_jump = max(
            0.05, float(rospy.get_param('~max_pose_jump', 0.5))
        )
        self.localization_gap_threshold = max(
            0.1, float(rospy.get_param('~localization_gap_threshold', 0.5))
        )
        self.disk_warning_mb = max(
            10.0, float(rospy.get_param('~disk_warning_mb', 512.0))
        )
        self.recovery_window = max(
            1.0, float(rospy.get_param('~recovery_window', 30.0))
        )
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.positions_file = os.path.expanduser(rospy.get_param(
            '~positions_file', '~/maps/robot_positions.txt'
        ))
        self.topology_file = os.path.expanduser(rospy.get_param(
            '~topology_file', '~/maps/topology.yaml'
        ))
        self.results_root = os.path.expanduser(rospy.get_param(
            '~results_root', '~/maps/nav_benchmark_results'
        ))
        requested_repo_path = rospy.get_param('~repo_path', '')
        self.repo_path = self._discover_repo_path(
            [requested_repo_path, os.path.dirname(os.path.abspath(__file__))]
        )

        self.poses = parse_robot_positions(self.positions_file)
        self.session_id, self.session_dir, recovered_session = self._open_session()
        self.marker_path = os.path.join(self.results_root, '.active_session.json')
        self.runtime_path = os.path.join(self.session_dir, 'runtime_state.json')
        self.meta_path = os.path.join(self.session_dir, 'session.json')
        self.summary_path = os.path.join(self.session_dir, 'live_summary.json')

        self.samples = DurableCsv(
            os.path.join(self.session_dir, 'samples.csv'), SAMPLE_FIELDS,
            self.sync_interval,
        )
        self.events = DurableCsv(
            os.path.join(self.session_dir, 'events.csv'), EVENT_FIELDS,
            self.sync_interval,
        )
        self.paths = DurableCsv(
            os.path.join(self.session_dir, 'paths.csv'), PATH_FIELDS,
            self.sync_interval,
        )
        self.tasks = DurableCsv(
            os.path.join(self.session_dir, 'tasks.csv'), TASK_FIELDS,
            self.sync_interval,
        )

        self.meta = self._load_or_create_meta(recovered_session)
        self.task_rows = self._load_task_rows()
        self.next_run_id = 1 + max(
            [int(float(row.get('run_id', 0) or 0)) for row in self.task_rows] or [0]
        )
        self.active_task = None
        self.pending_arrival = None
        self._restore_runtime_if_available(recovered_session)

        self.latest_pose = None
        self.latest_pose_wall = None
        self.latest_localization_wall = None
        self.odom = Twist()
        self.cmd_nav = Twist()
        self.cmd_candidate = Twist()
        self.cmd_output = Twist()
        self.cmd_source = 'none'
        self.last_cmd_source = 'none'
        self.fixed_route_mode = False
        self.topology_phase = 0
        self.bspline_code = 0
        self.bspline_received_wall = None
        self.reference_points = []
        self.reference_version = 0
        self.topology_version = 0
        self.collision_state = 'UNKNOWN'
        self.collision_reason = ''
        self.collision_scale = float('nan')
        self.last_collision_state = ''
        self.disk_warning_active = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/anav/task_status', String, self.task_status_callback,
                         queue_size=20)
        rospy.Subscriber('/Odometry', Odometry, self.localization_callback,
                         queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel/nav', Twist, self.cmd_nav_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel/candidate', ArbitratedCommand,
                         self.candidate_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_output_callback, queue_size=1)
        rospy.Subscriber('/anav/topology_safety_phase', UInt8,
                         self.topology_phase_callback, queue_size=1)
        rospy.Subscriber('/anav/fixed_route_mode', Bool,
                         self.fixed_route_callback, queue_size=1)
        rospy.Subscriber('/bspline_status', Vector3Stamped,
                         self.bspline_callback, queue_size=1)
        rospy.Subscriber('/reference_path', Path, self.reference_path_callback,
                         queue_size=1)
        rospy.Subscriber('/topology_plan', Path, self.topology_path_callback,
                         queue_size=1)
        rospy.Subscriber('/diagnostics', DiagnosticArray,
                         self.diagnostics_callback, queue_size=20)

        self.sample_timer = rospy.Timer(
            rospy.Duration(1.0 / self.sample_rate), self.sample_timer_callback
        )
        self.heartbeat_timer = rospy.Timer(
            rospy.Duration(1.0), self.heartbeat_timer_callback
        )
        rospy.on_shutdown(self.shutdown)
        self.record_event(
            'logger_restarted' if recovered_session else 'logger_started',
            state='RUNNING', detail=self.session_dir,
        )
        rospy.loginfo('Navigation benchmark logging to %s', self.session_dir)

    @staticmethod
    def _discover_repo_path(candidates):
        checked = set()
        for candidate in candidates:
            if not candidate:
                continue
            current = os.path.abspath(os.path.expanduser(candidate))
            if os.path.isfile(current):
                current = os.path.dirname(current)
            while current not in checked:
                checked.add(current)
                try:
                    top_level = subprocess.check_output(
                        ['git', '-C', current, 'rev-parse', '--show-toplevel'],
                        stderr=subprocess.DEVNULL, text=True, timeout=1.0,
                    ).strip()
                    if top_level:
                        return top_level
                except (OSError, subprocess.CalledProcessError,
                        subprocess.TimeoutExpired):
                    pass
                parent = os.path.dirname(current)
                if parent == current:
                    break
                current = parent
        return ''

    def _open_session(self):
        os.makedirs(self.results_root, exist_ok=True)
        marker_path = os.path.join(self.results_root, '.active_session.json')
        try:
            with open(marker_path, 'r', encoding='utf-8') as handle:
                marker = json.load(handle)
            age = time.time() - float(marker.get('heartbeat', 0.0))
            session_dir = marker.get('session_dir', '')
            if (not marker.get('clean_shutdown', True) and
                    0.0 <= age <= self.recovery_window and
                    os.path.isdir(session_dir) and
                    not process_is_alive(marker.get('pid'))):
                return marker['session_id'], session_dir, True
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        session_id = '{}_{}'.format(time.strftime('%Y%m%d_%H%M%S'), os.getpid())
        session_dir = os.path.join(self.results_root, session_id)
        os.makedirs(session_dir, exist_ok=False)
        return session_id, session_dir, False

    def _git_value(self, arguments):
        if not self.repo_path or not os.path.isdir(self.repo_path):
            return ''
        try:
            return subprocess.check_output(
                ['git', '-C', self.repo_path] + list(arguments),
                stderr=subprocess.DEVNULL, text=True, timeout=2.0,
            ).strip()
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return ''

    def _load_or_create_meta(self, recovered):
        if recovered:
            try:
                with open(self.meta_path, 'r', encoding='utf-8') as handle:
                    meta = json.load(handle)
            except (OSError, ValueError, json.JSONDecodeError):
                meta = {}
            meta['restart_count'] = int(meta.get('restart_count', 0)) + 1
            meta['last_restart_time'] = iso_time()
        else:
            fingerprints = {}
            for label, path in (
                ('robot_positions', self.positions_file),
                ('topology', self.topology_file),
            ):
                try:
                    fingerprints[label] = file_fingerprint(path)
                except OSError:
                    fingerprints[label] = ''
            meta = {
                'session_id': self.session_id,
                'start_time': iso_time(),
                'start_wall_time': time.time(),
                'end_time': None,
                'clean_shutdown': False,
                'restart_count': 0,
                'git_branch': self._git_value(['branch', '--show-current']),
                'git_commit': self._git_value(['rev-parse', 'HEAD']),
                'git_dirty': bool(self._git_value(['status', '--porcelain'])),
                'map_frame': self.map_frame,
                'base_frame': self.base_frame,
                'positions_file': self.positions_file,
                'topology_file': self.topology_file,
                'fingerprints': fingerprints,
                'sample_rate_hz': self.sample_rate,
                'settle_window_s': self.settle_window,
                'tag_metrics_enabled': False,
            }
        meta['clean_shutdown'] = False
        atomic_write_json(self.meta_path, meta)
        return meta

    def _load_task_rows(self):
        path = os.path.join(self.session_dir, 'tasks.csv')
        try:
            with open(path, 'r', encoding='utf-8', newline='') as handle:
                return list(csv.DictReader(handle))
        except OSError:
            return []

    def _restore_runtime_if_available(self, recovered):
        if not recovered:
            return
        try:
            with open(self.runtime_path, 'r', encoding='utf-8') as handle:
                state = json.load(handle)
            self.next_run_id = max(self.next_run_id, int(state.get('next_run_id', 1)))
            if state.get('active_task'):
                self.active_task = ActiveTask.restore(state['active_task'])
            pending = state.get('pending_arrival')
            if pending and pending.get('task'):
                self.pending_arrival = {
                    'task': ActiveTask.restore(pending['task']),
                    'result': pending.get('result', 'arrived'),
                    'detail': pending.get('detail', ''),
                    'finish_wall': float(pending.get('finish_wall', time.time())),
                    'arrival_pose': pending.get('arrival_pose'),
                    'settled_samples': [],
                    'deadline': time.monotonic() + self.settle_window,
                }
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            rospy.logwarn('Could not restore navigation benchmark runtime state')

    def _write_marker(self, clean_shutdown=False):
        atomic_write_json(self.marker_path, {
            'session_id': self.session_id,
            'session_dir': self.session_dir,
            'pid': os.getpid(),
            'heartbeat': time.time(),
            'clean_shutdown': bool(clean_shutdown),
        })

    def _write_runtime(self):
        pending = None
        if self.pending_arrival:
            pending = {
                'task': self.pending_arrival['task'].checkpoint(),
                'result': self.pending_arrival['result'],
                'detail': self.pending_arrival['detail'],
                'finish_wall': self.pending_arrival['finish_wall'],
                'arrival_pose': self.pending_arrival['arrival_pose'],
            }
        atomic_write_json(self.runtime_path, {
            'session_id': self.session_id,
            'updated_at': iso_time(),
            'next_run_id': self.next_run_id,
            'active_task': self.active_task.checkpoint() if self.active_task else None,
            'pending_arrival': pending,
        })

    def record_event(self, event_type, state='', reason='', source='', detail='',
                     path_version='', run_id=None):
        if run_id is None:
            run_id = self.active_task.run_id if self.active_task else ''
        self.events.append({
            'wall_time': iso_time(),
            'ros_time': number(rospy.Time.now().to_sec(), 9),
            'session_id': self.session_id,
            'run_id': run_id,
            'event_type': event_type,
            'state': state,
            'reason': reason,
            'source': source,
            'detail': detail,
            'path_version': path_version,
        }, force_sync=True)

    def localization_callback(self, _message):
        with self.lock:
            self.latest_localization_wall = time.monotonic()

    def odom_callback(self, message):
        with self.lock:
            self.odom = message.twist.twist

    def cmd_nav_callback(self, message):
        with self.lock:
            self.cmd_nav = message

    def cmd_output_callback(self, message):
        with self.lock:
            self.cmd_output = message

    def candidate_callback(self, message):
        with self.lock:
            self.cmd_candidate = message.command
            self.cmd_source = message.source or 'none'
            if self.cmd_source != self.last_cmd_source:
                previous = self.last_cmd_source
                self.last_cmd_source = self.cmd_source
                if self.active_task and self.cmd_source == 'teleop':
                    self.active_task.manual_intervention = True
                if self.active_task and self.cmd_source == 'safety':
                    self.active_task.estop_count += 1
                self.record_event(
                    'command_source_changed', state=self.cmd_source,
                    source=self.cmd_source, detail='{} -> {}'.format(
                        previous, self.cmd_source
                    ),
                )

    def topology_phase_callback(self, message):
        with self.lock:
            self.topology_phase = int(message.data)

    def fixed_route_callback(self, message):
        with self.lock:
            self.fixed_route_mode = bool(message.data)
            if self.active_task and self.fixed_route_mode:
                self.active_task.run_mode = 2

    def bspline_callback(self, message):
        with self.lock:
            previous = self.bspline_code
            self.bspline_code = int(round(message.vector.z))
            self.bspline_received_wall = time.monotonic()
            if self.bspline_code != previous:
                names = {1: 'ACTIVE', 2: 'PASSED', 3: 'DEVIATED'}
                self.record_event(
                    'bspline_status', state=names.get(self.bspline_code, str(self.bspline_code)),
                    detail='topology_version={} goal_index={}'.format(
                        int(round(message.vector.x)), int(round(message.vector.y))
                    ),
                    path_version=self.reference_version,
                )

    def diagnostics_callback(self, message):
        with self.lock:
            for status in message.status:
                if status.name != '/anav/collision_monitor':
                    continue
                values = {item.key: item.value for item in status.values}
                state = values.get('state', '')
                reason = values.get('reason', '')
                source = values.get('source', '')
                try:
                    scale = float(values.get('scale', 'nan'))
                except ValueError:
                    scale = float('nan')
                self.collision_state = state or 'UNKNOWN'
                self.collision_reason = reason
                self.collision_scale = scale
                if state != self.last_collision_state:
                    self.last_collision_state = state
                    if self.active_task:
                        if state == 'SLOWDOWN':
                            self.active_task.collision_slowdown_count += 1
                        elif state == 'COLLISION_STOP':
                            self.active_task.collision_stop_count += 1
                        elif state == 'EMERGENCY_STOP':
                            self.active_task.estop_count += 1
                    self.record_event(
                        'collision_state_changed', state=state, reason=reason,
                        source=source, detail=status.message,
                    )

    def _path_points(self, message):
        return [
            (pose.pose.position.x, pose.pose.position.y,
             quaternion_to_yaw(
                 pose.pose.orientation.x, pose.pose.orientation.y,
                 pose.pose.orientation.z, pose.pose.orientation.w,
             ))
            for pose in message.poses
        ]

    def _save_path(self, message, path_type, version):
        wall = iso_time()
        ros_stamp = message.header.stamp.to_sec() or rospy.Time.now().to_sec()
        run_id = self.active_task.run_id if self.active_task else ''
        points = self._path_points(message)
        for index, point in enumerate(points):
            self.paths.append({
                'wall_time': wall,
                'ros_time': number(ros_stamp, 9),
                'session_id': self.session_id,
                'run_id': run_id,
                'path_type': path_type,
                'path_version': version,
                'frame_id': message.header.frame_id,
                'point_index': index,
                'x': number(point[0]),
                'y': number(point[1]),
                'yaw_deg': number(math.degrees(point[2])),
            }, force_sync=index == len(points) - 1)
        return points

    def reference_path_callback(self, message):
        with self.lock:
            self.reference_version += 1
            self.reference_points = self._save_path(
                message, 'reference', self.reference_version
            )
            if self.active_task:
                self.active_task.reference_path_length = path_length(
                    self.reference_points
                )
            self.record_event(
                'reference_path_updated', state='UPDATED',
                detail='{} points'.format(len(self.reference_points)),
                path_version=self.reference_version,
            )

    def topology_path_callback(self, message):
        with self.lock:
            self.topology_version += 1
            points = self._save_path(message, 'topology', self.topology_version)
            if self.active_task:
                fingerprint = ';'.join(
                    '{:.4f},{:.4f}'.format(point[0], point[1]) for point in points
                )
                if self.active_task.topology_updates > 0:
                    self.active_task.observed_replan_count += 1
                self.active_task.topology_updates += 1
                self.active_task.last_topology_fingerprint = fingerprint
                self.active_task.topology_path_length = path_length(points)
                if points:
                    anchor = nearest_pose(self.poses, points[0][0], points[0][1])
                    if anchor:
                        self.active_task.start_node = dict(anchor)
            self.record_event(
                'topology_path_updated', state='UPDATED',
                detail='{} points'.format(len(points)),
                path_version=self.topology_version,
            )

    def _current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.03)
        )
        rotation = transform.transform.rotation
        pose = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )
        stamp = transform.header.stamp.to_sec()
        age_ms = max(0.0, (rospy.Time.now().to_sec() - stamp) * 1000.0) if stamp else float('nan')
        return pose, age_ms

    def task_status_callback(self, message):
        fields = message.data.split('\t', 3)
        if len(fields) != 4:
            rospy.logwarn('Invalid benchmark task status: %r', message.data)
            return
        state, requested_text, target_name, detail = fields
        try:
            requested_id = int(requested_text)
        except ValueError:
            rospy.logwarn('Invalid benchmark target id: %r', requested_text)
            return
        with self.lock:
            if state == 'running':
                if (self.active_task and self.active_task.recovered and
                        self.active_task.requested_id == requested_id):
                    self.record_event(
                        'task_recovered', state='running', detail=detail,
                        run_id=self.active_task.run_id,
                    )
                    return
                if self.pending_arrival:
                    self._finalize_pending_arrival(force=True)
                if self.active_task:
                    self._finalize_task(
                        self.active_task, 'incomplete',
                        'new task started before terminal status', time.time(),
                    )
                    self.active_task = None
                goal = resolve_requested_pose(self.poses, requested_id)
                start_pose = self.latest_pose
                start_node = nearest_pose(
                    self.poses, start_pose[0], start_pose[1]
                ) if start_pose else None
                self.active_task = ActiveTask(
                    self.next_run_id, requested_id, goal, target_name,
                    time.time(), start_pose, start_node,
                    2 if self.fixed_route_mode else 1,
                )
                self.next_run_id += 1
                self.record_event('task_started', state=state, detail=detail)
                self._write_runtime()
                return

            if state not in ('arrived', 'failed', 'canceled'):
                self.record_event('task_status', state=state, detail=detail)
                return
            if (self.pending_arrival and
                    self.pending_arrival['task'].requested_id == requested_id and
                    state == self.pending_arrival['result']):
                self.record_event(
                    'duplicate_terminal_status', state=state, detail=detail,
                    run_id=self.pending_arrival['task'].run_id,
                )
                return
            if not self.active_task:
                self.record_event(
                    'orphan_terminal_status', state=state, detail=detail,
                    run_id='',
                )
                return
            task = self.active_task
            self.active_task = None
            finish_wall = time.time()
            self.record_event(
                'task_{}'.format(state), state=state, detail=detail,
                run_id=task.run_id,
            )
            if state == 'arrived' and self.settle_window > 0.0:
                self.pending_arrival = {
                    'task': task,
                    'result': state,
                    'detail': detail,
                    'finish_wall': finish_wall,
                    'arrival_pose': self.latest_pose,
                    'settled_samples': [],
                    'deadline': time.monotonic() + self.settle_window,
                }
            else:
                self._finalize_task(task, state, detail, finish_wall)
            self._write_runtime()

    def _task_identity(self, task):
        start = task.start_node or {}
        goal = task.goal or {}
        start_name = start.get('name', 'unknown')
        goal_name = goal.get('name', task.target_name or 'unknown')
        return start, goal, start_name, goal_name

    def _pose_errors(self, pose, goal):
        if not pose or not goal:
            return float('nan'), float('nan')
        return (
            math.hypot(pose[0] - goal['x'], pose[1] - goal['y']),
            abs(math.degrees(wrap_angle(pose[2] - goal['yaw']))),
        )

    def _finalize_pending_arrival(self, force=False):
        pending = self.pending_arrival
        if not pending:
            return
        if not force and time.monotonic() < pending['deadline']:
            return
        samples = pending['settled_samples']
        settled_pose = None
        if samples:
            settled_pose = (
                statistics.median(sample[0] for sample in samples),
                statistics.median(sample[1] for sample in samples),
                circular_mean(sample[2] for sample in samples),
            )
        self._finalize_task(
            pending['task'], pending['result'], pending['detail'],
            pending['finish_wall'], pending['arrival_pose'], settled_pose,
        )
        self.pending_arrival = None

    def _finalize_task(self, task, result, detail, finish_wall,
                       arrival_pose=None, settled_pose=None):
        start, goal, start_name, goal_name = self._task_identity(task)
        arrival_pose = arrival_pose or task.last_pose
        settled_pose = settled_pose or arrival_pose
        arrival_position_error, arrival_yaw_error = self._pose_errors(
            arrival_pose, goal
        )
        settled_position_error, settled_yaw_error = self._pose_errors(
            settled_pose, goal
        )
        if task.localization_gap_started is not None:
            gap = time.monotonic() - task.localization_gap_started
            if gap >= self.localization_gap_threshold:
                task.localization_gap_count += 1
                task.max_localization_gap = max(task.max_localization_gap, gap)
        mean_cte = task.cte_sum / task.cte_count if task.cte_count else float('nan')
        rmse_cte = math.sqrt(task.cte_sum_sq / task.cte_count) if task.cte_count else float('nan')
        extra_path_ratio = float('nan')
        if finite(task.reference_path_length) and task.reference_path_length > 1e-6:
            extra_path_ratio = (
                task.actual_length - task.reference_path_length
            ) / task.reference_path_length
        row = {
            'session_id': self.session_id,
            'run_id': task.run_id,
            'direction': '{}->{}'.format(start_name, goal_name),
            'start_id': start.get('index', ''),
            'start_name': start_name,
            'goal_id': goal.get('index', task.requested_id),
            'goal_name': goal_name,
            'run_mode': task.run_mode,
            'start_time': iso_time(task.start_wall),
            'finish_time': iso_time(finish_wall),
            'duration_s': number(finish_wall - task.start_wall, 3),
            'result': result,
            'failure_reason': '' if result == 'arrived' else detail,
            'manual_intervention': bool_text(task.manual_intervention),
            'recovered_after_logger_restart': bool_text(task.recovered),
            'data_complete': bool_text(
                not task.recovered and task.localization_gap_count == 0
            ),
            'start_x': number(task.start_pose[0]) if task.start_pose else '',
            'start_y': number(task.start_pose[1]) if task.start_pose else '',
            'start_yaw_deg': number(math.degrees(task.start_pose[2])) if task.start_pose else '',
            'goal_x': number(goal.get('x')),
            'goal_y': number(goal.get('y')),
            'goal_yaw_deg': number(math.degrees(goal['yaw'])) if finite(goal.get('yaw')) else '',
            'arrival_x': number(arrival_pose[0]) if arrival_pose else '',
            'arrival_y': number(arrival_pose[1]) if arrival_pose else '',
            'arrival_yaw_deg': number(math.degrees(arrival_pose[2])) if arrival_pose else '',
            'arrival_position_error_m': number(arrival_position_error),
            'arrival_yaw_error_deg': number(arrival_yaw_error),
            'settled_x': number(settled_pose[0]) if settled_pose else '',
            'settled_y': number(settled_pose[1]) if settled_pose else '',
            'settled_yaw_deg': number(math.degrees(settled_pose[2])) if settled_pose else '',
            'settled_position_error_m': number(settled_position_error),
            'settled_yaw_error_deg': number(settled_yaw_error),
            'settle_window_s': number(self.settle_window, 3),
            'topology_path_length_m': number(task.topology_path_length),
            'reference_path_length_m': number(task.reference_path_length),
            'actual_path_length_m': number(task.actual_length),
            'extra_path_ratio': number(extra_path_ratio),
            'cte_sample_count': task.cte_count,
            'cte_sum_m': number(task.cte_sum),
            'cte_sum_sq_m2': number(task.cte_sum_sq),
            'mean_cte_m': number(mean_cte),
            'rmse_cte_m': number(rmse_cte),
            'p95_cte_m': number(percentile(task.cte_values, 95.0)),
            'max_cte_m': number(max(task.cte_values) if task.cte_values else float('nan')),
            'max_abs_angular_velocity': number(task.max_abs_omega),
            'rms_angular_acceleration': number(root_mean_square(task.angular_accelerations)),
            'sharp_turn_count': task.sharp_turn_count,
            'unexpected_stop_count': task.unexpected_stop_count,
            'observed_replan_count': task.observed_replan_count,
            'collision_slowdown_count': task.collision_slowdown_count,
            'collision_stop_count': task.collision_stop_count,
            'estop_count': task.estop_count,
            'localization_gap_count': task.localization_gap_count,
            'max_localization_gap_s': number(task.max_localization_gap, 3),
        }
        self.tasks.append(row, force_sync=True)
        self.task_rows.append({key: str(value) for key, value in row.items()})
        self._write_live_summary()

    def _sample_row(self, pose, tf_age_ms, valid):
        task = self.active_task
        start, goal, start_name, goal_name = self._task_identity(task) if task else ({}, {}, '', '')
        localization_age_ms = (
            (time.monotonic() - self.latest_localization_wall) * 1000.0
            if self.latest_localization_wall is not None else float('nan')
        )
        reference = None
        tracking = False
        if (valid and self.bspline_code == 1 and
                self.bspline_received_wall is not None and
                time.monotonic() - self.bspline_received_wall <= self.bspline_status_timeout and
                self.cmd_source == 'nav' and
                abs(self.odom.linear.x) >= self.tracking_min_speed):
            reference = closest_point_on_polyline(
                pose[0], pose[1], self.reference_points
            )
            tracking = reference is not None
        goal_position_error, goal_yaw_error = self._pose_errors(pose, goal)
        row = {
            'wall_time': iso_time(),
            'ros_time': number(rospy.Time.now().to_sec(), 9),
            'session_id': self.session_id,
            'run_id': task.run_id if task else '',
            'task_state': 'running' if task else ('settling' if self.pending_arrival else 'idle'),
            'start_id': start.get('index', ''),
            'start_name': start_name,
            'goal_id': goal.get('index', task.requested_id if task else ''),
            'goal_name': goal_name,
            'run_mode': task.run_mode if task else '',
            'x': number(pose[0]) if valid else '',
            'y': number(pose[1]) if valid else '',
            'yaw_deg': number(math.degrees(pose[2])) if valid else '',
            'tf_age_ms': number(tf_age_ms, 3),
            'localization_age_ms': number(localization_age_ms, 3),
            'odom_vx': number(self.odom.linear.x),
            'odom_vy': number(self.odom.linear.y),
            'odom_wz': number(self.odom.angular.z),
            'cmd_nav_vx': number(self.cmd_nav.linear.x),
            'cmd_nav_wz': number(self.cmd_nav.angular.z),
            'cmd_source': self.cmd_source,
            'cmd_candidate_vx': number(self.cmd_candidate.linear.x),
            'cmd_candidate_vy': number(self.cmd_candidate.linear.y),
            'cmd_candidate_wz': number(self.cmd_candidate.angular.z),
            'cmd_output_vx': number(self.cmd_output.linear.x),
            'cmd_output_vy': number(self.cmd_output.linear.y),
            'cmd_output_wz': number(self.cmd_output.angular.z),
            'topology_phase': self.topology_phase,
            'bspline_status': self.bspline_code,
            'reference_version': self.reference_version if reference else '',
            'reference_segment': reference['segment_index'] if reference else '',
            'reference_x': number(reference['x']) if reference else '',
            'reference_y': number(reference['y']) if reference else '',
            'cte_signed_m': number(reference['signed']) if reference else '',
            'cte_abs_m': number(reference['absolute']) if reference else '',
            'goal_x': number(goal.get('x')),
            'goal_y': number(goal.get('y')),
            'goal_yaw_deg': number(math.degrees(goal['yaw'])) if finite(goal.get('yaw')) else '',
            'goal_position_error_m': number(goal_position_error),
            'goal_yaw_error_deg': number(goal_yaw_error),
            'collision_state': self.collision_state,
            'collision_reason': self.collision_reason,
            'collision_scale': number(self.collision_scale),
            'sample_valid': bool_text(valid),
        }
        return row, tracking, reference

    def sample_timer_callback(self, _event):
        if self.shutdown_started:
            return
        with self.lock:
            valid = False
            pose = self.latest_pose or (float('nan'), float('nan'), float('nan'))
            tf_age_ms = float('nan')
            try:
                pose, tf_age_ms = self._current_pose()
                valid = all(finite(value) for value in pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
            now_mono = time.monotonic()
            if valid:
                self.latest_pose = pose
                self.latest_pose_wall = now_mono
                if self.active_task:
                    if self.active_task.localization_gap_started is not None:
                        gap = now_mono - self.active_task.localization_gap_started
                        if gap >= self.localization_gap_threshold:
                            self.active_task.localization_gap_count += 1
                            self.active_task.max_localization_gap = max(
                                self.active_task.max_localization_gap, gap
                            )
                            self.record_event(
                                'localization_recovered', state='RECOVERED',
                                detail='{:.3f} s gap'.format(gap),
                            )
                        self.active_task.localization_gap_started = None
                    self.active_task.update_pose(pose, self.max_pose_jump)
            elif self.active_task and self.active_task.localization_gap_started is None:
                self.active_task.localization_gap_started = now_mono
                self.record_event('localization_lost', state='LOST')

            row, tracking, reference = self._sample_row(pose, tf_age_ms, valid)
            self.samples.append(row)
            if self.active_task:
                if tracking:
                    self.active_task.update_tracking(
                        reference['absolute'], self.odom.angular.z,
                        self.odom.linear.x, now_mono, self.collision_state,
                        self.stop_speed, self.stop_duration,
                        self.sharp_omega, self.sharp_duration,
                    )
                else:
                    self.active_task.stop_tracking_window()
            if self.pending_arrival and valid:
                stopped = (
                    math.hypot(self.odom.linear.x, self.odom.linear.y) < self.stop_speed and
                    abs(self.odom.angular.z) < self.stop_speed
                )
                if stopped:
                    self.pending_arrival['settled_samples'].append(pose)
                self._finalize_pending_arrival()

    def _write_live_summary(self):
        rows = self.task_rows
        arrived = [row for row in rows if row.get('result') == 'arrived']
        failed = [row for row in rows if row.get('result') == 'failed']
        canceled = [row for row in rows if row.get('result') == 'canceled']
        incomplete = [row for row in rows if row.get('result') == 'incomplete']
        position_errors = [row.get('settled_position_error_m') for row in arrived]
        yaw_errors = [row.get('settled_yaw_error_deg') for row in arrived]
        position_errors = [float(value) for value in position_errors if finite(value)]
        yaw_errors = [float(value) for value in yaw_errors if finite(value)]
        autonomous_attempts = len(arrived) + len(failed)
        streak = 0
        longest_streak = 0
        for row in rows:
            if row.get('result') == 'arrived' and row.get('manual_intervention') != 'true':
                streak += 1
                longest_streak = max(longest_streak, streak)
            else:
                streak = 0
        by_direction = {}
        for row in rows:
            direction = row.get('direction', 'unknown')
            entry = by_direction.setdefault(direction, {'total': 0, 'arrived': 0, 'failed': 0})
            entry['total'] += 1
            if row.get('result') in entry:
                entry[row.get('result')] += 1
        summary = {
            'session_id': self.session_id,
            'updated_at': iso_time(),
            'session_directory': self.session_dir,
            'tasks_total': len(rows),
            'arrived': len(arrived),
            'failed': len(failed),
            'canceled': len(canceled),
            'incomplete': len(incomplete),
            'success_rate_excluding_canceled': (
                len(arrived) / autonomous_attempts if autonomous_attempts else None
            ),
            'completion_rate_all': len(arrived) / len(rows) if rows else None,
            'longest_success_streak': longest_streak,
            'manual_intervention_tasks': sum(
                row.get('manual_intervention') == 'true' for row in rows
            ),
            'settled_position_error_m': summarize(position_errors),
            'settled_yaw_error_deg': summarize(yaw_errors),
            'by_direction': by_direction,
            'active_run_id': self.active_task.run_id if self.active_task else None,
        }
        atomic_write_json(self.summary_path, summary)

    def heartbeat_timer_callback(self, _event):
        if self.shutdown_started:
            return
        with self.lock:
            self._write_marker(clean_shutdown=False)
            self._write_runtime()
            self._write_live_summary()
            free_mb = shutil.disk_usage(self.session_dir).free / (1024.0 * 1024.0)
            low = free_mb < self.disk_warning_mb
            if low != self.disk_warning_active:
                self.disk_warning_active = low
                self.record_event(
                    'disk_space', state='LOW' if low else 'RECOVERED',
                    detail='{:.1f} MB free'.format(free_mb),
                )

    def shutdown(self):
        with self.lock:
            if self.shutdown_started:
                return
            self.shutdown_started = True
            try:
                if self.pending_arrival:
                    self._finalize_pending_arrival(force=True)
                if self.active_task:
                    task = self.active_task
                    self.active_task = None
                    self._finalize_task(
                        task, 'incomplete', 'logger stopped during active task',
                        time.time(),
                    )
                self.record_event('logger_stopped', state='STOPPED', run_id='')
                self.meta['end_time'] = iso_time()
                self.meta['clean_shutdown'] = True
                atomic_write_json(self.meta_path, self.meta)
                self._write_runtime()
                self._write_live_summary()
                self._write_marker(clean_shutdown=True)
            finally:
                for writer in (self.samples, self.events, self.paths, self.tasks):
                    try:
                        writer.close()
                    except OSError:
                        pass


def main():
    rospy.init_node('navigation_benchmark')
    try:
        NavigationBenchmark()
    except Exception as error:
        rospy.logfatal('Navigation benchmark initialization failed: %s', error)
        raise
    rospy.spin()


if __name__ == '__main__':
    main()
