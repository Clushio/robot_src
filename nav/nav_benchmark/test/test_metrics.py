#!/usr/bin/env python3

import math
import os
import tempfile
import unittest
import json

from nav_benchmark.analysis import generate_report
from nav_benchmark.metrics import (
    closest_point_on_polyline,
    parse_robot_positions,
    percentile,
    resolve_requested_pose,
    wrap_angle,
)
from nav_benchmark.storage import DurableCsv, atomic_write_json


class MetricsTest(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(-math.pi / 2.0, wrap_angle(3.0 * math.pi / 2.0))
        self.assertAlmostEqual(math.pi / 2.0, wrap_angle(-3.0 * math.pi / 2.0))

    def test_percentile_interpolates(self):
        self.assertAlmostEqual(3.85, percentile([0, 1, 2, 3, 4], 96.25))

    def test_closest_point_uses_segments_not_samples(self):
        result = closest_point_on_polyline(0.5, 0.2, [(0.0, 0.0), (1.0, 0.0)])
        self.assertAlmostEqual(0.2, result['absolute'])
        self.assertAlmostEqual(0.2, result['signed'])
        self.assertAlmostEqual(0.5, result['x'])
        self.assertEqual(0, result['segment_index'])

    def test_signed_cte_changes_side(self):
        above = closest_point_on_polyline(0.5, 0.1, [(0.0, 0.0), (1.0, 0.0)])
        below = closest_point_on_polyline(0.5, -0.1, [(0.0, 0.0), (1.0, 0.0)])
        self.assertGreater(above['signed'], 0.0)
        self.assertLess(below['signed'], 0.0)

    def test_workstation_request_resolution(self):
        descriptor, path = tempfile.mkstemp(text=True)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
                handle.write('1 2 0 0 0 0 W1\n')
                handle.write('3 4 0 0 0 1.57\n')
            poses = parse_robot_positions(path)
            self.assertEqual('W1', resolve_requested_pose(poses, -1)['name'])
            self.assertEqual('P1', resolve_requested_pose(poses, 1)['name'])
            self.assertIsNone(resolve_requested_pose(poses, -2))
        finally:
            os.unlink(path)

    def test_durable_csv_and_atomic_json(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, 'rows.csv')
            json_path = os.path.join(directory, 'state.json')
            writer = DurableCsv(csv_path, ['a', 'b'], sync_interval=10.0)
            try:
                writer.append({'a': 1, 'b': 'two'}, force_sync=True)
            finally:
                writer.close()
            atomic_write_json(json_path, {'ok': True, 'not_a_number': float('nan')})
            with open(csv_path, 'r', encoding='utf-8') as handle:
                self.assertEqual(['a,b', '1,two'], handle.read().splitlines())
            with open(json_path, 'r', encoding='utf-8') as handle:
                self.assertEqual(
                    {'ok': True, 'not_a_number': None}, json.load(handle)
                )

    def test_generate_report_writes_final_files(self):
        with tempfile.TemporaryDirectory() as directory:
            atomic_write_json(
                os.path.join(directory, 'session.json'),
                {'session_id': 'test-session'},
            )
            tasks = DurableCsv(
                os.path.join(directory, 'tasks.csv'),
                [
                    'result', 'direction', 'manual_intervention',
                    'settled_position_error_m', 'settled_yaw_error_deg',
                    'observed_replan_count', 'collision_slowdown_count',
                    'collision_stop_count', 'unexpected_stop_count',
                    'sharp_turn_count', 'localization_gap_count',
                ],
                sync_interval=10.0,
            )
            samples = DurableCsv(
                os.path.join(directory, 'samples.csv'),
                ['task_state', 'cte_abs_m'],
                sync_interval=10.0,
            )
            try:
                tasks.append({
                    'result': 'arrived',
                    'direction': 'W1->W2',
                    'manual_intervention': 'false',
                    'settled_position_error_m': 0.04,
                    'settled_yaw_error_deg': 1.5,
                    'observed_replan_count': 1,
                }, force_sync=True)
                samples.append({
                    'task_state': 'running', 'cte_abs_m': 0.03,
                }, force_sync=True)
            finally:
                tasks.close()
                samples.close()

            json_path, text_path = generate_report([directory])
            self.assertTrue(os.path.isfile(json_path))
            self.assertTrue(os.path.isfile(text_path))
            with open(json_path, 'r', encoding='utf-8') as handle:
                summary = json.load(handle)
            self.assertEqual(1, summary['tasks_total'])
            self.assertEqual(1.0, summary['success_rate_excluding_canceled'])
            self.assertEqual(0.03, summary['bspline_cte_m']['mean'])


if __name__ == '__main__':
    unittest.main()
