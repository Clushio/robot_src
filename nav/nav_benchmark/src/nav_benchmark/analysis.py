import csv
import json
import os
import time

from nav_benchmark.metrics import finite, summarize
from nav_benchmark.storage import atomic_write_json


def read_csv(path):
    try:
        with open(path, 'r', encoding='utf-8', newline='') as handle:
            return list(csv.DictReader(handle))
    except OSError:
        return []


def numeric(rows, field, predicate=lambda _row: True):
    values = []
    for row in rows:
        value = row.get(field)
        if predicate(row) and finite(value):
            values.append(float(value))
    return values


def build_summary(directories):
    tasks = []
    samples = []
    sessions = []
    for directory in directories:
        tasks.extend(read_csv(os.path.join(directory, 'tasks.csv')))
        samples.extend(read_csv(os.path.join(directory, 'samples.csv')))
        try:
            with open(os.path.join(directory, 'session.json'),
                      'r', encoding='utf-8') as handle:
                sessions.append(json.load(handle))
        except (OSError, ValueError, json.JSONDecodeError):
            sessions.append({'session_id': os.path.basename(directory)})

    arrived = [row for row in tasks if row.get('result') == 'arrived']
    failed = [row for row in tasks if row.get('result') == 'failed']
    canceled = [row for row in tasks if row.get('result') == 'canceled']
    incomplete = [row for row in tasks if row.get('result') == 'incomplete']
    autonomous_attempts = len(arrived) + len(failed)
    cte = numeric(
        samples, 'cte_abs_m',
        lambda row: row.get('task_state') == 'running',
    )

    longest_streak = 0
    current_streak = 0
    by_direction = {}
    for row in tasks:
        direction = row.get('direction', 'unknown')
        group = by_direction.setdefault(direction, {
            'total': 0, 'arrived': 0, 'failed': 0, 'canceled': 0,
            'position_errors_m': [], 'yaw_errors_deg': [],
        })
        group['total'] += 1
        result = row.get('result')
        if result in ('arrived', 'failed', 'canceled'):
            group[result] += 1
        if result == 'arrived':
            if finite(row.get('settled_position_error_m')):
                group['position_errors_m'].append(
                    float(row['settled_position_error_m'])
                )
            if finite(row.get('settled_yaw_error_deg')):
                group['yaw_errors_deg'].append(
                    float(row['settled_yaw_error_deg'])
                )
        if result == 'arrived' and row.get('manual_intervention') != 'true':
            current_streak += 1
            longest_streak = max(longest_streak, current_streak)
        else:
            current_streak = 0

    for group in by_direction.values():
        attempts = group['arrived'] + group['failed']
        group['success_rate_excluding_canceled'] = (
            group['arrived'] / attempts if attempts else None
        )
        group['position_error_m'] = summarize(
            group.pop('position_errors_m')
        )
        group['yaw_error_deg'] = summarize(group.pop('yaw_errors_deg'))

    return {
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'session_directories': directories,
        'session_ids': [session.get('session_id') for session in sessions],
        'tasks_total': len(tasks),
        'arrived': len(arrived),
        'failed': len(failed),
        'canceled': len(canceled),
        'incomplete': len(incomplete),
        'success_rate_excluding_canceled': (
            len(arrived) / autonomous_attempts if autonomous_attempts else None
        ),
        'completion_rate_all': len(arrived) / len(tasks) if tasks else None,
        'longest_success_streak': longest_streak,
        'manual_intervention_tasks': sum(
            row.get('manual_intervention') == 'true' for row in tasks
        ),
        'settled_position_error_m': summarize(numeric(
            arrived, 'settled_position_error_m'
        )),
        'settled_yaw_error_deg': summarize(numeric(
            arrived, 'settled_yaw_error_deg'
        )),
        'bspline_cte_m': summarize(cte),
        'observed_replan_count': int(sum(numeric(
            tasks, 'observed_replan_count'
        ))),
        'collision_slowdown_count': int(sum(numeric(
            tasks, 'collision_slowdown_count'
        ))),
        'collision_stop_count': int(sum(numeric(
            tasks, 'collision_stop_count'
        ))),
        'unexpected_stop_count': int(sum(numeric(
            tasks, 'unexpected_stop_count'
        ))),
        'sharp_turn_count': int(sum(numeric(tasks, 'sharp_turn_count'))),
        'localization_gap_count': int(sum(numeric(
            tasks, 'localization_gap_count'
        ))),
        'by_direction': by_direction,
    }


def write_text(path, summary):
    position = summary['settled_position_error_m']
    yaw = summary['settled_yaw_error_deg']
    cte = summary['bspline_cte_m']

    def percent(value):
        return 'N/A' if value is None else '{:.2f}%'.format(value * 100.0)

    def metric(group, key, scale=1.0, suffix=''):
        value = group.get(key)
        return (
            'N/A' if value is None
            else '{:.3f}{}'.format(value * scale, suffix)
        )

    lines = [
        'Navigation benchmark summary',
        'Sessions: {}'.format(', '.join(summary['session_ids'])),
        'Tasks: {} (arrived {}, failed {}, canceled {}, incomplete {})'.format(
            summary['tasks_total'], summary['arrived'], summary['failed'],
            summary['canceled'], summary['incomplete'],
        ),
        'Success rate excluding canceled: {}'.format(
            percent(summary['success_rate_excluding_canceled'])
        ),
        'Completion rate for all tasks: {}'.format(
            percent(summary['completion_rate_all'])
        ),
        'Longest success streak: {}'.format(summary['longest_success_streak']),
        'Settled position error mean/P95/max: {} / {} / {}'.format(
            metric(position, 'mean', 100.0, ' cm'),
            metric(position, 'p95', 100.0, ' cm'),
            metric(position, 'max', 100.0, ' cm'),
        ),
        'Settled yaw error mean/P95/max: {} / {} / {}'.format(
            metric(yaw, 'mean', 1.0, ' deg'),
            metric(yaw, 'p95', 1.0, ' deg'),
            metric(yaw, 'max', 1.0, ' deg'),
        ),
        'B-spline CTE mean/RMSE/P95/max: {} / {} / {} / {}'.format(
            metric(cte, 'mean', 100.0, ' cm'),
            metric(cte, 'rmse', 100.0, ' cm'),
            metric(cte, 'p95', 100.0, ' cm'),
            metric(cte, 'max', 100.0, ' cm'),
        ),
        'Observed replans: {}'.format(summary['observed_replan_count']),
        'Collision slowdown/stops: {}/{}'.format(
            summary['collision_slowdown_count'],
            summary['collision_stop_count'],
        ),
        'Manual-intervention tasks: {}'.format(
            summary['manual_intervention_tasks']
        ),
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')


def generate_report(directories, output_dir=None):
    directories = [
        os.path.abspath(os.path.expanduser(path)) for path in directories
    ]
    if not directories:
        raise ValueError('at least one session directory is required')
    destination = (
        os.path.abspath(os.path.expanduser(output_dir))
        if output_dir else directories[0]
    )
    os.makedirs(destination, exist_ok=True)
    summary = build_summary(directories)
    json_path = os.path.join(destination, 'summary.json')
    text_path = os.path.join(destination, 'summary.txt')
    atomic_write_json(json_path, summary)
    write_text(text_path, summary)
    return json_path, text_path
