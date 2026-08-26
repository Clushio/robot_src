#!/usr/bin/env python3

import hashlib
import math
import os
import statistics


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def wrap_angle(angle):
    """Normalize radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def circular_mean(values):
    values = [float(value) for value in values if finite(value)]
    if not values:
        return float('nan')
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def percentile(values, percentage):
    values = sorted(float(value) for value in values if finite(value))
    if not values:
        return float('nan')
    if len(values) == 1:
        return values[0]
    percentage = min(100.0, max(0.0, float(percentage)))
    rank = percentage / 100.0 * (len(values) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return values[lower]
    fraction = rank - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def root_mean_square(values):
    values = [float(value) for value in values if finite(value)]
    if not values:
        return float('nan')
    return math.sqrt(sum(value * value for value in values) / len(values))


def path_length(points):
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += math.hypot(second[0] - first[0], second[1] - first[1])
    return total


def closest_point_on_polyline(x, y, points):
    """Return signed/absolute CTE and closest point on a directed polyline."""
    if len(points) < 2:
        return None
    best = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        ax, ay = first[:2]
        bx, by = second[:2]
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        projection = ((x - ax) * dx + (y - ay) * dy) / length_sq
        projection = min(1.0, max(0.0, projection))
        closest_x = ax + projection * dx
        closest_y = ay + projection * dy
        offset_x = x - closest_x
        offset_y = y - closest_y
        absolute = math.hypot(offset_x, offset_y)
        cross = dx * (y - ay) - dy * (x - ax)
        signed = math.copysign(absolute, cross) if absolute else 0.0
        candidate = (absolute, signed, closest_x, closest_y, index)
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        return None
    absolute, signed, closest_x, closest_y, index = best
    return {
        'absolute': absolute,
        'signed': signed,
        'x': closest_x,
        'y': closest_y,
        'segment_index': index,
    }


def parse_robot_positions(path):
    poses = []
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as handle:
        for raw_line in handle:
            line = raw_line.split('#', 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 6:
                raise ValueError('invalid robot position line: {}'.format(raw_line.rstrip()))
            numeric = [float(value) for value in fields[:6]]
            if not all(finite(value) for value in numeric):
                raise ValueError('non-finite robot position line: {}'.format(raw_line.rstrip()))
            index = len(poses)
            label = fields[6] if len(fields) >= 7 else ''
            poses.append({
                'index': index,
                'name': label or 'P{}'.format(index),
                'label': label,
                'x': numeric[0],
                'y': numeric[1],
                'yaw': numeric[5],
            })
    return poses


def resolve_requested_pose(poses, requested_id):
    requested_id = int(requested_id)
    if requested_id >= 0:
        return poses[requested_id] if requested_id < len(poses) else None
    label = 'W{}'.format(-requested_id)
    return next((pose for pose in poses if pose['label'] == label), None)


def nearest_pose(poses, x, y):
    if not poses or not finite(x) or not finite(y):
        return None
    return min(poses, key=lambda pose: math.hypot(x - pose['x'], y - pose['y']))


def file_fingerprint(path):
    path = os.path.expanduser(path)
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def summarize(values):
    values = [float(value) for value in values if finite(value)]
    if not values:
        return {
            'count': 0,
            'mean': None,
            'rmse': None,
            'p95': None,
            'max': None,
        }
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'rmse': root_mean_square(values),
        'p95': percentile(values, 95.0),
        'max': max(values),
    }
