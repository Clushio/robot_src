import math


def select_loop_start(targets, current_x, current_y, localization_age,
                      near_distance, pose_timeout=0.30):
    """Choose the first real loop target from the current map position.

    Returns ``(target_offset, anchor_offset, distance_a, distance_b)``.
    ``anchor_offset`` is ``None`` when the robot is not already near either
    endpoint. A nearby endpoint is only the uncounted starting anchor; the
    opposite endpoint becomes the first completed leg.
    """
    if len(targets) != 2:
        raise ValueError('loop navigation requires exactly two endpoints')
    if not all(math.isfinite(value) for value in (
            current_x, current_y, localization_age,
            near_distance, pose_timeout)):
        raise ValueError('loop start position and distance must be finite')
    if pose_timeout <= 0.0:
        raise ValueError('loop start pose timeout must be positive')
    if localization_age < 0.0 or localization_age > pose_timeout:
        raise ValueError('loop start localization is stale')
    if near_distance <= 0.0:
        raise ValueError('loop start near distance must be positive')

    distances = []
    for target in targets:
        target_x = float(target['x'])
        target_y = float(target['y'])
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            raise ValueError('loop endpoint position must be finite')
        distances.append(math.hypot(current_x - target_x,
                                    current_y - target_y))

    nearest_offset = 0 if distances[0] <= distances[1] else 1
    if distances[nearest_offset] <= near_distance:
        return 1 - nearest_offset, nearest_offset, distances[0], distances[1]
    return 0, None, distances[0], distances[1]
