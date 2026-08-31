from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from pathRecord import calculate_average_distance


def make_path(points):
    result = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        result.poses.append(pose)
    return result


def test_average_distance_keeps_original_nearest_pose_semantics():
    first = make_path([(0.0, 0.0), (1.0, 0.0)])
    second = make_path([(0.0, 1.0), (1.0, 1.0)])
    assert calculate_average_distance(first, second, num_samples=30) == 1.0
