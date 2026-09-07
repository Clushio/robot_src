from pathlib import Path

from ros2_runtime import (
    navigation_is_ready,
    parse_service_message,
    parse_service_success,
)


class FakeNode:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_node_names_and_namespaces(self):
        return self.nodes


class FakeActionClient:
    def __init__(self, ready):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


def test_runtime_uses_ros2_without_ros1_master_api():
    text = (Path(__file__).parent / 'ros2_runtime.py').read_text()
    assert 'import rclpy' in text
    assert 'rospy' not in text
    assert 'roscore' not in text


def test_parse_ros1_and_ros2_service_output_formats():
    assert parse_service_success('success: true\nmessage: "arrived"')
    assert parse_service_success(
        "SetInt_Response(success=True, message='arrived: P2')"
    )
    assert not parse_service_success(
        "SetInt_Response(success=False, message='blocked')"
    )
    assert parse_service_message(
        "SetInt_Response(success=True, message='arrived: P2')"
    ) == 'arrived: P2'


def test_navigation_requires_compat_node_and_nav2_action_server():
    node = FakeNode([('mxb_move_base', '/')])
    assert navigation_is_ready(node, FakeActionClient(True))
    assert not navigation_is_ready(node, FakeActionClient(False))
    assert not navigation_is_ready(FakeNode([]), FakeActionClient(True))
    assert not navigation_is_ready(None, FakeActionClient(True))
