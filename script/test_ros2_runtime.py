from pathlib import Path

from ros2_runtime import parse_service_message, parse_service_success


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
