import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "tcpserver.py"
SPEC = importlib.util.spec_from_file_location("robot_r_tcpserver", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_ros1_and_ros2_success_formats():
    assert MODULE.TCPServer.parse_service_success(b"success: true\n")
    assert MODULE.TCPServer.parse_service_success(
        b"x2bot_teleop.srv.SetInt_Response(success=True, message='ok')"
    )
    assert not MODULE.TCPServer.parse_service_success(b"success=False")
