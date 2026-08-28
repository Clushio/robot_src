import importlib.util
from pathlib import Path

import rclpy


SCRIPT = Path(__file__).parents[1] / "scripts" / "mm3v_serial_reader.py"
SPEC = importlib.util.spec_from_file_location("mm3v_serial_reader", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_frame(reader):
    frame = bytearray(reader.FRAME_LEN)
    frame[0] = reader.START_BYTE
    frame[1] = reader.TAG_PRESENT
    frame[2] = 7
    frame[3:6] = bytes([0, 100, 0])
    frame[6:9] = bytes([0, 200, 0])
    frame[9:12] = bytes([0, 30, 0])
    frame[12:15] = bytes([0, 1, 0])
    frame[15:18] = bytes([0, 2, 0])
    frame[18:20] = bytes([0x03, 0x20])
    frame[20:22] = bytes([0x02, 0x58])
    frame[22] = reader.calculate_bcc(frame[2:22])
    frame[23] = reader.END_BYTE
    return frame


def test_valid_frame_and_bcc_rejection():
    rclpy.init()
    reader = MODULE.MM3VSerialReader()
    try:
        frame = make_frame(reader)
        result = reader.parse_data_frame(frame)

        assert result["id"] == 7
        assert result["x"] == 50.0
        assert result["y"] == 100.0
        assert result["angle"] == -120.0
        assert result["xp_pixel"] == 800
        assert result["yp_pixel"] == 600

        invalid = bytearray(frame)
        invalid[22] ^= 0x01
        assert reader.parse_data_frame(invalid) is None
    finally:
        reader.destroy_node()
        rclpy.shutdown()
