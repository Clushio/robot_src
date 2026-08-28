#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import socket
import threading
import time
from collections import Counter, deque
from statistics import median

import rclpy
import serial
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MM3VSerialReader(Node):
    def __init__(self):
        super().__init__("mm3v_serial_reader")

        self.START_BYTE = 0xFF
        self.END_BYTE = 0x03
        self.TAG_PRESENT = 0x55
        self.FRAME_LEN = 24

        self.port = self._parameter("port", "/dev/ttyUSB0")
        self.baudrate = self._parameter("baudrate", 38400)
        self.frame_id = self._parameter("frame_id", "map")
        self.publish_rate = self._parameter("publish_rate", 50.0)
        self.enable_bcc_check = self._parameter("enable_bcc_check", True)

        # TagCtl keeps the ROS1 wire contract: x/y are centimetres by default.
        self.output_meter = self._parameter("output_meter", False)
        self.output_topic = self._parameter("output_topic", "/tag_position")
        self.publish_no_tag = self._parameter("publish_no_tag", True)
        self.tag_lost_timeout = max(
            0.0, float(self._parameter("tag_lost_timeout", 0.5))
        )
        self.udp_feedback_enable = self._parameter("udp_feedback_enable", True)
        self.udp_feedback_host = self._parameter(
            "udp_feedback_host", "192.168.3.17"
        )
        self.udp_feedback_port = self._parameter("udp_feedback_port", 22222)
        self.udp_feedback_rate = self._parameter("udp_feedback_rate", 2.0)
        self.udp_mode_window_size = int(
            self._parameter("udp_mode_window_size", 9)
        )
        self.udp_mode_position_bin_mm = self._parameter(
            "udp_mode_position_bin_mm", 5.0
        )
        self.udp_mode_angle_bin_deg = self._parameter(
            "udp_mode_angle_bin_deg", 0.5
        )
        self.x_sign = self._parameter("x_sign", 1.0)
        self.y_sign = self._parameter("y_sign", 1.0)
        self.yaw_sign = self._parameter("yaw_sign", -1.0)
        self.x_pixel_center = self._parameter("x_pixel_center", 400.0)
        self.y_pixel_center = self._parameter("y_pixel_center", 300.0)

        self.XP = self._parameter("xp_bias", 0.0)
        self.YP = self._parameter("yp_bias", 0.0)
        self.AP = self._parameter("angle_bias", 90.0)
        self.max_buffer_size = self._parameter("max_buffer_size", 4096)

        self.buffer = bytearray()
        self.latest_position = None
        self.last_valid_monotonic = None
        self.lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.last_log_time = {}
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_udp_feedback_monotonic = 0.0
        self.udp_mode_window = deque(maxlen=max(1, self.udp_mode_window_size))
        self.udp_mode_last_tag_id = None

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.02
        self.publish_timer = self.create_timer(period, self.publish_pose)

    def _parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def _log_throttle(self, level, key, period, message):
        now = time.monotonic()
        with self.log_lock:
            if now - self.last_log_time.get(key, -period) < period:
                return
            self.last_log_time[key] = now
        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level == "info":
            logger.info(message)
        elif level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            raise ValueError(f"Unsupported log level: {level}")

    @staticmethod
    def bytes_to_hex_string(data):
        return " ".join(f"{byte:02X}" for byte in data)

    @staticmethod
    def calculate_bcc(data):
        bcc = 0x00
        for byte in data:
            bcc ^= byte
        return bcc

    @staticmethod
    def parse_value_2int_1dec(int_high, int_low, dec_byte):
        integer = (int_high << 8) | int_low
        decimal = dec_byte / 100.0
        return integer + decimal

    @staticmethod
    def normalize_angle_deg(angle):
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def calculate_center_offset(value_cm, pixel, pixel_center):
        if pixel <= 0:
            return None
        return value_cm / float(pixel) * (pixel - pixel_center)

    def convert_camera_position(self, x_cm, y_cm, roll_deg, xp_pixel, yp_pixel):
        x_offset_cm = self.calculate_center_offset(
            x_cm, xp_pixel, self.x_pixel_center
        )
        y_offset_cm = self.calculate_center_offset(
            y_cm, yp_pixel, self.y_pixel_center
        )
        if x_offset_cm is None or y_offset_cm is None:
            return None

        if self.output_meter:
            x = x_offset_cm / 100.0
            y = y_offset_cm / 100.0
        else:
            x = x_offset_cm
            y = y_offset_cm

        yaw = self.normalize_angle_deg(self.yaw_sign * roll_deg)
        bx = self.x_sign * x - self.XP
        by = self.y_sign * y - self.YP
        ba = self.normalize_angle_deg(yaw - self.AP)
        return bx, by, ba

    def parse_data_frame(self, data):
        if len(data) != self.FRAME_LEN:
            return None
        if data[0] != self.START_BYTE or data[-1] != self.END_BYTE:
            return None

        if data[1] == self.TAG_PRESENT:
            if self.enable_bcc_check:
                calculated_bcc = self.calculate_bcc(data[2:22])
                received_bcc = data[22]
                if calculated_bcc != received_bcc:
                    self._log_throttle(
                        "warning",
                        "bcc",
                        1.0,
                        "[MM3V] BCC check failed. "
                        f"calc={calculated_bcc:02X}, recv={received_bcc:02X}, "
                        f"frame={self.bytes_to_hex_string(data)}",
                    )
                    return None

            tag_id = data[2]
            x_cm = self.parse_value_2int_1dec(data[3], data[4], data[5])
            y_cm = self.parse_value_2int_1dec(data[6], data[7], data[8])
            roll_deg = self.parse_value_2int_1dec(data[9], data[10], data[11])
            zx_cm = self.parse_value_2int_1dec(data[12], data[13], data[14])
            zy_cm = self.parse_value_2int_1dec(data[15], data[16], data[17])
            xp_pixel = (data[18] << 8) | data[19]
            yp_pixel = (data[20] << 8) | data[21]

            converted = self.convert_camera_position(
                x_cm, y_cm, roll_deg, xp_pixel, yp_pixel
            )
            if converted is None:
                self._log_throttle(
                    "warning",
                    "pixel",
                    1.0,
                    "[MM3V] Invalid pixel coordinate. "
                    f"Xp={xp_pixel}, Yp={yp_pixel}, "
                    f"frame={self.bytes_to_hex_string(data)}",
                )
                return None

            bx, by, ba = converted
            result = {
                "id": tag_id,
                "x": bx,
                "y": by,
                "angle": ba,
                "raw_x_cm": x_cm,
                "raw_y_cm": y_cm,
                "raw_roll_deg": roll_deg,
                "x_offset_cm": self.calculate_center_offset(
                    x_cm, xp_pixel, self.x_pixel_center
                ),
                "y_offset_cm": self.calculate_center_offset(
                    y_cm, yp_pixel, self.y_pixel_center
                ),
                "zx_cm": zx_cm,
                "zy_cm": zy_cm,
                "xp_pixel": xp_pixel,
                "yp_pixel": yp_pixel,
            }
            with self.lock:
                self.latest_position = result
                self.last_valid_monotonic = time.monotonic()
            return result

        frame_type = data[1]
        self._log_throttle(
            "debug",
            "heartbeat",
            1.0,
            "[MM3V] No tag / heartbeat frame. "
            f"type=0x{frame_type:02X}, frame={self.bytes_to_hex_string(data)}",
        )
        return None

    def publish_invalid_pose(self):
        self.reset_udp_mode_position()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.z = -99.0
        self.pose_pub.publish(msg)

    def reset_udp_mode_position(self):
        self.udp_mode_window.clear()
        self.udp_mode_last_tag_id = None

    def get_udp_position_bin(self):
        bin_m = self.udp_mode_position_bin_mm / 1000.0
        return bin_m if self.output_meter else bin_m * 100.0

    @staticmethod
    def quantize_udp_value(value, bin_size):
        if bin_size <= 0.0:
            return value
        return round(value / bin_size) * bin_size

    def median_angle_near_reference(self, angles, reference):
        adjusted = [
            reference + self.normalize_angle_deg(angle - reference)
            for angle in angles
        ]
        return self.normalize_angle_deg(median(adjusted))

    def update_udp_mode_position(self, pos):
        tag_id = pos["id"]
        if tag_id != self.udp_mode_last_tag_id:
            self.udp_mode_window.clear()
            self.udp_mode_last_tag_id = tag_id

        position_bin = self.get_udp_position_bin()
        angle_bin = self.udp_mode_angle_bin_deg
        qx = self.quantize_udp_value(pos["x"], position_bin)
        qy = self.quantize_udp_value(pos["y"], position_bin)
        qa = self.normalize_angle_deg(
            self.quantize_udp_value(pos["angle"], angle_bin)
        )
        key = (tag_id, qx, qy, qa)
        entry = (key, pos["x"], pos["y"], pos["angle"])
        self.udp_mode_window.append(entry)

        counts = Counter(item[0] for item in self.udp_mode_window)
        best_key = key
        best_count = counts[key]
        for item in reversed(self.udp_mode_window):
            candidate = item[0]
            count = counts[candidate]
            if count > best_count:
                best_key = candidate
                best_count = count

        cluster = [item for item in self.udp_mode_window if item[0] == best_key]
        return {
            "id": best_key[0],
            "x": median(item[1] for item in cluster),
            "y": median(item[2] for item in cluster),
            "angle": self.median_angle_near_reference(
                [item[3] for item in cluster], best_key[3]
            ),
        }

    def send_udp_feedback(self, pos, force=False):
        if not self.udp_feedback_enable:
            return

        mode_pos = self.update_udp_mode_position(pos)
        now = time.monotonic()
        min_interval = (
            1.0 / self.udp_feedback_rate if self.udp_feedback_rate > 0.0 else 0.0
        )
        if (
            not force
            and min_interval > 0.0
            and now - self.last_udp_feedback_monotonic < min_interval
        ):
            return

        message = {
            "id": mode_pos["id"],
            "x": mode_pos["x"],
            "y": mode_pos["y"],
            "angle": mode_pos["angle"],
        }
        try:
            data = json.dumps(message, ensure_ascii=False).encode("utf-8")
            self.udp_socket.sendto(
                data, (self.udp_feedback_host, int(self.udp_feedback_port))
            )
            self.last_udp_feedback_monotonic = now
        except OSError as error:
            self._log_throttle(
                "warning",
                "udp",
                2.0,
                f"[MM3V] UDP feedback send failed: {error}",
            )

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=0.1) as stream:
                    self.get_logger().info(
                        f"[MM3V] Opened serial port {self.port} "
                        f"at {self.baudrate} baud."
                    )
                    while rclpy.ok():
                        if stream.in_waiting > 0:
                            if stream.in_waiting > self.max_buffer_size:
                                self.get_logger().warning(
                                    "[MM3V] Serial input buffer too large, reset buffer."
                                )
                                stream.reset_input_buffer()
                                self.buffer.clear()
                                continue
                            self.buffer.extend(stream.read(stream.in_waiting))

                        self.extract_frames()
                        time.sleep(0.005)
            except serial.SerialException as error:
                self._log_throttle(
                    "error",
                    "serial",
                    2.0,
                    f"[MM3V] Serial error: {error}. Retry after 1s.",
                )
                time.sleep(1.0)

    def extract_frames(self):
        while len(self.buffer) >= self.FRAME_LEN:
            try:
                start_index = self.buffer.index(self.START_BYTE)
            except ValueError:
                self.buffer.clear()
                return

            if start_index > 0:
                del self.buffer[:start_index]
            if len(self.buffer) < self.FRAME_LEN:
                return

            if self.buffer[self.FRAME_LEN - 1] == self.END_BYTE:
                frame = bytes(self.buffer[: self.FRAME_LEN])
                self.parse_data_frame(frame)
                del self.buffer[: self.FRAME_LEN]
            else:
                del self.buffer[0]

    def publish_pose(self):
        with self.lock:
            now_monotonic = time.monotonic()
            tag_recent = (
                self.latest_position is not None
                and self.last_valid_monotonic is not None
                and now_monotonic - self.last_valid_monotonic
                <= self.tag_lost_timeout
            )
            pos = self.latest_position.copy() if tag_recent else None
            if not tag_recent and self.latest_position is not None:
                self.latest_position = None
                self.last_valid_monotonic = None

        if pos is not None:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = pos["x"]
            msg.pose.position.y = pos["y"]
            msg.pose.position.z = 0.0

            yaw_half = math.radians(pos["angle"]) * 0.5
            msg.pose.orientation.z = math.sin(yaw_half)
            msg.pose.orientation.w = math.cos(yaw_half)
            self.pose_pub.publish(msg)

            self._log_throttle(
                "info",
                "position",
                0.5,
                f"[MM3V] tag={pos['id']} x={pos['x']:.3f}, "
                f"y={pos['y']:.3f}, yaw={pos['angle']:.2f} deg, "
                f"Zx={pos['zx_cm']:.2f}cm, Zy={pos['zy_cm']:.2f}cm, "
                f"Xp={pos['xp_pixel']}, Yp={pos['yp_pixel']}",
            )
            self.send_udp_feedback(pos)
        elif self.publish_no_tag:
            self.publish_invalid_pose()
            self._log_throttle("info", "no_tag", 1.0, "[MM3V] no tag.")

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    reader = MM3VSerialReader()
    serial_thread = threading.Thread(target=reader.read_serial_loop, daemon=True)
    serial_thread.start()
    try:
        rclpy.spin(reader)
    except KeyboardInterrupt:
        pass
    finally:
        reader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
