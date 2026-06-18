#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class MM3VSerialReader:
    def __init__(self):
        self.START_BYTE = 0xFF
        self.END_BYTE = 0x03
        self.TAG_PRESENT = 0x55
        self.FRAME_LEN = 24

        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 38400)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = rospy.get_param("~publish_rate", 50.0)
        self.enable_bcc_check = rospy.get_param("~enable_bcc_check", True)

        # 旧 TagCtl 订阅 /tag_position，并把 pose.position.y 按 cm 再除以 100。
        # 因此这里默认保持旧接口：位置单位 cm。
        self.output_meter = rospy.get_param("~output_meter", False)
        self.output_topic = rospy.get_param("~output_topic", "/tag_position")
        self.publish_no_tag = rospy.get_param("~publish_no_tag", True)
        self.x_sign = rospy.get_param("~x_sign", 1.0)
        self.y_sign = rospy.get_param("~y_sign", 1.0)
        self.yaw_sign = rospy.get_param("~yaw_sign", -1.0)
        self.x_pixel_center = rospy.get_param("~x_pixel_center", 400.0)
        self.y_pixel_center = rospy.get_param("~y_pixel_center", 300.0)

        # 安装偏置，按你之前代码保留
        # 单位：如果 output_meter=True，这里建议也填 m
        # 如果你想直接用 cm，则 output_meter=False
        self.XP = rospy.get_param("~xp_bias", 0.0)
        self.YP = rospy.get_param("~yp_bias", 0.0)
        self.AP = rospy.get_param("~angle_bias", 90.0)

        self.max_buffer_size = rospy.get_param("~max_buffer_size", 4096)

        self.buffer = bytearray()
        self.latest_position = None
        self.lock = threading.Lock()

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)

    def bytes_to_hex_string(self, data):
        return " ".join(f"{b:02X}" for b in data)

    def calculate_bcc(self, data):
        bcc = 0x00
        for b in data:
            bcc ^= b
        return bcc

    def parse_value_2int_1dec(self, int_high, int_low, dec_byte):
        """
        MM3V 手册里的格式：
        2 字节整数位 + 1 字节小数位
        小数位十进制范围 0~99
        """
        integer = (int_high << 8) | int_low
        decimal = dec_byte / 100.0
        return integer + decimal

    def normalize_angle_deg(self, angle):
        """
        转到 [-180, 180)
        """
        angle = (angle + 180.0) % 360.0 - 180.0
        return angle

    def calculate_center_offset(self, value_cm, pixel, pixel_center):
        if pixel <= 0:
            return None
        return value_cm / float(pixel) * (pixel - pixel_center)

    def convert_camera_position(self, x_cm, y_cm, roll_deg, xp_pixel, yp_pixel):
        """
        MM3V 帧里的 X/Y 是依据像素坐标换算出来的物理量。

        旧 Tag 控制用的是相对图像中心的偏差，所以这里按手册
        使用 Xp-400、Yp-300 还原光心物理偏差，再发布给旧控制器。
        """
        x_offset_cm = self.calculate_center_offset(x_cm, xp_pixel, self.x_pixel_center)
        y_offset_cm = self.calculate_center_offset(y_cm, yp_pixel, self.y_pixel_center)

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

        # 视觉识别帧：第 2 字节是 0x55
        if data[1] == self.TAG_PRESENT:
            # BCC 校验区域：按手册说明，从第 3 字节到第 22 字节
            # Python 下标是 data[2:22]，BCC 在 data[22]
            if self.enable_bcc_check:
                calculated_bcc = self.calculate_bcc(data[2:22])
                received_bcc = data[22]
                if calculated_bcc != received_bcc:
                    rospy.logwarn_throttle(
                        1.0,
                        f"[MM3V] BCC check failed. calc={calculated_bcc:02X}, recv={received_bcc:02X}, frame={self.bytes_to_hex_string(data)}"
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

            converted = self.convert_camera_position(x_cm, y_cm, roll_deg, xp_pixel, yp_pixel)
            if converted is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"[MM3V] Invalid pixel coordinate. Xp={xp_pixel}, Yp={yp_pixel}, frame={self.bytes_to_hex_string(data)}"
                )
                with self.lock:
                    self.latest_position = None
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
                "x_offset_cm": self.calculate_center_offset(x_cm, xp_pixel, self.x_pixel_center),
                "y_offset_cm": self.calculate_center_offset(y_cm, yp_pixel, self.y_pixel_center),
                "zx_cm": zx_cm,
                "zy_cm": zy_cm,
                "xp_pixel": xp_pixel,
                "yp_pixel": yp_pixel,
            }

            with self.lock:
                self.latest_position = result

            return result

        else:
            # 状态心跳帧，未识别到标签时会发
            frame_type = data[1]
            with self.lock:
                self.latest_position = None

            rospy.logdebug_throttle(
                1.0,
                f"[MM3V] No tag / heartbeat frame. type=0x{frame_type:02X}, frame={self.bytes_to_hex_string(data)}"
            )
            return None

    def publish_invalid_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.z = -99.0
        self.pose_pub.publish(msg)

    def read_serial_loop(self):
        while not rospy.is_shutdown():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=0.1) as ser:
                    rospy.loginfo(f"[MM3V] Opened serial port {self.port} at {self.baudrate} baud.")

                    while not rospy.is_shutdown():
                        if ser.in_waiting > 0:
                            if ser.in_waiting > self.max_buffer_size:
                                rospy.logwarn("[MM3V] Serial input buffer too large, reset buffer.")
                                ser.reset_input_buffer()
                                self.buffer.clear()
                                continue

                            self.buffer.extend(ser.read(ser.in_waiting))

                        self.extract_frames()

                        time.sleep(0.005)

            except serial.SerialException as e:
                rospy.logerr_throttle(2.0, f"[MM3V] Serial error: {e}. Retry after 1s.")
                time.sleep(1.0)

    def extract_frames(self):
        while len(self.buffer) >= self.FRAME_LEN:
            # 找起始字节
            try:
                start_index = self.buffer.index(self.START_BYTE)
            except ValueError:
                self.buffer.clear()
                return

            # 丢掉起始位前面的乱码
            if start_index > 0:
                del self.buffer[:start_index]

            if len(self.buffer) < self.FRAME_LEN:
                return

            # 判断第 24 字节是不是结束符
            if self.buffer[self.FRAME_LEN - 1] == self.END_BYTE:
                frame = bytes(self.buffer[:self.FRAME_LEN])
                self.parse_data_frame(frame)
                del self.buffer[:self.FRAME_LEN]
            else:
                # 当前 FF 不是正确帧头，丢掉它，继续找下一个 FF
                del self.buffer[0]

    def publish_pose(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            with self.lock:
                pos = self.latest_position.copy() if self.latest_position else None

            if pos is not None:
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id

                msg.pose.position.x = pos["x"]
                msg.pose.position.y = pos["y"]
                msg.pose.position.z = 0.0

                yaw_rad = math.radians(pos["angle"])
                q = quaternion_from_euler(0.0, 0.0, yaw_rad)

                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]

                self.pose_pub.publish(msg)

                rospy.loginfo_throttle(
                    0.5,
                    f"[MM3V] tag={pos['id']} x={pos['x']:.3f}, y={pos['y']:.3f}, yaw={pos['angle']:.2f} deg, "
                    f"Zx={pos['zx_cm']:.2f}cm, Zy={pos['zy_cm']:.2f}cm, "
                    f"Xp={pos['xp_pixel']}, Yp={pos['yp_pixel']}"
                )
            elif self.publish_no_tag:
                self.publish_invalid_pose()
                rospy.loginfo_throttle(1.0, "[MM3V] no tag.")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mm3v_serial_reader")

    reader = MM3VSerialReader()

    serial_thread = threading.Thread(target=reader.read_serial_loop)
    serial_thread.daemon = True
    serial_thread.start()

    reader.publish_pose()
