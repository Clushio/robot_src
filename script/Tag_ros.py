#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import math
import threading
from geometry_msgs.msg import PoseStamped
import socket,json
import threading

class SerialTagReader(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=38400, max_buffer_size=2048):
        super().__init__('serial_tag_reader')
        self.START_BYTE = b'\xFF'
        self.END_BYTE = b'\x03'
        self.TAG_PRESENT = b'\x55'

        self.port = port
        self.baudrate = baudrate
        self.max_buffer_size = max_buffer_size

        self.position_lock = threading.Lock()

        self.XP = -7.58
        self.YP = -15.36
        self.AP = -183.8

        self.buffer = bytearray()
        self.latest_position = None  # 最新结果

        self.publisher = self.create_publisher(PoseStamped, '/tag_position', 10)

        # UDP 配置
        self.udp_host = '192.168.3.216'  # 本机 IP
        self.udp_port = 22222            # 本机端口
        self.target_ip = '192.168.3.17' # 目标 IP
        self.target_port = 22222         # 目标端口
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建 UDP 套接字

    # 新增的 UDP 发送函数
    def send_udp_message(self, message):
        try:
            # 将消息转换为 JSON 格式字符串
            import json
            udp_message = json.dumps(message)
            # 发送消息到目标地址
            self.udp_socket.sendto(udp_message.encode('utf-8'), (self.target_ip, self.target_port))
            self.get_logger().info(f"UDP message sent: {udp_message}")
        except Exception as e:
            self.get_logger().error(f"Error sending UDP message: {e}")

    def check_receiver_status(self):
        test_message = {"status": "ping"}
        try:
            self.udp_socket.settimeout(0.5)  # 设置较长时间的超时
            self.udp_socket.sendto(json.dumps(test_message).encode('utf-8'), (self.target_ip, self.target_port))
            data, addr = self.udp_socket.recvfrom(1024)  # 尝试接收响应
            if data.decode('utf-8') == '{"status":"pong"}':
                self.get_logger().info("Receiver is online.")
                return True
        except socket.timeout:
            self.get_logger().warning("Receiver did not respond. It might be offline.")
        except Exception as e:
            self.get_logger().error(f"Error checking receiver status: {e}")
        return False

    def bytes_to_hex_string(self, data):
        return ' '.join(f'{byte:02X}' for byte in data)

    def calculate_bcc(self, data):
        bcc = 0x00
        for byte in data:
            bcc ^= byte
        return bcc

    def parse_angle(self, high_byte, low_byte):
        decimal_part = (high_byte >> 4) / 10.0
        integer_part_high = high_byte & 0x0F
        integer_part_low = low_byte
        integer_part = (integer_part_high << 8) | integer_part_low
        return integer_part + decimal_part

    def calculate_camera_position(self, X_m, Y_m, a_m):
        a_m_rad = math.radians(a_m)
        X_c_prime = -(X_m * math.cos(a_m_rad) + Y_m * math.sin(a_m_rad))
        Y_c_prime = -(Y_m * math.cos(a_m_rad) - X_m * math.sin(a_m_rad))
        a_c_prime = -a_m
        return X_c_prime, Y_c_prime, a_c_prime

    def parse_data_frame(self, data):
        if len(data) != 17 or data[0] != ord(self.START_BYTE) or data[-1] != ord(self.END_BYTE):
            return None

        if data[1] == ord(self.TAG_PRESENT):
            id_ = data[2]
            x = (data[3] << 8) + data[4]
            y = (data[5] << 8) + data[6]
            angle = self.parse_angle(data[7], data[8])
            distance = (data[9] << 8) + data[10]
            xl = (data[11] << 8) + data[12]
            yl = (data[13] << 8) + data[14]

            if xl > 0 and yl > 0:
                X_prime = 5.4 * (x - 400) / xl
                Y_prime = 5.4 * (y - 300) / yl
                xp, yp, ap = self.calculate_camera_position(X_prime, Y_prime, angle)
                bx = xp - self.XP
                by = yp - self.YP
                ba = ap - self.AP

                # 更新最新位置
                with self.position_lock:
                    self.latest_position = {
                        "id": id_,
                        "x": bx,
                        "y": by,
                        "angle": ba
                    }
            else:
                self.get_logger().warning("Error: XL or YL is zero.")
                with self.position_lock:
                    self.latest_position = None
        else:
            version = data[1]
            status = "Normal" if data[2] == 0x00 else "Abnormal"
            error_type = ["No Error", "Cable Not Connected", "Sensor Error"][data[3]]
            product_type = data[4]
            self.get_logger().info(f"No Tag - Version: {version}, Status: {status}, Error Type: {error_type}, Product Type: {product_type}")
            with self.position_lock:
                self.latest_position = None

        return True

    def find_start_byte(self, buffer):
        try:
            return buffer.index(ord(self.START_BYTE))
        except ValueError:
            return -1

    def run_serial(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.1) as ser:
                self.get_logger().info(f"Opened {self.port} at {self.baudrate} baud.")
                while not rclpy.ok():
                    if ser.in_waiting > 0:
                        if ser.in_waiting > self.max_buffer_size:
                            ser.reset_input_buffer()
                        self.buffer.extend(ser.read(ser.in_waiting))

                    start_index = self.find_start_byte(self.buffer)
                    while start_index >= 0 and len(self.buffer) >= start_index + 17 and self.buffer[start_index + 16] == ord(self.END_BYTE):
                        data = bytes(self.buffer[start_index:start_index + 17])
                        self.parse_data_frame(data)
                        self.buffer = self.buffer[start_index + 17:]
                        start_index = self.find_start_byte(self.buffer)
                    time.sleep(0.02)
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening port: {e}")

    def publish_position(self):
        rate = self.create_rate(50)  # 10 Hz
        receiver_available = True
        loop_counter = 0
        while not rclpy.ok():
            with self.position_lock:
                if self.latest_position:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "map"

                    # 设置位置和方向
                    pose_msg.pose.position.x = self.latest_position["x"]
                    pose_msg.pose.position.y = self.latest_position["y"]
                    pose_msg.pose.position.z = 0.0  # 假设 z=0

                    # 将角度转换为四元数
                    roll = 0.0
                    pitch = 0.0
                    yaw = math.radians(self.latest_position["angle"])
                    pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
                    pose_msg.pose.orientation.w = math.cos(yaw / 2.0)

                    # 发布消息
                    self.publisher.publish(pose_msg)
                    self.get_logger().info(f"Published position: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}, yaw={math.degrees(yaw):.2f}")

                    # 仅在接收端可用时发送 UDP 消息
                    # loop_counter += 1
                    # if loop_counter%10 == 0:
                    udp_message = {
                            "id": self.latest_position["id"],
                            "x": self.latest_position["x"],
                            "y": self.latest_position["y"],
                            "angle": self.latest_position["angle"]
                        }
                    self.send_udp_message(udp_message)
                else:
                    self.get_logger().info("No Tag...")
                    default_message = {
                        "id": -99,
                        "x": 0.0,
                        "y": 0.0,
                        "angle": 0.0
                    }
                    self.send_udp_message(default_message)
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "map"
                    pose_msg.pose.position.z = -99  # 假设 z=0
                     # 发布消息
                    self.publisher.publish(pose_msg)
            rate.sleep()
            

if __name__ == '__main__':
    rclpy.init()
    reader = None
    try:
        reader = SerialTagReader()

        # 启动串口读取线程
        serial_thread = threading.Thread(target=reader.run_serial)
        serial_thread.daemon = True
        serial_thread.start()

        # 启动 ROS 发布线程
        reader.publish_position()

    except KeyboardInterrupt:
        pass
    finally:
        if reader is not None:
            reader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
