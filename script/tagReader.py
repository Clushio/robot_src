
import serial
import time
import math
import threading


import rospy
import serial
import time
import math
import threading
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class SerialTagReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=38400, max_buffer_size=2048):
        self.START_BYTE = b'\xFF'
        self.END_BYTE = b'\x03'
        self.TAG_PRESENT = b'\x55'

        self.port = port
        self.baudrate = baudrate
        self.max_buffer_size = max_buffer_size

        self.XP = -4.037
        self.YP = 7.25
        self.AP = -181.2

        self.buffer = bytearray()
        self.last_valid_frame = None
        self.latest_position = None  # 新增：保存最新结果

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

        # 注释掉BCC校验（与原始代码一致）
        # calculated_bcc = self.calculate_bcc(data[2:16])
        # received_bcc = data[15]
        # if calculated_bcc != received_bcc:
        #     return None

        if data[1] == ord(self.TAG_PRESENT):
            id_ = data[2]
            x = (data[3] << 8) + data[4]
            y = (data[5] << 8) + data[6]
            angle = self.parse_angle(data[7], data[8])
            distance = (data[9] << 8) + data[10]
            xl = (data[11] << 8) + data[12]
            yl = (data[13] << 8) + data[14]

            if xl > 0 and yl > 0:
                #X_prime = 2.7 * (x - 400) / xl
                #Y_prime = 2.7 * (y - 300) / yl
                X_prime = 5.4 * (x-400) / (xl)
                Y_prime = 5.4 * (y-300) / (yl)
                Zx = 3857.143 / xl
                Zy = 3857.143 / yl
                xp, yp, ap = self.calculate_camera_position(X_prime, Y_prime, angle)
                bx = xp - self.XP
                by = yp - self.YP
                ba = ap - self.AP
                #print(f"Robot Position Bias - ID: {id_}, X: {bx:.2f}, Y: {by:.2f}, Angle: {ba:.2f}")
                self.latest_position = {
                    "id": id_,
                    "x": bx,
                    "y": by,
                    "angle": ba
                }
            else:
                print("Error: XL or YL is zero.")
                self.latest_position = None
        else:
            version = data[1]
            status = "Normal" if data[2] == 0x00 else "Abnormal"
            error_type = ["No Error", "Cable Not Connected", "Sensor Error"][data[3]]
            product_type = data[4]
            print(f"No Tag - Version: {version}, Status: {status}, Error Type: {error_type}, Product Type: {product_type}")
            self.latest_position = None

        return True

    def find_start_byte(self, buffer):
        try:
            return buffer.index(ord(self.START_BYTE))
        except ValueError:
            return -1

    def run(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.1) as ser:
                print(f"Opened {self.port} at {self.baudrate} baud.")
                while True:
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
                    time.sleep(0.01)
        except serial.SerialException as e:
            print(f"Error opening port: {e}")

if __name__ == '__main__':
    reader = SerialTagReader()
    thread = threading.Thread(target=reader.run)
    thread.daemon = True
    thread.start()

    # 主程序循环获取最新结果
    while True:
        if reader.latest_position:
            print("最新定位:", reader.latest_position)
        else:
            print("无新数据...")
        time.sleep(0.01)