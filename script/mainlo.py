import serial
import time

START_BYTE = b'\xFF'
END_BYTE = b'\x03'
TAG_PRESENT = b'\x55'

XP = 0
YP = 0
AP = 0

def bytes_to_hex_string(data):
    """将字节序列转换为十六进制字符串表示"""
    return ' '.join(f'{byte:02X}' for byte in data)

def calculate_bcc(data):
    """计算BCC校验值"""
    bcc = 0x00
    for byte in data:
        bcc ^= byte
    return bcc

def parse_angle(high_byte, low_byte):
    """解析角度值，返回浮点数形式的角度"""
    # 提取高字节的第一个字符作为小数部分
    decimal_part = (high_byte >> 4) / 10.0  # 高字节的前4位表示小数部分
    
    # 提取整数部分
    integer_part_high = high_byte & 0x0F  # 高字节的后4位
    integer_part_low = low_byte  # 低字节全部

    # 合并整数部分
    integer_part = (integer_part_high << 8) | integer_part_low

    # 组合整数部分和小数部分
    angle = integer_part + decimal_part
    return angle

import math

def calculate_camera_position_in_marker_frame(X_m, Y_m, a_m):
    """
    计算在 marker 坐标系下摄像头的位置和角度。
    
    参数:
        X_m (float): 在摄像头坐标系中 marker 的 X 坐标。
        Y_m (float): 在摄像头坐标系中 marker 的 Y 坐标。
        a_m (float): 在摄像头坐标系中 marker 的角度（以度数为单位）。
    
    返回:
        tuple: (X_c', Y_c', a_c')，即在 marker 坐标系中摄像头的位置和角度。
    """
    # 将角度从度数转换为弧度
    a_m_rad = math.radians(a_m)
    
    # 计算摄像头在 marker 坐标系中的位置
    X_c_prime = -(X_m * math.cos(a_m_rad) + Y_m * math.sin(a_m_rad))
    Y_c_prime = -(Y_m * math.cos(a_m_rad) - X_m * math.sin(a_m_rad))
    
    # 计算摄像头在 marker 坐标系中的角度
    a_c_prime = -a_m
    
    return X_c_prime, Y_c_prime, a_c_prime


def parse_data_frame(data):
    """解析数据帧并输出中间结果或十六进制格式"""
    if len(data) != 17 or data[0] != ord(START_BYTE) or data[-1] != ord(END_BYTE):
        print(f"Invalid frame length or start/end bytes. Data: {bytes_to_hex_string(data)}")
        return None
    
    # 校验BCC
    calculated_bcc = calculate_bcc(data[2:16])
    received_bcc = data[15]
  #  if calculated_bcc != received_bcc:
  #      print(f"BCC check failed. Calculated: {calculated_bcc:02X}, Received: {received_bcc:02X}. Data: {bytes_to_hex_string(data)}")
  #      return None

    # 输出十六进制格式的数据
    #print(f"Data Frame (Hex): {bytes_to_hex_string(data)}")

    if data[1] == ord(TAG_PRESENT):  # 有标签的数据
        id_ = data[2]
        x = (data[3] << 8) + data[4]
        y = (data[5] << 8) + data[6]
        angle = parse_angle(data[7],data[8])
        #(data[7] << 8) + data[8]
        distance = (data[9] << 8) + data[10]
        xl = (data[11] << 8) + data[12]
        yl = (data[13] << 8) + data[14]

        # 计算物理坐标和距离
        if xl > 0 and yl > 0:
            #X_prime = 2.7 * (x-400) / (xl)
            #Y_prime = 2.7 * (y-300) / (yl)
            #Zx = 1976.79 / xl
            #Zy = 1976.79 / yl
            X_prime = 5.4 * (x-400) / (xl)
            Y_prime = 5.4 * (y-300) / (yl)
            Zx = 3857.143 / xl
            Zy = 3857.143 / yl
            print(f"Physical Coordinates - X': {X_prime:.2f} cm, Y': {Y_prime:.2f} cm, angle={angle},Distance Zx: {Zx:.2f} cm, Distance Zy: {Zy:.2f} cm")
            xp,yp,ap = calculate_camera_position_in_marker_frame(X_prime,Y_prime,angle)
         # 解析并输出中间结果
            print(f"camera position in tag - ID: {id_}, X: {xp}, Y: {yp}, Angle: {ap}, Distance: {distance}, XL: {xl}, YL: {yl}")
            bx = xp-XP
            by = yp-YP
            ba = ap-AP
            print(f"robot position bias- ID: {id_}, X: {bx}, Y: {by}, Angle: {ba}")

        else:
            print("Error: XL or YL is zero, cannot compute physical coordinates.")

        # 解析并输出中间结果
        #print(f"Tag Detected - ID: {id_}, X: {x}, Y: {y}, Angle: {angle}, Distance: {distance}, XL: {xl}, YL: {yl}")

    else:  # 没有标签的数据
        version = data[1]
        status = "Normal" if data[2] == 0x00 else "Abnormal"
        error_type = ["No Error", "Cable Not Connected", "Sensor Error"][data[3]]
        product_type = data[4]
        print(f"No Tag - Version: {version}, Status: {status}, Error Type: {error_type}, Product Type: {product_type}")

    return True

def find_start_byte(buffer):
    """查找起始字节的位置"""
    try:
        return buffer.index(ord(START_BYTE))
    except ValueError:
        return -1

def main():
    port = '/dev/ttyUSB0'  # Windows系统下的串口名，Linux或Mac下可能是'/dev/ttyUSB0'或类似
    baudrate = 38400  # 根据实际情况调整波特率
    max_buffer_size = 2048  # 设置最大缓冲区大小以防止溢出

    try:
        with serial.Serial(port, baudrate, timeout=0.1) as ser:
            buffer = bytearray()
            last_valid_frame = None
            print(f"Opened serial port {port} at {baudrate} baud.")
            while True:
                if ser.in_waiting > 0:
                    # 清空超过最大缓冲区大小的部分
                    if ser.in_waiting > max_buffer_size:
                        ser.reset_input_buffer()

                    buffer.extend(ser.read(ser.in_waiting))

                # 查找最新的有效帧
                start_index = find_start_byte(buffer)
                while start_index >= 0 and len(buffer) >= start_index + 17 and buffer[start_index + 16] == ord(END_BYTE):
                    data = bytes(buffer[start_index:start_index + 17])
                    if parse_data_frame(data):
                        last_valid_frame = data
                    buffer = buffer[start_index + 17:]  # 移除已处理的数据
                    start_index = find_start_byte(buffer)

                # 如果有新的有效帧，则打印并重置
                if last_valid_frame:
                    parse_data_frame(last_valid_frame)
                    last_valid_frame = None

                # 睡眠一段时间以降低CPU占用率
                time.sleep(0.5)  # 调整此值以控制处理频率

    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")

if __name__ == '__main__':
    main()