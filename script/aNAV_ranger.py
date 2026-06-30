import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton ,QLabel, QVBoxLayout, QHBoxLayout, QLineEdit
import subprocess
import time

from PyQt5.QtCore import Qt,QTimer
from PyQt5.QtGui import QDoubleValidator

import os
import signal

import rospy
from sensor_msgs.msg import Joy

from tagReader import SerialTagReader
import threading



def robot_r_path():
    try:
        return subprocess.check_output(['rospack', 'find', 'robot_r'], text=True).strip()
    except Exception:
        return '/home/nav/suvrobot/src/robot_r'


ROBOT_R_PATH = robot_r_path()
PANEL_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz.rviz')
PLAIN_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz_no_panel.rviz')

def check_and_start_roscore():
    try:
        # 检查roscore是否已经在运行
        result = subprocess.run(['pgrep', '-f', 'roscore'], capture_output=True, text=True)
        if result.stdout.strip():
            print("roscore is already running.")
        else:
            print("Starting roscore...")
            # 启动roscore
            roscore_process = subprocess.Popen(['gnome-terminal', '--hide-menubar', '--', 'roscore'])
            
            # 等待一段时间，确保roscore已经启动
            time.sleep(3)
            
            # 获取 roscore 的窗口 ID
            window_id = None
            while not window_id:
                try:
                    window_id = subprocess.check_output(['wmctrl', '-l']).strip().decode('utf-8')
                    for line in window_id.split('\n'):
                        if 'roscore' in line:
                            window_id = line.split()[0]
                            break
                except subprocess.CalledProcessError:
                    pass
                time.sleep(1)
            
            # 最小化窗口
            if window_id:
                subprocess.run(['wmctrl', '-i', '-r', window_id, '-b', 'add,hidden'])
    except Exception as e:
        print(f"Error checking or starting roscore: {e}")
        sys.exit(1)

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        # 设置窗口始终在最上方
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setFixedSize(480, 850)

        check_and_start_roscore()

        # 初始化ROS节点
        rospy.init_node('upmachine_publisher', anonymous=True)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)

        self.initUI()
        self.setpointProcess = None
        self.localizationProcess = None
        self.rvizProcess = None
        self.rviz_panel_mode = False
        self.runPntsNavProcess = None
        self.add_pnt.hide()
        self.add_workstation.hide()
        self.location_pnt.hide()
        self.nav_runstart.hide()
        self.nav_Pause.hide()
        self.nav_resume.hide()
        self.set_nav_shortcuts_visible(False)

        self.currentID = 0
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_angle = 0.0

        #self.init_reader()

        #self.initTimer()

    def init_reader(self):
        self.reader = SerialTagReader()
        thread = threading.Thread(target=self.reader.run)
        thread.daemon = True
        thread.start()

    def initTimer(self):
        # 定时器触发数据更新（这里可以设置更新频率）
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_position)
        self.timer.start(200)  # 每秒更新一次

    def update_position(self):
        # 定时触发时启动线程获取数据（这里可以省略，因为线程在后台持续运行）
        self.position_label.setText(
            f"X={self.reader.latest_position['x']:.1f}, Y={self.reader.latest_position['y']:.1f}, 角度={self.reader.latest_position['angle']:.1f}"
        )
        pass  # 如果需要，可以在这里添加其他逻辑

    def initUI(self):
        # 创建启动按钮
        self.start_button = QPushButton('设置导航点', self)
        self.start_button.move(18, 10)
        self.start_button.clicked.connect(self.start_setlocation)
        self.start_button.resize(150,70)

        # 创建停止按钮
        self.stop_button = QPushButton('结束设置', self)
        self.stop_button.move(168, 10)
        self.stop_button.clicked.connect(self.stop_setlocation)
        self.stop_button.resize(150,70)

        self.rviz_button = QPushButton('Rviz', self)
        self.rviz_button.move(338, 10)
        self.rviz_button.clicked.connect(self.start_rviz)
        self.rviz_button.resize(118,70)

        # 创建tianjia按钮
        self.add_pnt = QPushButton('添加点', self)
        self.add_pnt.move(18, 100)
        self.add_pnt.clicked.connect(self.add_pnt_fun)
        self.add_pnt.resize(138,70)

        self.add_workstation = QPushButton('添加工位', self)
        self.add_workstation.move(168, 100)
        self.add_workstation.clicked.connect(self.add_workstation_fun)
        self.add_workstation.resize(138,70)

        self.location_pnt = QPushButton('显示点位', self)
        self.location_pnt.move(318, 100)
        self.location_pnt.clicked.connect(self.show_pnt_fun)
        self.location_pnt.resize(138,70)

        # 创建2d按钮
        self.g2d_button = QPushButton('开始定位', self)
        self.g2d_button.move(18, 200)
        self.g2d_button.clicked.connect(self.start_navgation)
        self.g2d_button.resize(150,70)

        # save2d按钮
        self.g2d_exit_button = QPushButton('退出定位', self)
        self.g2d_exit_button.move(168, 200)
        self.g2d_exit_button.clicked.connect(self.quit_navigation)
        self.g2d_exit_button.resize(150,70)

         # nav按钮
        self.nav_button = QPushButton('AutoNAV', self)
        self.nav_button.move(18, 300)
        self.nav_button.clicked.connect(self.startNAV)
        self.nav_button.resize(150,70)

                       # nav按钮
        self.nav5_button = QPushButton('MoveBase', self)
        self.nav5_button.move(178, 300)
        self.nav5_button.clicked.connect(self.nav5)
        self.nav5_button.resize(95,70)

        # nav按钮
        self.quitnav_button = QPushButton('退出Nav', self)
        self.quitnav_button.move(283, 300)
        self.quitnav_button.clicked.connect(self.exitNAV)
        self.quitnav_button.resize(95,70)

        self.nav_runstart = QPushButton('开始', self)
        self.nav_runstart.move(74, 390)
        self.nav_runstart.clicked.connect(self.Nav_start)
        self.nav_runstart.resize(95,40)

        self.nav_Pause = QPushButton('暂停', self)
        self.nav_Pause.move(190, 390)
        self.nav_Pause.clicked.connect(self.Nav_Zanting)
        self.nav_Pause.resize(95,40)

        self.nav_resume = QPushButton('继续', self)
        self.nav_resume.move(306, 390)
        self.nav_resume.clicked.connect(self.Nav_jixu)
        self.nav_resume.resize(95,40)
        
        gapx = 90
        self.p1_button = QPushButton('导航\n点1', self)
        self.p1_button.move(18, 455)
        self.p1_button.clicked.connect(lambda: self.gotop(0)) # 直接连接到QWidget的close()方法
        self.p1_button.setToolTip('调用 /plan_path_and_go，目标点索引 0')
        self.p1_button.resize(75,70)

        self.p2_button = QPushButton('导航\n点2', self)
        self.p2_button.move(18+gapx, 455)
        self.p2_button.clicked.connect(lambda: self.gotop(1))  # 直接连接到QWidget的close()方法
        self.p2_button.setToolTip('调用 /plan_path_and_go，目标点索引 1，W1 工位对应导航点')
        self.p2_button.resize(75,70)

        self.p3_button = QPushButton('导航\n点3', self)
        self.p3_button.move(18+2*gapx, 455)
        self.p3_button.clicked.connect(lambda: self.gotop(2))  # 直接连接到QWidget的close()方法
        self.p3_button.setToolTip('调用 /plan_path_and_go，目标点索引 2，W2 工位对应导航点')
        self.p3_button.resize(75,70)

        self.p4_button = QPushButton('导航\n点4', self)
        self.p4_button.move(18+3*gapx, 455)
        self.p4_button.clicked.connect(lambda: self.gotop(3)) # 直接连接到QWidget的close()方法
        self.p4_button.setToolTip('调用 /plan_path_and_go，目标点索引 3')
        self.p4_button.resize(75,70)

        self.p5_button = QPushButton('导航\n点5', self)
        self.p5_button.move(18+4*gapx, 455)
        self.p5_button.clicked.connect(lambda: self.gotop(4))  # 直接连接到QWidget的close()方法
        self.p5_button.setToolTip('调用 /plan_path_and_go，目标点索引 4')
        self.p5_button.resize(75,70)

        self.w1_button = QPushButton('W1', self)
        self.w1_button.move(18, 535)
        self.w1_button.clicked.connect(lambda: self.gotoworkstation(1))
        self.w1_button.setToolTip('调用 /plan_path_and_go，目标工位 W1')
        self.w1_button.resize(75,25)

        self.w2_button = QPushButton('W2', self)
        self.w2_button.move(18+gapx, 535)
        self.w2_button.clicked.connect(lambda: self.gotoworkstation(2))
        self.w2_button.setToolTip('调用 /plan_path_and_go，目标工位 W2')
        self.w2_button.resize(75,25)

        self.w3_button = QPushButton('W3', self)
        self.w3_button.move(18+2*gapx, 535)
        self.w3_button.clicked.connect(lambda: self.gotoworkstation(3))
        self.w3_button.setToolTip('调用 /plan_path_and_go，目标工位 W3')
        self.w3_button.resize(75,25)

        #lambda: self.gotop5("Hello", "World"

        self.x_label = QLabel('X=', self)
        self.x_label.move(18, 590)
        self.x_label.resize(25,30)

        self.x_input = QLineEdit('0.00', self)
        self.x_input.move(43, 585)
        self.x_input.resize(75,40)
        self.x_input.setValidator(QDoubleValidator(-10.0, 10.0, 3, self))
        self.x_input.returnPressed.connect(self.send_target_input)

        self.y_label = QLabel('Y=', self)
        self.y_label.move(128, 590)
        self.y_label.resize(25,30)

        self.y_input = QLineEdit('0.00', self)
        self.y_input.move(153, 585)
        self.y_input.resize(75,40)
        self.y_input.setValidator(QDoubleValidator(-10.0, 10.0, 3, self))
        self.y_input.returnPressed.connect(self.send_target_input)

        self.angle_label = QLabel('Angle=', self)
        self.angle_label.move(238, 590)
        self.angle_label.resize(50,30)

        self.angle_input = QLineEdit('0.00', self)
        self.angle_input.move(288, 585)
        self.angle_input.resize(75,40)
        self.angle_input.setValidator(QDoubleValidator(-360.0, 360.0, 3, self))
        self.angle_input.returnPressed.connect(self.send_target_input)

        self.target_send_button = QPushButton('发送', self)
        self.target_send_button.move(373, 585)
        self.target_send_button.clicked.connect(self.send_target_input)
        self.target_send_button.resize(75,40)

        self.target_value_label = QLabel('目标 X=0.00 Y=0.00 Angle=0.00', self)
        self.target_value_label.move(18, 630)
        self.target_value_label.resize(430,25)

        quick_y = [(-0.1, 'Y=-0.10'), (0.0, 'Y=0.00'), (0.07, 'Y=0.07'), (0.1, 'Y=0.10'), (0.2, 'Y=0.20')]
        self.y_quick_buttons = []
        for index, (value, text) in enumerate(quick_y):
            button = QPushButton(text, self)
            button.move(18+index*gapx, 660)
            button.clicked.connect(lambda checked=False, y=value: self.set_y_value(y, True))
            button.setToolTip(f'设置 Y 输入框并调用 /set_target_y，目标 Y={value:.2f} m')
            button.resize(75,70)
            self.y_quick_buttons.append(button)

        # exit nav按钮
        #self.nav_exit_button = QPushButton('exit导航模式', self)
        #self.nav_exit_button.move(200, 50)
        #self.nav_exit_button.clicked.connect(self.f_ext_nav_button)
        self.can_button = QPushButton('CANStart', self)
        self.can_button.move(18, 740)
        self.can_button.clicked.connect(self.start_can)  # 直接连接到QWidget的close()方法
        self.can_button.resize(75,70)

        self.base_button = QPushButton('BaseStart', self)
        self.base_button.move(108, 740)
        self.base_button.clicked.connect(self.start_base)  # 直接连接到QWidget的close()方法
        self.base_button.resize(75,70)

        self.joy_button = QPushButton('JOY', self)
        self.joy_button.move(198, 740)
        self.joy_button.clicked.connect(self.start_joy)  # 直接连接到QWidget的close()方法
        self.joy_button.resize(75,70)

        self.joy_button = QPushButton('TAG', self)
        self.joy_button.move(288, 740)
        self.joy_button.clicked.connect(self.start_tag)  # 直接连接到QWidget的close()方法
        self.joy_button.resize(75,70)


         # 创建退出按钮
        self.quit_button = QPushButton('退出', self)
        self.quit_button.move(378, 740)
        self.quit_button.clicked.connect(self.close_app)  # 直接连接到QWidget的close()方法
        self.quit_button.resize(75,70)
        #self.initbash()

        # 显示定start_tag)  # 直接连接到QWidget的close()方法
        self.joy_button.resize(75,70)


         # 创建退出按钮
        self.quit_button = QPushButton('退出', self)
        self.quit_button.move(378, 740)
        self.quit_button.clicked.connect(self.close_app)  # 直接连接到QWidget的close()方法
        self.quit_button.resize(75,70)
        #self.initbash()

        # 显示定位数据的标签
        self.position_label = QLabel("最新定位数据：无", self)
        self.position_label.setStyleSheet("font-size: 14px; color: blue;")
        self.position_label.move(18, 825)
        self.position_label.resize(300,20)
        #layout.addWidget(self.position_label)

        #self.setLayout(layout)

    def set_nav_shortcuts_visible(self, visible):
        for button in (
            self.p1_button, self.p2_button, self.p3_button, self.p4_button, self.p5_button,
            self.w1_button, self.w2_button, self.w3_button
        ):
            button.setVisible(visible)

    def read_target_input(self, line_edit):
        text = line_edit.text().strip()
        if not text:
            return 0.0
        return float(text)

    def set_target_values(self, target_x, target_y, target_angle, send=False):
        self.current_target_x = float(target_x)
        self.current_target_y = float(target_y)
        self.current_target_angle = float(target_angle)
        self.x_input.setText(f"{self.current_target_x:.2f}")
        self.y_input.setText(f"{self.current_target_y:.2f}")
        self.angle_input.setText(f"{self.current_target_angle:.2f}")
        self.target_value_label.setText(
            f"目标 X={self.current_target_x:.2f} Y={self.current_target_y:.2f} Angle={self.current_target_angle:.2f}"
        )
        if send:
            self.slowMove(self.current_target_x, self.current_target_y, self.current_target_angle)

    def set_y_value(self, posY, send=False):
        try:
            target_x = self.read_target_input(self.x_input)
            target_angle = self.read_target_input(self.angle_input)
        except ValueError:
            self.target_value_label.setText("目标值输入错误")
            return
        self.set_target_values(target_x, posY, target_angle, send)

    def send_target_input(self):
        try:
            target_x = self.read_target_input(self.x_input)
            target_y = self.read_target_input(self.y_input)
            target_angle = self.read_target_input(self.angle_input)
        except ValueError:
            self.target_value_label.setText("目标值输入错误")
            return
        self.set_target_values(target_x, target_y, target_angle, True)

    def send_y_input(self):
        self.send_target_input()
    
    def slowMove(self, posX, posY, posAngle): #go to work  pose
        service_name = '/set_target_y'
        xx = f"{float(posX):.3f}"
        yy = f"{float(posY):.3f}"
        aa = f"{float(posAngle):.3f}"
         # 构建完整的 ROS 命令
        ros_command = f'rosservice call {service_name} "{{target_x: {xx}, target_y: {yy}, target_angle: {aa}}}"'
       # 构建终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command]
            # 使用 subprocess.Popen 启动新的终端
        self.runGoProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程    

    #import subprocess

    def gotop1(self, id):
        # 定义服务名称
        service_name = '/plan_path_and_go'
        
        # 构建请求数据
        data = str(id)  # 第一个参数
        curid = str(self.currentID)
        run = '1'       # 第二个参数

        # 构建完整的 ROS 命令
        ros_command = f'rosservice call {service_name} {data} {curid} {run}'
        
        try:
            # 使用 subprocess 调用 ROS 服务并捕获输出
            result = subprocess.run(
                ros_command,
                shell=True,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # 打印或处理返回值
            print("Service call successful!")
            print("Output:", result.stdout)
            
            # 更新 currentID
            self.currentID = id
            
            # 返回服务的输出（可以根据需要进一步解析）
            return result.stdout
        
        except subprocess.CalledProcessError as e:
            # 捕获错误并打印错误信息
            print("Service call failed!")
            print("Error:", e.stderr)
            return None

    
    def gotop(self,id):  #go to position id
        # 定义服务名称
        service_name = '/plan_path_and_go'
        # 构建请求数据，确保 data 和 run 是整数
        # 构建请求数据（位置参数）
        data = str(id)  # 第一个参数
        curid = str(self.currentID)
        run = '1'       # 第二个参数

        # 构建完整的 ROS 命令
        request = f"{{data: {data}, currentID: {curid}, run: {run}}}"
        command = ['rosservice', 'call', service_name, request]

        def call_service():
            try:
                result = subprocess.run(
                    command,
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                print("plan_path_and_go successful:", result.stdout)
                self.currentID = id
            except subprocess.CalledProcessError as e:
                print("plan_path_and_go failed:", e.stderr)

        thread = threading.Thread(target=call_service)
        thread.daemon = True
        thread.start()

    def gotoworkstation(self, station_id):  # go to workstation W1/W2/W3
        self.gotop(-station_id)

    def start_joy(self):
        ros_command = 'roslaunch x2bot_teleop x2bot_joy_PXN.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.runJoyProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程    

    def start_base(self):
                     # 定义要执行的 ROS 命令
        ros_command = 'roslaunch ranger_bringup ranger_mini_v2.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.run5NavProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程    

    def start_tag(self):
    # 创建完整的 gnome-terminal 命令字符串
    # 定义每个标签页的命令
        commands = [
            ['gnome-terminal', '--tab', '--title=MM3V Tag', '--', 'bash', '-c', 'roslaunch robot_r 6tagReadAndCtl_mm3v.launch; exec bash'],
            ['gnome-terminal', '--tab', '--title=TCP Server', '--', 'bash', '-c', 'python3 tcpserver.py; exec bash']
        ]
        try:
            # 分别启动每个标签页
            for cmd in commands:
                subprocess.Popen(cmd)
        except Exception as e:
            print(f"Failed to run command: {e}")
                            
    def start_can(self):
        try:
            # 构建设置 CAN 接口的命令（自动提供密码）
            can_command = "echo '1' | sudo -S ip link set can0 up type can bitrate 500000"

            # 构建完整的终端命令
            terminal_command = ['gnome-terminal', '--', 'bash', '-c', can_command + '; exec bash']

            # 使用 subprocess.Popen 启动新的终端窗口
            subprocess.Popen(terminal_command)

            # 显示成功消息
            #QMessageBox.information(self, '成功', '已启动新的终端并设置 CAN 接口！')
        except Exception as e:
            # 如果失败，显示错误消息
            error_message = f"错误: {str(e)}"
            #QMessageBox.critical(self, '错误', error_message)

    def send_joy_message(self,cmd):
        #"""发送 /joy 消息"""
        if(cmd=="add"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 1, 0,0,0]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy add message sent")
        if(cmd=="show"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 0, 0,0,1]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy show message sent")

        if(cmd=="add_workstation"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 0, 0,1,0]       # 工位点
            self.joy_pub.publish(joy_msg)
            print("Joy add workstation message sent")

        if(cmd=="NavStart"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 0, 1,0,0]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy NavStart message sent")

        if(cmd=="NavPause"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [1, 0, 0, 0,0,0]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy NavPause message sent")

        if(cmd=="Navresume"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 1, 0,0,0]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy Navresume message sent")

    def add_pnt_fun(self):
        self.send_joy_message("add")

    def add_workstation_fun(self):
        self.send_joy_message("add_workstation")

    def show_pnt_fun(self):
        self.send_joy_message("show")

    def Nav_start(self):
        self.send_joy_message("NavStart")
    
    def Nav_Zanting(self):
        self.send_joy_message("NavPause")
    
    def Nav_jixu(self):
        self.send_joy_message("Navresume")

    def start_setlocation(self):
            # 定义要执行的 ROS 命令
        ros_command = 'roslaunch robot_r 3settinglocation.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.setpointProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程          
            # 禁用启动按钮
        self.start_button.setEnabled(False)
                  # 禁用终止按钮
        self.stop_button.setEnabled(True)
        self.location_pnt.show()
        self.add_pnt.show()
        self.add_workstation.show()

    def stop_setlocation(self):
        if self.setpointProcess:
                    # 获取进程组 ID
            pgid = os.getpgid(self.setpointProcess.pid)              
                # 使用 subprocess.call 发送 kill 命令
            # 使用 pkill 命令终止进程组
            subprocess.call(['pkill', '-9', '-g', str(pgid)])
                # 重置进程变量
            self.setpointProcess = None          
                # 重新启用启动按钮
            self.start_button.setEnabled(True)     
                # 禁用终止按钮
            self.stop_button.setEnabled(False)
            self.add_pnt.hide()
            self.add_workstation.hide()
            self.location_pnt.hide()

    def is_rviz_running(self):
        if self.rvizProcess and self.rvizProcess.poll() is None:
            return True
        result = subprocess.run(['pgrep', '-x', 'rviz'], capture_output=True, text=True)
        return bool(result.stdout.strip())

    def start_plain_rviz(self):
        if self.is_rviz_running():
            return
        config = PLAIN_RVIZ_CONFIG if os.path.exists(PLAIN_RVIZ_CONFIG) else PANEL_RVIZ_CONFIG
        command = ['rviz', '-d', config]
        self.rvizProcess = subprocess.Popen(command, preexec_fn=os.setpgrp)
        self.rviz_panel_mode = False

    def start_navgation(self):
        if self.localizationProcess and self.localizationProcess.poll() is None:
            print('3startlocation.launch is already running.')
        else:
            command = ['roslaunch', 'robot_r', '3startlocation.launch', 'rviz_enable:=false']
            self.localizationProcess = subprocess.Popen(command, preexec_fn=os.setpgrp)
            self.g2d_button.setEnabled(False)
            self.g2d_exit_button.setEnabled(True)
        if not self.is_rviz_running():
            self.start_plain_rviz()

    def start_rviz(self):
        if self.is_rviz_running():
            print('RViz is already running.')
            return
        command = ['rviz', '-d', PANEL_RVIZ_CONFIG]
        self.rvizProcess = subprocess.Popen(command, preexec_fn=os.setpgrp)
        self.rviz_panel_mode = True
        self.rviz_button.setEnabled(False)

    def nav5(self):
                  # 定义要执行的 ROS 命令
        ros_command = 'roslaunch robot_r 5nav.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.run5NavProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程     

    # load points and auto run
    def startNAV(self):
          # 定义要执行的 ROS 命令
        ros_command = 'roslaunch robot_r 3navlocations.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.runPntsNavProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程          
        self.nav_button.setEnabled(False)
        #self.nav_button.setText("启动OK")
        self.quitnav_button.setEnabled(True)
        self.nav_runstart.show()
        self.nav_Pause.show()
        self.nav_resume.show()
        self.set_nav_shortcuts_visible(True)

    def exitNAV(self):
        if self.runPntsNavProcess:
                    # 获取进程组 ID
            pgid = os.getpgid(self.runPntsNavProcess.pid)              
                # 使用 subprocess.call 发送 kill 命令
            # 使用 pkill 命令终止进程组
            subprocess.call(['pkill', '-9', '-g', str(pgid)])
                # 重置进程变量
            self.runPntsNavProcess = None          
                # 重新启用启动按钮
            self.nav_button.setEnabled(True)     
                # 禁用终止按钮
            self.quitnav_button.setEnabled(False)
            self.nav_runstart.hide()
            self.nav_Pause.hide()
            self.nav_resume.hide()
            self.set_nav_shortcuts_visible(False)


    def quit_navigation(self):
        if self.localizationProcess and self.localizationProcess.poll() is None:
            pgid = os.getpgid(self.localizationProcess.pid)
            subprocess.call(['pkill', '-15', '-g', str(pgid)])
            self.localizationProcess = None
        else:
            process_name = '3startlocation'
            try:
                process_id = subprocess.check_output(['pgrep', '-f', process_name]).decode().strip()
            except subprocess.CalledProcessError:
                process_id = ''
            for pid in process_id.split():
                subprocess.call(['kill', '-15', pid])
        self.g2d_button.setEnabled(True)

    def stop_rviz(self):
        if self.rvizProcess and self.rvizProcess.poll() is None:
            pgid = os.getpgid(self.rvizProcess.pid)
            subprocess.call(['pkill', '-15', '-g', str(pgid)])
            self.rvizProcess = None
        self.rviz_panel_mode = False
        self.rviz_button.setEnabled(True)

 
    def close_app(self):
        self.exitNAV()
        self.quit_navigation()
        self.stop_rviz()
        self.stop_setlocation()
        self.close()  # 关闭窗口，这将触发closeEvent事件，从而结束整个应用程序

    def closeEvent(self, event):
        # 当窗口关闭时，确保清理资源等
        event.accept()  # 接受关闭事件，默认行为是关闭窗口并退出程序
        


if __name__ == '__main__':
    app = QApplication(sys.argv)



    mainWin = MyWindow()
    mainWin.show()

    sys.exit(app.exec_())
