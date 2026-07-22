import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLineEdit, QGroupBox, QTabWidget, QFrame, QSizePolicy
)
import subprocess
import time
import shlex

from PyQt5.QtCore import Qt,QTimer
from PyQt5.QtGui import QDoubleValidator

import os
import signal

import rospy
from sensor_msgs.msg import Joy

from tagReader import SerialTagReader
import threading


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(SCRIPT_DIR)


def load_workspace_environment():
    """自动加载脚本所在 catkin 工作空间，避免要求用户先 source setup.bash。"""
    env_script = os.path.join(WORKSPACE_ROOT, 'devel', 'env.sh')
    if not os.path.isfile(env_script):
        return False
    try:
        output = subprocess.check_output([env_script, 'env', '-0'])
        loaded_environment = {}
        for entry in os.fsdecode(output).split('\0'):
            if '=' in entry:
                key, value = entry.split('=', 1)
                loaded_environment[key] = value
        os.environ.update(loaded_environment)
        return True
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"Unable to load workspace environment: {error}")
        return False


WORKSPACE_ENV_LOADED = load_workspace_environment()



def robot_r_path():
    try:
        return subprocess.check_output(['rospack', 'find', 'robot_r'], text=True).strip()
    except Exception:
        return os.path.join(WORKSPACE_ROOT, 'robot_r')


ROBOT_R_PATH = robot_r_path()
PANEL_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz.rviz')
PLAIN_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz_no_panel.rviz')

def check_and_start_roscore():
    """确保 ROS master 可用，且不让 GUI 在等待窗口标题时无限卡住。"""
    try:
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            return True

        print("Starting roscore...")
        subprocess.Popen(
            ['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp
        )
        for _ in range(16):
            time.sleep(0.5)
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                return True
    except Exception as e:
        print(f"Error checking or starting roscore: {e}")
    return False

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('移动机器人作业控制台')
        self.setMinimumSize(820, 640)
        self.resize(960, 700)

        self.preview_mode = os.environ.get('ANAV_GUI_PREVIEW') == '1'
        self.ros_available = False if self.preview_mode else check_and_start_roscore()
        self.joy_pub = None
        if self.ros_available:
            try:
                rospy.init_node('upmachine_publisher', anonymous=True, disable_signals=True)
                self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
            except Exception as error:
                print(f"ROS initialization failed: {error}")
                self.ros_available = False

        self.initUI()
        self.set_chip(
            self.ros_chip,
            'ROS 已连接' if self.ros_available else 'ROS 未连接',
            self.ros_available,
        )
        if not self.ros_available and not self.preview_mode:
            self.set_status('ROS 连接失败，请检查 ROS_MASTER_URI 或启动日志。', 'error')
        self.setpointProcess = None
        self.localizationProcess = None
        self.rvizProcess = None
        self.rviz_panel_mode = False
        self.runPntsNavProcess = None
        self.moveBaseProcess = None
        self.baseProcess = None
        self.auto_nav_start_pending = False
        self.move_base_wait_attempts = 0
        self.add_pnt.hide()
        self.add_workstation.hide()
        self.location_pnt.hide()
        self.nav_runstart.setEnabled(False)
        self.nav_Pause.setEnabled(False)
        self.nav_resume.setEnabled(False)
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
            f"最新标签定位：X {self.reader.latest_position['x']:.1f}  ·  "
            f"Y {self.reader.latest_position['y']:.1f}  ·  角度 {self.reader.latest_position['angle']:.1f}°"
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

        self.tag_button = QPushButton('TAG', self)
        self.tag_button.move(288, 740)
        self.tag_button.clicked.connect(self.start_tag)
        self.tag_button.resize(75,70)


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
        self.build_user_layout()

    def build_user_layout(self):
        """将原有控件组织成面向操作员的工作流界面。"""
        self.setStyleSheet("""
            QWidget {
                background: #F5F7FA;
                color: #182230;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
                font-size: 14px;
            }
            QLabel#pageTitle { color: #101828; font-size: 24px; font-weight: 700; }
            QLabel#subTitle { color: #667085; font-size: 13px; }
            QLabel#chip {
                background: #EEF2F7; color: #475467; border-radius: 12px;
                padding: 5px 10px;
            }
            QLabel#statusBar {
                background: #EEF4FF; color: #175CD3; border-radius: 7px;
                padding: 9px 12px;
            }
            QGroupBox {
                background: #FFFFFF; border: 1px solid #E1E7EF; border-radius: 10px;
                margin-top: 14px; padding: 14px 12px 12px; font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 12px; padding: 0 5px; color: #344054;
            }
            QGroupBox QLabel { background: transparent; }
            QPushButton {
                min-height: 36px; background: #FFFFFF; border: 1px solid #CDD5DF;
                border-radius: 7px; padding: 4px 13px;
            }
            QPushButton:hover { background: #F8FAFC; border-color: #84A3CC; }
            QPushButton:pressed { background: #EAF0F8; }
            QPushButton:disabled { background: #F2F4F7; color: #98A2B3; border-color: #EAECF0; }
            QPushButton[primary="true"] {
                background: #1769E0; color: white; border-color: #1769E0; font-weight: 600;
            }
            QPushButton[primary="true"]:hover { background: #145BC2; }
            QPushButton[danger="true"] { color: #B42318; border-color: #FDA29B; }
            QPushButton[primary="true"]:disabled,
            QPushButton[danger="true"]:disabled {
                background: #F2F4F7; color: #98A2B3; border-color: #EAECF0;
            }
            QLineEdit {
                min-height: 34px; background: #FFFFFF; border: 1px solid #CDD5DF;
                border-radius: 6px; padding: 0 9px;
            }
            QLineEdit:focus { border: 2px solid #1769E0; }
            QTabWidget::pane { border: 0; }
            QTabBar::tab { background: transparent; color: #667085; padding: 10px 22px; }
            QTabBar::tab:selected {
                color: #175CD3; font-weight: 600; border-bottom: 2px solid #175CD3;
            }
        """)

        self.g2d_button.setText('启动定位')
        self.g2d_exit_button.setText('停止定位')
        self.rviz_button.setText('打开 RViz')
        self.nav_button.setText('启动自动导航')
        self.nav5_button.setText('启动 MoveBase')
        self.quitnav_button.setText('停止导航')
        self.nav_runstart.setText('开始任务')
        self.nav_Pause.setText('暂停任务')
        self.nav_resume.setText('继续任务')
        self.start_button.setText('进入点位设置')
        self.stop_button.setText('结束点位设置')
        self.add_pnt.setText('添加导航点')
        self.add_workstation.setText('添加工位')
        self.location_pnt.setText('在 RViz 显示点位')
        self.target_send_button.setText('发送目标')
        self.can_button.setText('启动 CAN')
        self.base_button.setText('启动底盘')
        self.joy_button.setText('启动手柄')
        self.tag_button.setText('启动标签读取')
        self.quit_button.setText('退出控制台')
        self.position_label.setText('最新标签定位：暂无数据')

        for button in (self.g2d_button, self.nav_button, self.nav_runstart, self.target_send_button,
                       self.start_button):
            button.setProperty('primary', True)
        for button in (self.g2d_exit_button, self.quitnav_button, self.stop_button, self.quit_button):
            button.setProperty('danger', True)

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 16, 24, 14)
        root.setSpacing(12)

        header = QHBoxLayout()
        heading = QVBoxLayout()
        title = QLabel('移动机器人作业控制台')
        title.setObjectName('pageTitle')
        subtitle = QLabel('定位、目标下发与点位管理')
        subtitle.setObjectName('subTitle')
        heading.addWidget(title)
        heading.addWidget(subtitle)
        header.addLayout(heading)
        header.addStretch()
        self.ros_chip = self.create_status_chip('ROS 已连接')
        self.localization_chip = self.create_status_chip('定位 未启动')
        self.nav_chip = self.create_status_chip('导航 未启动')
        header.addWidget(self.ros_chip)
        header.addWidget(self.localization_chip)
        header.addWidget(self.nav_chip)
        root.addLayout(header)

        divider = QFrame()
        divider.setFrameShape(QFrame.HLine)
        divider.setStyleSheet('color: #E1E7EF;')
        root.addWidget(divider)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_navigation_page(), '导航作业')
        self.tabs.addTab(self.build_points_page(), '点位管理')
        self.tabs.addTab(self.build_tools_page(), '设备与工具')
        root.addWidget(self.tabs, 1)

        footer = QHBoxLayout()
        self.status_label = QLabel('系统就绪，请先启动定位。')
        self.status_label.setObjectName('statusBar')
        self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        footer.addWidget(self.status_label, 1)
        footer.addWidget(self.quit_button)
        root.addLayout(footer)

        self.stop_button.setEnabled(False)
        self.g2d_exit_button.setEnabled(False)
        self.quitnav_button.setEnabled(False)

    def create_status_chip(self, text):
        chip = QLabel(text)
        chip.setObjectName('chip')
        chip.setAlignment(Qt.AlignCenter)
        return chip

    def build_navigation_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(14)

        startup = QGroupBox('1  作业准备')
        startup_layout = QGridLayout(startup)
        startup_layout.setHorizontalSpacing(10)
        startup_layout.setVerticalSpacing(10)
        help_text = QLabel('先完成定位，再启动导航。RViz 用于查看地图和机器人位姿。')
        help_text.setObjectName('subTitle')
        startup_buttons = (
            self.g2d_button, self.g2d_exit_button, self.rviz_button,
            self.nav_button, self.nav5_button, self.quitnav_button,
        )
        startup_layout.addWidget(help_text, 0, 0, 1, len(startup_buttons))
        for column, button in enumerate(startup_buttons):
            startup_layout.addWidget(button, 1, column)
            startup_layout.setColumnStretch(column, 1)
        layout.addWidget(startup)

        task = QGroupBox('2  任务控制与快捷目标')
        task_layout = QVBoxLayout(task)
        task_layout.setSpacing(10)
        control_row = QHBoxLayout()
        control_row.setSpacing(8)
        for button in (self.nav_runstart, self.nav_Pause, self.nav_resume):
            control_row.addWidget(button)
        control_row.addStretch()
        task_layout.addLayout(control_row)
        destination_grid = QGridLayout()
        destination_grid.setHorizontalSpacing(8)
        destination_grid.setVerticalSpacing(8)
        destination_buttons = (
            self.p1_button, self.p2_button, self.p3_button, self.p4_button, self.p5_button,
            self.w1_button, self.w2_button, self.w3_button,
        )
        for index, button in enumerate(destination_buttons):
            button.setText(f'导航点 {index + 1}' if index < 5 else f'工位 W{index - 4}')
            destination_grid.addWidget(button, 0, index)
            destination_grid.setColumnStretch(index, 1)
        task_layout.addLayout(destination_grid)
        layout.addWidget(task)

        target = QGroupBox('3  精细调整')
        target_layout = QVBoxLayout(target)
        target_layout.setSpacing(9)
        target_row = QHBoxLayout()
        target_row.setSpacing(8)
        self.x_label.setText('X (m)')
        self.y_label.setText('Y (m)')
        self.angle_label.setText('角度 (°)')
        for label, editor in ((self.x_label, self.x_input), (self.y_label, self.y_input),
                              (self.angle_label, self.angle_input)):
            target_row.addWidget(label)
            target_row.addWidget(editor)
        target_row.addWidget(self.target_send_button)
        target_layout.addLayout(target_row)
        self.target_value_label.setText('当前目标：X 0.00 m  ·  Y 0.00 m  ·  角度 0.00°')
        self.target_value_label.setObjectName('subTitle')
        target_layout.addWidget(self.target_value_label)
        quick_row = QHBoxLayout()
        quick_row.setSpacing(8)
        quick_row.addWidget(QLabel('Y 快捷值'))
        quick_values = (-0.10, 0.00, 0.07, 0.10, 0.20)
        for button, value in zip(self.y_quick_buttons, quick_values):
            button.setText(f'{value:+.2f} m')
            quick_row.addWidget(button)
        quick_row.addStretch()
        target_layout.addLayout(quick_row)
        layout.addWidget(target)
        layout.addStretch()
        return page

    def build_points_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(14)
        editor = QGroupBox('点位采集')
        editor_layout = QVBoxLayout(editor)
        editor_layout.setSpacing(12)
        note = QLabel('进入设置模式后，将机器人移到目标位置，再添加导航点或工位。')
        note.setWordWrap(True)
        note.setObjectName('subTitle')
        editor_layout.addWidget(note)
        mode_row = QHBoxLayout()
        mode_row.setSpacing(10)
        mode_row.addWidget(self.start_button)
        mode_row.addWidget(self.stop_button)
        mode_row.addStretch()
        editor_layout.addLayout(mode_row)
        action_row = QHBoxLayout()
        action_row.setSpacing(10)
        action_row.addWidget(self.add_pnt)
        action_row.addWidget(self.add_workstation)
        action_row.addWidget(self.location_pnt)
        action_row.addStretch()
        editor_layout.addLayout(action_row)
        layout.addWidget(editor)
        layout.addStretch()
        return page

    def build_tools_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(14)
        devices = QGroupBox('设备启动')
        grid = QGridLayout(devices)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        for index, button in enumerate((self.can_button, self.base_button, self.joy_button, self.tag_button)):
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(devices)
        self.position_label.setObjectName('statusBar')
        layout.addWidget(self.position_label)
        layout.addStretch()
        return page

    def set_status(self, message, level='info'):
        colors = {
            'info': ('#EEF4FF', '#175CD3'),
            'success': ('#ECFDF3', '#067647'),
            'warning': ('#FFFAEB', '#B54708'),
            'error': ('#FEF3F2', '#B42318'),
        }
        background, foreground = colors.get(level, colors['info'])
        self.status_label.setText(message)
        self.status_label.setStyleSheet(
            f'background: {background}; color: {foreground}; border-radius: 7px; padding: 9px 12px;'
        )

    def set_chip(self, chip, text, active=False):
        if active:
            chip.setStyleSheet('background: #ECFDF3; color: #067647; border-radius: 12px; padding: 5px 10px;')
        else:
            chip.setStyleSheet('background: #EEF2F7; color: #475467; border-radius: 12px; padding: 5px 10px;')
        chip.setText(text)

    def start_managed_process(self, command):
        """直接启动子进程并创建独立进程组，便于可靠停止 roslaunch 及其子节点。"""
        return subprocess.Popen(command, start_new_session=True)

    def stop_managed_process(self, process, label):
        """先请求 ROS 正常退出，超时后再逐级强制结束。"""
        if process is None:
            return False
        if process.poll() is not None:
            return True
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass
        except OSError as error:
            print(f"Failed to stop {label}: {error}")
            return False
        return True

    def is_move_base_ready(self):
        try:
            result = subprocess.run(
                ['rosnode', 'list'], capture_output=True, text=True, timeout=1
            )
            nodes = {name.strip() for name in result.stdout.splitlines()}
            return '/mxb_move_base' in nodes
        except (OSError, subprocess.TimeoutExpired):
            return False

    def set_nav_shortcuts_visible(self, visible):
        for button in (
            self.p1_button, self.p2_button, self.p3_button, self.p4_button, self.p5_button,
            self.w1_button, self.w2_button, self.w3_button
        ):
            button.setEnabled(visible)

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
            f"当前目标：X {self.current_target_x:.2f} m  ·  "
            f"Y {self.current_target_y:.2f} m  ·  角度 {self.current_target_angle:.2f}°"
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
        try:
            self.runGoProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)
            self.set_status(
                f'已发送精细调整目标：X {float(posX):.2f} m，Y {float(posY):.2f} m，角度 {float(posAngle):.2f}°',
                'success',
            )
        except Exception as error:
            self.set_status(f'目标发送失败：{error}', 'error')

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

        target_name = f'导航点 {id + 1}' if id >= 0 else f'工位 W{-id}'
        self.set_status(f'正在下发目标：{target_name}…', 'info')

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
        self.runJoyProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)
        self.set_status('手柄驱动已启动。', 'success')

    def start_base(self):
                     # 定义要执行的 ROS 命令
        ros_command = 'roslaunch ranger_bringup ranger_mini_v2.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.baseProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)
        self.set_status('底盘驱动已启动。', 'success')

    def start_tag(self):
    # 创建完整的 gnome-terminal 命令字符串
    # 定义每个标签页的命令
        tcp_server = shlex.quote(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tcpserver.py'))
        commands = [
            ['gnome-terminal', '--tab', '--title=MM3V Tag', '--', 'bash', '-c', 'roslaunch robot_r 6tagReadAndCtl_mm3v.launch; exec bash'],
            ['gnome-terminal', '--tab', '--title=TCP Server', '--', 'bash', '-c', f'python3 {tcp_server}; exec bash']
        ]
        try:
            # 分别启动每个标签页
            for cmd in commands:
                subprocess.Popen(cmd)
            self.set_status('标签读取与 TCP 服务已启动。', 'success')
        except Exception as e:
            print(f"Failed to run command: {e}")
            self.set_status(f'标签服务启动失败：{e}', 'error')
                            
    def start_can(self):
        try:
            # 不在代码中保存 sudo 密码，由系统终端正常请求授权。
            can_command = "sudo ip link set can0 up type can bitrate 500000"

            # 构建完整的终端命令
            terminal_command = ['gnome-terminal', '--', 'bash', '-c', can_command + '; exec bash']

            # 使用 subprocess.Popen 启动新的终端窗口
            subprocess.Popen(terminal_command)
            self.set_status('CAN 启动命令已打开，请在终端完成授权。', 'warning')

            # 显示成功消息
            #QMessageBox.information(self, '成功', '已启动新的终端并设置 CAN 接口！')
        except Exception as e:
            # 如果失败，显示错误消息
            error_message = f"错误: {str(e)}"
            self.set_status(f'CAN 启动失败：{error_message}', 'error')
            #QMessageBox.critical(self, '错误', error_message)

    def send_joy_message(self,cmd):
        #"""发送 /joy 消息"""
        if self.joy_pub is None:
            self.set_status('ROS 未连接，无法发送点位指令。', 'error')
            return
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
        if self.joy_pub is not None:
            self.set_status('已发送“添加导航点”指令。', 'success')

    def add_workstation_fun(self):
        self.send_joy_message("add_workstation")
        if self.joy_pub is not None:
            self.set_status('已发送“添加工位”指令。', 'success')

    def show_pnt_fun(self):
        self.send_joy_message("show")
        if self.joy_pub is not None:
            self.set_status('已在 RViz 中请求显示点位。', 'success')

    def Nav_start(self):
        self.send_joy_message("NavStart")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已开始。', 'success')
    
    def Nav_Zanting(self):
        self.send_joy_message("NavPause")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已暂停。', 'warning')
    
    def Nav_jixu(self):
        self.send_joy_message("Navresume")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已继续。', 'success')

    def start_setlocation(self):
        if self.setpointProcess and self.setpointProcess.poll() is None:
            self.set_status('点位设置已在运行。', 'info')
            return
        try:
            self.setpointProcess = self.start_managed_process(
                ['roslaunch', 'robot_r', '3settinglocation.launch']
            )
        except OSError as error:
            self.set_status(f'点位设置启动失败：{error}', 'error')
            return
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.location_pnt.show()
        self.add_pnt.show()
        self.add_workstation.show()
        self.set_status('已进入点位设置模式，请移动机器人后添加点位。', 'success')

    def stop_setlocation(self):
        had_process = self.setpointProcess is not None
        self.stop_managed_process(self.setpointProcess, 'point setting')
        self.setpointProcess = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.add_pnt.hide()
        self.add_workstation.hide()
        self.location_pnt.hide()
        if had_process:
            self.set_status('已结束点位设置，相关 ROS 节点已退出。', 'info')

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
            self.set_chip(self.localization_chip, '定位 运行中', True)
            self.set_status('定位已启动，正在等待地图和位姿数据。', 'success')
        if not self.is_rviz_running():
            self.start_plain_rviz()

    def start_rviz(self):
        if self.is_rviz_running():
            self.set_status('RViz 已在运行。', 'info')
            return
        command = ['rviz', '-d', PANEL_RVIZ_CONFIG]
        self.rvizProcess = subprocess.Popen(command, preexec_fn=os.setpgrp)
        self.rviz_panel_mode = True
        self.rviz_button.setEnabled(False)
        self.set_status('RViz 已启动。', 'success')

    def nav5(self):
        if self.is_move_base_ready():
            self.set_status('MoveBase 已在运行。', 'info')
            return True
        if self.moveBaseProcess and self.moveBaseProcess.poll() is None:
            self.set_status('MoveBase 正在启动…', 'info')
            return True
        try:
            self.moveBaseProcess = self.start_managed_process(
                ['roslaunch', 'robot_r', '5nav.launch']
            )
        except OSError as error:
            self.set_status(f'MoveBase 启动失败：{error}', 'error')
            return False
        self.quitnav_button.setEnabled(True)
        self.set_chip(self.nav_chip, '导航 启动中', False)
        self.set_status('MoveBase 正在启动…', 'info')
        return True

    # load points and auto run
    def startNAV(self):
        if self.runPntsNavProcess and self.runPntsNavProcess.poll() is None:
            self.set_status('自动导航已在运行。', 'info')
            return
        if self.auto_nav_start_pending:
            self.set_status('正在等待 MoveBase 就绪…', 'info')
            return
        if self.is_move_base_ready():
            self.launch_auto_navigation()
            return

        self.auto_nav_start_pending = True
        self.move_base_wait_attempts = 0
        self.nav_button.setEnabled(False)
        self.quitnav_button.setEnabled(True)
        if not self.nav5():
            self.auto_nav_start_pending = False
            self.nav_button.setEnabled(True)
            return
        self.set_status('未检测到 MoveBase，已自动启动，正在等待就绪…', 'info')
        QTimer.singleShot(500, self.wait_for_move_base_then_start_auto_nav)

    def wait_for_move_base_then_start_auto_nav(self):
        if not self.auto_nav_start_pending:
            return
        if self.is_move_base_ready():
            self.auto_nav_start_pending = False
            self.launch_auto_navigation()
            return
        if self.moveBaseProcess and self.moveBaseProcess.poll() is not None:
            self.auto_nav_start_pending = False
            self.nav_button.setEnabled(True)
            self.quitnav_button.setEnabled(False)
            self.set_chip(self.nav_chip, '导航 启动失败', False)
            self.set_status('MoveBase 进程已退出，自动导航未启动，请查看终端日志。', 'error')
            return
        self.move_base_wait_attempts += 1
        if self.move_base_wait_attempts >= 60:
            self.auto_nav_start_pending = False
            self.nav_button.setEnabled(True)
            self.set_chip(self.nav_chip, '导航 启动超时', False)
            self.set_status('MoveBase 在 30 秒内未就绪，已取消启动自动导航。', 'error')
            return
        QTimer.singleShot(500, self.wait_for_move_base_then_start_auto_nav)

    def launch_auto_navigation(self):
        try:
            self.runPntsNavProcess = self.start_managed_process(
                ['roslaunch', 'robot_r', '3navlocations.launch']
            )
        except OSError as error:
            self.nav_button.setEnabled(True)
            self.set_status(f'自动导航启动失败：{error}', 'error')
            return
        self.nav_button.setEnabled(False)
        self.quitnav_button.setEnabled(True)
        self.nav_runstart.setEnabled(True)
        self.nav_Pause.setEnabled(True)
        self.nav_resume.setEnabled(True)
        self.set_nav_shortcuts_visible(True)
        self.set_chip(self.nav_chip, '导航 运行中', True)
        self.set_status('自动导航已启动，可选择快捷目标。', 'success')

    def exitNAV(self):
        stopped = (
            self.auto_nav_start_pending or self.runPntsNavProcess is not None
            or self.moveBaseProcess is not None
        )
        self.auto_nav_start_pending = False
        self.set_status('正在停止自动导航和 MoveBase…', 'info')
        self.stop_managed_process(self.runPntsNavProcess, 'auto navigation')
        self.runPntsNavProcess = None
        self.stop_managed_process(self.moveBaseProcess, 'MoveBase')
        self.moveBaseProcess = None
        self.nav_button.setEnabled(True)
        self.quitnav_button.setEnabled(False)
        self.nav_runstart.setEnabled(False)
        self.nav_Pause.setEnabled(False)
        self.nav_resume.setEnabled(False)
        self.set_nav_shortcuts_visible(False)
        if stopped:
            self.set_chip(self.nav_chip, '导航 未启动', False)
            self.set_status('自动导航和 MoveBase 已停止。', 'info')


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
        self.g2d_exit_button.setEnabled(False)
        self.set_chip(self.localization_chip, '定位 未启动', False)
        self.set_status('定位已停止。', 'info')

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
