import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLineEdit, QGroupBox, QTabWidget, QFrame, QSizePolicy,
    QRadioButton, QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QMessageBox, QFileDialog, QDialog, QFormLayout
)
import subprocess
import time
import shlex
import shutil
import math
import glob

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QDoubleValidator

import os
import signal

import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray

from tagReader import SerialTagReader
import threading


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def find_workspace_root():
    """兼容 <workspace>/script 和标准的 <workspace>/src/script 两种目录结构。"""
    parent = os.path.dirname(SCRIPT_DIR)
    candidates = (parent, os.path.dirname(parent))
    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, 'devel', 'env.sh')):
            return candidate
    return parent


WORKSPACE_ROOT = find_workspace_root()


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
for python_path in os.environ.get('PYTHONPATH', '').split(os.pathsep):
    if python_path and python_path not in sys.path:
        sys.path.insert(0, python_path)

from cmd_vel_arbiter.srv import FinishMotion, FinishMotionRequest



def robot_r_path():
    try:
        return subprocess.check_output(['rospack', 'find', 'robot_r'], text=True).strip()
    except Exception:
        direct_path = os.path.join(WORKSPACE_ROOT, 'robot_r')
        standard_path = os.path.join(WORKSPACE_ROOT, 'src', 'robot_r')
        return direct_path if os.path.isdir(direct_path) else standard_path


ROBOT_R_PATH = robot_r_path()
PANEL_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz.rviz')
PLAIN_RVIZ_CONFIG = os.path.join(ROBOT_R_PATH, 'rviz', 'nav_rviz_no_panel.rviz')
MAP_OUTPUT_PREFIX = os.path.join(os.path.expanduser('~'), 'maps', 'map')
ROBOT_POSITIONS_FILE = os.path.join(
    os.path.expanduser('~'), 'maps', 'robot_positions.txt'
)
TOPOLOGY_BUILDER = os.path.join(SCRIPT_DIR, 'build_topology.py')
RVIZ_RECORD_POSE_TOPIC = '/anav/record_pose'
RVIZ_RECORD_MARKER_TOPIC = '/anav/record_markers'


class TerminalTabProcess:
    """Expose the terminal child PID while using gnome-terminal --wait for lifecycle checks."""

    def __init__(self, launcher, child_pid):
        self.launcher = launcher
        self.pid = child_pid

    def poll(self):
        return self.launcher.poll()

    def wait(self, timeout=None):
        return self.launcher.wait(timeout=timeout)


class PointEditDialog(QDialog):
    """Edit a saved point manually or request a replacement pose from RViz."""

    rviz_requested = pyqtSignal()
    save_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('修改选中点位')
        self.setModal(False)
        self.setMinimumWidth(480)

        layout = QVBoxLayout(self)
        note = QLabel(
            '可以直接修改数值；也可以从 RViz 重新选点，收到位姿后再确认保存。'
        )
        note.setWordWrap(True)
        note.setObjectName('subTitle')
        layout.addWidget(note)

        form = QFormLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(10)
        self.name_input = QLineEdit()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.yaw_input = QLineEdit()
        self.x_input.setValidator(QDoubleValidator(-10000.0, 10000.0, 6, self))
        self.y_input.setValidator(QDoubleValidator(-10000.0, 10000.0, 6, self))
        self.yaw_input.setValidator(QDoubleValidator(-360.0, 360.0, 3, self))
        form.addRow('名称', self.name_input)
        form.addRow('X (m)', self.x_input)
        form.addRow('Y (m)', self.y_input)
        form.addRow('角度 (°)', self.yaw_input)
        layout.addLayout(form)

        self.rviz_note = QLabel('尚未请求 RViz 位姿。')
        self.rviz_note.setObjectName('subTitle')
        self.rviz_note.setWordWrap(True)
        layout.addWidget(self.rviz_note)

        actions = QHBoxLayout()
        self.rviz_button = QPushButton('从 RViz 重新选点')
        self.rviz_button.clicked.connect(self.rviz_requested.emit)
        self.cancel_button = QPushButton('取消')
        self.cancel_button.clicked.connect(self.reject)
        self.save_button = QPushButton('确认修改')
        self.save_button.setProperty('primary', True)
        self.save_button.clicked.connect(self.save_requested.emit)
        actions.addWidget(self.rviz_button)
        actions.addStretch()
        actions.addWidget(self.cancel_button)
        actions.addWidget(self.save_button)
        layout.addLayout(actions)

    def set_point(self, index, pose):
        self.setWindowTitle(f'修改点位 P{index}')
        self.name_input.setText(pose.get('label', ''))
        self.x_input.setText(f"{pose['x']:.6f}")
        self.y_input.setText(f"{pose['y']:.6f}")
        self.yaw_input.setText(f"{math.degrees(pose['yaw']):.3f}")

    def set_rviz_pose(self, x, y, yaw):
        self.x_input.setText(f'{x:.6f}')
        self.y_input.setText(f'{y:.6f}')
        self.yaw_input.setText(f'{math.degrees(yaw):.3f}')
        self.rviz_note.setText(
            '已收到 RViz 位姿并回填；检查数值后点击“确认修改”。'
        )

    def values(self):
        name = self.name_input.text().strip()
        return (
            name,
            float(self.x_input.text().strip()),
            float(self.y_input.text().strip()),
            float(self.yaw_input.text().strip()),
        )


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
    status_requested = pyqtSignal(str, str)
    localization_ready_requested = pyqtSignal()
    record_pose_requested = pyqtSignal(float, float, float, str)
    odometry_requested = pyqtSignal(float, float, float, float, float)
    health_snapshot_requested = pyqtSignal(object)
    topology_finished_requested = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('移动机器人作业控制台')
        self.setMinimumSize(820, 720)
        self.resize(960, 820)

        self.preview_mode = os.environ.get('ANAV_GUI_PREVIEW') == '1'
        self.ros_available = False if self.preview_mode else check_and_start_roscore()
        self.joy_pub = None
        self.safety_pub = None
        self.finish_motion_proxy = None
        self.task_status_sub = None
        self.localization_odom_sub = None
        self.record_pose_sub = None
        self.record_marker_pub = None
        self.localizationProcess = None
        self.localization_ready = False
        self.currentID = 0
        self.pending_record_pose = None
        self.point_edit_dialog = None
        self.point_edit_index = None
        self.point_edit_file_signature = None
        self.point_edit_waiting_for_rviz = False
        self.point_edit_preview_pose = None
        self.record_session_active = False
        self.record_file_backup = ''
        self.point_undo_stack = []
        self.positions_file_stamp = None
        self.dynamic_nav_buttons = []
        self.nav_shortcuts_enabled = False
        self.last_odom_monotonic = 0.0
        self.health_check_running = False
        self.estop_active = False
        self.shutdown_in_progress = False
        self.topology_build_running = False
        if self.ros_available:
            try:
                rospy.init_node('upmachine_publisher', anonymous=True, disable_signals=True)
                self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
                self.safety_pub = rospy.Publisher(
                    '/cmd_vel/safety', Twist, queue_size=1
                )
                self.finish_motion_proxy = rospy.ServiceProxy(
                    '/cmd_vel_arbiter/finish_motion', FinishMotion
                )
                self.record_marker_pub = rospy.Publisher(
                    RVIZ_RECORD_MARKER_TOPIC, MarkerArray, queue_size=1, latch=True
                )
            except Exception as error:
                print(f"ROS initialization failed: {error}")
                self.ros_available = False

        self.initUI()
        self.status_requested.connect(self.set_status)
        self.localization_ready_requested.connect(self.mark_localization_ready)
        self.record_pose_requested.connect(self.accept_record_pose)
        self.odometry_requested.connect(self.update_odometry_display)
        self.health_snapshot_requested.connect(self.apply_health_snapshot)
        self.topology_finished_requested.connect(self.on_topology_finished)
        if self.ros_available:
            try:
                self.task_status_sub = rospy.Subscriber(
                    '/anav/task_status', String, self.on_task_status, queue_size=10
                )
                self.localization_odom_sub = rospy.Subscriber(
                    '/Odometry', Odometry, self.on_localization_odometry, queue_size=1
                )
                self.record_pose_sub = rospy.Subscriber(
                    RVIZ_RECORD_POSE_TOPIC,
                    PoseStamped,
                    self.on_record_pose,
                    queue_size=1,
                )
            except Exception as error:
                print(f"ROS status subscription failed: {error}")
        self.set_chip(
            self.ros_chip,
            'ROS 已连接' if self.ros_available else 'ROS 未连接',
            self.ros_available,
        )
        if not self.ros_available and not self.preview_mode:
            self.set_status('ROS 连接失败，请检查 ROS_MASTER_URI 或启动日志。', 'error')
        self.setpointProcess = None
        self.rvizProcess = None
        self.rviz_panel_mode = False
        self.runPntsNavProcess = None
        self.moveBaseProcess = None
        self.bsplineLogProcess = None
        self.baseProcess = None
        self.joyProcess = None
        self.tagProcesses = []
        self.mappingProcess = None
        self.g2dProcess = None
        self.mapSaverProcess = None
        self.map_save_generation = 0
        self.auto_nav_start_pending = False
        self.move_base_wait_attempts = 0
        self.add_pnt.hide()
        self.add_workstation.hide()
        self.location_pnt.hide()
        self.nav_runstart.setEnabled(False)
        self.nav_Pause.setEnabled(False)
        self.nav_resume.setEnabled(False)
        self.set_nav_shortcuts_visible(False)
        self.refresh_points_table()

        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_angle = 0.0

        self.estop_timer = QTimer(self)
        self.estop_timer.timeout.connect(self.publish_safety_stop)
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.schedule_health_check)
        self.health_timer.start(1000)
        QTimer.singleShot(0, self.schedule_health_check)

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

        self.real_record_mode = QRadioButton('实车采点', self)
        self.rviz_record_mode = QRadioButton('RViz 采点（无需动车）', self)
        self.real_record_mode.setChecked(True)
        self.real_record_mode.toggled.connect(self.on_record_mode_changed)

        self.record_mode_note = QLabel(self)
        self.record_mode_note.setWordWrap(True)
        self.record_mode_note.setObjectName('subTitle')
        self.record_pose_label = QLabel('尚未从 RViz 选择目标位姿', self)
        self.record_pose_label.setWordWrap(True)
        self.record_pose_label.setObjectName('subTitle')

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
            QLabel#dataValue {
                background: #F8FAFC; color: #344054; border-radius: 7px;
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
            QPushButton[emergency="true"] {
                background: #D92D20; color: white; border-color: #D92D20;
                font-weight: 700;
            }
            QPushButton[emergency="true"]:hover { background: #B42318; }
            QPushButton[primary="true"]:disabled,
            QPushButton[danger="true"]:disabled,
            QPushButton[emergency="true"]:disabled {
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
            QTableWidget {
                background: #FFFFFF; alternate-background-color: #F8FAFC;
                border: 1px solid #E1E7EF; border-radius: 7px;
                gridline-color: #EAECF0;
            }
            QHeaderView::section {
                background: #F2F4F7; color: #344054; border: 0;
                border-bottom: 1px solid #D0D5DD; padding: 7px;
                font-weight: 600;
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
        self.tabs.addTab(self.build_mapping_page(), '地图构建')
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

        task = QGroupBox('2  任务控制与工位快捷目标')
        task_layout = QVBoxLayout(task)
        task_layout.setSpacing(10)
        control_row = QHBoxLayout()
        control_row.setSpacing(8)
        for button in (self.nav_runstart, self.nav_Pause, self.nav_resume):
            control_row.addWidget(button)
        self.cancel_task_button = QPushButton('取消当前任务')
        self.cancel_task_button.setProperty('danger', True)
        self.cancel_task_button.clicked.connect(self.cancel_current_task)
        self.cancel_task_button.setEnabled(False)
        control_row.addWidget(self.cancel_task_button)
        control_row.addStretch()
        self.estop_button = QPushButton('紧急停止')
        self.estop_button.setProperty('emergency', True)
        self.estop_button.clicked.connect(self.activate_emergency_stop)
        self.release_estop_button = QPushButton('解除急停')
        self.release_estop_button.clicked.connect(self.release_emergency_stop)
        self.release_estop_button.setEnabled(False)
        control_row.addWidget(self.estop_button)
        control_row.addWidget(self.release_estop_button)
        task_layout.addLayout(control_row)
        for button in (
            self.p1_button, self.p2_button, self.p3_button, self.p4_button,
            self.p5_button, self.w1_button, self.w2_button, self.w3_button,
        ):
            button.hide()
        self.destination_hint = QLabel(
            '暂无工位，请先在“点位管理”中添加或命名 W1、W2…工位。'
        )
        self.destination_hint.setObjectName('subTitle')
        task_layout.addWidget(self.destination_hint)
        self.dynamic_destination_widget = QWidget()
        self.dynamic_destination_grid = QGridLayout(self.dynamic_destination_widget)
        self.dynamic_destination_grid.setContentsMargins(0, 0, 0, 0)
        self.dynamic_destination_grid.setHorizontalSpacing(8)
        self.dynamic_destination_grid.setVerticalSpacing(8)
        task_layout.addWidget(self.dynamic_destination_widget)
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
        mode_select_row = QHBoxLayout()
        mode_select_row.setSpacing(18)
        mode_select_row.addWidget(QLabel('采集方式'))
        mode_select_row.addWidget(self.real_record_mode)
        mode_select_row.addWidget(self.rviz_record_mode)
        mode_select_row.addStretch()
        editor_layout.addLayout(mode_select_row)
        editor_layout.addWidget(self.record_mode_note)
        editor_layout.addWidget(self.record_pose_label)
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

        point_list = QGroupBox('已保存点位')
        point_list_layout = QVBoxLayout(point_list)
        point_list_layout.setSpacing(10)
        self.points_table = QTableWidget(0, 6)
        self.points_table.setHorizontalHeaderLabels(
            ('序号', '类型/名称', 'X (m)', 'Y (m)', '朝向 (°)', '操作目标')
        )
        self.points_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.points_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.points_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.points_table.setAlternatingRowColors(True)
        self.points_table.verticalHeader().setVisible(False)
        self.points_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.points_table.cellDoubleClicked.connect(
            lambda _row, _column: self.navigate_selected_point()
        )
        point_list_layout.addWidget(self.points_table)

        point_actions = QGridLayout()
        point_actions.setHorizontalSpacing(8)
        point_actions.setVerticalSpacing(8)
        self.refresh_points_button = QPushButton('刷新点位')
        self.refresh_points_button.clicked.connect(self.refresh_points_table)
        self.navigate_point_button = QPushButton('导航到选中点')
        self.navigate_point_button.setProperty('primary', True)
        self.navigate_point_button.clicked.connect(self.navigate_selected_point)
        self.rename_point_button = QPushButton('修改选中点')
        self.rename_point_button.clicked.connect(self.edit_selected_point)
        self.delete_point_button = QPushButton('删除选中点')
        self.delete_point_button.setProperty('danger', True)
        self.delete_point_button.clicked.connect(self.delete_selected_point)
        self.move_point_up_button = QPushButton('上移')
        self.move_point_up_button.clicked.connect(lambda: self.move_selected_point(-1))
        self.move_point_down_button = QPushButton('下移')
        self.move_point_down_button.clicked.connect(lambda: self.move_selected_point(1))
        self.undo_point_button = QPushButton('撤销修改')
        self.undo_point_button.clicked.connect(self.undo_point_change)
        self.restore_points_button = QPushButton('恢复最近备份')
        self.restore_points_button.clicked.connect(self.restore_latest_points_backup)
        self.import_points_button = QPushButton('导入点位')
        self.import_points_button.clicked.connect(self.import_points_file)
        self.export_points_button = QPushButton('导出点位')
        self.export_points_button.clicked.connect(self.export_points_file)
        point_action_buttons = (
            self.refresh_points_button, self.navigate_point_button,
            self.rename_point_button, self.delete_point_button,
            self.move_point_up_button, self.move_point_down_button,
            self.undo_point_button, self.restore_points_button,
            self.import_points_button, self.export_points_button,
        )
        for index, button in enumerate(point_action_buttons):
            point_actions.addWidget(button, index // 5, index % 5)
            point_actions.setColumnStretch(index % 5, 1)
        point_list_layout.addLayout(point_actions)
        layout.addWidget(point_list)
        layout.addStretch()
        self.on_record_mode_changed()
        return page

    def build_mapping_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(14)

        introduction = QLabel(
            '先构建三维点云地图，结束构建后再生成并保存二维占据地图。'
        )
        introduction.setObjectName('subTitle')
        introduction.setWordWrap(True)
        layout.addWidget(introduction)

        mapping_group = QGroupBox('1  构建三维地图')
        mapping_layout = QVBoxLayout(mapping_group)
        mapping_layout.setSpacing(10)
        mapping_note = QLabel('启动 LIO 建图后，控制机器人完成场地扫描。')
        mapping_note.setObjectName('subTitle')
        mapping_layout.addWidget(mapping_note)
        mapping_buttons = QHBoxLayout()
        mapping_buttons.setSpacing(10)
        self.mapping_start_button = QPushButton('开始构建地图')
        self.mapping_start_button.setProperty('primary', True)
        self.mapping_start_button.clicked.connect(self.start_mapping)
        self.mapping_stop_button = QPushButton('结束地图构建')
        self.mapping_stop_button.setProperty('danger', True)
        self.mapping_stop_button.setEnabled(False)
        self.mapping_stop_button.clicked.connect(self.stop_mapping)
        mapping_buttons.addWidget(self.mapping_start_button)
        mapping_buttons.addWidget(self.mapping_stop_button)
        mapping_buttons.addStretch()
        mapping_layout.addLayout(mapping_buttons)
        layout.addWidget(mapping_group)

        map_2d_group = QGroupBox('2  生成二维地图')
        map_2d_layout = QVBoxLayout(map_2d_group)
        map_2d_layout.setSpacing(10)
        output_label = QLabel(f'保存位置：{MAP_OUTPUT_PREFIX}.yaml / .pgm')
        output_label.setObjectName('subTitle')
        output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        map_2d_layout.addWidget(output_label)
        map_2d_buttons = QHBoxLayout()
        map_2d_buttons.setSpacing(10)
        self.generate_2d_button = QPushButton('生成并保存二维地图')
        self.generate_2d_button.setProperty('primary', True)
        self.generate_2d_button.clicked.connect(self.start_2d_map_generation)
        self.stop_2d_button = QPushButton('停止二维地图生成')
        self.stop_2d_button.setProperty('danger', True)
        self.stop_2d_button.setEnabled(False)
        self.stop_2d_button.clicked.connect(self.stop_2d_map_generation)
        map_2d_buttons.addWidget(self.generate_2d_button)
        map_2d_buttons.addWidget(self.stop_2d_button)
        map_2d_buttons.addStretch()
        map_2d_layout.addLayout(map_2d_buttons)
        layout.addWidget(map_2d_group)
        layout.addStretch()
        return page

    def build_tools_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 10, 0, 0)
        layout.setSpacing(12)

        devices = QGroupBox('设备启动')
        grid = QGridLayout(devices)
        grid.setHorizontalSpacing(10)
        device_buttons = (
            self.can_button, self.base_button, self.joy_button, self.tag_button
        )
        for index, button in enumerate(device_buttons):
            grid.addWidget(button, 0, index)
            grid.setColumnStretch(index, 1)
        layout.addWidget(devices)

        health = QGroupBox('实时运行状态')
        health_grid = QGridLayout(health)
        health_grid.setHorizontalSpacing(10)
        health_grid.setVerticalSpacing(8)
        health_names = (
            ('can', 'CAN'), ('base', '底盘'), ('arbiter', '速度仲裁'),
            ('localization', '定位数据'), ('move_base', 'MoveBase'),
            ('auto_nav', 'AutoNAV'), ('tag', '标签读取'),
        )
        self.health_chips = {}
        self.health_display_names = {}
        for index, (key, name) in enumerate(health_names):
            chip = self.create_status_chip(f'{name} · 未检测')
            chip.setMinimumHeight(30)
            self.health_chips[key] = chip
            self.health_display_names[key] = name
            row = index // 4
            column = index % 4
            health_grid.addWidget(chip, row, column)
            health_grid.setColumnStretch(column, 1)
        layout.addWidget(health)

        live_data = QGroupBox('实时数据')
        live_data_layout = QVBoxLayout(live_data)
        live_data_layout.setSpacing(8)
        self.odometry_label = QLabel('机器人位姿：暂无有效定位数据')
        self.odometry_label.setObjectName('dataValue')
        self.odometry_label.setMinimumHeight(36)
        self.odometry_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.position_label.setObjectName('dataValue')
        self.position_label.setMinimumHeight(36)
        live_data_layout.addWidget(self.odometry_label)
        live_data_layout.addWidget(self.position_label)
        layout.addWidget(live_data)

        topology = QGroupBox('导航功能')
        topology_layout = QHBoxLayout(topology)
        self.build_topology_button = QPushButton('构建导航拓扑')
        self.build_topology_button.setProperty('primary', True)
        self.build_topology_button.clicked.connect(self.build_topology)
        topology_note = QLabel(
            '根据 robot_positions.txt 与 map.yaml 生成 ~/maps/topology.yaml'
        )
        topology_note.setObjectName('subTitle')
        topology_layout.addWidget(self.build_topology_button)
        topology_layout.addWidget(topology_note, 1)
        layout.addWidget(topology)

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

    def on_task_status(self, message):
        """显示所有 /plan_path_and_go 调用的状态，包括 TCP 发起的任务。"""
        fields = message.data.split('\t', 3)
        if len(fields) != 4:
            print(f"Invalid /anav/task_status message: {message.data!r}")
            return

        state, requested_id_text, target_name, detail = fields
        if state == 'running':
            self.status_requested.emit(f'正在前往：{target_name}', 'info')
        elif state == 'arrived':
            try:
                self.currentID = int(requested_id_text)
            except ValueError:
                pass
            self.status_requested.emit(f'已到达：{target_name}', 'success')
        elif state == 'planned':
            self.status_requested.emit(f'路径规划完成：{target_name}', 'success')
        elif state == 'failed':
            failure_detail = detail or '未知原因'
            self.status_requested.emit(
                f'导航失败（{target_name}）：{failure_detail}', 'error'
            )

    def on_record_mode_changed(self, _checked=False):
        if self.rviz_record_mode.isChecked():
            self.record_mode_note.setText(
                '进入设置后，在 RViz 工具栏选择“记录点位”（快捷键 R），'
                '在地图上拖出箭头确定位置和朝向。'
            )
            self.record_pose_label.show()
        else:
            self.record_mode_note.setText(
                '进入设置后，将机器人移动到目标位置，再添加导航点或工位。'
            )
            self.record_pose_label.hide()

    def on_record_pose(self, message):
        """ROS 回调只解析消息，界面更新交给 Qt 主线程。"""
        recording_new_point = (
            self.record_session_active and self.rviz_record_mode.isChecked()
        )
        editing_point = (
            self.point_edit_waiting_for_rviz
            and self.point_edit_dialog is not None
        )
        if not recording_new_point and not editing_point:
            return
        frame_id = message.header.frame_id or 'map'
        if frame_id != 'map':
            self.status_requested.emit(
                f'拒绝记录 {frame_id} 坐标系的点；RViz Fixed Frame 必须设为 map。',
                'error',
            )
            return

        q = message.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.record_pose_requested.emit(
            message.pose.position.x,
            message.pose.position.y,
            yaw,
            frame_id,
        )

    def accept_record_pose(self, x, y, yaw, frame_id):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.set_status('RViz 返回的目标位姿包含无效数值。', 'error')
            return
        if (
            self.point_edit_waiting_for_rviz
            and self.point_edit_dialog is not None
        ):
            self.point_edit_waiting_for_rviz = False
            self.point_edit_preview_pose = (x, y, yaw)
            self.point_edit_dialog.set_rviz_pose(x, y, yaw)
            self.publish_record_markers()
            self.point_edit_dialog.raise_()
            self.point_edit_dialog.activateWindow()
            self.set_status(
                '已从 RViz 获取新位姿，请检查数值后确认修改。', 'success'
            )
            return
        if not self.record_session_active or not self.rviz_record_mode.isChecked():
            return
        self.pending_record_pose = (x, y, yaw)
        self.record_pose_label.setText(
            f'待记录目标（{frame_id}）：X {x:.3f} m  ·  Y {y:.3f} m  ·  '
            f'朝向 {math.degrees(yaw):.1f}°；Z/Roll/Pitch 保存时固定为 0'
        )
        self.publish_record_markers()
        self.set_status(
            '已收到 RViz 目标位姿，请选择“添加导航点”或“添加工位”。',
            'success',
        )

    def backup_and_reset_record_file(self):
        directory = os.path.dirname(ROBOT_POSITIONS_FILE)
        os.makedirs(directory, exist_ok=True)
        self.record_file_backup = ''
        if (
            os.path.isfile(ROBOT_POSITIONS_FILE)
            and os.path.getsize(ROBOT_POSITIONS_FILE) > 0
        ):
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            backup = os.path.join(
                directory, f'robot_positions_{timestamp}.txt'
            )
            suffix = 1
            while os.path.exists(backup):
                backup = os.path.join(
                    directory, f'robot_positions_{timestamp}_{suffix}.txt'
                )
                suffix += 1
            shutil.copy2(ROBOT_POSITIONS_FILE, backup)
            self.record_file_backup = backup

        with open(ROBOT_POSITIONS_FILE, 'w', encoding='utf-8'):
            pass

    def read_record_positions(self):
        poses = []
        if not os.path.isfile(ROBOT_POSITIONS_FILE):
            return poses
        try:
            with open(ROBOT_POSITIONS_FILE, 'r', encoding='utf-8') as handle:
                for line_number, raw_line in enumerate(handle, 1):
                    parts = raw_line.split()
                    if not parts:
                        continue
                    if len(parts) < 6:
                        print(
                            f'Ignore invalid point at line {line_number}: '
                            f'{raw_line.rstrip()}'
                        )
                        continue
                    try:
                        poses.append({
                            'x': float(parts[0]),
                            'y': float(parts[1]),
                            'z': float(parts[2]),
                            'roll': float(parts[3]),
                            'pitch': float(parts[4]),
                            'yaw': float(parts[5]),
                            'label': parts[6] if len(parts) > 6 else '',
                        })
                    except ValueError:
                        print(
                            f'Ignore invalid numeric point at line {line_number}: '
                            f'{raw_line.rstrip()}'
                        )
        except OSError as error:
            print(f'Unable to read {ROBOT_POSITIONS_FILE}: {error}')
        return poses

    def point_request_id(self, index, pose):
        label = pose.get('label', '')
        if label.startswith('W') and label[1:].isdigit():
            return -int(label[1:])
        return index

    def point_display_name(self, index, pose):
        return pose.get('label') or f'P{index}'

    def refresh_points_table(self):
        if not hasattr(self, 'points_table'):
            return
        poses = self.read_record_positions()
        self.points_table.setRowCount(len(poses))
        for index, pose in enumerate(poses):
            label = pose.get('label', '')
            point_type = label if label else f'导航点 P{index}'
            request_id = self.point_request_id(index, pose)
            values = (
                str(index),
                point_type,
                f"{pose['x']:.3f}",
                f"{pose['y']:.3f}",
                f"{math.degrees(pose['yaw']):.1f}",
                str(request_id),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                if column == 0:
                    item.setData(Qt.UserRole, index)
                self.points_table.setItem(index, column, item)
        self.points_table.resizeRowsToContents()
        self.positions_file_stamp = self.positions_file_signature()
        self.refresh_navigation_shortcuts(poses)
        has_points = bool(poses)
        for button in (
            self.rename_point_button, self.delete_point_button, self.move_point_up_button,
            self.move_point_down_button, self.export_points_button,
        ):
            button.setEnabled(has_points)
        self.navigate_point_button.setEnabled(
            has_points and self.nav_shortcuts_enabled
        )
        self.undo_point_button.setEnabled(bool(self.point_undo_stack))

    def positions_file_signature(self):
        try:
            stat = os.stat(ROBOT_POSITIONS_FILE)
            return stat.st_mtime_ns, stat.st_size
        except OSError:
            return None

    def maybe_refresh_points(self):
        if not hasattr(self, 'points_table'):
            return
        signature = self.positions_file_signature()
        if signature != self.positions_file_stamp:
            self.refresh_points_table()

    def refresh_navigation_shortcuts(self, poses=None):
        if not hasattr(self, 'dynamic_destination_grid'):
            return
        while self.dynamic_destination_grid.count():
            item = self.dynamic_destination_grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.dynamic_nav_buttons = []
        poses = self.read_record_positions() if poses is None else poses
        workstations = [
            (index, pose) for index, pose in enumerate(poses)
            if pose.get('label', '').startswith('W')
            and pose.get('label', '')[1:].isdigit()
        ]
        self.destination_hint.setVisible(not workstations)
        for column in range(4):
            self.dynamic_destination_grid.setColumnStretch(column, 0)
        self.dynamic_destination_grid.setColumnStretch(4, 1)
        for button_index, (index, pose) in enumerate(workstations):
            label = self.point_display_name(index, pose)
            request_id = self.point_request_id(index, pose)
            button = QPushButton(label)
            button.setToolTip(
                f"目标 {label}：X {pose['x']:.3f} m，Y {pose['y']:.3f} m，"
                f"朝向 {math.degrees(pose['yaw']):.1f}°"
            )
            button.clicked.connect(
                lambda _checked=False, target=request_id: self.gotop(target)
            )
            button.setFixedWidth(132)
            button.setEnabled(self.nav_shortcuts_enabled)
            self.dynamic_destination_grid.addWidget(
                button, button_index // 4, button_index % 4
            )
            self.dynamic_nav_buttons.append(button)

    def selected_point_index(self):
        selection = self.points_table.selectionModel().selectedRows()
        if not selection:
            self.set_status('请先在点位列表中选择一行。', 'warning')
            return None
        return selection[0].row()

    def navigate_selected_point(self):
        index = self.selected_point_index()
        if index is None:
            return
        poses = self.read_record_positions()
        if index >= len(poses):
            self.refresh_points_table()
            self.set_status('点位文件已变化，请重新选择目标。', 'warning')
            return
        self.gotop(self.point_request_id(index, poses[index]))

    def ensure_points_editable(self):
        if self.record_session_active:
            self.set_status('请先结束点位采集，再编辑点位列表。', 'warning')
            return False
        return True

    def snapshot_points_for_undo(self):
        try:
            with open(ROBOT_POSITIONS_FILE, 'r', encoding='utf-8') as handle:
                content = handle.read()
        except FileNotFoundError:
            content = ''
        self.point_undo_stack.append(content)
        if len(self.point_undo_stack) > 20:
            self.point_undo_stack.pop(0)

    def write_record_positions(self, poses, remember=True):
        if remember:
            self.snapshot_points_for_undo()
        directory = os.path.dirname(ROBOT_POSITIONS_FILE)
        os.makedirs(directory, exist_ok=True)
        temporary = f'{ROBOT_POSITIONS_FILE}.tmp.{os.getpid()}'
        try:
            with open(temporary, 'w', encoding='utf-8') as handle:
                for pose in poses:
                    line = (
                        f"{pose['x']:.6f} {pose.get('y', 0.0):.6f} "
                        f"{pose.get('z', 0.0):.6f} {pose.get('roll', 0.0):.6f} "
                        f"{pose.get('pitch', 0.0):.6f} {pose['yaw']:.6f}"
                    )
                    if pose.get('label'):
                        line += f" {pose['label']}"
                    handle.write(line + '\n')
            os.replace(temporary, ROBOT_POSITIONS_FILE)
        except OSError:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            raise
        self.refresh_points_table()
        self.publish_record_markers()

    def edit_selected_point(self):
        if not self.ensure_points_editable():
            return
        index = self.selected_point_index()
        if index is None:
            return
        poses = self.read_record_positions()
        if index >= len(poses):
            self.refresh_points_table()
            self.set_status('点位文件已变化，请重新选择要修改的点。', 'warning')
            return

        if self.point_edit_dialog is not None:
            self.point_edit_dialog.close()
        dialog = PointEditDialog(self)
        dialog.set_point(index, poses[index])
        dialog.rviz_requested.connect(self.request_rviz_point_for_edit)
        dialog.save_requested.connect(self.save_edited_point)
        dialog.finished.connect(self.finish_point_edit)
        self.point_edit_dialog = dialog
        self.point_edit_index = index
        self.point_edit_file_signature = self.positions_file_signature()
        self.point_edit_waiting_for_rviz = False
        self.point_edit_preview_pose = None
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self.set_status(
            f'正在修改 {self.point_display_name(index, poses[index])}。', 'info'
        )

    def request_rviz_point_for_edit(self):
        if self.point_edit_dialog is None or self.point_edit_index is None:
            return
        if not self.ros_available:
            self.set_status('ROS 未连接，无法从 RViz 获取点位。', 'error')
            return
        if self.is_move_base_ready():
            self.set_status(
                'MoveBase 正在运行；为避免误操作，请先停止导航再修改点位。',
                'error',
            )
            return
        try:
            if not self.is_rviz_running():
                self.start_plain_rviz()
        except OSError as error:
            self.set_status(f'RViz 启动失败：{error}', 'error')
            return
        self.point_edit_waiting_for_rviz = True
        self.point_edit_dialog.rviz_note.setText(
            '等待 RViz 位姿：请选择工具栏“记录点位”（快捷键 R），'
            '在地图上拖出位置和朝向。'
        )
        self.set_status('等待从 RViz 选择新的点位和朝向。', 'info')

    def save_edited_point(self):
        dialog = self.point_edit_dialog
        index = self.point_edit_index
        if dialog is None or index is None:
            return
        if self.positions_file_signature() != self.point_edit_file_signature:
            self.set_status(
                '点位文件在修改期间发生变化，请关闭窗口后重新选择该点。',
                'error',
            )
            return
        try:
            name, x, y, yaw_degrees = dialog.values()
        except ValueError:
            self.set_status('X、Y、角度必须填写有效数字。', 'error')
            return
        if not all(math.isfinite(value) for value in (x, y, yaw_degrees)):
            self.set_status('点位数值不能是 NaN 或无穷大。', 'error')
            return
        if any(character.isspace() for character in name):
            self.set_status('点位名称不能包含空格。', 'error')
            return
        poses = self.read_record_positions()
        if index >= len(poses):
            self.set_status('选中的点位已经不存在，请重新选择。', 'error')
            return
        if name.startswith('W'):
            if not name[1:].isdigit() or int(name[1:]) <= 0:
                self.set_status('工位名称必须是 W1、W2…格式。', 'error')
                return
            if any(
                row != index and pose.get('label') == name
                for row, pose in enumerate(poses)
            ):
                self.set_status(f'工位名称 {name} 已存在。', 'error')
                return
        pose = poses[index]
        pose['label'] = name
        pose['x'] = x
        pose['y'] = y
        pose['yaw'] = math.radians(yaw_degrees)
        try:
            self.write_record_positions(poses)
        except OSError as error:
            self.set_status(f'点位修改保存失败：{error}', 'error')
            return
        self.points_table.selectRow(index)
        point_name = self.point_display_name(index, pose)
        dialog.accept()
        self.set_status(
            f'已修改 {point_name}：X {x:.3f} m，Y {y:.3f} m，'
            f'朝向 {yaw_degrees:.1f}°；导航前请重新构建拓扑。',
            'success',
        )

    def finish_point_edit(self, _result=0):
        self.point_edit_waiting_for_rviz = False
        self.point_edit_preview_pose = None
        self.point_edit_dialog = None
        self.point_edit_index = None
        self.point_edit_file_signature = None
        self.publish_record_markers()

    def delete_selected_point(self):
        if not self.ensure_points_editable():
            return
        index = self.selected_point_index()
        if index is None:
            return
        poses = self.read_record_positions()
        name = self.point_display_name(index, poses[index])
        answer = QMessageBox.question(
            self, '删除点位', f'确定删除 {name} 吗？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        poses.pop(index)
        try:
            self.write_record_positions(poses)
        except OSError as error:
            self.set_status(f'删除点位失败：{error}', 'error')
            return
        self.set_status(f'已删除 {name}；导航前请重新构建拓扑。', 'success')

    def move_selected_point(self, offset):
        if not self.ensure_points_editable():
            return
        index = self.selected_point_index()
        if index is None:
            return
        poses = self.read_record_positions()
        target = index + offset
        if target < 0 or target >= len(poses):
            self.set_status('点位已经在列表边界。', 'info')
            return
        poses[index], poses[target] = poses[target], poses[index]
        try:
            self.write_record_positions(poses)
        except OSError as error:
            self.set_status(f'调整点位顺序失败：{error}', 'error')
            return
        self.points_table.selectRow(target)
        self.set_status('点位顺序已调整；导航前请重新构建拓扑。', 'success')

    def undo_point_change(self):
        if not self.ensure_points_editable() or not self.point_undo_stack:
            self.set_status('没有可撤销的点位修改。', 'info')
            return
        content = self.point_undo_stack.pop()
        directory = os.path.dirname(ROBOT_POSITIONS_FILE)
        os.makedirs(directory, exist_ok=True)
        try:
            with open(ROBOT_POSITIONS_FILE, 'w', encoding='utf-8') as handle:
                handle.write(content)
        except OSError as error:
            self.set_status(f'撤销失败：{error}', 'error')
            return
        self.refresh_points_table()
        self.publish_record_markers()
        self.set_status('已撤销上一次点位修改。', 'success')

    def restore_latest_points_backup(self):
        if not self.ensure_points_editable():
            return
        directory = os.path.dirname(ROBOT_POSITIONS_FILE)
        backups = sorted(
            glob.glob(os.path.join(directory, 'robot_positions_[0-9]*.txt')),
            key=os.path.getmtime,
            reverse=True,
        )
        if not backups:
            self.set_status('没有找到点位备份文件。', 'warning')
            return
        latest = backups[0]
        answer = QMessageBox.question(
            self, '恢复点位备份',
            f'恢复最近备份 {os.path.basename(latest)}？当前点位可撤销。',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.snapshot_points_for_undo()
        try:
            shutil.copy2(latest, ROBOT_POSITIONS_FILE)
        except OSError as error:
            self.set_status(f'恢复备份失败：{error}', 'error')
            return
        self.refresh_points_table()
        self.publish_record_markers()
        self.set_status('已恢复最近点位备份；导航前请重新构建拓扑。', 'success')

    def import_points_file(self):
        if not self.ensure_points_editable():
            return
        source, _filter = QFileDialog.getOpenFileName(
            self, '导入点位文件', os.path.dirname(ROBOT_POSITIONS_FILE),
            '文本文件 (*.txt);;所有文件 (*)',
        )
        if not source:
            return
        try:
            with open(source, 'r', encoding='utf-8') as handle:
                lines = handle.readlines()
            for line_number, line in enumerate(lines, 1):
                parts = line.split()
                if not parts:
                    continue
                if len(parts) < 6:
                    raise ValueError(f'第 {line_number} 行字段不足')
                for value in parts[:6]:
                    float(value)
                if len(parts) > 7:
                    raise ValueError(f'第 {line_number} 行字段过多')
        except (OSError, ValueError) as error:
            self.set_status(f'导入文件无效：{error}', 'error')
            return
        self.snapshot_points_for_undo()
        try:
            shutil.copy2(source, ROBOT_POSITIONS_FILE)
        except OSError as error:
            self.set_status(f'导入失败：{error}', 'error')
            return
        self.refresh_points_table()
        self.publish_record_markers()
        self.set_status('点位已导入；导航前请重新构建拓扑。', 'success')

    def export_points_file(self):
        if not os.path.isfile(ROBOT_POSITIONS_FILE):
            self.set_status('当前没有可导出的点位文件。', 'warning')
            return
        destination, _filter = QFileDialog.getSaveFileName(
            self, '导出点位文件',
            os.path.join(os.path.dirname(ROBOT_POSITIONS_FILE), 'robot_positions_export.txt'),
            '文本文件 (*.txt);;所有文件 (*)',
        )
        if not destination:
            return
        try:
            shutil.copy2(ROBOT_POSITIONS_FILE, destination)
        except OSError as error:
            self.set_status(f'导出失败：{error}', 'error')
            return
        self.set_status(f'点位已导出到 {destination}。', 'success')

    def next_workstation_label(self):
        highest = 0
        for pose in self.read_record_positions():
            label = pose['label']
            if label.startswith('W') and label[1:].isdigit():
                highest = max(highest, int(label[1:]))
        return f'W{highest + 1}'

    def save_pending_record_pose(self, workstation=False):
        if not self.record_session_active:
            self.set_status('请先进入点位设置模式。', 'warning')
            return False
        if self.pending_record_pose is None:
            self.set_status(
                '尚未收到 RViz 目标位姿；请先在地图上拖出记录箭头。',
                'warning',
            )
            return False

        x, y, yaw = self.pending_record_pose
        label = self.next_workstation_label() if workstation else ''
        line = f'{x:.6f} {y:.6f} 0 0 0 {yaw:.6f}'
        if label:
            line += f' {label}'
        try:
            self.snapshot_points_for_undo()
            with open(ROBOT_POSITIONS_FILE, 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
        except OSError as error:
            self.set_status(f'点位文件写入失败：{error}', 'error')
            return False

        point_number = len(self.read_record_positions())
        point_name = label or f'导航点 P{point_number - 1}'
        self.pending_record_pose = None
        self.refresh_points_table()
        self.record_pose_label.setText(
            f'已保存 {point_name}；请在 RViz 选择下一个目标位姿。'
        )
        self.publish_record_markers()
        self.set_status(
            f'已保存 {point_name}：X {x:.3f} m，Y {y:.3f} m，'
            f'朝向 {math.degrees(yaw):.1f}°。',
            'success',
        )
        return True

    def publish_record_markers(self):
        if self.record_marker_pub is None:
            return

        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        poses = self.read_record_positions()
        if self.pending_record_pose is not None:
            x, y, yaw = self.pending_record_pose
            poses.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'label': '待记录',
                'pending': True,
            })
        if self.point_edit_preview_pose is not None:
            x, y, yaw = self.point_edit_preview_pose
            poses.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'label': '待修改',
                'pending': True,
            })

        stamp = rospy.Time.now()
        for index, pose in enumerate(poses):
            pending = pose.get('pending', False)
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = stamp
            arrow.ns = 'anav_record_pose'
            arrow.id = index * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pose['x']
            arrow.pose.position.y = pose['y']
            arrow.pose.position.z = 0.06
            arrow.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
            arrow.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
            arrow.scale.x = 0.45
            arrow.scale.y = 0.09
            arrow.scale.z = 0.09
            arrow.color.r = 0.10 if pending else 1.0
            arrow.color.g = 0.85 if pending else 0.70
            arrow.color.b = 0.25 if pending else 0.05
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)

            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = stamp
            text_marker.ns = 'anav_record_label'
            text_marker.id = index * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose['x']
            text_marker.pose.position.y = pose['y']
            text_marker.pose.position.z = 0.34
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.22
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            text_marker.text = (
                pose['label'] or f'P{index}'
            )
            marker_array.markers.append(text_marker)

        self.record_marker_pub.publish(marker_array)

    def on_localization_odometry(self, message):
        """LIO 仅在全局定位成功后发布 /Odometry。"""
        self.last_odom_monotonic = time.monotonic()
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        linear_speed = math.hypot(
            message.twist.twist.linear.x, message.twist.twist.linear.y
        )
        self.odometry_requested.emit(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            math.degrees(yaw),
            linear_speed,
            message.twist.twist.angular.z,
        )
        process = self.localizationProcess
        if (
            self.localization_ready
            or process is None
            or process.poll() is not None
        ):
            return
        self.localization_ready = True
        self.localization_ready_requested.emit()

    def mark_localization_ready(self):
        process = self.localizationProcess
        if process is None or process.poll() is not None:
            return
        self.set_chip(self.localization_chip, '定位 已就绪', True)
        self.set_status('定位成功，已接收到有效位姿数据。', 'success')

    def update_odometry_display(self, x, y, yaw_degrees, linear_speed, angular_speed):
        self.odometry_label.setText(
            f'机器人位姿：X {x:.3f} m  ·  Y {y:.3f} m  ·  '
            f'朝向 {yaw_degrees:.1f}°  ·  线速度 {linear_speed:.3f} m/s  ·  '
            f'角速度 {angular_speed:.3f} rad/s'
        )

    def set_chip(self, chip, text, active=False):
        if active:
            chip.setStyleSheet('background: #ECFDF3; color: #067647; border-radius: 12px; padding: 5px 10px;')
        else:
            chip.setStyleSheet('background: #EEF2F7; color: #475467; border-radius: 12px; padding: 5px 10px;')
        chip.setText(text)

    def set_health_chip(self, key, text, state='inactive'):
        chip = self.health_chips.get(key)
        if chip is None:
            return
        colors = {
            'active': ('#ECFDF3', '#067647'),
            'warning': ('#FFFAEB', '#B54708'),
            'error': ('#FEF3F2', '#B42318'),
            'inactive': ('#EEF2F7', '#475467'),
        }
        background, foreground = colors.get(state, colors['inactive'])
        chip.setStyleSheet(
            f'background: {background}; color: {foreground}; border-radius: 12px; '
            'padding: 5px 10px;'
        )
        display_name = self.health_display_names.get(key, key)
        chip.setText(f'{display_name} · {text}')

    def schedule_health_check(self):
        self.sync_process_states()
        self.maybe_refresh_points()
        if self.preview_mode or self.health_check_running:
            return
        self.health_check_running = True

        def collect():
            snapshot = {
                'ros': False,
                'nodes': set(),
                'can_exists': os.path.exists('/sys/class/net/can0'),
                'can_up': False,
            }
            if snapshot['can_exists']:
                try:
                    with open(
                        '/sys/class/net/can0/flags', 'r', encoding='utf-8'
                    ) as handle:
                        flags = int(handle.read().strip(), 16)
                        snapshot['can_up'] = bool(flags & 0x1)
                except (OSError, ValueError):
                    pass
            try:
                result = subprocess.run(
                    ['rosnode', 'list'], capture_output=True, text=True, timeout=2
                )
                snapshot['ros'] = result.returncode == 0
                if snapshot['ros']:
                    snapshot['nodes'] = {
                        line.strip() for line in result.stdout.splitlines()
                        if line.strip()
                    }
            except (OSError, subprocess.TimeoutExpired):
                pass
            self.health_snapshot_requested.emit(snapshot)

        thread = threading.Thread(target=collect, daemon=True)
        thread.start()

    def apply_health_snapshot(self, snapshot):
        self.health_check_running = False
        nodes = snapshot.get('nodes', set())
        ros_ok = bool(snapshot.get('ros'))
        self.set_chip(
            self.ros_chip, 'ROS 已连接' if ros_ok else 'ROS 未连接', ros_ok
        )

        can_exists = snapshot.get('can_exists', False)
        can_up = snapshot.get('can_up', False)
        if can_up:
            self.set_health_chip('can', '已连接', 'active')
        elif can_exists:
            self.set_health_chip('can', '未启用', 'warning')
        else:
            self.set_health_chip('can', '未检测', 'inactive')

        checks = {
            'base': ('/ranger_base_node', '运行中'),
            'arbiter': ('/cmd_vel_arbiter', '运行中'),
            'move_base': ('/mxb_move_base', '运行中'),
            'auto_nav': ('/runnav', '运行中'),
        }
        for key, (node, active_text) in checks.items():
            active = node in nodes
            self.set_health_chip(
                key, active_text if active else '未运行',
                'active' if active else 'inactive',
            )

        tag_nodes = {'/TagCtl_service', '/mm3v_serial_reader'}
        tag_count = len(tag_nodes.intersection(nodes))
        if tag_count == len(tag_nodes):
            self.set_health_chip('tag', '运行中', 'active')
        elif tag_count:
            self.set_health_chip('tag', '部分运行', 'warning')
        else:
            self.set_health_chip('tag', '未运行', 'inactive')

        odom_age = (
            time.monotonic() - self.last_odom_monotonic
            if self.last_odom_monotonic else float('inf')
        )
        if odom_age <= 2.0:
            self.localization_ready = True
            self.set_health_chip('localization', '数据正常', 'active')
            self.set_chip(self.localization_chip, '定位 已就绪', True)
        else:
            had_odometry = self.last_odom_monotonic > 0.0
            self.localization_ready = False
            localization_running = (
                self.localizationProcess is not None
                and self.localizationProcess.poll() is None
            )
            if localization_running and had_odometry:
                self.set_health_chip('localization', '数据中断', 'error')
                self.set_chip(self.localization_chip, '定位 数据中断', False)
                self.odometry_label.setText(
                    f'机器人位姿：定位数据已中断（{odom_age:.1f} 秒未更新）'
                )
            elif localization_running:
                self.set_health_chip('localization', '等待数据', 'warning')
                self.set_chip(self.localization_chip, '定位 等待数据', False)
            else:
                self.set_health_chip('localization', '未运行', 'inactive')
                self.set_chip(self.localization_chip, '定位 未启动', False)
                if had_odometry:
                    self.odometry_label.setText('机器人位姿：定位已停止')

        if '/runnav' in nodes:
            self.set_chip(self.nav_chip, '导航 运行中', True)
        elif '/mxb_move_base' in nodes:
            self.set_chip(self.nav_chip, 'MoveBase 待命', False)
        elif not self.auto_nav_start_pending:
            self.set_chip(self.nav_chip, '导航 未启动', False)

    def sync_process_states(self):
        process_states = (
            ('mappingProcess', 'mapping_start_button', 'mapping_stop_button'),
            ('g2dProcess', 'generate_2d_button', 'stop_2d_button'),
        )
        for process_name, start_name, stop_name in process_states:
            process = getattr(self, process_name, None)
            if process is not None and process.poll() is not None:
                setattr(self, process_name, None)
                start_button = getattr(self, start_name, None)
                stop_button = getattr(self, stop_name, None)
                if start_button:
                    start_button.setEnabled(True)
                if stop_button:
                    stop_button.setEnabled(False)

        if self.localizationProcess and self.localizationProcess.poll() is not None:
            self.localizationProcess = None
            self.localization_ready = False
            self.g2d_button.setEnabled(True)
            self.g2d_exit_button.setEnabled(False)
            self.set_chip(self.localization_chip, '定位 未启动', False)
            self.set_health_chip('localization', '未运行', 'inactive')
        if self.runPntsNavProcess and self.runPntsNavProcess.poll() is not None:
            self.runPntsNavProcess = None
            self.nav_button.setEnabled(True)
            self.nav_runstart.setEnabled(False)
            self.nav_Pause.setEnabled(False)
            self.nav_resume.setEnabled(False)
            self.cancel_task_button.setEnabled(False)
            self.set_nav_shortcuts_visible(False)
        if self.moveBaseProcess and self.moveBaseProcess.poll() is not None:
            self.moveBaseProcess = None
        if self.baseProcess and self.baseProcess.poll() is not None:
            self.baseProcess = None
        if self.joyProcess and self.joyProcess.poll() is not None:
            self.joyProcess = None
        self.tagProcesses = [
            process for process in self.tagProcesses
            if process is not None and process.poll() is None
        ]

    def start_managed_process(self, command):
        """直接启动子进程并创建独立进程组，便于可靠停止 roslaunch 及其子节点。"""
        return subprocess.Popen(command, start_new_session=True)

    def start_terminal_tab_process(self, command, title):
        """在当前 Terminal 窗口新建标签页，同时记录真实命令 PID。"""
        pid_file = os.path.join(
            '/tmp', f'anav_gui_{os.getpid()}_{time.time_ns()}.pid'
        )
        command_text = ' '.join(shlex.quote(str(part)) for part in command)
        shell_command = (
            f"printf '%s\\n' \"$$\" > {shlex.quote(pid_file)}; "
            f"exec {command_text}"
        )
        launcher = subprocess.Popen([
            'gnome-terminal', '--tab', '--active', f'--title={title}', '--wait',
            '--working-directory=' + SCRIPT_DIR,
            '--', 'bash', '-c', shell_command,
        ])

        child_pid = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if os.path.isfile(pid_file):
                try:
                    with open(pid_file, 'r', encoding='utf-8') as handle:
                        child_pid = int(handle.read().strip())
                    break
                except (OSError, ValueError):
                    pass
            if launcher.poll() is not None:
                break
            time.sleep(0.05)
        try:
            os.unlink(pid_file)
        except OSError:
            pass
        if child_pid is None:
            if launcher.poll() is None:
                launcher.terminate()
            raise RuntimeError(f'无法获取“{title}”标签页中的进程 PID')
        return TerminalTabProcess(launcher, child_pid)

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

    def start_bspline_log(self):
        if self.bsplineLogProcess and self.bsplineLogProcess.poll() is None:
            return True
        log_command = (
            'echo "[B-spline] Only showing JGL reference path logs."; '
            'echo "[B-spline] Press Ctrl+C to stop this log view."; '
            'rostopic echo /rosout_agg/msg | '
            'grep --line-buffered "JGL reference path"'
        )
        try:
            self.bsplineLogProcess = self.start_terminal_tab_process(
                ['bash', '-c', log_command], 'B-spline Log'
            )
            return True
        except (OSError, RuntimeError) as error:
            self.bsplineLogProcess = None
            self.set_status(f'B-spline 日志标签页启动失败：{error}', 'warning')
            return False

    def build_topology(self):
        if self.topology_build_running:
            self.set_status('导航拓扑正在构建。', 'info')
            return
        maps_directory = os.path.dirname(ROBOT_POSITIONS_FILE)
        required = (
            ROBOT_POSITIONS_FILE,
            os.path.join(maps_directory, 'map.yaml'),
        )
        missing = [path for path in required if not os.path.isfile(path)]
        if missing:
            self.set_status(
                '无法构建拓扑，缺少：'
                + '、'.join(os.path.basename(path) for path in missing),
                'error',
            )
            return
        if len(self.read_record_positions()) < 2:
            self.set_status('至少需要两个有效点位才能构建拓扑。', 'warning')
            return

        self.topology_build_running = True
        self.build_topology_button.setEnabled(False)
        self.set_status('正在根据点位和二维地图构建导航拓扑…', 'info')

        def run_builder():
            try:
                result = subprocess.run(
                    [
                        sys.executable, TOPOLOGY_BUILDER,
                        '--maps-dir', maps_directory,
                    ],
                    capture_output=True, text=True, timeout=120,
                )
                output = (result.stdout or result.stderr).strip()
                self.topology_finished_requested.emit(
                    result.returncode == 0, output
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                self.topology_finished_requested.emit(False, str(error))

        threading.Thread(target=run_builder, daemon=True).start()

    def on_topology_finished(self, success, output):
        self.topology_build_running = False
        self.build_topology_button.setEnabled(True)
        summary = output.splitlines()[-1] if output else '无输出'
        if success:
            topology_path = os.path.join(
                os.path.dirname(ROBOT_POSITIONS_FILE), 'topology.yaml'
            )
            self.set_status(
                f'导航拓扑构建完成：{topology_path}（{summary}）', 'success'
            )
        else:
            self.set_status(f'导航拓扑构建失败：{summary}', 'error')

    def start_mapping(self):
        if self.mappingProcess and self.mappingProcess.poll() is None:
            self.set_status('地图构建已在运行。', 'info')
            return
        try:
            self.mappingProcess = self.start_terminal_tab_process(
                ['roslaunch', 'robot_r', 's2lam.launch'], '地图构建'
            )
        except (OSError, RuntimeError) as error:
            self.set_status(f'地图构建启动失败：{error}', 'error')
            return
        self.mapping_start_button.setEnabled(False)
        self.mapping_stop_button.setEnabled(True)
        self.set_status('三维地图构建已在新标签页启动。', 'success')

    def stop_mapping(self):
        had_process = self.mappingProcess is not None
        self.stop_managed_process(self.mappingProcess, 'mapping')
        self.mappingProcess = None
        self.mapping_start_button.setEnabled(True)
        self.mapping_stop_button.setEnabled(False)
        if had_process:
            self.set_status('地图构建已结束。', 'info')

    def start_2d_map_generation(self):
        if self.g2dProcess and self.g2dProcess.poll() is None:
            self.set_status('二维地图生成已在运行。', 'info')
            return
        try:
            self.g2dProcess = self.start_terminal_tab_process(
                ['roslaunch', 'robot_r', '4genmap.launch'], '生成二维地图'
            )
        except (OSError, RuntimeError) as error:
            self.set_status(f'二维地图生成启动失败：{error}', 'error')
            return
        self.map_save_generation += 1
        generation = self.map_save_generation
        self.generate_2d_button.setEnabled(False)
        self.stop_2d_button.setEnabled(True)
        self.set_status('二维转换已启动，5 秒后自动保存地图…', 'info')
        QTimer.singleShot(5000, lambda: self.save_2d_map(generation))

    def save_2d_map(self, generation):
        if generation != self.map_save_generation:
            return
        if not self.g2dProcess or self.g2dProcess.poll() is not None:
            self.generate_2d_button.setEnabled(True)
            self.stop_2d_button.setEnabled(False)
            self.set_status('二维转换进程已退出，地图未保存。', 'error')
            return
        try:
            os.makedirs(os.path.dirname(MAP_OUTPUT_PREFIX), exist_ok=True)
            self.mapSaverProcess = self.start_managed_process([
                'rosrun', 'map_server', 'map_saver', '__name:=map_saver',
                '-f', MAP_OUTPUT_PREFIX,
            ])
        except OSError as error:
            self.set_status(f'地图保存启动失败：{error}', 'error')
            return
        self.set_status('正在保存二维地图…', 'info')
        QTimer.singleShot(300, lambda: self.check_map_saver(generation))

    def check_map_saver(self, generation):
        if generation != self.map_save_generation or self.mapSaverProcess is None:
            return
        return_code = self.mapSaverProcess.poll()
        if return_code is None:
            QTimer.singleShot(300, lambda: self.check_map_saver(generation))
            return
        self.mapSaverProcess = None
        if return_code == 0:
            self.set_status(f'二维地图已保存：{MAP_OUTPUT_PREFIX}.yaml / .pgm', 'success')
        else:
            self.set_status(f'地图保存失败，map_saver 退出码：{return_code}', 'error')

    def stop_2d_map_generation(self):
        had_process = self.g2dProcess is not None or self.mapSaverProcess is not None
        self.map_save_generation += 1
        self.stop_managed_process(self.mapSaverProcess, 'map saver')
        self.mapSaverProcess = None
        self.stop_managed_process(self.g2dProcess, '2D map generation')
        self.g2dProcess = None
        self.generate_2d_button.setEnabled(True)
        self.stop_2d_button.setEnabled(False)
        if had_process:
            self.set_status('二维地图生成已停止。', 'info')

    def set_nav_shortcuts_visible(self, visible):
        self.nav_shortcuts_enabled = visible
        for button in self.dynamic_nav_buttons:
            button.setEnabled(visible)
        if hasattr(self, 'navigate_point_button'):
            self.navigate_point_button.setEnabled(
                visible and self.points_table.rowCount() > 0
            )
        if hasattr(self, 'cancel_task_button'):
            self.cancel_task_button.setEnabled(visible)

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
        if self.estop_active:
            self.set_status('紧急停止尚未解除，不能发送精细调整目标。', 'error')
            return
        service_name = '/set_target_y'
        xx = f"{float(posX):.3f}"
        yy = f"{float(posY):.3f}"
        aa = f"{float(posAngle):.3f}"
         # 构建完整的 ROS 命令
        ros_command = f'rosservice call {service_name} "{{target_x: {xx}, target_y: {yy}, target_angle: {aa}}}"'
       # 构建终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = [
            'gnome-terminal', '--tab', '--active', '--title=目标微调',
            '--', 'bash', '-c', ros_command,
        ]
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
        if self.estop_active:
            self.set_status('紧急停止尚未解除，不能下发导航目标。', 'error')
            return
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
                response_lines = [line.strip() for line in result.stdout.splitlines()]
                success_value = next(
                    (line.split(':', 1)[1].strip().lower()
                     for line in response_lines if line.lower().startswith('success:')),
                    '',
                )
                message = next(
                    (line.split(':', 1)[1].strip().strip('"\'')
                     for line in response_lines if line.lower().startswith('message:')),
                    '',
                )
                if success_value in ('true', '1') and 'arrived:' in result.stdout.lower():
                    self.currentID = id
                    self.status_requested.emit(f'已到达：{target_name}', 'success')
                elif success_value in ('true', '1'):
                    self.currentID = id
                    self.status_requested.emit(f'任务已完成：{target_name}', 'success')
                else:
                    detail = message or '服务未返回成功状态'
                    self.status_requested.emit(
                        f'未能到达 {target_name}：{detail}', 'error'
                    )
            except (subprocess.CalledProcessError, OSError) as e:
                detail = getattr(e, 'stderr', '') or str(e)
                print("plan_path_and_go failed:", detail)
                self.status_requested.emit(
                    f'目标调用失败（{target_name}）：{detail.strip()}', 'error'
                )

        thread = threading.Thread(target=call_service)
        thread.daemon = True
        thread.start()

    def gotoworkstation(self, station_id):  # go to workstation W1/W2/W3
        self.gotop(-station_id)

    def start_joy(self):
        if self.joyProcess and self.joyProcess.poll() is None:
            self.set_status('手柄驱动已在运行。', 'info')
            return
        try:
            self.joyProcess = self.start_terminal_tab_process(
                ['roslaunch', 'x2bot_teleop', 'x2bot_joy_PXN.launch'], '手柄'
            )
        except (OSError, RuntimeError) as error:
            self.set_status(f'手柄驱动启动失败：{error}', 'error')
            return
        self.set_status('手柄驱动已在新标签页启动。', 'success')

    def start_base(self):
        if self.baseProcess and self.baseProcess.poll() is None:
            self.set_status('底盘驱动已在运行。', 'info')
            return
        try:
            self.baseProcess = self.start_terminal_tab_process(
                ['roslaunch', 'ranger_bringup', 'ranger_mini_v2.launch'], '底盘'
            )
        except (OSError, RuntimeError) as error:
            self.set_status(f'底盘驱动启动失败：{error}', 'error')
            return
        self.set_status('底盘驱动已在新标签页启动。', 'success')

    def start_tag(self):
        self.tagProcesses = [
            process for process in self.tagProcesses
            if process is not None and process.poll() is None
        ]
        if self.tagProcesses:
            self.set_status('标签读取与 TCP 服务已在运行。', 'info')
            return
        tcp_server = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'tcpserver.py'
        )
        try:
            self.tagProcesses = []
            self.tagProcesses.append(self.start_terminal_tab_process(
                ['roslaunch', 'robot_r', '6tagReadAndCtl_mm3v.launch'],
                'MM3V Tag',
            ))
            self.tagProcesses.append(self.start_terminal_tab_process(
                [sys.executable, tcp_server], 'TCP Server'
            ))
            self.set_status('标签读取与 TCP 服务已启动。', 'success')
        except (OSError, RuntimeError) as error:
            for process in self.tagProcesses:
                self.stop_managed_process(process, 'tag service')
            self.tagProcesses = []
            print(f"Failed to run command: {error}")
            self.set_status(f'标签服务启动失败：{error}', 'error')
                            
    def start_can(self):
        try:
            # 调试阶段使用设备的默认 sudo 密码。
            can_command = "echo '1' | sudo -S ip link set can0 up type can bitrate 500000"

            # 构建完整的终端命令
            terminal_command = [
                'gnome-terminal', '--tab', '--active', '--title=CAN',
                '--', 'bash', '-c', can_command + '; exec bash',
            ]

            # 使用 subprocess.Popen 启动新的终端窗口
            subprocess.Popen(terminal_command)
            self.set_status('CAN 启动命令已执行。', 'success')

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
        if self.rviz_record_mode.isChecked():
            self.save_pending_record_pose(workstation=False)
            return
        self.send_joy_message("add")
        if self.joy_pub is not None:
            self.set_status('已发送“添加导航点”指令。', 'success')

    def add_workstation_fun(self):
        if self.rviz_record_mode.isChecked():
            self.save_pending_record_pose(workstation=True)
            return
        self.send_joy_message("add_workstation")
        if self.joy_pub is not None:
            self.set_status('已发送“添加工位”指令。', 'success')

    def show_pnt_fun(self):
        if self.rviz_record_mode.isChecked():
            self.publish_record_markers()
            self.set_status('已在 RViz 中刷新当前录制点位。', 'success')
            return
        self.send_joy_message("show")
        if self.joy_pub is not None:
            self.set_status('已在 RViz 中请求显示点位。', 'success')

    def publish_safety_stop(self):
        if self.safety_pub is not None:
            self.safety_pub.publish(Twist())

    def publish_stop_burst(self):
        self.publish_safety_stop()
        for delay in (60, 120, 180, 240):
            QTimer.singleShot(delay, self.publish_safety_stop)

    def request_motion_finish(self, source, reason, service_timeout=0.5):
        if not self.ros_available or self.finish_motion_proxy is None:
            return False, 'ROS 或回正服务未初始化'
        try:
            rospy.wait_for_service(
                '/cmd_vel_arbiter/finish_motion', timeout=service_timeout
            )
            response = self.finish_motion_proxy(source=source, reason=reason)
            return bool(response.centered), response.message
        except (rospy.ROSException, rospy.ServiceException) as error:
            return False, str(error)

    def activate_emergency_stop(self):
        if self.estop_active:
            return
        self.estop_active = True
        self.send_joy_message('NavPause')
        self.publish_safety_stop()
        self.estop_timer.start(100)
        self.estop_button.setEnabled(False)
        self.release_estop_button.setEnabled(True)
        self.nav_runstart.setEnabled(False)
        self.nav_resume.setEnabled(False)
        safety_connections = (
            self.safety_pub.get_num_connections()
            if self.safety_pub is not None else 0
        )
        if safety_connections:
            message = '紧急停止已生效：安全通道正在持续发送零速度。'
        else:
            message = (
                '界面已锁定，但未检测到速度仲裁订阅；'
                '请立即使用物理急停并检查底盘节点。'
            )
        self.set_status(message, 'error')

    def release_emergency_stop(self):
        if not self.estop_active:
            return
        answer = QMessageBox.question(
            self, '解除紧急停止',
            '请确认机器人周围安全。解除后不会自动恢复导航，是否继续？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.publish_stop_burst()
        self.estop_timer.stop()
        self.estop_active = False
        self.estop_button.setEnabled(True)
        self.release_estop_button.setEnabled(False)
        auto_nav_running = (
            self.runPntsNavProcess is not None
            and self.runPntsNavProcess.poll() is None
        )
        self.nav_runstart.setEnabled(auto_nav_running)
        self.nav_resume.setEnabled(auto_nav_running)
        self.set_status(
            '紧急停止已解除；机器人保持暂停，请确认后手动继续任务。', 'warning'
        )

    def cancel_current_task(self):
        if self.estop_active:
            self.set_status('当前处于紧急停止状态，请先确认现场安全。', 'warning')
            return
        self.send_joy_message('NavPause')
        centered, center_message = self.request_motion_finish(
            'nav', FinishMotionRequest.TASK_CANCELED
        )
        self.stop_managed_process(self.runPntsNavProcess, 'auto navigation')
        self.runPntsNavProcess = None
        self.nav_button.setEnabled(True)
        self.nav_runstart.setEnabled(False)
        self.nav_Pause.setEnabled(False)
        self.nav_resume.setEnabled(False)
        self.cancel_task_button.setEnabled(False)
        self.set_nav_shortcuts_visible(False)
        if self.is_move_base_ready():
            self.set_chip(self.nav_chip, 'MoveBase 待命', False)
        else:
            self.set_chip(self.nav_chip, '导航 未启动', False)
        message = '当前导航任务已取消，MoveBase 保持待命。'
        if not centered:
            message += f' 轮组回正未确认：{center_message}'
        self.set_status(message, 'warning')

    def Nav_start(self):
        if self.estop_active:
            self.set_status('紧急停止尚未解除，不能开始任务。', 'error')
            return
        self.send_joy_message("NavStart")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已开始。', 'success')
    
    def Nav_Zanting(self):
        self.send_joy_message("NavPause")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已暂停。', 'warning')
    
    def Nav_jixu(self):
        if self.estop_active:
            self.set_status('紧急停止尚未解除，不能继续任务。', 'error')
            return
        self.send_joy_message("Navresume")
        if self.joy_pub is not None:
            self.set_status('自动导航任务已继续。', 'success')

    def start_setlocation(self):
        if self.record_session_active:
            self.set_status('点位设置已在运行。', 'info')
            return
        if not self.ros_available:
            self.set_status('ROS 未连接，无法进入点位设置模式。', 'error')
            return

        rviz_mode = self.rviz_record_mode.isChecked()
        if rviz_mode and self.is_move_base_ready():
            self.set_status(
                'MoveBase 正在运行。为避免记录箭头触发车辆动作，请先停止导航。',
                'error',
            )
            return

        if not rviz_mode:
            try:
                self.setpointProcess = self.start_terminal_tab_process(
                    ['roslaunch', 'robot_r', '3settinglocation.launch'], '点位设置'
                )
            except (OSError, RuntimeError) as error:
                self.set_status(f'点位设置启动失败：{error}', 'error')
                return
        else:
            try:
                if not self.is_rviz_running():
                    self.start_plain_rviz()
            except OSError as error:
                self.set_status(f'RViz 启动失败：{error}', 'error')
                return

        try:
            self.backup_and_reset_record_file()
        except OSError as error:
            self.stop_managed_process(self.setpointProcess, 'point setting')
            self.setpointProcess = None
            self.set_status(f'无法准备点位文件：{error}', 'error')
            return
        self.refresh_points_table()

        self.record_session_active = True
        self.pending_record_pose = None
        self.real_record_mode.setEnabled(False)
        self.rviz_record_mode.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.location_pnt.show()
        self.add_pnt.show()
        self.add_workstation.show()
        self.publish_record_markers()
        if rviz_mode:
            self.record_pose_label.setText(
                '等待 RViz 目标位姿：请选择工具栏中的“记录点位”'
                '（快捷键 R）。'
            )
            message = '已进入 RViz 采点模式；请在地图上拖出记录箭头。'
        else:
            message = '已进入实车采点模式；请移动机器人后添加点位。'
        if self.record_file_backup:
            message += f' 原点位已备份到 {self.record_file_backup}。'
        self.set_status(message, 'success')

    def stop_setlocation(self):
        had_session = self.record_session_active
        self.stop_managed_process(self.setpointProcess, 'point setting')
        self.setpointProcess = None
        self.record_session_active = False
        self.pending_record_pose = None
        self.real_record_mode.setEnabled(True)
        self.rviz_record_mode.setEnabled(True)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.add_pnt.hide()
        self.add_workstation.hide()
        self.location_pnt.hide()
        self.publish_record_markers()
        self.refresh_points_table()
        if had_session:
            self.set_status(
                f'已结束点位设置，点位保存在 {ROBOT_POSITIONS_FILE}。',
                'info',
            )

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
            if self.localization_ready:
                self.set_status('定位已成功，位姿数据正在持续更新。', 'success')
            else:
                self.set_status('定位正在运行，等待有效位姿数据。', 'info')
        else:
            self.localization_ready = False
            self.last_odom_monotonic = 0.0
            command = ['roslaunch', 'robot_r', '3startlocation.launch', 'rviz_enable:=false']
            self.localizationProcess = subprocess.Popen(command, preexec_fn=os.setpgrp)
            self.g2d_button.setEnabled(False)
            self.g2d_exit_button.setEnabled(True)
            self.set_chip(self.localization_chip, '定位 运行中', True)
            self.set_status('定位已启动，正在等待地图和位姿数据。', 'info')
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
            self.start_bspline_log()
            self.set_status('MoveBase 已在运行。', 'info')
            return True
        if self.moveBaseProcess and self.moveBaseProcess.poll() is None:
            self.start_bspline_log()
            self.set_status('MoveBase 正在启动…', 'info')
            return True
        try:
            self.moveBaseProcess = self.start_terminal_tab_process(
                ['roslaunch', 'robot_r', '5nav.launch'], 'MoveBase'
            )
        except (OSError, RuntimeError) as error:
            self.set_status(f'MoveBase 启动失败：{error}', 'error')
            return False
        self.start_bspline_log()
        self.quitnav_button.setEnabled(True)
        self.set_chip(self.nav_chip, '导航 启动中', False)
        self.set_status('MoveBase 正在启动…', 'info')
        return True

    # load points and auto run
    def startNAV(self):
        if self.estop_active:
            self.set_status('紧急停止尚未解除，不能启动自动导航。', 'error')
            return
        if self.runPntsNavProcess and self.runPntsNavProcess.poll() is None:
            self.set_status('自动导航已在运行。', 'info')
            return
        if self.auto_nav_start_pending:
            self.set_status('正在等待 MoveBase 就绪…', 'info')
            return
        if self.is_move_base_ready():
            self.start_bspline_log()
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
            self.stop_managed_process(self.bsplineLogProcess, 'B-spline log')
            self.bsplineLogProcess = None
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
            self.runPntsNavProcess = self.start_terminal_tab_process(
                ['roslaunch', 'robot_r', '3navlocations.launch'], 'AutoNav'
            )
        except (OSError, RuntimeError) as error:
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
            or self.moveBaseProcess is not None or self.bsplineLogProcess is not None
        )
        self.auto_nav_start_pending = False
        self.set_status('正在停止自动导航和 MoveBase…', 'info')
        if stopped:
            self.request_motion_finish(
                'nav', FinishMotionRequest.TASK_CANCELED
            )
        self.stop_managed_process(self.runPntsNavProcess, 'auto navigation')
        self.runPntsNavProcess = None
        self.stop_managed_process(self.bsplineLogProcess, 'B-spline log')
        self.bsplineLogProcess = None
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


    def quit_navigation(self, stop_external=True):
        if self.localizationProcess and self.localizationProcess.poll() is None:
            pgid = os.getpgid(self.localizationProcess.pid)
            subprocess.call(['pkill', '-15', '-g', str(pgid)])
            self.localizationProcess = None
        elif stop_external:
            process_name = '3startlocation'
            try:
                process_id = subprocess.check_output(['pgrep', '-f', process_name]).decode().strip()
            except subprocess.CalledProcessError:
                process_id = ''
            for pid in process_id.split():
                subprocess.call(['kill', '-15', pid])
        self.localization_ready = False
        self.last_odom_monotonic = 0.0
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
        self.close()

    def closeEvent(self, event):
        if self.shutdown_in_progress:
            event.accept()
            return
        self.shutdown_in_progress = True
        self.health_timer.stop()
        self.estop_active = True
        self.estop_timer.start(100)
        self.publish_stop_burst()
        centered, center_message = self.request_motion_finish(
            'safety', FinishMotionRequest.SOFTWARE_ESTOP,
            service_timeout=1.0,
        )
        if not centered:
            print(f'GUI shutdown centering was not confirmed: {center_message}')
        self.exitNAV()
        self.quit_navigation(stop_external=False)
        self.stop_rviz()
        self.stop_setlocation()
        self.stop_2d_map_generation()
        self.stop_mapping()
        self.stop_managed_process(self.joyProcess, 'joystick')
        self.joyProcess = None
        self.stop_managed_process(self.baseProcess, 'robot base')
        self.baseProcess = None
        for process in self.tagProcesses:
            self.stop_managed_process(process, 'tag service')
        self.tagProcesses = []
        self.publish_stop_burst()
        self.estop_timer.stop()
        event.accept()
        


if __name__ == '__main__':
    app = QApplication(sys.argv)



    mainWin = MyWindow()
    mainWin.show()

    sys.exit(app.exec_())
