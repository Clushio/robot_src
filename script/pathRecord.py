import sys
import os
import time
import subprocess
import rospy
import rosbag
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QFileDialog, QLabel
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import sys
import os
import time
import subprocess
import rospy
import rosbag
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QFileDialog, QLabel, QTextEdit
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import random

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


def find_nearest_pose(target_pose, poses):
    min_distance = float('inf')
    nearest_pose = None
    for pose in poses:
        pos1 = target_pose.pose.position
        pos2 = pose.pose.position
        distance = ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5  # 忽略 z 坐标
        if distance < min_distance:
            min_distance = distance
            nearest_pose = pose
    return nearest_pose

def calculate_average_distance(path1, path2, num_samples=30):
    if not path1.poses or not path2.poses:
        raise ValueError("Paths must contain at least one pose to calculate average distance.")

    sample_indices = random.sample(range(len(path1.poses)), min(num_samples, len(path1.poses)))
    total_distance = 0.0

    for index in sample_indices:
        pose1 = path1.poses[index]
        nearest_pose2 = find_nearest_pose(pose1, path2.poses)
        pos1 = pose1.pose.position
        pos2 = nearest_pose2.pose.position
        distance = ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5  # 忽略 z 坐标
        total_distance += distance

    return total_distance / len(sample_indices)

class RosbagThread(QThread):
    finished = pyqtSignal()

    def __init__(self, file_path, topic, msg_type, parent=None):
        super(RosbagThread, self).__init__(parent)
        self.file_path = file_path
        self.topic = topic
        self.msg_type = msg_type
        self.running = False

    def run(self):
        self.running = True
        with rosbag.Bag(self.file_path, 'w') as bag:
            while not rospy.is_shutdown() and self.running:
                try:
                    msg = rospy.wait_for_message(self.topic, self.msg_type, timeout=1.0)
                    bag.write(self.topic, msg)
                except rospy.ROSException:
                    break
        self.finished.emit()

    def stop(self):
        self.running = False

class RvizThread(QThread):
    started = pyqtSignal()
    stopped = pyqtSignal()

    def __init__(self, config_file, parent=None):
        super(RvizThread, self).__init__(parent)
        self.config_file = config_file
        self.process = None

    def run(self):
        self.process = subprocess.Popen(['rviz', '-d', self.config_file])
        self.started.emit()
        self.process.wait()
        self.stopped.emit()

    def stop(self):
        if self.process:
            self.process.terminate()
            self.process.wait()

class TrajectoryRecorder(QWidget):
    def __init__(self):
        super(TrajectoryRecorder, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Trajectory Recorder with RViz')

        layout = QVBoxLayout()

        button_layout = QHBoxLayout()
        self.start_button = QPushButton('Start Recording', self)
        self.stop_button = QPushButton('Stop Recording', self)
        self.open_button = QPushButton('Open and Replay', self)
        self.rviz_button = QPushButton('Start RViz', self)

        # 设置按钮字体大小
        font = QFont()
        font.setPointSize(14)
        self.start_button.setFont(font)
        self.stop_button.setFont(font)
        self.open_button.setFont(font)
        self.rviz_button.setFont(font)

        # 设置按钮最小尺寸
        self.start_button.setMinimumSize(200, 50)
        self.stop_button.setMinimumSize(200, 50)
        self.open_button.setMinimumSize(200, 50)
        self.rviz_button.setMinimumSize(200, 50)

        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.open_button)
        button_layout.addWidget(self.rviz_button)

        layout.addLayout(button_layout)

        self.rviz_label = QLabel("RViz will be displayed here", self)
        self.rviz_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.rviz_label)

        self.log_text = QTextEdit(self)
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)

        self.setLayout(layout)

        self.start_button.clicked.connect(self.start_recording)
        self.stop_button.clicked.connect(self.stop_recording)
        self.open_button.clicked.connect(self.open_and_replay)
        self.rviz_button.clicked.connect(self.start_rviz)

        self.recording_thread = None
        self.bag_file_path = None
        self.rviz_thread = None
        self.config_file = "/home/jgl20/testrecord/1.rviz"  # 配置文件路径

    def start_recording(self):
        if self.recording_thread is None or not self.recording_thread.isRunning():
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            self.bag_file_path = f"/home/jgl20/testrecord/{timestamp}.bag"
            self.recording_thread = RosbagThread(self.bag_file_path, '/mavros/vision_pose/pose', PoseStamped)
            self.recording_thread.start()

    def stop_recording(self):
        if self.recording_thread is not None and self.recording_thread.isRunning():
            self.recording_thread.stop()
            self.recording_thread.wait()
            self.recording_thread = None

    def open_and_replay(self):
        options = QFileDialog.Options()
        file_names, _ = QFileDialog.getOpenFileNames(self, "Open Bag Files", "/home/jgl20/testrecord/", "Bag Files (*.bag);;All Files (*)", options=options)
        if file_names:
            topics = ['/trajectory1', '/trajectory2', '/trajectory3']
            paths = []

            for i, file_name in enumerate(file_names[:3]):
                bag = rosbag.Bag(file_name)
                path_msg = Path()
                path_msg.header.frame_id = "map"  # 设置参考坐标系
                for topic, msg, t in bag.read_messages(topics=['/mavros/vision_pose/pose']):
                    pose_stamped = PoseStamped()
                    pose_stamped.header = msg.header
                    pose_stamped.pose = msg.pose
                    path_msg.poses.append(pose_stamped)
                bag.close()
                paths.append(path_msg)
                path_pubs[i].publish(path_msg)

            # 计算并输出平均距离
            if len(paths) > 1:
                for i in range(len(paths)):
                    for j in range(i + 1, len(paths)):
                        avg_distance = calculate_average_distance(paths[i], paths[j])
                        self.log_text.append(f"Average distance between trajectory {i+1} and trajectory {j+1}: {avg_distance:.2f} meters")

    def start_rviz(self):
        if self.rviz_thread is None or not self.rviz_thread.isRunning():
            self.rviz_thread = RvizThread(self.config_file)
            self.rviz_thread.started.connect(self.update_rviz_label)
            self.rviz_thread.stopped.connect(self.update_rviz_label)
            self.rviz_thread.start()

    def update_rviz_label(self):
        if self.rviz_thread and self.rviz_thread.isRunning():
            self.rviz_label.setText("RViz is running")
        else:
            self.rviz_label.setText("RViz has stopped")

if __name__ == '__main__':
    check_and_start_roscore()
    rospy.init_node('trajectory_recorder', anonymous=True)
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    path_pubs = [
        rospy.Publisher('/trajectory1', Path, queue_size=10),
        rospy.Publisher('/trajectory2', Path, queue_size=10),
        rospy.Publisher('/trajectory3', Path, queue_size=10)
    ]

    app = QApplication(sys.argv)
    ex = TrajectoryRecorder()
    ex.show()
    sys.exit(app.exec_())