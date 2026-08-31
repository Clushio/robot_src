#!/usr/bin/env python3
"""Record and compare up to three PoseStamped trajectories with rosbag2."""

import os
import random
import signal
import subprocess
import sys
import time

import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.serialization import deserialize_message

from ros2_runtime import Ros2Runtime


RECORD_TOPIC = '/mavros/vision_pose/pose'
RECORD_DIRECTORY = os.path.expanduser('~/testrecord')


def find_nearest_pose(target_pose, poses):
    return min(
        poses,
        key=lambda pose: (
            (target_pose.pose.position.x - pose.pose.position.x) ** 2
            + (target_pose.pose.position.y - pose.pose.position.y) ** 2
        ),
    )


def calculate_average_distance(path1, path2, num_samples=30):
    if not path1.poses or not path2.poses:
        raise ValueError('Paths must contain at least one pose')
    indices = random.sample(
        range(len(path1.poses)), min(num_samples, len(path1.poses))
    )
    total = 0.0
    for index in indices:
        pose1 = path1.poses[index]
        pose2 = find_nearest_pose(pose1, path2.poses)
        total += (
            (pose1.pose.position.x - pose2.pose.position.x) ** 2
            + (pose1.pose.position.y - pose2.pose.position.y) ** 2
        ) ** 0.5
    return total / len(indices)


def bag_uri(path):
    return os.path.dirname(path) if os.path.isfile(path) else path


def read_pose_path(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    result = Path()
    result.header.frame_id = 'map'
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic == RECORD_TOPIC:
            result.poses.append(deserialize_message(data, PoseStamped))
    return result


class RosbagThread(QThread):
    finished = pyqtSignal()

    def __init__(self, output_directory, parent=None):
        super().__init__(parent)
        self.output_directory = output_directory
        self.process = None

    def run(self):
        self.process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', self.output_directory, RECORD_TOPIC],
            start_new_session=True,
        )
        self.process.wait()
        self.finished.emit()

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            self.process.wait(timeout=5.0)


class RvizThread(QThread):
    started = pyqtSignal()
    stopped = pyqtSignal()

    def __init__(self, config_file, parent=None):
        super().__init__(parent)
        self.config_file = config_file
        self.process = None

    def run(self):
        self.process = subprocess.Popen(['rviz2', '-d', self.config_file])
        self.started.emit()
        self.process.wait()
        self.stopped.emit()

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=3.0)


class TrajectoryRecorder(QWidget):
    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
        self.path_pubs = [
            runtime.node.create_publisher(Path, f'/trajectory{i}', 10)
            for i in range(1, 4)
        ]
        self.recording_thread = None
        self.rviz_thread = None
        self.config_file = os.path.join(
            get_package_share_directory('robot_r'),
            'rviz', 'nav_rviz_no_panel.rviz',
        )
        self.setWindowTitle('Trajectory Recorder with RViz2')
        layout = QVBoxLayout(self)
        buttons = QHBoxLayout()
        for text, callback in (
            ('Start Recording', self.start_recording),
            ('Stop Recording', self.stop_recording),
            ('Open and Compare', self.open_and_replay),
            ('Start RViz2', self.start_rviz),
        ):
            button = QPushButton(text, self)
            font = QFont()
            font.setPointSize(14)
            button.setFont(font)
            button.setMinimumSize(180, 50)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.rviz_label = QLabel('RViz2 is not running', self)
        self.rviz_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.rviz_label)
        self.log_text = QTextEdit(self)
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)

    def start_recording(self):
        if self.recording_thread is not None and self.recording_thread.isRunning():
            return
        os.makedirs(RECORD_DIRECTORY, exist_ok=True)
        output = os.path.join(RECORD_DIRECTORY, time.strftime('%Y%m%d-%H%M%S'))
        self.recording_thread = RosbagThread(output, self)
        self.recording_thread.start()
        self.log_text.append(f'Recording {RECORD_TOPIC} to {output}')

    def stop_recording(self):
        if self.recording_thread is not None and self.recording_thread.isRunning():
            self.recording_thread.stop()
            self.recording_thread.wait()
        self.recording_thread = None

    def open_and_replay(self):
        files, _ = QFileDialog.getOpenFileNames(
            self, 'Open ROS2 bag database files', RECORD_DIRECTORY,
            'ROS2 Bag (*.db3);;All Files (*)',
        )
        paths = []
        for index, filename in enumerate(files[:3]):
            try:
                path = read_pose_path(filename)
            except Exception as error:
                self.log_text.append(f'Cannot read {filename}: {error}')
                continue
            paths.append(path)
            self.path_pubs[index].publish(path)
            self.log_text.append(
                f'Published trajectory {index + 1}: {len(path.poses)} poses'
            )
        for first in range(len(paths)):
            for second in range(first + 1, len(paths)):
                distance = calculate_average_distance(
                    paths[first], paths[second]
                )
                self.log_text.append(
                    f'Average distance {first + 1}↔{second + 1}: '
                    f'{distance:.2f} m'
                )

    def start_rviz(self):
        if self.rviz_thread is None or not self.rviz_thread.isRunning():
            self.rviz_thread = RvizThread(self.config_file, self)
            self.rviz_thread.started.connect(self.update_rviz_label)
            self.rviz_thread.stopped.connect(self.update_rviz_label)
            self.rviz_thread.start()

    def update_rviz_label(self):
        running = self.rviz_thread and self.rviz_thread.isRunning()
        self.rviz_label.setText(
            'RViz2 is running' if running else 'RViz2 has stopped'
        )

    def closeEvent(self, event):
        self.stop_recording()
        if self.rviz_thread is not None:
            self.rviz_thread.stop()
        event.accept()


if __name__ == '__main__':
    runtime = Ros2Runtime('trajectory_recorder')
    app = QApplication(sys.argv)
    window = TrajectoryRecorder(runtime)
    window.show()
    exit_code = app.exec_()
    runtime.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)
