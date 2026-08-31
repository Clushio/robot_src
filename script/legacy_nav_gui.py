#!/usr/bin/env python3
"""ROS2 implementation shared by the historical B/C/D navigation GUIs."""

import os
import signal
import sys

import rclpy
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget
from sensor_msgs.msg import Joy

from ros2_runtime import Ros2Runtime


class LegacyNavWindow(QWidget):
    def __init__(self, variant):
        super().__init__()
        self.variant = variant
        self.setWindowTitle(f'AutoNAV legacy {variant} (ROS2)')
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.resize(340, 570)
        self.runtime = Ros2Runtime(f'anav_legacy_{variant.lower()}')
        self.joy_pub = self.runtime.node.create_publisher(Joy, '/joy', 10)
        self.processes = {}
        self._button('设置导航点', 10, 10, self.start_setlocation)
        self._button('结束设置', 160, 10, lambda: self.stop('setlocation'))
        self._button('添加点', 10, 100, lambda: self.send_joy('add'))
        self._button('显示点', 160, 100, lambda: self.send_joy('show'))
        self._button('开始定位', 10, 200, self.start_localization)
        self._button('结束定位', 160, 200, lambda: self.stop('localization'))
        self._button('启动 AutoNAV', 10, 300, self.start_autonav)
        self._button('启动 Nav2', 160, 300, self.start_nav2)
        self._button('停止导航', 235, 300, self.stop_navigation, width=95)
        self._button('开始', 10, 400, lambda: self.send_joy('start'), width=100)
        self._button('暂停', 120, 400, lambda: self.send_joy('pause'), width=100)
        self._button('继续', 230, 400, lambda: self.send_joy('resume'), width=100)
        self._button('退出', 120, 500, self.close, width=100, height=50)

    def _button(self, text, x, y, callback, width=150, height=70):
        button = QPushButton(text, self)
        button.setGeometry(x, y, width, height)
        button.clicked.connect(callback)
        return button

    def start(self, key, package, launch_file):
        process = self.processes.get(key)
        if process is not None and process.poll() is None:
            return
        import subprocess
        self.processes[key] = subprocess.Popen(
            ['ros2', 'launch', package, launch_file],
            start_new_session=True,
        )

    def stop(self, key):
        process = self.processes.pop(key, None)
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=4.0)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def start_setlocation(self):
        self.start('setlocation', 'robot_r', '3settinglocation.launch.py')

    def start_localization(self):
        self.start('localization', 'robot_r', '3startlocation.launch.py')

    def start_nav2(self):
        self.start('nav2', 'robot_r', '5nav.launch.py')

    def start_autonav(self):
        self.start('autonav', 'robot_r', '3navlocations.launch.py')

    def stop_navigation(self):
        self.stop('autonav')
        self.stop('nav2')

    def send_joy(self, command):
        buttons = {
            'add': [0, 0, 1, 0, 0, 0],
            'show': [0, 0, 0, 0, 0, 1],
            'start': [0, 0, 0, 1, 0, 0],
            'pause': [1, 0, 0, 0, 0, 0],
            'resume': [0, 0, 1, 0, 0, 0],
        }
        message = Joy()
        message.axes = [0.0, 0.0, 0.0]
        message.buttons = buttons[command]
        self.joy_pub.publish(message)

    def closeEvent(self, event):
        for key in list(self.processes):
            self.stop(key)
        self.runtime.shutdown()
        event.accept()


def run_legacy_nav_gui(variant):
    app = QApplication(sys.argv)
    window = LegacyNavWindow(variant)
    window.show()
    result = app.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    return result
