#!/usr/bin/env python3
"""Historical compact mapping GUI, ported to ROS2 launch and map saver."""

import os
import signal
import subprocess
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget


class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('地图构建（ROS2）')
        self.resize(180, 740)
        self.processes = {}
        self._button('构建地图', 50, self.start_mapping)
        self._button('结束构建', 200, lambda: self.stop('mapping'))
        self._button('生成2D地图', 350, self.start_2d)
        self._button('结束2D地图', 500, lambda: self.stop('2d'))
        self._button('退出', 650, self.close)

    def _button(self, text, y, callback):
        button = QPushButton(text, self)
        button.setGeometry(10, y, 150, 70)
        button.clicked.connect(callback)

    def start(self, key, command):
        process = self.processes.get(key)
        if process is not None and process.poll() is None:
            return
        self.processes[key] = subprocess.Popen(command, start_new_session=True)

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

    def start_mapping(self):
        self.start(
            'mapping', ['ros2', 'launch', 'robot_r', 's2lam.launch.py']
        )

    def start_2d(self):
        self.start(
            '2d', ['ros2', 'launch', 'robot_r', '4genmap.launch.py']
        )
        QTimer.singleShot(5000, self.save_2d_map)

    def save_2d_map(self):
        converter = self.processes.get('2d')
        if converter is None or converter.poll() is not None:
            return
        self.start(
            'map_saver', [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', os.path.expanduser('~/maps/map'),
            ]
        )

    def closeEvent(self, event):
        for key in list(self.processes):
            self.stop(key)
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
