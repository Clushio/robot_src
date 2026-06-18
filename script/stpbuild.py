
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
import subprocess
import time

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        # 创建启动按钮
        self.start_button = QPushButton('构建地图', self)
        self.start_button.move(10, 50)
        self.start_button.clicked.connect(self.start_roslaunch)
        self.start_button.resize(150,70)

        # 创建停止按钮
        self.stop_button = QPushButton('结束构建', self)
        self.stop_button.move(10, 200)
        self.stop_button.clicked.connect(self.stop_roslaunch)
        self.stop_button.resize(150,70)

        # 创建2d按钮
        self.g2d_button = QPushButton('生成2D地图', self)
        self.g2d_button.move(10, 350)
        self.g2d_button.clicked.connect(self.f_g2d_button)
        self.g2d_button.resize(150,70)

        # save2d按钮
        self.g2d_exit_button = QPushButton('exit_2D地图', self)
        self.g2d_exit_button.move(10, 500)
        self.g2d_exit_button.clicked.connect(self.f_g2d_exit_button)
        self.g2d_exit_button.resize(150,70)



         # nav按钮
        #self.nav_button = QPushButton('启动导航模式', self)
        #self.nav_button.move(200, 50)
        #self.nav_button.clicked.connect(self.f_nav_button)

        # exit nav按钮
        #self.nav_exit_button = QPushButton('exit导航模式', self)
        #self.nav_exit_button.move(200, 50)
        #self.nav_exit_button.clicked.connect(self.f_ext_nav_button)

         # 创建退出按钮
        self.quit_button = QPushButton('退出', self)
        self.quit_button.move(10, 650)
        self.quit_button.clicked.connect(self.close_app)  # 直接连接到QWidget的close()方法

        #self.initbash()

    #def initbash(self):
        #command = ["bash", "--login"]
        #subprocess.Popen(command)


    def start_roslaunch(self):
        # 替换下面的命令为你实际要启动的roslaunch命令
        command = ['roslaunch', 'robot_r', 's2lam.launch']
        subprocess.Popen(command)
        self.start_button.isEnabled = False

    def f_g2d_button(self):
        # 替换下面的命令为你实际要启动的roslaunch命令
        command = ['roslaunch', 'robot_r', '4genmap.launch']
        subprocess.Popen(command)
        self.start_button.isEnabled = False
        time.sleep(5)
        command = ['rosrun', 'map_server', 'map_saver','-f', '/home/nav/maps/map']
        subprocess.Popen(command)
        self.g2d_button.isEnabled = False


    def f_g2d_exit_button(self):
        # 结束roslaunch通常需要找到其对应的进程ID并kill它，这里假设你已经知道了进程名为'your_launch_file'
        # 注意，直接kill可能会导致不优雅的退出，如果可能，最好通过ros本身提供的机制来关闭节点
        # 这里只是一个示例，实际应用中可能需要更复杂的逻辑来确保安全地关闭节点
        process_name = '4genmap'  # 替换成你的roslaunch启动的具体名称或者相关的节点名
        # 使用ps aux | grep查找进程，并假设grep后的第二列是进程ID
        # 实际情况请根据实际情况调整，确保正确找到对应进程ID
        process_id = subprocess.check_output(["pgrep", "-f", process_name]).decode().strip()
        
        if process_id:
            # 将字符串转换成整数列表并杀死进程
            for pid in process_id.split():
                subprocess.call(['kill', '-15', pid])  # 使用SIGTERM信号(-15)尝试正常关闭进程
                # 如果需要强制结束，可替换为 'kill -9 {}'.format(pid)，但非必要时尽量避免
        self.g2d_button.isEnabled = True

    def stop_roslaunch(self):
        # 结束roslaunch通常需要找到其对应的进程ID并kill它，这里假设你已经知道了进程名为'your_launch_file'
        # 注意，直接kill可能会导致不优雅的退出，如果可能，最好通过ros本身提供的机制来关闭节点
        # 这里只是一个示例，实际应用中可能需要更复杂的逻辑来确保安全地关闭节点
        process_name = 's2lam'  # 替换成你的roslaunch启动的具体名称或者相关的节点名
        # 使用ps aux | grep查找进程，并假设grep后的第二列是进程ID
        # 实际情况请根据实际情况调整，确保正确找到对应进程ID
        process_id = subprocess.check_output(["pgrep", "-f", process_name]).decode().strip()
        
        if process_id:
            # 将字符串转换成整数列表并杀死进程
            for pid in process_id.split():
                subprocess.call(['kill', '-15', pid])  # 使用SIGTERM信号(-15)尝试正常关闭进程
                # 如果需要强制结束，可替换为 'kill -9 {}'.format(pid)，但非必要时尽量避免
        self.start_button.isEnabled  = True

    def close_app(self):
        self.close()  # 关闭窗口，这将触发closeEvent事件，从而结束整个应用程序

    def closeEvent(self, event):
        # 当窗口关闭时，确保清理资源等
        event.accept()  # 接受关闭事件，默认行为是关闭窗口并退出程序
        

if __name__ == '__main__':
    app = QApplication(sys.argv)

    mainWin = MyWindow()
    mainWin.show()

    sys.exit(app.exec_())