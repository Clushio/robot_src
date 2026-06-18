import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
import subprocess
import time

from PyQt5.QtCore import Qt

import os
import signal

import rospy
from sensor_msgs.msg import Joy

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

        check_and_start_roscore()

        # 初始化ROS节点
        rospy.init_node('upmachine_publisher', anonymous=True)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)

        self.initUI()
        self.setpointProcess = None
        self.runPntsNavProcess = None
        self.add_pnt.hide()
        self.location_pnt.hide()
        self.nav_runstart.hide()
        self.nav_Pause.hide()
        self.nav_resume.hide()



    def initUI(self):
        # 创建启动按钮
        self.start_button = QPushButton('设置导航点', self)
        self.start_button.move(10, 10)
        self.start_button.clicked.connect(self.start_setlocation)
        self.start_button.resize(150,70)

        # 创建停止按钮
        self.stop_button = QPushButton('结束设置', self)
        self.stop_button.move(160, 10)
        self.stop_button.clicked.connect(self.stop_setlocation)
        self.stop_button.resize(150,70)

        # 创建tianjia按钮
        self.add_pnt = QPushButton('添加点', self)
        self.add_pnt.move(10, 100)
        self.add_pnt.clicked.connect(self.add_pnt_fun)
        self.add_pnt.resize(150,70)

        self.location_pnt = QPushButton('xianshi', self)
        self.location_pnt.move(160, 100)
        self.location_pnt.clicked.connect(self.show_pnt_fun)
        self.location_pnt.resize(150,70)

        # 创建2d按钮
        self.g2d_button = QPushButton('开始导航', self)
        self.g2d_button.move(10, 200)
        self.g2d_button.clicked.connect(self.start_navgation)
        self.g2d_button.resize(150,70)

        # save2d按钮
        self.g2d_exit_button = QPushButton('exit_导航', self)
        self.g2d_exit_button.move(160, 200)
        self.g2d_exit_button.clicked.connect(self.quit_navigation)
        self.g2d_exit_button.resize(150,70)

         # nav按钮
        self.nav_button = QPushButton('启动NAV', self)
        self.nav_button.move(10, 300)
        self.nav_button.clicked.connect(self.startNAV)
        self.nav_button.resize(150,70)

                # nav按钮
        self.nav5_button = QPushButton('5nav', self)
        self.nav5_button.move(160, 300)
        self.nav5_button.clicked.connect(self.nav5)
        self.nav5_button.resize(75,70)

        # nav按钮
        self.quitnav_button = QPushButton('Exit_nav', self)
        self.quitnav_button.move(235, 300)
        self.quitnav_button.clicked.connect(self.exitNAV)
        self.quitnav_button.resize(75,70)

        self.nav_runstart = QPushButton('开始', self)
        self.nav_runstart.move(10, 400)
        self.nav_runstart.clicked.connect(self.Nav_kaishi)
        self.nav_runstart.resize(100,70)

        self.nav_Pause = QPushButton('暂停', self)
        self.nav_Pause.move(120, 400)
        self.nav_Pause.clicked.connect(self.Nav_Zanting)
        self.nav_Pause.resize(100,70)

        self.nav_resume = QPushButton('继续', self)
        self.nav_resume.move(230, 400)
        self.nav_resume.clicked.connect(self.Nav_jixu)
        self.nav_resume.resize(100,70)

        # exit nav按钮
        #self.nav_exit_button = QPushButton('exit导航模式', self)
        #self.nav_exit_button.move(200, 50)
        #self.nav_exit_button.clicked.connect(self.f_ext_nav_button)

         # 创建退出按钮
        self.quit_button = QPushButton('退出', self)
        self.quit_button.move(150, 500)
        self.quit_button.clicked.connect(self.close_app)  # 直接连接到QWidget的close()方法
        self.quit_button.resize(100,50)
        #self.initbash()

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

        if(cmd=="Navkaishi"):
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0]  # 根据需要设置轴值
            joy_msg.buttons = [0, 0, 0, 1,0,0]       # 假设第一个按钮被按下
            self.joy_pub.publish(joy_msg)
            print("Joy Navkaishi message sent")

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

    def show_pnt_fun(self):
        self.send_joy_message("show")

    def Nav_kaishi(self):
        self.send_joy_message("Navkaishi")
    
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
            self.location_pnt.hide()

    def start_navgation(self):
        # 替换下面的命令为你实际要启动的roslaunch命令
        command = ['roslaunch', 'robot_r', '3startlocation_withNav_D.launch']
        subprocess.Popen(command)
        self.start_button.isEnabled = False

    def nav5(self):
                  # 定义要执行的 ROS 命令
        ros_command = 'roslaunch robot_r 5nav_D.launch'          
            # 构建完整的终端命令，使用 gnome-terminal 打开新的终端窗口并执行 ROS 命令
        terminal_command = ['gnome-terminal', '--', 'bash', '-c', ros_command + '; exec bash']          
            # 使用 subprocess.Popen 启动新的终端
        self.run5NavProcess = subprocess.Popen(terminal_command, preexec_fn=os.setpgrp)  # 记录进程          
        # 替换下面的命令为你实际要启动的roslaunch命令
        #command = ['roslaunch', 'robot_r', '5nav.launch']
        #subprocess.Popen(command)
        #self.start_button.isEnabled = False

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


    def quit_navigation(self):
        process_name = '3startlocation_withNav'  # 替换成你的roslaunch启动的具体名称或者相关的节点名
        process_id = subprocess.check_output(["pgrep", "-f", process_name]).decode().strip()       
        if process_id:
            # 将字符串转换成整数列表并杀死进程
            for pid in process_id.split():
                subprocess.call(['kill', '-15', pid])  # 使用SIGTERM信号(-15)尝试正常关闭进程
                # 如果需要强制结束，可替换为 'kill -9 {}'.format(pid)，但非必要时尽量避免
        self.start_button.isEnabled  = True

 
    def close_app(self):
        self.exitNAV()
        self.quit_navigation()
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