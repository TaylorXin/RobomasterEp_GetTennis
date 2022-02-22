import sys
from connect import Ui_MainWindow
from robot_ui import Ui_Form
from qr_ui import Ui_Dialog
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, qApp, QColorDialog
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QEvent
import logging
from robomaster import conn, robot, camera, led, action
from MyQR import myqr
from threading import Thread
import time
import cv2
import numpy as np


QRCODE_NAME = "qrcode.png"
FRAMEWIDTH = 640
FRAMEHEIGTH = 360
IsConnected = False
IsFire = False
IsDetectEp = False
IsNumFire = False
IsHsv = False
IsFollow = False
IsFollowTennis = False
IsGripTenis = False
IsInfantry = True
IsGetTenis = False
IsScanningSound = True
stop_sound_threads = False
zeroyaw = 0
fire_counter = 0
led_status = 0
gripper_status = ""
first_tennis_x = 320
x_val = 0.3
y_val = 0.3
z_val = 30
yaw = 0
pitch = 0
roll = 0
r = 123
g = 211
b = 15
speed = 60
rlspeed = 100
initBB = None
color_dist = {
    'red': {'Lower': np.array([0, 58, 72]), 'Upper': np.array([7, 255, 160])},
    'blue': {'Lower': np.array([100, 43, 46]), 'Upper': np.array([124, 255, 255])},
    'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
    'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},
    'orange': {'Lower': np.array([0, 91, 208]), 'Upper': np.array([25, 255, 255])},
    'white': {'Lower': np.array([0, 0, 142]), 'Upper': np.array([237, 12, 255])},
}


def try_except(func):
    # try-except function. Usage: @try_except decorator
    def handler(*args, **kwargs):
        try:
            func(*args, **kwargs)
        except Exception as e:
            print(e)

    return handler


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.setupUi(self)
        logging.basicConfig(level=logging.INFO)
        self.label_title_5.setVisible(False)
        self.label_title_6.setVisible(False)
        self.label_title_7.setVisible(False)
        self.ep_robot = robot.Robot()
        self.connect_type = "sta"
        self.pushButton.clicked.connect(self.connect_ep)
        try:
            self.label_QR.installEventFilter(self)
        except Exception as e:
            print(e)

    def eventFilter(self, source, event):
        if source == self.label_QR and event.type() == QEvent.MouseButtonDblClick:
            print("clicked label3")
            self.qrform = QrForm()
            self.qrform.show()
        return QWidget.eventFilter(self, source, event)

    def connect_ep(self):
        global helper, Isignore, IsInfantry
        if not self.checkBox_4.isChecked():
            Isignore = False
            if self.radioButton.isChecked():
                self.connect_type = "ap"
                self.robot_form = RobotForm(self.connect_type)
                self.robot_form.show()
                self.close()
            if self.comboBox_robot_style.currentIndex() == 0:
                IsInfantry = True
            else:
                IsInfantry = False
            if self.radioButton_2.isChecked():
                self.label_QR.show()
                self.label_stacon_tip.show()
                self.connect_type = "sta"
                try:
                    self.ep_robot.initialize(conn_type=self.connect_type)
                    self.robot_form = RobotForm(self.connect_type)
                    self.robot_form.show()
                    self.close()
                except Exception as e:
                    print("机器人连接失败，原因：", str(e))
                    ssid = self.lineEdit_wifissid.text()
                    password = self.lineEdit_wifipawd.text()
                    helper = conn.ConnectionHelper()
                    info = helper.build_qrcode_string(ssid=ssid, password=password)
                    myqr.run(words=info)
                    png = QPixmap(QRCODE_NAME)
                    png = png.scaled(200, 200, aspectRatioMode=Qt.KeepAspectRatio)
                    self.label_QR.setPixmap(png)
                    self.label_QR.setScaledContents(True)
                    self.label_QR.setAlignment(Qt.AlignCenter)
                    self.label_stacon_tip.setText("确认EP连接方式为wifi模式，按下主控上红色连接按钮，扫描右侧二维码。")
                    self.stacon_thread = StcConnectThread()
                    self.stacon_thread.trigger.connect(self.connect_success)
                    self.stacon_thread.start()
            if self.radioButton_3.isChecked():
                self.connect_type = "rndis"
                self.robot_form = RobotForm(self.connect_type)
                # self.robot_form.setWindowFlags(Qt.FramelessWindowHint)  # 无边框
                self.robot_form.show()
                # self.close()
        else:
            Isignore = True
            logging.info("ignore robot connection")
            self.connect_type = "none"
            self.robot_form = RobotForm(self.connect_type)
            self.robot_form.show()
            app.quit()

    def connect_success(self, str1):
        print("success")
        print(str1)
        self.robot_form = RobotForm(self.connect_type)
        self.robot_form.show()
        self.close()


class StcConnectThread(QThread):
    trigger = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)

    def run(self):
        logging.info("等待机器人扫描连接")
        global IsConnected
        try:
            if helper.wait_for_connection():
                print("连接成功！")
                IsConnected = True
            else:
                IsConnected = False
                print("连接失败！")
        except Exception as e:
            logging.error(e)
        self.trigger.emit(IsConnected)


class PointInfo:
    def __init__(self, x, y, theta, c):
        self._x = x
        self._y = y
        self._theta = theta
        self._c = c

    @property
    def pt(self):
        return int(self._x * FRAMEWIDTH), int(self._y * FRAMEHEIGTH)

    @property
    def color(self):
        return 255, 255, 255


class MarkerInfo:

    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * FRAMEWIDTH), int((self._y - self._h / 2) * FRAMEHEIGTH)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * FRAMEWIDTH), int((self._y + self._h / 2) * FRAMEHEIGTH)

    @property
    def center(self):
        return int(self._x * FRAMEWIDTH), int(self._y * FRAMEHEIGTH)

    @property
    def text(self):
        return self._info


class QrForm(QWidget, Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        png = QPixmap(QRCODE_NAME)
        self.label.setPixmap(png)
        self.label.setScaledContents(True)
        self.label.setAlignment(Qt.AlignCenter)


class RobotForm(QWidget, Ui_Form):
    def __init__(self, con_type):
        super().__init__()
        self.setupUi(self)
        if not Isignore:
            self.action = action.Action()
            self.ep_robot = robot.Robot()
            self.ep_robot.initialize(conn_type=con_type)
            version = self.ep_robot.get_version()
            print("Robot Version: {0}".format(version))
            SN = self.ep_robot.get_sn()
            sn_msg = "机器人序列号: {0}".format(SN)
            self.IsLinePatrol = False
            self.IsTracker = False
            self.lineinfo = []
            self.markinfo = []
            self.markdict = dict()
            self.markdict_inorder = dict()
            self.line_type = 0
            self.tracker = cv2.legacy.TrackerMOSSE_create()
            self.initBB = None
            self.label_sn.setText(sn_msg)
            self.pushButton.clicked.connect(self.selectColor)
            self.pushButton_reset.clicked.connect(self.robot_reset)
            self.pushButton_grip_tennis.clicked.connect(self.start_grip_tennis)

            if IsInfantry:
                self.ep_gimbal = self.ep_robot.gimbal
                self.ep_gimbal.recenter().wait_for_completed()
            else:
                self.ep_arm = self.ep_robot.robotic_arm
                self.ep_gripper = self.ep_robot.gripper
                self.ep_arm.sub_position(freq=5, callback=self.sub_data_handler)
                self.ep_gripper.sub_status(freq=1, callback=self.sub_gripper_handler)
                self.ep_arm.moveto(90, 29)
                # if  gripper_status =="normal" or gripper_status =="closed":
                self.ep_gripper.open(power=30)
                time.sleep(0.2)
                # self.ep_gripper.unsub_status()

            self.ep_battery = self.ep_robot.battery
            self.ep_chassis = self.ep_robot.chassis
            self.ep_camera = self.ep_robot.camera

            self.ep_led = self.ep_robot.led
            self.ep_vision = self.ep_robot.vision
            self.ep_blaster = self.ep_robot.blaster
            self.ep_sensor = self.ep_robot.sensor
            self.uart = self.ep_robot.uart  # 定义串口
            self.uart.serial_param_set(baud_rate=4, data_bit=1, odd_even=0, \
                                       stop_bit=0, rx_en=1, tx_en=1, rx_size=50, tx_size=50)
            self.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
            self.ep_battery.sub_battery_info(1, self.sub_info_handler)
            self.ep_chassis.sub_position(cs=0, freq=2, callback=self.sub_position_handler)
            self.ep_chassis.sub_attitude(freq=2, callback=self.sub_attitude_handler)
            self.ep_chassis.sub_velocity(freq=2, callback=self.sub_velocity_handler)
            self.ep_sensor.sub_distance(freq=5, callback=self.sub_tofdata_handler)
            self.ep_vision.sub_detect_info(name="marker", callback=self.on_detect_marker)
            self.startcam()

    def startcam(self):
        self.timer1 = QTimer()
        self.timer1.timeout.connect(self.visionfunction)
        self.timer1.start(10)

    # 目标跟随
    def start_follow_fun(self):
        global IsFollow, IsFollowTennis
        if IsFollow:
            IsFollow = False
            self.ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            self.pushButton_start_follow.setText("开始目标跟随")
        else:
            IsFollow = True
            # self.ep_gimbal.move(pitch=-10, yaw=0).wait_for_completed(1)
            self.pushButton_start_follow.setText("停止目标跟随")
            if self.checkBox_tennis.isChecked():
                IsFollowTennis = True
            if self.checkBox_robot.isChecked():
                IsFollowTennis = False

    def start_grip_tennis(self):
        global IsGripTenis, stop_sound_threads, zeroyaw
        if IsGripTenis:
            IsGripTenis = False
            stop_sound_threads = True
            self.pushButton_grip_tennis.setText("开始抓取")
            self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        else:
            IsGripTenis = True
            self.ep_led.set_led(comp=led.COMP_BOTTOM_ALL, r=185, g=10, b=5, effect=led.EFFECT_FLASH)
            zeroyaw = time.perf_counter()
            self.pushButton_grip_tennis.setText("停止抓取")

    def robot_reset(self):
        thread = Thread(target=self.reset_task, daemon=True)
        # print(' success (%gx%g at %.2f FPS).' % (w, h, fps))
        thread.start()

    def reset_task(self):
        # self.ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        if IsInfantry:
            self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            self.ep_gimbal.recenter().wait_for_completed()
        else:
            self.ep_arm.moveto(90, 29)
            if gripper_status == "normal" or gripper_status == "closed":
                self.ep_gripper.open(power=30)
                time.sleep(0.2)

    def set_robot_mode(self):
        if self.comboBox_set_mode.currentIndex() == 0:
            self.ep_robot.set_robot_mode(mode=robot.FREE)
        if self.comboBox_set_mode.currentIndex() == 1:
            self.ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
        if self.comboBox_set_mode.currentIndex() == 2:
            self.ep_robot.set_robot_mode(mode=robot.GIMBAL_LEAD)

    def on_detect_marker(self, marker_info):
        number = len(marker_info)
        self.markinfo.clear()
        self.markdict.clear()
        self.markdict_inorder.clear()
        for i in range(0, number):
            x, y, w, h, info = marker_info[i]
            self.markdict[info] = (x, y, w, h)
            self.markinfo.append(MarkerInfo(x, y, w, h, info))
        if IsNumFire:
            for i in sorted(self.markdict):
                self.markdict_inorder[i] = self.markdict[i]
            print(self.markdict_inorder)

    @try_except
    def on_detect_line(self, line_info):
        number = len(line_info)
        self.lineinfo.clear()
        if number > 0:
            self.line_type = line_info[0]
        # print('line_type', self.line_type)
        for i in range(1, number):
            x, y, ceta, c = line_info[i]
            self.lineinfo.append(PointInfo(x, y, ceta, c))
        # print(self.lineinfo)

    # 设置底盘颜色
    def selectColor(self):
        global r, g, b
        col = QColorDialog.getColor()
        r = col.red()
        g = col.green()
        b = col.blue()
        self.pushButton.setStyleSheet(
            'QPushButton{background-color: %s }'
            % col.name()
        )
        self.ep_led.set_led(comp=led.COMP_ALL, r=r, g=g, b=b, effect=led.EFFECT_ON)

    @try_except
    def closeEvent(self, event):
        print("close robot")
        self.timer1.stop()
        self.ep_chassis.unsub_attitude()
        self.ep_chassis.unsub_position()
        self.ep_chassis.unsub_status()
        self.ep_camera.stop_video_stream()
        self.ep_battery.unsub_battery_info()
        self.ep_robot.close()
        app.quit()
        self.close()

    def sub_gripper_handler(self, sub_info):
        global gripper_status
        gripper_status = sub_info


    def sub_tofdata_handler(self, sub_info):
        global distance
        distance = sub_info[0] / 10
        tof_msg = "超声波距离：{0}厘米".format(distance)
        self.label_tof.setText(tof_msg)

    def sub_data_handler(self, sub_info):
        pos_x, pos_y = sub_info
        arm_msg = "机械臂位置: pos x:{0}, pos y:{1}".format(pos_x, pos_y)
        # print(arm_msg)
        self.label_arm_info.setText(arm_msg)

    # 返回电池信息
    def sub_info_handler(self, batter_info):
        percent = batter_info
        msg = "机器人电量: {0}%.".format(percent)
        self.label_bat.setText(msg)
        if percent < 5:
            self.close()

    def sub_position_handler(self, position_info):
        x, y, z = position_info
        x = round(x, 3)
        y = round(y, 3)
        xyz_msg = "机器人位置: x:{0} y:{1} z:{2}".format(x, y, yaw)
        self.label_posi.setText(xyz_msg)

    def sub_attitude_handler(self, attitude_info):
        global yaw, pitch, roll
        yaw, pitch, roll = attitude_info
        yaw = round(yaw, 3)

    def sub_velocity_handler(self, velocity_info):
        vgx, vgy, vgz, vbx, vby, vbz = velocity_info
        vgx = round(vgx, 3)
        vgy = round(vgy, 3)
        vgz = round(vgz, 3)
        vgxyz_msg = "机器人速度: vgx:{0} vgy:{0} vgz:{0}".format(vgx)
        self.label_speed.setText(vgxyz_msg)

    # 捕捉键盘事件，注意需要在控件中设置setFocus(),否则方向键和空格键无法捕捉
    def keyPressEvent(self, event):
        try:
            key = event.key()
            if key == Qt.Key_Up:
                self.ep_arm.move(x=0, y=15).wait_for_completed(timeout=0.5)
                time.sleep(0.1)
            elif key == Qt.Key_Down:
                self.ep_arm.move(x=0, y=-15).wait_for_completed(timeout=0.5)
                time.sleep(0.1)
            elif key == Qt.Key_Right:
                self.ep_arm.move(x=15, y=0).wait_for_completed(timeout=0.5)
                time.sleep(0.1)
            elif key == Qt.Key_Left:
                self.ep_arm.move(x=-15, y=0).wait_for_completed(timeout=0.5)
                time.sleep(0.1)
            elif key == Qt.Key_E:
                self.ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_Q:
                self.ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_A:
                self.ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_D:
                self.ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_W:
                self.ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_S:
                self.ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
                time.sleep(0.1)
            elif key == Qt.Key_Space:
                self.ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            elif key == Qt.Key_Z:
                self.ep_gripper.open(power=50)
                time.sleep(0.2)
                self.ep_gripper.pause()
            elif key == Qt.Key_X:
                self.ep_gripper.close(power=50)
                time.sleep(0.2)
                self.ep_gripper.pause()
            elif key == Qt.Key_L:
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=10)
                time.sleep(0.2)
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            elif key == Qt.Key_J:
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=-10)
                time.sleep(0.2)
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            elif key == Qt.Key_I:
                self.ep_gimbal.drive_speed(pitch_speed=10, yaw_speed=0)
                time.sleep(0.2)
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            elif key == Qt.Key_K:
                self.ep_gimbal.drive_speed(pitch_speed=-10, yaw_speed=0)
                time.sleep(0.2)
                self.ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        except Exception as e:
            print(e)

    # 键盘释放事件，发送停止运行指令
    def keyReleaseEvent(self, event):
        # pass
        if event.isAutoRepeat():
            event.ignore()
        else:
            # print("stop move")
            self.ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    def pid_user(self, k, setv, getv):
        return k * (setv - getv)

    # 开线程执行抓起网球动作，保证画面不卡顿
    def grip_tennis_thread(self):
        thread = Thread(target=self.grip_tennis, daemon=True)
        thread.start()

    def grip_tennis(self):
        global IsGetTenis
        try:
            # time.sleep(2)
            self.ep_arm.moveto(180, -98).wait_for_completed(timeout=2)
            # time.sleep(2)
            self.ep_gripper.close(power=50)
            time.sleep(1)
            self.ep_arm.moveto(100, 100).wait_for_completed(timeout=1)
            # time.sleep(1)
            IsGetTenis = True
        except Exception as e:
            print(e)

    # 开线程执行放下网球动作，保证画面不卡顿
    def realease_tennis_thread(self):
        thread1 = Thread(target=self.realease_tennis, daemon=True)
        thread1.start()

    def realease_tennis(self):
        global IsGetTenis, IsGripTenis, zeroyaw
        try:
            IsGetTenis = False
            self.ep_arm.moveto(170, 130).wait_for_completed(timeout=2)
            # time.sleep(1)
            self.ep_gripper.open(power=50)
            time.sleep(1)
            self.ep_chassis.drive_speed(x=-0.4, y=0, z=0)
            time.sleep(0.5)
            self.ep_chassis.drive_speed(x=0, y=0, z=0)
            self.ep_arm.moveto(90, 30).wait_for_completed(timeout=2)
            IsGripTenis = True
            zeroyaw = time.perf_counter()
            self.pushButton_grip_tennis.setText("停止抓取")
        except Exception as e:
            print(e)

    # 开线程执行播放音频提醒程序
    def play_sound_thread(self):
        scanning_thread = Thread(target=self.play_fundtenis_sound, daemon=True)
        scanning_thread.start()

    def play_fundtenis_sound(self):
        global stop_sound_threads
        try:
            while True:
                time.sleep(0.2)
                if not stop_sound_threads:
                    if not self.ep_robot.action_dispatcher.has_in_progress_actions:
                        if IsScanningSound:
                            self.ep_robot.play_sound(robot.SOUND_ID_SCANNING).wait_for_completed()
                        else:
                            self.ep_robot.play_sound(robot.SOUND_ID_RECOGNIZED).wait_for_completed()
                            break
                else:
                    break
        except Exception as e:
            print(e)

    def visionfunction(self):
        '''完成视觉检测各个子功能'''
        global fps, fire_counter, IsFollow, IsFollowTennis, IsGripTenis, IsGetTenis, \
            first_tennis_x, stop_sound_threads, IsScanningSound
        try:
            frame = self.ep_camera.read_cv2_image()
            frame_hsv = frame.copy()
            height, width, bytesPerComponent = frame.shape
            bytesPerLine = bytesPerComponent * width
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 开启抓取网球
            if IsGripTenis:
                coutours_dict = dict()
                max2coutourlist = list()
                low_hsv = (34, 102, 125)
                high_hsv = (60, 255, 255)
                frame_hsv = cv2.cvtColor(frame_hsv, cv2.COLOR_BGR2HSV)
                newimg = cv2.inRange(frame_hsv, lowerb=low_hsv, upperb=high_hsv)
                contours, hier = cv2.findContours(newimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                maxa = 0
                maxi = 0
                ballr = 62.5 / 2
                f = 310
                # 定义视野中心
                target_x, target_y = 320, 300
                for i in range(len(contours)):
                    tmp = cv2.contourArea(contours[i])
                    coutours_dict[i] = tmp
                    if tmp > maxa:
                        maxa = tmp
                coutours_dict_inorder = {k: v for k, v in
                                         sorted(coutours_dict.items(), key=lambda item: item[1], reverse=True)}
                if maxa < 150:
                    speedx = 0
                    speedy = 0
                    self.ep_chassis.drive_speed(x=0, y=0, z=30)  # 旋转寻找目标
                    # print("时间差：",zeroyaw - time.perf_counter())
                    # 通过计时判断是否旋转一周，若一周未发现目标，则停止检测
                    IsScanningSound = True
                    stop_sound_threads = False
                    self.play_sound_thread()
                    if abs(time.perf_counter() - zeroyaw) > 12.0:
                        self.pushButton_grip_tennis.click()
                        self.ep_led.set_led(comp=led.COMP_BOTTOM_ALL, r=0, g=0, b=255, effect=led.EFFECT_FLASH)
                else:
                    IsScanningSound = False
                    self.ep_chassis.drive_speed(x=0, y=0, z=0)
                    num = 0
                    for k in coutours_dict_inorder.keys():
                        if num < 2:
                            num += 1
                            max2coutourlist.append(k)
                        else:
                            break
                    # 在最大轮廓区域求最小闭包圆，获得圆心在图像区域中的位置坐标和半径
                    if len(max2coutourlist) == 2:
                        (bx1, by1), radius1 = cv2.minEnclosingCircle(contours[max2coutourlist[0]])
                        (bx2, by2), radius2 = cv2.minEnclosingCircle(contours[max2coutourlist[1]])
                        diff1 = abs(bx1 - first_tennis_x)
                        diff2 = abs(bx2 - first_tennis_x)
                        if diff1 < diff2:
                            bx = bx1
                            by = by1
                            radius = radius1
                        else:
                            bx = bx2
                            by = by2
                            radius = radius2
                    else:
                        (bx, by), radius = cv2.minEnclosingCircle(contours[max2coutourlist[0]])
                    center = (int(bx), int(by))
                    first_tennis_x = int(bx)
                    # print("第一个识别的球中心X位置", first_tennis_x)
                    radius = int(radius)
                    img = cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    # 利用已知的焦距、图像中的球的半径以及实际球的大小，求网球到相机间的距离
                    if radius != 0:
                        distance = f * ballr / radius
                    # print("距离网球距离：", distance)
                    # 根据小球在图像中的位置，以视野中心点为目标，控制小球沿Y方向运动，使球保持于视野中心
                    if abs(target_x - bx) < 10:
                        speedy = 0
                    else:
                        speedy = -self.pid_user(0.2, target_x, bx)
                    if abs(target_y - by) < 15:
                        speedx = 0
                    else:
                        speedx = self.pid_user(0.2, target_y, by) / 150
                    if speedx == 0 and speedy == 0:
                        stop_sound_threads = True
                        print("开始抓取")
                        # print("EP正在执行的任务", self.action.target)
                        IsGripTenis = False
                        self.pushButton_grip_tennis.setText("开始抓取")
                        try:
                            self.grip_tennis_thread()
                        except Exception as e:
                            print(e)
                    else:
                        # print("输出速度：", speedx, speedy)
                        self.ep_chassis.drive_speed(x=speedx, y=0, z=speedy)

            if IsGetTenis:
                if len(self.markinfo) > 0:
                    frame = cv2.rectangle(frame, self.markinfo[0].pt1, self.markinfo[0].pt2, (0, 255, 0), 2)
                    target_x = 320
                    target_height = 135  # 通过识别目标高度确定距离
                    bx = self.markinfo[0].center[0]
                    by = self.markinfo[0].pt2[1] - self.markinfo[0].pt1[1]
                    if abs(target_x - bx) < 10:
                        speedy = 0
                    else:
                        speedy = -self.pid_user(0.2, target_x, bx)
                    if abs(target_height - by) < 10:
                        speedx = 0
                    else:
                        speedx = self.pid_user(0.2, target_height, by) / 50
                    print(speedx, speedy)
                    if speedx == 0 and speedy == 0:
                        self.realease_tennis_thread()
                    else:
                        self.ep_chassis.drive_speed(x=speedx, y=0, z=speedy)
                else:
                    self.ep_chassis.drive_speed(x=0, y=0, z=25)

            # 开启目标跟随（网球）
            if IsFollow:
                if IsFollowTennis:
                    low_hsv = (34, 102, 125)
                    high_hsv = (60, 255, 255)
                    frame_hsv = cv2.cvtColor(frame_hsv, cv2.COLOR_BGR2HSV)
                    newimg = cv2.inRange(frame_hsv, lowerb=low_hsv, upperb=high_hsv)
                    contours, hier = cv2.findContours(newimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    maxa = 0
                    maxi = 0
                    ballr = 62.5 / 2
                    f = 310
                    # 定义视野中心
                    target_x, target_y = 320, 220
                    for i in range(len(contours)):
                        tmp = cv2.contourArea(contours[i])
                        if tmp > maxa:
                            maxa = tmp
                            maxi = i
                    if maxa < 200:
                        speedx = 0
                        speedy = 0
                    else:
                        # 在最大轮廓区域求最小闭包圆，获得圆心在图像区域中的位置坐标和半径
                        (bx, by), radius = cv2.minEnclosingCircle(contours[maxi])
                        center = (int(bx), int(by))
                        print(center)
                        radius = int(radius)
                        img = cv2.circle(frame, center, radius, (0, 255, 0), 2)
                        # 利用已知的焦距、图像中的球的半径以及实际球的大小，求网球到相机间的距离
                        distance = f * ballr / radius
                        print(distance)
                        # 根据小球在图像中的位置，以视野中心点为目标，控制小球沿Y方向运动，使球保持于视野中心
                        if abs(target_x - bx) < 20:
                            speedy = 0
                        else:
                            speedy = -self.pid_user(0.2, target_x, bx)
                        # 根据小球到相机的距离，控制小车是否拦截小球
                        if distance < 500:
                            speedx = 0
                        else:
                            speedx = -self.pid_user(0.001, 200, distance) / 2
                    print(speedx, speedy)
                    self.ep_chassis.drive_speed(x=speedx, y=0, z=speedy)
            # 转为QImage对象
            self.image = QImage(
                frame.data, width, height, bytesPerLine, QImage.Format_RGB888
            )
            self.label_cam.setPixmap(QPixmap.fromImage(self.image))
            self.label_cam.setAlignment(Qt.AlignCenter)
            self.label_cam.setScaledContents(True)
        except Exception as e:
            print("！！！！！视频浏览错误！！！！！")
            print(e)
            # self.stopshow1()
            self.label_cam.clear()
            self.label_cam.setText("等待画面重新连接")
            self.label_cam.setStyleSheet(
                "QLabel {background: #82c9ff;font: 32px;color: white;}"
            )
            self.label_cam.setAlignment(Qt.AlignCenter)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
