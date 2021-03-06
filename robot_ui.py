# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(785, 827)
        Form.setFocusPolicy(QtCore.Qt.StrongFocus)
        Form.setStyleSheet("QLabel{font: 12pt \"幼圆\";color:white;min-height:30px}\n"
"QWidget{background-color: rgb(6, 57, 241);}\n"
"QPushButton{font: 63 12pt \"Adobe Fan Heiti Std\";\n"
"background-color:rgb(0, 166, 77);color:white;min-height:40px}\n"
"QDoubleSpinBox{font: 12pt \"3ds\";color: rgb(255, 255, 255);}\n"
"QComboBox{font: 12pt \"3ds\";color: rgb(255, 255, 255);}\n"
"QSpinBox{font: 12pt \"3ds\";color: rgb(255, 255, 255);}\n"
"QRadioButton{font:12pt \"3ds\";color:rgb(255,255,255)}\n"
"QCheckBox{font:12pt \"3ds\";color:rgb(255,255,255)}")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(270, 660, 221, 42))
        self.pushButton.setMinimumSize(QtCore.QSize(0, 42))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_reset = QtWidgets.QPushButton(Form)
        self.pushButton_reset.setGeometry(QtCore.QRect(520, 660, 221, 42))
        self.pushButton_reset.setMinimumSize(QtCore.QSize(0, 42))
        self.pushButton_reset.setObjectName("pushButton_reset")
        self.label_cam = QtWidgets.QLabel(Form)
        self.label_cam.setGeometry(QtCore.QRect(50, 20, 671, 381))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.label_cam.sizePolicy().hasHeightForWidth())
        self.label_cam.setSizePolicy(sizePolicy)
        self.label_cam.setStyleSheet("background-color: rgb(0, 170, 255);")
        self.label_cam.setText("")
        self.label_cam.setObjectName("label_cam")
        self.pushButton_grip_tennis = QtWidgets.QPushButton(Form)
        self.pushButton_grip_tennis.setGeometry(QtCore.QRect(20, 660, 221, 42))
        self.pushButton_grip_tennis.setMinimumSize(QtCore.QSize(0, 42))
        self.pushButton_grip_tennis.setObjectName("pushButton_grip_tennis")
        self.checkBox_tennis = QtWidgets.QCheckBox(Form)
        self.checkBox_tennis.setGeometry(QtCore.QRect(60, 610, 91, 19))
        self.checkBox_tennis.setChecked(True)
        self.checkBox_tennis.setAutoExclusive(True)
        self.checkBox_tennis.setObjectName("checkBox_tennis")
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setGeometry(QtCore.QRect(60, 740, 581, 61))
        self.label_5.setLineWidth(1)
        self.label_5.setMidLineWidth(1)
        self.label_5.setWordWrap(True)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setGeometry(QtCore.QRect(60, 720, 121, 31))
        self.label_6.setObjectName("label_6")
        self.splitter_3 = QtWidgets.QSplitter(Form)
        self.splitter_3.setGeometry(QtCore.QRect(20, 430, 731, 90))
        self.splitter_3.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_3.setObjectName("splitter_3")
        self.splitter = QtWidgets.QSplitter(self.splitter_3)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.label_sn = QtWidgets.QLabel(self.splitter)
        self.label_sn.setObjectName("label_sn")
        self.label_bat = QtWidgets.QLabel(self.splitter)
        self.label_bat.setStyleSheet("")
        self.label_bat.setObjectName("label_bat")
        self.label_posi = QtWidgets.QLabel(self.splitter)
        self.label_posi.setObjectName("label_posi")
        self.splitter_2 = QtWidgets.QSplitter(self.splitter_3)
        self.splitter_2.setOrientation(QtCore.Qt.Vertical)
        self.splitter_2.setObjectName("splitter_2")
        self.label_speed = QtWidgets.QLabel(self.splitter_2)
        self.label_speed.setObjectName("label_speed")
        self.label_arm_info = QtWidgets.QLabel(self.splitter_2)
        self.label_arm_info.setObjectName("label_arm_info")
        self.label_tof = QtWidgets.QLabel(self.splitter_2)
        self.label_tof.setObjectName("label_tof")
        self.splitter_6 = QtWidgets.QSplitter(Form)
        self.splitter_6.setGeometry(QtCore.QRect(60, 550, 491, 31))
        self.splitter_6.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_6.setObjectName("splitter_6")
        self.splitter_4 = QtWidgets.QSplitter(self.splitter_6)
        self.splitter_4.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_4.setObjectName("splitter_4")
        self.label = QtWidgets.QLabel(self.splitter_4)
        self.label.setMinimumSize(QtCore.QSize(0, 30))
        self.label.setObjectName("label")
        self.doubleSpinBox_V = QtWidgets.QDoubleSpinBox(self.splitter_4)
        self.doubleSpinBox_V.setMinimumSize(QtCore.QSize(0, 30))
        self.doubleSpinBox_V.setStyleSheet("color: rgb(255, 255, 255);")
        self.doubleSpinBox_V.setDecimals(1)
        self.doubleSpinBox_V.setSingleStep(0.1)
        self.doubleSpinBox_V.setProperty("value", 0.3)
        self.doubleSpinBox_V.setObjectName("doubleSpinBox_V")
        self.label_3 = QtWidgets.QLabel(self.splitter_4)
        self.label_3.setObjectName("label_3")
        self.splitter_5 = QtWidgets.QSplitter(self.splitter_6)
        self.splitter_5.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_5.setObjectName("splitter_5")
        self.label_2 = QtWidgets.QLabel(self.splitter_5)
        self.label_2.setMinimumSize(QtCore.QSize(0, 30))
        self.label_2.setObjectName("label_2")
        self.doubleSpinBox_W = QtWidgets.QDoubleSpinBox(self.splitter_5)
        self.doubleSpinBox_W.setMinimumSize(QtCore.QSize(0, 30))
        self.doubleSpinBox_W.setStyleSheet("color:rgb(255, 255, 255)")
        self.doubleSpinBox_W.setDecimals(0)
        self.doubleSpinBox_W.setMaximum(540.0)
        self.doubleSpinBox_W.setProperty("value", 60.0)
        self.doubleSpinBox_W.setObjectName("doubleSpinBox_W")
        self.label_4 = QtWidgets.QLabel(self.splitter_5)
        self.label_4.setObjectName("label_4")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "设置灯光颜色"))
        self.pushButton_reset.setText(_translate("Form", "机器人复位"))
        self.pushButton_grip_tennis.setText(_translate("Form", "开始抓取（工程形态）"))
        self.checkBox_tennis.setText(_translate("Form", "网球"))
        self.label_5.setText(_translate("Form", "WSAD控制机器人前进后退左移和右移；QE控制机器人左转和右转；箭头上下左右控制机械抓抬高降低前移后移；ZX控制机械抓开合。"))
        self.label_6.setText(_translate("Form", "键盘控制说明："))
        self.label_sn.setText(_translate("Form", "机器人编号:"))
        self.label_bat.setText(_translate("Form", "机器人电量:"))
        self.label_posi.setText(_translate("Form", "机器人位置:"))
        self.label_speed.setText(_translate("Form", "机器人速度:"))
        self.label_arm_info.setText(_translate("Form", "机械臂位置:"))
        self.label_tof.setText(_translate("Form", "超声波距离:"))
        self.label.setText(_translate("Form", "平移速度值:"))
        self.label_3.setText(_translate("Form", "m/s"))
        self.label_2.setText(_translate("Form", "转向速度值:"))
        self.label_4.setText(_translate("Form", "。/s"))
import ep_rc
