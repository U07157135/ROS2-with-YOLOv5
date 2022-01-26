# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/user/dev_ws/src/example/example/untitled.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(897, 649)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setStyleSheet("border-width: 1px;\n"
"border-color: rgb(255,0,0)")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.camera_btn = QtWidgets.QPushButton(self.centralwidget)
        self.camera_btn.setGeometry(QtCore.QRect(720, 290, 89, 25))
        self.camera_btn.setObjectName("camera_btn")
        self.detect_checkbox = QtWidgets.QCheckBox(self.centralwidget)
        self.detect_checkbox.setGeometry(QtCore.QRect(720, 250, 92, 23))
        self.detect_checkbox.setCheckable(False)
        self.detect_checkbox.setObjectName("detect_checkbox")
        self.view_label = QtWidgets.QLabel(self.centralwidget)
        self.view_label.setGeometry(QtCore.QRect(100, 100, 500, 400))
        self.view_label.setAutoFillBackground(False)
        self.view_label.setStyleSheet("background-color : rgb(85, 87, 83)")
        self.view_label.setText("")
        self.view_label.setObjectName("view_label")
        self.formLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget.setGeometry(QtCore.QRect(690, 170, 160, 71))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.fps_text = QtWidgets.QLabel(self.formLayoutWidget)
        self.fps_text.setObjectName("fps_text")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.fps_text)
        self.fps_value = QtWidgets.QLabel(self.formLayoutWidget)
        self.fps_value.setObjectName("fps_value")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.fps_value)
        self.resultion_text = QtWidgets.QLabel(self.formLayoutWidget)
        self.resultion_text.setObjectName("resultion_text")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.resultion_text)
        self.resultion_value = QtWidgets.QLabel(self.formLayoutWidget)
        self.resultion_value.setObjectName("resultion_value")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.resultion_value)
        self.time_text = QtWidgets.QLabel(self.formLayoutWidget)
        self.time_text.setObjectName("time_text")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.time_text)
        self.time_value = QtWidgets.QLabel(self.formLayoutWidget)
        self.time_value.setObjectName("time_value")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.time_value)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 897, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.camera_btn.setText(_translate("MainWindow", "camera"))
        self.detect_checkbox.setText(_translate("MainWindow", "Detect"))
        self.fps_text.setText(_translate("MainWindow", "FPS"))
        self.fps_value.setText(_translate("MainWindow", "0"))
        self.resultion_text.setText(_translate("MainWindow", "resultion"))
        self.resultion_value.setText(_translate("MainWindow", "0x0"))
        self.time_text.setText(_translate("MainWindow", "Time"))
        self.time_value.setText(_translate("MainWindow", "00:00:00"))
