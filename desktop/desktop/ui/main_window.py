# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'desktop/ui/mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(205, 370)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pb_send = QtWidgets.QPushButton(self.centralwidget)
        self.pb_send.setGeometry(QtCore.QRect(100, 250, 81, 31))
        self.pb_send.setObjectName("pb_send")
        self.list_pos = QtWidgets.QListWidget(self.centralwidget)
        self.list_pos.setGeometry(QtCore.QRect(10, 10, 171, 192))
        self.list_pos.setObjectName("list_pos")
        self.input_x = QtWidgets.QTextEdit(self.centralwidget)
        self.input_x.setGeometry(QtCore.QRect(10, 210, 81, 31))
        self.input_x.setObjectName("input_x")
        self.pb_save = QtWidgets.QPushButton(self.centralwidget)
        self.pb_save.setGeometry(QtCore.QRect(10, 250, 81, 31))
        self.pb_save.setObjectName("pb_save")
        self.input_y = QtWidgets.QTextEdit(self.centralwidget)
        self.input_y.setGeometry(QtCore.QRect(100, 210, 81, 31))
        self.input_y.setObjectName("input_y")
        self.pb_clear = QtWidgets.QPushButton(self.centralwidget)
        self.pb_clear.setGeometry(QtCore.QRect(10, 290, 81, 31))
        self.pb_clear.setObjectName("pb_clear")
        self.pb_stop = QtWidgets.QPushButton(self.centralwidget)
        self.pb_stop.setGeometry(QtCore.QRect(100, 290, 81, 31))
        self.pb_stop.setObjectName("pb_stop")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 205, 22))
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
        self.pb_send.setText(_translate("MainWindow", "Kirim"))
        self.pb_save.setText(_translate("MainWindow", "Simpan"))
        self.pb_clear.setText(_translate("MainWindow", "Hapus"))
        self.pb_stop.setText(_translate("MainWindow", "Stop"))
