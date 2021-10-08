import sys
from time import sleep
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot
from desktop.core.ws.ws import Ws

from desktop.ui import Ui_MainWindow

app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QMainWindow()
ui = Ui_MainWindow()

ws = Ws()
ws.start()
def save():
    x = ui.input_x.toPlainText()
    y = ui.input_y.toPlainText()
    ws.add_data(x, y)
    ui.list_pos.addItems([f'x : {x} y : {y}'])

def send():
    ws.send_data()

def main():
    ui.setupUi(main_window)
    ui.pb_save.clicked.connect(save)
    ui.pb_send.clicked.connect(send)
    main_window.show()
    sys.exit(app.exec_())