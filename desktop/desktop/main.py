import sys
from time import sleep
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot
from desktop.core.ws.odom_data import Odomdata
from desktop.core.ws.ws import Ws
from random import randint, random
from desktop.ui import Ui_MainWindow
from desktop.ui.graph_window import GraphWindow

app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QMainWindow()
en_graph_window = GraphWindow()
ex_graph_window = GraphWindow()
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

arr_x = list()
arr_y = list()

@pyqtSlot(Odomdata)
def draw_graph(data:Odomdata):
    try:
        x = data.data.x
        y = data.data.y
        arr_x.append(x)
        arr_y.append(y)
        print(f'x : {x} y : {y}')
        en_graph_window.plot(arr_x, arr_y)
        en_graph_window.show()
    except Exception as e:
        print(e)

def main():
    ui.setupUi(main_window)
    ui.pb_save.clicked.connect(save)
    ui.pb_send.clicked.connect(send)
    main_window.show()
    ws.robot_position.connect(draw_graph)
    sys.exit(app.exec_())