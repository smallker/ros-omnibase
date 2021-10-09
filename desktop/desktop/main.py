import sys
from time import sleep
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot, reset
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

def move_forward():
    ws.send_movement('forward')

def move_backward():
    ws.send_movement('backward')


def turn_right():
    ws.send_movement('turn_right')

def turn_left():
    ws.send_movement('turn_left')

def move_stop():
    ws.send_movement('stop')

def reset_pos():
    ws.send_reset()

def main():
    ui.setupUi(main_window)
    ui.pb_save.clicked.connect(save)
    ui.pb_send.clicked.connect(send)
    ui.pb_move_forward.clicked.connect(move_forward)
    ui.pb_move_backward.clicked.connect(move_backward)
    ui.pb_turn_left.clicked.connect(turn_left)
    ui.pb_turn_right.clicked.connect(turn_right)
    ui.pb_move_stop.clicked.connect(move_stop)
    ui.pb_reset_pos.clicked.connect(reset_pos)
    main_window.show()
    ws.robot_position.connect(draw_graph)
    sys.exit(app.exec_())