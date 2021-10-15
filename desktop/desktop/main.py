import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot, reset
from desktop.core.ws.odom_data import Odomdata
from desktop.core.ws.ws import Ws
from desktop.ui import Ui_MainWindow
from desktop.ui.graph_window import GraphWindow
from math import atan2, pi, sqrt

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

def get_distance_goal_and_heading(goal_x, goal_y, pose_x, pose_y, ):
    diff_x = goal_x - pose_x
    diff_y = goal_y - pose_y
    goal_distance = (sqrt((diff_y ** 2) + (diff_x ** 2))).__round__(2)
    goal_heading = atan2(diff_y, diff_x)
    return goal_distance, goal_heading
    
# Pysignal
x = 0
y = 0

def set_forward_dist():
    goal_x = float(ui.cb_forward.currentText())
    goal_y = 0
    data = [{'x':goal_x, 'y': goal_y}]
    print(data)
    ws.send_custom_data({
        'type':0,
        'data': data
    })
def pos_x_onchanged():
    global x
    x = float(ui.edit_pos_x.toPlainText())
    distance, heading = get_distance_goal_and_heading(x, y, 0, 0)
    text = f'Jarak {distance} m, sudut { heading * 180 / pi}°'
    ui.tv_auto_calculate_heading.setText(text)

def pos_y_onchanged():
    global y
    y = float(ui.edit_pos_y.toPlainText())
    distance, heading = get_distance_goal_and_heading(x, y, 0, 0)
    text = f'Jarak {distance} m, sudut {heading * 180 / pi}°'
    ui.tv_auto_calculate_heading.setText(text)

def set_heading():
    heading = float(int(ui.cb_heading.currentText()) * pi / 180)
    
def set_position():
    data = [{'x':x, 'y': y}]
    print(data)
    ws.send_custom_data({
        'type':0,
        'data': data
    })
def reset():
    ws.send_reset()

def main():
    ui.setupUi(main_window)
    main_window.show()
    ui.pb_set_forward.clicked.connect(set_forward_dist)
    ui.pb_set_heading.clicked.connect(set_heading)
    ui.edit_pos_x.textChanged.connect(pos_x_onchanged)
    ui.edit_pos_y.textChanged.connect(pos_y_onchanged)
    ui.pb_set_pos.clicked.connect(set_position)
    ui.pb_reset.clicked.connect(reset)
    # ws.robot_position.connect(draw_graph)
    sys.exit(app.exec_())