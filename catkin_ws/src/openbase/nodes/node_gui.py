from math import atan2, pi, sqrt
import sys
# ROS stuff
import rospy
from std_msgs.msg import Empty, Float32, Int32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose2D

from time import sleep
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot, reset

from openbase.ui import Ui_MainWindow

app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QMainWindow()

ui = Ui_MainWindow()

def get_distance_goal_and_heading(goal_x, goal_y, pose_x, pose_y, ):
    diff_x = goal_x - pose_x
    diff_y = goal_y - pose_y
    goal_distance = (sqrt((diff_y ** 2) + (diff_x ** 2))).__round__(2)
    goal_heading = atan2(diff_y, diff_x)
    return goal_distance, goal_heading
    
# Pysignal
x = 0
y = 0

def publish_sp_pos(goal_x, goal_y):
    point = Point()
    point.x = float(goal_x)
    point.y = float(goal_y)
    marker = Marker()
    marker.points.append(point)
    marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)
    marker_pub.publish(marker)
    sleep(0.5)
    rospy.Publisher('/pivot_mode', Empty, queue_size=1).publish()

def set_forward_dist():
    goal_x = float(ui.cb_forward.currentText())
    goal_y = 0
    publish_sp_pos(goal_x, goal_y)

def set_heading():
    heading = Float32()
    heading.data = float(int(ui.cb_heading.currentText()) * pi / 180)
    rospy.Publisher('/sp_heading', Float32, queue_size=1).publish(heading)

def pos_x_onchanged():
    global x
    x = float(ui.edit_pos_x.toPlainText())
    distance, heading = get_distance_goal_and_heading(x, y, 0, 0)
    text = f'Jarak {distance} m, sudut { - heading * 180 / pi}°'
    ui.tv_auto_calculate_heading.setText(text)

def pos_y_onchanged():
    global y
    y = float(ui.edit_pos_y.toPlainText())
    distance, heading = get_distance_goal_and_heading(x, y, 0, 0)
    text = f'Jarak {distance} m, sudut {- heading * 180 / pi}°'
    ui.tv_auto_calculate_heading.setText(text)

def set_position():
    publish_sp_pos(x, y)
    
def reset():
    rospy.Publisher('/reset_pos', Empty, queue_size=1).publish()

def on_robot_pose(msg:Pose2D):
    ui.x_internal_val.setText(str(msg.x.__round__(2)))
    ui.y_internal_val.setText(str(msg.y.__round__(2)))
    try:
        ui.th_internal_val.setText(f'{int(msg.theta * 180/pi)} °')
    except:
        ui.th_internal_val.setText('0°')

def on_robot_heading(msg:Int32):
    ui.compass_val.setText(f'{msg.data} °')

if __name__=='__main__':
    while not rospy.is_shutdown():
        rospy.init_node('node_gui')
        base_frame_id = rospy.get_param('~base_frame_id')
        # rospy.Subscriber(f'/{base_frame_id}/pose_data',
        #                  Pose2D, on_robot_pose)
        # rospy.Subscriber(f'/{base_frame_id}/heading',
        #                  Int32, on_robot_heading)
        ui.setupUi(main_window)
        ui.pb_set_forward.clicked.connect(set_forward_dist)
        ui.pb_set_heading.clicked.connect(set_heading)
        ui.edit_pos_x.textChanged.connect(pos_x_onchanged)
        ui.edit_pos_y.textChanged.connect(pos_y_onchanged)
        ui.pb_set_pos.clicked.connect(set_position)
        ui.pb_reset.clicked.connect(reset)
        main_window.show()
        sys.exit(app.exec_())