from math import pi
import math
from time import time
import rospy
from std_msgs.msg import Int32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from openbase.pid import Pid
from tf.transformations import euler_from_quaternion
import numpy as np

class NodeAutonomous:
    position = 0
    finish = False
    pid_x = Pid(0.5, 0, 0.2)
    pid_y = Pid(0.5, 0, 0.2)
    pid_w = Pid(0.1, 0, 0.25)
    twist = Twist()
    start_timestamp = 0
    euler_deg = 0
    last_euler_deg = 0
    def odom_callback(self, msg_data: Odometry):
        (roll, pitch, yaw) = euler_from_quaternion([
            msg_data.pose.pose.orientation.x,
            msg_data.pose.pose.orientation.y,
            msg_data.pose.pose.orientation.z,
            msg_data.pose.pose.orientation.w,
        ])

        # Konversi dari quaternion ke euler (0 - 360 deg)
        euler_deg = yaw * 180 / math.pi
        if euler_deg < 0:
            euler_deg = 180 + np.interp(euler_deg, [-179, -1], [0, 180])

        # Konversi ke continuous heading
        if abs(euler_deg - self.last_euler_deg) > 300:
            if euler_deg - self.last_euler_deg < 0:
                offset = (360 - self.last_euler_deg) + euler_deg
                self.euler_deg += offset
            else:
                offset = (360 - euler_deg) + self.last_euler_deg
                self.euler_deg = self.last_euler_deg - (self.last_euler_deg + offset)
        else:
            self.euler_deg += euler_deg - self.last_euler_deg
        self.last_euler_deg = euler_deg
        
        self.pid_x.pos = msg_data.pose.pose.position.x
        self.pid_y.pos = msg_data.pose.pose.position.y
        self.pid_w.pos = (self.euler_deg) * pi / 180
        if self.finish is not True:
            self.twist.linear.x = self.pid_y.pid()
            self.twist.linear.y = - self.pid_x.pid()
            self.twist.angular.z = - self.pid_w.pid()
            self.cmd_vel.publish(self.twist)

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.01 and abs(self.pid_y.sp - self.pid_y.pos) < 0.01 and abs(self.pid_w.sp - self.pid_w.pos) < 0.01:
            self.position += 1
            if self.position < self.arr.__len__():
                self.pid_x.reset_err()
                self.pid_y.reset_err()
                self.pid_w.reset_err()
                self.pid_x.sp = self.arr[self.position][0]
                self.pid_y.sp = self.arr[self.position][1]
                self.pid_w.sp = self.arr[self.position][2]
            else:
                self.finish = True
                self.twist.linear.x = 0
                self.twist.linear.y = 0
                self.twist.angular.z = 0
                self.cmd_vel.publish(self.twist)

    def __reset(self, msg):
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_w.reset_err()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.angular.z = 0
        self.cmd_vel.publish(self.twist)
        self.finish = True

    def __setpoint(self, msg:PoseStamped):
        (roll, pitch, yaw) = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        # Konversi dari quaternion ke euler (0 - 360 deg)
        euler_deg = yaw * 180 / math.pi
        if euler_deg < 0:
            euler_deg = 180 + np.interp(euler_deg, [-179, -1], [0, 180])
        self.pid_x.sp = msg.pose.position.x
        self.pid_y.sp = msg.pose.position.y
        self.pid_w.sp = euler_deg * pi / 180
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_w.reset_err()
        self.start_timestamp = int(time() * 1000)
        self.finish = False

    def __cmp_callback(self, msg:Int32):
        self.pid_w.pos = msg.data * pi / 180

    def __init__(self) -> None:
        self.arr = [[0, 0, 0]]
        self.pid_x.sp = self.arr[self.position][0]
        self.pid_y.sp = self.arr[self.position][1]
        self.pid_w.sp = self.arr[self.position][2]
        rospy.init_node('node_autonomous', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__setpoint)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

if __name__ == "__main__":
    n = NodeAutonomous()
    rate = rospy.Rate(hz=20)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()