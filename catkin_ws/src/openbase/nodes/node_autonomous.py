from math import atan2, pi, sqrt
import math
from time import time
import rospy
from std_msgs.msg import Empty, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from openbase.pid import Pid
from tf.transformations import euler_from_quaternion
import numpy as np

class NodeAutonomous:
    array_position = 0
    finish = False
    pid_x = Pid(0.1, 0, 0.2)
    pid_y = Pid(0.1, 0, 0.2)
    pid_w = Pid(0.1, 0, 0.25)
    twist = Twist()
    start_timestamp = 0
    euler_deg = 0
    last_euler_deg = 0

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2*pi
        while alpha < -pi:
            alpha += 2*pi
        return alpha
    
    def on_odometry(self, msg_data: Odometry):
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
            goal_heading = atan2(self.pid_y.sp - self.pid_y.pos, self.pid_x.sp - self.pid_x.pos)
            magnitude = sqrt(((self.pid_y.sp - self.pid_y.pos)**2)- ((self.pid_x.sp - self.pid_x.pos)**2))
            self.twist.linear.x = magnitude
            self.twist.angular.z = goal_heading
            
            self.cmd_vel.publish(self.twist)

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.01 and abs(self.pid_y.sp - self.pid_y.pos) < 0.01:
            if self.array_position < self.arr.__len__():
                self.pid_x.reset_err()
                self.pid_y.reset_err()
                self.pid_w.reset_err()
                self.pid_x.sp = self.arr[self.array_position].x
                self.pid_y.sp = self.arr[self.array_position].y
                self.pid_w.sp = math.atan(self.pid_y.sp/(self.pid_x.sp+0.00001))
                self.array_position += 1
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
        self.array_position = 0

    def on_marker_set(self, msg:Marker):
        self.marker = msg

    def on_marker_follower(self, msg:Empty):
        self.arr = self.marker.points
        rospy.loginfo(self.marker.points)
        self.pid_x.sp = self.arr[self.array_position].x
        self.pid_y.sp = self.arr[self.array_position].y
        self.pid_w.sp = math.atan(self.pid_y.sp/(self.pid_x.sp+0.00001))
        self.finish = False

    def __init__(self) -> None:
        point = Point()
        self.arr = [point]
        self.pid_x.sp = self.arr[self.array_position].x
        self.pid_y.sp = self.arr[self.array_position].y
        self.pid_w.sp = 0
        rospy.init_node('node_autonomous', anonymous=True)
        self.base_frame_id = rospy.get_param('~base_frame_id')
        rospy.Subscriber(f'/{self.base_frame_id}/odom', Odometry, self.on_odometry)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        rospy.Subscriber('/marker', Marker, self.on_marker_set)
        rospy.Subscriber('/marker_follower', Empty, self.on_marker_follower)
        self.cmd_vel = rospy.Publisher(f'/{self.base_frame_id}/cmd_vel', Twist, queue_size=1)

if __name__ == "__main__":
    n = NodeAutonomous()
    rate = rospy.Rate(hz=20)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()