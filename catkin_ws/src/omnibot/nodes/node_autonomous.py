from math import pi
from time import time
import rospy
from std_msgs.msg import Int32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Point
from omnibot.pid import Pid


class NodePlay:
    # f = open('log.txt')
    position = 0
    finish = False
    pid_x = Pid(0.5, 0, 0.2)
    pid_y = Pid(0.5, 0, 0.2)
    pid_z = Pid(0.8, 0, 0.25)
    twist = Twist()
    start_timestamp = 0
    def odom_callback(self, msg_data: Odometry):
        self.pid_x.pos = msg_data.pose.pose.position.x
        self.pid_y.pos = msg_data.pose.pose.position.y
        # self.pid_z.pos = msg_data.pose.pose.orientation.z
        # rospy.loginfo(f'x : {self.pid_x.last_err} y : {self.pid_y.last_err} z : {self.pid_z.last_err}')
        if self.finish is not True:
            self.twist.linear.x = self.pid_y.pid()
            self.twist.linear.y = - self.pid_x.pid()
            self.twist.angular.z = - self.pid_z.pid()
            # rospy.loginfo(f'zErr : {self.pid_z.last_err} pid : {self.twist.angular.z}')
            self.cmd_vel.publish(self.twist)
            timestamp = int(time() * 1000)
            rospy.loginfo(f'{timestamp - self.start_timestamp} {self.pid_x.pos} {self.pid_y.pos} {self.pid_z.pos}')
        if abs(self.pid_x.sp - self.pid_x.pos) < 0.01 and abs(self.pid_y.sp - self.pid_y.pos) < 0.01 and abs(self.pid_z.sp - self.pid_z.pos) < 0.01:
            self.position += 1
            if self.position < self.arr.__len__():
                self.pid_x.reset_err()
                self.pid_y.reset_err()
                self.pid_z.reset_err()
                self.pid_x.sp = self.arr[self.position][0]
                self.pid_y.sp = self.arr[self.position][1]
                self.pid_z.sp = self.arr[self.position][2]
            else:
                self.finish = True
                self.twist.linear.x = 0
                self.twist.linear.y = 0
                self.twist.angular.z = 0
                self.cmd_vel.publish(self.twist)
                rospy.loginfo('finish')

    def __reset(self, msg):
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_z.reset_err()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.angular.z = 0
        self.cmd_vel.publish(self.twist)
        self.finish = True

    def __setpoint(self, msg:PoseStamped):
        self.pid_x.sp = msg.pose.position.x
        self.pid_y.sp = msg.pose.position.y
        self.pid_z.sp = 0
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_z.reset_err()
        self.start_timestamp = int(time() * 1000)
        # rospy.loginfo(f'x : {self.pid_x.sp} y : {self.pid_y.sp} z : {self.pid_z.sp}')
        self.finish = False

    def __cmp_callback(self, msg:Int32):
        self.pid_z.pos = msg.data * pi / 180


    def __set_pid(self, msg:Point):
        self.pid_x = Pid(msg.x, msg.y, msg.z)
        self.pid_y = Pid(msg.x, msg.y, msg.z)

    def __set_angular_pid(self, msg:Point):
        self.pid_z = Pid(msg.x, msg.y, msg.z)

    def __init__(self) -> None:
        # self.arr = self.f.readlines()
        # self.arr = [[1, 0, 1], [0, 0, 1], [0, 1, 1], [0, 0, 1]]
        # self.arr = [[0,1,1], [0,0,1]]
        self.arr = [[0, 0, 0]]
        self.pid_x.sp = self.arr[self.position][0]
        self.pid_y.sp = self.arr[self.position][1]
        self.pid_z.sp = self.arr[self.position][2]
        rospy.init_node('node_autonomous')
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('/sensor/compass', Int32, self.__cmp_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__setpoint)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        # rospy.Subscriber('/set_pid', Point, self.__set_pid)
        # rospy.Subscriber('/set_angular_pid', Point, self.__set_angular_pid)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # for i in range(arr.__len__()):
        #     print(arr[i])


if __name__ == "__main__":
    n = NodePlay()

    while not rospy.is_shutdown():
        rospy.spin()
