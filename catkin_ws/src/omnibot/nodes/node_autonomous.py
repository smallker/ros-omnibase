import rospy
from std_msgs.msg import Int32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from omnibot.pid import Pid


class NodePlay:
    # f = open('log.txt')
    position = 0
    finish = False
    pid_x = Pid(0.5, 0, 0)
    pid_y = Pid(0.5, 0, 0)
    pid_z = Pid(0.5, 0, 0)
    twist = Twist()

    def callback(self, msg_data: Odometry):
        self.pid_x.pos = msg_data.pose.pose.position.x
        self.pid_y.pos = msg_data.pose.pose.position.y
        self.pid_z.pos = msg_data.pose.pose.orientation.z

        if self.finish is not True:
            self.twist.linear.x = self.pid_y.pid()
            self.twist.linear.y = - self.pid_x.pid()
            self.twist.angular.z = - self.pid_z.pid()
            self.cmd_vel.publish(self.twist)

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.5 and abs(self.pid_y.sp - self.pid_y.pos) < 0.05:
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

    def __reset(self, msg):
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_z.reset_err()
        self.finish = True

    def __setpoint(self, msg:PoseStamped):
        self.pid_x.sp = msg.pose.position.x
        self.pid_y.sp = msg.pose.position.y
        self.pid_z.sp = 0
        self.pid_x.reset_err()
        self.pid_y.reset_err()
        self.pid_z.reset_err()
        rospy.loginfo(f'x : {self.pid_x.sp} y : {self.pid_y.sp} z : {self.pid_z.sp}')
        self.finish = False

    def __init__(self) -> None:
        # self.arr = self.f.readlines()
        # self.arr = [[1, 0, 1], [0, 0, 1], [0, 1, 1], [0, 0, 1]]
        # self.arr = [[0,1,1], [0,0,1]]
        self.arr = [[0, 0, 0]]
        self.pid_x.sp = self.arr[self.position][0]
        self.pid_y.sp = self.arr[self.position][1]
        self.pid_z.sp = self.arr[self.position][2]
        rospy.init_node('node_autonomous')
        rospy.Subscriber('odom', Odometry, self.callback)
        # rospy.Subscriber('/sensor/compass', Int32, )
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__setpoint)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # for i in range(arr.__len__()):
        #     print(arr[i])


if __name__ == "__main__":
    n = NodePlay()

    while not rospy.is_shutdown():
        pass
