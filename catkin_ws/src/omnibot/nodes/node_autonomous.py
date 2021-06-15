import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from omnibot.pid import Pid


class NodePlay:
    # f = open('log.txt')
    position = 0
    finish = False
    pid_x = Pid(0.01, 0.0001, 0)
    pid_y = Pid(0.01, 0.0001, 0)
    pid_z = Pid(0.01, 0.0001, 0)

    def callback(self, msg_data: Odometry):
        self.pid_x.pos = msg_data.pose.pose.position.x
        self.pid_y.pos = msg_data.pose.pose.position.y
        self.pid_z.pos = msg_data.pose.pose.orientation.z

        if self.finish is not True:
            twist = Twist()
            twist.linear.x = self.pid_y.pid()
            twist.linear.y = - self.pid_x.pid()
            # twist.angular.z = self.pid_z.pid()
            self.cmd_vel.publish(twist)

        if abs(self.pid_x.sp - self.pid_x.pos) < 0.01 and abs(self.pid_y.sp - self.pid_y.pos) < 0.01:
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

    def __init__(self) -> None:
        # self.arr = self.f.readlines()
        self.arr = [[1, 0, 1], [0, 0, 1], [0, 1, 1], [0, 0, 1]]
        # self.arr = [[0,1,1], [0,0,1]]
        self.pid_x.sp = self.arr[self.position][0]
        self.pid_y.sp = self.arr[self.position][1]
        self.pid_z.sp = self.arr[self.position][2]
        rospy.init_node('node_autonomous')
        rospy.loginfo('node autonomous started')
        self.odom = rospy.Subscriber('odom', Odometry, self.callback)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # for i in range(arr.__len__()):
        #     print(arr[i])


if __name__ == "__main__":
    # n = NodePlay()

    while not rospy.is_shutdown():
        pass
