#! /usr/bin/python3
import math
import rospy
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from threading import Thread
from openbase.kinematics import DifferentialDrive

class RobotMockNode:
    pose:Pose2D = Pose2D()
    twist_msg:Twist = None
    sampling_time = 0.07
    def __encoder(self):
        move = DifferentialDrive(self.sampling_time, self.base_wheel)
        while(True):
            if(self.twist_msg is not None):
                vmx, vmy, dTh = move.get_odometry(self.twist_msg, self.pose.x, self.pose.y, self.pose.theta)
                self.pose.theta += dTh
                self.pose.x += vmx
                self.pose.y += vmy
                # self.pose.x += ((math.cos(self.pose.theta) * vmx) - (math.sin(self.pose.theta) * vmy))
                # self.pose.y += ((math.sin(self.pose.theta) * vmx) + (math.cos(self.pose.theta) * vmy))
                self.pose_publisher.publish(self.pose)
                goal = PoseStamped()
                goal.pose.position.x = self.pose.x
                goal.pose.position.y = self.pose.y

            rospy.sleep(self.sampling_time)

    def publish_encoder(self):
        while(True):
            try:
                rospy.sleep(0.1)
            except Exception as e:
                pass

    def on_twist(self, msg):
        self.twist_msg = msg

    def read_param(self):
        self.d_wheel_mm = rospy.get_param('~d_wheel')
        self.ppr = rospy.get_param('~ppr')
        self.base_wheel = rospy.get_param('~wheel_base')
        self.base_frame_id = rospy.get_param('~base_frame_id')

    def on_reset_pos(self, msg):
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0
    def listen(self):
        rospy.init_node('mock_robot', anonymous=True)
        rospy.loginfo('ROBOT SIMULATION STARTED')
        self.read_param()
        rospy.Subscriber(f'/{self.base_frame_id}/cmd_vel', Twist, callback=self.on_twist)
        rospy.Subscriber('/reset_pos', Empty, callback=self.on_reset_pos)
        self.pose_publisher = rospy.Publisher(f'/{self.base_frame_id}/pose_data', Pose2D, queue_size=10)
        Thread(target=self.__encoder, args=(), daemon=True).start()
        Thread(target=self.publish_encoder, args=(), daemon=True).start()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RobotMockNode()
        node.listen()

    except rospy.ROSInterruptException:
        pass