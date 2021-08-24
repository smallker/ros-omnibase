#! /usr/bin/python3
import math
import rospy
from geometry_msgs.msg import Twist, Pose2D
from threading import Thread
from openbase.kinematics import Kinematics

class RobotMockNode:
    pose:Pose2D = Pose2D()
    twist_msg:Twist = None
    sampling_time = 0.1
    def __encoder(self):
        move = Kinematics()
        w = 0.00
        while(True):
            if(self.twist_msg is not None):
                v1, v2, v3 = move.set_speed(self.twist_msg)
                vmx = (2 * v2 - v1 - v3) / 3 # Sampling kecepatan x
                vmy = ((math.sqrt(3) * v3) - (math.sqrt(3) * v1)) / 3 # Sampling kecepatan y
                w += ((v1 + v2+ v3) / (self.base_wheel * 3)) / (1 / self.sampling_time) # Dalam radian
                self.pose.x -= ((math.cos(w) * vmx) - (math.sin(w) * vmy)) / (1 / self.sampling_time)# x (meter)
                self.pose.y -= ((math.sin(w) * vmx) + (math.cos(w) * vmy)) / (1 / self.sampling_time)# y (meter)
                self.pose.theta = w
                self.pose_publisher.publish(self.pose)
            rospy.sleep(self.sampling_time)

    def publish_encoder(self):
        while(True):
            try:
                rospy.sleep(0.1)
            except Exception as e:
                pass

    def callback(self, msg):
        self.twist_msg = msg

    def read_param(self):
        self.d_wheel_mm = rospy.get_param('~d_wheel')
        self.ppr = rospy.get_param('~ppr')
        self.base_wheel = rospy.get_param('~wheel_base')

    def listen(self):
        rospy.init_node('mock_robot', anonymous=True)
        rospy.loginfo('ROBOT SIMULATION STARTED')
        self.read_param()
        rospy.Subscriber('/cmd_vel', Twist, callback=self.callback)
        self.pose_publisher = rospy.Publisher('/pose_data', Pose2D, queue_size=10)
        Thread(target=self.__encoder, args=(), daemon=True).start()
        Thread(target=self.publish_encoder, args=(), daemon=True).start()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RobotMockNode()
        node.listen()

    except rospy.ROSInterruptException:
        pass