#! /usr/bin/python3
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from threading import Thread
from omnibot.kinematics import Kinematics
from omnibot.msg import MotorSpeed, MotorEncoder

class RobotMockNode:

    m_speed:MotorSpeed = MotorSpeed()
    m_enc:MotorEncoder = MotorEncoder()
    twist_msg:Twist = None
    cmp:Int32 = Int32()
    def __init__(self) -> None:
        self.cmp.data = 0
        pass
    
    def __encoder(self):
        move = Kinematics()
        while(True):
            if(self.twist_msg is not None):
                self.m_speed = move.set_speed(self.twist_msg)
                self.m_enc.en_a += int((self.m_speed.a/2 * math.pi) * self.ppr / 10)
                self.m_enc.en_b += int((self.m_speed.b/2 * math.pi) * self.ppr / 10)
                self.m_enc.en_c += int((self.m_speed.c/2 * math.pi) * self.ppr / 10)
                self.cmp.data = int((math.pi/180)*((self.m_enc.en_a + self.m_enc.en_b + self.m_enc.en_c) / (self.base_wheel * 3)))
            rospy.sleep(0.1)

    def publish_encoder(self):
        pub_speed = rospy.Publisher('motor_speed', MotorSpeed, queue_size=1)
        pub_encoder = rospy.Publisher('motor_encoder', MotorEncoder, queue_size=1)
        pub_cmp = rospy.Publisher('/sensor/compass', Int32, queue_size=1)
        while(True):
            try:
                rospy.sleep(0.1)
                pub_speed.publish(self.m_speed)
                pub_encoder.publish(self.m_enc)
                pub_cmp.publish(self.cmp)
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
        rospy.loginfo('start mock robot')
        self.read_param()
        rospy.Subscriber('/cmd_vel', Twist, callback=self.callback)
        Thread(target=self.__encoder, args=(), daemon=True).start()
        Thread(target=self.publish_encoder, args=(), daemon=True).start()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RobotMockNode()
        node.listen()

    except rospy.ROSInterruptException:
        pass