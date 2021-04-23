#! /usr/bin/python3
import rospy
from std_msgs import msg
from geometry_msgs.msg import Twist
from threading import Thread
from omnibot.kinematics import Kinematics
from omnibot.msg import MotorSpeed

class RobotMockNode:

    m_speed:MotorSpeed = MotorSpeed()
    twist_msg:Twist = None

    def __init__(self) -> None:
        pass
    
    def __encoder(self):
        move = Kinematics()
        while(True):
            if(self.twist_msg is not None):
                self.m_speed = move.set_speed(self.twist_msg)
            rospy.sleep(0.1)

    def publish_encoder(self):
        publisher = rospy.Publisher('motor_speed', MotorSpeed, queue_size=1)
        while(True):
            try:
                rospy.sleep(0.1)
                publisher.publish(self.m_speed)
                # rospy.loginfo(self.m_speed)
            except:
                pass

    def callback(self, msg):
        self.twist_msg = msg

    def listen(self):
        rospy.init_node('mock_robot', anonymous=True)
        rospy.Subscriber('cmd_vel', Twist, callback=self.callback)
        rospy.loginfo('start mock robot')
        Thread(target=self.__encoder, args=(), daemon=True).start()
        Thread(target=self.publish_encoder, args=(), daemon=True).start()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RobotMockNode()
        node.listen()

    except rospy.ROSInterruptException:
        pass