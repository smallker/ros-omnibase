from omnibot.msg import MotorEncoder
import math

from omnibot.pose import Pose

class Odometry:
    def __init__(self, d_wheel, base_wheel, ppr) -> None:
        self.pose = Pose()
        self.last_pose = Pose()
        self.last_time = 0
        self.d_wheel = d_wheel
        self.base_wheel = base_wheel
        self.ppr = ppr

    def update_encoder(self, en_msg:MotorEncoder):
        self.en_a = en_msg.en_a
        self.en_b = en_msg.en_b
        self.en_c = en_msg.en_c

    def update_pose(self, new_time):
        delta_time = new_time - self.last_time
        self.pose.y = 3 * math.pi ((0.33 * self.en_a) - (0.33 * self.en_b) - (0.66 * self.en_c))
        self.pose.x = 1.7 * math.pi ((0.33 * self.en_c) + (0.58 * self.en_b))
        self.pose.theta = (self.d_wheel/(6 * self.base_wheel)) * 2 * math.pi * (self.en_a + self.en_b + self.en_c)
        self.pose.xVel = abs((self.pose.x - self.last_pose.x) / delta_time)
        self.pose.yVel = abs((self.pose.y - self.last_pose.y)/ delta_time)
        self.pose.thetaVel = abs((self.pose.theta - self.last_pose.thetaVel)/delta_time)
        self.last_pose.y = self.pose.y
        self.last_pose.x = self.pose.x
        self.last_pose.theta = self.pose.theta
        self.last_time = new_time
    
    def get_pose(self) -> Pose:
        return self.pose