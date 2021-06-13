from omnibot.msg import MotorEncoder
import math

from omnibot.pose import Pose

sqrt3 = 1.732050807568877193176604123436845839023590087890625

class Odometry:
    def __init__(self, d_wheel, base_wheel, ppr) -> None:
        self.pose = Pose()
        self.last_pose = Pose()
        self.last_time = 0
        self.d_wheel = d_wheel
        self.base_wheel = base_wheel
        self.ppr = ppr
        self.m_a:float = 0.0
        self.m_b:float = 0.0
        self.m_c:float = 0.0

    def set_time(self, time):
        self.last_time = time

    def update_encoder(self, en_msg:MotorEncoder):
        self.m_a = (en_msg.en_a / self.ppr) * math.pi * self.d_wheel
        self.m_b = (en_msg.en_b / self.ppr) * math.pi * self.d_wheel
        self.m_c = (en_msg.en_c / self.ppr) * math.pi * self.d_wheel

    def update_pose(self, new_time):
        delta_time = new_time - self.last_time
        # self.pose.y = (sqrt3 * self.m_a) - (sqrt3 * self.m_b)
        # self.pose.x = -1*((2 * self.m_c) - self.m_a - self.m_b)
        self.pose.y = ((sqrt3 * self.m_b) - (sqrt3 * self.m_a))/ -3
        self.pose.x = ((2*self.m_c) - self.m_a - self.m_b)/ -3
        self.pose.theta = -1 * ((self.m_a + self.m_c + self.m_b) / (self.base_wheel * 3))
        self.pose.xVel = abs((self.pose.x - self.last_pose.x) / delta_time)
        self.pose.yVel = abs((self.pose.y - self.last_pose.y)/ delta_time)
        self.pose.thetaVel = abs((self.pose.theta - self.last_pose.thetaVel)/delta_time)
        self.last_pose.y = self.pose.y
        self.last_pose.x = self.pose.x
        self.last_pose.theta = self.pose.theta
        self.last_time = new_time
        
    def get_pose(self) -> Pose:
        return self.pose

    def set_pose(self, pose):
        self.pose = pose