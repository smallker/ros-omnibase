import math
from geometry_msgs.msg import Twist

from abc import ABC, abstractclassmethod, abstractmethod

class Kinematics(ABC):    

    @abstractmethod    
    def __init__(self) -> None:
        pass

    @abstractmethod
    def set_speed(self, speed:Twist, heading:float):
        pass
    
    
class OmniBaseY(Kinematics):

    def __init__(self, sampling_time, base_wheel) -> None:
        self.sampling_time = sampling_time
        self.base_wheel = base_wheel

    def set_speed(self, speed: Twist, heading: float):
        x = speed.linear.x
        y = speed.linear.y
        w = speed.angular.z
        v1 = (0.58 * x) + (-0.33 * y) + (0.33 * w)
        v2 = (0 * x) + (0.67 * y) + (0.33 * w)
        v3 = (-0.58 * x) + (-0.33 * y) + (0.33 * w)
        vmx = (2 * v2 - v1 - v3) / 3 # Sampling kecepatan x
        vmy = ((math.sqrt(3) * v3) - (math.sqrt(3) * v1)) / 3 # Sampling kecepatan y
        dTh = ((v1 + v2+ v3) / (self.base_wheel * 3)) / (1 / self.sampling_time)
        deltaX = - ((math.cos(heading) * vmx) - (math.sin(heading) * vmy))
        deltaY = - ((math.sin(heading) * vmx) + (math.cos(heading) * vmy))
        deltaX = deltaX / (1 / self.sampling_time)
        deltaY = deltaY / (1 / self.sampling_time)
        return deltaX, deltaY, dTh

class DifferentialDrive(Kinematics):
    radius = 0.03
    theta = 0
    def __init__(self, sampling_time, base_wheel) -> None:
        self.sampling_time = sampling_time
        self.base_wheel = base_wheel

    def set_speed(self, speed: Twist, heading: float):
        # Koreksi heading
        heading = (((heading * 180/math.pi)+90)) * math.pi/180
        rightTravel = (speed.linear.x + (speed.angular.z * 0.1)) / (1/self.sampling_time)
        leftTravel = (speed.linear.x - (speed.angular.z * 0.1))/ (1/self.sampling_time)
        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / (self.base_wheel)
        deltaX = deltaTravel * math.cos(heading)
        deltaY = deltaTravel * math.sin(heading)
        return deltaX, deltaY, deltaTheta
