from geometry_msgs.msg import Twist
from omnibot.msg import MotorSpeed
class Kinematics:
    def __init__(self) -> None:
        pass
    
    def set_speed(self, speed:Twist):
        motor = MotorSpeed()
        x = speed.linear.x
        y = speed.linear.y
        w = speed.angular.z
        motor.a = (0.58 * x) + (-0.33 * y) + (0.33 * w)
        motor.b = (-0.58 * x) + (-0.33 * y) + (0.33 * w)
        motor.c = (0 * x) + (0.67 * y) + (0.33 * w)
        return motor