from geometry_msgs.msg import Twist


class Kinematics:
    def __init__(self) -> None:
        pass

    def set_speed(self, speed: Twist):
        x = speed.linear.x
        y = speed.linear.y
        w = speed.angular.z
        v1 = (0.58 * x) + (-0.33 * y) + (0.33 * w)
        v2 = (0 * x) + (0.67 * y) + (0.33 * w)
        v3 = (-0.58 * x) + (-0.33 * y) + (0.33 * w)
        return v1, v2, v3
