from enum import Enum
from math import atan2, pi, sqrt

from openbase.pid import Pid
import rospy
from geometry_msgs.msg import Point, Twist, Pose2D
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class Mode(Enum):
    DIRECT = 0
    PIVOT = 1


class NodeAutoDiffdrive:
    lin_pid = Pid(kp=0.3, ki=0, kd=1)
    ang_pid = Pid(kp=0.3, ki=0, kd=1)
    pose: Pose2D = Pose2D()
    pose_control_started = False
    marker_array_pos = 0
    marker: Marker = Marker()
    twist = Twist()
    mode = Mode.DIRECT

    def get_distance_goal_and_heading(self, goal_x, goal_y, pose_x, pose_y, ):
        diff_x = goal_x - pose_x
        diff_y = goal_y - pose_y
        goal_distance = sqrt((diff_y ** 2) + (diff_x ** 2))
        goal_heading = atan2(diff_x, diff_y)
        return goal_distance, goal_heading

    def on_marker_set(self, msg: Marker):
        self.marker = msg

    def on_robot_pose(self, pose: Pose2D):
        if self.mode == Mode.DIRECT:
            self.direct_mode(pose)
        if self.mode == Mode.PIVOT:
            self.pivot_mode(pose)

    def direct_mode(self, pose: Pose2D):
        if self.pose_control_started and self.marker_array_pos < self.marker.points.__len__():
            goal_x = self.marker.points[self.marker_array_pos].x
            goal_y = self.marker.points[self.marker_array_pos].y
            distance, heading = self.get_distance_goal_and_heading(
                goal_x, goal_y, pose.x, pose.y,)
            self.lin_pid.pos = distance
            self.ang_pid.sp = - heading
            self.ang_pid.pos = pose.theta
            self.twist.linear.x = self.lin_pid.compute_from_err(distance)
            self.twist.angular.z = self.ang_pid.compute()
            if abs(self.lin_pid.sp - self.lin_pid.pos) < 0.01 and abs(self.ang_pid.sp - self.ang_pid.pos) < 0.01:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.marker_array_pos += 1
                self.mode = Mode.PIVOT
                self.ang_pid.reset_err()
                self.lin_pid.reset_err()
            self.cmd_vel.publish(self.twist)

    def pivot_mode(self, pose: Pose2D):
        if self.pose_control_started and self.marker_array_pos < self.marker.points.__len__():
            goal_x = self.marker.points[self.marker_array_pos].x
            goal_y = self.marker.points[self.marker_array_pos].y
            distance, heading = self.get_distance_goal_and_heading(
                goal_x, goal_y, pose.x, pose.y,)
            self.lin_pid.pos = distance
            self.ang_pid.sp = - heading
            self.ang_pid.pos = pose.theta
            # self.twist.linear.x = self.lin_pid.compute_from_err(distance)
            self.twist.angular.z = self.ang_pid.compute()
            if abs(self.ang_pid.sp - self.ang_pid.pos) < 0.01:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                # self.marker_array_pos += 1
                self.mode = Mode.DIRECT
                self.ang_pid.reset_err()
                self.lin_pid.reset_err()
            self.cmd_vel.publish(self.twist)

    def __reset(self, _):
        self.ang_pid.reset_err()
        self.lin_pid.reset_err()
        self.pose_control_started = False
        self.marker_array_pos = 0

    def on_marker_follower(self, _):
        rospy.loginfo('ON MARKER FOLLOWER CALLED')
        self.pose_control_started = True
        self.mode = Mode.DIRECT

    def on_pivot_mode(self, _):
        rospy.loginfo('ON PIVOT MODE CALLED')
        self.mode = Mode.PIVOT
        self.marker_array_pos = 0
        self.pose_control_started = True

    def __init__(self) -> None:
        rospy.init_node('node_autonomous', anonymous=True)
        self.base_frame_id = rospy.get_param('~base_frame_id')
        rospy.Subscriber(f'/{self.base_frame_id}/pose_data',
                         Pose2D, self.on_robot_pose)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        rospy.Subscriber('/marker', Marker, self.on_marker_set)
        rospy.Subscriber('/marker_follower', Empty, self.on_marker_follower)
        rospy.Subscriber('/pivot_mode', Empty, self.on_pivot_mode)
        self.cmd_vel = rospy.Publisher(
            f'/{self.base_frame_id}/cmd_vel', Twist, queue_size=1)


if __name__ == "__main__":
    n = NodeAutoDiffdrive()
    rate = rospy.Rate(hz=20)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
