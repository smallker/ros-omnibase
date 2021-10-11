from math import atan2, pi, sqrt

from openbase.pid import Pid
import rospy
from geometry_msgs.msg import Point, Twist, Pose2D
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class NodeAutoDiffdrive:
    lin_pid = Pid(kp=0.3, ki=0, kd=1)
    ang_pid = Pid(kp=0.3, ki=0, kd=1)
    pose: Pose2D = Pose2D()
    pose_control_started = False
    marker: Marker = Marker()
    twist = Twist()

    def get_distance_goal_and_heading(self, goal_x, goal_y, pose_x, pose_y, ):
        diff_x = goal_x - pose_x
        diff_y = goal_y - pose_y
        goal_distance = sqrt((diff_y ** 2) + (diff_x **2))
        goal_heading = atan2(diff_y, diff_x)
        return goal_distance, goal_heading

    def on_marker_set(self, msg: Marker):
        self.marker = msg

    def on_robot_pose(self, pose: Pose2D):

        if self.pose_control_started:
            goal_x = self.marker.points[1].x
            goal_y = self.marker.points[1].y
            distance, heading = self.get_distance_goal_and_heading(
                 goal_y, goal_x, pose.y, pose.x)
            self.lin_pid.pos = distance
            self.ang_pid.sp = - heading
            self.ang_pid.pos = pose.theta
            self.twist.linear.x = self.lin_pid.compute_from_err(distance)
            self.twist.angular.z = self.ang_pid.pid()
            if abs(self.lin_pid.sp - self.lin_pid.pos) <0.01 and abs(self.ang_pid.sp - self.ang_pid.pos) < 0.01:
                self.pose_control_started = False
                rospy.Publisher('/log', String, queue_size=1).publish('FINISH')
                self.twist.linear.x = 0
                self.twist.angular.z = 0
            self.cmd_vel.publish(self.twist)

    def __reset(self, msg):
        self.pose_control_started = False
        self.ang_pid.reset_err()
        self.lin_pid.reset_err()

    def on_marker_follower(self, msg):
        rospy.loginfo('ON MARKER FOLLOWER CALLED')
        goal_x = self.marker.points[1].x
        goal_y = self.marker.points[1].y
        goal_distance, goal_heading = self.get_distance_goal_and_heading(
            goal_y, goal_x, 0, 0, )
        self.lin_pid.sp = goal_distance
        self.ang_pid.sp = - goal_heading
        rospy.loginfo(f'{goal_x} , {goal_y}')
        self.pose_control_started = True
        rospy.loginfo(f'{self.lin_pid.sp} {self.ang_pid.sp}')
        rospy.loginfo(self.marker.points)

    def __init__(self) -> None:
        rospy.init_node('node_autonomous', anonymous=True)
        self.base_frame_id = rospy.get_param('~base_frame_id')
        rospy.Subscriber(f'/{self.base_frame_id}/pose_data',
                         Pose2D, self.on_robot_pose)
        rospy.Subscriber('/reset_pos', Empty, self.__reset)
        rospy.Subscriber('/marker', Marker, self.on_marker_set)
        rospy.Subscriber('/marker_follower', Empty, self.on_marker_follower)
        self.cmd_vel = rospy.Publisher(
            f'/{self.base_frame_id}/cmd_vel', Twist, queue_size=1)


if __name__ == "__main__":
    n = NodeAutoDiffdrive()
    rate = rospy.Rate(hz=20)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
