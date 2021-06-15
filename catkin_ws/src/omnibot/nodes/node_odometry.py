#! /usr/bin/python3
from __future__ import division

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int32
from omnibot import pose, odometry
from omnibot.msg import MotorEncoder

class OdometryNode:

    def __init__(self):
        self.m_encoder = MotorEncoder()
    def main(self):
        rospy.init_node('node_odometry')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        self.rate = float(rospy.get_param('~rate', 10.0))
        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_link')
        self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')
        self.d_wheel = rospy.get_param('~d_wheel', 0.06)
        self.base_wheel = rospy.get_param('~wheel_base', 0.11)
        self.ppr = rospy.get_param('~ppr', 900)
        self.odometry = odometry.Odometry(self.d_wheel, self.base_wheel, self.ppr)
        # self.odometry.set_time(rospy.get_time())
        rate = rospy.Rate(self.rate)
        self.nodeName = rospy.get_name()

        rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
                         self.on_initial_pose)
        rospy.Subscriber('motor_encoder', MotorEncoder, self.encoder_callback)
        rospy.Subscriber('/sensor/compass', Int32, self.compass_callback)

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def encoder_callback(self, msg:MotorEncoder):
        self.m_encoder = msg

    def compass_callback(self, msg:Int32):
        self.odometry.update_compass(msg.data)

    def publish(self):
        self.odometry.update_encoder(self.m_encoder)
        self.odometry.update_pose(rospy.get_time())
        now = rospy.get_rostime()
        pose = self.odometry.get_pose()
        q = quaternion_from_euler(0, 0, pose.theta)
        self.tf_pub.sendTransform(
            (pose.x, pose.y, 0),
            (q[0], q[1], q[2], q[3]),
            now,
            self.baseFrameID,
            self.odomFrameID
        )
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.angular.z = pose.thetaVel
        self.odom_pub.publish(odom)
        rospy.loginfo(f'y : {pose.y} x : {pose.x} w : {pose.theta}')

    def on_initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = pose.Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        rospy.loginfo('Setting initial pose to %s', pose)
        self.odometry.set_pose(pose)
if __name__ == '__main__':
    try:
        rospy.loginfo('running node odom')
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
