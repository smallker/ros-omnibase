#! /usr/bin/python3
from __future__ import division

import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int32
from openbase.msg import MotorEncoder, OmnibaseOdom

class OdometryNode:

    def __init__(self):
        self.m_encoder = MotorEncoder()
    def main(self):
        rospy.init_node('node_odometry')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        self.rate = float(rospy.get_param('~rate', 150.0))
        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_link')
        self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')
        rate = rospy.Rate(self.rate)
        self.nodeName = rospy.get_name()        
        while not rospy.is_shutdown():
            rate.sleep()

    def on_odometry(self, pose:OmnibaseOdom):
        now = rospy.get_rostime()
        q = quaternion_from_euler(0, 0, pose.w)
        self.tf_pub.sendTransform(
            (pose.x, pose.y, pose.w),
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
        self.odom_pub.publish(odom)

    def publish(self):
        now = rospy.get_rostime()
        self.odometry.update_encoder(self.m_encoder)
        self.odometry.update_pose(now.to_time())
        pose = self.odometry.get_pose()
        q = quaternion_from_euler(0, 0, pose.theta)
        self.tf_pub.sendTransform(
            (pose.x, pose.y, pose.theta),
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
        odom.twist.twist.linear.y = pose.yVel
        odom.twist.twist.angular.z = pose.thetaVel
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        rospy.loginfo('running node odom')
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
