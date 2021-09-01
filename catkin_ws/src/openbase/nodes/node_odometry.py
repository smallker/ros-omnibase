#! /usr/bin/python3
from __future__ import division

import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D

class OdometryNode:
    seq = 0
    def main(self):
        rospy.init_node('node_odometry', anonymous=True)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_pub = TransformBroadcaster()
        self.rate = float(rospy.get_param('~rate', 20))
        self.base_frame_id = rospy.get_param('~base_frame_id')
        self.odom_frame_id = rospy.get_param('~odom_frame_id')
        rospy.Subscriber(f'/{self.base_frame_id}/pose_data', Pose2D, self.on_pose_data)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

    def on_pose_data(self, pose:Pose2D):
        now = rospy.get_rostime()

        q = quaternion_from_euler(0, 0, pose.theta)
        self.tf_pub.sendTransform(
            (pose.x, pose.y, pose.theta),
            (q[0], q[1], q[2], q[3]),
            now,
            self.base_frame_id,
            self.odom_frame_id
        )
        odom = Odometry()
        odom.header.stamp = now
        odom.header.seq = self.seq
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)
        self.seq += 1
        rospy.loginfo(f'{pose.x},{pose.y},{pose.theta}')
if __name__ == '__main__':
    try:
        rospy.loginfo('running node odom')
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
