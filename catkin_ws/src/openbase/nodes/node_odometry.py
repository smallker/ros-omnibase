#! /usr/bin/python3
from __future__ import division

import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker
class OdometryNode:
    seq = 0
    def main(self):
        rospy.init_node('node_odometry', anonymous=True)
        self.tf_pub = TransformBroadcaster()
        self.rate = float(rospy.get_param('~rate', 20))
        self.base_frame_id = rospy.get_param('~base_frame_id')
        self.odom_frame_id = rospy.get_param('~odom_frame_id')
        self.odom_pub = rospy.Publisher(f'/{self.base_frame_id}/odom', Odometry, queue_size=10)
        self.marker_pub = rospy.Publisher(f'/{self.base_frame_id}/marker', Marker, queue_size=10)
        rospy.Subscriber(f'/{self.base_frame_id}/pose_data', Pose2D, self.on_pose_data)
        rospy.Subscriber('/reset_pos', Empty, self.on_reset_pos)
        rate = rospy.Rate(self.rate)
        self.marker_setting()
        while not rospy.is_shutdown():
            rate.sleep()
    
    def on_reset_pos(self, msg):
        self.marker.points.clear()
        self.marker_pub.publish(self.marker)

    def marker_setting(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # self.marker scale
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        # self.marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0

        # self.marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.points = []
        line_point = Point()
        line_point.x = 0
        line_point.y = 0
        self.marker.points.append(line_point)

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
        line_point = Point()
        line_point.x = pose.x
        line_point.y = pose.y
        self.marker.points.append(line_point)
        self.marker_pub.publish(self.marker)
        rospy.loginfo(f'{pose.x},{pose.y},{pose.theta}')
if __name__ == '__main__':
    try:
        rospy.loginfo('running node odom')
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
