import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_about_axis
import numpy as np

class NodeImu:
    def __parser(self, msg:String):
        data = msg.data.split(',')
        imu = Imu()
        accel_x = float(data[0])
        accel_y = float(data[1])
        accel_z = float(data[2])
        gyro_x  = float(data[3])
        gyro_y  = float(data[4])
        gyro_z  = float(data[5])
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        o = imu.orientation
        o.x, o.y, o.z, o.z = orientation
        imu.linear_acceleration.x = accel_x
        imu.linear_acceleration.y = accel_y
        imu.linear_acceleration.z = accel_z

        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = gyro_y
        imu.angular_velocity.z = gyro_z

        imu.header.frame_id = "imu"
        imu.header.stamp = rospy.Time.now()
        self.pub_imu.publish(imu)

    def __init__(self) -> None:
        rospy.init_node('node_imu')
        rospy.Subscriber('/string_imu', String, self.__parser)
        rospy.loginfo('node imu started')
        self.pub_imu = rospy.Publisher('/sensor/imu', Imu, queue_size=1)

if __name__ == "__main__":
    n = NodeImu()
    while not rospy.is_shutdown():
        rospy.spin()
    