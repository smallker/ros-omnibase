import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Empty

class NodeMarker:

    '''
    Inisialisasi node marker
    '''
    def __init__(self) -> None:
        rospy.init_node('node_marker')
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_clicked_point)
        rospy.Subscriber('/reset_pos', Empty, self.on_reset_pos)
        rospy.Subscriber('/marker_follower', Empty, self.on_marker_follower)
        self.publisher = rospy.Publisher('/marker', Marker, queue_size=10)
        self.marker_setting()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    '''
    Berfungsi untuk mengubah warna marker menjadi merah
    ketika kendali posisi dimulai
    '''
    def on_marker_follower(self, msg):
        # self.marker color
        self.marker.color.a = 0.7
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.publisher.publish(self.marker)

    '''
    Membuat marker baru ketika operator memilih
    titik pada peta
    '''
    def on_clicked_point(self, msg:PoseStamped):
        line_point = Point()
        line_point.x = msg.pose.position.x
        line_point.y = msg.pose.position.y
        self.marker.points.append(line_point)
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.publisher.publish(self.marker)
    

    '''
    Pengaturan warna marker
    LINE_STRIP untuk tipe marker garis
    warna diatur ke warna kuning saat pengguna
    memilih titik di peta
    '''
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
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

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
    
    '''
    Saat fungsi dipanggil maka akan menghapus semua
    marker yang ada di peta
    '''
    def on_reset_pos(self, msg):
        self.marker.points = []
        line_point = Point()
        line_point.x = 0
        line_point.y = 0
        self.marker.points.append(line_point)
        self.publisher.publish(self.marker)

if __name__=='__main__':
    try:
        marker = NodeMarker()
    except rospy.ROSInitException:
        pass
    