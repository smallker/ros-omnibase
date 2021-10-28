#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

// Atur waktu tunda pengiriman data ke ROS
#define PUBLISH_DELAY_MS 50

// Uncomment untuk menggunakan ROS serial
// Comment jika ingin menggunakan ROS TCP WiFi
// #define ROS_USE_SERIAL
/*
    Karena perangkat lunak yang digunakan mendukung OTA,
    untuk pengaturan awal dapat dengan mengkoneksikan PC
    ke akses poin robot. Lalu ganti alamat IP dengan IP
    PC yang terinstal ROS. Setelah itu upload ulang perangkat
    lunak ke ESP32 melalui wifi
*/
IPAddress server(192, 168, 43, 101); // IP PC yang terinstal ROS
IPAddress ip_address;
int status = WL_IDLE_STATUS;

/*
    Menggunakan koneksi socket wifi sebagai
    pengganti protokol serial. Keduanya mewarisi
    kelas Stream sehingga memiliki method
    yang sama. Socket harus diatur dalam mode asinkron
    dengan perintah fcntl(client, F_SETFL, O_NONBLOCK);
*/
WiFiClient client;
class WiFiHardware
{

public:
    WiFiHardware(){};

    void init()
    {
        fcntl(client, F_SETFL, O_NONBLOCK);
        client.connect(server, 11411);
    }
    int read()
    {
        return client.read();
    }
    void write(uint8_t *data, int length)
    {
        for (int i = 0; i < length; i++)
            client.write(data[i]);
    }
    unsigned long time()
    {
        return millis();
    }
};

/*
    Inisialisasi variable ros message
    - std_msgs berisi tipe data primitif (int, float, bool, dll)
    - twist merupakan tipe data untuk mengirim data kecepatan robot
    - point merupakan tipe data untuk mengirim koordinat robot dalam x,y,z
    - pose2d merupakan tipe data untuk mengirim koordinat robot dalam x,y, theta
*/
geometry_msgs::Twist vel_data;
geometry_msgs::Pose2D pose_data;
visualization_msgs::Marker marker_data;

// Inisialisasi ros node
#if defined(ROS_USE_SERIAL)
ros::NodeHandle nh;
#else
ros::NodeHandle_ <WiFiHardware> nh;
#endif
// Inisialisasi ros publisher
ros::Publisher pose_pub("/real/pose_data", &pose_data);

// Inisialisasi fungsi callback subscriber
void onCmdVel(const geometry_msgs::Twist &msg_data);
void onResetPose(const std_msgs::Empty &mgs_data);
void onMarkerSet(const visualization_msgs::Marker &msg_data);
void onMarkerFollower(const std_msgs::Empty &msg_data);

// Inisialisasi ros subscriber
ros::Subscriber<geometry_msgs::Twist> vel_sub("/real/cmd_vel", onCmdVel);
ros::Subscriber<std_msgs::Empty> rst_pos_sub("/reset_pos", onResetPose);
ros::Subscriber<visualization_msgs::Marker> marker_sub("/marker", onMarkerSet);
ros::Subscriber<std_msgs::Empty> marker_follower_sub("/marker_follower", onMarkerFollower);