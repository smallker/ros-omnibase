#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#define PUBLISH_DELAY_MS 50

IPAddress server(192, 168, 43, 101); // IP PC yang terinstal ROS
IPAddress ip_address;
int status = WL_IDLE_STATUS;

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

// Inisialisasi variable ros message
std_msgs::Int32 heading_data;
geometry_msgs::Twist vel_data;
geometry_msgs::Point pid;
geometry_msgs::Pose2D pose_data;
// Inisialisasi ros node
ros::NodeHandle_<WiFiHardware> nh;

// Inisialisasi ros publisher
ros::Publisher pose_pub("/real/pose_data", &pose_data);

// Inisialisasi fungsi callback subscriber
void onCmdVel(const geometry_msgs::Twist &msg_data);
void onSetMotorPid(const geometry_msgs::Point &msg_data);
void onMoveBaseToGoal(const geometry_msgs::PoseStamped &msg_data);
void onResetPose(const std_msgs::Empty &mgs_data);
void onGoalXPid(const geometry_msgs::Point &msg_data);
void onGoalYPid(const geometry_msgs::Point &msg_data);
void onGoalWPid(const geometry_msgs::Point &msg_data);
// Inisialisasi ros subscriber
ros::Subscriber<geometry_msgs::Twist> vel_sub("/real/cmd_vel", onCmdVel);
ros::Subscriber<geometry_msgs::Point> pid_sub("/pid", onSetMotorPid);
ros::Subscriber<geometry_msgs::PoseStamped> goal_sub("/move_base_simple/goal", onMoveBaseToGoal);
ros::Subscriber<std_msgs::Empty> rst_pos_sub("/reset_pos", onResetPose);
ros::Subscriber<geometry_msgs::Point> goal_x_pid_sub("/goal_x_pid", onGoalXPid);
ros::Subscriber<geometry_msgs::Point> goal_y_pid_sub("/goal_y_pid", onGoalYPid);
ros::Subscriber<geometry_msgs::Point> goal_w_pid_sub("/goal_w_pid", onGoalWPid);
