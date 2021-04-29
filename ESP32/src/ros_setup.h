#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

IPAddress server(192, 168, 43, 44); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

class WiFiHardware
{

public:
    WiFiHardware(){};

    void init()
    {
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

std_msgs::Int32 heading_data;
geometry_msgs::Twist vel_data;
ros::NodeHandle_<WiFiHardware> nh;
ros::Publisher heading_pub("heading", &heading_data);

void velCallback(const geometry_msgs::Twist &msg_data);
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCallback);