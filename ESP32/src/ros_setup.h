#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>

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

std_msgs::String message;

ros::NodeHandle_<WiFiHardware> nh;
ros::Publisher publisher("arduino", &message);
