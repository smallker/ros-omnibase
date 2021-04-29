#include "main.h"

void blinker(void *parameters)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    vTaskDelay(500);
  }
}

void initNode(void *parameters)
{
  nh.initNode();
  nh.advertise(heading_pub);
  nh.subscribe(vel_sub);
  for (;;)
  {
    if (client.connected() != 1)
      ESP.restart();
    nh.spinOnce();
    vTaskDelay(100);
  }
}

void publishMessage(void *parameter)
{
  for (;;)
  {
    if (client.connected() == 1)
    {
      heading_data.data = heading;
      heading_pub.publish(&heading_data);
    }
    vTaskDelay(100);
  }
}

void readCompass(void *parameters)
{
  QMC5883LCompass compass;
  compass.init();
  for (;;)
  {
    compass.read();
    heading = compass.getAzimuth();
    vTaskDelay(100);
  }
}

void moveBase(void *parameters)
{

  for (;;)
  {
    float x = vel_data.linear.x;
    float y = vel_data.linear.y;
    float z = vel_data.linear.z;
    float ax = vel_data.angular.x;
    float ay = vel_data.angular.y;
    float az = vel_data.angular.z;
    base.setSpeed(x, y, z, ax, ay, az);
    vTaskDelay(10);
  }
}
void velCallback(const geometry_msgs::Twist &msg_data)
{
  vel_data = msg_data;
}

void setup()
{
  Serial.begin(115200);
  base.setMotor(m1, m2, m3);
  setupOta();
  xTaskCreatePinnedToCore(blinker, "blink", 1000, NULL, 1, &blink, 0);
  xTaskCreatePinnedToCore(initNode, "ros init node", 5000, NULL, 5, &ros_task, 0);
  xTaskCreatePinnedToCore(publishMessage, "ros publisher", 10000, NULL, 2, &ros_pub, 1);
  xTaskCreatePinnedToCore(readCompass, "read compass", 10000, NULL, 2, &cmp_task, 1);
  xTaskCreatePinnedToCore(moveBase, "move base", 1000, NULL, 2, &motor_task, 1);
}
void loop()
{
  ArduinoOTA.handle();
}