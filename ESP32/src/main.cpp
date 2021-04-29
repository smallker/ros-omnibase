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
  // Inisialisasi ros node
  // Mendaftarkan publisher dan subscriber
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

  // Spawn task RTOS
  // xTaskCreatePinnedToCore(fungsi, "nama fungsi", alokasi memori, prioritas, task handle, core);
  // ESP32 memiliki 3 core, yaitu core 0, core 1, dan ULP
  // Sebisa mungkin prioritas task disamakan untuk menghindari crash
  // Task yang paling sering dijalankan diberikan prioritas paling tinggi
  xTaskCreatePinnedToCore(blinker, "blink", 1000, NULL, 1, &blink, 0);
  xTaskCreatePinnedToCore(initNode, "node", 5000, NULL, 5, &ros_task, 0);
  xTaskCreatePinnedToCore(publishMessage, "publisher", 10000, NULL, 2, &ros_pub, 1);
  xTaskCreatePinnedToCore(readCompass, "compass", 10000, NULL, 2, &cmp_task, 1);
  xTaskCreatePinnedToCore(moveBase, "base", 1000, NULL, 2, &motor_task, 1);
}
void loop()
{
  ArduinoOTA.handle();
}