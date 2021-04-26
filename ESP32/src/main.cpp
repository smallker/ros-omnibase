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
  nh.advertise(publisher);
  nh.advertise(heading_pub);
  for (;;)
  {
    if(client.connected() != 1) ESP.restart();
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
      message.data = "Hore";
      heading_data.data = heading;
      publisher.publish(&message);
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

void setup()
{
  Serial.begin(115200);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_A, OUTPUT);
  pinMode(M3_B, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  setupOta();

  xTaskCreatePinnedToCore(blinker, "blink", 1000, NULL, 1, &blink, 0);
  xTaskCreatePinnedToCore(initNode, "ros init node", 5000, NULL, 5, &ros_task, 0);
  xTaskCreatePinnedToCore(publishMessage, "ros publisher", 10000, NULL, 2, &ros_pub, 1);
  xTaskCreatePinnedToCore(readCompass, "read compass", 10000, NULL, 2, &cmp_task, 1);
}

void loop()
{
  ArduinoOTA.handle();
}