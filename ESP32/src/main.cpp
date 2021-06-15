#include "main.h"
void ICACHE_RAM_ATTR EN1_ISR()
{
  portENTER_CRITICAL(&mux);
  m1.isrHandler();
  portEXIT_CRITICAL(&mux);
}

void ICACHE_RAM_ATTR EN2_ISR()
{
  portENTER_CRITICAL(&mux);
  m2.isrHandler();
  portEXIT_CRITICAL(&mux);
}

void ICACHE_RAM_ATTR EN3_ISR()
{
  portENTER_CRITICAL(&mux);
  m3.isrHandler();
  portEXIT_CRITICAL(&mux);
}

void setPidCb(const geometry_msgs::Point &msg_data)
{
  m3.pid(msg_data.x, msg_data.y, msg_data.z, 1000);
  Serial.println(msg_data.x);
}

void velCb(const geometry_msgs::Twist &msg_data)
{
  vel_data = msg_data;
  sp_heading = heading;
}

void resetPositionCb(const std_msgs::Empty &msg_data)
{
  m1.encoder_tick_acc = 0;
  m2.encoder_tick_acc = 0;
  m3.encoder_tick_acc = 0;

  compass.read();
  reset_heading = compass.getAzimuth();;
  heading = 0;
}

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
  for (;;)
  {
    while (true)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      if (WiFi.softAPgetStationNum() > 0)
        break;
    }
    if (client.connected() != 1)
    {
      nh.initNode();
      nh.advertise(heading_pub);
      // nh.advertise(odom_pub);
      // nh.advertise(imu_pub);
      nh.advertise(encoder_pub);
      nh.subscribe(vel_sub);
      nh.subscribe(pid_sub);
      nh.subscribe(rst_pos_sub);
    }
    if (client.connected() == 1)
      nh.spinOnce();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void publishMessage(void *parameter)
{
  for (;;)
  {
    if (client.connected() == 1)
    {
      heading_pub.publish(&heading_data);
      // odom_pub.publish(&odom_data);
      encoder_pub.publish(&encoder_data);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void readCompass(void *parameters)
{
  compass.init();
  compass.read();
  last_compass_reading = compass.getAzimuth();
  long continuous = last_compass_reading;
  for (;;)
  {
    compass.read();
    int now = compass.getAzimuth();
    if (abs(now - last_compass_reading) > 300)
    {
      int offset;
      if (now - last_compass_reading < 0)
      {
        offset = (360 - last_compass_reading) + now;
        continuous = continuous + offset;
      }
      else
      {
        offset = (360 - now) + last_compass_reading;
        continuous = continuous - (last_compass_reading + offset);
      }
    }
    else
      continuous = continuous + (now - last_compass_reading);

    heading = continuous;
    last_compass_reading = now;
    heading_data.data = heading;
    Serial.println(heading);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void moveBase(void *parameters)
{
  for (;;)
  {
    if (WiFi.softAPgetStationNum() > 0)
    {
      float x = vel_data.linear.x;
      float y = vel_data.linear.y;
      float z = vel_data.linear.z;
      float ax = vel_data.angular.x;
      float ay = vel_data.angular.y;
      float az = vel_data.angular.z;
      base.setSpeed(x, y, z, ax, ay, az);
    }
    else
      base.setSpeed(0, 0, 0, 0, 0, 0);
    vTaskDelay(10);
  }
}

void countRpm(void *parameters)
{
  const int sampling_time_ms = 20;
  for (;;)
  {
    m1.calculateRpm(sampling_time_ms);
    m2.calculateRpm(sampling_time_ms);
    m3.calculateRpm(sampling_time_ms);
    // Serial.printf("m1 : %d m2 : %d m3 : %d\n", m1.rpm, m2.rpm, m3.rpm);
    vTaskDelay(sampling_time_ms / portTICK_PERIOD_MS);
  }
}

void readRpm(void *parameters)
{
  for (;;)
  {
    Serial.println("m1 : " + (String)m1.rpm + " m2 : " + (String)m2.rpm + " m3 : " + (String)m3.rpm);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void gainFromCompass(void *parameters)
{
  // float err, d_err, i_err, last_err;
  for (;;)
  {
    if (m1.rpm > 50 && m2.rpm > 50)
    {

      // belok ke arah m2
      if ((sp_heading - heading) > 10)
      {
        m2.correction = abs(sp_heading - heading) / 10 * 50;
        m1.correction = 0;
        Serial.println("tambah rpm m2");
      }

      // belok ke arah m1
      else if ((sp_heading - heading) < 10)
      {
        m1.correction = abs(sp_heading - heading) / 10 * 50;
        m2.correction = 0;
        Serial.println("tambah rpm m1");
      }
      else
      {
        Serial.println("konstan");
        m1.correction = 0;
        m2.correction = 0;
      }
    }
    // Serial.println(sp_heading);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void odometry(void *parameters)
{

  for (;;)
  {
    encoder_data.en_a = m1.encoder_tick_acc;
    encoder_data.en_b = m2.encoder_tick_acc;
    encoder_data.en_c = m3.encoder_tick_acc;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  analogWriteFrequency(10000);
  m1.pid(7, 0.05, 1, 255);
  m2.pid(7, 0.05, 1, 255);
  m3.pid(6, 0.05, 1, 255);
  base.setMotor(m1, m2, m3);
  attachInterrupt(digitalPinToInterrupt(m1.en_a), EN1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m2.en_a), EN2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m3.en_a), EN3_ISR, FALLING);
  // Spawn task RTOS
  // xTaskCreatePinnedToCore(fungsi, "nama fungsi", alokasi memori, prioritas, task handle, core);
  // ESP32 memiliki 3 core, yaitu core 0, core 1, dan ULP
  // Sebisa mungkin prioritas task disamakan untuk menghindari crash
  // Task yang paling sering dijalankan diberikan prioritas paling tinggi
  xTaskCreatePinnedToCore(wifiSetup, "wifi setup", 10000, NULL, 5, &wifi_task, 0); // Pengaturan akses poin
  xTaskCreatePinnedToCore(blinker, "blink", 1000, NULL, 1, &blink, 1); // Test apakah RTOS dapat berjalan
  xTaskCreatePinnedToCore(initNode, "node", 5000, NULL, 5, &ros_task, 0); // Inisialisasi ros node
  xTaskCreatePinnedToCore(publishMessage, "publisher", 10000, NULL, 2, &ros_pub, 1); // Task publish ros messsage
  xTaskCreatePinnedToCore(readCompass, "compass", 10000, NULL, 2, &cmp_task, 1); // Membaca sensor kompas
  // xTaskCreatePinnedToCore(gainFromCompass, "gain compass", 5000, NULL, 2, &cmp_task, 1);
  xTaskCreatePinnedToCore(moveBase, "base", 5000, NULL, 2, &motor_task, 1); // Menggerakkan base robot
  xTaskCreatePinnedToCore(countRpm, "rpm", 5000, NULL, 2, &rpm_task, 1); // Menghitung RPM
  xTaskCreatePinnedToCore(odometry, "odometry", 5000, NULL, 2, &odometry_task, 1); // Set data untuk message MotorEncoder
}
void loop()
{
  vTaskDelay(1);
}