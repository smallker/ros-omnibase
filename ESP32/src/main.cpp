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

void onCmdVel(const geometry_msgs::Twist &msg_data)
{
  vel_data = msg_data;
  sp_heading = heading;
  last_command_time = millis();
}

void onResetPose(const std_msgs::Empty &msg_data)
{
  m1.encoder_tick_acc = 0;
  m2.encoder_tick_acc = 0;
  m3.encoder_tick_acc = 0;
  heading = 0;
  base.x = 0;
  base.y = 0;
  goal_x.setpoint = 0;
  goal_y.setpoint = 0;
  goal_w.setpoint = 0;
  goal_x.reset();
  goal_y.reset();
  goal_w.reset();
  pose_control_begin = false;
}

void onMoveBaseToGoal(const geometry_msgs::PoseStamped &msg_data)
{
  goal_x.setpoint = msg_data.pose.position.x;
  goal_y.setpoint = msg_data.pose.position.y;
  goal_w.setpoint = 0;
  pose_control_begin = true;
}

void onMarkerSet(const visualization_msgs::Marker &msg_data)
{
  marker_data = msg_data;
}

void onMarkerFollower(const std_msgs::Empty &msg_data)
{
  DEBUG.println("MARKER FOLLOWER STARTED");
  finish = false;
  for (int i = 0; i < marker_data.points_length; i++)
  {
    DEBUG.printf("X : %.f Y : %.f Z : %.f\n", marker_data.points[i].x, marker_data.points[i].y, marker_data.points[i].z);
  }
}
/*
  LED akan berkedip setiap 300ms saat robot
  belum tersambung dan berkedip setiap 2s
  ketika robot tersambung ke ROS
*/
void blink(void *parameters)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    vTaskDelay(is_ros_ready ? 2000 / portTICK_PERIOD_MS : 300 / portTICK_PERIOD_MS);
  }
}

/*
  Inisialisasi ros node
  Mendaftarkan publisher dan subscriber
*/
void initNode(void *parameters)
{
  for (;;)
  {
    while (true)
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      if (WiFi.softAPgetStationNum() > 0)
        break;
      else
        is_ros_ready = false;
    }
    if (rosClient.connected() != 1)
    {
      nh.initNode();
      nh.subscribe(vel_sub);
      nh.subscribe(rst_pos_sub);
      // nh.subscribe(goal_sub);
      nh.subscribe(marker_sub);
      nh.subscribe(marker_follower_sub);
      nh.advertise(pose_pub);
      heading = 0;
    }
    if (rosClient.connected() == 1)
    {
      nh.spinOnce();
    }
    vTaskDelay(PUBLISH_DELAY_MS / portTICK_PERIOD_MS);
  }
}

/*
  Mempublish message pada topik ROS
  - heading_pub untuk mengirim data kompas
  - encoder_pub untuk mengirim data rotary encoder
*/
void publishMessage(void *parameter)
{
  for (;;)
  {
    if (rosClient.connected() == 1)
    {
      is_ros_ready = true;
      pose_pub.publish(&pose_data);
    }
    vTaskDelay(PUBLISH_DELAY_MS / portTICK_PERIOD_MS);
  }
}

/*
  Data kompas akan terus increment/decrement ketika robot terus
  menerus berputar. Kompas perlu dikalibrasi terlebih dahulu
  menggunakan contoh program pada library
*/
void readCompass(void *parameters)
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  QMC5883LCompass compass;
  compass.init();
  compass.setCalibration(-1112, 1340, -1485, 966, -950, 0);
  compass.read();
  last_compass_reading = compass.getAzimuth();
  heading = last_compass_reading;
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
        heading = heading + offset;
      }
      else
      {
        offset = (360 - now) + last_compass_reading;
        heading = heading - (last_compass_reading + offset);
      }
    }
    else
      heading = heading + (now - last_compass_reading);

    last_compass_reading = now;
    heading_data.data = heading;
    vTaskDelay(PUBLISH_DELAY_MS / portTICK_PERIOD_MS);
  }
}

/*
  Robot menerima perintah dari rostopic cmd_vel
  Robot dapat bergerak ketika PC terkoneksi ke AP robot dan
  perintah terakhir kurang dari 0.5 detik yang lalu
  untuk menghindari robot bergerak terus menerus ketika
  terputus dari PC
*/
void moveBase(void *parameters)
{
  for (;;)
  {
    if (WiFi.softAPgetStationNum() > 0 && (millis() - last_command_time < 500))
    {
      float lin_x = vel_data.linear.y;
      float lin_y = vel_data.linear.x;
      float ang_z = -1 * vel_data.angular.z;
      base.setSpeed(lin_x, lin_y, ang_z);
    }
    else
      base.setSpeed(0, 0, 0);
    vTaskDelay(10);
  }
}

/*
  Menghitung RPM motor, jeda kalkulasi
  20 ms
*/
void countRpm(void *parameters)
{
  const int sampling_time_ms = 5;
  for (;;)
  {
    m1.calculateRpm(sampling_time_ms);
    m2.calculateRpm(sampling_time_ms);
    base.calculatePosition(heading);
    // DEBUG.printf("x : %.2f y : %.2f w : %.2f\n", base.x, base.y, base.w);
    // DEBUG.printf("m1 : %.3f m2 : %.3f\n", m1.speed_ms, m2.speed_ms);
    vTaskDelay(sampling_time_ms / portTICK_PERIOD_MS);
  }
}

/*
  Membaca data encoder terkini dan
  mengcopy nilainya ke data pada topik
  encoder
*/
void odometry(void *parameters)
{
  for (;;)
  {
    pose_data.x = base.x;
    pose_data.y = base.y;
    pose_data.theta = base.w;
    vTaskDelay(PUBLISH_DELAY_MS / portTICK_PERIOD_MS);
  }
}

/*
  Kendali PID posisi robot
  dengan umpan balik
  posisi robot saat ini

*/
void poseControl(void *parameters)
{
  for (;;)
  {
    if (!finish)
    {
      float lin_x = goal_x.compute(base.x);
      float lin_y = goal_y.compute(base.y);
      float ang_z = goal_w.compute(base.w);
      base.setSpeed(-lin_x, lin_y, -ang_z);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (abs(goal_x.setpoint - base.x) < 0.01 and abs(goal_y.setpoint - base.y) < 0.01 and abs(goal_w.setpoint - base.w) < 0.01)
    {
      if (marker_array_position < marker_data.points_length)
      {
        goal_x.reset();
        goal_y.reset();
        goal_w.reset();
        goal_x.setpoint = marker_data.points[marker_array_position].x;
        goal_y.setpoint = marker_data.points[marker_array_position].y;
        goal_w.setpoint = 0;
        marker_array_position++;
      }
      else
      {
        finish = true;
        marker_array_position = 0;
      }
    }
  }
}

void initWebSocket(void *parameters)
{
#ifndef AP
  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
      break;
  }
#endif

  WiFiServer wifiServer(80);
  wifiServer.begin();
  DEBUG.println("WS server begin");
  for (;;)
  {
    WiFiClient client = wifiServer.available();
    if (client)
    {
      while (client.connected())
      {
        while (client.available() > 0)
        {
          String c = client.readStringUntil('\n');
          StaticJsonDocument<1000> doc;
          DeserializationError error = deserializeJson(doc, c);
          if (error)
          {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
          }
          for (JsonObject item : doc.as<JsonArray>())
          {
            float x = item["x"];
            float y = item["y"];
            markers.points_length++;
            markers.markers_x.push_back(x);
            markers.markers_y.push_back(y);
          }
        }
        vTaskDelay(1);
      }
      client.stop();
      DEBUG.println("Client disconnected");
    }
    vTaskDelay(1);
  }
}

void setup()
{
  DEBUG.begin(115200);
  analogWriteFrequency(10000);
  attachInterrupt(digitalPinToInterrupt(m1.en_a), EN1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m2.en_a), EN2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m3.en_a), EN3_ISR, FALLING);
  m1.pid(7, 0.05, 1, 255);
  m2.pid(7, 0.05, 1, 255);
  base.setMotor(m1, m2);
  // Spawn task RTOS
  // xTaskCreatePinnedToCore(fungsi, "nama fungsi", alokasi memori, prioritas, task handle, core);
  // ESP32 memiliki 3 core, yaitu core 0, core 1, dan ULP
  // Sebisa mungkin prioritas task disamakan untuk menghindari crash
  // Task yang paling sering dijalankan diberikan prioritas paling tinggi
  // sem_i2c = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(wifiSetup, "wifi setup", 10000, NULL, 5, &wifi_task, 0);    // Pengaturan akses poin
  xTaskCreatePinnedToCore(blink, "blink", 1000, NULL, 2, &blink_task, 1);             // Test apakah RTOS dapat berjalan
  xTaskCreatePinnedToCore(initWebSocket, "node", 10000, NULL, 5, &websocket_task, 0); // Inisialisasi ros node
  // xTaskCreatePinnedToCore(publishMessage, "publisher", 10000, NULL, 2, &ros_pub, 1);           // Task publish ros messsage
  xTaskCreatePinnedToCore(readCompass, "compass", 10000, NULL, 2, &cmp_task, 1);               // Membaca sensor kompas
  xTaskCreatePinnedToCore(moveBase, "base", 5000, NULL, 2, &motor_task, 1);                    // Menggerakkan base robot
  xTaskCreatePinnedToCore(countRpm, "rpm", 5000, NULL, 2, &rpm_task, 1);                       // Menghitung RPM
  xTaskCreatePinnedToCore(odometry, "odometry", 5000, NULL, 2, &odometry_task, 1);             // Set data untuk message MotorEncoder
  xTaskCreatePinnedToCore(poseControl, "pose control", 10000, NULL, 2, &pose_control_task, 1); // Set data untuk message MotorEncoder
}

void loop()
{
  vTaskDelay(1);
}