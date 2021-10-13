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

void ICACHE_RAM_ATTR EN_EXT1_ISR()
{
  portENTER_CRITICAL(&mux);
  en_ext1.isrHandler();
  portEXIT_CRITICAL(&mux);
}

void ICACHE_RAM_ATTR EN_EXT2_ISR()
{
  portENTER_CRITICAL(&mux);
  en_ext2.isrHandler();
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
  heading = 0;
  base.x = 0;
  base.y = 0;
  base.w = 0;
  lin_pid.setpoint = 0;
  ang_pid.setpoint = 0;
  lin_pid.reset();
  ang_pid.reset();
  pose_control_started = false;
}

void onMoveBaseToGoal(const geometry_msgs::PoseStamped &msg_data)
{
  lin_pid.setpoint = msg_data.pose.position.x;
  ang_pid.setpoint = msg_data.pose.position.y;
  pose_control_begin = true;
}

void onMarkerSet(const visualization_msgs::Marker &msg_data)
{
  marker_data = msg_data;
}

void onPivotMode(const std_msgs::Empty &msg_data)
{
  DEBUG.println("MARKER FOLLOWER STARTED");
  mode = PIVOT;
  marker_array_position = 0;
  pose_control_started = true;
  for (int i = 0; i < marker_data.points_length; i++)
  {
    DEBUG.printf("X : %.f Y : %.f Z : %.f\n", marker_data.points[i].x, marker_data.points[i].y, marker_data.points[i].z);
  }
}

void onHeadingMode(const std_msgs::Float32 &msg_data){
  mode = HEADING;
  pose_control_started = true;
  ang_pid.setpoint = msg_data.data;
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
      nh.subscribe(pivot_mode_sub);
      nh.subscribe(marker_sub);
      nh.subscribe(heading_mode_sub);
      nh.advertise(pose_pub);
      nh.advertise(heading_int_pub);
      // nh.advertise(pose_ext_pub);
      // nh.advertise(heading_pub);
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
      // ambil data kompas dan heading odom
      // char buffer[30];
      // int heading_odom = base.w * 180 / PI;
      // float left_wheel = m1.encoder_tick_acc / m1.ppr * 0.204;
      // float right_wheel = m2.encoder_tick_acc / m2.ppr * 0.204;
      // sprintf(buffer, "%.2f,%.2f,%d,%d,", base.x, base.y, heading, heading_odom);
      // heading_str_data.data = buffer;
      // heading_pub.publish(&heading_str_data);

      is_ros_ready = true;
      pose_pub.publish(&pose_data);
      heading_int_pub.publish(&heading_data);
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
  compass.setCalibration(-1465, 1283, -1105, 1558, -1525, 0);
  // compass.setCalibration(-1112, 1340, -1485, 966, -950, 0);
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
    if (millis() - last_command_time < 500)
    {
      float lin_x = vel_data.linear.y;
      float lin_y = vel_data.linear.x;
      float ang_z = vel_data.angular.z;
      base.setSpeed(lin_x, lin_y, ang_z);
      // DEBUG.printf("lin_x : %.2f, lin_y : %.2f, ang_z : %.2f\n", lin_x, lin_y, ang_z);
    }
    else
    {
      base.setSpeed(0, 0, 0);
      // DEBUG.println("STOPPED");
    }
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
  // unsigned long delay_printf = millis();
  for (;;)
  {
    m1.calculateRpm(sampling_time_ms);
    m2.calculateRpm(sampling_time_ms);
    // m1.speed_ms = 0.001;
    // m2.speed_ms = 0.001;
    base.calculatePosition(base.w);
    // if (millis() - delay_printf > 100)
    // {
    //   // DEBUG.printf("m1 : %d m2 : %d\n", m1.encoder_tick_acc, m2.encoder_tick_acc);
    //   // DEBUG.printf("x : %.2f y : %.2f w : %.2f\n", base.x, base.y, base.w);
    //   delay_printf = millis();
    // }
    // DEBUG.printf("m1 : %.3f m2 : %.3f\n", m1.speed_ms, m2.speed_ms);

    // DEBUG.println(m1.rpm_abs);
    vTaskDelay(sampling_time_ms / portTICK_PERIOD_MS);
  }
}

/*
  Menghitung RPM motor, jeda kalkulasi
  20 ms
*/
void odomExtern(void *parameters)
{
  en_ext1.ppr = 370;
  en_ext2.ppr = 370;
  const int sampling_time_ms = 5;
  // unsigned long delay_printf = millis();
  for (;;)
  {
    en_ext1.calculateRpm(sampling_time_ms);
    en_ext2.calculateRpm(sampling_time_ms);
    // m1.speed_ms = 0.001;
    // m2.speed_ms = 0.001;
    // base_ext.calculatePosition(base_ext.w);
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
    // pose_data.x = m1.encoder_tick_acc;
    // pose_data.y = m2.encoder_tick_acc;
    pose_data.x = base.x;
    pose_data.y = base.y;
    pose_data.theta = base.w;
    // DEBUG.printf("x : %.2f y : %.2f w : %.2f\n", base.x, base.y, base.w);
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
    if (mode == DIRECT)
      directMode();
    if (mode == PIVOT)
      pivotMode();
    if (mode == HEADING)
      headingMode();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void directMode()
{
  if (pose_control_started == true && (marker_array_position < marker_data.points_length))
  {
    float goal_x = marker_data.points[marker_array_position].x;
    float goal_y = marker_data.points[marker_array_position].y;
    float goal_distance = base.getGoalDistance(goal_x, goal_y);
    float goal_heading = base.getGoalHeading(goal_x, goal_y);
    lin_pid.pos = goal_distance;
    ang_pid.setpoint = - goal_heading;
    ang_pid.pos = base.w;

    float linear = lin_pid.compute_from_err(goal_distance);
    float angular = ang_pid.compute();
    if (abs(lin_pid.setpoint - lin_pid.pos) < 0.01 && abs(ang_pid.setpoint - ang_pid.pos) < 0.1)
    {
      linear = 0;
      angular = 0;
      // mode = PIVOT;
      // marker_array_position++;
      DEBUG.println("CHANGE TO PIVOT MODE");
      pose_control_started = false;
    }
    base.setSpeed(0, linear, angular);
  }
}

void pivotMode()
{
  if (pose_control_started && (marker_array_position < marker_data.points_length))
  {
    float goal_x = marker_data.points[marker_array_position].x;
    float goal_y = marker_data.points[marker_array_position].y;
    float goal_distance = base.getGoalDistance(goal_x, goal_y);
    float goal_heading = base.getGoalHeading(goal_x, goal_y);
    lin_pid.pos = goal_distance;
    ang_pid.setpoint = -goal_heading;
    ang_pid.pos = base.w;
    DEBUG.printf("dist : %.2f head : %.2f\n", goal_distance, goal_heading);
    float angular = ang_pid.compute();
    if (abs(ang_pid.setpoint - ang_pid.pos) < 0.1)
    {
      mode = DIRECT;
    }
    base.setSpeed(0, 0, angular);
  }
}

void headingMode()
{
  if (pose_control_started)
  {
    ang_pid.pos = base.w;
    float angular = ang_pid.compute();
    base.setSpeed(0, 0, angular);
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
  vTaskDelay(3000);
  WiFiServer wifiServer(80);
  wifiServer.begin();
  DEBUG.println("WS server begin");
  unsigned long millis_server = millis();
  for (;;)
  {
    WiFiClient client = wifiServer.available();
    if (client)
    {
      while (client.connected())
      {
        if (millis() - millis_server > 500)
        {
          StaticJsonDocument<200> doc;

          doc["type"] = 1;
          doc["sc"] = "en";

          JsonObject data = doc.createNestedObject("data");
          data["x"] = base.x;
          data["y"] = base.y;
          data["w"] = (base.w) * 180 / PI;
          String buffer;
          serializeJson(doc, buffer);
          // Serial.printf("x : %.2f y : %.2f w : %.2f\n", base.x, base.y, base.w);
          // client.println(buffer);
          // client.printf("%.2f,%.2f", base.x + rand() % 100, base.y + rand() % 100);
          // client.printf("m1 : %d m2 : %d", m1.encoder_tick_acc, m2.encoder_tick_acc);
          millis_server = millis();
        }

        while (client.available() > 0)
        {
          String c = client.readStringUntil('\n');
          // DEBUG.println(c);
          // String input;

          StaticJsonDocument<500> doc;

          DeserializationError error = deserializeJson(doc, c);

          if (error)
          {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
          }
          /*
            data type
            0 = setpoint
            1 = speed
            2 = reset
          */
          int type = doc["type"];
          if (type == 0)
          {
            for (JsonObject data_item : doc["data"].as<JsonArray>())
            {
              float data_item_x = data_item["x"]; // 3.3, 3.3, 3.3, 3.3, 3.3, 3.3
              float data_item_y = data_item["y"]; // 3.1, 3.1, 3.1, 3.1, 3.1, 3.1
              markers.points_length++;
              markers.markers_x.push_back(data_item_x);
              markers.markers_y.push_back(data_item_y);
              DEBUG.printf("x => %.2f , y => %.2f\n", data_item_x, data_item_y);
            }
            pose_control_started = true;
          }

          if (type == 1)
          {
            float data_lin_speed = doc["data"]["lin_speed"]; //
            float data_ang_speed = doc["data"]["ang_speed"]; // 0.5
            base_speed.y_speed = data_lin_speed;
            base_speed.w_speed = data_ang_speed;
            // DEBUG.printf("lin : %.2f  ang : %.2f\n", data_lin_speed, data_ang_speed);
            // last_command_time = millis();
          }
          if (type == 2)
          {
            base_speed.x_speed = 0;
            base_speed.y_speed = 0;
            base_speed.w_speed = 0;
            base.x = 0;
            base.y = 0;
            base.w = 0;
            m1.encoder_tick_acc = 0;
            m2.encoder_tick_acc = 0;
            pose_control_started = true;
            lin_pid.reset();
            ang_pid.reset();
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

void publishData(void *parameters)
{
#ifndef AP
  while (true)
  {
    if (ws_ready)
      break;
  }
#endif
}

void setup()
{
  DEBUG.begin(115200);
  analogWriteFrequency(10000);
  attachInterrupt(digitalPinToInterrupt(m1.en_a), EN1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m2.en_a), EN2_ISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(en_ext1.en_a), EN_EXT1_ISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(en_ext2.en_a), EN_EXT2_ISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(m3.en_a), EN3_ISR, FALLING);
  m1.pid(7, 0, 1, 255);
  m2.pid(9, 0, 1, 255);
  base.setMotor(m1, m2);
  base_ext.setMotor(en_ext1, en_ext2);
  // Spawn task RTOS
  // xTaskCreatePinnedToCore(fungsi, "nama fungsi", alokasi memori, prioritas, task handle, core);
  // ESP32 memiliki 3 core, yaitu core 0, core 1, dan ULP
  // Sebisa mungkin prioritas task disamakan untuk menghindari crash
  // Task yang paling sering dijalankan diberikan prioritas paling tinggi
  // sem_i2c = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(wifiSetup, "wifi setup", 10000, NULL, 5, &wifi_task, 0);
  xTaskCreatePinnedToCore(initNode, "node", 10000, NULL, 5, &node_handle_task, 0); // Inisialisasi ros node
  xTaskCreatePinnedToCore(blink, "blink", 1000, NULL, 2, &blink_task, 1);          // Test apakah RTOS dapat berjalan
  // xTaskCreatePinnedToCore(initWebSocket, "node", 10000, NULL, 5, &websocket_task, 0); // Inisialisasi ros node
  xTaskCreatePinnedToCore(publishMessage, "publisher", 10000, NULL, 2, &ros_pub, 1);           // Task publish ros messsage
  xTaskCreatePinnedToCore(readCompass, "compass", 10000, NULL, 2, &cmp_task, 1);               // Membaca sensor kompas
  xTaskCreatePinnedToCore(moveBase, "base", 5000, NULL, 2, &motor_task, 1);                    // Menggerakkan base robot
  xTaskCreatePinnedToCore(countRpm, "rpm", 5000, NULL, 2, &rpm_task, 1);                       // Menghitung RPM
  xTaskCreatePinnedToCore(odometry, "odometry", 5000, NULL, 2, &odometry_task, 1);             // Set data untuk message MotorEncoder
  xTaskCreatePinnedToCore(poseControl, "pose control", 10000, NULL, 2, &pose_control_task, 1); // Set data untuk message MotorEncoder
  // xTaskCreatePinnedToCore(odomExtern, "rpm", 5000, NULL, 2, &odom_extern_task, 1);                       // Menghitung RPM
}

void loop()
{
  vTaskDelay(1);
}