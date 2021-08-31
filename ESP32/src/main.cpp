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

void onSetMotorPid(const geometry_msgs::Point &msg_data)
{
  m3.pid(msg_data.x, msg_data.y, msg_data.z, 1000);
  Serial.println(msg_data.x);
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
  goal_x = 0;
  goal_y = 0;
  pose_control_begin = false;
}

void onMoveBaseToGoal(const geometry_msgs::PoseStamped &msg_data)
{
  goal_x = msg_data.pose.position.x;
  goal_y = msg_data.pose.position.y;
  pose_control_begin = true;
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
    vTaskDelay(is_ros_ready ? 2000 : 300);
  }
}

/*
  Inisialisasi ros node
  Mendaftarkan publisher dan subscriber
  Publisher :
  - heading_pub -> data kompas
  - encoder_pub -> data rotary encoder
  Subscriber :
  - vel_sub     -> data setpoint kecepatan robot
  - rst_pos_sub -> reset posisi robot
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
    }
    if (client.connected() != 1)
    {
      nh.initNode();
      nh.subscribe(vel_sub);
      nh.subscribe(rst_pos_sub);
      nh.subscribe(goal_sub);
      nh.advertise(pose_pub);
    }
    if (client.connected() == 1)
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
    if (client.connected() == 1)
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
    analogWriteFrequency(10000);
    m1.pid(7, 0.05, 1, 255);
    m2.pid(7, 0.05, 1, 255);
    m3.pid(6, 0.05, 1, 255);
    base.setMotor(m1, m2, m3);
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
    m3.calculateRpm(sampling_time_ms);
    base.omnibaseOdom(heading);
    // Serial.printf("x : %.2f y : %.2f w : %.2f\n", base.x, base.y, base.w);
    // Serial.printf("m1 : %.3f m2 : %.3f m3 : %.3f\n", m1.speed_ms, m2.speed_ms, m3.speed_ms);
    // vTaskDelay(sampling_time_ms / portTICK_PERIOD_MS);
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

void poseControl(void *parameters)
{
  Pid pid_x = Pid(0.5, 0, 0);
  Pid pid_y = Pid(0.5, 0, 0);
  Pid pid_w = Pid(0.1, 0, 0);
  for (;;)
  {
    if (pose_control_begin)
    {
      pid_x.setpoint = goal_x;
      pid_y.setpoint = goal_y;
      pid_w.setpoint = 0;
      float lin_x = pid_x.compute(base.x);
      float lin_y = pid_y.compute(base.y);
      float ang_z = pid_w.compute(base.w);
      base.setSpeed(- lin_x, lin_y, - ang_z);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(m1.en_a), EN1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m2.en_a), EN2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(m3.en_a), EN3_ISR, FALLING);
  // Spawn task RTOS
  // xTaskCreatePinnedToCore(fungsi, "nama fungsi", alokasi memori, prioritas, task handle, core);
  // ESP32 memiliki 3 core, yaitu core 0, core 1, dan ULP
  // Sebisa mungkin prioritas task disamakan untuk menghindari crash
  // Task yang paling sering dijalankan diberikan prioritas paling tinggi
  // sem_i2c = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(wifiSetup, "wifi setup", 10000, NULL, 5, &wifi_task, 0);   // Pengaturan akses poin
  xTaskCreatePinnedToCore(blink, "blink", 1000, NULL, 1, &blink_task, 1);            // Test apakah RTOS dapat berjalan
  xTaskCreatePinnedToCore(initNode, "node", 5000, NULL, 5, &ros_task, 0);            // Inisialisasi ros node
  xTaskCreatePinnedToCore(publishMessage, "publisher", 10000, NULL, 2, &ros_pub, 1); // Task publish ros messsage
  xTaskCreatePinnedToCore(readCompass, "compass", 10000, NULL, 2, &cmp_task, 1);     // Membaca sensor kompas
  xTaskCreatePinnedToCore(moveBase, "base", 5000, NULL, 2, &motor_task, 1);          // Menggerakkan base robot
  xTaskCreatePinnedToCore(countRpm, "rpm", 5000, NULL, 2, &rpm_task, 1);             // Menghitung RPM
  xTaskCreatePinnedToCore(odometry, "odometry", 5000, NULL, 2, &odometry_task, 1);   // Set data untuk message MotorEncoder
  xTaskCreatePinnedToCore(poseControl, "pose control", 10000, NULL, 2, &pose_control_task, 1);   // Set data untuk message MotorEncoder
}
void loop()
{
  vTaskDelay(1);
}