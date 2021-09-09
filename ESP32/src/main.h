#include <Arduino.h>
#include <analogWrite.h>
#include <FreeRTOS.h>
#include <QMC5883LCompass.h>
#include <kinematic.h>
#include <pid.h>
#include "wifi_setup.h"
#include "ros_setup.h"
// Map input/output ke nama yg mudah diingat
// kode M untuk pin motor
#define M1_A 16
#define M1_B 4
#define M1_PWM 17
#define M2_A 18
#define M2_B 5
#define M2_PWM 19
#define M3_A 33
#define M3_B 32
#define M3_PWM 25

// kode EN untuk pin encoder
#define EN1_A 26
#define EN1_B 35
#define EN2_A 27
#define EN2_B 34
#define EN3_A 13
#define EN3_B 39

// Digunakan mematikan interrupt termasuk RTOS
// saat eksternal interrupt aktif
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task handle untuk RTOS
TaskHandle_t wifi_task;
TaskHandle_t blink_task;
TaskHandle_t ros_task;
TaskHandle_t ros_pub;
TaskHandle_t cmp_task;
TaskHandle_t motor_task;
TaskHandle_t rpm_task;
TaskHandle_t odometry_task;
TaskHandle_t imu_task;
TaskHandle_t pose_control_task;
// Task RTOS

void blink(void *parameters);
void initNode(void *parameters);
void publishMessage(void *parameter);
void readCompass(void *parameters);
void moveBase(void *parameters);
void countRpm(void *parameters);
void odometry(void *parameters);
void poseControl(void *parameters);

// primitive global variable
volatile int heading;
volatile int sp_heading;
volatile int last_compass_reading;
volatile bool is_ros_ready, pose_control_begin;
volatile unsigned long last_command_time;
volatile bool finish;
volatile int marker_array_position;

// inisialisasi objek motor
Motor m1(M1_A, M1_B, M1_PWM, EN1_A, EN1_B);
Motor m2(M2_A, M2_B, M2_PWM, EN2_A, EN2_B);
Motor m3(M3_A, M3_B, M3_PWM, EN3_A, EN3_B);

// inisialisasi objek kinematik
Kinematic base(BASE_OMNI_Y);

Pid goal_x = Pid(5, 0, 0);
Pid goal_y = Pid(5, 0, 0);
Pid goal_w = Pid(5, 0, 0);