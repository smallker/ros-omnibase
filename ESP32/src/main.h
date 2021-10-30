#include <Arduino.h>
#include <analogWrite.h>
#include <FreeRTOS.h>
#include <QMC5883LCompass.h>
#include <kinematic.h>
#include <pid.h>
#include "wifi_setup.h"
#include "ros_setup.h"

// Port serial untuk debugging
#define DEBUG   Serial

// Map input/output ke nama yg mudah diingat
// kode M untuk pin motor
#define M1_A    4
#define M1_B    16
#define M1_PWM  17
#define M2_A    5
#define M2_B    18
#define M2_PWM  19
#define M3_A    32
#define M3_B    33
#define M3_PWM  25

// kode EN untuk pin encoder
#define EN1_A   26
#define EN1_B   35
#define EN1_PPR 700
#define EN2_A   27
#define EN2_B   34
#define EN2_PPR 300
#define EN3_A   13
#define EN3_B   39
#define EN3_PPR 800

#define D_WHEEL 0.06

// Digunakan mematikan interrupt termasuk RTOS
// saat eksternal interrupt aktif
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task handle untuk RTOS
TaskHandle_t wifi_task;
TaskHandle_t blink_task;
TaskHandle_t ros_task;
TaskHandle_t ros_pub_task;
TaskHandle_t compass_task;
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
void odometry(void *parameters);
void poseControl(void *parameters);

// primitive global variable
volatile int heading_deg;
volatile int last_compass_reading;
volatile bool is_ros_ready, finish;
volatile unsigned long last_command_time;
volatile int marker_array_position;

// inisialisasi objek motor
Motor m1(M1_A, M1_B, M1_PWM, EN1_A, EN1_B, EN1_PPR, D_WHEEL);
Motor m2(M2_A, M2_B, M2_PWM, EN2_A, EN2_B, EN2_PPR, D_WHEEL);
Motor m3(M3_A, M3_B, M3_PWM, EN3_A, EN3_B, EN3_PPR, D_WHEEL);

// inisialisasi objek kinematik
Kinematic base(BASE_OMNI_Y);

/*
    Robot dapat bergerak dalam 3DOF
    goal_x merupakan arah robot dalam sumbu X
    goal_y merupakan arah robot dalam sumbu Y
    goal_w merupakan arah gerak memutar robot
*/
Pid goal_x = Pid(5, 0, 1);
Pid goal_y = Pid(5, 0, 1);
Pid goal_w = Pid(7, 0, 0);