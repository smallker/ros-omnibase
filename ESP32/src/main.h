#include <Arduino.h>
#include <analogWrite.h>
#include <FreeRTOS.h>
#include <QMC5883LCompass.h>
#include <kinematic.h>
#include <pid.h>
#include <ArduinoJson.h>
#include "wifi_setup.h"
#include "ros_setup.h"
// Port serial untuk debugging
#define DEBUG   Serial

// Map input/output ke nama yg mudah diingat
// kode M untuk pin motor
#define M1_A 16
#define M1_B 4
#define M1_PWM 17
#define M2_A 18
#define M2_B 5
#define M2_PWM 19

// kode EN untuk pin encoder
#define EN1_A 35
#define EN1_B 26
#define EN2_A 27
#define EN2_B 34
#define EN3_A 39
#define EN3_B 13
#define EN4_A 15
#define EN4_B 14
// Digunakan mematikan interrupt termasuk RTOS
// saat eksternal interrupt aktif
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task handle untuk RTOS
TaskHandle_t wifi_task;
TaskHandle_t blink_task;
TaskHandle_t node_handle_task;
TaskHandle_t ros_pub;
TaskHandle_t cmp_task;
TaskHandle_t motor_task;
TaskHandle_t rpm_task;
TaskHandle_t odometry_task;
TaskHandle_t imu_task;
TaskHandle_t pose_control_task;
TaskHandle_t odom_extern_task;
// Task RTOS

void blink(void *parameters);
void initNode(void *parameters);
void publishMessage(void *parameter);
void readCompass(void *parameters);
void moveBase(void *parameters);
void countRpm(void *parameters);
void odometry(void *parameters);
void poseControl(void *parameters);
void odomExtern(void *parameters);

// Helper function
void directMode(void);
void pivotMode(void);
void headingMode(void);

// primitive global variable
volatile int heading;
volatile int sp_heading;
volatile int last_compass_reading;
volatile bool is_ros_ready, pose_control_begin;
volatile unsigned long last_command_time;
volatile bool pose_control_started;
volatile int marker_array_position;

// inisialisasi objek motor
Motor m1(M1_A, M1_B, M1_PWM, EN1_A, EN1_B, 700);
Motor m2(M2_A, M2_B, M2_PWM, EN2_A, EN2_B, 900);
Motor en_ext1(EN3_A, EN3_B);
Motor en_ext2(EN4_A, EN4_B);
// Motor m3(M3_A, M3_B, M3_PWM, EN3_A, EN3_B);

// inisialisasi objek kinematik
Kinematic base(BASE_DIFF_DRIVE);
Kinematic base_ext(BASE_DIFF_DRIVE);
Pid lin_pid = Pid(0.2, 0, 1);
Pid ang_pid = Pid(0.2, 0, 1);

// Array marker
struct Markers
{
    std::vector<std::float_t> markers_x;
    std::vector<std::float_t> markers_y;
    int points_length;
} markers;

struct BaseSpeed
{
    float x_speed;
    float y_speed;
    float w_speed;
} base_speed;

// Websocket control flow
volatile bool ws_ready;

// Either direct or pivot mode
volatile Mode mode;