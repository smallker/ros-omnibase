#include <Arduino.h>
#include <analogWrite.h>
#include <FreeRTOS.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <kinematic.h>
#include "ota.h"
#include "ros_setup.h"
#define M1_A    4
#define M1_B    16
#define M1_PWM  17
#define M2_A    5
#define M2_B    18
#define M2_PWM  19
#define M3_A    32
#define M3_B    33
#define M3_PWM  25

#define EN1_A   35
#define EN1_B   26
#define EN2_A   34
#define EN2_B   27
#define EN3_A   39
#define EN3_B   13

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t blink;
TaskHandle_t ros_task;
TaskHandle_t ros_pub;
TaskHandle_t cmp_task;
TaskHandle_t motor_task;


void blinker(void *parameters);
void initNode(void *parameters);

volatile int heading;

Motor m1(M1_A, M1_B, M1_PWM);
Motor m2(M2_A, M2_B, M2_PWM);
Motor m3(M3_A, M3_B, M3_PWM);

Kinematic base(OMNIBASE_Y);