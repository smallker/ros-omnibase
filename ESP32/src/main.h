#include <Arduino.h>
#include <analogWrite.h>
#include <FreeRTOS.h>
#include "ota.h"
#include "ros_setup.h"
#include <Wire.h>
#include <QMC5883LCompass.h>
#define M1_A    4
#define M1_B    16
#define M1_PWM  17
#define M2_A    5
#define M2_B    18
#define M2_PWM  19
#define M3_A    32
#define M3_B    33
#define M3_PWM  25

TaskHandle_t blink;
TaskHandle_t ros_task;
TaskHandle_t ros_pub;
TaskHandle_t cmp_task;

void blinker(void *parameters);
void initNode(void *parameters);

volatile int heading;