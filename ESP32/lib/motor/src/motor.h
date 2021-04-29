#ifndef _motor_h
#define _motor_h
#include <Arduino.h>
#define PWM62K    1
#define PWM8K     2

#include <analogWrite.h>
class Motor
{
    /*
        Library Motor
        #define EMS -> jika menggunakan driver EMS, biarkan kosong jika menggunakan driver lain
    */
    // #define EMS
private:
    byte a_pin, b_pin, pwm_pin;
    float ppr = 135;
    float kp, ki, kd, windup, pwmPid, err, lastErr, mIntegral;
    void forward(int pwm);
    void reverse(int pwm);
    bool pidEnable;
    int min_pwm = 0;
    int max_pwm = 150;
    byte divisor = 3;
    int threshold = 50;
    void setPwmFrequency();
public:
    byte en_a, en_b;
    volatile int rpm;
    volatile int encoder_tick;
    volatile long encoder_tick_acc;
    Motor(byte a_pin, byte b_pin, byte pwm_pin);
    Motor(byte a_pin, byte b_pin, byte pwm_pin, byte en_a, byte en_b);
    Motor(byte a_pin, byte b_pin, byte pwm_pin, byte en_a, byte en_b, int ppr);
    void pid(float kp, float ki, float kd , float windup);
    void speed(int target);
    void brake();
    void isrHandler();
    void calculateRpm(int sampling_time_ms);
    void setPwmFrequency(byte divisor);
    void setPidThreshold(int up_thres, int down_thres);
    void setPwmThreshold(int threshold);
};

#endif