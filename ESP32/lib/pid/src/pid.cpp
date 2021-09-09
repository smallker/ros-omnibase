#include "pid.h"

Pid::Pid(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float Pid::compute(float now){
    if(i_err > windup) i_err = 0;
    float err = setpoint - now;
    d_err = err - last_err;
    last_err = err;
    i_err = i_err + err;
    float result = (kp * err) + (ki * i_err) + (kd * d_err);
    return result > 0.5? 0.5 : result;
}

void Pid::reset(){
    d_err = 0;
    i_err = 0;
    last_err = 0;
    setpoint = 0;
}

void Pid::setPid(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}