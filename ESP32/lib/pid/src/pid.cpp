#include "pid.h"

Pid::Pid(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float Pid::compute()
{
    if (i_err > windup)
        i_err = 0;
    float err = setpoint - pos;
    d_err = err - last_err;
    last_err = err;
    i_err = i_err + err;
    float result = (kp * err) + (ki * i_err) + (kd * d_err);
    if (result >= 0)
    {
        return result > limit ? limit : result;
    }
    else {
        return result < - limit ? - limit : result;
    }
}

float Pid::compute_from_err(float err)
{
    if (i_err > windup)
        i_err = 0;
    d_err = err - last_err;
    last_err = err;
    i_err = i_err + err;
    float result = (kp * err) + (ki * i_err) + (kd * d_err);
    if (result >= 0)
    {
        return result > limit ? limit : result;
    }
    else {
        return result < - limit ? - limit : result;
    }
}

void Pid::reset()
{
    d_err = 0;
    i_err = 0;
    last_err = 0;
    setpoint = 0;
}

void Pid::setPid(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}