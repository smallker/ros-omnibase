#include "pid.h"
/*
    Mengatur nilai Kp, Ki, dan Kd
*/
Pid::Pid(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/*
    Menghitung nilai PWM berdasarkan error yang ada
    (error titik pengesetan, error integral, dan error derivatif)
*/
float Pid::compute(float now)
{
    if (i_err > windup)
        i_err = 0;
    float err = setpoint - now;
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
/*
    Mengatur ulang semua perhitungan error
*/
void Pid::reset()
{
    d_err = 0;
    i_err = 0;
    last_err = 0;
    setpoint = 0;
}
/*
    Mengubah nilai Kp, Ki, Kd
*/
void Pid::setPid(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}