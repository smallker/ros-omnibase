#ifndef _kinematic_h
#define _kinematic_h

#include <Arduino.h>
#include <motor.h>

#define DIFFDRIVE       0
#define OMNIBASE_Y      1
#define OMNIBASE_X      2
#define MECHANUM        3


class Kinematic
{
private:
    byte mode;
    Motor *m1, *m2, *m3, *m4;
    float r_base = 0.112; // jari-jari base (khusus roda 3)
    double sqrt3 = 1.732050807568877193176604123436845839023590087890625;
    float d_wheel = 0.06;

public:
    float x, y, w;
    Kinematic(byte mode);
    void setMotor(Motor &m1, Motor &m2);
    void setMotor(Motor &m1, Motor &m2, Motor &m3);
    void setMotor(Motor &m1, Motor &m2, Motor &m3, Motor &m4);
    void setSpeed(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z);
    void omniBaseOdom();
};

#endif