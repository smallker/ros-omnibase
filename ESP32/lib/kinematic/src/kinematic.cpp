#include "kinematic.h"

Kinematic::Kinematic(Base base)
{
    this->base = base;
}
void Kinematic::setMotor(Motor &m1, Motor &m2, Motor &m3)
{
    this->m1 = &m1;
    this->m3 = &m2;
    this->m2 = &m3;
}

void Kinematic::setSpeed(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z)
{
    if (base == BASE_OMNI_Y)
    {
        float inv_m1 = (0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float inv_m2 = (0 * linear_x) + (0.67 * linear_y) + (0.33 * angular_z);
        float inv_m3 = (-0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float sp_m1 = (inv_m1 / (PI * d_wheel)) * 60;
        float sp_m2 = (inv_m2 / (PI * d_wheel)) * 60;
        float sp_m3 = (inv_m3 / (PI * d_wheel)) * 60;
        m1->speed(sp_m1);
        m2->speed(sp_m2);
        m3->speed(sp_m3);
    }
}

void Kinematic::setSpeed(float lin_x, float lin_y, float ang_z)
{
    if (base == BASE_OMNI_Y)
    {
        float inv_m1 = (-0.33 * lin_x) + (0.58 * lin_y) + (0.33 * ang_z);
        float inv_m2 = (0.67 * lin_x) + (0 * lin_y) + (0.33 * ang_z);
        float inv_m3 = (-0.33 * lin_x) + (-0.58 * lin_y) + (0.33 * ang_z);
        float sp_m1 = (inv_m1 / (PI * d_wheel)) * 60;
        float sp_m2 = (inv_m2 / (PI * d_wheel)) * 60;
        float sp_m3 = (inv_m3 / (PI * d_wheel)) * 60;
        m1->speed(sp_m1);
        m2->speed(sp_m2);
        m3->speed(sp_m3);
    }
}

void Kinematic::calculatePosition(float heading)
{
    float v1 = (m1->speed_ms);
    float v2 = (m2->speed_ms);
    float v3 = (m3->speed_ms);
    float vmx = (2 * v2 - v1 - v3) / 3;
    float vmy = ((sqrt3 * v3) - (sqrt3 * v1)) / 3;
    w = heading * PI / 180;
    x += (cos(w) * vmx) - (sin(w) * vmy);
    y += (sin(w) * vmx) + (cos(w) * vmy);
}