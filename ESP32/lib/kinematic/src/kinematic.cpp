#include "kinematic.h"

Kinematic::Kinematic(byte mode)
{
    this->mode = mode;
}
void Kinematic::setMotor(Motor &m1, Motor &m2, Motor &m3)
{
    this->m1 = &m1;
    this->m2 = &m2;
    this->m3 = &m3;
}

void Kinematic::setSpeed(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z){
    if(mode == OMNIBASE_Y){
        float a = (0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float b = (-0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float c = (0 * linear_x) + (0.67 * linear_y) + (0.33 * angular_z);
        float sp_a = a * 6.0 * 1000.0;
        float sp_b = b * 6.0 * 1000.0;
        float sp_c = c * 6.0 * 1000.0;
        m1->speed(sp_a);
        m2->speed(sp_b);
        m3->speed(sp_c);
        // Serial.println("a : "+(String)sp_a+" b : "+(String)sp_b+" c : "+(String)sp_c);
    }
}
