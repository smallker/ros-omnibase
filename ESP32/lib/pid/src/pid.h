class Pid
{
public:
    Pid(float kp, float ki, float kd);
    float compute(float now);
    void reset();
    float setpoint;
    void setPid(float kp, float ki, float kd);
private:
    float kp, ki, kd;
    float i_err;
    float d_err;
    float last_err;
    float windup=0.1;
    float now = 0;
    float limit = 0.6;
};