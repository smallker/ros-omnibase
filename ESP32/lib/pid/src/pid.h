class Pid
{
public:
    Pid(float kp, float ki, float kd);
    float compute();
    void reset();
    float setpoint;
    void setPid(float kp, float ki, float kd);
    float compute_from_err(float err);
    float pos = 0;

private:
    float kp, ki, kd;
    float i_err;
    float d_err;
    float last_err;
    float windup=0.1;
    float limit = 0.6;
};