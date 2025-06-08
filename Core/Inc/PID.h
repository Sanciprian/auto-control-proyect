#ifndef INC_PID_H_
#define INC_PID_H_

class PID
{
private:
    float last_error = 0;
    float integral = 0;
    float kP = 0, kI = 0, kD = 0;
    float out_min = 0, out_max = 100;

public:
    PID() = default;
    void set(float kp, float ki, float kd, float outMin, float outMax);
    float calculate(float setpoint, float input, float dt);
};

#endif /* INC_PID_H_ */
