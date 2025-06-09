#include "PID.h"

void PID::set(float kp, float ki, float kd, float outMin, float outMax)
{
    kP = kp;
    kI = ki;
    kD = kd;
    out_min = outMin;
    out_max = outMax;
}

float PID::calculate(float setpoint, float input, float dt)
{
    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - last_error) / dt;
    float output = kP * error + kI * integral + kD * derivative;

    if (output > out_max)
        output = out_max;
    if (output < out_min)
        output = out_min;

    last_error = error;
    return output;
}
