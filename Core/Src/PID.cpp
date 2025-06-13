#include "PID.h"

void PID::set(float kp, float ki, float kd, float outMin, float outMax, float N)
{
    kP = kp;
    kI = ki;
    kD = kd;
    out_min = outMin;
    out_max = outMax;
    alpha = 1.0f / (1.0f + N); // Filtro pasa-bajas en derivada (si dt = 1)
    deriv_filtered = 0.0f;
}

float PID::calculate(float setpoint, float input, float dt)
{
    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - last_error) / dt;
    alpha = 1.0f / (1.0f + filter_N * dt);
    deriv_filtered = alpha * deriv_filtered + (1.0f - alpha) * derivative;

    float output = kP * error + kI * integral + kD * deriv_filtered;

    if (output > out_max)
        output = out_max;
    if (output < out_min)
        output = out_min;

    last_error = error;
    return output;
}
