/*
 * Motor.h
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "PID.h"
#include "Constants.h"
#include "stm32f1xx_hal.h"     // Para tipos como TIM_HandleTypeDef y GPIO
#include "stm32f1xx_hal_tim.h" // Específicamente para funciones de TIM (PWM)

class Motor
{
private:
    // Configuration: A(fwd) and B(rev) //
    Pin enable;
    Pin pinA;
    Pin pinB;
    uint16_t encoder;
    PID pidController;
    TIM_HandleTypeDef *htim;
    uint32_t pwm_channel;
    uint32_t last_time_ms = 0;

    // Usefull variables //
    uint16_t current_pwm_value;
    uint32_t ticks = 0;
    uint32_t last_ticks = 0;
    float delta_ticks = 0;
    float distance_cm = 0;
    float target_speed_cm_s = 0;
    float actual_speed_cm_s = 0;
    float kp = Constants::kMotorKP;
    float ki = Constants::kMotorKI;
    float kd = Constants::kMotorKD;
    float integral = 0;
    float last_error = 0;
    float pwm_out = 0;

public:
    Motor();
    void init(Pin _pinA, Pin _pinB, uint16_t _encoder, uint32_t _pwm_channel, TIM_HandleTypeDef *_htim);
    void set_pwm_forward(uint16_t pwm_value);
    void update_motor(uint32_t current_time);
    void stop_motor();
    void setTarget(float _target_speed_cm_s);
    float getDistance();
    void addTicks();
    float getPWM();
};

#endif /* MOTOR_H_ */
