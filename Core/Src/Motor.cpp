/*
 * Motor.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#include "Motor.h"

Motor::Motor(float KP, float KI, float KD, float Ns)
{
    pidController.set(KP, KI, KD, 0, Constants::kMaxPWM, Ns);
}
void Motor::init(Pin _pinA, Pin _pinB, uint16_t _encoder, uint32_t _pwm_channel, TIM_HandleTypeDef *_htim)
{
    this->pinA = _pinA;
    this->pinB = _pinB;
    this->encoder = _encoder;
    this->pwm_channel = _pwm_channel;
    this->htim = _htim;
}

void Motor::set_pwm_forward(uint16_t pwm_value)
{
    // Dirección hacia adelante: A = HIGH, B = LOW
    HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_RESET);

    // PWM limitado al ARR máximo
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint16_t duty = (pwm_value > arr) ? arr : pwm_value;

    // Enviar PWM
    HAL_TIM_PWM_Start(htim, pwm_channel);
    __HAL_TIM_SET_COMPARE(htim, pwm_channel, duty);
}
void Motor::set_pwm_backward(uint16_t pwm_value)
{
    // Dirección hacia adelante: A = HIGH, B = LOW
    HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_SET);

    // PWM limitado al ARR máximo
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint16_t duty = (pwm_value > arr) ? arr : pwm_value;

    // Enviar PWM
    HAL_TIM_PWM_Start(htim, pwm_channel);
    __HAL_TIM_SET_COMPARE(htim, pwm_channel, duty);
}
void Motor::update_motor(uint32_t current_time)
{
    float dt = (current_time - last_time_ms) / 1000.0f;
    if (dt <= 0.0f)
        return;

    delta_ticks = ticks - last_ticks;
    distance_cm += delta_ticks * Constants::kCMPerTick;
    actual_speed_cm_s = (delta_ticks * Constants::kCMPerTick) / dt;

    // Convert cm/s to PWM equivalent
    float max_cm_s = (Constants::kMotorsRPM * 3.14159f * Constants::kWheelDiameter) / 60.0f;
    float scaled_target_pwm = (target_speed_cm_s / max_cm_s) * Constants::kMaxPWM;
    float scaled_actual_pwm = (actual_speed_cm_s / max_cm_s) * Constants::kMaxPWM;

    output = pidController.calculate(scaled_target_pwm, scaled_actual_pwm, dt);
    if (std::abs(target_speed_cm_s - actual_speed_cm_s) > 0.3)
    {
        pwm_out = std::min(std::max(output, Constants::kMinPWM), Constants::kMaxPWM);
    }

    // if (target_speed_cm_s < 0)
    // {
    //     // Direccion hacia atras
    //     setDir(!dir);
    //     __HAL_TIM_SET_COMPARE(htim, pwm_channel, (uint16_t)(pwm_out));
    // }
    // else
    // {
    //     // Dirección hacia adelante
    //     setDir(dir);
    // }
    __HAL_TIM_SET_COMPARE(htim, pwm_channel, (uint16_t)pwm_out);
    HAL_TIM_PWM_Start(htim, pwm_channel);
    last_ticks = ticks;
    last_time_ms = current_time;
}
void Motor::stop_motor()
{
    // Active brake: both inputs HIGH
    HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_SET);

    // Stop PWM signal
    __HAL_TIM_SET_COMPARE(htim, pwm_channel, 0);
}

void Motor::setTarget(float _target_speed_cm_s)
{
    target_speed_cm_s = _target_speed_cm_s;
}

float Motor::getDistance()
{
    return distance_cm;
}

void Motor::addTicks()
{
    ticks++;
}

float Motor::getPWM()
{
    return pwm_out;
}

float Motor::getSpeed()
{
    return actual_speed_cm_s;
}

int Motor::getOutput()
{
    return (int)output;
}

float Motor::getTarget()
{
    return target_speed_cm_s;
}

void Motor::updateWithoutPID(uint32_t current_time)
{

    float dt = (current_time - last_time_ms) / 1000.0f;
    delta_ticks = ticks - last_ticks;
    distance_cm += delta_ticks * Constants::kCMPerTick;
    actual_speed_cm_s = (delta_ticks * Constants::kCMPerTick) / dt;
    last_ticks = ticks;
    last_time_ms = current_time;
}

void Motor::setDir(bool temp_dir)
{
    if (!temp_dir)
    {
        // Adelante
        HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_RESET);
    }
    else
    {
        // Atras
        HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_SET);
    }
}

void Motor::forward()
{
    dir = false;
}

void Motor::backward()
{
    dir = true;
}