/*
 * Motor.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#include "Motor.h"

Motor::Motor(){
    pidController.set(Constants::kMotorKP, Constants::kMotorKI, Constants::kMotorKD, Constants::kMotorKImax, Constants::kMotorMinOut, Constants::kMotorMaxOut);
}
void Motor::init(Pin _pinA, Pin _pinB, Pin _encoder, uint32_t _pwm_channel, TIM_HandleTypeDef* _htim){
    this->pinA = _pinA;
    this->pinB = _pinB;
    this->encoder = _encoder;
    this->pwm_channel = _pwm_channel;
    this->htim = _htim;
    pidController.set(Constants::kMotorKP, Constants::kMotorKI, Constants::kMotorKD, Constants::kMotorKImax, Constants::kMotorMinOut, Constants::kMotorMaxOut);
}

void Motor::set_pwm_forward(uint16_t pwm_value){
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
void Motor::update_motor(uint32_t current_time){
    float dt = (current_time - last_time_ms) / 1000.0f;
    if (dt <= 0.0f) return;

    delta_ticks = ticks - last_ticks;
    distance_cm += delta_ticks * Constants::kCMPerTick;
    actual_speed_cm_s = (delta_ticks * Constants::kCMPerTick) / dt;

    float error = target_speed_cm_s - actual_speed_cm_s;
    integral += error * dt;
    float derivative = (error - last_error) / dt;

    float output = kp * error + ki * integral + kd * derivative;
    pwm_out = std::min(std::max(output, 0.0f), Constants::kMaxPWM); // PWM (0–50)

    // Dirección hacia adelante
    HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(htim, pwm_channel, (uint16_t)pwm_out);
    HAL_TIM_PWM_Start(htim, pwm_channel);

    last_error = error;
    last_ticks = ticks;
    last_time_ms = current_time;
}
void Motor::stop_motor(){
    // Active brake: both inputs HIGH
    HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_SET);

    // Stop PWM signal
    __HAL_TIM_SET_COMPARE(htim, pwm_channel, 0);
}

void Motor::setTarget(float _target_speed_cm_s){
    target_speed_cm_s = _target_speed_cm_s;
}

float Motor::getDistance(){
    return distance_cm;
}