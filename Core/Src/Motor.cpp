/*
 * Motor.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#include "Motor.h"

Motor::Motor(){
//    this->pinA = {};
//    this->pinB = 0;
//    this->encoder = 0;
    pidController.set(Constants::kMotorKP, Constants::kMotorKI, Constants::kMotorKD, Constants::kMotorKImax, Constants::kMotorMinOut, Constants::kMotorMaxOut);
}

void Motor::init(Pin enable, Pin pinA, Pin pinB, Pin encoder){
	this->enable = enable;
    this->pinA = pinA;
    this->pinB = pinB;
    this->encoder = encoder;

//    pinMode(pinA, OUTPUT);
//    pinMode(pinB, OUTPUT);
//    pinMode(encoder, INPUT_PULLUP);
}

void Motor::setTimer(TIM_HandleTypeDef* htim, uint32_t channel) {
    this->htim = htim;
    this->pwm_channel = channel;
}

void Motor::encoderInterrupt(){
    if( io.direction )
        io.ticks++;
    else
        io.ticks--;
}

void Motor::periodicIO(unsigned long current_time){
    if (io.direction){
//        analogWrite(pinA, io.demand);
//        analogWrite(pinB, 0);
    } else {
//        analogWrite(pinA, 0);
//        analogWrite(pinB, io.demand);
    }

    //overflow
    if( std::abs(io.ticks) > 2147483647){
        io.ticks = 0;
        io.last_ticks = 0;
    }
    io.delta_time = (current_time - io.last_time) / 1000.0;
    io.delta_ticks = io.ticks - io.last_ticks;
    io.speed = (io.delta_ticks / io.delta_time) * (Constants::kWheelDiameter * 3.14 / (Constants::kEncoderTicksPerRevolution/2) );

    io.last_ticks = io.ticks;
    io.last_time = current_time;

}

// Set the speed of the motor in m/s
void Motor::setSpeed(float speed){

    if( std::abs(speed) < 20 ){
        stop();
        return;
    }

    io.direction = speed > 0;
    float current_speed = pidController.calculate(std::abs(speed), io.speed, io.delta_time);
    float pwm = current_speed / getMaxVelocity() * 50;
    setPWM(pwm);
}

void Motor::setSpeed(float speed, unsigned long current_time){
    if( std::abs(speed) < 0.05 ){
        stop();
        return;
    }
    io.direction = speed > 0;
    float current_speed = pidController.calculate(std::abs(speed), std::abs(io.speed), (current_time - io.pid_last_time));
    float pwm = current_speed / getMaxVelocity() * 255;
    setPWM(pwm);
    io.pid_last_time = current_time;
}

// Set the speed of the motor in PWM
void Motor::setPWM(int pwm){
	// Asegura que esté dentro del rango y actualiza la demanda
//	    io.demand = std::min(std::max(std::abs(pwm), Constants::kMotorMinPWM), 50);

	    if (htim == nullptr) return; // Seguridad en caso de que no se haya llamado setTimer()

	    // Dirección física en los pines
	    if (io.direction) {
	        HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_SET);    // A = HIGH
	        HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_RESET);  // B = LOW
	    } else {
	        HAL_GPIO_WritePin(pinA.port, pinA.pin, GPIO_PIN_RESET);  // A = LOW
	        HAL_GPIO_WritePin(pinB.port, pinB.pin, GPIO_PIN_SET);    // B = HIGH
	    }

	    // Convertimos pwm (0–255) al valor proporcional a ARR del timer
	    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
	    uint16_t duty = (io.demand * arr) / 255;

	    // Aplicamos PWM
	    __HAL_TIM_SET_COMPARE(htim, pwm_channel, duty);
	    HAL_TIM_PWM_Start(htim, pwm_channel);
}

// Get the motor current PWM
int Motor::getPWM(){
    return io.demand;
}

void Motor::stop(){
    io.demand = 0;
//    analogWrite(pinA, 0);
//    analogWrite(pinB, 0);
}

void Motor::hardStop(){
    io.demand = 0;
//    analogWrite(pinA, 255);
//    analogWrite(pinB, 255);
}

float Motor::getMaxVelocity(){
    return Constants::kWheelDiameter * 3.14 * Constants::kMotorsRPM / 60;
}

void Motor::resetEncoder(){
    io.ticks = 0;
    io.last_ticks = 0;
    io.delta_ticks = 0;
    io.speed = 0;
}

long Motor::getTicks(){
    return io.ticks;
}

float Motor::getSpeed(){
    return io.speed;
}

float Motor::getTargetSpeed(){
    return io.target_speed;
}

void Motor::setVerbose(bool verbose){
    this->verbose = verbose;
}
