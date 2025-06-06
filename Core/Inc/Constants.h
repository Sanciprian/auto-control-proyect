/*
 * Constants.h
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

/*
 *
 * Includes
 *
 * */
#include "stm32f1xx_hal.h"
#include <cstdlib>
#include <cmath>
#include <algorithm>

/*
 *
 * Variables
 *
 * */
inline TIM_HandleTypeDef htim1;
inline TIM_HandleTypeDef htim3;

// Struct //
struct Pin
{
	GPIO_TypeDef *port;
	uint16_t pin;
};

class Constants
{
private:
	/* data */
public:
	// Drive constants //
	inline static constexpr float kWheelDiameter = 7.3; // cm
	inline static constexpr float kWheelTrack = 0.23;	// meters
	inline static constexpr float kWheelBase = 0.155;	// meters
	inline static constexpr float kDriveKP = 0.017;
	inline static constexpr float kDriveKD = 0.00;
	inline static constexpr float kDriveKI = 0.00;

	// BNO //
	inline static constexpr float kBNOKP = 0.1;
	inline static constexpr float kBNOKI = 0.0;
	inline static constexpr float kBNOKD = 1.0;
	inline static constexpr float kBNOKImax = 0.1;
	inline static constexpr float KBNOMaxAngular = 2.0;
	inline static constexpr float kBNOMinAngular = -2.0;
	inline static constexpr float kAngleTolerance = 2.5;

	// Motor General //
	inline static constexpr float kMotorsRPM = 100;														 // meters
	inline static constexpr float kMotorMinPWM = 60;													 // meters
	inline static constexpr float kEncoderTicksPerRevolution = 632;										 // meters
	inline static constexpr float kCMPerTick = (3.14159f * kWheelDiameter) / kEncoderTicksPerRevolution; // ya está en cm

	// Enable pins (PWM) //
	inline static constexpr Pin kFrontLeftEnable = {GPIOA, GPIO_PIN_6};
	inline static constexpr Pin kFrontRightEnable = {GPIOA, GPIO_PIN_8};
	inline static constexpr Pin kBackLeftEnable = {GPIOA, GPIO_PIN_7};
	inline static constexpr Pin kBackRightEnable = {GPIOB, GPIO_PIN_1};

	// Direction pins (A: fwd, B: bck) //
	inline static constexpr Pin kFrontLeftA = {GPIOB, GPIO_PIN_4};
	inline static constexpr Pin kFrontLeftB = {GPIOA, GPIO_PIN_11};
	inline static constexpr Pin kFrontRightA = {GPIOB, GPIO_PIN_10};
	inline static constexpr Pin kFrontRightB = {GPIOB, GPIO_PIN_0};
	inline static constexpr Pin kBackLeftA = {GPIOB, GPIO_PIN_13};
	inline static constexpr Pin kBackLeftB = {GPIOB, GPIO_PIN_14};
	inline static constexpr Pin kBackRightA = {GPIOB, GPIO_PIN_12};
	inline static constexpr Pin kBackRightB = {GPIOB, GPIO_PIN_15};

	// Encoder pins (one per encoder) //
	inline static constexpr uint16_t kFrontLeftEncoder = GPIO_PIN_1;
	inline static constexpr uint16_t kFrontRightEncoder = GPIO_PIN_0;
	inline static constexpr uint16_t kBackLeftEncoder = GPIO_PIN_2;
	inline static constexpr uint16_t kBackRightEncoder = GPIO_PIN_3;

	// PID //
	inline static constexpr float kMotorKP = 1;
	inline static constexpr float kMotorKI = 1.8;
	inline static constexpr float kMotorKImax = 0.0;
	inline static constexpr float kMotorKD = 0.0;
	inline static constexpr float kMotorMaxOut = 10.0;
	inline static constexpr float kMotorMinOut = 0.0;

	// PWM Quality //
	inline static constexpr float kMaxPWM = 255.0f;
	inline static constexpr float kMinPWM = 153.0f;
};

#endif /* INC_CONSTANTS_H_ */
