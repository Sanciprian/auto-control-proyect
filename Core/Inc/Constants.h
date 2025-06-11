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
	inline static constexpr float kWheelTrack = 13.0;	// cambiar a cm
	inline static constexpr float kWheelBase = 0.155;	// meters
	inline static constexpr float kDriveKP = 0.017;
	inline static constexpr float kDriveKD = 0.00;
	inline static constexpr float kDriveKI = 0.00;
	inline static constexpr int kTimeDelay = 10;

	// BNO //
	inline static constexpr float kBNOKP = 1.0;
	inline static constexpr float kBNOKI = 0.0;
	inline static constexpr float kBNOKD = 0.0;
	inline static constexpr float KBNOMaxAngular = 250.0;
	inline static constexpr float kBNOMinAngular = -250.0;
	inline static constexpr float kAngleTolerance = 2.0;
	inline static constexpr float kBNON = 2.0;

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

	// PID // Ya no se usa
	inline static constexpr float kMotorKP = 2.0;
	inline static constexpr float kMotorKI = 0.0;
	inline static constexpr float kMotorKImax = 0.0;
	inline static constexpr float kMotorKD = 0.0;
	inline static constexpr float kMotorMaxOut = 34.0;
	inline static constexpr float kMotorMinOut = 6.0;

	// PWM Quality //
	inline static constexpr float kMaxPWM = 255.0f;
	inline static constexpr float kMinPWM = -255.0f;

	/*
	 *
	 * PID Motores
	 * left right
	 * */
	// FRONT LEFT //
	inline static constexpr float kFrontLeftKP = 0.177164;
	inline static constexpr float kFrontLeftKI = 24.0612;
	inline static constexpr float kFrontLeftKD = -0.00598361;
	inline static constexpr float kFrontLeftN = 27.2220;

	// FRONT RIGHT //
	inline static constexpr float kFrontRightKP = 0.177770;
	inline static constexpr float kFrontRightKI = 22.5228;
	inline static constexpr float kFrontRightKD = -0.00649922;
	inline static constexpr float kFrontRightN = 25.2957;

	// BACK LEFT //
	inline static constexpr float kBackLeftKP = 0.176407;
	inline static constexpr float kBackLeftKI = 22.1301;
	inline static constexpr float kBackLeftKD = -0.00652223;
	inline static constexpr float kBackLeftN = 25.0333;

	// BACK RIGHT //
	inline static constexpr float kBackRightKP = 0.199595;
	inline static constexpr float kBackRightKI = 24.5142;
	inline static constexpr float kBackRightKD = -0.00755871;
	inline static constexpr float kBackRightN = 24.4810;
};
#endif /* INC_CONSTANTS_H_ */
