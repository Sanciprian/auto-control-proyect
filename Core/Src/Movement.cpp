/*
 * Movement.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Diego
 */

#include "Movement.h"


void movementInit(){
	back_right_motor.init(Constants::kBackRightEnable, Constants::kBackRightA, Constants::kBackRightB, Constants::kBackRightEncoder);
	back_left_motor.init(Constants::kBackLeftEnable, Constants::kBackLeftA, Constants::kBackLeftB, Constants::kBackLeftEncoder);
	front_right_motor.init(Constants::kFrontRightEnable, Constants::kFrontRightA, Constants::kFrontRightB, Constants::kFrontRightEncoder);
	front_left_motor.init(Constants::kFrontLeftEnable, Constants::kFrontLeftA, Constants::kFrontLeftB, Constants::kFrontLeftEncoder);
}
void distance(){}
void setSpeed(){}
void stop(){}
