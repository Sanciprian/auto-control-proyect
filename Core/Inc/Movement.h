/*
 * Movement.h
 *
 *  Created on: May 29, 2025
 *      Author: Diego
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "Constants.h"
#include "Motor.h"
#include "BNOController.h"

void movementInit();
void distance();
void setSpeed(int speed);
void stop();
void updateMovement(uint32_t current_time);
void setKinematicSpeeds(float _linear_x_cm_s, uint32_t current_time);
void updateWithoutPID(uint32_t current_time);
void setRotation(bool dir);
void setForward();
void setBackward();

// Motors
inline Motor frontLeftMotor(Constants::kFrontLeftKP, Constants::kFrontLeftKI, Constants::kFrontLeftKD, Constants::kFrontLeftN);
inline Motor frontRightMotor(Constants::kFrontRightKP, Constants::kFrontRightKI, Constants::kFrontRightKD, Constants::kFrontRightN);
inline Motor backLeftMotor(Constants::kBackLeftKP, Constants::kBackLeftKI, Constants::kBackLeftKD, Constants::kBackLeftN);
inline Motor backRightMotor(Constants::kBackRightKP, Constants::kBackRightKI, Constants::kBackRightKD, Constants::kBackRightN);
inline BNOController bno;

#endif /* MOVvoEMENT_H_ */
