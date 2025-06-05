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

void movementInit();
void distance();
void setSpeed(int speed);
void stop();
void updateMovement(uint32_t current_time);

// Motors
inline Motor frontLeftMotor;
inline Motor frontRightMotor;
inline Motor backLeftMotor;
inline Motor backRightMotor;

#endif /* MOVvoEMENT_H_ */
