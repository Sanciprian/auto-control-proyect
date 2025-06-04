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
void setSpeed();
void stop();

// Motors //
extern Motor back_right_motor;
extern Motor back_left_motor;
extern Motor front_right_motor;
extern Motor front_left_motor;

#endif /* MOVvoEMENT_H_ */
