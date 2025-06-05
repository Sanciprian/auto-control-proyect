/*
 * Movement.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Diego
 */

#include "Movement.h"

void movementInit()
{
    // Motores (Checar que los canales sean los correctos)
    frontLeftMotor.init(
        Constants::kFrontLeftA,
        Constants::kFrontLeftB,
        Constants::kFrontLeftEncoder,
        TIM_CHANNEL_1,
        &htim3);
    frontRightMotor.init(
        Constants::kFrontRightA,
        Constants::kFrontRightB,
        Constants::kFrontRightEncoder,
        TIM_CHANNEL_1,
        &htim1);

    backRightMotor.init(
        Constants::kBackRightA,
        Constants::kBackRightB,
        Constants::kBackRightEncoder,
        TIM_CHANNEL_4,
        &htim3);

    backLeftMotor.init(
        Constants::kBackLeftA,
        Constants::kBackLeftB,
        Constants::kBackLeftEncoder,
        TIM_CHANNEL_2,
        &htim3);
}
void distance() {}
void setSpeed(int speed)
{
    frontLeftMotor.setTarget(speed);
    frontRightMotor.setTarget(speed);
    backLeftMotor.setTarget(speed);
    backRightMotor.setTarget(speed);
}
void stop()
{
    frontLeftMotor.stop_motor();
    frontRightMotor.stop_motor();
    backLeftMotor.stop_motor();
    backRightMotor.stop_motor();
}
void updateMovement(uint32_t current_time)
{
    frontLeftMotor.update_motor(current_time);
    frontRightMotor.update_motor(current_time);
    backLeftMotor.update_motor(current_time);
    backRightMotor.update_motor(current_time);
}