/*
 * Movement.cpp
 *
 *  Created on: May 29, 2025
 *      Author: Diego
 */

#include "Movement.h"
#include <cmath>

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

    // Paso 3: Actualizar los motores físicamente
    frontLeftMotor.update_motor(current_time);
    frontRightMotor.update_motor(current_time);
    backLeftMotor.update_motor(current_time);
    backRightMotor.update_motor(current_time);
}

// Kinematics
void setKinematicSpeeds(float _linear_x_cm_s, uint32_t current_time)
{
    // Paso 1: Calcular salida del PID del BNO como velocidad angular deseada
    bno.updateYawControl(current_time);
    float angular_z_deg_s = bno.getSpeed(); // °/s
    float linear_x_cm_s = _linear_x_cm_s;
    float omega_rad_s = angular_z_deg_s * M_PI / 180.0f;
    if (angular_z_deg_s < 10.0f && angular_z_deg_s > -10.0f && linear_x_cm_s == 0)
    {
        omega_rad_s = 0;
        frontLeftMotor.setTarget(0);
        backLeftMotor.setTarget(0);
        frontRightMotor.setTarget(0);
        backRightMotor.setTarget(0);
        return;
    }
    float L = Constants::kWheelTrack; // track width en cm

    float left_speed = linear_x_cm_s - (omega_rad_s * L / 2.0f);
    float right_speed = linear_x_cm_s + (omega_rad_s * L / 2.0f);

    // if (linear_x_cm_s == 0)
    // {
    //     left_speed *= 1.3;
    //     right_speed *= 1.3;
    // }
    if (left_speed < 0)
    { // Llanatas izquierdas  para atras
        frontLeftMotor.setTarget(left_speed * -1);
        backLeftMotor.setTarget(left_speed * -1);
        frontRightMotor.setTarget(right_speed);
        backRightMotor.setTarget(right_speed);

        frontLeftMotor.setDir(true);
        backLeftMotor.setDir(true);
        frontRightMotor.setDir(false);
        backRightMotor.setDir(false);
    }
    else if (right_speed < 0) // Llantas derechas para atras
    {
        frontLeftMotor.setTarget(left_speed);
        backLeftMotor.setTarget(left_speed);
        frontRightMotor.setTarget(right_speed * -1);
        backRightMotor.setTarget(right_speed * -1);

        frontRightMotor.setDir(true);
        backRightMotor.setDir(true);
        frontLeftMotor.setDir(false);
        backLeftMotor.setDir(false);
    }
    else if (right_speed > 0 && left_speed > 0 && linear_x_cm_s > 0)
    {
        frontLeftMotor.setDir(false);
        backLeftMotor.setDir(false);
        frontRightMotor.setDir(false);
        backRightMotor.setDir(false);
        frontLeftMotor.setTarget(left_speed);
        backLeftMotor.setTarget(left_speed);
        frontRightMotor.setTarget(right_speed);
        backRightMotor.setTarget(right_speed);
    }
}

void updateWithoutPID(uint32_t current_time)
{
    frontLeftMotor.updateWithoutPID(current_time);
    frontRightMotor.updateWithoutPID(current_time);
    backLeftMotor.updateWithoutPID(current_time);
    backRightMotor.updateWithoutPID(current_time);
}

void setRotation(bool dir)
{ // True Right, False, Left
    if (dir)
    {
        frontLeftMotor.forward();
        backLeftMotor.forward();
        frontRightMotor.backward();
        backRightMotor.backward();
    }
    else
    {
        frontLeftMotor.backward();
        backLeftMotor.backward();
        frontRightMotor.forward();
        backRightMotor.forward();
    }
}

void setForward()
{
    frontLeftMotor.forward();
    backLeftMotor.forward();
    frontRightMotor.forward();
    backRightMotor.forward();
}
void setBackward()
{
    frontLeftMotor.backward();
    backLeftMotor.backward();
    frontRightMotor.backward();
    backRightMotor.backward();
}