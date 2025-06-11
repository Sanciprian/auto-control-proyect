#include "BNOController.h"
#include <cmath>

void BNOController::init()
{
    BNO055_Init_t config;
    config.Unit_Sel = UNIT_ORI_ANDROID | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2 | UNIT_TEMP_CELCIUS;
    config.Axis = DEFAULT_AXIS_REMAP;
    config.Axis_sign = DEFAULT_AXIS_SIGN;
    config.Clock_Source = CLOCK_EXTERNAL;
    config.Mode = BNO055_NORMAL_MODE;
    config.ACC_Range = Range_4G;
    config.OP_Modes = NDOF;

    BNO055_Init(config);

    yawPID.set(Constants::kBNOKP, Constants::kBNOKI, Constants::kBNOKD, Constants::kBNOMinAngular, Constants::KBNOMaxAngular, Constants::kBNON);
}

float BNOController::getYaw()
{
    ReadData(&data, SENSOR_EULER);
    return data.Euler.X; // X es Heading (Yaw) según datasheet
}

void BNOController::setTargetYaw(float yaw)
{
    targetYaw = yaw;
}

float BNOController::computeYawPID(float currentYaw, float dt)
{
    return yawPID.calculate(targetYaw, currentYaw, dt);
}
float BNOController::getYawRate(uint32_t currentTimeMs)
{
    ReadData(&data, SENSOR_EULER);
    float currentYaw = data.Euler.X;
    float dt = (currentTimeMs - lastTime) / 1000.0f;

    // Corrige wrap-around si el ángulo pasa de 360 a 0 o viceversa
    float deltaYaw = currentYaw - lastYaw;
    if (deltaYaw > 180.0f)
        deltaYaw -= 360.0f;
    else if (deltaYaw < -180.0f)
        deltaYaw += 360.0f;

    float yawRate = 0.0f;
    if (dt > 0.0f)
        yawRate = deltaYaw / dt;

    // Actualiza valores anteriores
    lastYaw = currentYaw;
    lastTime = currentTimeMs;

    return yawRate; // En grados por segundo
}

void BNOController::updateYawControl(uint32_t now)
{
    float dt = (now - lastYaw) / 1000.0f;
    if (dt <= 0.0f)
        return;

    float current_yaw = getYaw();

    // Calculamos el error angular mínimo (con wrap-around)
    float error = targetYaw - current_yaw;
    if (error > 180.0f)
    {
        error -= 360.0f;
    }

    if (error < -180.0f)
    {
        error += 360.0f;
    }

    // PID devuelve velocidad angular en °/s
    float output_deg_per_sec = yawPID.calculate(0.0f, -error, dt); // Medimos cómo alejados estamos de 0

    // Limitamos el valor de salida
    output_deg_per_sec = std::min(std::max(output_deg_per_sec, Constants::kBNOMinAngular), Constants::KBNOMaxAngular);
    if (fabs(error) < Constants::kAngleTolerance)
    {
        output_deg_per_sec = 0;
    }
    lastTime = now;
    speed = output_deg_per_sec;
}

float BNOController::getSpeed()
{
    return speed;
}

float BNOController::getYawRad()
{
    ReadData(&data, SENSOR_EULER);
    return data.Euler.X * M_PI / 180.0f; // X es Heading (Yaw) según datasheet
}
