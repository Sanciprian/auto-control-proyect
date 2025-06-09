#ifndef BNO_CONTROLLER_H
#define BNO_CONTROLLER_H

extern "C"
{
#include "BNO055_STM32.h"
}
#include "PID.h"
#include "Constants.h"

// #include "BNO055_STM32.h"

class BNOController
{
public:
  void init();    // Inicializa BNO con configuración estándar NDOF
  float getYaw(); // Retorna heading (Yaw) en grados
  void setTargetYaw(float yaw);
  float computeYawPID(float currentYaw, float dt);
  float getYawRate(uint32_t currentTimeMs);
  void updateYawControl(uint32_t now);
  float getSpeed();
  float getYawRad();

private:
  BNO055_Sensors_t data;
  PID yawPID;
  float targetYaw = 0.0f;
  float lastYaw = 0.0f;
  float speed = 0.0f;
  uint32_t lastTime = 0;
};

#endif // BNO_CONTROLLER_H
