#ifndef BLUETOOTH_UART_H
#define BLUETOOTH_UART_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include "Movement.h"
#include <cstdio>

inline UART_HandleTypeDef huart1;

void BT_Send(const char *msg);
void sendMotorSpeeds();
void sendYaw(float yaw);

#endif // BLUETOOTH_UART_H
