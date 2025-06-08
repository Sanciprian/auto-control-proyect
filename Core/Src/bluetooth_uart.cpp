#include "bluetooth_uart.h"

void BT_Send(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void sendMotorSpeeds()
{
    char buffer[64];
    sprintf(buffer,
            "FL:%d FR:%d BL:%d BR:%d\r\n",
            (int)frontLeftMotor.getSpeed(),
            (int)frontRightMotor.getSpeed(),
            (int)backLeftMotor.getSpeed(),
            (int)backRightMotor.getSpeed());

    BT_Send(buffer);
}