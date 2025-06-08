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
            (int)frontLeftMotor.getTarget(),
            (int)frontRightMotor.getTarget(),
            (int)backLeftMotor.getTarget(),
            (int)backRightMotor.getTarget());

    BT_Send(buffer);
}

void sendYaw(float yaw)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Yaw: %d\r\n", (int)yaw);
    BT_Send(buffer);
}