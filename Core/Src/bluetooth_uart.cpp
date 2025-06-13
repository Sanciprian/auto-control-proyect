#include "bluetooth_uart.h"

void BT_Send(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void sendMotorSpeeds(int pwm, float yaw, float error)
{
    char buffer[64];
    sprintf(buffer,
            "Rate:%d YAW:%d Error:%d FL:%d FR:%d BL:%d BR:%d\r\n",
            (int)pwm,
            (int)yaw,
            (int)error,
            (int)frontLeftMotor.getTarget(),
            (int)frontRightMotor.getTarget(),
            (int)backLeftMotor.getSpeed(),
            (int)backRightMotor.getSpeed());

    BT_Send(buffer);
}

void sendYaw(float yaw)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Yaw: %d\r\n", (int)yaw);
    BT_Send(buffer);
}
