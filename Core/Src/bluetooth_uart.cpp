#include "bluetooth_uart.h"

void BT_Send(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void sendMotorSpeeds(int pwm, float yaw)
{
    char buffer[64];
    sprintf(buffer,
            "PWM:%d YAW:%d FL:%d FR:%d BL:%d BR:%d\r\n",
			pwm,
			(int)yaw,
            (int)frontLeftMotor.getSpeed(),
            (int)frontRightMotor.getSpeed(),
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
