/*
 * Motor.h
 *
 *  Created on: May 29, 2025
 *      Author: Sanci
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "PID.h"
#include "Constants.h"
#include "stm32f1xx_hal.h"       // Para tipos como TIM_HandleTypeDef y GPIO
#include "stm32f1xx_hal_tim.h"   // Específicamente para funciones de TIM (PWM)


class Motor {
private:
        //A(fwd) and B(rev) pins
        Pin enable;
		Pin pinA;
        Pin pinB;
        Pin encoder;
        PID pidController;
        TIM_HandleTypeDef* htim;
        uint32_t pwm_channel;
        bool verbose = false;

        struct periodicIO{
            //INPUT
            volatile long ticks = 0;
            long last_ticks = 0;
            unsigned long last_time = 0;
            unsigned long pid_last_time = 0;
            float delta_ticks = 0;
            float delta_time = 0;
            float speed = 0;
            //OUTPUT
            float target_speed = 0;
            float demand = 0;
            bool direction = 1; //1 = forward, 0 = reverse
        };
        periodicIO io;


    public:
        Motor();
        void init(Pin enable, Pin pinA, Pin pinB, Pin encoder);
        void setTimer(TIM_HandleTypeDef* htim, uint32_t channel);
        void setSpeed(float speed);
        void setSpeed(float speed, unsigned long current_time);
        void setPWM(int pwm);
        int getPWM();
        void stop();
        void hardStop();
        void periodicIO(unsigned long current_time);
        float getMaxVelocity();
        float getTargetSpeed();
        void encoderInterrupt();
        void resetEncoder();
        long getTicks();
        float getSpeed();
        void setVerbose(bool verbose);
};

#endif /* MOTOR_H_ */
