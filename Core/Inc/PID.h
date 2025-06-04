/*
 * PID.h
 *
 *  Created on: May 29, 2025
 *      Author: Diego
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "Constants.h"

class PID{
    private:
        float last_error = 0;
        float integral = 0;
        float kP;
        float kI;
        float kD;
        float kImax;
        float out_max;
        float out_min;
    public:
        PID();
        void set(float kP, float kI, float kD, float kImax, float out_min, float out_max);
        float calculate(float setpoint, float input, float dt);
};



#endif /* INC_PID_H_ */
