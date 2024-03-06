//
// Created by QT on 2023/11/16.
//

#ifndef FOC_PID_H
#define FOC_PID_H


#include <stdint.h>
#include <stdbool.h>



typedef struct pid_ctrl
{
    float kp;
    float ki;
    float kd;
    float ka, kaChanged;
    float outLowLimit, outUpLimit;

    float lastErr, clampErr;
    float ti;
    float integral, integralMax;
    float saturation;

    float output;

} pid_ctrl_t;


int pid_ctrl_init(pid_ctrl_t* pid, float p, float i, float d, float ka, float lowLimit, float upLimit);
int pid_ctrl_set_limit(pid_ctrl_t* pid, float lowLimit, float upLimit);



int anti_windup_pi_ctrl(pid_ctrl_t* pid, float set, float feedback, float* output);
int position_pid_ctrl(pid_ctrl_t* pid, float set, float feedback, float* output);


#endif //FOC_PID_H
