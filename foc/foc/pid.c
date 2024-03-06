//
// Created by LiuDongPeng on 2023/11/16.
//

#include "pid.h"
#include "utils.h"

#include <string.h>
#include <math.h>


int pid_ctrl_init(pid_ctrl_t *pid, float p, float i, float d, float ka, float lowLimit, float upLimit)
{
    if (pid == NULL)
        return -1;

    memset(pid, 0, sizeof(pid_ctrl_t));

    pid->kp = p;
    pid->ki = i;
    pid->ka = ka;
    pid->outLowLimit = lowLimit;
    pid->outUpLimit = upLimit;

    pid->ti = 1.0f / 20000.0f;

    return 0;
}


/**
 *
 * @param pid
 * @param lowLimit
 * @param upLimit
 * @return
 */
int pid_ctrl_set_limit(pid_ctrl_t* pid, float lowLimit, float upLimit)
{
    if (pid == NULL)
        return -1;

    pid->outLowLimit = lowLimit;
    pid->outUpLimit = upLimit;

    return 0;
}


/**
 *
 * @param pid
 * @param set
 * @param feedback
 * @param output
 * @return
 */
int anti_windup_pi_ctrl(pid_ctrl_t* pid, float set, float feedback, float* output)
{
    if (pid == NULL)
        return -1;

    const float dt = 1.0f / 20000;

    float err = set - feedback;
    float u = (pid->kp * err) + (pid->integral) + (pid->ki * err * dt);
    float aw = FOC_CLAMP(u, pid->outLowLimit, pid->outUpLimit) - u;
    pid->integral += (pid->ka * aw) + (err * dt);

    float clampOut = FOC_CLAMP((pid->kp * err) + (pid->integral), pid->outLowLimit, pid->outUpLimit);

    /* Output */
    pid->output = clampOut;
    if (output)
        *output = pid->output;

    return 0;
}


/**
 *
 * @param pid
 * @param set
 * @param feedback
 * @param output
 * @return
 */
int position_pid_ctrl(pid_ctrl_t* pid, float set, float feedback, float* output)
{
    if (pid == NULL)
        return -1;

    float err = set - feedback;
    if (fabsf(err) >= 0.01f)
    {
        pid->integral = FOC_CLAMP(pid->integral + err, -100.0f, 100.0f);
        pid->output = (pid->kp * err) + (pid->ki * pid->integral) + (pid->kd * (err - pid->lastErr));
        pid->lastErr = err;
    }
    else
    {
        pid->output = 0;
    }

    if (output)
        *output = pid->output;

    return 0;
}

