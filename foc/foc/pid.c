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

    pid->ts = 1.0f / 20000.0f;

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
 * @param ref
 * @param fbk
 * @param output
 * @return
 */
int anti_windup_pi_ctrl(pid_ctrl_t* pid, float ref, float fbk, float* output)
{
    if (pid == NULL)
        return -1;

    float err = ref - fbk;
    float u = (pid->kp * err) + (pid->integral) + (pid->ki * err * pid->ts);
    float aw = FOC_CLAMP(u, pid->outLowLimit, pid->outUpLimit) - u;
    pid->integral += (pid->ka * aw) + (err * pid->ts);

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



/**
 *
 * @param pid
 * @param err
 * @param output
 * @return
 */
int pi_ctrl(pid_ctrl_t* pid, float err, float* output)
{
    int ret = 0;

    if (pid == NULL)
        return -1;

    float temp = pid->kp * err;

    pid->integral += err;
    temp += pid->ki * pid->integral;

    // Q2O(q, Qn) ((q) >> (Qn))
    //temp = temp >> 1;

    if (temp > pid->outUpLimit)
    {
        temp = pid->outUpLimit;
        if (err > 0)
        {
            pid->integral -= err;
        }
    }
    else if (temp < pid->outLowLimit)
    {
        temp = pid->outLowLimit;
        if (err < 0)
        {
            pid->integral -= err;
        }
    }
    else
    {}

    pid->output = temp;
    if (output)
    {
        *output = temp;
    }

    return ret;
}

