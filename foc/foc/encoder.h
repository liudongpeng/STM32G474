//
// Created by LiuDongPeng on 2023/11/29.
//

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H


#include <stdint.h>

#include "main.h"
#include "arm_math.h"


typedef struct encoder encoder_t;
typedef int (*encoder_get_data_callback_t)(encoder_t* encoder, int* raw, float* angle, float* angleRad);

typedef struct encoder
{
    float theta;

    encoder_get_data_callback_t encoderGetDataCallback;

    /* pll */
    float pllKp, pllKi;
    float pllIntegral;
    float pllErr;
    float pllTheta, pllSpeedRpm;

} encoder_t;


int encoder_init(encoder_t* encoder, encoder_get_data_callback_t encoderGetDataHandle);

void encoder_update_pll_gains(encoder_t* encoder, float bandwidth);
int encoder_pll_update(encoder_t* encoder, float sinTheta, float cosTheta);


#endif //FOC_ENCODER_H
