//
// Created by LiuDongPeng on 2023/11/29.
//

#include "encoder.h"



/**
 * @brief Encoder init
 * @param[out]  encoder
 * @param[in]   encoderGetDataHandle
 * @return
 */
int encoder_init(encoder_t* encoder, encoder_get_data_callback_t encoderGetDataHandle)
{
    if (!encoder || !encoderGetDataHandle)
        return -1;

    encoder->encoderGetDataCallback = encoderGetDataHandle;

    return 0;
}


/**
 *
 * @param encoder
 * @param bandwidth
 */
void encoder_update_pll_gains(encoder_t* encoder, float bandwidth)
{
    if (!encoder)
        return;

    encoder->pllKp = 2.0f * bandwidth;
    encoder->pllKi = 0.25f * (encoder->pllKp * encoder->pllKp);

    encoder->pllKp = 20.0f;
    encoder->pllKi = 12.0f;
}

/**
 *
 * @param encoder
 * @param sinTheta
 * @param cosTheta
 * @return
 */
int encoder_pll_update(encoder_t* encoder, float sinTheta, float cosTheta)
{
    if (!encoder)
        return -1;

    float pllSinVal = arm_sin_f32(encoder->pllTheta);
    float pllCosVal = arm_cos_f32(encoder->pllTheta);

    float err = cosTheta * pllCosVal - sinTheta * pllSinVal;
    if (err > M_PI / 6)
        err = M_PI / 6;
    if (err < -M_PI / 6)
        err = -M_PI / 6;
    encoder->pllErr = err;

    encoder->pllIntegral += (encoder->pllKi * err);
    float piOutput = (encoder->pllKp * err) + encoder->pllIntegral;
    encoder->pllSpeedRpm = piOutput * ((60.0f) / (7 * M_TWOPI));

    encoder->pllTheta += encoder->pllSpeedRpm;

    return 0;
}
