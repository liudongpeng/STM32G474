//
// Created by LiuDongPeng on 2023/11/16.
//

#include "motor.h"
#include "foc.h"

#include "tim.h"

#include "digital_filter.h"
#include "utils.h"


int motor_create(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    memset(motor, 0, sizeof(motor_t));

    motor->VBus = 12.0f;
    motor->timArr = FOC_TIM1_ARR;
    motor->encoderCpr = 16384;


    /* 三环控制频率 */
    motor->currentLoopFreq = 20000;
    motor->speedLoopFreq = 5000;
    motor->positionLoopFreq = 1000;

    /* HFI 参数 */
    motor->hfiIdOffsetSampleCnt = 20;
    motor->hfiVoltAmpl = 12.0f * 0.1f;
    motor->hfiCurrAmpl = 0.3f;
    motor->sensorless = true;
    pid_ctrl_init(&motor->hfiPll.pid, 3000.0f, 47000.0f, 0, 0, -2000.0f, 2000.0f);
    lpf_init(&motor->hfiSpeedEstLpf, 200.0f, 1.0f / 20000.0f);

//    motor_set_id_ref(motor, 0.3f);
    motor_set_speed(motor, 300.0f);

    /* 电机控制模式 */
//    motor_set_ctrl_mode(motor, MOTOR_CTRL_MODE_VOLT_ENCODER);
//    motor_set_ctrl_mode(motor, MOTOR_CTRL_MODE_VOLT_ANGLE_INC);
//    motor_set_ctrl_mode(motor, MOTOR_CTRL_MODE_CURRENT);
    motor_set_ctrl_mode(motor, MOTOR_CTRL_MODE_CURRENT_SPEED);
//    motor_set_ctrl_mode(motor, MOTOR_CTRL_MODE_CURRENT_SPEED_POSITION);
    motor_set_status(motor, MOTOR_STATUS_STOP);

    // ZD2808航模电机
    if (true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 2.073303f;
        motor->polePairs = 7;
        motor->Rs = 105.5f / 1000.0f;
        motor->Ld = 33.21f / 1e6f;
    }

    // 2312S航模电机
    if (!true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 2.297776f;
        motor->polePairs = 7;
        motor->Rs = 0.11247408f;
        motor->Ld = 0.00002235f;
        motor->Lq = 0.00002567f;
    }

    // 2804云台电机
    if (!true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 2.297776f;
        motor->polePairs = 7;
        motor->Rs = 1.9f;
        motor->Ld = 2.6f / 1000.0f;
    }

    // 2806云台电机
    if (!true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 4.081667f;
        motor->polePairs = 7;
        motor->Rs = 3.3f;
        motor->Ld = 2.6f / 1000.0f;
    }

    // 3510云台电机
    if (!true)
    {
        motor->encoderDir = 1;
        motor->angleRadOffset = 1.613109f;
        motor->polePairs = 11;
        motor->Rs = 7.8f;
        motor->Ld = 3.95f / 1000.0f;
    }

    // 6824云台电机
    if (!true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 2.297776f;
        motor->polePairs = 11;
        motor->Rs = 2.3613f;
        motor->Ld = 210.0f / 1e6f;
    }

    // 5010电机
    if (!true)
    {
        motor->encoderDir = -1;
        motor->angleRadOffset = 2.297776f;
        motor->polePairs = 7;
        motor->Rs = 0.12f;
        motor->Ld = 50.0f / 1e6f;
    }

    /* 相电流低通滤波器初始化 */
    lpf_init(&motor->iaLpFilter, 1000.0f, 1.0f / 20e3f);
    lpf_init(&motor->ibLpFilter, 1000.0f, 1.0f / 20e3f);
    lpf_init(&motor->icLpFilter, 1000.0f, 1.0f / 20e3f);

    /* 电流环PI参数 */
    float kp = motor->Ld * 500;
    float ki = motor->Rs * 500;
    float kd = 0;
    float upLimit = motor->VBus / SQRT3 * 0.9f; // uq
    pid_ctrl_init(&motor->idPid, kp, ki, 0, ki / kp, -upLimit, upLimit);
    pid_ctrl_init(&motor->iqPid, kp, ki, 0, ki / kp, -upLimit, upLimit);

    /* 电机转速PI控制器初始化 */
    upLimit = 6.0f; // iq
//    upLimit = 10.0f; // iq
    kp = 0.004f;
    ki = 0.001f;
    pid_ctrl_init(&motor->speedPid, kp, ki, 0, ki / kp, -upLimit, upLimit);
    motor->speedAcc = 0.1f;
    motor->isUseSpeedRamp = true;

    /* 电机转速计算PLL初始化 */
    upLimit = 10000.0f; // speed
    kp = 0.05f;
    ki = 0.0f;
    foc_pll_init(&motor->speedPll, kp, ki, 0, 0, -upLimit, upLimit);

    /* 电机位置PI控制器初始化 */
    upLimit = 600.0f; // speedRpm
    kp = 5.5f;
    ki = 0.1f;
    kd = 0;
    pid_ctrl_init(&motor->positionPid, kp, ki, kd, 0, -upLimit, upLimit);

    return 0;
}

/**
 *
 * @param motor
 */
void motor_enable(motor_t *motor)
{
    if (motor == NULL)
        return;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 *
 * @param motor
 */
void motor_disable(motor_t *motor)
{
    if (motor == NULL)
        return;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief Set motor status
 * @param[out]  motor
 * @param[in]   status
 */
void motor_set_status(motor_t *motor, motor_status_t status)
{
    if (motor != NULL)
    {
        motor->status = status;
    }
}

/**
 *
 * @param motor
 * @param mode
 */
void motor_set_ctrl_mode(motor_t *motor, motor_ctrl_mode_t mode)
{
    if (motor != NULL)
    {
        motor->ctrlMode = mode;
    }
}

/**
 * @brief Check if the motor has an encoder
 * @param motor
 * @return
 */
bool motor_has_encoder(motor_t *motor)
{
    return motor->encoder != NULL;
}

/**
 * @brief Link encoder to get motor angle
 * @param motor
 * @param encoder
 * @return
 */
bool motor_link_encoder(motor_t *motor, encoder_t *encoder)
{
    if (motor == NULL || encoder == NULL)
        return false;

    motor->encoder = encoder;
    return true;
}

/**
 *
 * @param motor
 * @return
 */
encoder_t *motor_get_encoder(motor_t *motor)
{
    if (motor_has_encoder(motor))
        return motor->encoder;
    else
        return NULL;
}


/**
 *
 * @param motor
 * @return
 */
float motor_get_angle(motor_t *motor)
{
    if (motor != NULL && motor_has_encoder(motor))
    {
        const encoder_get_data_callback_t func = motor->encoder->encoderGetDataCallback;
        if (func)
        {
            func(motor->encoder, &motor->encoderRawData, &motor->angle, &motor->angleRad);
        }
        return motor->angle;
    }
    return 0;
}

/**
 * @brief Get motor angle, record tick and angle
 * @param motor
 * @return
 */
float motor_get_angle_rad(motor_t *motor)
{
    if (motor != NULL && motor_has_encoder(motor))
    {
        const encoder_get_data_callback_t func = motor->encoder->encoderGetDataCallback;
        if (func)
        {
            func(motor->encoder, &motor->encoderRawData, &motor->angle, &motor->angleRad);
        }

        return motor->angleRad;
    }
    return 0;
}

/**
 *
 * @param motor
 * @return
 */
uint16_t motor_get_encoder_raw_data(motor_t *motor)
{
    if (motor != NULL && motor_has_encoder(motor))
    {
        encoder_get_data_callback_t func = motor->encoder->encoderGetDataCallback;
        if (func)
        {
            func(motor->encoder, &motor->encoderRawData, &motor->angle, &motor->angleRad);
        }

        return motor->encoderRawData;
    }
    return 0;
}

/**
 * @brief Get motor velocity in r/min
 * @param motor
 * @return
 */
float motor_get_speed_rpm(motor_t *motor)
{
    if (motor == NULL)
        return 0;


    return 0;
}


/**
 *
 * @param motor
 * @param idRef
 */
void motor_set_id_ref(motor_t *motor, float idRef)
{
    if (motor)
    {
        motor->idRef = idRef;
    }
}

/**
 *
 * @param motor
 * @param iqRef
 */
void motor_set_iq_ref(motor_t *motor, float iqRef)
{
    if (motor)
    {
        motor->iqRef = iqRef;
    }
}

/**
 *
 * @param motor
 * @param id
 */
void motor_set_id(motor_t *motor, float id)
{
    if (motor)
    {
        motor->id = id;
    }
}

/**
 *
 * @param motor
 * @param iq
 */
void motor_set_iq(motor_t *motor, float iq)
{
    if (motor)
    {
        motor->iq = iq;
    }
}


/**
 * @brief Set 3 channel pwm duty
 * @param motor
 * @param ta
 * @param tb
 * @param tc
 */
void motor_set_and_apply_pwm_duty(motor_t *motor, float ta, float tb, float tc)
{
    if (motor == NULL)
        return;

    motor->ta = ta;
    motor->tb = tb;
    motor->tc = tc;

    motor->ccra = (int) ((float) motor->timArr * ta);
    motor->ccrb = (int) ((float) motor->timArr * tb);
    motor->ccrc = (int) ((float) motor->timArr * tc);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->ccra);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor->ccrb);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor->ccrc);
}

/**
 * @brief Get elec angle
 * @param motor
 * @return
 */
float motor_get_elec_angle(motor_t *motor)
{
    if (motor == NULL)
        return 0;

    motor_get_angle_rad(motor);

    if (motor->encoderDir == -1)
    {
        motor->angleRad = (float) M_TWOPI - motor->angleRad;
//        motor->encoderRawData = motor->encoderCpr - motor->encoderRawData;
//        motor->angle = 360.0f - motor->angle;
    }

    float val;
    if (motor->angleRad >= motor->angleRadOffset)
    {
        val = motor->angleRad - motor->angleRadOffset;
    }
    else
    {
        val = (float) M_TWOPI - motor->angleRadOffset + motor->angleRad;
    }
    motor->theta = fmodf(val * (float) motor->polePairs, (float) M_TWOPI);

    return motor->theta;
}


/**
 *
 * @param motor
 */
void motor_run(motor_t *motor)
{
    if (motor == NULL)
        return;

    int valid = 0;

//    valid = foc_mid_point_svm(motor->ualpha, motor->ubeta, motor->VBus, &motor->ta, &motor->tb, &motor->tc);

    valid = odriver_svm(motor->ualpha, motor->ubeta, &motor->ta, &motor->tb, &motor->tc, &motor->sector);
    if (valid)
    {
        motor_set_and_apply_pwm_duty(motor, motor->ta, motor->tb, motor->tc);
    }
}


/**
 *
 * @param motor
 * @param theta
 * @param ud
 * @param uq
 */
void motor_drag_theta(motor_t *motor, float theta, float ud, float uq)
{
    foc_inv_park(ud, uq, arm_sin_f32(theta), arm_cos_f32(theta), &motor->ualpha, &motor->ubeta);
    int valid = foc_mid_point_svm(motor->ualpha, motor->ubeta, motor->VBus, &motor->ta, &motor->tb, &motor->tc);
    if (valid)
    {
        motor_set_and_apply_pwm_duty(motor, motor->ta, motor->tb, motor->tc);
    }
}


/**
 * @brief Align electricity zero pos
 * @param motor
 */
void motor_align_encoder(motor_t *motor, float ud, float angle)
{
    if (motor == NULL)
        return;

    motor_enable(motor);

    float offset1 = 0, offset2 = 0, offset3 = 0;

    // 第一次定位到0°
    motor_drag_theta(motor, 0, ud, 0);
    HAL_Delay(500);
    offset1 = motor_get_angle_rad(motor);

    // 定位到+90°
    motor_drag_theta(motor, 0.5f * M_PI, ud, 0);
    HAL_Delay(500);

    // 第二次定位到0°
    motor_drag_theta(motor, 0, ud, 0);
    HAL_Delay(500);
    offset2 = motor_get_angle_rad(motor);

    // 定位到-90°
    motor_drag_theta(motor, -0.5f * M_PI, ud, 0);
    HAL_Delay(500);

    // 第三次定位到0°
    motor_drag_theta(motor, 0, ud, 0);
    HAL_Delay(500);
    offset3 = motor_get_angle_rad(motor);

    motor->angleRadOffset = (offset1 + offset2 + offset3) / 3.0f;
//    motor->angleRadOffset = offset1;

    motor_disable(motor);
}


/**
 *
 * @param motor
 * @param ud
 * @return
 */
int motor_calib_encoder(motor_t *motor, float ud)
{
    int angRawTable[12];

    motor_enable(motor);

    /* 正转 */
    for (int i = 0; i < 12; i++)
    {
        motor_drag_theta(motor, M_PI * i / 6, ud, 0);
        HAL_Delay(500);

        angRawTable[i] = motor_get_encoder_raw_data(motor) << 2;
    }


    motor_drag_theta(motor, 0, ud, 0);
    HAL_Delay(500);

    for (int i = 11; i >= 0; i--)
    {
        motor_drag_theta(motor, M_PI * i / 6, ud, 0);
        HAL_Delay(500);

        angRawTable[i] = (angRawTable[i] + (motor_get_encoder_raw_data(motor) << 2)) / 2;
    }

    motor_disable(motor);

    /* 处理溢出 */
    for (int i = 0; i < 11; i++)
    {
        int diff = angRawTable[i] - angRawTable[i + 1];
        if (diff > INT16_MAX)
            angRawTable[i + 1] += 65536;
        else if (diff < INT16_MIN)
            angRawTable[i + 1] -= 65536;
    }

    /* 线性回归 */
    float a, b;
    float q = LeastSquareLinearFit(angRawTable, 12, &a, &b);
    if (q > 40)
    {
        return -1;
    }

    int16_t pole_pairs = 1 / (fabsf(b) * 2 * M_PI / 65536.0f) + 0.5f;
    uint16_t offset = a + 0.5f;
    if (b < 0)
    {
        offset += 65536.0f / 2 / pole_pairs;
        pole_pairs = -pole_pairs;
    }

    motor->encoderOffset = offset;
    motor->polePairs = pole_pairs;

    return 0;
}


/**
 * @brief
 * @param motor
 * @param ud
 * @param uq
 */
void motor_open_loop_test(motor_t *motor, float ud, float uq)
{
    if (motor == NULL)
        return;

    /* 1. clark */
    foc_clark(motor->ia, motor->ib, &motor->ialpha, &motor->ibeta);

    /* 2. park */
    foc_park(motor->ialpha, motor->ibeta, motor->sinTheta, motor->cosTheta, &motor->id, &motor->iq);

//    const float voltage_normalize = 1.5f / motor->VBus;
//    motor->ud *= voltage_normalize;
//    motor->uq *= voltage_normalize;

    /* 4. inv park */
    foc_inv_park(ud, uq, motor->sinTheta, motor->cosTheta, &motor->ualpha, &motor->ubeta);

    /* 5. svpwm */
    int valid = 0;
    valid = foc_mid_point_svm(motor->ualpha, motor->ubeta, motor->VBus, &motor->ta, &motor->tb, &motor->tc);
//    valid = odriver_svm(motor->ualpha, motor->ubeta, &motor->ta, &motor->tb, &motor->tc, &motor->sector);
    if (valid)
    {
        motor_set_and_apply_pwm_duty(motor, motor->ta, motor->tb, motor->tc);
    }
}

/**
 * @brief Current pi ctrl
 * @param[in]   motor
 * @return
 */
int odriver_current_pi_ctrl(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    float mod_to_V = (2.0f / 3.0f) * motor->VBus;
    float V_to_mod = 1.0f / mod_to_V;

    float idErr = motor->idRef - motor->id;
    float iqErr = motor->iqRef - motor->iq;

    float ud = V_to_mod * (motor->idPid.integral + idErr * motor->idPid.kp);
    float uq = V_to_mod * (motor->iqPid.integral + iqErr * motor->iqPid.kp);

    float modScalefactor = 0.8f * SQRT3_DIV_2 / sqrtf(ud * ud + uq * uq);
    if (modScalefactor < 1.0f)
    {
        ud *= modScalefactor;
        uq *= modScalefactor;
        motor->idPid.integral *= 0.99f;
        motor->iqPid.integral *= 0.99f;
    }
    else
    {
        motor->idPid.integral += idErr * (motor->idPid.ki / 20e3f);
        motor->iqPid.integral += iqErr * (motor->iqPid.ki / 20e3f);
    }

    motor->ud = ud;
    motor->uq = uq;

    return 0;
}

/**
 * @brief Current closed loop
 * @param motor
 * @return
 */
int motor_current_closed_loop(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    /* 1. clark */
    foc_clark(motor->ia, motor->ib, &motor->ialpha, &motor->ibeta);

    /* 2. park */
    foc_park(motor->ialpha, motor->ibeta, motor->sinTheta, motor->cosTheta, &motor->id, &motor->iq);

    /* 3. id, iq pid ctrl */
    const float voltage_normalize = 1.5f / motor->VBus;
//    motor->ud *= voltage_normalize;
//    motor->uq *= voltage_normalize;
//    odriver_current_pi_ctrl(motor);

    anti_windup_pi_ctrl(&motor->idPid, motor->idRef, motor->id, &motor->ud);
    anti_windup_pi_ctrl(&motor->iqPid, motor->iqRef, motor->iq, &motor->uq);

    /* 4. inv park */
    foc_inv_park(motor->ud, motor->uq, motor->sinTheta, motor->cosTheta, &motor->ualpha, &motor->ubeta);

    /* 5. svpwm */
    int valid = 0;
//    valid = foc_mid_point_svm(motor->ualpha, motor->ubeta, motor->VBus, &motor->ta, &motor->tb, &motor->tc);
    valid = odriver_svm(motor->ualpha, motor->ubeta, &motor->ta, &motor->tb, &motor->tc, &motor->sector);
    if (valid)
    {
        motor_set_and_apply_pwm_duty(motor, motor->ta, motor->tb, motor->tc);

        /* 6. Recover ud uq */
        motor->ud /= voltage_normalize;
        motor->uq /= voltage_normalize;
    }

    return 0;
}


/**
 *
 * @param motor
 * @param speedRef
 */
void motor_set_speed(motor_t *motor, float speedRef)
{
    if (motor)
    {
        motor->speedRef = speedRef;
    }
}

/**
 * @brief Speed closed loop.
 * @param motor
 * @param speedRef
 * @param iqRef
 * @return
 */
int motor_speed_closed_loop(motor_t *motor, float speedRpmRef, float speedRpmFbk, float *iqRef)
{
    if (motor == NULL)
        return -1;

//    pid_ctrl_speed_calc(&motor->speedPid, speedRef, motor->speedRpm, &motor->speedPid.output);
    int ret = anti_windup_pi_ctrl(&motor->speedPid, speedRpmRef, speedRpmFbk, &motor->speedPid.output);

    if (iqRef)
        *iqRef = motor->speedPid.output;

    return ret;
}

/**
 *
 * @param motor
 * @param speedRampTime
 */
void motor_set_speed_ramp_time(motor_t *motor, float speedRampTime)
{
    if (motor)
    {
        motor->speedRampTime = speedRampTime;
    }
}

/**
 *
 * @param motor
 * @param speedAcc
 */
void motor_set_speed_acc(motor_t *motor, float speedAcc)
{
    if (motor)
    {
        motor->speedAcc = speedAcc;
    }
}


/**
 * @brief Speed closed loop.
 * @param motor
 * @param posRef
 * @param speedRef
 * @return
 */
int motor_position_closed_loop(motor_t *motor, float posRef, float *speedRef)
{
    if (motor == NULL)
        return -1;

    int ret = 0;
    pid_ctrl_t *pid = &motor->positionPid;

    float err = posRef - motor->angle;
    if (err > 180.0f)
        err -= 360.0f;
    else if (err < -180.0f)
        err += 360.0f;
    else
        err;


//    pi_ctrl(pid, err, &pid->output);

    float out = (pid->kp * err) + pid->integral;
    float clampOut = FOC_CLAMP(out, pid->outLowLimit, pid->outUpLimit);
    pid->integral += (pid->ki * err) + (pid->ka * (clampOut - out));
    pid->output = clampOut;

    if (speedRef)
        *speedRef = pid->output;

    return ret;
}

/**
 *
 * @param motor
 * @param angle
 */
void motor_set_position(motor_t *motor, float angle)
{
    if (motor)
    {
        motor->positionRef = angle;
    }
}



/**
 *
 * @param motor
 * @param n
 * @param dir
 * @return
 */
int motor_round_to_angle(motor_t *motor, float n, int dir, float* targetAngle)
{
    int ret = 0;

    if (motor == NULL)
        return -1;

    *targetAngle = n * 360.0f;

    return ret;
}

/**
 *
 * @param motor
 * @param n
 * @param dir
 * @return
 */
void motor_set_round(motor_t *motor, float n, int dir)
{
    if (motor)
    {
        motor->roundRef = n;
        motor->dir = dir;
    }
}


/**
 *
 * @param motor
 * @return
 */
int motor_meas_phase_resistance(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    int ret = 0;
    float ud = 0, uq = 0;
    float is = 0;
    float rs = 0;

    motor_enable(motor);

    const float udMax = 12.0f;
    const float isMax = 4.0f;

    do
    {
        /* 1. 设置电角度为0，令vd = 0, vq = 0 */
        motor_drag_theta(motor, 0, ud, uq);
        ud += 0.1f;
        HAL_Delay(10);

        /* 2. 电流采样，进行Clark变换，is = sqrt(ialpha^2 + ibeta^2)，r = vd / is */
        motor->get_phase_current(motor);
        foc_clark(motor->ia, motor->ib, &motor->ialpha, &motor->ibeta);
        is = sqrtf(powf(motor->ialpha, 2) + powf(motor->ibeta, 2));
    } while (is < isMax && ud < udMax);

    HAL_Delay(5000);
    motor->get_phase_current(motor);
    foc_clark(motor->ia, motor->ib, &motor->ialpha, &motor->ibeta);
    is = sqrtf(powf(motor->ialpha, 2) + powf(motor->ibeta, 2));

    /* r = vd / is */
    rs = ud / is;

//    printf("ud: %f, is: %f, rs: %f\n", ud, is, rs);

    motor_disable(motor);

    motor->Rs = rs;
    return ret;
}

/**
 *
 * @param motor
 * @return
 */
int motor_meas_phase_resistance2(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    int ret = 0;
    float ra, rb;
    float duty = 0.2f;

    motor_enable(motor);

    // Ra
    motor_set_and_apply_pwm_duty(motor, duty, 0, 0);
    HAL_Delay(1000);
    motor->get_vbus_voltage(motor);
    motor->get_phase_current(motor);
    ra = motor->VBus * duty / -motor->ia / 1.5f;

    // Delay
    motor_set_and_apply_pwm_duty(motor, 0, 0, 0);
    HAL_Delay(1000);

    // Rb
    motor_set_and_apply_pwm_duty(motor, 0, duty, 0);
    HAL_Delay(1000);
    motor->get_vbus_voltage(motor);
    motor->get_phase_current(motor);
    rb = motor->VBus * duty / -motor->ib / 1.5f;

    motor->Rs = (ra + rb);

    motor_disable(motor);
    return ret;
}


/**
 *
 * @param motor
 * @param thetaErr
 * @return
 */
int motor_hfi_pi_calc(motor_t *motor, float thetaErr)
{
    int ret = 0;

    if (motor == NULL)
        return -1;

    pid_ctrl_t *pid = &motor->hfiPll.pid;

    float we = pid->kp * thetaErr + pid->integral;

    motor->hfiThetaEst = motor->hfiPll.integral;
    motor->hfiSpeedEst = we;

    if (motor->hfiPll.integral >= M_TWOPI)
    {
        motor->hfiPll.integral -= M_TWOPI;
    }
    else if (motor->hfiPll.integral < 0)
    {
        motor->hfiPll.integral += M_TWOPI;
    }
    else
    {}

    pid->integral += pid->ki * thetaErr * pid->ts;
    motor->hfiPll.integral += we * pid->ts;

    return ret;
}


/**
 * @brief
 * @param motor
 * @return
 */
int motor_ctrl_mode_loop(motor_t *motor)
{
    int ret = 0;

    if (motor == NULL)
        return -1;

    switch (motor->ctrlMode)
    {
        case MOTOR_CTRL_MODE_VOLT_ANGLE_INC:
        {
            motor->theta += 0.01f;
            if (motor->theta > M_TWOPI)
            {
                motor->theta -= M_TWOPI;
            }

            motor->sinTheta = arm_sin_f32(motor->theta);
            motor->cosTheta = arm_cos_f32(motor->theta);

            motor_open_loop_test(motor, motor->ud, motor->uq);
            break;
        }

        case MOTOR_CTRL_MODE_VOLT_ENCODER:
        {
            motor->sinTheta = arm_sin_f32(motor->theta);
            motor->cosTheta = arm_cos_f32(motor->theta);
            motor_open_loop_test(motor, motor->ud, motor->uq);
            break;
        }

        case MOTOR_CTRL_MODE_CURRENT_SPEED_POSITION:
        {
            if (++motor->positionLoopCnt >= motor->currentLoopFreq / motor->positionLoopFreq)
            {
                motor->positionLoopCnt = 0; // reset counter

                //motor_round_to_angle(motor, motor->roundRef, 1, &motor->positionRef);
                motor_position_closed_loop(motor, motor->positionRef, &motor->speedRef);

            }
        }

        case MOTOR_CTRL_MODE_CURRENT_SPEED:
        {
            if (++motor->speedLoopCnt >= motor->currentLoopFreq / motor->speedLoopFreq)
            {
                motor->speedLoopCnt = 0; // reset counter

                /* Speed ramp */
                if (motor->isUseSpeedRamp)
                {
                    if (motor->speedRef > motor->speedShadow)
                    {
                        motor->speedShadow += motor->speedAcc;
                        if (motor->speedShadow > motor->speedRef)
                            motor->speedShadow = motor->speedRef;
                    }
                    else if (motor->speedRef < motor->speedShadow)
                    {
                        motor->speedShadow -= motor->speedAcc;
                        if (motor->speedShadow < motor->speedRef)
                            motor->speedShadow = motor->speedRef;
                    }
                    else
                    {}
                }

                float speedRpm;
                if (motor->sensorless)
                    speedRpm = motor->hfiSpeedRpm;
                else
                    speedRpm = motor->speedRpm;

                motor_speed_closed_loop(motor, motor->speedShadow, speedRpm, &motor->iqRef);
            }
        }

        case MOTOR_CTRL_MODE_CURRENT:
        default:
        {
            /* 1. Clark */
            foc_clark(motor->ia, motor->ib, &motor->ialpha, &motor->ibeta);

            /* HFI 位置、转速观测器 */
            if (motor->sensorless)
            {
                /* 提取高频信号 */
                motor->hfiIalpha = 0.5f * (motor->ialpha - motor->ialphaLast);
                motor->hfiIbeta = 0.5f * (motor->ibeta - motor->ibetaLast);
//                motor->hfiIalpha = 0.5f * (-motor->ialpha + motor->ialphaLast);
//                motor->hfiIbeta = 0.5f * (-motor->ibeta + motor->ibetaLast);
                motor->ialphaLast = motor->ialpha;
                motor->ibetaLast = motor->ibeta;

                /* 高频电流差值 */
                motor->hfiIalphaDiff = motor->hfiIalpha - motor->hfiIalphaLast;
                motor->hfiIbetaDiff = motor->hfiIbeta - motor->hfiIbetaLast;
                motor->hfiIalphaLast = motor->hfiIalpha;
                motor->hfiIbetaLast = motor->hfiIbeta;

                /* 包络线 */
                int udhSign = sign(motor->hfiUdOffset);
                motor->hfiIalphaEnvelope = motor->hfiIalphaDiff * (float) udhSign;
                motor->hfiIbetaEnvelope = motor->hfiIbetaDiff * (float) udhSign;

                /* 矢量叉乘 */
                motor->hfiThetaDiff = motor->hfiIbetaEnvelope * arm_cos_f32(motor->hfiThetaEst) -
                                      motor->hfiIalphaEnvelope * arm_sin_f32(motor->hfiThetaEst);

                /* 位置误差跟踪 */
                motor_hfi_pi_calc(motor, motor->hfiThetaDiff);
                motor->hfiSpeedEst = lpf_work(&motor->hfiSpeedEstLpf, motor->hfiSpeedEst);
                motor->hfiSpeedRpm = motor->hfiSpeedEst * 9.55f / (float) motor->polePairs;
                motor->hfiTheta = motor->hfiThetaEst;
            }

            /* 使用编码器角度还是无感观测出的角度 */
            if (motor->sensorless)
            {
                motor->sinTheta = arm_sin_f32(motor->hfiTheta);
                motor->cosTheta = arm_cos_f32(motor->hfiTheta);
            }
            else
            {
                motor->sinTheta = arm_sin_f32(motor->theta);
                motor->cosTheta = arm_cos_f32(motor->theta);
            }

            /* 2. Park */
            foc_park(motor->ialpha, motor->ibeta, motor->sinTheta, motor->cosTheta, &motor->id, &motor->iq);

            /* 3. id, iq pid ctrl */
            const float voltage_normalize = 1.5f / motor->VBus;

            /* HFI 提取基频信号 */
            if (motor->sensorless)
            {
                motor->hfiId = 0.5f * (motor->id + motor->idLast);
                motor->hfiIq = 0.5f * (motor->iq + motor->iqLast);
                motor->idLast = motor->id;
                motor->iqLast = motor->iq;

                motor_set_id(motor, motor->hfiId);
                motor_set_iq(motor, motor->hfiIq);
            }

            /* HFI 磁极辨识 id偏置注入 */
            if (true && motor->sensorless && !motor->isHfiNsiOvered)
            {
                static int nsiCnt = 0; // 磁极辨识计数器
                static float idOffsetSumP = 0, idOffsetSumN = 0; // 累积的正负idh

                nsiCnt++;
                if (nsiCnt <= 200)
                {
                    motor_set_id_ref(motor, motor->hfiCurrAmpl);
                }
                else if (nsiCnt > 200 && nsiCnt <= 400)
                {
                    idOffsetSumP += fabsf(motor->hfiId * motor->hfiCurrAmpl);
                }
                else if (nsiCnt > 400 && nsiCnt <= 600)
                {
                    motor_set_id_ref(motor, 0);
                }
                else if (nsiCnt > 600 && nsiCnt <= 800)
                {
                    motor_set_id_ref(motor, -motor->hfiCurrAmpl);
                }
                else if (nsiCnt > 800 && nsiCnt <= 1000)
                {
                    idOffsetSumN += fabsf(motor->hfiId * -motor->hfiCurrAmpl);
                }
                else
                {
                    nsiCnt = 0;
                    motor_set_id_ref(motor, 0);

                    /* 磁极判断 */
//                    motor->isHfiNsiOvered = true;
                    if (idOffsetSumN < idOffsetSumP)
                    {
                        motor->hfiThetaEst += M_PI;
                        if (motor->hfiThetaEst > M_TWOPI)
                            motor->hfiThetaEst -= M_TWOPI;
                    }
                    idOffsetSumP = idOffsetSumN = 0;
                }
            }
            odriver_current_pi_ctrl(motor);

            /* HFI 无传感器模式，注入高频方波信号 */
            static bool isDirChanged = true;
            if (motor->sensorless)
            {
                if (isDirChanged)
                {
                    motor->hfiUdOffset = motor->hfiVoltAmpl * voltage_normalize;
                }
                else
                {
                    motor->hfiUdOffset = motor->hfiVoltAmpl * voltage_normalize * -1;
                }
                motor->ud += motor->hfiUdOffset;
                isDirChanged = !isDirChanged;
            }

            /* 4. Inv park */
            foc_inv_park(motor->ud, motor->uq, motor->sinTheta, motor->cosTheta, &motor->ualpha, &motor->ubeta);
            /* 5. SVM */
            ret = odriver_svm(motor->ualpha, motor->ubeta, &motor->ta, &motor->tb, &motor->tc, &motor->sector);
            if (ret)
            {
                motor_set_and_apply_pwm_duty(motor, motor->ta, motor->tb, motor->tc);

                /* 6. Recover ud uq */
                motor->ud /= voltage_normalize;
                motor->uq /= voltage_normalize;
            }

            break;
        }
    }

    return ret;
}


/**
 * @brief Called in 20K adc interrupt.
 * @param motor
 * @return
 */
int motor_status_loop(motor_t *motor)
{
    int ret = 0;

    if (motor == NULL)
        return -1;

    /* Get phase current. */
    motor->get_phase_current(motor);

    /* 电机处于 编码器校准模式 */
    if (motor->status == MOTOR_STATUS_CALIB_ENCODER)
        return MOTOR_STATUS_CALIB_ENCODER;

    /* Get position. */
    if (!motor->sensorless)
    {
        motor_get_elec_angle(motor);

        foc_pll_calc(&motor->speedPll, motor->encoderRawData);
        motor->speedRpm = motor->speedPll.speed * motor->currentLoopFreq * 60.0f;
    }

    switch (motor->status)
    {
        case MOTOR_STATUS_IDLE:
            break;

        case MOTOR_STATUS_RUN:
            motor_ctrl_mode_loop(motor);
            break;

        case MOTOR_STATUS_STOP:
        default:
            break;
    }

    return ret;
}
