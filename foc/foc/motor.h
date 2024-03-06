//
// Created by LiuDongPeng on 2023/11/16.
//

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H


#include "main.h"

#include <stdbool.h>
#include "arm_math.h"

#include "encoder.h"
#include "foc.h"
#include "pid.h"
#include "digital_filter.h"



typedef struct motor motor_t;

typedef int (*motor_get_phase_current_callback_t)(motor_t* motor);
typedef int (*motor_get_phase_voltage_callback_t)(motor_t* motor);
typedef int (*motor_get_vbus_voltage_callback_t)(motor_t* motor);


typedef enum
{
    MOTOR_STATUS_IDLE,
    MOTOR_STATUS_INIT,
    MOTOR_STATUS_START,
    MOTOR_STATUS_RUN,
    MOTOR_STATUS_STOP,
    MOTOR_STATUS_BRAKE,
    MOTOR_STATUS_WAIT,
    MOTOR_STATUS_FAULT,
} motor_status_t;


typedef enum
{
    MOTOR_CTRL_MODE_VOLT_ANGLE_INC,
    MOTOR_CTRL_MODE_VOLT_ENCODER,
    MOTOR_CTRL_MODE_CURRENT,
    MOTOR_CTRL_MODE_CURRENT_SPEED,
    MOTOR_CTRL_MODE_CURRENT_SPEED_POSITION,
} motor_ctrl_mode_t;


typedef struct motor
{
    motor_get_phase_current_callback_t get_phase_current;
    motor_get_phase_voltage_callback_t get_phase_voltage;
    motor_get_vbus_voltage_callback_t get_vbus_voltage;

    float VBus, IBus;
    float ua, ub, uc;
    float ia, ib, ic;
    float ialpha, ibeta;
    float id, iq, idLast, iqLast;
    float ud, uq;
    float ualpha, ubeta;

    lpf_t iaLpFilter;
    lpf_t ibLpFilter;
    lpf_t icLpFilter;

    /* Current loop param. */
    pid_ctrl_t idPid;
    pid_ctrl_t iqPid;
    float idSet, iqSet;
    int currentLoopFreq;

    /* Speed loop param. */
    pid_ctrl_t speedPid;
    float speedSet, speedShadow;
    float speedRampTime;
    float speedAcc;
    bool isUseSpeedRamp;
    int speedLoopFreq, speedLoopCnt;

    /* Position loop param. */
    pid_ctrl_t positionPid;
    float positionSet;
    int positionLoopFreq, positionLoopCnt;

    /* PWM */
    int timArr;
    float ta, tb, tc;
    int ccra, ccrb, ccrc;
    int sector;

    /* Encoder */
    bool sensorless;
    encoder_t* encoder;
    int encoderCpr;
    int encoderDir;
    int encoderRawData, encoderOffset;
    float angle, angleRad, angleRadOffset, theta, speedRpm;
    float sinTheta, cosTheta;
    lpf_t speedLdFilter;
    foc_pll_t speedPll;

    /* HFI */
    float hfiVoltAmpl;
    float hfiTheta;
    float hfiId, hfiIq;
    float hfiIdLast, hfiIqLast;
    pid_ctrl_t hfiPid;

    /* motor param */
    int polePairs;
    float Rs, Ld, Lq;

    motor_status_t status;

    motor_ctrl_mode_t ctrlMode;

} motor_t;


int motor_create(motor_t* motor);
void motor_set_status(motor_t* motor, motor_status_t status);
void motor_set_ctrl_mode(motor_t* motor, motor_ctrl_mode_t mode);
int motor_ctrl_mode_loop(motor_t *motor);
int motor_status_loop(motor_t *motor);

void motor_enable(motor_t* motor);
void motor_disable(motor_t* motor);

bool motor_has_encoder(motor_t* motor);
bool motor_link_encoder(motor_t *motor, encoder_t *encoder);
encoder_t* motor_get_encoder(motor_t* motor);



uint16_t motor_get_encoder_raw_data(motor_t* motor);
float motor_get_speed_rpm(motor_t* motor);
float motor_get_angle_rad(motor_t* motor);
float motor_get_angle(motor_t* motor);

void motor_set_and_apply_pwm_duty(motor_t* motor, float ta, float tb, float tc);

void motor_set_id_set(motor_t* motor, float idSet);
void motor_set_iq_set(motor_t* motor, float iqSet);


float motor_get_elec_angle(motor_t* motor);
void motor_align_encoder(motor_t* motor, float ud, float angle);
int motor_calib_encoder(motor_t* motor, float ud);

void motor_drag_theta(motor_t *motor, float theta, float ud, float uq);

void motor_run(motor_t *motor);


void motor_open_loop_test(motor_t* motor, float ud, float uq);
int odriver_current_pi_ctrl(motor_t* motor);
int motor_current_closed_loop(motor_t *motor);


void motor_set_speed(motor_t *motor, float speedSet);
int motor_speed_closed_loop(motor_t *motor, float speedSet, float* iqSet);
void motor_set_speed_ramp_time(motor_t *motor, float speedRampTime);
void motor_set_speed_acc(motor_t *motor, float speedAcc);


int motor_position_closed_loop(motor_t *motor, float posSet, float* speedSet);

int motor_meas_phase_resistance(motor_t *motor);
int motor_meas_phase_resistance2(motor_t *motor);




#endif //FOC_MOTOR_H
