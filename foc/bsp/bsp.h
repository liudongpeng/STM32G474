//
// Created by LiuDongPeng on 2023/10/27.
//

#ifndef FOC_BSP_H
#define FOC_BSP_H

#ifdef __cplusplus
extern "C" {
#endif


/* User Hardware */
#include "button.h"

#include "encoder.h"
#include "mt6701.h"
#include "led.h"
#include "motor.h"


typedef struct foc_board
{
    bool isAdcCalibOvered;
    int VBusOffset;
    int uaOffset, ubOffset, ucOffset;
    int iaOffset, ibOffset, icOffset;
    float shuntRs;

} foc_board_t;


/**
 * @brief On board device
 */
extern led_t led1, led2, led3;
extern button_t btn1, btn2, btn3, btn4;
extern mt6701_t mt6701;
extern motor_t motor1;
extern foc_board_t foc_board;







int bsp_init();
void bsp_btn_ticks();

uint32_t bsp_get_us_tick();

void bsp_get_phase_voltage(foc_board_t* board, float * ua, float* ub, float* uc);
void bsp_get_phase_current(foc_board_t* board, float * ia, float* ib, float* ic);
void bsp_get_phase_current_with_sector(foc_board_t* board, int sector, float* ia, float* ib, float* ic);

void bsp_store_encoder_angle_offset(motor_t* motor);
float bsp_restore_encoder_angle_offset(motor_t* motor);



int bsp_calib_vbus(foc_board_t* board);
int bsp_get_vbus(foc_board_t* board, float* vbus);


void bsp_set_phase_pwm(int ch1, float ta, int ch2, float tb, int ch3, float tc);


#ifdef __cplusplus
}
#endif

#endif //FOC_BSP_H
