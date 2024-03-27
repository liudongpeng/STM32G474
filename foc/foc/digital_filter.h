//
// Created by LiuDongPeng on 2023/11/16.
//

#ifndef FOC_DIGITAL_FILTER_H
#define FOC_DIGITAL_FILTER_H


#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "arm_math.h"


typedef struct low_pass_filter
{
	float fc;   // 截止频率
	float dt;   // 采样周期
	float alpha; // 滤波系数

	bool first;
	float lastVal;
} lpf_t;


typedef struct fir_low_pass_filter
{
    arm_fir_instance_f32 armFirIns;
} fir_lp_t;



int lpf_init(lpf_t* filter, float fc, float dt);
float lpf_work(lpf_t* filter, float val);


#endif //FOC_DIGITAL_FILTER_H
