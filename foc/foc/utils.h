//
// Created by LiuDongPeng on 2024/1/11.
//

#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include "main.h"



/**
 * @brief sqrt(3)
 */
#define SQRT_3   1.732050807568877f
#define SQRT3   1.732050807568877f
#define SQRT3_DIV_3   (SQRT_3 / 3.0f)
#define SQRT3_DIV_2   (SQRT_3 / 2.0f)
#define one_by_sqrt3 0.57735026919f
#define two_by_sqrt3 1.15470053838f

#define FOC_CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


/**
 * @brief Get system micros
 */
uint32_t get_sys_micros();



float LeastSquareLinearFit(int y[], const int num, float *a, float *b);

int GetCurrentAbsTotalValue(int lValue);

int sign(float val);
float sat(float s, float delta);


#endif //FOC_UTILS_H
