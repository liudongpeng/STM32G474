//
// Created by LiuDongPeng on 2023/11/16.
//

#ifndef FOC_FOC_H
#define FOC_FOC_H

#include "main.h"
#include "pid.h"


typedef struct foc_pll
{
    pid_ctrl_t pid;

    int lastAngle;
    float theta;
    float speed;

} foc_pll_t;




typedef struct hfi_pll
{
    pid_ctrl_t pid;

    float integral;

} hfi_pll_t;




static inline int foc_pid_diff(int a, int b, int rand)
{
    int diff = a - b;
    if (diff > rand / 2)
        diff -= rand;
    if (diff < -rand / 2)
        diff += rand;
    return diff;
}

static inline float foc_pid_diff_float(float a, float b, float rand)
{
    float diff = a - b;
    if (diff > rand / 2)
        diff -= rand;
    if (diff < -rand / 2)
        diff += rand;
    return diff;
}





void foc_clark(float ia, float ib, float* ialpha, float* ibeta);
void foc_inv_clark();

void foc_park(float ialpha, float ibeta, float sinTheta, float cosTheta, float* id, float* iq);
void foc_inv_park(float ud, float uq, float sinTheta, float cosTheta, float* ualpha, float* ubeta);

void foc_svpwm(float ualpha, float ubeta, float udc, float tpwm, float* ta, float* tb, float* tc);
int foc_mid_point_svm(float ualpha, float ubeta, float VBus, float* ta, float* tb, float* tc);
int odriver_svm(float ualpha, float ubeta, float* ta, float* tb, float* tc, int* sector);

int foc_pll_init(foc_pll_t* pll, float p, float i, float d, float ka, float lowLimit, float upLimit);
int foc_pll_calc(foc_pll_t* pll, int rawAngle);


int hfi_pll_calc(hfi_pll_t* pll, float alpha, float beta, float thetaErr, float* weEst, float* thetaEst);



#endif //FOC_FOC_H
