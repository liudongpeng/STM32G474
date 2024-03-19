//
// Created by LiuDongPeng on 2024/3/19.
//

#ifndef FOC_SMO_H
#define FOC_SMO_H


int smo_calc(float ualpha, float ubeta, float ialpha, float ibeta, float* ealpha, float* ebeta);
int smo_pll(float ealpha, float ebeta, float* theat, float* we);


#endif //FOC_SMO_H
