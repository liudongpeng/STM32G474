//
// Created by LiuDongPeng on 2024/1/11.
//

#include "utils.h"
#include <stdlib.h>
#include <math.h>


__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag()
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}


/**
 * @brief Get system micros
 */
uint32_t get_sys_micros()
{
    uint32_t millis = HAL_GetTick();
    __IO uint32_t tms = SysTick->LOAD + 1;
    __IO uint32_t u = tms - SysTick->VAL;

    if (LL_SYSTICK_IsActiveCounterFlag())
    {
        millis = HAL_GetTick();
        u = tms - SysTick->VAL;
    }

    return (millis * 1000 + (u * 1000) / tms);
}


float LeastSquareLinearFit(int y[], const int num, float *a, float *b)
{
    float sum_x2 = 0;
    float sum_y = 0;
    float sum_x = 0;
    float sum_xy = 0;
    float sum_q = 0;

    for (int i = 0; i < num; i++)
    {
        float x = M_PI * i / 6;
        sum_x2 += x * x;
        sum_y += y[i];
        sum_x += x;
        sum_xy += x * y[i];
    }

    *a = (sum_x2 * sum_y - sum_x * sum_xy) / (num * sum_x2 - sum_x * sum_x);
    *b = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);

    for (int i = 0; i < num; i++)
    {
        float ya = y[i] - (*a) - (*b) * (M_PI * i / 6);
        sum_q += ya * ya;
    }

    return sqrtf(sum_q / num);
}


int GetCurrentAbsTotalValue(int lValue)
{
#define ENCODER_RESOLUTION 360
    int m_absEncTotal;           // 计算编码器总时间片变化值
    static int absEnc_Q_count;     // 单圈绝对值编码器圈数计算
    static int data_count = 0;       // 计一次运行改为1，用于数据缓存初始化
    static int m_absEncBuff[5] = {0}; // 绝对值编码器数据缓存

    m_absEncBuff[0] = lValue; //  读取数据缓存

    if (data_count < 1)
    {//开机初始化
        data_count++;
        m_absEncBuff[1] = m_absEncBuff[2] = m_absEncBuff[3] = m_absEncBuff[0];
        absEnc_Q_count = 1000; // 初始圈数
    }

    if (abs(m_absEncBuff[0] - m_absEncBuff[1]) > (ENCODER_RESOLUTION + 100))
    {// 去除异常数据
        m_absEncBuff[0] = m_absEncBuff[1] + (m_absEncBuff[1] - m_absEncBuff[2]);
    }


    if ((m_absEncBuff[0] - m_absEncBuff[1]) > 300)
    {// 提升或圆盘顺时针过程,数值减小
        absEnc_Q_count--;
    }
    else if ((m_absEncBuff[1] - m_absEncBuff[0]) > 300)
    {// 下降或圆盘逆时针过程,数值增加
        absEnc_Q_count++;
    }
    else
    {}

    m_absEncTotal = absEnc_Q_count * ENCODER_RESOLUTION + m_absEncBuff[0];

    m_absEncBuff[3] = m_absEncBuff[2];
    m_absEncBuff[2] = m_absEncBuff[1];
    m_absEncBuff[1] = m_absEncBuff[0];

    return (m_absEncTotal);
}
