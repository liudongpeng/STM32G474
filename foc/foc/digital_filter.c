//
// Created by LiuDongPeng on 2023/11/16.
//

#include "digital_filter.h"



/**
 *
 * @param filter
 * @param fc
 * @param dt
 * @return
 */
int lpf_init(lpf_t *filter, float fc, float dt)
{
    if (filter == NULL)
        return -1;

    float b = M_TWOPI * fc * dt;
    filter->alpha = b / (b + 1);

    filter->first = true;

    return 0;
}

/**
 * @brief Filter work
 * @param[out]	filter
 * @param[in]	val
 * @return
 */
float lpf_work(lpf_t *filter, float val)
{
    if (filter == NULL)
        return 0;

    float out;

    /* 第一次进入，给lastVal赋值 */
    if (filter->first)
    {
        filter->first = false;
        filter->lastVal = 0;
    }

    out = filter->lastVal + filter->alpha * (val - filter->lastVal);
    filter->lastVal = out;

    return out;
}
