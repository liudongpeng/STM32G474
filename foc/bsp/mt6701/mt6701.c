//
// Created by LiuDongPeng on 2023/12/18.
//

#include "mt6701.h"
#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "spi.h"


/**
 * @brief mt6701 abs encoder init
 * @param[out]  dev
 * @param[in]   spi_send_recv_handle
 * @param[in]   spi_cs_set_handle
 * @return
 */
int mt6701_init(mt6701_t *dev, spi_send_recv_callback_t spi_send_recv_handle, spi_cs_set_callback_t spi_cs_set_handle)
{
    if (dev == NULL)
        return -1;

    if (spi_send_recv_handle == NULL)
        return -2;

    dev->resolution = 16384;

    dev->spi_send_recv_callback = spi_send_recv_handle;
    dev->spi_cs_set_callback = spi_cs_set_handle;
    dev->cs_valid_level = 0;

//    return mt6701_get_raw_data(dev, NULL);
    return 0;
}

/**
 * @brief Get raw data
 * @param[in]   dev
 * @param[out]  raw
 * @return
 */
int mt6701_get_raw_data(mt6701_t *dev, uint16_t *raw)
{
    int ret = 0;
    uint16_t tx = 0;
    uint16_t rx = 0;

    if (dev == NULL)
        return -1;

    /**
     * [D13:D0]     [Mg3:Mg0]    [CRC5:CRC0]
     * 14 bit Data  4 bit Status 6 bit CRC
     */

    /* Reset CS pin, Start transform */
    dev->spi_cs_set_callback(dev->cs_valid_level);

    ret = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx, (uint8_t*)&rx, 1, 100);

    /* Set cs pin, Stop transform */
    dev->spi_cs_set_callback(!dev->cs_valid_level);

    /* Check read data frame */

    /* Get 14bit data */
    if (raw != NULL)
    {
        *raw = rx >> 2;
    }

    return ret;
}


/**
 * @brief Get data
 * @param[in]   dev
 * @param[out]  raw
 * @param[out]  angle
 * @param[out]  rad
 * @return
 */
int mt6701_get_data(mt6701_t *dev, uint16_t *raw, float *angle, float *rad)
{
    int ret = 0;

    if (dev == NULL)
        return -1;

    uint16_t tmp;
    ret = mt6701_get_raw_data(dev, &tmp);

    if (raw != NULL)
        *raw = tmp;

    if (angle != NULL)
        *angle = (float)tmp / dev->resolution * 360.0f;

    if (rad != NULL)
        *rad = (float)tmp / dev->resolution * (float) M_TWOPI;

    return ret;
}
