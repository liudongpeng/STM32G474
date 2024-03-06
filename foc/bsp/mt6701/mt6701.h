//
// Created by LiuDongPeng on 2023/12/18.
//

#ifndef MT6701_MT6701_H
#define MT6701_MT6701_H


#include <stdint.h>
#include <stddef.h>


enum mt6701_mg
{
    MT6701_MG_NORMAL = 0,
    MT6701_MG_UP,
    MT6701_MG_LOW,
    MT6701_MG_NONE,
};



typedef struct mt6701
{
    int (*spi_send_recv_callback)(uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeOut);
    void (*spi_cs_set_callback)(uint8_t val);
    uint8_t cs_valid_level; // low level

    float resolution; // 14bit 0x3fff

} mt6701_t;


typedef int (*spi_send_recv_callback_t)(uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeOut);
typedef void (*spi_cs_set_callback_t)(uint8_t val);


int mt6701_init(mt6701_t* dev, spi_send_recv_callback_t spi_send_recv_handle, spi_cs_set_callback_t spi_cs_set_handle);

int mt6701_get_raw_data(mt6701_t* dev, uint16_t* raw);
int mt6701_get_data(mt6701_t* dev, uint16_t* raw, float* angle, float* rad);


#endif //MT6701_MT6701_H
