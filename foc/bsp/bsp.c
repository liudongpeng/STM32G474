//
// Created by LiuDongPeng on 2023/10/27.
//

#include "bsp.h"
#include "stdio.h"

/* ST HAL Lib */
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"

#include "digital_filter.h"



/**
 * @brief 10K flash to store system config
 */
#define SYSTEM_FLASH_END_ADDRESS ((__IO uint32_t)0x8020000)
#define SYSTEM_FLASH_CONFIG_START_ADDRESS ((__IO uint32_t)0x801D800)


/**
* @brief 控制板上的led
*/

led_t led1, led2, led3;

// led1
#define LED1_GPIO_PORT  Led1_GPIO_Port
#define LED1_GPIO_PIN   Led1_Pin

// led2
#define LED2_GPIO_PORT  Led2_GPIO_Port
#define LED2_GPIO_PIN   Led2_Pin

// led3
#define LED3_GPIO_PORT  Led3_GPIO_Port
#define LED3_GPIO_PIN   Led3_Pin

static void led1_set_level(uint8_t val)
{
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led2_set_level(uint8_t val)
{
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led3_set_level(uint8_t val)
{
    HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


/**
 * @brief 控制板上的按键
 */

button_t btn1, btn2, btn3, btn4;

// btn1
#define BTN1_GPIO_PORT  Key1_GPIO_Port
#define BTN1_GPIO_PIN   Key1_Pin

// btn2
#define BTN2_GPIO_PORT  Key2_GPIO_Port
#define BTN2_GPIO_PIN   Key2_Pin

// btn3
#define BTN3_GPIO_PORT  Key3_GPIO_Port
#define BTN3_GPIO_PIN   Key3_Pin

// btn4
#define BTN4_GPIO_PORT  Key4_GPIO_Port
#define BTN4_GPIO_PIN   Key4_Pin

static uint8_t get_btn1_level()
{
    return HAL_GPIO_ReadPin(BTN1_GPIO_PORT, BTN1_GPIO_PIN);
}

static uint8_t get_btn2_level()
{
    return HAL_GPIO_ReadPin(BTN2_GPIO_PORT, BTN2_GPIO_PIN);
}

static uint8_t get_btn3_level()
{
    return HAL_GPIO_ReadPin(BTN3_GPIO_PORT, BTN3_GPIO_PIN);
}

static uint8_t get_btn4_level()
{
    return HAL_GPIO_ReadPin(BTN4_GPIO_PORT, BTN4_GPIO_PIN);
}

/**
 * @brief 按键扫描时钟滴答，5ms调用一次
 * @return
 */
void bsp_btn_ticks()
{
    button_ticks();
}


/**
 * @brief MT6701磁编码器
 */

mt6701_t mt6701;

#define MT6701_SPI_HANDLE  hspi1
#define MT6701_SPI_CS_PORT GPIOD
#define MT6701_SPI_CS_PIN  GPIO_PIN_2

static int encoder_spi_send_recv_callback(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, uint32_t timeOut)
{
    return HAL_SPI_TransmitReceive(&MT6701_SPI_HANDLE, pTxData, pRxData, size, timeOut);
}

static void encoder_spi_cs_set_callback(uint8_t val)
{
    HAL_GPIO_WritePin(MT6701_SPI_CS_PORT, MT6701_SPI_CS_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 编码器
 */
encoder_t encoder1;

static int encoder_get_data_callback(encoder_t *encoder, int *raw, float *angle, float *angleRad)
{
    return mt6701_get_data(&mt6701, (uint16_t *) raw, angle, angleRad);
}


/**
 * @brief 电机开发板
 */
foc_board_t foc_board = {0};


/**
 * @brief 电机
 */
motor_t motor1 = {0};

/**
 *
 * @param motor
 * @return
 */
int motor_get_phase_current_callback(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    bsp_get_phase_current(&foc_board, &motor->ia, &motor->ib, &motor->ic);

    return 0;
}

/**
 *
 * @param motor
 * @return
 */
int motor_get_phase_voltage_callback(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    bsp_get_phase_voltage(&foc_board, &motor->ua, &motor->ub, &motor->uc);

    return 0;
}

/**
 *
 * @param motor
 * @return
 */
int motor_get_vbus_voltage_callback(motor_t *motor)
{
    if (motor == NULL)
        return -1;

    bsp_get_vbus(&foc_board, &motor->VBus);

    return 0;
}


/**
 * @brief 开发板硬件外设初始化
 * @return
 */
int bsp_init()
{
    int ret = 0;

    /* MT6701初始化 */
    ret = mt6701_init(&mt6701, encoder_spi_send_recv_callback, encoder_spi_cs_set_callback);

    /* 编码器初始化 */
    ret = encoder_init(&encoder1, encoder_get_data_callback);
    //encoder_update_pll_gains(&encoder, 200.0f * 6.28f);

    /* 电机初始化 */
    ret = motor_create(&motor1);
    motor_link_encoder(&motor1, &encoder1);
    motor1.get_phase_current = motor_get_phase_current_callback;
    motor1.get_phase_voltage = motor_get_phase_voltage_callback;
    motor1.get_vbus_voltage = motor_get_vbus_voltage_callback;

    /* 按键初始化 */
    ret = button_create(&btn1, 0, get_btn1_level);
    ret = button_create(&btn2, 0, get_btn2_level);
    ret = button_create(&btn3, 0, get_btn3_level);
    ret = button_create(&btn4, 0, get_btn4_level);

    /* led初始化 */
    ret = led_create(&led1, LED_LEVEL_HIGH, led1_set_level);
    ret = led_create(&led2, LED_LEVEL_HIGH, led2_set_level);
    ret = led_create(&led3, LED_LEVEL_HIGH, led3_set_level);

    /* ADC自校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
//    bsp_calib_vbus(&foc_board);

    /* 开启ADC注入组转换中断 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 关闭PWM输出 */
    motor_disable(&motor1);

    return ret;
}


/**
 *
 * @param board
 * @return
 */
int bsp_calib_vbus(foc_board_t *board)
{
    if (board == NULL)
        return -1;

    int val[3] = {0};
    int offset = 0;

    for (int cnt = 0; cnt < 10; cnt++)
    {
        for (int i = 0; i < 3; i++)
        {
            HAL_ADC_Start(&hadc2);
            HAL_ADC_PollForConversion(&hadc2, 100);
            val[i] = HAL_ADC_GetValue(&hadc2);
        }
        offset += val[0];
    }

    board->VBusOffset = offset / 10;

    return 0;
}

/**
 *
 * @param board
 * @param vbus
 * @return
 */
int bsp_get_vbus(foc_board_t *board, float *vbus)
{
    int val[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 100);
        val[i] = HAL_ADC_GetValue(&hadc2);
    }

    // VBus
    if (vbus)
        *vbus = (float) (val[0] - board->VBusOffset) * (3.3f / 4096) * (32.0f / 2.0f);

    return 0;
}

/**
 *
 * @param board
 * @param ua
 * @param ub
 * @param uc
 */
void bsp_get_phase_voltage(foc_board_t *board, float *ua, float *ub, float *uc)
{
    int adcA = 0, adcB = 0, adcC = 0;

    if (board == NULL)
        return;

    float val[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 100);
        val[i] = (float) (HAL_ADC_GetValue(&hadc2) * (3.3f / 4096) * (32.0f / 2.0f));
    }

    adcA = val[1];
    adcB = val[2];

    if (ua)
        *ua = (adcA - board->uaOffset) * ((3.3f / 4096) * (32.0f / 2.0f));

    if (ub)
        *ub = (adcB - board->ubOffset) * ((3.3f / 4096) * (32.0f / 2.0f));
}

/**
 * @brief Get board u v w phase current
 * @param board
 * @param ia
 * @param ib
 * @param ic
 */
void bsp_get_phase_current(foc_board_t *board, float *ia, float *ib, float *ic)
{
    int adcA = 0, adcB = 0, adcC = 0;

    if (board == NULL)
        return;

    adcA = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adcB = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

    if (ia)
        *ia = (float) (adcA - board->iaOffset) * (0.008056640625f);
    if (ib)
        *ib = (float) (adcB - board->ibOffset) * (0.008056640625f);

//    *ia = low_pass_filter_work(&motor1.iaLpFilter, *ia);
//    *ib = low_pass_filter_work(&motor1.ibLpFilter, *ib);
}


/**
 * @brief Get board u v w phase current at present sector
 * @param board
 * @param sector
 * @param ia
 * @param ib
 * @param ic
 */
void bsp_get_phase_current_with_sector(foc_board_t *board, int sector, float *ia, float *ib, float *ic)
{
    uint32_t adcA, adcB, adcC;

    if (board == NULL || ia == NULL || ib == NULL || ic == NULL)
        return;

    if (sector > 6)
        sector = 6;
    if (sector < 1)
        sector = 1;

    switch (sector)
    {
        case 4:
        case 5:
            /* Ia Ib < 0 */
            adcA = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            adcB = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            *ia = (float) (adcA - board->iaOffset) * 0.01933593f;
            *ib = (float) (adcB - board->ibOffset) * 0.01933593f;
            *ic = -*ia - *ib;
            break;

        case 1:
        case 6:
            /* Ib Ic < 0 */
            adcC = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            adcB = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            *ic = (float) (adcC - board->icOffset) * 0.01933593f;
            *ib = (float) (adcB - board->ibOffset) * 0.01933593f;
            *ia = -*ib - *ic;
            break;


        case 2:
        case 3:
        default:
            /* Ia Ic < 0 */
            adcA = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            adcC = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            *ia = (float) (adcA - board->iaOffset) * 0.01933593f;
            *ic = (float) (adcC - board->icOffset) * 0.01933593f;
            *ib = -*ia - *ic;
            break;
    }
}


/**
 * @brief
 * @param motor
 */
void bsp_store_encoder_angle_offset(motor_t *motor)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SYSTEM_FLASH_CONFIG_START_ADDRESS,
                      (uint64_t) motor->angleRadOffset);
    FLASH_WaitForLastOperation(500);
    HAL_FLASH_Lock();
}

/**
 * @brief
 * @param[out]  motor
 * @return
 */
float bsp_restore_encoder_angle_offset(motor_t *motor)
{
    if (motor != NULL)
    {
        motor->angleRadOffset = (float) *((__IO uint64_t *) SYSTEM_FLASH_CONFIG_START_ADDRESS);
    }

    return 0;
}


/**
 *
 * @param ch1: 1: h on, l off; 0: h off, l off; -1: h off, l on
 * @param ta
 * @param ch2
 * @param tb
 * @param ch3
 * @param tc
 */
void bsp_set_phase_pwm(int ch1, float ta, int ch2, float tb, int ch3, float tc)
{
    if (ch1 == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_TIM1_ARR * ta);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    }
    else if (ch1 == -1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_TIM1_ARR * ta);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    }

    if (ch2 == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_TIM1_ARR * tb);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    }
    else if (ch2 == -1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_TIM1_ARR * tb);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    }

    if (ch3 == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_TIM1_ARR * tc);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    }
    else if (ch3 == -1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_TIM1_ARR * tc);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    }
}
