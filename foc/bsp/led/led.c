/**
  * @file    led.c
  * @author  LiuDongPeng
  * @version V0.1
  * @date    2023-1-5
  * @brief   led模块源文件
  *
  * @attention
  * 使用此led模块, 需要用户实现设置led引脚电平的接口, 接口函数格式为: int led_set_level(uint8_t level);
  * 此处给出一个示例:
  * int my_led_set_level(uint8_t level)
  * {
  * 	return HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, level == LED_LEVEL_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);
  * }
  *
  * @changelog
  * 2023-1-5	修改变量, 函数等的命名风格为小写字母加下划线组合
  */

#include "led.h"



/**
 * @brief Create led object
 * @param[out]	led			led object
 * @param[in]	valid_level	点亮led的有效电平
 * @param[in]	set_level	设置led接口电平的接口函数
 * @return
 */
int led_create(led_t* led, uint8_t valid_level, led_set_level_t set_level)
{
	if (led == NULL)
		return -1;

	led->valid_level = valid_level;
	led->led_set_level = set_level;

	return led_init(led);
}

/**
 * @brief Init led object
 * @param[in]	led
 * @return
 */
int led_init(led_t* led)
{
	if (led == NULL || led->led_set_level == NULL)
		return -1;

	led_close(led);

	return 0;
}

/**
 * @brief Turn on led
 * @param[in]	led
 * @return
 */
int led_open(led_t *led)
{
	if (led == NULL || led->led_set_level == NULL)
		return -1;

	led->led_set_level(led->valid_level == LED_LEVEL_LOW ? LED_LEVEL_LOW : LED_LEVEL_HIGH);
	led->is_open = 1;

	return 0;
}

/**
 * @brief Turn off led
 * @param[in]	led
 * @return
 */
int led_close(led_t *led)
{
	if (led == NULL || led->led_set_level == NULL)
		return -1;

	led->led_set_level(led->valid_level == LED_LEVEL_LOW ? LED_LEVEL_HIGH : LED_LEVEL_LOW);
	led->is_open = 0;

	return 0;
}

/**
 * @brief Toggle led
 * @param[in]	led
 * @return
 */
int led_toggle(led_t* led)
{
    if (led == NULL || led->led_set_level == NULL)
        return -1;

    led->is_open = led->is_open ? 0 : 1;
    if (led->is_open)
        led_open(led);
    else
        led_close(led);

    return 0;
}

/**
 * @brief Led is on
 * @param[in]	led
 * @return
 */
uint8_t led_is_open(led_t* led)
{
	if (led == NULL)
		return 0;

	return led->is_open;
}
