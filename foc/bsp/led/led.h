/**
  * @file    led.h
  * @author  LiuDongPeng
  * @version V0.1
  * @date    2023-1-5
  * @brief   led模块头文件
  *
  * @attention
  * 使用此led模块, 需要用户实现设置led引脚电平的接口, 接口函数格式为:
  * 	int led_set_level(uint8_t level);
  *
  * @changelog
  * 2023-1-5	修改变量, 函数等的命名风格为小写字母加下划线组合
  */


#ifndef F103_TEST_LED_H
#define F103_TEST_LED_H


#include <stdint.h>
#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief led引脚电平高低
 */
#define LED_LEVEL_HIGH  1   /* led引脚高电平 */
#define LED_LEVEL_LOW   0   /* led引脚低电平 */


/**
 * @brief led对象结构
 */
typedef struct led
{
	/* led工作模式 */
	uint8_t mode;

	/* led当前的亮灭状态 */
	uint8_t is_open;

	/* led有效电平 */
	uint8_t valid_level;

	/**
	 * @brief 用户实现设置led引脚电平的接口
	 * @param[in] level
	 */
	void (*led_set_level)(uint8_t level);

} led_t;


typedef void (*led_set_level_t)(uint8_t level);


int led_create(led_t* led, uint8_t valid_level, led_set_level_t set_level);
int led_init(led_t* led);

int led_open(led_t* led);
int led_close(led_t* led);
int led_toggle(led_t* led);

uint8_t led_is_open(led_t* led);
int led_get_state(led_t* led);


#ifdef __cplusplus
}
#endif

#endif //F103_TEST_LED_H
