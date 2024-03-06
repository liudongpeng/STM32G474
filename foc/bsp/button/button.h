/**
  * @file    buttonh.
  * @author  LiuDongPeng
  * @version V0.1
  * @date    2023-1-21
  * @brief   按键模块头文件
  *
  * @attention
  * 使用此按键模块, 需要用户实现读取按键引脚电平的接口和一个5ms的定时器中断,
  * 在5ms的定时中断中调用 button_ticks() 来完成按键扫描, 例如:
  * while (1)
  * {
  * 	button_ticks();
  * 	delay_ms();
  * }
  *
  * @changelog
  *
  */

#ifndef MULTIBUTTON_MULTI_BUTTON_H
#define MULTIBUTTON_MULTI_BUTTON_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>



/**
 * @brief 按键事件
 */
typedef enum button_event
{
    ButtonEvent_PressDown = 0,      /* 按键按下 */
    ButtonEvent_PressUp,            /* 按键松开 */
    ButtonEvent_RepeatPress,        /* 按键重复按下 */
    ButtonEvent_SingleClick,        /* 单击 */
    ButtonEvent_DoubleClick,        /* 双击 */
    ButtonEvent_LongPressStart,     /* 长按开始 */
    ButtonEvent_LongPressHold,      /* 长按保持 */
    NUMBER_OF_EVENT,                /* 按键事件数量 */
    ButtonEvent_None,               /* 无 */
} button_event_t;


/**
 * @brief 按键状态
 */
enum button_state
{
    ButtonState_None = 0,
    ButtonState_PressDown,
    ButtonState_PressUp,
    ButtonState_RepeatPress,
    ButtonState_LongPressStart,
};


/**
 * @brief 按键事件处理回调
 */
typedef void (*btn_event_callback_t)(void *btn, int event);



#define BUTTON_TICKS_INTERVAL   5   /* 按键计数增长(ms) */
#define BUTTON_DEBOUNCE_TICKS   3   /* 按键防反跳计数次数 */
#define BUTTON_SHORT_TICKS      (300 / BUTTON_TICKS_INTERVAL)   /* 按键短按时钟计数 */
#define BUTTON_LONG_PRESS_TICKS (1000 / BUTTON_TICKS_INTERVAL)  /* 按键长按时钟计数 */


/**
 * @brief 按键对象
 */
typedef struct button
{
    uint32_t ticks; /* 时钟滴答 */

    uint8_t repeat; /* 重复按下次数 */
    uint8_t event;  /* 按键当前触发的事件 */
    uint8_t state;  /* 按键状态 */

    uint8_t debounce_cnt: 6;  /* 防反跳(消抖) */
    uint8_t active_level: 1;  /* 有效电平 */
    uint8_t btn_level: 1;   /* 按键电平 */

    /**
     * @brief 获取按键引脚电平
     * @return
     */
    uint8_t (*get_btn_level)();

    btn_event_callback_t event_cb_list[NUMBER_OF_EVENT];  /* 按键事件回调列表 */

    struct button *next;    /* 指向下一个按键对象 */

} button_t;


typedef uint8_t (*get_level_t)();


int button_create(button_t *btn, uint8_t active_level, get_level_t get_level);

int button_install_event_callback(button_t *btn, button_event_t event, btn_event_callback_t btn_event_cb);

button_event_t get_button_event(button_t *btn);

int button_start(button_t *btn);

void button_stop(button_t *btn);

void button_ticks();


#ifdef __cplusplus
}
#endif

#endif //MULTIBUTTON_MULTI_BUTTON_H
