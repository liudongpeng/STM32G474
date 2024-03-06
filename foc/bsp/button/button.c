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


#include "button.h"


static int button_init(button_t *btn);
static void button_handler(button_t *btn);


/* 调用按键对应的事件回调 */
#define BUTTON_EVENT_CB(btn, event) if (btn->event_cb_list[event]) \
                                        btn->event_cb_list[event](btn, event)


static button_t *s_btn_list_head;   /* 按键列表头指针 */


/**
 * @brief 创建按键对象
 * @param[out]  btn
 * @param[in]   active_level
 * @param[in]   get_level
 * @return
 */
int button_create(button_t *btn, uint8_t active_level, get_level_t get_level)
{
    if (btn == NULL || get_level == NULL)
        return -1;

    memset(btn, 0, sizeof(button_t));

    btn->active_level = active_level;
    btn->get_btn_level = get_level;
    btn->state = ButtonState_None;
    btn->btn_level = btn->get_btn_level();

    return button_init(btn);
}

/**
 * @brief 按键初始化
 * @param[in]   btn
 * @return
 */
static int button_init(button_t *btn)
{
    /* 启用按键 */
    return button_start(btn);
}

/**
 * @brief 安装事件处理回调
 * @param[in]   btn
 * @param[in]   event
 * @param[in]   btn_event_cb
 * @return
 */
int button_install_event_callback(button_t *btn, button_event_t event, btn_event_callback_t btn_event_cb)
{
    if (btn == NULL || btn_event_cb == NULL)
        return -1;

    /* 输入事件合法性检查 */
    if (event > NUMBER_OF_EVENT - 1)
        return -2;

    btn->event_cb_list[event] = btn_event_cb;

    return 0;
}

/**
 * @brief 获取按键事件
 * @param[in]   btn
 * @return
 */
button_event_t get_button_event(button_t *btn)
{
    if (btn == NULL)
        return ButtonEvent_None;

    return btn->event;
}

/**
 * @brief 开始按键任务, 把此按键添加到扫描列表
 * @param[in]   btn
 * @return
 */
int button_start(button_t *btn)
{
    button_t *p = s_btn_list_head;

    if (btn == NULL)
        return -1;

    /* 检查此按键是否已经加入到按键列表 */
    while (p != NULL)
    {
        if (p == btn)
            return -1;
        p = p->next;
    }

    btn->next = s_btn_list_head;
    s_btn_list_head = btn;

    return 0;
}

/**
 * @brief 停止按键任务, 把此按键从扫描列表删除
 * @param btn
 */
void button_stop(button_t *btn)
{
    button_t *prev, *cur;

    if (btn == NULL)
        return;

    prev = s_btn_list_head;
    cur = prev;
    while (cur != NULL)
    {
        if (cur == btn)
        {
            /* 删除头节点 */
            if (cur == s_btn_list_head)
            {
                s_btn_list_head = cur->next;
                return;
            }

            prev->next = cur->next;
            return;
        }

        prev = cur;
        cur = cur->next;
    }
}

/**
 * @brief 按键处理核心
 * @param[in]   btn
 */
static void button_handler(button_t *btn)
{
    uint8_t cur_level;

    if (btn == NULL || btn->get_btn_level == NULL)
        return;

    /* 按键状态不是none的时候, 进行计数 */
    if (btn->state != ButtonState_None)
    {
        btn->ticks++;
    }

    /* 先获取一次当前按键电平 */
    cur_level = btn->get_btn_level();

    /* ------------------- 防反跳(消抖) ------------------- */
    if (cur_level != btn->btn_level)   /* 当前和上次的按键电平不同 */
    {
        if (++btn->debounce_cnt >= BUTTON_DEBOUNCE_TICKS)
        {
            btn->btn_level = cur_level;
            btn->debounce_cnt = 0;
        }
    }
    else    /* 当前和上次的按键电平相同 */
    {
        btn->debounce_cnt = 0;
    }

    /* ------------------- 按键状态机 ------------------- */
    switch (btn->state)
    {
        /* 按键状态是 none */
        case ButtonState_None:
            /* 按键电平是触发电平, 按键是按下的 */
            if (btn->btn_level == btn->active_level)
            {
                btn->event = ButtonEvent_PressDown;
                btn->ticks = 0;
                btn->repeat = 1;

                /* 按键状态改变为 按键按下 */
                btn->state = ButtonState_PressDown;
                BUTTON_EVENT_CB(btn, ButtonEvent_PressDown);
            }
            break;

        /* 按键状态是 按下状态 */
        case ButtonState_PressDown:
            /* 按键电平不是触发电平, 按键是松开的 */
            if (btn->btn_level != btn->active_level)
            {
                btn->event = ButtonEvent_PressUp;
                btn->ticks = 0;
                btn->state = ButtonState_PressUp;
            }

            /* 按键电平是触发电平, 即按键是按下的, 且时钟计数超过设置的长按事件, 为长按 */
            else if (btn->ticks > BUTTON_LONG_PRESS_TICKS)
            {
                btn->event = ButtonEvent_LongPressStart;
                BUTTON_EVENT_CB(btn, ButtonEvent_LongPressStart);
                btn->state = ButtonState_LongPressStart;
            }
            else
            {
            }
            break;

        /* 按键状态是 松开状态 */
        case ButtonState_PressUp:
            /* 按键按下 */
            if (btn->btn_level == btn->active_level)
            {
                btn->event = ButtonEvent_PressDown;
                BUTTON_EVENT_CB(btn, ButtonEvent_PressDown);

                /* 重复按下次数加1 */
                btn->repeat++;
                BUTTON_EVENT_CB(btn, ButtonEvent_RepeatPress);
                btn->ticks = 0;
                btn->state = ButtonState_RepeatPress;
            }

            /* 按键松开, 且距上次松开的时间大于短按的设定值 */
            else if (btn->ticks > BUTTON_SHORT_TICKS)
            {
                switch (btn->repeat)
                {
                    case 1: /* 按键重复按下的次数是1 */
                        btn->event = ButtonEvent_SingleClick;
                        BUTTON_EVENT_CB(btn, ButtonEvent_SingleClick);
                        break;

                    case 2: /* 按键重复按下的次数是2 */
                        btn->event = ButtonEvent_DoubleClick;
                        BUTTON_EVENT_CB(btn, ButtonEvent_DoubleClick);
                        break;



                    default:
                        break;
                }

                /* 重置按键状态 */
                btn->state = ButtonState_None;
            }
            else
            {
            }
            break;

        /* 按键状态是 重复按下状态 */
        case ButtonState_RepeatPress:
            /* 按键松开 */
            if (btn->btn_level != btn->active_level)
            {
                btn->event = ButtonEvent_PressUp;
                BUTTON_EVENT_CB(btn, ButtonEvent_PressUp);

                /* 按键时钟计数小于短按设定值 */
                if (btn->ticks < BUTTON_SHORT_TICKS)
                {
                    btn->ticks = 0;
                    btn->state = ButtonState_PressUp;
                }
                else
                {
                    btn->state = ButtonState_None;
                }
            }
            break;

        /* 按键状态是 开始长按 */
        case ButtonState_LongPressStart:
            /* 按键电平是触发电平, 即按键是按下的, 按键状态是 长按保持 */
            if (btn->btn_level == btn->active_level)
            {
                btn->event = ButtonEvent_LongPressHold;
                BUTTON_EVENT_CB(btn, ButtonEvent_LongPressHold);
            }
            else    /* 按键电平不是触发电平, 即按键是松开的 */
            {
                btn->event = ButtonEvent_PressUp;
                BUTTON_EVENT_CB(btn, ButtonEvent_PressUp);
                btn->state = ButtonState_None;
            }
            break;

        default:
            break;
    }

}

/**
 * @brief 后台时钟滴答计数, 每5ms重复一次
 */
void button_ticks()
{
    button_t *cur = s_btn_list_head;

    while (cur != NULL)
    {
        button_handler(cur);
        cur = cur->next;
    }
}