#ifndef __KEY_H_
#define __KEY_H_

#include "main.h"
#include <stdint.h>
#include <stddef.h>

/* 按键事件定义 */
typedef enum {
    KEY_EVENT_NONE = 0,        /* 无事件 */
    KEY_EVENT_SINGLE_CLICK,    /* 单击 / 短按 */
    KEY_EVENT_DOUBLE_CLICK,    /* 双击 */
    KEY_EVENT_LONG_PRESS,      /* 长按 */
} KeyEvent_TypeDef;

/* 按键状态机内部状态 */
typedef enum {
    KEY_STATE_IDLE = 0,        /* 空闲状态（等待首次按下） */
    KEY_STATE_DEBOUNCE,        /* 首次按下消抖 */
    KEY_STATE_PRESSED,         /* 确认按下，计时是否达到长按 */
    KEY_STATE_WAIT_DOUBLE,     /* 松开后，等待双击窗口（超时则判定为单击） */
    KEY_STATE_DEBOUNCE_2,      /* 第二次按下消抖 */
    KEY_STATE_LONG_PRESS,      /* 长按已触发，等待按键释放 */
    KEY_STATE_WAIT_RELEASE,    /* 双击已触发，等待按键释放 */
} KeyState_TypeDef;

/* 按键控制句柄结构体（支持多按键实例化） */
typedef struct KeyHandle_t {
    GPIO_TypeDef *GPIO_Port;        /* 按键 GPIO 端口 (如 GPIOE) */
    uint16_t GPIO_Pin;              /* 按键引脚 (如 KEY_Pin) */
    GPIO_PinState ActiveLevel;      /* 有效触发电平 (默认 GPIO_PIN_RESET，按下接地) */
    
    /* 状态机内部计时与状态变量 */
    KeyState_TypeDef State;         /* 当前状态 */
    uint16_t PressTicks;            /* 按下持续时间计数 */
    uint16_t ReleaseTicks;          /* 松开持续时间计数 */
    KeyEvent_TypeDef Event;         /* 当前产生的按键事件 */
    
    /* 回调函数指针 (注册后事件触发时自动执行) */
    void (*SingleClickCallback)(void); /* 短按回调 */
    void (*DoubleClickCallback)(void); /* 双击回调 */
    void (*LongPressCallback)(void);   /* 长按回调 */
} KeyHandle_TypeDef;

/* 外部可用默认按键句柄 */
extern KeyHandle_TypeDef hkey_user;

/* 函数接口声明 */
void Key_Init(KeyHandle_TypeDef *hkey, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_level);
void Key_AttachCallback(KeyHandle_TypeDef *hkey, 
                        void (*single_click)(void), 
                        void (*double_click)(void), 
                        void (*long_press)(void));
void Key_Scan(KeyHandle_TypeDef *hkey);
KeyEvent_TypeDef Key_GetEvent(KeyHandle_TypeDef *hkey);

/* FreeRTOS 任务函数 */
void vKeyTask(void *pvParameters);

#endif /* __KEY_H_ */
