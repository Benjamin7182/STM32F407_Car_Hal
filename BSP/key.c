#include "key.h"
#include "headfile.h"

/* 默认按键实例（对应板载 KEY 引脚：PE0） */
KeyHandle_TypeDef hkey_user;

/* 时间参数配置（基于 10ms 扫描周期） */
#define KEY_DEBOUNCE_TICKS       2    /* 消抖时间：2 * 10ms = 20ms */
#define KEY_DOUBLE_CLICK_TICKS  25    /* 双击判定窗口：25 * 10ms = 250ms */
#define KEY_LONG_PRESS_TICKS   100    /* 长按判定时间：100 * 10ms = 1000ms (1秒) */

/**
 * @brief 初始化按键句柄
 * @param hkey 按键句柄指针
 * @param port GPIO 端口 (如 GPIOE)
 * @param pin  GPIO 引脚 (如 KEY_Pin)
 * @param active_level 有效触发电平 (按下拉低通常为 GPIO_PIN_RESET)
 */
void Key_Init(KeyHandle_TypeDef *hkey, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_level)
{
    if (hkey == NULL) return;

    hkey->GPIO_Port = port;
    hkey->GPIO_Pin = pin;
    hkey->ActiveLevel = active_level;
    hkey->State = KEY_STATE_IDLE;
    hkey->PressTicks = 0;
    hkey->ReleaseTicks = 0;
    hkey->Event = KEY_EVENT_NONE;
    hkey->SingleClickCallback = NULL;
    hkey->DoubleClickCallback = NULL;
    hkey->LongPressCallback = NULL;
}

/**
 * @brief 注册按键事件回调函数
 * @param hkey 按键句柄指针
 * @param single_click 单击/短按回调函数（不需要可传 NULL）
 * @param double_click 双击回调函数（不需要可传 NULL）
 * @param long_press   长按回调函数（不需要可传 NULL）
 */
void Key_AttachCallback(KeyHandle_TypeDef *hkey, 
                        void (*single_click)(void), 
                        void (*double_click)(void), 
                        void (*long_press)(void))
{
    if (hkey == NULL) return;

    hkey->SingleClickCallback = single_click;
    hkey->DoubleClickCallback = double_click;
    hkey->LongPressCallback = long_press;
}

/**
 * @brief 查询并清除当前按键事件（用于轮询模式）
 * @param hkey 按键句柄指针
 * @return KeyEvent_TypeDef 按键事件
 */
KeyEvent_TypeDef Key_GetEvent(KeyHandle_TypeDef *hkey)
{
    if (hkey == NULL) return KEY_EVENT_NONE;

    KeyEvent_TypeDef evt = hkey->Event;
    hkey->Event = KEY_EVENT_NONE; /* 读取后清除事件 */
    return evt;
}

/**
 * @brief 按键扫描核心状态机（必须以固定周期调用，推荐 10ms）
 * @param hkey 按键句柄指针
 */
void Key_Scan(KeyHandle_TypeDef *hkey)
{
    if (hkey == NULL || hkey->GPIO_Port == NULL) return;

    /* 检测当前引脚是否处于有效按下电平 */
    uint8_t is_pressed = (HAL_GPIO_ReadPin(hkey->GPIO_Port, hkey->GPIO_Pin) == hkey->ActiveLevel);

    switch (hkey->State)
    {
        case KEY_STATE_IDLE:
            if (is_pressed)
            {
                hkey->PressTicks = 0;
                hkey->State = KEY_STATE_DEBOUNCE;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            if (is_pressed)
            {
                hkey->PressTicks++;
                if (hkey->PressTicks >= KEY_DEBOUNCE_TICKS)
                {
                    /* 经过消抖确认首次按下，进入按下保持状态 */
                    hkey->State = KEY_STATE_PRESSED;
                }
            }
            else
            {
                /* 抖动干扰，返回空闲状态 */
                hkey->State = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            if (is_pressed)
            {
                hkey->PressTicks++;
                if (hkey->PressTicks >= KEY_LONG_PRESS_TICKS)
                {
                    /* 按住达到长按时间，立即触发长按事件 */
                    hkey->Event = KEY_EVENT_LONG_PRESS;
                    if (hkey->LongPressCallback != NULL)
                    {
                        hkey->LongPressCallback();
                    }
                    /* 进入长按保持状态，避免松开时误触发单击 */
                    hkey->State = KEY_STATE_LONG_PRESS;
                }
            }
            else
            {
                /* 在达到长按前松开，进入等待双击状态 */
                hkey->ReleaseTicks = 0;
                hkey->State = KEY_STATE_WAIT_DOUBLE;
            }
            break;

        case KEY_STATE_WAIT_DOUBLE:
            if (is_pressed)
            {
                /* 在双击时间窗口内再次检测到按下，进入第二次消抖 */
                hkey->PressTicks = 0;
                hkey->State = KEY_STATE_DEBOUNCE_2;
            }
            else
            {
                hkey->ReleaseTicks++;
                if (hkey->ReleaseTicks >= KEY_DOUBLE_CLICK_TICKS)
                {
                    /* 双击窗口超时仍未检测到第二次按下，确认为单击/短按 */
                    hkey->Event = KEY_EVENT_SINGLE_CLICK;
                    if (hkey->SingleClickCallback != NULL)
                    {
                        hkey->SingleClickCallback();
                    }
                    hkey->State = KEY_STATE_IDLE;
                }
            }
            break;

        case KEY_STATE_DEBOUNCE_2:
            if (is_pressed)
            {
                hkey->PressTicks++;
                if (hkey->PressTicks >= KEY_DEBOUNCE_TICKS)
                {
                    /* 确认第二次按下，触发双击事件 */
                    hkey->Event = KEY_EVENT_DOUBLE_CLICK;
                    if (hkey->DoubleClickCallback != NULL)
                    {
                        hkey->DoubleClickCallback();
                    }
                    /* 等待按键释放，防止重复触发 */
                    hkey->State = KEY_STATE_WAIT_RELEASE;
                }
            }
            else
            {
                /* 抖动，退回等待双击 */
                hkey->State = KEY_STATE_WAIT_DOUBLE;
            }
            break;

        case KEY_STATE_LONG_PRESS:
        case KEY_STATE_WAIT_RELEASE:
            if (!is_pressed)
            {
                /* 按键完全松开，回到空闲状态 */
                hkey->State = KEY_STATE_IDLE;
            }
            break;

        default:
            hkey->State = KEY_STATE_IDLE;
            break;
    }
}

/* ================= 示例：测试回调函数 ================= */

/**
 * @brief 短按示例回调：翻转 LED
 */
static void OnKeySingleClick(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    target_speed_l = (target_speed_l < 450.0f) ? 600.0f : 300.0f;
}

/**
 * @brief 双击示例回调：蜂鸣器发声提示
 */
static void OnKeyDoubleClick(void)
{
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
}

/**
 * @brief 长按示例回调：同时翻转 LED 和蜂鸣器
 */
static void OnKeyLongPress(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
}

/**
 * @brief FreeRTOS 按键服务任务
 */
void vKeyTask(void *pvParameters)
{
    /* 初始化板载按键 (PE0, 低电平有效) */
    Key_Init(&hkey_user, KEY_GPIO_Port, KEY_Pin, GPIO_PIN_RESET);

    /* 绑定短按、双击、长按回调 */
    Key_AttachCallback(&hkey_user, OnKeySingleClick, OnKeyDoubleClick, OnKeyLongPress);

    while (1)
    {
        /* 周期性执行按键扫描 */
        Key_Scan(&hkey_user);

        /* 10ms 扫描周期 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
