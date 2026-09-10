/**
 ******************************************************************************
 * @file    led.c
 * @brief   LED 驱动模块实现文件
 *
 * 提供 LED 的初始化、点亮、熄灭、切换及非阻塞闪烁功能。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* Private variables ---------------------------------------------------------*/

/** @defgroup LED_Private_Variables 私有变量
 * @{
 */

/** LED 剩余翻转次数 */
static uint8_t  blink_count;

/** LED 闪烁间隔时间（毫秒） */
static uint32_t blink_interval;

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup LED_Exported_Functions 导出函数实现
 * @{
 */

/**
 * @brief  LED 初始化函数
 * @retval 无
 */
void LED_Init(void)
{
    LED_OFF();
}

/**
 * @brief  点亮 LED
 * @retval 无
 */
void LED_ON(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, LED_ON_LEVEL);
}

/**
 * @brief  熄灭 LED
 * @retval 无
 */
void LED_OFF(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, LED_OFF_LEVEL);
}

/**
 * @brief  切换 LED 状态
 * @retval 无
 */
void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_PIN);
}

/**
 * @brief  LED 闪烁函数（非阻塞）
 * @param  interval: 闪烁间隔时间（毫秒）
 * @param  count: 闪烁次数
 * @retval 无
 * @note   调用后由 LED_Blink_Callback() 在后台执行，闪烁期间不接受新的闪烁请求
 */
void LED_Blink(uint32_t interval, uint8_t count)
{
    if (interval == 0 || count == 0) {
        return;
    }

    if (blink_count > 0) {
        return;
    }

    blink_count = count * 2;
    blink_interval = interval;
}

/**
 * @brief  LED 闪烁执行函数
 * @retval 无
 * @note   由 SysTick 中断函数 SysTick_Handler() 每 1ms 调用一次，非阻塞执行
 */
void LED_Blink_Callback(void)
{
    static uint32_t last_tick = 0;

    if (blink_count > 0) {
        last_tick++;

        if (last_tick >= blink_interval) {
            last_tick = 0;
            blink_count--;
            LED_Toggle();
        }
    }
}

/** @} */
