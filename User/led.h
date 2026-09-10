/**
 ******************************************************************************
 * @file    led.h
 * @brief   LED 驱动模块头文件
 *
 * 提供 LED 的初始化、点亮、熄灭、切换及非阻塞闪烁功能。
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/** @defgroup LED_Exported_Constants LED 模块导出常量
 * @{
 */

/** LED GPIO 端口 */
#define LED_GPIO_Port       GPIOB

/** LED 引脚编号 */
#define LED_PIN             GPIO_PIN_12

/** LED 点亮电平（低电平） */
#define LED_ON_LEVEL        GPIO_PIN_RESET

/** LED 熄灭电平（高电平） */
#define LED_OFF_LEVEL       GPIO_PIN_SET

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup LED_Exported_Functions LED 模块导出函数
 * @{
 */

/**
 * @brief  LED 初始化函数
 * @retval 无
 */
void LED_Init(void);

/**
 * @brief  点亮 LED
 * @retval 无
 */
void LED_ON(void);

/**
 * @brief  熄灭 LED
 * @retval 无
 */
void LED_OFF(void);

/**
 * @brief  切换 LED 状态
 * @retval 无
 */
void LED_Toggle(void);

/**
 * @brief  LED 闪烁函数（非阻塞）
 * @param  interval: 闪烁间隔时间（毫秒）
 * @param  count: 闪烁次数
 * @retval 无
 * @note   调用后由 LED_Blink_Callback() 在后台执行，闪烁期间不接受新的闪烁请求
 */
void LED_Blink(uint32_t interval, uint8_t count);

/**
 * @brief  LED 闪烁执行函数
 * @retval 无
 * @note   由 SysTick 中断函数 SysTick_Handler() 每 1ms 调用一次，非阻塞执行
 */
void LED_Blink_Callback(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
