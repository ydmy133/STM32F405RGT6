/**
 ******************************************************************************
 * @file    heading_sensor.h
 * @brief   航向角传感器驱动头文件(基于 Modbus RTU)
 *
 * 相对参考工程(STM32F405RGT6)的增强:
 *   - 数据新鲜度监测:距上次成功采集超过 HEADING_SENSOR_FRESH_TIMEOUT_MS(500ms),
 *     返回 HEADING_SENSOR_TIMEOUT
 *   - 帧头校验容错:帧头校验失败时沿用上次采集值并计数,连续超过
 *     HEADING_SENSOR_FRAME_ERR_MAX(3)次才作为硬错误;调用方可用
 *     HeadingSensor_GetFrameErrCount() 区分"≤3 次可恢复"与">3 次硬错误"
 ******************************************************************************
 */

#ifndef __HEADING_SENSOR_H
#define __HEADING_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/** 日志打印开关 (1=开启, 0=关闭) */
#define HEADING_SENSOR_LOG_PRINT_EN       0

/** 通信串口句柄(USART3, 4800) */
#define HEADING_SENSOR_UART_HANDLE        huart3

/** 从站地址 */
#define HEADING_SENSOR_ADDR               0x60

/** 单次接收 HAL 超时(ms) */
#define HEADING_SENSOR_RX_TIMEOUT_MS      100u

/** 数据新鲜度阈值(ms):距上次成功采集超过此值判 TIMEOUT(增强) */
#define HEADING_SENSOR_FRESH_TIMEOUT_MS   500u

/** 帧头连续错误上限:超过此值判 ERR_FRAME 硬错误(增强) */
#define HEADING_SENSOR_FRAME_ERR_MAX      3u

/** 原始值满量程分母(int16 最大值) */
#define HEADING_SENSOR_RAW_SCALE          32768.0f

/** 角度量程(±180°) */
#define HEADING_SENSOR_ANGLE_RANGE        180.0f

/** 角速度量程(±2000°/s) */
#define HEADING_SENSOR_GYRO_RANGE         2000.0f

/** 磁偏角(东偏为正,西偏为负,单位:度) */
#define HEADING_SENSOR_MAG_DECLINATION    (-3.27f)

/* Exported types ------------------------------------------------------------*/

/** 航向角传感器状态 */
typedef enum {
    HEADING_SENSOR_OK = 0,       /**< 成功采集到新数据 */
    HEADING_SENSOR_ERR_TX,       /**< 发送请求失败 */
    HEADING_SENSOR_ERR_RX,       /**< 接收响应失败 */
    HEADING_SENSOR_ERR_LEN,      /**< 响应帧长度不符 */
    HEADING_SENSOR_ERR_FRAME,    /**< 帧头校验失败(连续 >MAX 次为硬错误) */
    HEADING_SENSOR_ERR_CRC,      /**< CRC 校验失败 */
    HEADING_SENSOR_TIMEOUT       /**< 距上次成功采集超过 FRESH_TIMEOUT_MS(增强) */
} HeadingSensor_Status_t;

/** 航向角传感器数据(角度 + 角速度) */
typedef struct {
    float roll;                  /**< 横滚角(°) */
    float pitch;                 /**< 俯仰角(°) */
    float yaw;                   /**< 航向角(°,0~360,正北为0,顺时针增) */
    float gx;                    /**< 角速度X(°/s) */
    float gy;                    /**< 角速度Y(°/s) */
    float gz;                    /**< 角速度Z(航向角速度)(°/s) */
} HeadingSensor_Data_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化航向角传感器模块
 * @note   清零数据与内部状态,需在串口初始化完成后调用
 */
void HeadingSensor_Init(void);

/**
 * @brief  更新传感器数据(单次读取寄存器 0x37~0x3F)
 * @retval HeadingSensor_Status_t 操作状态
 * @note   成功采集后记录时间并清零帧头计数;
 *         帧头失败沿用上次值并计数,连续 >FRAME_ERR_MAX 次返回 ERR_FRAME;
 *         距上次成功 >FRESH_TIMEOUT_MS 返回 TIMEOUT
 */
HeadingSensor_Status_t HeadingSensor_Update(void);

/**
 * @brief  获取传感器数据
 * @return HeadingSensor_Data_t 传感器数据(失败时保持上次有效值)
 */
HeadingSensor_Data_t HeadingSensor_GetData(void);

/**
 * @brief  获取连续帧头错误计数
 * @return uint8_t 当前连续帧头失败次数(成功采集后清零)
 */
uint8_t HeadingSensor_GetFrameErrCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __HEADING_SENSOR_H */
