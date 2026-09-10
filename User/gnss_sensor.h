/**
 ******************************************************************************
 * @file    gnss_sensor.h
 * @brief   GNSS 传感器驱动头文件(基于 Modbus RTU)
 *
 * 相对参考工程(STM32F405RGT6)的增强:
 *   - 数据新鲜度监测:距上次成功采集超过 GNSS_SENSOR_FRESH_TIMEOUT_MS(500ms),
 *     返回 GNSS_SENSOR_TIMEOUT
 *   - 帧头校验容错:帧头校验失败时沿用上次采集值并计数,连续超过
 *     GNSS_SENSOR_FRAME_ERR_MAX(3)次才作为硬错误;调用方可用
 *     GNSSSensor_GetFrameErrCount() 区分"≤3 次可恢复"与">3 次硬错误"
 ******************************************************************************
 */

#ifndef __GNSS_SENSOR_H
#define __GNSS_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/** 日志打印开关 (1=开启, 0=关闭);移植后关闭,避免每成功一帧打印整表 */
#define GNSS_SENSOR_LOG_PRINT_EN       0

/** 通信串口句柄(USART2, 4800) */
#define GNSS_SENSOR_UART_HANDLE        huart2

/** 从站地址 */
#define GNSS_SENSOR_ADDR               0x50

/** 单次接收 HAL 超时(ms) */
#define GNSS_SENSOR_RX_TIMEOUT_MS      100u

/** 数据新鲜度阈值(ms):距上次成功采集超过此值判 TIMEOUT(增强) */
#define GNSS_SENSOR_FRESH_TIMEOUT_MS   500u

/** 帧头连续错误上限:超过此值判 ERR_FRAME 硬错误(增强) */
#define GNSS_SENSOR_FRAME_ERR_MAX      3u

/** 经纬度组合除数 */
#define GNSS_LONLAT_DIVISOR            10000000U

/** 海拔除数(m) */
#define GNSS_HEIGHT_DIVISOR            10.0f

/** 航向除数(°) */
#define GNSS_YAW_DIVISOR               100.0f

/** 地速除数(km/h,与参考工程原遥测语义一致;main 里再 /3.6 转 m/s 上报) */
#define GNSS_SPEED_DIVISOR             1000.0f

/* Exported types ------------------------------------------------------------*/

/** GNSS 传感器状态 */
typedef enum {
    GNSS_SENSOR_OK = 0,        /**< 成功采集到新数据 */
    GNSS_SENSOR_ERR_TX,        /**< 发送请求失败 */
    GNSS_SENSOR_ERR_RX,        /**< 接收响应失败 */
    GNSS_SENSOR_ERR_LEN,       /**< 响应帧长度不符 */
    GNSS_SENSOR_ERR_FRAME,     /**< 帧头校验失败(连续 >MAX 次为硬错误) */
    GNSS_SENSOR_ERR_CRC,       /**< CRC 校验失败 */
    GNSS_SENSOR_TIMEOUT        /**< 距上次成功采集超过 FRESH_TIMEOUT_MS(增强) */
} GNSSSensor_Status_t;

/** GNSS 数据 */
typedef struct {
    float longitude;           /**< 经度(十进制度),东经为正 */
    float latitude;            /**< 纬度(十进制度),北纬为正 */
    float height;              /**< 海拔(m) */
    float yaw;                 /**< 航向(°) */
    float speed;               /**< 地速(m/s) */
} GNSSSensor_Data_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化 GNSS 传感器模块
 * @note   清零数据与内部状态,需在串口初始化完成后调用
 */
void GNSSSensor_Init(void);

/**
 * @brief  更新 GNSS 数据(单次读取寄存器 0x49~0x50)
 * @retval GNSSSensor_Status_t 操作状态
 * @note   成功采集后记录时间并清零帧头计数;
 *         帧头失败沿用上次值并计数,连续 >FRAME_ERR_MAX 次返回 ERR_FRAME;
 *         距上次成功 >FRESH_TIMEOUT_MS 返回 TIMEOUT
 */
GNSSSensor_Status_t GNSSSensor_Update(void);

/**
 * @brief  获取 GNSS 数据
 * @return GNSSSensor_Data_t GNSS 数据(失败时保持上次有效值)
 */
GNSSSensor_Data_t GNSSSensor_GetData(void);

/**
 * @brief  获取连续帧头错误计数
 * @return uint8_t 当前连续帧头失败次数(成功采集后清零)
 */
uint8_t GNSSSensor_GetFrameErrCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_SENSOR_H */
