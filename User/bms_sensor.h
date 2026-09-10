/**
 ******************************************************************************
 * @file    bms_sensor.h
 * @brief   电池组BMS保护板Modbus RTU驱动头文件
 *
 * 基于 Modbus RTU 协议的电池组保护板通信接口。
 * 通过 RS485 串口进行数据收发，采用 HAL_UARTEx_ReceiveToIdle() 轮询阻塞式接收。
 * 支持读取电池组 RSOC（剩余容量百分比）数据。
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMS_SENSOR_H
#define __BMS_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/** @defgroup BMS_Sensor_Exported_Constants BMS传感器导出常量
 * @{
 */

/** BMS传感器使用的串口句柄 */
#define BMS_SENSOR_UART_HANDLE  huart1

/** BMS传感器日志打印开关 (1=开启, 0=关闭) */
#define BMS_SENSOR_LOG_PRINT_EN         1

/** BMS传感器数据更新成功后以表格形式打印数据 (1=开启, 0=关闭) */
#define BMS_SENSOR_DEBUG_TABLE_EN       0

/** 电池组保护板设备地址（Modbus从机地址，可设定） */
#define BMS_SENSOR_ADDR         0x01

/** UART 发送/接收超时时间（毫秒） */
#define BMS_SENSOR_TIMEOUT_MS   100

/** @} */

/* Exported types ------------------------------------------------------------*/

/** @defgroup BMS_Sensor_Exported_Types BMS传感器导出类型
 * @{
 */

/**
 * @brief 传感器操作状态枚举
 */
typedef enum {
    BMS_SENSOR_OK = 0,             /**< 操作成功 */
    BMS_SENSOR_ERR_TX,             /**< 发送失败（UART 超时或硬件错误） */
    BMS_SENSOR_ERR_RX,             /**< 接收失败（UART 超时或硬件错误） */
    BMS_SENSOR_ERR_LEN,            /**< 接收数据长度不匹配预期帧长 */
    BMS_SENSOR_ERR_FRAME,          /**< 帧头校验失败（地址/功能码/数据字节数不匹配） */
    BMS_SENSOR_ERR_CRC             /**< CRC16 校验失败 */
} BMSSensor_Status_t;

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup BMS_Sensor_Exported_Functions BMS传感器导出函数
 * @{
 */

/**
 * @brief  初始化 BMS 传感器模块
 * @note   在系统启动阶段调用一次，将所有数据清零
 * @retval 无
 */
void BMSSensor_Init(void);

/**
 * @brief  更新 RSOC 数据（单次读取寄存器 0x0028）
 * @note   阻塞式调用，依次执行以下操作：
 *         1. 通过串口发送 8 字节请求帧
 *         2. 使用 HAL_UARTEx_ReceiveToIdle() 接收 7 字节响应帧
 *         3. 解析响应帧，进行帧头和 CRC16 校验
 *         4. 将解析后的 RSOC 值存储到内部缓冲区
 * @retval BMSSensor_Status_t 操作状态，返回 BMS_SENSOR_OK 表示数据有效
 */
BMSSensor_Status_t BMSSensor_Update(void);

/**
 * @brief  获取最新的电池组 RSOC 数据
 * @note   返回内部静态数据结构，仅在下次调用 BMSSensor_Update()
 *         成功时才会更新
 * @return uint8_t RSOC 百分比值，范围 0~100
 */
uint8_t BMSSensor_GetData(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __BMS_SENSOR_H */
