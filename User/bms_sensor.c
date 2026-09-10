/**
 ******************************************************************************
 * @file    bms_sensor.c
 * @brief   电池组BMS保护板Modbus RTU驱动实现文件
 *
 * 功能：
 *   - 通过 RS485 串口与电池组保护板进行 Modbus RTU 协议通信
 *   - 单次读取寄存器 0x0028（共1个寄存器，2字节数据）
 *     请求帧: 01 03 00 28 00 01 [CRC]
 *   - 响应帧: 设备地址(1) + 功能码(1) + 数据长度(1) + 数据(2) + CRC(2) = 7 字节
 *   - RSOC 解析: 高字节预留，低字节为百分比值，1 表示 1%，最大 100
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bms_sensor.h"
#include "usart.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/** @defgroup BMS_Sensor_Private_Defines 私有宏定义
 * @{
 */

/** 日志打印宏，根据开关决定是否编译 */
#if BMS_SENSOR_LOG_PRINT_EN
#define BMS_LOG_PRINT(fmt, ...)    printf("[BMSSensor] " fmt, ##__VA_ARGS__)
#else
#define BMS_LOG_PRINT(fmt, ...)
#endif

/** 响应帧总长度（字节）: 1(地址) + 1(功能码) + 1(数据长度) + 2(1寄存器数据) + 2(CRC) */
#define RX_FRAME_SIZE          7

/** 请求帧总长度（字节） */
#define TX_FRAME_SIZE          8

/** 响应帧中数据区域字节数（1个寄存器 × 2字节） */
#define DATA_BYTE_COUNT        2

/** RSOC 寄存器地址 */
#define RSOC_REG_ADDR          0x0028

/** 读取寄存器个数 */
#define READ_REG_COUNT         0x0001

/** @} */

/* Private variables ---------------------------------------------------------*/

/** @defgroup BMS_Sensor_Private_Variables 私有变量
 * @{
 */

/** 内部 RSOC 值（静态存储） */
static uint8_t s_rsoc = 0;

/** 请求帧缓冲区（运行时构建） */
static uint8_t tx_frame[TX_FRAME_SIZE];

/** @} */

/* Private functions ---------------------------------------------------------*/

/** @defgroup BMS_Sensor_Private_Functions 私有函数
 * @{
 */

/**
 * @brief  计算 Modbus CRC16 校验值
 * @param  data: 待校验数据缓冲区指针
 * @param  len: 数据长度
 * @retval uint16_t CRC16 校验值（多项式 0xA001，初值 0xFFFF）
 */
static uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  以表格形式打印 BMS 数据（调试用）
 * @param  data: BMS 数据指针
 * @retval 无
 */
static void PrintDataTable(uint8_t rsoc)
{
#if BMS_SENSOR_DEBUG_TABLE_EN
    printf("\r\n");
    printf("+----------------+------------------+\r\n");
    printf("|   Field        |      Value       |\r\n");
    printf("+----------------+------------------+\r\n");
    printf("| rsoc           | %-16u |\r\n", rsoc);
    printf("+----------------+------------------+\r\n");
    printf("\r\n");
#else
    (void)rsoc;
#endif
}

/**
 * @brief  构建请求帧并发送接收解析传感器响应
 * @retval BMSSensor_Status_t 操作状态
 */
static BMSSensor_Status_t SendAndReceive(void)
{
    uint8_t rx_buf[RX_FRAME_SIZE];
    uint16_t rx_len = 0;

    /* 构建请求帧：地址 + 功能码03 + 起始地址 + 寄存器个数 + CRC */
    tx_frame[0] = BMS_SENSOR_ADDR;
    tx_frame[1] = 0x03;
    tx_frame[2] = (uint8_t)(RSOC_REG_ADDR >> 8);
    tx_frame[3] = (uint8_t)(RSOC_REG_ADDR & 0xFF);
    tx_frame[4] = (uint8_t)(READ_REG_COUNT >> 8);
    tx_frame[5] = (uint8_t)(READ_REG_COUNT & 0xFF);

    uint16_t crc = CRC16_Modbus(tx_frame, 6);
    tx_frame[6] = (uint8_t)(crc & 0xFF);        /* CRC 低字节 */
    tx_frame[7] = (uint8_t)((crc >> 8) & 0xFF); /* CRC 高字节 */

    /* 发送请求帧 */
    if (HAL_UART_Transmit(&BMS_SENSOR_UART_HANDLE, tx_frame, TX_FRAME_SIZE, BMS_SENSOR_TIMEOUT_MS) != HAL_OK) {
        return BMS_SENSOR_ERR_TX;
    }

    /* 接收响应帧 */
    if (HAL_UARTEx_ReceiveToIdle(&BMS_SENSOR_UART_HANDLE, rx_buf, RX_FRAME_SIZE, &rx_len, BMS_SENSOR_TIMEOUT_MS) != HAL_OK) {
        return BMS_SENSOR_ERR_RX;
    }

    /* 校验帧长度 */
    if (rx_len != RX_FRAME_SIZE) {
        return BMS_SENSOR_ERR_LEN;
    }

    /* 校验帧头（设备地址、功能码、数据字节数） */
    if (rx_buf[0] != BMS_SENSOR_ADDR || rx_buf[1] != 0x03 || rx_buf[2] != DATA_BYTE_COUNT) {
        return BMS_SENSOR_ERR_FRAME;
    }

    /* 校验 CRC16 */
    uint16_t crc_calc = CRC16_Modbus(rx_buf, RX_FRAME_SIZE - 2);
    uint16_t crc_recv = ((uint16_t)rx_buf[RX_FRAME_SIZE - 1] << 8) | rx_buf[RX_FRAME_SIZE - 2];
    if (crc_calc != crc_recv) {
        return BMS_SENSOR_ERR_CRC;
    }

    /* 解析 RSOC：高字节预留，低字节为百分比值 */
    uint16_t rsoc_raw = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
    s_rsoc = (uint8_t)(rsoc_raw & 0xFF);

    return BMS_SENSOR_OK;
}

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup BMS_Sensor_Exported_Functions 导出函数实现
 * @{
 */

/**
 * @brief  初始化 BMS 传感器模块
 * @retval 无
 */
void BMSSensor_Init(void)
{
    s_rsoc = 0;
}

/**
 * @brief  更新 BMS RSOC 数据（单次读取寄存器 0x0028）
 * @retval BMSSensor_Status_t 操作状态
 */
BMSSensor_Status_t BMSSensor_Update(void)
{
    BMSSensor_Status_t status = SendAndReceive();

    if (status != BMS_SENSOR_OK) {
        switch (status) {
            case BMS_SENSOR_ERR_TX:   BMS_LOG_PRINT("TX failed\r\n"); break;
            case BMS_SENSOR_ERR_RX:   BMS_LOG_PRINT("RX failed\r\n"); break;
            case BMS_SENSOR_ERR_LEN:  BMS_LOG_PRINT("Frame length mismatch\r\n"); break;
            case BMS_SENSOR_ERR_FRAME:BMS_LOG_PRINT("Frame header error\r\n"); break;
            case BMS_SENSOR_ERR_CRC:  BMS_LOG_PRINT("CRC check failed\r\n"); break;
            default:                  BMS_LOG_PRINT("Unknown error\r\n"); break;
        }
    } else {
        PrintDataTable(s_rsoc);
    }

    return status;
}

/**
 * @brief  获取 BMS 电池组 RSOC 数据
 * @return uint8_t RSOC 百分比值，范围 0~100
 */
uint8_t BMSSensor_GetData(void)
{
    return s_rsoc;
}

/** @} */
