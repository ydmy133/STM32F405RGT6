/**
 ******************************************************************************
 * @file    heading_sensor.c
 * @brief   航向角传感器驱动实现文件(基于 Modbus RTU)
 *
 * 功能:
 *   - 通过 USART3 与航向角传感器进行 Modbus RTU 协议通信
 *   - 单次读取寄存器 0x37~0x3F(共9个寄存器,18字节数据),同时获取角度和角速度
 *     请求帧: 60 03 00 37 00 09 [CRC]
 *   - 响应帧: 设备地址(1) + 功能码(1) + 数据长度(1) + 数据(18) + CRC(2) = 23 字节
 *   - 角度转换: angle = raw / 32768.0 * 180.0
 *   - 角速度转换: gyro = raw / 32768.0 * 2000.0
 *   - 航向:取反(逆时针→顺时针) + 磁偏角校正 + 归一到 0~360°
 *
 * 增强(相对参考工程):
 *   - 数据新鲜度:距上次成功采集超过 500ms 判 HEADING_SENSOR_TIMEOUT
 *   - 帧头容错:帧头校验失败时沿用上次值并计数,连续超过 3 次才作为硬错误,
 *     调用方用 HeadingSensor_GetFrameErrCount() 区分软/硬
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "heading_sensor.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/** 日志打印宏,根据开关决定是否编译 */
#if HEADING_SENSOR_LOG_PRINT_EN
#define HEADING_LOG_PRINT(fmt, ...)    printf("[HeadingSensor] " fmt, ##__VA_ARGS__)
#else
#define HEADING_LOG_PRINT(fmt, ...)
#endif

/** 响应帧总长度(字节): 1(地址) + 1(功能码) + 1(数据长度) + 18(数据) + 2(CRC) */
#define RX_FRAME_SIZE          23

/** 请求帧总长度(字节) */
#define TX_FRAME_SIZE          8

/** 响应帧中数据区域字节数(9个寄存器 × 2字节) */
#define DATA_BYTE_COUNT        18

/* Private variables ---------------------------------------------------------*/

/** 内部传感器数据缓冲区(静态存储) */
static HeadingSensor_Data_t sensor_data = {0};

/** 请求帧: 地址 0x60, 功能码 0x03, 起始地址 0x0037, 寄存器数 0x0009, CRC 0x733C */
static const uint8_t tx_frame[TX_FRAME_SIZE] = {0x60, 0x03, 0x00, 0x37, 0x00, 0x09, 0x3C, 0x73};

/** 上次成功采集时刻(ms),用于新鲜度监测 */
static uint32_t s_last_success_tick = 0;

/** 连续帧头错误计数,成功采集后清零 */
static uint8_t s_frame_err_count = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  计算 Modbus CRC16 校验值
 * @param  data: 待校验数据缓冲区指针
 * @param  len: 数据长度
 * @retval uint16_t CRC16 校验值(多项式 0xA001,初值 0xFFFF)
 */
static uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
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
 * @brief  以表格形式打印传感器数据(调试用)
 * @param  data: 传感器数据指针
 * @retval 无
 */
static void PrintDataTable(const HeadingSensor_Data_t *data)
{
#if HEADING_SENSOR_LOG_PRINT_EN
    if (data == NULL) return;

    printf("\r\n");
    printf("+----------------+------------------+\r\n");
    printf("|   Field        |      Value       |\r\n");
    printf("+----------------+------------------+\r\n");
    printf("| roll           | %-16.2f |\r\n", (double)data->roll);
    printf("| pitch          | %-16.2f |\r\n", (double)data->pitch);
    printf("| yaw            | %-16.2f |\r\n", (double)data->yaw);
    printf("| gx             | %-16.2f |\r\n", (double)data->gx);
    printf("| gy             | %-16.2f |\r\n", (double)data->gy);
    printf("| gz             | %-16.2f |\r\n", (double)data->gz);
    printf("+----------------+------------------+\r\n");
    printf("\r\n");
#else
    (void)data;
#endif
}

/**
 * @brief  发送请求并接收解析传感器响应
 * @retval HeadingSensor_Status_t 操作状态
 * @note   校验链:帧长 → 帧头(地址/功能码/字节数)→ CRC16
 */
static HeadingSensor_Status_t SendAndReceive(void)
{
    uint8_t rx_buf[RX_FRAME_SIZE];
    uint16_t rx_len = 0;

    /* 发送请求帧 */
    if (HAL_UART_Transmit(&HEADING_SENSOR_UART_HANDLE, (uint8_t*)tx_frame, TX_FRAME_SIZE, HEADING_SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
        return HEADING_SENSOR_ERR_TX;
    }

    /* 接收响应帧 */
    if (HAL_UARTEx_ReceiveToIdle(&HEADING_SENSOR_UART_HANDLE, rx_buf, RX_FRAME_SIZE, &rx_len, HEADING_SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
        return HEADING_SENSOR_ERR_RX;
    }

    /* 校验帧长度 */
    if (rx_len != RX_FRAME_SIZE) {
        return HEADING_SENSOR_ERR_LEN;
    }

    /* 校验帧头(设备地址、功能码、数据字节数) */
    if (rx_buf[0] != HEADING_SENSOR_ADDR || rx_buf[1] != 0x03 || rx_buf[2] != DATA_BYTE_COUNT) {
        return HEADING_SENSOR_ERR_FRAME;
    }

    /* 校验 CRC16 */
    {
        uint16_t crc_calc = CRC16_Modbus(rx_buf, RX_FRAME_SIZE - 2);
        uint16_t crc_recv = ((uint16_t)rx_buf[RX_FRAME_SIZE - 1] << 8) | rx_buf[RX_FRAME_SIZE - 2];
        if (crc_calc != crc_recv) {
            return HEADING_SENSOR_ERR_CRC;
        }
    }

    /* 解析角速度数据(寄存器 0x37~0x39,偏移 3~8) */
    {
        int16_t gx_raw = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[4]);
        int16_t gy_raw = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[6]);
        int16_t gz_raw = (int16_t)((uint16_t)rx_buf[7] << 8 | rx_buf[8]);

        sensor_data.gx = gx_raw / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_GYRO_RANGE;
        sensor_data.gy = gy_raw / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_GYRO_RANGE;
        sensor_data.gz = gz_raw / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_GYRO_RANGE;
    }

    /* 解析角度数据(寄存器 0x3D~0x3F,偏移 15~20) */
    {
        int16_t roll_raw  = (int16_t)((uint16_t)rx_buf[15] << 8 | rx_buf[16]);
        int16_t pitch_raw = (int16_t)((uint16_t)rx_buf[17] << 8 | rx_buf[18]);
        int16_t yaw_raw   = (int16_t)((uint16_t)rx_buf[19] << 8 | rx_buf[20]);

        sensor_data.roll  = roll_raw  / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_ANGLE_RANGE;
        sensor_data.pitch = pitch_raw / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_ANGLE_RANGE;

        /* 航向:取反以转换为顺时针增加,加磁偏角校正,归一到 0~360° */
        float yaw_standard = -yaw_raw / HEADING_SENSOR_RAW_SCALE * HEADING_SENSOR_ANGLE_RANGE;
        yaw_standard += HEADING_SENSOR_MAG_DECLINATION;
        sensor_data.yaw = fmodf(yaw_standard + 360.0f, 360.0f);
    }

    return HEADING_SENSOR_OK;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化航向角传感器模块
 * @retval 无
 */
void HeadingSensor_Init(void)
{
    sensor_data.roll  = 0.0f;
    sensor_data.pitch = 0.0f;
    sensor_data.yaw   = 0.0f;
    sensor_data.gx    = 0.0f;
    sensor_data.gy    = 0.0f;
    sensor_data.gz    = 0.0f;

    s_last_success_tick = 0;
    s_frame_err_count   = 0;
}

/**
 * @brief  更新传感器数据(单次读取全部寄存器 0x37~0x3F)
 * @retval HeadingSensor_Status_t 操作状态
 * @note   增强逻辑:
 *         - 成功采集:记录时间、清零帧头计数,返回 OK
 *         - 帧头失败:沿用上次值并计数,连续 >FRAME_ERR_MAX 次返回 ERR_FRAME;
 *           调用方用 HeadingSensor_GetFrameErrCount() 判断 ≤3 次为可恢复
 *         - 距上次成功采集 >FRESH_TIMEOUT_MS:返回 TIMEOUT
 *         - 其余失败保留具体错误码返回,数据保持上次有效值
 */
HeadingSensor_Status_t HeadingSensor_Update(void)
{
    HeadingSensor_Status_t status = SendAndReceive();

    if (status == HEADING_SENSOR_OK)
    {
        s_last_success_tick = HAL_GetTick();
        s_frame_err_count   = 0;
        PrintDataTable(&sensor_data);
        return HEADING_SENSOR_OK;
    }

    /* 帧头校验失败:沿用上次采集的值并计数,连续超过上限才作为硬错误返回 */
    if (status == HEADING_SENSOR_ERR_FRAME)
    {
        s_frame_err_count++;
        if (s_frame_err_count > HEADING_SENSOR_FRAME_ERR_MAX)
        {
            HEADING_LOG_PRINT("Frame header error (count=%d)\r\n", s_frame_err_count);
            return HEADING_SENSOR_ERR_FRAME;
        }
        /* ≤3 次:沿用上次值,返回具体错误码(调用方据计数判断软/硬) */
    }

    /* 新鲜度监测:距上次成功采集超过 500ms → 判 TIMEOUT */
    if ((HAL_GetTick() - s_last_success_tick) > HEADING_SENSOR_FRESH_TIMEOUT_MS)
    {
        return HEADING_SENSOR_TIMEOUT;
    }

    /* 未超时:保留具体错误码返回 */
    return status;
}

/**
 * @brief  获取传感器数据
 * @return HeadingSensor_Data_t 传感器数据(失败时保持上次有效值)
 */
HeadingSensor_Data_t HeadingSensor_GetData(void)
{
    return sensor_data;
}

/**
 * @brief  获取连续帧头错误计数
 * @return uint8_t 当前连续帧头失败次数(成功采集后清零)
 */
uint8_t HeadingSensor_GetFrameErrCount(void)
{
    return s_frame_err_count;
}
