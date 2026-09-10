/**
 ******************************************************************************
 * @file    gnss_sensor.c
 * @brief   GNSS 传感器驱动实现文件(基于 Modbus RTU)
 *
 * 功能:
 *   - 通过 USART2 与 GNSS 传感器进行 Modbus RTU 协议通信
 *   - 单次读取寄存器 0x49~0x50(共8个寄存器,16字节数据)
 *     请求帧: 50 03 00 49 00 08 [CRC]
 *   - 响应帧: 设备地址(1) + 功能码(1) + 数据长度(1) + 数据(16) + CRC(2) = 21 字节
 *   - 数据转换公式:
 *     * 经纬度: degree = raw_32 / 10000000, DD = degree + minute/60.0
 *     * 海拔: height = raw / 10.0 (m)
 *     * 航向: yaw = raw / 100.0 (°)
 *     * 地速: speed = raw_32 / 1000.0 (km/h,与参考工程遥测语义一致)
 *
 * 增强(相对参考工程):
 *   - 数据新鲜度:距上次成功采集超过 500ms 判 GNSS_SENSOR_TIMEOUT
 *   - 帧头容错:帧头校验失败时沿用上次值并计数,连续超过 3 次才作为硬错误,
 *     调用方用 GNSSSensor_GetFrameErrCount() 区分软/硬
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gnss_sensor.h"
#include "usart.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/** 日志打印宏,根据开关决定是否编译 */
#if GNSS_SENSOR_LOG_PRINT_EN
#define GNSS_LOG_PRINT(fmt, ...)    printf("[GNSSSensor] " fmt, ##__VA_ARGS__)
#else
#define GNSS_LOG_PRINT(fmt, ...)
#endif

/** 响应帧总长度(字节): 1(地址) + 1(功能码) + 1(数据长度) + 16(数据) + 2(CRC) */
#define RX_FRAME_SIZE          21

/** 请求帧总长度(字节) */
#define TX_FRAME_SIZE          8

/** 响应帧中数据区域字节数(8个寄存器 × 2字节) */
#define DATA_BYTE_COUNT        16

/** 经纬度组合时的位移位数(16位拼接为32位) */
#define LONLAT_SHIFT_BITS      16

/** 经纬度小数部分转换系数 */
#define LONLAT_FRAC_DIVISOR    100000.0f

/* Private variables ---------------------------------------------------------*/

/** 内部 GNSS 数据缓冲区(静态存储) */
static GNSSSensor_Data_t sensor_data = {0};

/** 全量请求帧: 地址 0x50, 功能码 0x03, 起始地址 0x0049, 寄存器数 0x0008, CRC 0x985B */
static const uint8_t tx_all[TX_FRAME_SIZE] = {0x50, 0x03, 0x00, 0x49, 0x00, 0x08, 0x98, 0x5B};

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
 * @brief  以表格形式打印 GNSS 数据(调试用)
 * @param  data: GNSS 数据指针
 * @retval 无
 */
static void PrintDataTable(const GNSSSensor_Data_t *data)
{
#if GNSS_SENSOR_LOG_PRINT_EN
    if (data == NULL) return;

    printf("\r\n");
    printf("+----------------+------------------+\r\n");
    printf("|   Field        |      Value       |\r\n");
    printf("+----------------+------------------+\r\n");
    printf("| longitude      | %-16.6f |\r\n", (double)data->longitude);
    printf("| latitude       | %-16.6f |\r\n", (double)data->latitude);
    printf("| height         | %-16.1f |\r\n", (double)data->height);
    printf("| yaw            | %-16.1f |\r\n", (double)data->yaw);
    printf("| speed          | %-16.3f |\r\n", (double)data->speed);
    printf("+----------------+------------------+\r\n");
    printf("\r\n");
#else
    (void)data;
#endif
}

/**
 * @brief  发送请求并接收解析传感器响应
 * @param  tx_buf: 请求帧数据指针
 * @retval GNSSSensor_Status_t 操作状态
 * @note   校验链:帧长 → 帧头(地址/功能码/字节数)→ CRC16
 */
static GNSSSensor_Status_t SendAndReceive(const uint8_t *tx_buf)
{
    uint8_t rx_buf[RX_FRAME_SIZE];
    uint16_t rx_len = 0;

    /* 发送请求帧 */
    if (HAL_UART_Transmit(&GNSS_SENSOR_UART_HANDLE, (uint8_t*)tx_buf, TX_FRAME_SIZE, GNSS_SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
        return GNSS_SENSOR_ERR_TX;
    }

    /* 接收响应帧 */
    if (HAL_UARTEx_ReceiveToIdle(&GNSS_SENSOR_UART_HANDLE, rx_buf, RX_FRAME_SIZE, &rx_len, GNSS_SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
        return GNSS_SENSOR_ERR_RX;
    }

    /* 校验帧长度 */
    if (rx_len != RX_FRAME_SIZE) {
        return GNSS_SENSOR_ERR_LEN;
    }

    /* 校验帧头(设备地址、功能码、数据字节数) */
    if (rx_buf[0] != GNSS_SENSOR_ADDR || rx_buf[1] != 0x03 || rx_buf[2] != DATA_BYTE_COUNT) {
        return GNSS_SENSOR_ERR_FRAME;
    }

    /* 校验 CRC16 */
    {
        uint16_t crc_calc = CRC16_Modbus(rx_buf, RX_FRAME_SIZE - 2);
        uint16_t crc_recv = ((uint16_t)rx_buf[RX_FRAME_SIZE - 1] << 8) | rx_buf[RX_FRAME_SIZE - 2];
        if (crc_calc != crc_recv) {
            return GNSS_SENSOR_ERR_CRC;
        }
    }

    /* 解析经纬度 */
    {
        uint16_t lon_l = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
        uint16_t lon_h = ((uint16_t)rx_buf[5] << 8) | rx_buf[6];
        uint16_t lat_l = ((uint16_t)rx_buf[7] << 8) | rx_buf[8];
        uint16_t lat_h = ((uint16_t)rx_buf[9] << 8) | rx_buf[10];

        uint32_t lon_32 = ((uint32_t)lon_h << LONLAT_SHIFT_BITS) | lon_l;
        uint32_t lat_32 = ((uint32_t)lat_h << LONLAT_SHIFT_BITS) | lat_l;

        uint32_t lon_deg = lon_32 / GNSS_LONLAT_DIVISOR;
        float lon_min = (float)(lon_32 % GNSS_LONLAT_DIVISOR) / LONLAT_FRAC_DIVISOR;
        uint32_t lat_deg = lat_32 / GNSS_LONLAT_DIVISOR;
        float lat_min = (float)(lat_32 % GNSS_LONLAT_DIVISOR) / LONLAT_FRAC_DIVISOR;

        sensor_data.longitude = (float)lon_deg + lon_min / 60.0f;
        sensor_data.latitude  = (float)lat_deg + lat_min / 60.0f;
    }

    /* 解析 GPS 数据(海拔、航向、地速) */
    {
        uint16_t height_raw = ((uint16_t)rx_buf[11] << 8) | rx_buf[12];
        uint16_t yaw_raw    = ((uint16_t)rx_buf[13] << 8) | rx_buf[14];
        uint16_t gps_vl     = ((uint16_t)rx_buf[15] << 8) | rx_buf[16];
        uint16_t gps_vh     = ((uint16_t)rx_buf[17] << 8) | rx_buf[18];

        uint32_t speed_32 = ((uint32_t)gps_vh << LONLAT_SHIFT_BITS) | gps_vl;

        sensor_data.height = (float)height_raw / GNSS_HEIGHT_DIVISOR;
        sensor_data.yaw    = (float)yaw_raw / GNSS_YAW_DIVISOR;
        sensor_data.speed  = (float)speed_32 / GNSS_SPEED_DIVISOR;
    }

    return GNSS_SENSOR_OK;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化 GNSS 传感器模块
 * @retval 无
 */
void GNSSSensor_Init(void)
{
    sensor_data.longitude = 0.0f;
    sensor_data.latitude  = 0.0f;
    sensor_data.height    = 0.0f;
    sensor_data.yaw       = 0.0f;
    sensor_data.speed     = 0.0f;

    s_last_success_tick = 0;
    s_frame_err_count   = 0;
}

/**
 * @brief  更新 GNSS 数据(单次读取全部寄存器 0x49~0x50)
 * @retval GNSSSensor_Status_t 操作状态
 * @note   增强逻辑:
 *         - 成功采集:记录时间、清零帧头计数,返回 OK
 *         - 帧头失败:沿用上次值并计数,连续 >FRAME_ERR_MAX 次返回 ERR_FRAME;
 *           调用方用 GNSSSensor_GetFrameErrCount() 判断 ≤3 次为可恢复
 *         - 距上次成功采集 >FRESH_TIMEOUT_MS:返回 TIMEOUT
 *         - 其余失败保留具体错误码返回,数据保持上次有效值
 */
GNSSSensor_Status_t GNSSSensor_Update(void)
{
    GNSSSensor_Status_t status = SendAndReceive(tx_all);

    if (status == GNSS_SENSOR_OK)
    {
        s_last_success_tick = HAL_GetTick();
        s_frame_err_count   = 0;
        PrintDataTable(&sensor_data);
        return GNSS_SENSOR_OK;
    }

    /* 帧头校验失败:沿用上次采集的值并计数,连续超过上限才作为硬错误返回 */
    if (status == GNSS_SENSOR_ERR_FRAME)
    {
        s_frame_err_count++;
        if (s_frame_err_count > GNSS_SENSOR_FRAME_ERR_MAX)
        {
            GNSS_LOG_PRINT("Frame header error (count=%d)\r\n", s_frame_err_count);
            return GNSS_SENSOR_ERR_FRAME;
        }
        /* ≤3 次:沿用上次值,返回具体错误码(调用方据计数判断软/硬) */
    }

    /* 新鲜度监测:距上次成功采集超过 500ms → 判 TIMEOUT */
    if ((HAL_GetTick() - s_last_success_tick) > GNSS_SENSOR_FRESH_TIMEOUT_MS)
    {
        return GNSS_SENSOR_TIMEOUT;
    }

    /* 未超时:保留具体错误码返回 */
    return status;
}

/**
 * @brief  获取 GNSS 数据
 * @return GNSSSensor_Data_t GNSS 数据(失败时保持上次有效值)
 */
GNSSSensor_Data_t GNSSSensor_GetData(void)
{
    return sensor_data;
}

/**
 * @brief  获取连续帧头错误计数
 * @return uint8_t 当前连续帧头失败次数(成功采集后清零)
 */
uint8_t GNSSSensor_GetFrameErrCount(void)
{
    return s_frame_err_count;
}
