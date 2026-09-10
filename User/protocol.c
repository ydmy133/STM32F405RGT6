/**
 ******************************************************************************
 * @file    protocol.c
 * @brief   无人船通信模块实现文件
 *
 * 实现 PC 端与 STM32 端之间的 JSON 格式通信协议。
 * 增加堆内存 - 在 startup_stm32f405xx.s 中将 Heap_Size 从 0x200 (512字节) 改为 0x1000 (4KB)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "cJSON.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "led.h"

/* Private defines -----------------------------------------------------------*/

/** @defgroup Protocol_Private_Defines 私有宏定义
 * @{
 */

/** 通讯模块日志打印宏，根据开关决定是否编译 */
#if PROTOCOL_PRINT_EN
#define PROTOCOL_PRINT(fmt, ...)    printf("[PROTOCOL] " fmt, ##__VA_ARGS__)
#else
#define PROTOCOL_PRINT(fmt, ...)
#endif

/** @} */

/* Private variables ---------------------------------------------------------*/

/** @defgroup Protocol_Private_Variables 私有变量
 * @{
 */

/** UART 发送缓冲区 */
static uint8_t s_tx_buffer[PROTOCOL_TX_BUFFER_SIZE];

/** UART 接收缓冲区 */
static uint8_t s_rx_buffer[PROTOCOL_RX_BUFFER_SIZE];

/** JSON 数据缓冲区（ISR/主循环共享，避免 ISR 中 malloc） */
static uint8_t s_json_buffer[PROTOCOL_JSON_MAX_SIZE];

/** 解析后的 PC 端指令数据（仅主循环访问） */
static PCCommand_t s_parsed_command;

/** 是否有新数据标志（ISR/主循环共享） */
static volatile uint8_t s_has_new_data = 0;

/** ISR 中设置的待解析标志 */
static volatile uint8_t s_rx_pending = 0;

/** 最后一次接收时间戳（ms） */
static volatile uint32_t s_last_receive_time = 0;

/** 是否曾成功接收到过数据（0=从未收到，1=曾收到） */
static volatile uint8_t s_has_ever_received = 0;

/** 控制模式字符串映射表 */
static const char *MODE_STRINGS[] = {
    "manual",
    "navigate",
    "stable_anchor",
    "cruise_speed",
    "cruise_dir",
    "fixed_point"
};

/** 控制模式字符串映射表长度 */
static const size_t MODE_STRING_COUNT = sizeof(MODE_STRINGS) / sizeof(MODE_STRINGS[0]);

/** @} */

/* Private functions ---------------------------------------------------------*/

/** @defgroup Protocol_Private_Functions 私有函数
 * @{
 */

/**
 * @brief  将整数值限制在指定范围内
 * @param  value: 待限制的整数值
 * @param  min: 最小值
 * @param  max: 最大值
 * @retval int8_t 限制后的值
 */
static int8_t Protocol_ClampInt(int value, int min, int max)
{
    if (value < min) return (int8_t)min;
    if (value > max) return (int8_t)max;
    return (int8_t)value;
}

/**
 * @brief  将浮点值限制在指定范围内
 * @param  value: 待限制的浮点值
 * @param  min: 最小值
 * @param  max: 最大值
 * @retval float 限制后的值
 */
static float Protocol_ClampFloat(double value, double min, double max)
{
    if (value < min) return (float)min;
    if (value > max) return (float)max;
    return (float)value;
}

/**
 * @brief  将控制模式字符串转换为枚举值
 * @param  mode_str: 模式字符串（如"manual"、"navigate"等）
 * @retval ControlMode_t 对应的枚举值
 */
static ControlMode_t Protocol_StringToMode(const char *mode_str)
{
    /* 参数校验：检查指针是否为空 */
    if (mode_str == NULL) {
        return CONTROL_MODE_INVALID;
    }

    /* 遍历模式字符串映射表，查找匹配的模式 */
    for (size_t i = 0; i < MODE_STRING_COUNT; i++) {
        if (strcmp(mode_str, MODE_STRINGS[i]) == 0) {
            return (ControlMode_t)i;
        }
    }

    return CONTROL_MODE_INVALID;
}

/**
 * @brief  从 JSON 对象中解析 int8 类型字段
 * @param  root: JSON 根对象
 * @param  key: 字段键名
 * @param  min: 最小值
 * @param  max: 最大值
 * @param  found: 输出参数，字段是否存在
 * @retval int8_t 解析并限制后的值
 */
static int8_t Protocol_ParseInt8Field(cJSON *root, const char *key, int min, int max, bool *found)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(item)) {
        *found = true;
        return Protocol_ClampInt((int)cJSON_GetNumberValue(item), min, max);
    }
    *found = false;
    return 0;
}

/**
 * @brief  从 JSON 对象中解析 float 类型字段
 * @param  root: JSON 根对象
 * @param  key: 字段键名
 * @param  min: 最小值
 * @param  max: 最大值
 * @param  found: 输出参数，字段是否存在
 * @retval float 解析并限制后的值
 */
static float Protocol_ParseFloatField(cJSON *root, const char *key, double min, double max, bool *found)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(item)) {
        *found = true;
        return Protocol_ClampFloat(cJSON_GetNumberValue(item), min, max);
    }
    *found = false;
    return 0.0f;
}

/**
 * @brief  解析 PC 端下发的 JSON 指令字符串
 * @param  json_str: 指向 JSON 字符串的指针
 * @param  cmd: 输出指令数据结构指针
 * @retval uint8_t 1=解析成功，0=解析失败
 */
static uint8_t Protocol_ParseCommandJSON(const char *json_str, PCCommand_t *cmd)
{
    /* 参数校验：检查指针是否有效 */
    if (json_str == NULL || cmd == NULL) {
        PROTOCOL_PRINT("Invalid parameters for parsing command\r\n");
        return 0;
    }

    /* 解析 JSON 字符串，生成 JSON 对象 */
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            PROTOCOL_PRINT("JSON parse error before: %s\r\n", error_ptr);
        } else {
            PROTOCOL_PRINT("JSON parse failed, unknown error\r\n");
        }
        return 0;
    }

    /* 解析 control_mode 字段，字符串类型 */
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "control_mode");
    if (cJSON_IsString(item) && (item->valuestring != NULL)) {
        cmd->control_mode = Protocol_StringToMode(item->valuestring);
        if (cmd->control_mode == CONTROL_MODE_INVALID) {
            cJSON_Delete(root);
            PROTOCOL_PRINT("Invalid control_mode: %s\r\n", item->valuestring);
            return 0;
        }
    } else {
        /* 如果未提供 control_mode，默认使用手动模式 */
        cmd->control_mode = CONTROL_MODE_MANUAL;
    }

    /* 解析各数值字段，并进行范围限制，任一必需字段缺失则解析失败 */
    bool field_found = false;

    cmd->move_speed = Protocol_ParseInt8Field(root, "move_speed", -100, 100, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: move_speed\r\n");
        return 0;
    }

    field_found = false;
    cmd->steer_speed = Protocol_ParseInt8Field(root, "steer_speed", -100, 100, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: steer_speed\r\n");
        return 0;
    }

    field_found = false;
    cmd->target_lat = Protocol_ParseFloatField(root, "target_lat", -90.0, 90.0, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: target_lat\r\n");
        return 0;
    }

    field_found = false;
    cmd->target_lon = Protocol_ParseFloatField(root, "target_lon", -180.0, 180.0, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: target_lon\r\n");
        return 0;
    }

    field_found = false;
    cmd->return_lat = Protocol_ParseFloatField(root, "return_lat", -90.0, 90.0, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: return_lat\r\n");
        return 0;
    }

    field_found = false;
    cmd->return_lon = Protocol_ParseFloatField(root, "return_lon", -180.0, 180.0, &field_found);
    if (!field_found) {
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: return_lon\r\n");
        return 0;
    }

    field_found = false;
    cmd->dc_prot = Protocol_ParseInt8Field(root, "dc_prot", 0, 1, &field_found);
    if (!field_found) {
        cmd->dc_prot = 1;
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: dc_prot\r\n");
        return 0;
    }

    field_found = false;
    cmd->lb_prot = Protocol_ParseInt8Field(root, "lb_prot", 0, 1, &field_found);
    if (!field_found) {
        cmd->lb_prot = 1;
        cJSON_Delete(root);
        PROTOCOL_PRINT("Missing required field: lb_prot\r\n");
        return 0;
    }

    cJSON_Delete(root);
    return 1;
}

/**
 * @brief  以表格形式打印解析后的 PC 指令数据（调试用）
 * @param  cmd: 指向指令数据结构的指针
 * @retval 无
 */
static void Protocol_PrintCommandTable(const PCCommand_t *cmd)
{
#if PROTOCOL_DEBUG_TABLE_EN
    if (cmd == NULL) {
        return;
    }

    const char *mode_str = "UNKNOWN";
    if (cmd->control_mode < MODE_STRING_COUNT) {
        mode_str = MODE_STRINGS[cmd->control_mode];
    }

    printf("\r\n");
    printf("+----------------+------------------+\r\n");
    printf("|   Field        |      Value       |\r\n");
    printf("+----------------+------------------+\r\n");
    printf("| control_mode   | %-16s |\r\n", mode_str);
    printf("| move_speed     | %-16d |\r\n", cmd->move_speed);
    printf("| steer_speed    | %-16d |\r\n", cmd->steer_speed);
    printf("| target_lat     | %-16.6f |\r\n", cmd->target_lat);
    printf("| target_lon     | %-16.6f |\r\n", cmd->target_lon);
    printf("| return_lat     | %-16.6f |\r\n", cmd->return_lat);
    printf("| return_lon     | %-16.6f |\r\n", cmd->return_lon);
    printf("| dc_prot        | %-16d |\r\n", cmd->dc_prot);
    printf("| lb_prot        | %-16d |\r\n", cmd->lb_prot);
    printf("+----------------+------------------+\r\n");
    printf("\r\n");
#else
    (void)cmd;
#endif
}

/**
 * @brief  主循环中调用，处理待解析的 UART 接收数据
 * @note   将 JSON 解析从 ISR 移至主循环，避免在中断中使用 malloc
 * @retval 无
 */
static void Protocol_ParsePendingData(void)
{
    if (!s_rx_pending) {
        return;
    }
    s_rx_pending = 0;

    if (s_json_buffer[0] == '\0') {
        return;
    }

    size_t len = strlen((const char *)s_json_buffer);
    if (len < 2 || s_json_buffer[0] != '{' || s_json_buffer[len - 1] != '}') {
        PROTOCOL_PRINT("Invalid JSON format, len: %d, first char: %c, last char: %c\r\n", len, s_json_buffer[0], s_json_buffer[len - 1]);
        PROTOCOL_PRINT("Raw data: %s\r\n", s_json_buffer);
        return;
    }

    if (Protocol_ParseCommandJSON((const char *)s_json_buffer, &s_parsed_command)) {
        s_has_new_data = 1;
        s_has_ever_received = 1;
        s_last_receive_time = HAL_GetTick();
        Protocol_PrintCommandTable(&s_parsed_command);
    } else {
        PROTOCOL_PRINT("Command parse failed, raw data: %s\r\n", s_json_buffer);
    }
}

/**
 * @brief  启动通信串口中断接收
 * @retval 无
 */
static void Protocol_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&PROTOCOL_UART_HANDLE, s_rx_buffer, PROTOCOL_RX_BUFFER_SIZE);
}

/**
 * @brief  打包 STM32 遥测数据为 JSON 字符串
 * @param  telemetry: 指向遥测数据结构的指针
 * @param  buffer: 输出缓冲区指针
 * @param  buffer_size: 输出缓冲区大小
 * @retval 无
 */
static void Protocol_PackTelemetryJSON(const STM32Telemetry_t *telemetry, char *buffer, uint16_t buffer_size)
{
    if (telemetry == NULL || buffer == NULL || buffer_size == 0) {
        return;
    }

    int ret = snprintf(buffer, buffer_size,
        "{\"data_valid\":%d,\"nav_reached\":%d,\"heading\":%.2f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"battery_level\":%.2f,"
        "\"dev_lat\":%f,\"dev_lon\":%f,\"speed\":%.2f,"
        "\"altitude\":%.2f,\"gyro_z\":%.2f,"
        "\"move_speed\":%d,\"steer_speed\":%d,"
        "\"target_lat\":%f,\"target_lon\":%f,"
        "\"dist_error\":%.2f,\"heading_error\":%.2f}",
        (int)telemetry->data_valid,
        (int)telemetry->nav_reached,
        (double)telemetry->heading,
        (double)telemetry->roll,
        (double)telemetry->pitch,
        (double)telemetry->battery_level,
        (double)telemetry->dev_lat,
        (double)telemetry->dev_lon,
        (double)telemetry->speed,
        (double)telemetry->altitude,
        (double)telemetry->gyro_z,
        (int)telemetry->move_speed,
        (int)telemetry->steer_speed,
        (double)telemetry->target_lat,
        (double)telemetry->target_lon,
        (double)telemetry->dist_error,
        (double)telemetry->heading_error);

    if (ret < 0 || (uint16_t)ret >= buffer_size) {
        buffer[0] = '\0';
    }
}

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup Protocol_Exported_Functions 导出函数实现
 * @{
 */

/**
 * @brief  初始化通信模块
 * @retval 无
 */
void Protocol_Init(void)
{
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    memset(s_json_buffer, 0, sizeof(s_json_buffer));
    memset((void *)&s_parsed_command, 0, sizeof(PCCommand_t));

    s_parsed_command.control_mode = CONTROL_MODE_MANUAL;
    s_parsed_command.dc_prot = 1;
    s_parsed_command.lb_prot = 1;

    s_has_new_data = 0;
    s_rx_pending = 0;
    s_last_receive_time = 0;
    s_has_ever_received = 0;

    Protocol_StartReceive();
}

/**
 * @brief  UART 空闲中断接收回调函数（ISR 上下文）
 * @param  Size: 接收到的数据长度
 * @note   仅做数据拷贝和标志位设置，JSON 解析在主循环中执行
 * @retval 无
 */
void Protocol_UART_RxEventCallback(uint16_t Size)
{
    if (Size > 0 && Size <= PROTOCOL_RX_BUFFER_SIZE) {
        uint16_t copy_size = (Size < PROTOCOL_JSON_MAX_SIZE - 1) ? Size : (PROTOCOL_JSON_MAX_SIZE - 1);
        memcpy(s_json_buffer, s_rx_buffer, copy_size);
        s_json_buffer[copy_size] = '\0';
        s_rx_pending = 1;
    }

    HAL_UARTEx_ReceiveToIdle_IT(&PROTOCOL_UART_HANDLE, s_rx_buffer, PROTOCOL_RX_BUFFER_SIZE);
}

/**
 * @brief  通过串口发送 STM32 遥测数据
 * @param  telemetry: 指向遥测数据结构的指针
 * @retval 无
 */
void Protocol_SendTelemetry(const STM32Telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    s_tx_buffer[0] = '\0';
    Protocol_PackTelemetryJSON(telemetry, (char *)s_tx_buffer, sizeof(s_tx_buffer));

    uint16_t len = (uint16_t)strlen((const char *)s_tx_buffer);
    if (len == 0) {
        PROTOCOL_PRINT("Send telemetry failed: empty JSON\r\n");
        return;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit(&PROTOCOL_UART_HANDLE, (uint8_t *)s_tx_buffer, len, 100);
    if (ret != HAL_OK) {
        PROTOCOL_PRINT("UART transmit failed, ret=%d\r\n", ret);
        return;
    }

    const char *crlf = "\r\n";
    HAL_UART_Transmit(&PROTOCOL_UART_HANDLE, (uint8_t *)crlf, 2, 100);

    /* 每次发送遥测数据时切换LED状态 */
    LED_Toggle();
}

/**
 * @brief  获取最新接收到的 PC 端指令
 * @param  cmd: 输出指令数据结构指针
 * @retval uint8_t 1=有新数据，0=无新数据
 * @note   此函数在主循环中调用，会自动处理待解析数据
 */
uint8_t Protocol_GetLatestCommand(PCCommand_t *cmd)
{
    if (cmd == NULL) {
        return 0;
    }

    Protocol_ParsePendingData();

    uint8_t result = 0;

    if (s_has_new_data) {
        __disable_irq();
        memcpy(cmd, (const void *)&s_parsed_command, sizeof(PCCommand_t));
        s_has_new_data = 0;
        __enable_irq();
        result = 1;
    }

    return result;
}

/**
 * @brief  检查是否断连超时
 * @retval uint8_t 1=已超时，0=未超时
 */
uint8_t Protocol_IsTimeout(void)
{
    if (!s_has_ever_received) {
        return 0;
    }
    if ((HAL_GetTick() - s_last_receive_time) > PROTOCOL_DISCONNECT_TIMEOUT_MS) {
        return 1;
    }
    return 0;
}

/** @} */
