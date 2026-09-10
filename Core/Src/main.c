/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "led.h"          /**< LED驱动模块 */
#include "protocol.h"     /**< 无人船通信模块 */
#include "gnss_sensor.h"  /**< GNSS传感器驱动模块 */
#include "gnss_pos_filter.h" /**< GNSS经纬度三层滤波 */
#include "heading_sensor.h" /**< 航向角传感器驱动模块 */
#include "navigation_controller.h" /**< 导航控制器模块 */
#include "bms_sensor.h"   /**< BMS电池保护板Modbus驱动模块 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_SEND_INTERVAL_MS  1000     /**< 遥测数据发送间隔（毫秒） */

#define DRIVER_SEND_INTERVAL_MS   100        /**< 驱动板数据发送间隔（毫秒） */

#define NAV_UPDATE_INTERVAL_MS    100        /**< 导航控制更新间隔（毫秒） */
#define NAV_DBG_PRINT_INTERVAL_MS 3200       /**< 导航调试打印间隔（毫秒） */

#define DIR_CRUISE_STEP_DISTANCE    32.0f    /**< 定向巡航目标位置平移步长（米） */

#define BATTERY_LOW_VOLTAGE       10.0f      /**< 低电量返航阈值（RSOC百分比） */

/** @name 断连动作选择宏
 *  @{
 */
#define DISCONNECT_ACTION_STOP_MOTOR    1   /**< 断连后停止电机 */
#define DISCONNECT_ACTION_STABLE_ANCHOR 2   /**< 断连后进入稳泊模式 */
#define DISCONNECT_ACTION    DISCONNECT_ACTION_STOP_MOTOR   /**< 断连动作选择，默认停止电机 */
/** @} */

/** 传感器连续异常上限:任一超过则强制停电机并切回手动(移植自 control_borad) */
#define SENSOR_ANOMALY_LIMIT   3

#if 1                                        /**< 主模块调试日志打印开关 (1=启用, 0=禁用) */
#define MAIN_DBG(fmt, ...)    printf("[MAIN] " fmt, ##__VA_ARGS__)
#else
#define MAIN_DBG(fmt, ...)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/** @name 系统全局变量
 *  @{
 */
/* 控制速度 */
static int8_t  s_move_speed;                                /**< 行进速度，范围[-100, 100]，正前负后 */
static int8_t  s_steer_speed;                               /**< 转向速度，范围[-100, 100]，正左负右 */
static float   s_battery_level = 0.0f;                      /**< 电池电压（伏特） */

/* 定向巡航 */
static float  s_dir_cruise_base_heading = 0.0f;             /**< 定向巡航基准航向（度） */
static double s_dir_cruise_target_lat = 0.0;                /**< 定向巡航当前目标纬度 */
static double s_dir_cruise_target_lon = 0.0;                /**< 定向巡航当前目标经度 */

/* 自动返航标志 */
static uint8_t s_return_home_active = 0;                    /**< 断连返航激活标志，1=已激活 */
static uint8_t s_low_battery_return_active = 0;             /**< 低电量返航激活标志，1=已激活 */
static uint8_t s_disconnect_stable_active = 0;              /**< 断连稳泊激活标志，1=已激活 */

/* 导航状态信息 */
NavStatus_t s_nav_status;                                   /**< 导航状态信息结构体 */

/* 通信协议结构体 */
static STM32Telemetry_t s_telemetry;                        /**< STM32端上报遥测数据结构体 */
static PCCommand_t      s_command;                          /**< PC端下发指令数据结构体 */

/* 传感器数据 */
static GNSSSensor_Data_t    s_gnss;                         /**< GNSS传感器数据 */
static HeadingSensor_Data_t s_heading;                      /**< 航向传感器数据 */

static const char *mode_names[] = {"MANUAL", "NAVIGATE", "STABLE_ANCHOR", "CRUISE_SPEED", "CRUISE_DIR", "FIXED_POINT", "INVALID"};
#define MODE_NAMES_COUNT  (sizeof(mode_names) / sizeof(mode_names[0]))
/** @} */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/** @name 私有函数声明
 *  @{
 */
static void System_UpdateTelemetry(void);   /**< 更新并发送遥测数据 */
static void System_ProcessCommand(void);    /**< 处理PC端指令 */
static void System_UpdateSensors(void);     /**< 轮流更新传感器数据 */
static void System_UpdateNavigation(void);  /**< 更新导航控制器 */
static void System_SetSpeed(int8_t move, int8_t steer); /**< 设置行进速度和转向速度 */
static void System_SendDriverCommand(int8_t move_speed, int8_t turn_speed); /**< 发送控制指令到驱动板 */
static void System_CheckAutoReturn(void); /**< 检查并执行自动返航逻辑 */
static uint8_t System_NavActive(void);      /**< 当前是否在用 GNSS 做导航闭环 */
static void System_GetNavLatLon(double *lat, double *lon); /**< 速度环用的滤波后经纬度 */
/** @} */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_CAN1_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化用户模块 */
  LED_Init();                   /* LED初始化 */
  LED_Blink(100, 10);           /* LED闪烁10次，间隔100ms，表示系统启动 */
  GNSSSensor_Init();            /* GNSS传感器初始化 */
  GNSSPosFilter_Init();         /* 经纬度滤波初始化 */
  HeadingSensor_Init();         /* 航向角传感器初始化 */
  NavigationController_Init();  /* 导航控制器初始化 */
  BMSSensor_Init();             /* BMS电池保护板传感器初始化 */
  Protocol_Init();              /* 通信模块初始化，启动UART中断接收 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 刷新看门狗，防止超时复位 */
    HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 轮流更新传感器数据，每50ms更新一个 */
    System_UpdateSensors();

    /* 更新并发送遥测数据，发送给上位机 */
    System_UpdateTelemetry();

    /* 处理PC端下发的指令 */
    System_ProcessCommand();

    /* 检查断连保护和低电量保护 */
    System_CheckAutoReturn();
    
    /* 更新导航控制器 */
    System_UpdateNavigation();

    /* 发送控制指令到驱动板 */
    System_SendDriverCommand(s_move_speed, s_steer_speed);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  设置行进速度和转向速度
 * @param  move: 行进速度，范围[-100, 100]，正前负后
 * @param  steer: 转向速度，范围[-100, 100]，正左负右
 * @retval None
 */
static void System_SetSpeed(int8_t move, int8_t steer)
{
    /* 速度变化时打印调试信息 */
    if (s_move_speed != move || s_steer_speed != steer) {
        // MAIN_DBG("Speed change: move=%d, steer=%d\r\n", move, steer);
    }
    s_move_speed = move;
    s_steer_speed = steer;
}

/**
 * @brief  发送控制指令到驱动板
 * @param  move_speed: 行进速度，范围[-100, 100]，正值前进，负值后退
 * @param  turn_speed: 转向速度，范围[-100, 100]，正值左转，负值右转
 * @retval None
 * @note   通过UART5发送，帧格式：帧头(0x55) + 预留(3) + 数据(4) + 帧尾(0xAA)
 */
static void System_SendDriverCommand(int8_t move_speed, int8_t turn_speed)
{
    static uint32_t last_send_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    /* 检查是否到达发送时间 */
    if (current_tick - last_send_tick < DRIVER_SEND_INTERVAL_MS) {
        return;
    }
    last_send_tick = current_tick;

    /* 组装发送帧 */
    uint8_t tx_buf[9];
    tx_buf[0] = 0x55;                   /* 帧头 */
    tx_buf[1] = 0;                      /* 预留字节1 */
    tx_buf[2] = 0;                      /* 预留字节2 */
    tx_buf[3] = 0;                      /* 预留字节3 */
    tx_buf[4] = (uint8_t)move_speed;    /* 行进速度 */
    tx_buf[5] = (uint8_t)turn_speed;    /* 转向速度 */
    tx_buf[6] = 0;                      /* 前灯亮度（暂不使用） */
    tx_buf[7] = 0;                      /* 后灯亮度（暂不使用） */
    tx_buf[8] = 0xAA;                   /* 帧尾 */

    /* 通过UART6发送数据 */
    HAL_UART_Transmit(&huart6, tx_buf, 9, 100);
}

/**
 * @brief  当前是否在用 GNSS 位置做导航闭环
 */
static uint8_t System_NavActive(void)
{
    if (s_return_home_active || s_low_battery_return_active || s_disconnect_stable_active) {
        return 1u;
    }
    if (s_command.control_mode == CONTROL_MODE_NAVIGATE ||
        s_command.control_mode == CONTROL_MODE_STABLE_ANCHOR ||
        s_command.control_mode == CONTROL_MODE_FIXED_POINT ||
        s_command.control_mode == CONTROL_MODE_CRUISE_DIR) {
        return 1u;
    }
    return 0u;
}

/**
 * @brief  速度环用的当前经纬度(滤波后;尚未就绪则回退原始)
 */
static void System_GetNavLatLon(double *lat, double *lon)
{
    float f_lat;
    float f_lon;

    if ((lat == 0) || (lon == 0)) {
        return;
    }
    if (GNSSPosFilter_Get(&f_lat, &f_lon) != 0u) {
        *lat = (double)f_lat;
        *lon = (double)f_lon;
        return;
    }
    *lat = (double)s_gnss.latitude;
    *lon = (double)s_gnss.longitude;
}

/**
 * @brief  轮流更新传感器数据，每50ms更新一个
 * @retval None
 * @note   在主循环中调用，交替更新GNSS传感器和航向角传感器
 *         移植自 control_borad:统计各传感器连续异常,任一超过
 *         SENSOR_ANOMALY_LIMIT 次则强制停电机并切回手动(等上位机重新下发恢复)
 */
static void System_UpdateSensors(void)
{
    static uint32_t last_update_tick = 0;
    static uint8_t sensor_index = 0;        /* 只在程序启动时初始化一次 */
    static uint8_t gnss_anomaly_count = 0;  /* GNSS 连续异常计数 */
    static uint8_t heading_anomaly_count = 0; /* 航向传感器连续异常计数 */
    static uint32_t gnss_ok_tick = 0;       /* 上次 GNSS 成功采样时刻 */
    uint32_t current_tick = HAL_GetTick();

    /* 检查是否到达更新时间（50ms间隔） */
    if (current_tick - last_update_tick < 50) {
        return;
    }
    last_update_tick = current_tick;

    /* 轮流更新传感器 */
    switch (sensor_index) {
        case 0:
            /* 更新GNSS传感器数据,统计异常;成功则推进经纬度滤波 */
            {
                GNSSSensor_Status_t gnss_st = GNSSSensor_Update();
                float dt_s;
                float dist_m;

                if (gnss_st != GNSS_SENSOR_OK) {
                    if (gnss_anomaly_count < 255u) gnss_anomaly_count++;
                } else {
                    gnss_anomaly_count = 0;
                }
                s_gnss = GNSSSensor_GetData();
                if (gnss_st == GNSS_SENSOR_OK) {
                    dt_s = 0.1f;
                    if (gnss_ok_tick != 0u) {
                        dt_s = (float)(current_tick - gnss_ok_tick) * 0.001f;
                    }
                    gnss_ok_tick = current_tick;
                    dist_m = System_NavActive() ? (float)s_nav_status.distance : -1.0f;
                    GNSSPosFilter_Update(s_gnss.latitude, s_gnss.longitude,
                                         s_gnss.speed / 3.6f, dt_s, dist_m);
                }
            }
            break;
        case 1:
            /* 更新航向角传感器数据,统计异常 */
            if (HeadingSensor_Update() != HEADING_SENSOR_OK) {
                if (heading_anomaly_count < 255u) heading_anomaly_count++;
            } else {
                heading_anomaly_count = 0;
            }
            s_heading = HeadingSensor_GetData();
            break;
        default:
            break;
    }

    /* 切换传感器索引 */
    sensor_index = (sensor_index + 1) % 2;

    /* 连续数据异常超过上限:强制停电机并切回手动,数据恢复后需上位机重新下发指令 */
    if (gnss_anomaly_count > SENSOR_ANOMALY_LIMIT ||
        heading_anomaly_count > SENSOR_ANOMALY_LIMIT) {
        s_command.control_mode = CONTROL_MODE_MANUAL;
        s_return_home_active = 0;
        s_low_battery_return_active = 0;
        s_disconnect_stable_active = 0;
        System_SetSpeed(0, 0);
        MAIN_DBG("Sensor anomaly limit reached, forced manual stop\r\n");
    }
}

/**
 * @brief  处理PC端下发的指令
 * @retval None
 * @note   在主循环中调用，根据控制模式执行相应操作，并检查断连超时
 */
static void System_ProcessCommand(void)
{
    /* 获取最新指令并处理 */
    if (Protocol_GetLatestCommand(&s_command)) {
        /* 通信恢复，取消断连返航/稳泊状态 */
        if (s_return_home_active) {
            s_return_home_active = 0;
        }
        if (s_disconnect_stable_active) {
            s_disconnect_stable_active = 0;
            MAIN_DBG("Disconnect stable anchor exited, communication restored\r\n");
        }

        /* 低电量保护期间：屏蔽PC端控制，仅检查退出指令 */
        if (s_low_battery_return_active) {
            /* 若接收到 move_speed == 20，则主动退出低电量保护，恢复PC端控制 */
            if (s_command.move_speed == 20) {
                s_low_battery_return_active = 0;
                MAIN_DBG("Low battery protection exited via move_speed=20\r\n");
            }
            return;
        }
        
        /* 记录上一次的控制模式，用于判断是否需要切换 */
        static ControlMode_t s_last_mode = CONTROL_MODE_INVALID;

        /* 模式切换时打印当前模式 */
        if (s_last_mode != s_command.control_mode) {
            if (s_command.control_mode < MODE_NAMES_COUNT) {
                MAIN_DBG("Mode switch: %s\r\n", mode_names[s_command.control_mode]);
            }
        }

        /* 根据控制模式执行相应操作 */
        switch (s_command.control_mode) {
            case CONTROL_MODE_MANUAL:
            {
                /* 手动模式：提取行进速度和转向速度 */
                System_SetSpeed(s_command.move_speed, s_command.steer_speed);
                break;
            }
            case CONTROL_MODE_NAVIGATE:
            case CONTROL_MODE_FIXED_POINT:
            {
                /* 导航模式或定点抛锚模式：使用 move_speed 作为导航最大速度 */
                NavigationController_SetMaxSpeed(s_command.move_speed);

                /* 模式切换或目标坐标变化时设置导航目标 */
                static float s_last_target_lat = 0.0f;
                static float s_last_target_lon = 0.0f;
                if (s_last_mode != s_command.control_mode ||
                    s_command.target_lat != s_last_target_lat ||
                    s_command.target_lon != s_last_target_lon) {
                    s_nav_status.arrived = 0;
                    s_last_target_lat = s_command.target_lat;
                    s_last_target_lon = s_command.target_lon;
                    NavigationController_SetTarget(s_command.target_lat, s_command.target_lon);
                }
                break;
            }
            case CONTROL_MODE_STABLE_ANCHOR:
            {
                /* 稳泊模式：模式切换时，将当前坐标设置为导航目标 */
                if (s_last_mode != s_command.control_mode) {
                    double lock_lat;
                    double lock_lon;
                    System_GetNavLatLon(&lock_lat, &lock_lon);
                    NavigationController_SetTarget(lock_lat, lock_lon);
                }
                break;
            }
            case CONTROL_MODE_CRUISE_SPEED:
            {
                /* 定速巡航：提取行进速度，转向速度设置为0 */
                System_SetSpeed(s_command.move_speed, 0);
                break;
            }
            case CONTROL_MODE_CRUISE_DIR:
            {
                /* 定向巡航：模式切换时记录当前航向为基准，计算初始目标点 */
                if (s_last_mode != s_command.control_mode) {
                    double here_lat;
                    double here_lon;
                    s_dir_cruise_base_heading = s_heading.yaw;
                    System_GetNavLatLon(&here_lat, &here_lon);
                    NavigationController_CalculateTargetPosition(
                        here_lat, here_lon,
                        s_dir_cruise_base_heading,
                        DIR_CRUISE_STEP_DISTANCE,
                        &s_dir_cruise_target_lat, &s_dir_cruise_target_lon);
                    NavigationController_SetTarget(s_dir_cruise_target_lat, s_dir_cruise_target_lon);
                }
                break;
            }
            default:
                break;
        }

        /* 更新上一次的控制模式 */
        s_last_mode = s_command.control_mode;
    }
}

/**
 * @brief  检查并执行自动返航逻辑
 * @retval None
 * @note   检查断连超时和低电量状态，触发自动返航
 */
static void System_CheckAutoReturn(void)
{
    /* 断连保护：超时后导航前往返航点（受 dc_prot 控制） */
    if (s_command.dc_prot && Protocol_IsTimeout()) {
        if (!s_return_home_active && !s_low_battery_return_active) {
            s_return_home_active = 1;
            /* 将导航最大速度设为最大，确保尽快返航 */
            NavigationController_SetMaxSpeed(NAV_DEFAULT_MAX_SPEED);
            MAIN_DBG("Disconnect protection triggered, navigating to return point (%.6f, %.6f)\r\n",
                   (double)s_command.return_lat, (double)s_command.return_lon);
            NavigationController_SetTarget(s_command.return_lat, s_command.return_lon);
        }
    }

    /* 未开启断连保护时，断连后根据宏定义选择停止电机或稳泊 */
    if (!s_command.dc_prot && Protocol_IsTimeout()) {
        #if DISCONNECT_ACTION == DISCONNECT_ACTION_STABLE_ANCHOR
            if (!s_disconnect_stable_active && !s_return_home_active && !s_low_battery_return_active) {
                s_disconnect_stable_active = 1;
                NavigationController_SetMaxSpeed(NAV_DEFAULT_MAX_SPEED);
                {
                    double lock_lat;
                    double lock_lon;
                    System_GetNavLatLon(&lock_lat, &lock_lon);
                    NavigationController_SetTarget(lock_lat, lock_lon);
                }
                MAIN_DBG("Disconnect detected (no prot), entering stable anchor\r\n");
            }
        #else
            System_SetSpeed(0, 0);
            MAIN_DBG("Disconnect detected (no prot), stopping motors\r\n");
        #endif
    }

    /* 低电量保护：电压低于阈值后导航前往返航点（不受PC端控制，受 lb_prot 控制） */
    if (s_command.lb_prot && s_battery_level < BATTERY_LOW_VOLTAGE && s_battery_level != 0) {
        if (!s_low_battery_return_active && !s_return_home_active) {
            s_low_battery_return_active = 1;
            /* 将导航最大速度设为最大，确保尽快返航 */
            NavigationController_SetMaxSpeed(NAV_DEFAULT_MAX_SPEED);
            MAIN_DBG("Low battery protection triggered (%.2fV < %.2fV), navigating to return point (%.6f, %.6f)\r\n",
                   (double)s_battery_level, (double)BATTERY_LOW_VOLTAGE,
                   (double)s_command.return_lat, (double)s_command.return_lon);
            NavigationController_SetTarget(s_command.return_lat, s_command.return_lon);
        }
    }

    /* 低电量保护：到达返航点后退出保护，避免无法控制 */
    if (s_low_battery_return_active && s_nav_status.arrived) {
        s_low_battery_return_active = 0;
        MAIN_DBG("Low battery protection exited, arrived at return point\r\n");
    }
}

/**
 * @brief  更新导航控制器
 * @retval None
 * @note   在主循环中调用，仅在导航相关模式下执行导航控制计算
 */
static void System_UpdateNavigation(void)
{
    static uint32_t last_update_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    /* 控制导航更新频率 */
    if (current_tick - last_update_tick < NAV_UPDATE_INTERVAL_MS) {
        return;
    }
    last_update_tick = current_tick;

    /* 仅在导航相关模式、断连返航、低电量返航或断连稳泊时执行导航控制计算 */
    if (!System_NavActive()) {
        return;
    }

    /* 归一化航向角到 [0, 360) 范围 */
    float normalized_heading = s_heading.yaw;
    while (normalized_heading < 0.0f) normalized_heading += 360.0f;
    while (normalized_heading >= 360.0f) normalized_heading -= 360.0f;

    /* 串级PI:位置用滤波后经纬度,角速度(°/s)作转向内环反馈 */
    {
        double nav_lat;
        double nav_lon;
        System_GetNavLatLon(&nav_lat, &nav_lon);
        NavigationController_Update(nav_lat, nav_lon, normalized_heading, s_heading.gz);
    }

    /* 先更新导航状态信息，确保后续判断使用最新数据 */
    s_nav_status = NavigationController_GetStatus();

    /* 根据控制模式执行不同的导航更新逻辑 */
    /* 低电量保护期间：仅以返航点为目标，屏蔽PC端控制模式的影响 */
    if (!s_low_battery_return_active && s_command.control_mode == CONTROL_MODE_CRUISE_DIR) {
        /* 定向巡航模式：动态更新目标点。如果距离小于阈值，更新目标点 */
        if (s_nav_status.distance < DIR_CRUISE_STEP_DISTANCE * 0.5f) {
            NavigationController_CalculateTargetPosition(
                s_dir_cruise_target_lat, s_dir_cruise_target_lon,
                s_dir_cruise_base_heading,
                DIR_CRUISE_STEP_DISTANCE,
                &s_dir_cruise_target_lat, &s_dir_cruise_target_lon);
            NavigationController_SetTarget(s_dir_cruise_target_lat, s_dir_cruise_target_lon);
        }
    }

    /* 获取导航控制输出 */
    NavControlOutput_t nav_output = NavigationController_GetControl();

    /* 控制调试打印频率（与导航更新频率解耦） */
    {
        static uint32_t last_print_tick = 0;
        if (current_tick - last_print_tick >= NAV_DBG_PRINT_INTERVAL_MS) {
            last_print_tick = current_tick;
            MAIN_DBG("Nav: dist=%.2fm, heading_err=%.2f, arrived=%d\r\n",
                     (double)s_nav_status.distance, (double)s_nav_status.heading_error, s_nav_status.arrived);
        }
    }

    /* 设置行进速度和转向速度 */
    System_SetSpeed(nav_output.speed, nav_output.steering);
}

/**
 * @brief  更新遥测数据并按固定频率发送
 * @retval None
 * @note   在主循环中调用，内部使用定时器控制发送频率为1Hz（1000ms间隔）
 */
static void System_UpdateTelemetry(void)
{
    static uint32_t last_send_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    /* 检查是否到达发送时间 */
    if (current_tick - last_send_tick < TELEMETRY_SEND_INTERVAL_MS) {
        return;
    }
    last_send_tick = current_tick;

    /* 每1000ms同步更新一次BMS电池电量（RSOC百分比） */
//    BMSSensor_Update();

//    s_battery_level = (float)BMSSensor_GetData();

    s_telemetry.data_valid = 1;                         /* GPS数据有效标志，1=有效 */
    s_telemetry.nav_reached = s_nav_status.arrived;     /* 导航目标是否到达，0=未到达 */
    s_telemetry.heading = s_heading.yaw;                /* 航向角，范围[0, 359] */
    s_telemetry.roll = s_heading.roll;                  /* 翻滚角，范围[-180, 180] */
    s_telemetry.pitch = s_heading.pitch;                /* 俯仰角，范围[-90, 90] */
    s_telemetry.battery_level = s_battery_level;        /* 电池电量百分比(RSOC)，范围0~100 */
    s_telemetry.dev_lat = s_gnss.latitude;              /* 原始纬度（未滤波，便于对照） */
    s_telemetry.dev_lon = s_gnss.longitude;             /* 原始经度（未滤波，便于对照） */
    s_telemetry.speed = s_gnss.speed / 3.6f;            /* 对地速度，km/h转m/s */
    s_telemetry.altitude = s_gnss.height;               /* 海拔高度，单位: m */
    s_telemetry.gyro_z = s_heading.gz;                  /* 实测偏航角速度，单位: °/s */
    s_telemetry.move_speed = s_move_speed;              /* 当前行进速度指令 */
    s_telemetry.steer_speed = s_steer_speed;            /* 当前转向速度指令 */
    {                                                   /* 当前导航目标坐标(未设目标为0,0) */
        GeoCoordinate_t tgt = NavigationController_GetTarget();
        s_telemetry.target_lat = (float)tgt.latitude;
        s_telemetry.target_lon = (float)tgt.longitude;
    }
    s_telemetry.dist_error = (float)s_nav_status.distance;         /* 到目标距离，m */
    s_telemetry.heading_error = (float)s_nav_status.heading_error; /* 航向误差，° */

    /* 通过UART发送遥测数据JSON */
    Protocol_SendTelemetry(&s_telemetry);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
