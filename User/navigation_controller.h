/**
 ******************************************************************************
 * @file    navigation_controller.h
 * @brief   导航控制器模块头文件(增量式串级 PI)
 *
 * 相对参考工程(STM32F405RGT6)的控制升级:
 *   - 转向控制改为串级增量式 PI:
 *       外环 航向增量式 PI -> 目标转向量
 *       内环 转向量增量式 PI -> 差速舵令(反馈为当前偏航角速度 gz)
 *   - 行进控制改为增量式 PI:沿船首向的目标点距离误差 -> 行进速度
 *       设定点是目标点本身,不是 2m 圈缘;限幅[0,max],禁止倒车
 *   - 2m 到达圈只做模式切换,不是位置环设定点:
 *       ≤2m: 清转向积分,锁定目标船首向(优先上一帧目标船首向,无则保持当前船首向),继续稳泊输出
 *       >2m: 导航回目标点(目标船首向=目标方位角)
 *   - 原「到达停机」「航向误差>32°停速原地转向」已停用
 ******************************************************************************
 */

#ifndef __NAVIGATION_CONTROLLER_H
#define __NAVIGATION_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/** 日志打印开关 (1=开启, 0=关闭) */
#define NAV_CONTROLLER_LOG_PRINT_EN         1

/** 数学常量 */
#define NAV_PI                              3.14159265358979323846
#define NAV_EARTH_RADIUS                    6371000.0

/** 最大行进/转向速度默认值 (0-100) */
#define NAV_DEFAULT_MAX_SPEED               80

/** 到达圈半径(米):只用于圈内锁船首向/圈外导航回目标;位置环设定点始终是目标点,不是该圈缘 */
#define NAV_ARRIVAL_THRESHOLD               2.0f

/** 重新接近时航向误差阈值(度):超过则停速原地转向(该逻辑已停用,宏保留备查) */
#define NAV_ANGLE_ERROR_MAX                 32.0f

/** 经纬度有效范围 */
#define NAV_LAT_MIN                         -90.0
#define NAV_LAT_MAX                         90.0
#define NAV_LON_MIN                         -180.0
#define NAV_LON_MAX                         180.0

/** @name 串级增量式 PI 增益(初值,需上船整定)
 * @{
 */
#define NAV_HEADING_KP                      0.5     /**< 转向外环比例:航向误差(°)->目标转向量 */
#define NAV_HEADING_KI                      0.01    /**< 转向外环积分 */
#define NAV_TURN_RATE_KP                    1.0     /**< 转向内环比例:转向量误差->差速舵令 */
#define NAV_TURN_RATE_KI                    0.10    /**< 转向内环积分 */
#define NAV_POSITION_KP                     1.0     /**< 行进环比例:沿船首向目标点误差(m)->行进速度 */
#define NAV_POSITION_KI                     0.10   /**< 行进环积分(稳态可留下顶风推力) */
#define NAV_MAX_TURN_RATE                   80.0   /**< 期望最大转向速度，目标转向量限幅(与舵令同量纲) */
#define NAV_STEER_CMD_TO_DEG_S              1.0     /**< 角速度(°/s)换算到转向量量纲的系数 */
/** @} */

/* Exported types ------------------------------------------------------------*/

/** 地理坐标结构体 */
typedef struct {
    double latitude;              /**< 纬度(WGS84) */
    double longitude;             /**< 经度(WGS84) */
} GeoCoordinate_t;

/** 导航控制输出结构体 */
typedef struct {
    int8_t speed;                 /**< 行进速度,范围[-100,100],正值前进 */
    int8_t steering;              /**< 转向速度,范围[-100,100],正值左转 */
} NavControlOutput_t;

/** 导航状态信息结构体 */
typedef struct {
    double distance;              /**< 到目标点的距离(米) */
    double heading_error;         /**< 航向误差(度,±180) */
    uint8_t arrived;              /**< 导航到达标志 */
} NavStatus_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化导航控制器模块
 * @note   在系统启动阶段调用一次
 */
void NavigationController_Init(void);

/**
 * @brief  设置导航最大速度
 * @param  max_speed: 最大速度值,超过 NAV_DEFAULT_MAX_SPEED 时自动限制
 */
void NavigationController_SetMaxSpeed(int8_t max_speed);

/**
 * @brief  设置导航目标坐标
 * @param  target_lat: 目标纬度(WGS84)
 * @param  target_lon: 目标经度(WGS84)
 */
void NavigationController_SetTarget(double target_lat, double target_lon);

/**
 * @brief  更新导航控制器数据(周期调用)
 * @param  current_lat:      当前纬度(WGS84)
 * @param  current_lon:      当前经度(WGS84)
 * @param  current_heading:  当前航向角(度,0~360)
 * @param  current_yaw_rate: 当前偏航角速度(度/秒,即航向角速度 gz)
 */
void NavigationController_Update(double current_lat, double current_lon,
                                 double current_heading, double current_yaw_rate);

/**
 * @brief  获取导航控制输出
 * @retval NavControlOutput_t 导航控制输出结构体
 */
NavControlOutput_t NavigationController_GetControl(void);

/**
 * @brief  获取导航状态信息
 * @retval NavStatus_t 导航状态信息结构体
 */
NavStatus_t NavigationController_GetStatus(void);

/**
 * @brief  获取当前导航目标坐标
 * @retval GeoCoordinate_t 当前目标坐标结构体
 */
GeoCoordinate_t NavigationController_GetTarget(void);

/**
 * @brief  根据起点、航向和距离计算目标位置
 * @param  start_lat: 起点纬度(度)
 * @param  start_lon: 起点经度(度)
 * @param  heading:   航向角(度,0~360,正北为0)
 * @param  distance:  距离(米)
 * @param  out_lat:   输出参数,目标纬度(度)
 * @param  out_lon:   输出参数,目标经度(度)
 */
void NavigationController_CalculateTargetPosition(double start_lat, double start_lon,
                                                  double heading, double distance,
                                                  double *out_lat, double *out_lon);

#ifdef __cplusplus
}
#endif

#endif /* __NAVIGATION_CONTROLLER_H */
