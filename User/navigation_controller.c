/**
 ******************************************************************************
 * @file    navigation_controller.c
 * @brief   导航控制器模块实现文件(增量式串级 PI)
 *
 * 控制结构:
 *   转向:外环 航向增量式PI -> 目标转向量 s_target_turn
 *        内环 转向量增量式PI -> 差速舵令 s_steer_cmd(反馈为当前偏航角速度)
 *   行进:增量式PI:沿船首向的目标点距离误差 -> 行进速度 s_speed_cmd
 *     设定点是目标点,不是 2m 圈缘;限幅[0,max],禁止倒车
 *     越过目标或船首没对准时沿航向误差≤0,速度收到 0;出圈后转向再前进回来
 *   2m 到达圈只做模式切换:
 *     ≤2m: 清转向积分,锁定目标船首向后继续稳泊输出
 *     >2m: 导航回目标点
 *   原「到达停机」「航向误差>32°停速原地转向」已停用
 *
 * 增量式 PI 天然抗积分饱和(累加的是限幅后输出,误差反向即退出饱和),
 * 模式切换/换目标时清零累加器即可平滑起步。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "navigation_controller.h"
#include <stdio.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/** 日志打印宏,根据开关决定是否编译 */
#if NAV_CONTROLLER_LOG_PRINT_EN
#define NAV_LOG_PRINT(fmt, ...)    printf("[NAV CTRL] " fmt, ##__VA_ARGS__)
#else
#define NAV_LOG_PRINT(fmt, ...)
#endif

/* Private variables ---------------------------------------------------------*/

/** 目标坐标(WGS84) */
static GeoCoordinate_t s_target = {0.0, 0.0};

/** 当前坐标(WGS84) */
static GeoCoordinate_t s_current = {0.0, 0.0};

/** 控制输出(行进速度、转向速度) */
static NavControlOutput_t s_control_output = {0, 0};

/** 导航状态信息(距离、航向误差、到达标志) */
static NavStatus_t s_nav_status = {0.0, 0.0, 0};

/** 最大速度限制(行进速度和转向速度共用) */
static int8_t s_max_speed = NAV_DEFAULT_MAX_SPEED;

/** 串级增量式 PI 状态变量 */
static double s_heading_error_prev = 0.0;   /**< 转向外环:上一次航向误差 */
static double s_target_turn = 0.0;          /**< 转向外环:目标转向量(累加=积分) */
static double s_turn_error_prev = 0.0;      /**< 转向内环:上一次转向量误差 */
static double s_steer_cmd = 0.0;            /**< 转向内环:差速舵令(累加=积分) */
static double s_pos_error_prev = 0.0;       /**< 行进环:上一次沿船首向目标点误差 */
static double s_speed_cmd = 0.0;            /**< 行进环:行进速度(累加=积分) */

/** 2m 圈稳泊:目标船首向锁定 */
static uint8_t s_in_arrival_circle = 0;     /**< 上一拍是否在到达圈内 */
static double  s_desired_heading = 0.0;     /**< 目标船首向(°),圈外为方位角,圈内锁定 */
static uint8_t s_desired_heading_valid = 0; /**< 是否已有目标船首向(含上一帧) */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  将 double 值钳位到 [min, max]
 */
static double ClampDouble(double value, double min, double max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief  验证目标坐标是否有效
 * @retval uint8_t 1=有效,0=无效
 */
static uint8_t ValidateTarget(double lat, double lon)
{
    if (lat < NAV_LAT_MIN || lat > NAV_LAT_MAX) return 0;
    if (lon < NAV_LON_MIN || lon > NAV_LON_MAX) return 0;
    return 1;
}

/**
 * @brief  验证输入参数是否有效
 * @retval uint8_t 1=有效,0=无效
 */
static uint8_t ValidateParameters(double lat, double lon, double heading)
{
    if (lat < NAV_LAT_MIN || lat > NAV_LAT_MAX) return 0;
    if (lon < NAV_LON_MIN || lon > NAV_LON_MAX) return 0;
    if (heading < 0.0 || heading >= 360.0) return 0;
    return 1;
}

/**
 * @brief  计算两点间的大圆距离(Haversine公式)
 * @retval double 两点间距离(米)
 */
static double CalculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * NAV_PI / 180.0;
    double lat2_rad = lat2 * NAV_PI / 180.0;
    double delta_lat = (lat2 - lat1) * NAV_PI / 180.0;
    double delta_lon = (lon2 - lon1) * NAV_PI / 180.0;

    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
              cos(lat1_rad) * cos(lat2_rad) *
              sin(delta_lon / 2.0) * sin(delta_lon / 2.0);

    /* 限制a的范围在[0,1]内,防止浮点误差 */
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return NAV_EARTH_RADIUS * c;
}

/**
 * @brief  计算从起点到终点的方位角
 * @retval double 方位角(度,0~360,正北为0,顺时针)
 */
static double CalculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * NAV_PI / 180.0;
    double lat2_rad = lat2 * NAV_PI / 180.0;
    double delta_lon = (lon2 - lon1) * NAV_PI / 180.0;

    double x = sin(delta_lon) * cos(lat2_rad);
    double y = cos(lat1_rad) * sin(lat2_rad) -
              sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);

    double bearing = atan2(x, y) * 180.0 / NAV_PI;
    return fmod(bearing + 360.0, 360.0);
}

/**
 * @brief  将角度差归一化到 -180° ~ 180° 范围
 */
static double NormalizeAngle(double angle_diff)
{
    while (angle_diff > 180.0) angle_diff -= 360.0;
    while (angle_diff < -180.0) angle_diff += 360.0;
    return angle_diff;
}

/**
 * @brief  清转向环积分(外环目标转向量、内环舵令)
 * @note   进出 2m 圈时调用;行进环积分不清
 */
static void ClearHeadingIntegral(double current_turn)
{
    s_target_turn = 0.0;
    s_steer_cmd = 0.0;
    s_turn_error_prev = 0.0 - current_turn;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化导航控制器模块
 * @retval 无
 */
void NavigationController_Init(void)
{
    s_target.latitude = 0.0;
    s_target.longitude = 0.0;
    s_current.latitude = 0.0;
    s_current.longitude = 0.0;

    s_control_output.speed = 0;
    s_control_output.steering = 0;

    s_nav_status.distance = 0.0;
    s_nav_status.heading_error = 0.0;
    s_nav_status.arrived = 0;

    s_max_speed = NAV_DEFAULT_MAX_SPEED;

    /* 清串级 PI 状态 */
    s_heading_error_prev = 0.0;
    s_target_turn = 0.0;
    s_turn_error_prev = 0.0;
    s_steer_cmd = 0.0;
    s_pos_error_prev = 0.0;
    s_speed_cmd = 0.0;

    s_in_arrival_circle = 0;
    s_desired_heading = 0.0;
    s_desired_heading_valid = 0;
}

/**
 * @brief  设置导航最大速度
 * @retval 无
 */
void NavigationController_SetMaxSpeed(int8_t max_speed)
{
    if (max_speed > NAV_DEFAULT_MAX_SPEED) {
        max_speed = NAV_DEFAULT_MAX_SPEED;
    }
    if (max_speed < 0) {
        max_speed = 0;
    }
    s_max_speed = max_speed;
}

/**
 * @brief  设置导航目标坐标
 * @retval 无
 */
void NavigationController_SetTarget(double target_lat, double target_lon)
{
    if (!ValidateTarget(target_lat, target_lon)) {
        NAV_LOG_PRINT("Invalid target coordinates: lat=%.6f, lon=%.6f\r\n",
                      (double)target_lat, (double)target_lon);
        return;
    }

    /* 目标为零点则忽略,避免误设为无效目标 */
    if (target_lat == 0.0 && target_lon == 0.0) {
        return;
    }

    s_target.latitude = target_lat;
    s_target.longitude = target_lon;
    s_nav_status.arrived = 0;

    /* 切换目标/模式:重置串级 PI 全部状态,避免上一段航程累积量影响本次导航 */
    s_heading_error_prev = 0.0;
    s_target_turn = 0.0;
    s_turn_error_prev = 0.0;
    s_steer_cmd = 0.0;
    s_pos_error_prev = 0.0;
    s_speed_cmd = 0.0;

    /* 换目标后上一帧目标船首向作废,下一拍按是否已在圈内重新锁定 */
    s_in_arrival_circle = 0;
    s_desired_heading = 0.0;
    s_desired_heading_valid = 0;
}

/**
 * @brief  更新导航控制器数据(周期调用)
 * @retval 无
 */
void NavigationController_Update(double current_lat, double current_lon,
                                 double current_heading, double current_yaw_rate)
{
    /* 校验输入参数是否有效 */
    if (!ValidateParameters(current_lat, current_lon, current_heading)) {
        NAV_LOG_PRINT("Invalid input: lat=%.6f, lon=%.6f, heading=%.2f\r\n",
                      (double)current_lat, (double)current_lon, (double)current_heading);
        return;
    }

    /* 更新当前坐标 */
    s_current.latitude = current_lat;
    s_current.longitude = current_lon;

    /* 计算到目标点的距离与方位角 */
    double distance = CalculateDistance(
        s_current.latitude, s_current.longitude,
        s_target.latitude, s_target.longitude);

    double target_bearing = CalculateBearing(
        s_current.latitude, s_current.longitude,
        s_target.latitude, s_target.longitude);

    /* 当前偏航角速度换算到转向量量纲(°/s -> 转向量单位) */
    double current_turn = (double)current_yaw_rate / NAV_STEER_CMD_TO_DEG_S;

    uint8_t in_circle = (distance <= (double)NAV_ARRIVAL_THRESHOLD) ? 1u : 0u;

    /* ---------- 旧逻辑:进入到达圈完全停机(已停用) ---------- */
#if 0
    /* 航向误差(当前航向与目标方位的差值,归一化到±180°) */
    double heading_error = NormalizeAngle(current_heading - target_bearing);

    s_nav_status.distance = distance;
    s_nav_status.heading_error = heading_error;

    if (distance <= NAV_ARRIVAL_THRESHOLD) {
        if (!s_nav_status.arrived) {
            NAV_LOG_PRINT("Arrived at target! Distance: %.2f m\r\n", (double)distance);
        }
        s_nav_status.arrived = 1;

        s_speed_cmd = 0.0;
        s_steer_cmd = 0.0;
        /* 同步误差记忆,避免退出到达状态时 PI 突跳 */
        s_heading_error_prev = heading_error;
        s_turn_error_prev = 0.0 - current_turn;
        s_pos_error_prev = distance - NAV_ARRIVAL_THRESHOLD;

        s_control_output.speed = 0;
        s_control_output.steering = 0;
        return;
    }
    s_nav_status.arrived = 0;
#endif

    /* ---------- 2m 圈:圈内稳泊,圈外导航回目标 ---------- */
    if (in_circle) {
        /* 圈内:继续稳泊输出,不把速度/舵令清零 */
        if (!s_in_arrival_circle) {
            /* 刚进入 2m 圈:目标船首向改成上一帧的目标船首向 */
            if (!s_desired_heading_valid) {
                /* 没有上一帧目标船首向则保持当前船首向 */
                s_desired_heading = current_heading;
                s_desired_heading_valid = 1;
            }
            /* 进出圈切换时清转向积分,行进环积分保持不变 */
            ClearHeadingIntegral(current_turn);
            NAV_LOG_PRINT("Enter hold circle, lock heading=%.2f, dist=%.2f m\r\n",
                          (double)s_desired_heading, (double)distance);
        }
        s_nav_status.arrived = 1;
    } else {
        /* 圈外:先导航回目标点,目标船首向取目标方位角 */
        if (s_in_arrival_circle) {
            ClearHeadingIntegral(current_turn);
            NAV_LOG_PRINT("Leave hold circle, navigate back, dist=%.2f m\r\n",
                          (double)distance);
        }
        s_desired_heading = target_bearing;
        s_desired_heading_valid = 1;
        s_nav_status.arrived = 0;
    }

    /* 航向误差相对锁定/导航用的目标船首向,而非圈内跳动的瞬时方位角 */
    double heading_error = NormalizeAngle(current_heading - s_desired_heading);
    s_nav_status.distance = distance;
    s_nav_status.heading_error = heading_error;

    if (in_circle != s_in_arrival_circle) {
        /* 切圈后同步误差记忆,避免转向外环 P 项突跳 */
        s_heading_error_prev = heading_error;
    }
    s_in_arrival_circle = in_circle;

    /* ---------- 转向外环:增量式 PI(航向误差 -> 目标转向量) ---------- */
    double delta_turn = NAV_HEADING_KP * (heading_error - s_heading_error_prev) +
                        NAV_HEADING_KI * heading_error;
    s_target_turn += delta_turn;
    s_target_turn = ClampDouble(s_target_turn, -NAV_MAX_TURN_RATE, NAV_MAX_TURN_RATE);
    s_heading_error_prev = heading_error;

    /* ---------- 转向内环:增量式 PI(转向量误差 -> 差速舵令) ---------- */
    double turn_error = s_target_turn - current_turn;
    double steer_delta = NAV_TURN_RATE_KP * (turn_error - s_turn_error_prev) +
                         NAV_TURN_RATE_KI * turn_error;
    s_steer_cmd += steer_delta;
    s_steer_cmd = ClampDouble(s_steer_cmd, -(double)s_max_speed, (double)s_max_speed);
    s_turn_error_prev = turn_error;

    /* ---------- 位置环:设定点=目标点,不是 2m 圈缘 ---------- */
#if 0
    /* 旧逻辑:航向误差超过 NAV_ANGLE_ERROR_MAX(32°)时停速原地转向 */
    if (heading_error > NAV_ANGLE_ERROR_MAX || heading_error < -NAV_ANGLE_ERROR_MAX) {
        /* 航向误差过大:停速原地转向,避免偏离方向 */
        s_speed_cmd = 0.0;
        s_pos_error_prev = distance - NAV_ARRIVAL_THRESHOLD;
    } else {
        /* 旧行进误差:distance-2m,平衡点在圈缘,顶风时进不了圈、也稳不住点 */
        double pos_error = distance - NAV_ARRIVAL_THRESHOLD;
        double delta_speed = NAV_POSITION_KP * (pos_error - s_pos_error_prev) +
                             NAV_POSITION_KI * pos_error;
        s_speed_cmd += delta_speed;
        s_speed_cmd = ClampDouble(s_speed_cmd, 0.0, (double)s_max_speed);
        s_pos_error_prev = pos_error;
    }
#endif
    {
        /* 把到目标点的距离投到当前船首向:
         *   >0 目标在前方,前进; ≤0 已越过或船首没对准,速度收到 0(禁止倒车)
         * 圈缘处误差仍约等于距离(~2m),能继续出力顶风进圈,而不是在圈上停速 */
        double bearing_off = NormalizeAngle(target_bearing - current_heading);
        double pos_error = distance * cos(bearing_off * NAV_PI / 180.0);
        double delta_speed = NAV_POSITION_KP * (pos_error - s_pos_error_prev) +
                             NAV_POSITION_KI * pos_error;
        s_speed_cmd += delta_speed;
        s_speed_cmd = ClampDouble(s_speed_cmd, 0.0, (double)s_max_speed);
        s_pos_error_prev = pos_error;
    }

    /* 输出(截断为驱动板指令量纲) */
    s_control_output.speed = (int8_t)s_speed_cmd;
    s_control_output.steering = (int8_t)s_steer_cmd;
}

/**
 * @brief  获取导航控制输出
 * @retval NavControlOutput_t 导航控制输出结构体
 */
NavControlOutput_t NavigationController_GetControl(void)
{
    return s_control_output;
}

/**
 * @brief  获取导航状态信息
 * @retval NavStatus_t 导航状态信息结构体
 */
NavStatus_t NavigationController_GetStatus(void)
{
    return s_nav_status;
}

/**
 * @brief  获取当前导航目标坐标
 * @retval GeoCoordinate_t 当前目标坐标结构体
 */
GeoCoordinate_t NavigationController_GetTarget(void)
{
    return s_target;
}

/**
 * @brief  根据起点、航向和距离计算目标位置(大圆导航公式)
 * @retval 无
 */
void NavigationController_CalculateTargetPosition(double start_lat, double start_lon,
                                                  double heading, double distance,
                                                  double *out_lat, double *out_lon)
{
    if (out_lat == NULL || out_lon == NULL) {
        return;
    }

    double lat_rad = start_lat * NAV_PI / 180.0;
    double lon_rad = start_lon * NAV_PI / 180.0;
    double heading_rad = heading * NAV_PI / 180.0;

    double angular_distance = distance / NAV_EARTH_RADIUS;

    double new_lat_rad = asin(
        sin(lat_rad) * cos(angular_distance) +
        cos(lat_rad) * sin(angular_distance) * cos(heading_rad));

    double new_lon_rad = lon_rad + atan2(
        sin(heading_rad) * sin(angular_distance) * cos(lat_rad),
        cos(angular_distance) - sin(lat_rad) * sin(new_lat_rad));

    *out_lat = new_lat_rad * 180.0 / NAV_PI;
    *out_lon = new_lon_rad * 180.0 / NAV_PI;
}
