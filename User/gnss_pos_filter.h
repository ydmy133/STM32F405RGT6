/**
 ******************************************************************************
 * @file    gnss_pos_filter.h
 * @brief   GNSS 经纬度三层滤波(飞点丢弃 → 步进限幅 → 一阶低通)
 *
 * 只处理位置,不改速度环。输出 lat_f/lon_f 交给导航当当前点。
 * 全程使用,不是只在 2m 圈内才开。目标点不滤波。
 ******************************************************************************
 */

#ifndef __GNSS_POS_FILTER_H
#define __GNSS_POS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void GNSSPosFilter_Init(void);

/**
 * @brief  推入一帧原始 GNSS 位置
 * @param  lat: 纬度(°)
 * @param  lon: 经度(°)
 * @param  speed_ms: 对地速度(m/s),没有则传 0
 * @param  dt_s: 距上一帧成功采样的时间(s)
 * @param  dist_to_target_m: 到目标距离(m);未知传负数,不启用近目标 τ
 */
void GNSSPosFilter_Update(float lat, float lon, float speed_ms,
                          float dt_s, float dist_to_target_m);

/**
 * @brief  取滤波后经纬度
 * @retval 1=已有有效输出, 0=尚未初始化(仍用原始值)
 */
uint8_t GNSSPosFilter_Get(float *lat, float *lon);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_POS_FILTER_H */
