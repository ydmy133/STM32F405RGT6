/**
 ******************************************************************************
 * @file    gnss_pos_filter.c
 * @brief   GNSS 经纬度三层滤波
 *
 * 原始 lat/lon → 当地平面 EN → ①飞点丢弃 → ②步进限幅 → ③一阶低通 → lat_f/lon_f
 *
 * ① Δs = hypot(ΔE,ΔN), Δs_max = max(0.8, v·Δt+0.6)
 *    超门限则本帧不用、保持上一拍;连续同向 2 帧才放行
 * ② d_lim = (0.6 if v<0.3 else 2.0)·Δt + 0.4, 超出按比例缩到 d_lim
 * ③ α = Δt/(τ+Δt),  E_f ← α E_c + (1-α) E_f
 *    停着 v<0.15: τ=1.5 s; 走着 τ=0.5 s; 距目标 <5 m: τ=1.0 s; τ≤2 s
 ******************************************************************************
 */

#include "gnss_pos_filter.h"
#include <math.h>

#define M_PER_DEG_LAT      111132.92f
#define M_PER_DEG_LON_EQ   111412.84f
#define DEG2RAD            0.01745329252f
#define DT_MIN_S           0.02f
#define DT_MAX_S           1.00f
#define FLY_CONFIRM_N      2

static uint8_t s_ready;
static float s_lat0;
static float s_lon0;
static float s_cos_lat0;
static float s_ef;
static float s_nf;
static float s_lat_f;
static float s_lon_f;

static uint8_t s_pending_n;
static float s_pending_de;
static float s_pending_dn;

static void EnFromLatLon(float lat, float lon, float *e, float *n)
{
    *e = (lon - s_lon0) * M_PER_DEG_LON_EQ * s_cos_lat0;
    *n = (lat - s_lat0) * M_PER_DEG_LAT;
}

static void LatLonFromEn(float e, float n, float *lat, float *lon)
{
    *lat = s_lat0 + n / M_PER_DEG_LAT;
    *lon = s_lon0 + e / (M_PER_DEG_LON_EQ * s_cos_lat0);
}

static float Hypot2(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static float ClampDt(float dt_s)
{
    if (dt_s < DT_MIN_S) {
        return DT_MIN_S;
    }
    if (dt_s > DT_MAX_S) {
        return DT_MAX_S;
    }
    return dt_s;
}

static uint8_t LatLonOk(float lat, float lon)
{
    if ((lat == 0.0f) && (lon == 0.0f)) {
        return 0u;
    }
    if ((lat < -90.0f) || (lat > 90.0f) || (lon < -180.0f) || (lon > 180.0f)) {
        return 0u;
    }
    return 1u;
}

static void Seed(float lat, float lon)
{
    float c;

    s_lat0 = lat;
    s_lon0 = lon;
    c = cosf(lat * DEG2RAD);
    if (c < 0.2f) {
        c = 0.2f;
    }
    s_cos_lat0 = c;
    s_ef = 0.0f;
    s_nf = 0.0f;
    s_lat_f = lat;
    s_lon_f = lon;
    s_pending_n = 0u;
    s_ready = 1u;
}

void GNSSPosFilter_Init(void)
{
    s_ready = 0u;
    s_lat0 = 0.0f;
    s_lon0 = 0.0f;
    s_cos_lat0 = 1.0f;
    s_ef = 0.0f;
    s_nf = 0.0f;
    s_lat_f = 0.0f;
    s_lon_f = 0.0f;
    s_pending_n = 0u;
    s_pending_de = 0.0f;
    s_pending_dn = 0.0f;
}

void GNSSPosFilter_Update(float lat, float lon, float speed_ms,
                          float dt_s, float dist_to_target_m)
{
    float e;
    float n;
    float de;
    float dn;
    float ds;
    float ds_max;
    float d;
    float d_lim;
    float v_lim;
    float tau;
    float alpha;
    float v;
    uint8_t accept;

    if (!LatLonOk(lat, lon)) {
        return;
    }

    if (s_ready == 0u) {
        Seed(lat, lon);
        return;
    }

    dt_s = ClampDt(dt_s);
    v = (speed_ms > 0.0f) ? speed_ms : 0.0f;

    EnFromLatLon(lat, lon, &e, &n);
    de = e - s_ef;
    dn = n - s_nf;
    ds = Hypot2(de, dn);
    ds_max = v * dt_s + 0.6f;
    if (ds_max < 0.8f) {
        ds_max = 0.8f;
    }

    accept = 1u;
    if (ds > ds_max) {
        /* 同向:与上一帧被拒位移点积 > 0 */
        if ((s_pending_n > 0u) && ((de * s_pending_de + dn * s_pending_dn) > 0.0f)) {
            s_pending_n++;
        } else {
            s_pending_n = 1u;
            s_pending_de = de;
            s_pending_dn = dn;
        }
        if (s_pending_n < FLY_CONFIRM_N) {
            accept = 0u;
        } else {
            s_pending_n = 0u;
        }
    } else {
        s_pending_n = 0u;
    }

    if (accept == 0u) {
        return;
    }

    /* ② 步进限幅:相对滤波值 */
    de = e - s_ef;
    dn = n - s_nf;
    d = Hypot2(de, dn);
    v_lim = (v < 0.3f) ? 0.6f : 2.0f;
    d_lim = v_lim * dt_s + 0.4f;
    if ((d > d_lim) && (d > 1.0e-6f)) {
        e = s_ef + de * (d_lim / d);
        n = s_nf + dn * (d_lim / d);
    }

    /* ③ 一阶低通 */
    if (v < 0.15f) {
        tau = 1.5f;
    } else {
        tau = 0.5f;
    }
    if ((dist_to_target_m >= 0.0f) && (dist_to_target_m < 5.0f) && (tau < 1.0f)) {
        tau = 1.0f;
    }
    if (tau > 2.0f) {
        tau = 2.0f;
    }
    alpha = dt_s / (tau + dt_s);
    s_ef = alpha * e + (1.0f - alpha) * s_ef;
    s_nf = alpha * n + (1.0f - alpha) * s_nf;
    LatLonFromEn(s_ef, s_nf, &s_lat_f, &s_lon_f);
}

uint8_t GNSSPosFilter_Get(float *lat, float *lon)
{
    if ((lat == 0) || (lon == 0) || (s_ready == 0u)) {
        return 0u;
    }
    *lat = s_lat_f;
    *lon = s_lon_f;
    return 1u;
}
