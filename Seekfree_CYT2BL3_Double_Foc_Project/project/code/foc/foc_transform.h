#ifndef _FOC_TRANSFORM_H_
#define _FOC_TRANSFORM_H_

#include "zf_common_typedef.h"
#include <math.h>

// 数学常量：统一在变换库中定义，避免多处硬编码导致口径漂移
#define FOC_INV_SQRT3       (0.57735026919f)     // 1/sqrt(3)
#define FOC_SQRT3_OVER_2    (0.86602540378f)     // sqrt(3)/2
#define FOC_PI              (3.14159265359f)
#define FOC_TWO_PI          (6.28318530718f)

// sin/cos查表参数：4096点覆盖0~2pi，角分辨率约0.0879度
#define FOC_SIN_COS_LUT_SIZE        (4096)
#define FOC_SIN_COS_LUT_INV_TWO_PI  ((float)FOC_SIN_COS_LUT_SIZE / FOC_TWO_PI)

// 约定说明（必须统一）：
// 1) Clarke:  alpha = ia, beta = (ia + 2*ib)/sqrt(3)
// 2) Park:    id = alpha*cos + beta*sin, iq = -alpha*sin + beta*cos
// 3) 反Park:  alpha = vd*cos - vq*sin, beta = vd*sin + vq*cos
// 4) 反Clarke: va = alpha, vb = -0.5*alpha + sqrt(3)/2*beta, vc = -0.5*alpha - sqrt(3)/2*beta
// 以上 1<->4、2<->3 为严格镜像关系。

typedef struct
{
    float alpha;
    float beta;
}foc_clarke_result_t;

typedef struct
{
    float id;
    float iq;
}foc_park_result_t;

typedef struct
{
    float alpha;
    float beta;
}foc_inv_park_result_t;

typedef struct
{
    float va;
    float vb;
    float vc;
}foc_inv_clarke_result_t;

typedef struct
{
    float ia_err;
    float ib_err;
    float ic_err;
    float alpha_err;
    float beta_err;
}foc_transform_check_result_t;

static uint8 foc_trig_lut_inited = 0;
static float foc_sin_lut[FOC_SIN_COS_LUT_SIZE];
static float foc_cos_lut[FOC_SIN_COS_LUT_SIZE];

// 角度映射到LUT索引：[0, 2pi) -> [0, FOC_SIN_COS_LUT_SIZE)
static inline uint16 foc_lut_angle_to_index(float electrical_angle_rad)
{
    float angle_temp = electrical_angle_rad;
    int32 index = 0;

    while(angle_temp < 0.0f)
    {
        angle_temp += FOC_TWO_PI;
    }
    while(angle_temp >= FOC_TWO_PI)
    {
        angle_temp -= FOC_TWO_PI;
    }

    index = (int32)(angle_temp * FOC_SIN_COS_LUT_INV_TWO_PI + 0.5f);
    if(index >= FOC_SIN_COS_LUT_SIZE)
    {
        index = 0;
    }

    return (uint16)index;
}

// 显式建表初始化函数：建议系统初始化阶段调用一次
static inline void foc_trig_lut_init(void)
{
    uint16 i = 0;

    if(foc_trig_lut_inited)
    {
        return;
    }

    for(i = 0; i < FOC_SIN_COS_LUT_SIZE; i ++)
    {
        float angle = (float)i * FOC_TWO_PI / (float)FOC_SIN_COS_LUT_SIZE;
        foc_sin_lut[i] = sinf(angle);
        foc_cos_lut[i] = cosf(angle);
    }

    foc_trig_lut_inited = 1;
}

// sin查表函数（与cos查表分离，便于单独调用）
static inline float foc_lut_sin(float electrical_angle_rad)
{
    if(0 == foc_trig_lut_inited)
    {
        foc_trig_lut_init();
    }

    return foc_sin_lut[foc_lut_angle_to_index(electrical_angle_rad)];
}

// cos查表函数（与sin查表分离，便于单独调用）
static inline float foc_lut_cos(float electrical_angle_rad)
{
    if(0 == foc_trig_lut_inited)
    {
        foc_trig_lut_init();
    }

    return foc_cos_lut[foc_lut_angle_to_index(electrical_angle_rad)];
}

static inline foc_clarke_result_t foc_clarke_transform(float ia, float ib, float ic)
{
    foc_clarke_result_t result;
    (void)ic;

    result.alpha = ia;
    result.beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;

    return result;
}

static inline foc_park_result_t foc_park_transform(float alpha, float beta, float electrical_angle_rad)
{
    foc_park_result_t result;
    float sin_theta = foc_lut_sin(electrical_angle_rad);
    float cos_theta = foc_lut_cos(electrical_angle_rad);

    result.id = alpha * cos_theta + beta * sin_theta;
    result.iq = -alpha * sin_theta + beta * cos_theta;

    return result;
}

static inline foc_inv_park_result_t foc_inv_park_transform(float vd, float vq, float electrical_angle_rad)
{
    foc_inv_park_result_t result;
    float sin_theta = foc_lut_sin(electrical_angle_rad);
    float cos_theta = foc_lut_cos(electrical_angle_rad);

    result.alpha = vd * cos_theta - vq * sin_theta;
    result.beta = vd * sin_theta + vq * cos_theta;

    return result;
}

static inline foc_inv_clarke_result_t foc_inv_clarke_transform(float alpha, float beta)
{
    foc_inv_clarke_result_t result;

    result.va = alpha;
    result.vb = -0.5f * alpha + FOC_SQRT3_OVER_2 * beta;
    result.vc = -0.5f * alpha - FOC_SQRT3_OVER_2 * beta;

    return result;
}

static inline float foc_degree_to_radian(float degree)
{
    return degree * 0.01745329252f;
}

// 镜像一致性检查：
// 用 abc -> Clarke -> Park -> 反Park -> 反Clarke 回环，输出各量误差。
// 若误差接近0（比如 1e-4 量级），说明正反变换口径一致，可用于闭环。
static inline foc_transform_check_result_t foc_transform_mirror_check(float ia, float ib, float ic, float electrical_angle_rad)
{
    foc_transform_check_result_t check_result;
    foc_clarke_result_t clarke_result;
    foc_park_result_t park_result;
    foc_inv_park_result_t inv_park_result;
    foc_inv_clarke_result_t inv_clarke_result;

    clarke_result = foc_clarke_transform(ia, ib, ic);
    park_result = foc_park_transform(clarke_result.alpha, clarke_result.beta, electrical_angle_rad);
    inv_park_result = foc_inv_park_transform(park_result.id, park_result.iq, electrical_angle_rad);
    inv_clarke_result = foc_inv_clarke_transform(inv_park_result.alpha, inv_park_result.beta);

    check_result.ia_err = ia - inv_clarke_result.va;
    check_result.ib_err = ib - inv_clarke_result.vb;
    check_result.ic_err = ic - inv_clarke_result.vc;
    check_result.alpha_err = clarke_result.alpha - inv_park_result.alpha;
    check_result.beta_err = clarke_result.beta - inv_park_result.beta;

    return check_result;
}

#endif
