#ifndef _FOC_CONTROLLER_H_
#define _FOC_CONTROLLER_H_

#include "zf_common_typedef.h"
#include "foc_svpwm.h"

// 通用PI控制器
// 输出 = kp * err + integral
// integral 由 ki * err 累积，并受 [integral_min, integral_max] 限制
// 最终输出受 [out_min, out_max] 限制
// 该实现不依赖浮点数学库中的高级函数，适配中断快速执行

typedef struct
{
    float kp;
    float ki;

    float integral;
    float integral_min;
    float integral_max;

    float out_min;
    float out_max;
}foc_pi_t;

// 速度环（外环）状态
// 建议 2kHz 执行：在 20kHz ISR 中按分频计数触发
// 输出 iq_ref 提供给电流环

typedef struct
{
    foc_pi_t speed_pi;

    float speed_ref_rpm;
    float speed_fb_rpm;

    float iq_ref;
    float iq_limit;

    uint16 divider;      // 电流环/速度环分频比，例如 20k/2k=10
    uint16 counter;      // 分频计数器
}foc_speed_loop_t;

// 电流环（内环）状态
// 建议 20kHz 执行
// 输出 vd/vq，随后进入反Park/反Clarke/SVPWM

typedef struct
{
    foc_pi_t id_pi;
    foc_pi_t iq_pi;

    float id_ref;
    float iq_ref;

    float id_fb;
    float iq_fb;

    float vd;
    float vq;

    float vd_limit;
    float vq_limit;
}foc_current_loop_t;

// FOC闭环控制器总状态

typedef struct
{
    foc_speed_loop_t speed_loop;
    foc_current_loop_t current_loop;

    float electrical_angle_deg;
    float bus_voltage;

    foc_svpwm_duty_t svpwm_duty;
    uint16 compare_value[3];
}foc_closed_loop_t;

void foc_pi_init(foc_pi_t *pi,
                 float kp,
                 float ki,
                 float out_min,
                 float out_max,
                 float integral_min,
                 float integral_max);

void foc_pi_reset(foc_pi_t *pi);

float foc_pi_update(foc_pi_t *pi, float err);

void foc_closed_loop_init(foc_closed_loop_t *ctrl,
                          float speed_kp,
                          float speed_ki,
                          float id_kp,
                          float id_ki,
                          float iq_kp,
                          float iq_ki,
                          uint16 current_loop_hz,
                          uint16 speed_loop_hz,
                          float iq_limit,
                          float vdq_limit);

void foc_closed_loop_reset(foc_closed_loop_t *ctrl);

void foc_closed_loop_set_speed_ref(foc_closed_loop_t *ctrl, float speed_ref_rpm);

void foc_closed_loop_set_id_ref(foc_closed_loop_t *ctrl, float id_ref);

// 每次在电流环中断调用（例如20kHz）
// 1) 速度环按分频触发，更新 iq_ref
// 2) 电流环每次执行，更新 vd/vq
// 3) 执行SVPWM得到 compare_value[3]
void foc_closed_loop_step(foc_closed_loop_t *ctrl,
                          float id_fb,
                          float iq_fb,
                          float speed_fb_rpm,
                          float electrical_angle_deg,
                          float bus_voltage,
                          uint16 duty_max);

#endif
