#ifndef _FOC_CONTROLLER_H_
#define _FOC_CONTROLLER_H_

#include "zf_common_typedef.h"
#include "foc_svpwm.h"

// 通用PI控制器（比例-积分控制）
// 变量缩写说明：
// kp: proportional gain，比例系数
// ki: integral gain，积分系数
// err: error，误差（通常定义为 参考值 - 反馈值）
// integral: 积分项，离散实现中每次按 ki * err 累加
// 输出关系：out = kp * err + integral
// 约束关系：
// 1) integral 限制在 [integral_min, integral_max]
// 2) out 限制在 [out_min, out_max]
// 该实现不依赖复杂数学库，适合在中断中高频调用

typedef struct
{
    float kp;               // 比例系数（误差的即时放大倍数）
    float ki;               // 积分系数（误差累计速度）

    float integral;         // 当前积分状态量
    float integral_min;     // 积分下限，防止负向积分饱和
    float integral_max;     // 积分上限，防止正向积分饱和

    float out_min;          // 控制器输出下限
    float out_max;          // 控制器输出上限
}foc_pi_t;

// 速度环（外环）状态
// 建议在较低频率执行（例如2kHz），常见做法是在20kHz电流环中按分频触发
// 速度环输出为 iq_ref（q轴电流目标），作为电流环输入
// 缩写说明：
// speed_ref_rpm: 速度参考值（RPM）
// speed_fb_rpm : 速度反馈值（RPM）
// iq_ref       : q轴电流参考值（A）
// iq_limit     : q轴电流参考限幅（A）

typedef struct
{
    foc_pi_t speed_pi;      // 速度PI控制器

    float speed_ref_rpm;    // 目标速度（RPM）
    float speed_fb_rpm;     // 实际反馈速度（RPM）
    uint8 speed_ref_enable; // 速度参考使能：1=使用 speed_ref_rpm，0=由上层走兼容逻辑

    float iq_ref;           // 速度环输出的q轴电流目标（A）
    float iq_limit;         // q轴电流目标限幅（A）

    uint16 divider;         // 电流环/速度环分频比，例如20kHz/2kHz=10
    uint16 counter;         // 分频计数器，计满 divider 后执行一次速度环
}foc_speed_loop_t;

// 电流环（内环）状态
// 建议高频执行（例如20kHz）
// 输出 vd/vq（d/q轴电压指令），随后进入反Park -> 反Clarke -> SVPWM
// 缩写说明：
// id_ref/id_fb: d轴电流 参考/反馈（A）
// iq_ref/iq_fb: q轴电流 参考/反馈（A）
// vd/vq       : d/q轴电压控制输出（V）

typedef struct
{
    foc_pi_t id_pi;         // d轴电流PI
    foc_pi_t iq_pi;         // q轴电流PI

    float id_ref;           // d轴电流目标（A）
    float iq_ref;           // q轴电流目标（A）

    float id_fb;            // d轴电流反馈（A）
    float iq_fb;            // q轴电流反馈（A）

    float vd;               // d轴电压输出（V）
    float vq;               // q轴电压输出（V）

    float vd_limit;         // d轴电压输出限幅（V）
    float vq_limit;         // q轴电压输出限幅（V）
}foc_current_loop_t;

// FOC闭环控制器总状态

typedef struct
{
    foc_speed_loop_t speed_loop;      // 速度环状态（外环）
    foc_current_loop_t current_loop;  // 电流环状态（内环）

    float electrical_angle_deg;       // 当前电角度（deg）
    float bus_voltage;                // 当前母线电压（V）

    foc_svpwm_duty_t svpwm_duty;      // SVPWM计算得到的三相占空比（0.0~1.0）
    uint16 compare_value[3];          // 三相PWM比较值（A/B/C相，范围通常为0~duty_max）
}foc_closed_loop_t;

// PI初始化
// 参数说明：
// pi            PI实例指针
// kp            比例系数
// ki            积分系数
// out_min/max   输出限幅上下限
// integral_min/max 积分限幅上下限

void foc_pi_init(foc_pi_t *pi,
                 float kp,
                 float ki,
                 float out_min,
                 float out_max,
                 float integral_min,
                 float integral_max);

// 复位PI内部积分状态（不改参数）
// 推荐在闭环重新使能或模式切换时调用

void foc_pi_reset(foc_pi_t *pi);

// 执行一次PI更新
// 输入 err=参考-反馈，返回本次控制输出

float foc_pi_update(foc_pi_t *pi, float err);

// 初始化FOC闭环控制器
// speed_* 为速度环参数，id_*/iq_* 为电流环参数
// current_loop_hz / speed_loop_hz 决定速度环分频执行比
// iq_limit 为速度环输出限幅，vdq_limit 为电流环电压输出限幅
// 参数说明：
// ctrl            闭环控制器实例
// speed_kp/ki     速度环PI参数
// id_kp/ki        d轴电流环PI参数
// iq_kp/ki        q轴电流环PI参数
// current_loop_hz 电流环调用频率（Hz）
// speed_loop_hz   速度环调用频率（Hz）
// iq_limit        速度环输出iq_ref限幅（A）
// vdq_limit       电流环输出vd/vq限幅（V）

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

// 复位闭环运行状态（PI积分、中间量、PWM比较值）

void foc_closed_loop_reset(foc_closed_loop_t *ctrl);

// 设置速度环目标，单位RPM
// speed_ref_rpm: 目标机械转速（RPM）

void foc_closed_loop_set_speed_ref(foc_closed_loop_t *ctrl, float speed_ref_rpm);

// 设置速度参考使能状态。
// 参数说明：
// ctrl   闭环实例
// enable 使能标志，1=启用速度参考输入，0=关闭速度参考输入
// 备注信息：该接口只控制是否使用 speed_ref_rpm，不修改 speed_ref_rpm 本身
void foc_closed_loop_set_speed_ref_enable(foc_closed_loop_t *ctrl, uint8 enable);

// 清除速度参考。
// 参数说明：ctrl 为闭环实例
// 备注信息：调用后 speed_ref_rpm=0 且 speed_ref_enable=0
void foc_closed_loop_clear_speed_ref(foc_closed_loop_t *ctrl);

// 设置d轴电流目标，单位A
// id_ref: d轴电流目标值（A）

void foc_closed_loop_set_id_ref(foc_closed_loop_t *ctrl, float id_ref);

// 每次在电流环中断调用（例如20kHz）
// 1) 速度环按分频触发，更新 iq_ref
// 2) 电流环每次执行，更新 vd/vq
// 3) 执行SVPWM得到 compare_value[3]
// 参数说明：
// id_fb/iq_fb          d/q轴电流反馈（A）
// speed_fb_rpm         速度反馈（RPM）
// electrical_angle_deg 电角度（deg）
// bus_voltage          母线电压（V）
// duty_max             PWM计数最大值（用于占空比转比较值）
void foc_closed_loop_step(foc_closed_loop_t *ctrl,
                          float id_fb,
                          float iq_fb,
                          float speed_fb_rpm,
                          float electrical_angle_deg,
                          float bus_voltage,
                          uint16 duty_max);

#endif
