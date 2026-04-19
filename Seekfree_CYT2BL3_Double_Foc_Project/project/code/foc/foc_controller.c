#include "foc_controller.h"



// 函数作用：初始化一个通用PI控制器结构体。
// 如何调用：通常在系统初始化阶段调用一次；参数更新后可再次调用以重装配置。
// 参数说明：
// pi            PI实例指针
// kp            比例系数
// ki            积分系数（每次调用foc_pi_update时累加 ki * err）
// out_min/max   PI最终输出限幅
// integral_min/max 积分项限幅（防止积分漂移和过饱和）
// 内部逻辑：
// 1) 判空保护
// 2) 写入增益与上下限
// 3) 清零积分初值，避免上电带入历史状态
void foc_pi_init(foc_pi_t *pi,
                 float kp,
                 float ki,
                 float out_min,
                 float out_max,
                 float integral_min,
                 float integral_max)
{
    if(NULL == pi)
    {
        return;
    }

    pi->kp = kp;
    pi->ki = ki;

    pi->integral = 0.0f;
    pi->integral_min = integral_min;
    pi->integral_max = integral_max;

    pi->out_min = out_min;
    pi->out_max = out_max;
}

// 函数作用：复位PI积分状态。
// 如何调用：
// 1) 使能闭环前
// 2) 模式切换时
// 3) 大扰动/保护恢复后
// 参数说明：pi 为PI实例指针。
// 内部逻辑：仅清零积分项，不改动 kp/ki 和限幅参数。
void foc_pi_reset(foc_pi_t *pi)
{
    if(NULL == pi)
    {
        return;
    }

    pi->integral = 0.0f;
}
// 函数作用：复位闭环运行态，保留参数，仅清状态。
// 如何调用：停车、重新启动、模式切换、保护恢复后建议调用。
// 参数说明：ctrl 为闭环实例。
// 内部逻辑：
// 1) 复位速度环PI、电流环PI积分
// 2) 清空速度环分频计数和速度环输出iq_ref
// 3) 清空内环电压输出与PWM比较值
void foc_closed_loop_reset(foc_closed_loop_t *ctrl)
{
    if(NULL == ctrl)
    {
        return;
    }

    foc_pi_reset(&ctrl->speed_loop.speed_pi);
    foc_pi_reset(&ctrl->current_loop.id_pi);
    foc_pi_reset(&ctrl->current_loop.iq_pi);

    ctrl->speed_loop.counter = 0;
    ctrl->speed_loop.speed_ref_enable = 0;
    ctrl->speed_loop.iq_ref = 0.0f;

    ctrl->current_loop.vd = 0.0f;
    ctrl->current_loop.vq = 0.0f;
    ctrl->compare_value[0] = 0;
    ctrl->compare_value[1] = 0;
    ctrl->compare_value[2] = 0;
}
// 函数作用：初始化FOC双环控制器（速度环+电流环）及调制输出状态。
// 如何调用：系统启动时调用一次；若参数在线更新后也可重新调用。
// 参数说明：
// ctrl            闭环控制器实例
// speed_kp/ki     速度环PI参数（外环，输出iq_ref）
// id_kp/ki        d轴电流环PI参数（内环，输出vd）
// iq_kp/ki        q轴电流环PI参数（内环，输出vq）
// current_loop_hz 电流环执行频率（例如20kHz）
// speed_loop_hz   速度环执行频率（例如2kHz）
// iq_limit        速度环输出的iq_ref限幅
// vdq_limit       内环输出电压vd/vq限幅
// 内部计算逻辑：
// 1) 根据频率计算速度环分频比 divider = current_loop_hz / speed_loop_hz
// 2) 初始化速度环状态和速度PI
// 3) 初始化电流环状态与 id/iq 两个PI
// 4) 初始化调制相关状态（电角度、母线电压默认值、占空比/比较值）
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
                          float vdq_limit)
{
    uint16 divider = 1;

    if(NULL == ctrl)
    {
        return;
    }

    if(speed_loop_hz > 0)
    {
        divider = (uint16)(current_loop_hz / speed_loop_hz);
        if(divider == 0)
        {
            divider = 1;
        }
    }

    ctrl->speed_loop.divider = divider;
    ctrl->speed_loop.counter = 0;
    ctrl->speed_loop.speed_ref_rpm = 0.0f;
    ctrl->speed_loop.speed_fb_rpm = 0.0f;
    ctrl->speed_loop.speed_ref_enable = 0;
    ctrl->speed_loop.iq_ref = 0.0f;
    ctrl->speed_loop.iq_limit = (iq_limit > 0.0f) ? iq_limit : -iq_limit;

    foc_pi_init(&ctrl->speed_loop.speed_pi,
                speed_kp,
                speed_ki,
                -ctrl->speed_loop.iq_limit,
                 ctrl->speed_loop.iq_limit,
                -ctrl->speed_loop.iq_limit,
                 ctrl->speed_loop.iq_limit);

    ctrl->current_loop.id_ref = 0.0f;
    ctrl->current_loop.iq_ref = 0.0f;
    ctrl->current_loop.id_fb = 0.0f;
    ctrl->current_loop.iq_fb = 0.0f;
    ctrl->current_loop.vd = 0.0f;
    ctrl->current_loop.vq = 0.0f;
    ctrl->current_loop.vd_limit = (vdq_limit > 0.0f) ? vdq_limit : -vdq_limit;
    ctrl->current_loop.vq_limit = (vdq_limit > 0.0f) ? vdq_limit : -vdq_limit;

    foc_pi_init(&ctrl->current_loop.id_pi,
                id_kp,
                id_ki,
                -ctrl->current_loop.vd_limit,
                 ctrl->current_loop.vd_limit,
                -ctrl->current_loop.vd_limit,
                 ctrl->current_loop.vd_limit);

    foc_pi_init(&ctrl->current_loop.iq_pi,
                iq_kp,
                iq_ki,
                -ctrl->current_loop.vq_limit,
                 ctrl->current_loop.vq_limit,
                -ctrl->current_loop.vq_limit,
                 ctrl->current_loop.vq_limit);

    ctrl->electrical_angle_deg = 0.0f;
    ctrl->bus_voltage = 12.0f;
    ctrl->svpwm_duty.duty_a = 0.5f;
    ctrl->svpwm_duty.duty_b = 0.5f;
    ctrl->svpwm_duty.duty_c = 0.5f;
    ctrl->compare_value[0] = 0;
    ctrl->compare_value[1] = 0;
    ctrl->compare_value[2] = 0;
}
// 函数作用：执行一次离散PI更新，输出控制量。
// 如何调用：在固定周期控制回调中周期调用（周期由上层ISR频率决定）。
// 参数说明：
// pi   PI实例指针
// err  当前误差（参考值 - 反馈值）
// 返回值：本次PI输出。
// 内部计算逻辑：
// 1) 积分前保存 integral_before，用于后续抗积分饱和回退
// 2) 积分更新：integral += ki * err
// 3) 积分限幅：integral 限制在 [integral_min, integral_max]
// 4) 比例计算：proportional = kp * err
// 5) 输出合成：out_value = proportional + integral
// 6) 输出限幅与抗积分饱和：
//    当输出已饱和且误差仍推动同方向饱和时，撤销本次积分更新
//    可降低饱和区积分累积导致的恢复迟滞
float foc_pi_update(foc_pi_t *pi, float err)
{
    float integral_before = 0.0f;
    float proportional = 0.0f;
    float out_value = 0.0f;

    if(NULL == pi)
    {
        return 0.0f;
    }

    integral_before = pi->integral;
    pi->integral += pi->ki * err;
    pi->integral = foc_limit_value(pi->integral, pi->integral_min, pi->integral_max);

    proportional = pi->kp * err;
    out_value = proportional + pi->integral;

    // 抗积分饱和：当输出饱和且误差仍推动同方向饱和时，撤销本次积分
    if(out_value > pi->out_max)
    {
        if(err > 0.0f)
        {
            pi->integral = integral_before;
        }
        out_value = pi->out_max;
    }
    else if(out_value < pi->out_min)
    {
        if(err < 0.0f)
        {
            pi->integral = integral_before;
        }
        out_value = pi->out_min;
    }

    return out_value;
}




// 函数作用：设置速度环目标值（RPM）。
// 如何调用：上层速度指令更新时调用，可每个控制周期调用也可按需调用。
// 参数说明：
// ctrl          闭环实例
// speed_ref_rpm 速度目标，单位RPM
// 内部逻辑：仅写入目标，不做滤波与限幅。
void foc_closed_loop_set_speed_ref(foc_closed_loop_t *ctrl, float speed_ref_rpm)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->speed_loop.speed_ref_rpm = speed_ref_rpm;
}

// 函数作用：设置速度参考使能开关。
// 如何调用：
// 1) 进入速度闭环前调用 enable=1
// 2) 暂停速度闭环或切到其它模式时调用 enable=0
// 参数说明：
// ctrl   闭环实例
// enable 速度参考使能标志，非0为使能，0为关闭
// 内部逻辑：仅更新 speed_ref_enable，不改 speed_ref_rpm 数值。
void foc_closed_loop_set_speed_ref_enable(foc_closed_loop_t *ctrl, uint8 enable)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->speed_loop.speed_ref_enable = (enable != 0) ? 1 : 0;
}

// 函数作用：清空速度参考并关闭速度参考使能。
// 如何调用：电机停车、保护触发、模式切换退出速度环时调用。
// 参数说明：ctrl 为闭环实例。
// 内部逻辑：
// 1) speed_ref_rpm 清零
// 2) speed_ref_enable 置0
void foc_closed_loop_clear_speed_ref(foc_closed_loop_t *ctrl)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->speed_loop.speed_ref_rpm = 0.0f;
    ctrl->speed_loop.speed_ref_enable = 0;
}

// 函数作用：设置d轴电流目标（A）。
// 如何调用：
// 1) 常规FOC下可固定为0
// 2) 需要磁链控制/弱磁控制时由上层实时更新
// 参数说明：
// ctrl   闭环实例
// id_ref d轴电流目标，单位A
// 内部逻辑：仅写入目标，不做限幅。
void foc_closed_loop_set_id_ref(foc_closed_loop_t *ctrl, float id_ref)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->current_loop.id_ref = id_ref;
}

// 函数作用：设置q轴电流目标（A），用于力矩环模式。
// 如何调用：上层希望直接控制力矩（Iq）时调用。
// 参数说明：
// ctrl   闭环实例
// iq_ref q轴电流目标，单位A
// 内部逻辑：
// 1) 限幅到 [-iq_limit, iq_limit]
// 2) 关闭速度环输入，使控制器进入“直接电流环”路径
// 3) 同步写入 speed_loop.iq_ref，便于调试查看
void foc_closed_loop_set_iq_ref(foc_closed_loop_t *ctrl, float iq_ref)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->current_loop.iq_ref = foc_limit_value(iq_ref,
                                                -ctrl->speed_loop.iq_limit,
                                                 ctrl->speed_loop.iq_limit);

    ctrl->speed_loop.iq_ref = ctrl->current_loop.iq_ref;
    ctrl->speed_loop.speed_ref_enable = 0;
}

// 函数作用：执行一次完整双环闭环迭代，并生成三相PWM比较值。
// 如何调用：应在电流环中断中固定频率调用（例如20kHz）。
// 参数说明：
// ctrl                 闭环实例
// id_fb/iq_fb          dq轴电流反馈（A）
// speed_fb_rpm         速度反馈（RPM）
// electrical_angle_deg 当前电角度（deg）
// bus_voltage          母线电压（V）
// duty_max             PWM周期对应的最大比较值
// 内部计算逻辑：
// 1) 写入本周期反馈与状态（电角度、母线电压）
// 2) 速度环按分频执行：
//    2.1) 计数到分频阈值后计算速度误差
//    2.2) 速度PI输出 iq_ref，并按 iq_limit 限幅
// 3) 将速度环输出作为电流环 q轴参考
// 4) 电流环每周期执行：
//    4.1) 计算 id/iq 误差
//    4.2) 通过 id_pi/iq_pi 生成 vd/vq
//    4.3) 对 vd/vq 做电压限幅
// 5) 将 vd/vq + 电角度 + 母线电压送入调制链：
//    反Park -> 反Clarke -> SVPWM，输出 compare_value[3]
void foc_closed_loop_step(foc_closed_loop_t *ctrl,
                          float id_fb,
                          float iq_fb,
                          float speed_fb_rpm,
                          float electrical_angle_deg,
                          float bus_voltage,
                          uint16 duty_max)
{
    float speed_err = 0.0f;
    float id_err = 0.0f;
    float iq_err = 0.0f;

    if(NULL == ctrl)
    {
        return;
    }

    ctrl->current_loop.id_fb = id_fb;
    ctrl->current_loop.iq_fb = iq_fb;
    ctrl->speed_loop.speed_fb_rpm = speed_fb_rpm;
    ctrl->electrical_angle_deg = electrical_angle_deg;
    ctrl->bus_voltage = bus_voltage;

    // 速度模式：速度环分频执行（20kHz ISR下，divider=10即2kHz）
    if(ctrl->speed_loop.speed_ref_enable)
    {
        ctrl->speed_loop.counter ++;
        if(ctrl->speed_loop.counter >= ctrl->speed_loop.divider)
        {
            ctrl->speed_loop.counter = 0;

            speed_err = ctrl->speed_loop.speed_ref_rpm - ctrl->speed_loop.speed_fb_rpm;
            ctrl->speed_loop.iq_ref = foc_pi_update(&ctrl->speed_loop.speed_pi, speed_err);
            ctrl->speed_loop.iq_ref = foc_limit_value(ctrl->speed_loop.iq_ref,
                                                      -ctrl->speed_loop.iq_limit,
                                                       ctrl->speed_loop.iq_limit);
        }

        ctrl->current_loop.iq_ref = ctrl->speed_loop.iq_ref;
    }
    else
    {
        // 力矩模式：跳过速度环，直接使用 current_loop.iq_ref
        ctrl->speed_loop.counter = 0;
        ctrl->current_loop.iq_ref = foc_limit_value(ctrl->current_loop.iq_ref,
                                                    -ctrl->speed_loop.iq_limit,
                                                     ctrl->speed_loop.iq_limit);
        ctrl->speed_loop.iq_ref = ctrl->current_loop.iq_ref;
    }

    // 电流环每次执行（20kHz）
    id_err = ctrl->current_loop.id_ref - ctrl->current_loop.id_fb;
    iq_err = ctrl->current_loop.iq_ref - ctrl->current_loop.iq_fb;

    ctrl->current_loop.vd = foc_pi_update(&ctrl->current_loop.id_pi, id_err);
    ctrl->current_loop.vq = foc_pi_update(&ctrl->current_loop.iq_pi, iq_err);

    ctrl->current_loop.vd = foc_limit_value(ctrl->current_loop.vd,
                                            -ctrl->current_loop.vd_limit,
                                             ctrl->current_loop.vd_limit);
    ctrl->current_loop.vq = foc_limit_value(ctrl->current_loop.vq,
                                            -ctrl->current_loop.vq_limit,
                                             ctrl->current_loop.vq_limit);

    foc_voltage_to_svpwm(ctrl->current_loop.vd,
                         ctrl->current_loop.vq,
                         ctrl->electrical_angle_deg,
                         ctrl->bus_voltage,
                         duty_max,
                         &ctrl->svpwm_duty,
                         ctrl->compare_value);
}
// 内部工具函数：对输入值做上下限约束。
// 调用场景：PI积分限幅、PI输出限幅、电流/电压参考限幅。
// 参数含义：
// value     待限幅的输入值
// min_value 下限
// max_value 上限
// 返回值：约束后的数值，保证在 [min_value, max_value] 区间内。
static float foc_limit_value(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}