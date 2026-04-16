#include "foc_controller.h"

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

void foc_pi_reset(foc_pi_t *pi)
{
    if(NULL == pi)
    {
        return;
    }

    pi->integral = 0.0f;
}

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
    ctrl->speed_loop.iq_ref = 0.0f;

    ctrl->current_loop.vd = 0.0f;
    ctrl->current_loop.vq = 0.0f;
    ctrl->compare_value[0] = 0;
    ctrl->compare_value[1] = 0;
    ctrl->compare_value[2] = 0;
}

void foc_closed_loop_set_speed_ref(foc_closed_loop_t *ctrl, float speed_ref_rpm)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->speed_loop.speed_ref_rpm = speed_ref_rpm;
}

void foc_closed_loop_set_id_ref(foc_closed_loop_t *ctrl, float id_ref)
{
    if(NULL == ctrl)
    {
        return;
    }

    ctrl->current_loop.id_ref = id_ref;
}

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

    // 速度环分频执行：20kHz ISR下，divider=10即2kHz
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
