#include "foc_svpwm.h"

static float foc_limit_range(float value, float min_value, float max_value)
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

static float foc_max3(float a, float b, float c)
{
    float max_value = a;
    if(b > max_value)
    {
        max_value = b;
    }
    if(c > max_value)
    {
        max_value = c;
    }
    return max_value;
}

static float foc_min3(float a, float b, float c)
{
    float min_value = a;
    if(b < min_value)
    {
        min_value = b;
    }
    if(c < min_value)
    {
        min_value = c;
    }
    return min_value;
}

void foc_svpwm_generate(float va, float vb, float vc, float bus_voltage, foc_svpwm_duty_t *svpwm_duty)
{
    float v_max = 0.0f;
    float v_min = 0.0f;
    float v_offset = 0.0f;
    float va_cm = 0.0f;
    float vb_cm = 0.0f;
    float vc_cm = 0.0f;
    float v_limit = 0.0f;
    float v_abs_max = 0.0f;
    float scale = 1.0f;

    // 防御式检查：母线电压异常时不上报无意义占空比
    if(NULL == svpwm_duty || bus_voltage <= 0.0f)
    {
        return;
    }

    v_max = foc_max3(va, vb, vc);
    v_min = foc_min3(va, vb, vc);

    // 标准SVPWM零序注入：减去共模分量，等效最大化线电压利用率
    v_offset = 0.5f * (v_max + v_min);

    va_cm = va - v_offset;
    vb_cm = vb - v_offset;
    vc_cm = vc - v_offset;

    // 过调制保护：当任一相超限时，整体等比例缩放，保持三相比例关系
    v_limit = 0.5f * bus_voltage;
    v_abs_max = foc_max3((va_cm > 0.0f ? va_cm : -va_cm),
                         (vb_cm > 0.0f ? vb_cm : -vb_cm),
                         (vc_cm > 0.0f ? vc_cm : -vc_cm));

    if(v_abs_max > v_limit && v_abs_max > 0.0f)
    {
        scale = v_limit / v_abs_max;
        va_cm *= scale;
        vb_cm *= scale;
        vc_cm *= scale;
    }

    svpwm_duty->duty_a = foc_limit_range(0.5f + va_cm / bus_voltage, 0.0f, 1.0f);
    svpwm_duty->duty_b = foc_limit_range(0.5f + vb_cm / bus_voltage, 0.0f, 1.0f);
    svpwm_duty->duty_c = foc_limit_range(0.5f + vc_cm / bus_voltage, 0.0f, 1.0f);
}

void foc_svpwm_to_compare(const foc_svpwm_duty_t *svpwm_duty, uint16 duty_max, uint16 compare_value[3])
{
    if(NULL == svpwm_duty || NULL == compare_value)
    {
        return;
    }

    // 中心对齐PWM可直接使用该比较值；边沿对齐PWM由上层按定时器模式调整
    compare_value[0] = (uint16)(svpwm_duty->duty_a * duty_max);
    compare_value[1] = (uint16)(svpwm_duty->duty_b * duty_max);
    compare_value[2] = (uint16)(svpwm_duty->duty_c * duty_max);
}

void foc_voltage_to_svpwm(float vd,
                          float vq,
                          float electrical_angle_deg,
                          float bus_voltage,
                          uint16 duty_max,
                          foc_svpwm_duty_t *svpwm_duty,
                          uint16 compare_value[3])
{
    foc_inv_park_result_t inv_park_result;
    foc_inv_clarke_result_t inv_clarke_result;
    float electrical_angle_rad = foc_degree_to_radian(electrical_angle_deg);

    inv_park_result = foc_inv_park_transform(vd, vq, electrical_angle_rad);

    inv_clarke_result = foc_inv_clarke_transform(inv_park_result.alpha, inv_park_result.beta);

    foc_svpwm_generate(inv_clarke_result.va,
                       inv_clarke_result.vb,
                       inv_clarke_result.vc,
                       bus_voltage,
                       svpwm_duty);

    foc_svpwm_to_compare(svpwm_duty, duty_max, compare_value);
}
