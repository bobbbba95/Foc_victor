#ifndef _FOC_SVPWM_H_
#define _FOC_SVPWM_H_

#include "zf_common_typedef.h"
#include "foc_transform.h"

typedef struct
{
    float duty_a;     // A相占空比，范围[0,1]
    float duty_b;     // B相占空比，范围[0,1]
    float duty_c;     // C相占空比，范围[0,1]
}foc_svpwm_duty_t;

// 输入三相电压参考值（单位V）和母线电压（单位V），输出SVPWM占空比。
// 注意：本函数内部已做零序注入和过调制缩放。
void foc_svpwm_generate(float va, float vb, float vc, float bus_voltage, foc_svpwm_duty_t *svpwm_duty);

// 将占空比映射为定时器比较值 compare[0]=A, compare[1]=B, compare[2]=C。
void foc_svpwm_to_compare(const foc_svpwm_duty_t *svpwm_duty, uint16 duty_max, uint16 compare_value[3]);

// 一步FOC电压调制链路：vd/vq -> 反Park -> 反Clarke -> SVPWM -> compare
// 不依赖中断，可在上层控制任务中直接调用。
void foc_voltage_to_svpwm(float vd,
                          float vq,
                          float electrical_angle_deg,
                          float bus_voltage,
                          uint16 duty_max,
                          foc_svpwm_duty_t *svpwm_duty,
                          uint16 compare_value[3]);

#endif
