#ifndef _FOC_ADC_H_
#define _FOC_ADC_H_

#include "zf_common_typedef.h"

// 采样电阻10mR，运放增益20倍，12bit ADC默认3.3V参考
// 若你的硬件使用其它分流电阻，请按实际值修改该宏（单位：欧姆）
#define FOC_CURRENT_SHUNT_OHM      (0.01f)
#define FOC_CURRENT_AMP_GAIN       (20.0f)
#define FOC_ADC_VREF               (3.3f)
#define FOC_ADC_FULL_SCALE         (4095.0f)
#define FOC_CURRENT_CALIB_SAMPLES  (512)

typedef struct
{
    uint16 raw_via;
    uint16 raw_vib;
    uint16 raw_vic;

    int16 offset_via;
    int16 offset_vib;
    int16 offset_vic;

    float ia;
    float ib;
    float ic;

    float id;
    float iq;
}foc_current_group_t;

typedef struct
{
    foc_current_group_t motor_a;
    foc_current_group_t motor_b;
}foc_current_data_t;

extern volatile foc_current_data_t foc_current_data;

void foc_current_adc_init(void);
void foc_current_adc_calibrate(uint16 sample_count);
void foc_current_adc_sample_left_isr(void);
void foc_current_adc_sample_right_isr(void);
void foc_current_dq_update_left(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle);
float foc_calc_left_electrical_angle_deg(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle);

#endif
