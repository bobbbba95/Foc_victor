#ifndef _FOC_ADC_H_
#define _FOC_ADC_H_

#include "zf_common_typedef.h"

//**************************** 参数配置 ****************************
// 采样电阻10mR，运放增益20倍，12bit ADC默认3.3V参考
// 若你的硬件使用其它分流电阻，请按实际值修改该宏（单位：欧姆）
#define FOC_CURRENT_SHUNT_OHM      (0.005f)           //采样电阻阻值
#define FOC_CURRENT_AMP_GAIN       (20.0f)           //采样电流放大倍数
#define FOC_ADC_VREF               (3.3f)            //ADC参考电压
#define FOC_ADC_FULL_SCALE         (4095.0f)         //ADC满量程对应的计数值（12bit为4095）
#define FOC_CURRENT_CALIB_SAMPLES  (512)             //电流采样零点标定时的期望采样点数（当前未使用，保留接口兼容上层调用）
#define FOC_CURRENT_PER_COUNT      ((FOC_ADC_VREF / FOC_ADC_FULL_SCALE) / (FOC_CURRENT_SHUNT_OHM * FOC_CURRENT_AMP_GAIN))
//**************************** 参数配置 ****************************
// 结构体简介  单电机相电流与dq电流数据
// 成员说明    raw_vix: 原始ADC值   offset_vix: 零点偏置
//            ia/ib/ic: 三相电流(A)  id/iq: dq轴电流(A)
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

// 结构体简介  双电机电流数据总表
// 成员说明    motor_a: 左电机电流数据   motor_b: 右电机电流数据
typedef struct
{
    foc_current_group_t motor_a;
    foc_current_group_t motor_b;
}foc_current_data_t;

// 变量简介    全局电流采样与dq变换结果
// 备注信息    在 foc_adc.c 中定义，其他模块通过 extern 访问
extern volatile foc_current_data_t foc_current_data;


// 函数简介    FOC电流采样ADC初始化
// 传入参数    void
// 返回参数    void
// 使用示例    foc_current_adc_init();
// 备注信息    初始化相电流ADC通道并计算ADC计数到电流的换算系数
void foc_current_adc_init(void);

// 函数简介    电流采样零点校准（当前实现为固定中点偏置）
// 传入参数    sample_count 期望校准采样次数
// 返回参数    void
// 使用示例    foc_current_adc_calibrate(FOC_CURRENT_CALIB_SAMPLES);
// 备注信息    当前方案不做动态校准，仅保留接口兼容上层调用
void foc_current_adc_calibrate(uint16 sample_count);

// 函数简介    左电机电流采样中断服务入口
// 传入参数    void
// 返回参数    void
// 使用示例    在PWM/定时中断中调用 foc_current_adc_sample_left_isr();
// 备注信息    完成A/C相采样并重构B相
void foc_current_adc_sample_left_isr(void);

// 函数简介    右电机电流采样中断服务入口
// 传入参数    void
// 返回参数    void
// 使用示例    在PWM/定时中断中调用 foc_current_adc_sample_right_isr();
// 备注信息    当前硬件右电机无电流采样，接口内部返回0值
void foc_current_adc_sample_right_isr(void);

// 函数简介    左电机dq轴电流更新
// 传入参数    encoder_now         当前编码器值
//            zero_location       电角度零点位置
//            pole_pairs          极对数
//            rotation_direction  旋转方向(1/-1)
//            traction_angle      牵引角(兼容参数)
// 返回参数    void
// 使用示例    foc_current_dq_update_left(enc, zero, pp, dir, angle);
// 备注信息    内部执行 Clarke + Park 变换，结果写入 foc_current_data.motor_a.id/iq
void foc_current_dq_update_left(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle);

// 函数简介    计算左电机电角度(度)
// 传入参数    encoder_now         当前编码器值
//            zero_location       电角度零点位置
//            pole_pairs          极对数
//            rotation_direction  旋转方向(1/-1)
//            traction_angle      牵引角(兼容参数)
// 返回参数    电角度(单位: 度)
// 使用示例    angle_deg = foc_calc_left_electrical_angle_deg(enc, zero, pp, dir, angle);
// 备注信息    返回范围为单电周期 [0, 360)
float foc_calc_left_electrical_angle_deg(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle);
// 函数简介     ADC原始值转换电流值
// 传入参数     raw      ADC原始采样值
//             offset   ADC零点偏置
// 返回参数     float    电流值(A)
// 使用示例     ia = foc_raw_to_current(raw_ia, offset_ia);
// 备注信息     使用线性比例换算，比例系数由 foc_current_adc_init 计算
static float foc_raw_to_current(uint16 raw, int16 offset);
// 函数简介     清零并初始化单电机电流数据结构
// 传入参数     group    目标电流数据结构指针
// 返回参数     void
// 使用示例     foc_clear_group(&foc_current_data.motor_a);
// 备注信息     偏置默认置为12bit中点(2048)
static void foc_clear_group(volatile foc_current_group_t *group);
// 函数简介     左电机两电阻采样(A/C相)并重构B相
// 传入参数     group    目标电流数据结构指针
// 返回参数     void
// 使用示例     foc_sample_left_ac_only(&foc_current_data.motor_a);
// 备注信息     基于 ia + ib + ic = 0 计算 ib
static void foc_sample_left_ac_only(volatile foc_current_group_t *group);
#endif
