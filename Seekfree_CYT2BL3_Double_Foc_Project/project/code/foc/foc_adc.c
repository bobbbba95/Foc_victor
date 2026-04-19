#include "foc_adc.h"
#include "foc_transform.h"
#include "zf_common_headfile.h"
// 仅保留两路相电流采样：A相(P18.1) + C相(P12.1)
#define FOC_ADC_PHASE_A_LEFT_CH (ADC2_CH01_P18_1)
#define FOC_ADC_PHASE_C_LEFT_CH (ADC1_CH05_P12_1)
// 右电机两路相电流采样：A相(P12.0) + C相(P6.5)
#define FOC_ADC_PHASE_A_RIGHT_CH (ADC1_CH04_P12_0)
#define FOC_ADC_PHASE_C_RIGHT_CH (ADC0_CH05_P06_5)
// 变量简介    全局电流采样与dq变换结果
// 备注信息    在中断与控制环中被共同访问，使用 volatile 防止编译器优化误判
// 全局电流采样与dq电流结果
volatile foc_current_data_t foc_current_data;
// 函数简介     FOC电流采样ADC初始化
// 传入参数     void
// 返回参数     void
// 使用示例     foc_current_adc_init();
// 备注信息     初始化A/C两路ADC并计算电流换算系数
void foc_current_adc_init(void)
{
    // 先构建三角函数查找表，避免控制环首周期触发建表
    foc_trig_lut_init();

    // 仅初始化A/C两路电流采样ADC，分辨率统一12bit
    adc_init(FOC_ADC_PHASE_A_LEFT_CH, ADC_12BIT);
    adc_init(FOC_ADC_PHASE_C_LEFT_CH, ADC_12BIT);
    adc_init(FOC_ADC_PHASE_A_RIGHT_CH, ADC_12BIT);
    adc_init(FOC_ADC_PHASE_C_RIGHT_CH, ADC_12BIT);

    // 使用12bit中点作为固定偏置，不进行动态零漂校准
    foc_clear_group(&foc_current_data.motor_a);
    foc_clear_group(&foc_current_data.motor_b);
}
// 函数简介     左电机采样中断服务函数
// 传入参数     void
// 返回参数     void
// 使用示例     在PWM中断调用 foc_current_adc_sample_left_isr();
// 备注信息     完成一次A/C相采样与B相重构
void foc_current_adc_sample_left_isr(void)
{
    foc_sample_left_ac_only(&foc_current_data.motor_a);
}

// 函数简介     右电机采样中断服务函数
// 传入参数     void
// 返回参数     void
// 使用示例     在PWM中断调用 foc_current_adc_sample_right_isr();
// 备注信息     当前右路无硬件采样，数据固定置零
void foc_current_adc_sample_right_isr(void)
{
    foc_sample_right_ac_only(&foc_current_data.motor_b);
}
// 函数简介     计算左电机电角度(度)
// 传入参数     encoder_now         当前机械编码器值
//              zero_location       机械零点偏移
//              pole_pairs          电机极对数
//              rotation_direction  旋转方向(1/-1)
//              traction_angle      牵引角(兼容参数)
// 返回参数     float               单电周期电角度[0,360)
// 使用示例     angle_deg = foc_calc_left_electrical_angle_deg(enc, zero, pp, dir, angle);
// 备注信息     先做方向修正与零点修正，再映射到单电周期
float foc_calc_left_electrical_angle_deg(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle)
{
    (void)traction_angle;
    int32 encoder_now_temp = encoder_now;
    int32 encoder_max = ENCODER_PRECISION;
    int32 encoder_temp = 0;
    int32 encoder_per_electrical = 0;

    if(pole_pairs <= 0)
    {
        return 0.0f;
    }

    encoder_per_electrical = encoder_max / pole_pairs;
    if(encoder_per_electrical <= 0)
    {
        return 0.0f;
    }

    if(rotation_direction == -1)
    {
        encoder_now_temp = encoder_max - encoder_now_temp;
    }

    encoder_temp = encoder_now_temp - zero_location;

    while(encoder_temp < 0)
    {
        encoder_temp += encoder_max;
    }
    while(encoder_temp >= encoder_max)
    {
        encoder_temp -= encoder_max;
    }

    encoder_temp = encoder_temp % encoder_per_electrical;
    return (float)encoder_temp * 360.0f / (float)encoder_per_electrical;
}
// 函数简介     计算右电机电角度(度)
// 传入参数     encoder_now         当前机械编码器值
//              zero_location       机械零点偏移
//              pole_pairs          电机极对数
//              rotation_direction  旋转方向(1/-1)
//              traction_angle      牵引角(兼容参数)
// 返回参数     float               单电周期电角度[0,360)
// 使用示例     angle_deg = foc_calc_right_electrical_angle_deg(enc, zero, pp, dir, angle);
// 备注信息     先做方向修正与零点修正，再映射到单电周期
float foc_calc_right_electrical_angle_deg(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle)
{
    return foc_calc_left_electrical_angle_deg(encoder_now, zero_location, pole_pairs, rotation_direction, traction_angle);
}
// 函数简介     左电机dq电流更新
// 传入参数     encoder_now         当前编码器值
//              zero_location       电角度零点
//              pole_pairs          极对数
//              rotation_direction  旋转方向(1/-1)
//              traction_angle      牵引角(兼容参数)
// 返回参数     void
// 使用示例     foc_current_dq_update_left(enc, zero, pp, dir, angle);
// 备注信息     内部流程: 电角度计算 -> Clarke -> Park -> 写回id/iq
void foc_current_dq_update_left(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle)
{
    float electrical_angle_deg = 0.0f;
    float electrical_angle_rad = 0.0f;
    foc_clarke_result_t clarke_result;
    foc_park_result_t park_result;

    electrical_angle_deg = foc_calc_left_electrical_angle_deg(encoder_now, zero_location, pole_pairs, rotation_direction, traction_angle);
    electrical_angle_rad = foc_degree_to_radian(electrical_angle_deg);

    clarke_result = foc_clarke_transform(foc_current_data.motor_a.ia,
                                         foc_current_data.motor_a.ib,
                                         foc_current_data.motor_a.ic);

    park_result = foc_park_transform(clarke_result.alpha,
                                     clarke_result.beta,
                                     electrical_angle_rad);

    foc_current_data.motor_a.id = park_result.id;
    foc_current_data.motor_a.iq = park_result.iq;
}
// 函数简介     右电机dq电流更新
// 传入参数     encoder_now         当前编码器值
//              zero_location       电角度零点
//              pole_pairs          极对数
//              rotation_direction  旋转方向(1/-1)
//              traction_angle      牵引角(兼容参数)
// 返回参数     void
// 使用示例     foc_current_dq_update_right(enc, zero, pp, dir, angle);
// 备注信息     内部流程: 电角度计算 -> Clarke -> Park -> 写回id/iq
void foc_current_dq_update_right(int32 encoder_now, int16 zero_location, int16 pole_pairs, int16 rotation_direction, int32 traction_angle)
{
    float electrical_angle_deg = 0.0f;
    float electrical_angle_rad = 0.0f;
    foc_clarke_result_t clarke_result;
    foc_park_result_t park_result;

    electrical_angle_deg = foc_calc_right_electrical_angle_deg(encoder_now, zero_location, pole_pairs, rotation_direction, traction_angle);
    electrical_angle_rad = foc_degree_to_radian(electrical_angle_deg);

    clarke_result = foc_clarke_transform(foc_current_data.motor_b.ia,
                                         foc_current_data.motor_b.ib,
                                         foc_current_data.motor_b.ic);

    park_result = foc_park_transform(clarke_result.alpha,
                                     clarke_result.beta,
                                     electrical_angle_rad);

    foc_current_data.motor_b.id = park_result.id;
    foc_current_data.motor_b.iq = park_result.iq;
}




// 函数简介     电流采样偏置校准
// 传入参数     sample_count  期望校准样本数(当前未使用)
// 返回参数     void
// 使用示例     foc_current_adc_calibrate(FOC_CURRENT_CALIB_SAMPLES);
// 备注信息     当前策略为固定中点偏置，保留接口便于后续切换动态标定
void foc_current_adc_calibrate(uint16 sample_count)
{
    // 按需求关闭零漂标定：统一使用12bit中点固定偏置
    (void)sample_count;

    foc_current_data.motor_a.offset_via = 2048;
    foc_current_data.motor_a.offset_vic = 2048;
    foc_current_data.motor_a.offset_vib = 2048;

    foc_current_data.motor_b.offset_via = 2048;
    foc_current_data.motor_b.offset_vic = 2048;
    foc_current_data.motor_b.offset_vib = 2048;
}
// 函数简介     ADC原始值转换电流值
// 传入参数     raw      ADC原始采样值
//             offset   ADC零点偏置
// 返回参数     float    电流值(A)
// 使用示例     ia = foc_raw_to_current(raw_ia, offset_ia);
// 备注信息     使用线性比例换算，比例系数由 foc_current_adc_init 计算
static float foc_raw_to_current(uint16 raw, int16 offset)
{
    int32 delta = (int32)raw - (int32)offset;
    //原始采样值乘上一个ADC计数对应的电压值，再除以采样电阻和放大倍数，即可得到电流值
    return (float)delta * FOC_CURRENT_PER_COUNT;
}
// 函数简介     左电机两电阻采样(A/C相)并重构B相
// 传入参数     group    目标电流数据结构指针
// 返回参数     void
// 使用示例     foc_sample_left_ac_only(&foc_current_data.motor_a);
// 备注信息     基于 ia + ib + ic = 0 计算 ib
static void foc_sample_left_ac_only(volatile foc_current_group_t *group)
{
    group->raw_via = adc_convert(FOC_ADC_PHASE_A_LEFT_CH);
    group->raw_vic = adc_convert(FOC_ADC_PHASE_C_LEFT_CH);
    group->raw_vib = 0;

    group->ia = foc_raw_to_current(group->raw_via, group->offset_via);
    group->ic = foc_raw_to_current(group->raw_vic, group->offset_vic);
    // 两电阻采样重构B相电流：ia + ib + ic = 0
    group->ib = -(group->ia + group->ic);
}
// 函数简介     清零并初始化单电机电流数据结构
// 传入参数     group    目标电流数据结构指针
// 返回参数     void
// 使用示例     foc_clear_group(&foc_current_data.motor_a);
// 备注信息     偏置默认置为12bit中点(2048)
static void foc_clear_group(volatile foc_current_group_t *group)
{
    group->raw_via = 0;
    group->raw_vib = 0;
    group->raw_vic = 0;

    group->offset_via = 2048;  // ADC 12bit 中点作为偏置
    group->offset_vib = 2048;
    group->offset_vic = 2048;

    group->ia = 0.0f;
    group->ib = 0.0f;
    group->ic = 0.0f;
    group->id = 0.0f;
    group->iq = 0.0f;
}
// 函数简介     右电机两电阻采样(A/C相)并重构B相
// 传入参数     group    目标电流数据结构指针
// 返回参数     void
// 使用示例     foc_sample_right_ac_only(&foc_current_data.motor_b);
// 备注信息     基于 ia + ib + ic = 0 计算 ib
static void foc_sample_right_ac_only(volatile foc_current_group_t *group)
{
    group->raw_via = adc_convert(FOC_ADC_PHASE_A_RIGHT_CH);
    group->raw_vic = adc_convert(FOC_ADC_PHASE_C_RIGHT_CH);
    group->raw_vib = 0;

    group->ia = foc_raw_to_current(group->raw_via, group->offset_via);
    group->ic = foc_raw_to_current(group->raw_vic, group->offset_vic);
    // 两电阻采样重构B相电流：ia + ib + ic = 0
    group->ib = -(group->ia + group->ic);
}