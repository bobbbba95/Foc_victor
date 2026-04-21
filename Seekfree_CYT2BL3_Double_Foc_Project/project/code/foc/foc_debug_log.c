#include "foc_debug_log.h"
#include "foc_adc.h"
#include "zf_common_headfile.h"

#define FOC_DEBUG_MAX_SAMPLE_COUNT   (1000)                                          //最大采样点数

// 增加左右电机的采样缓冲区
// 为了节省内存，主要采集 Id, Iq 和 Speed，这三个参数最能反映 FOC 控制状态
static int32 foc_debug_l_id_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};
static int32 foc_debug_l_iq_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};
static int32 foc_debug_l_spd_rpm[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};

static int32 foc_debug_r_id_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};
static int32 foc_debug_r_iq_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};
static int32 foc_debug_r_spd_rpm[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};

void foc_debug_capture_and_send(uint16 sample_count,
                                uint16 sample_interval_ms,
                                int32 target_speed_rpm)
{
    uint16 sample_index = 0;
    uint16 send_index = 0;
    uint16 driver_loop_elapsed_ms = 0;
    int64 bus_voltage_sum_mv = 0;
    int32 bus_voltage_avg_mv = 0;
    char uart0_tx_buffer[160]; // 增加缓冲区长度以容纳更多数据

    if((0 == sample_count) || (sample_count > FOC_DEBUG_MAX_SAMPLE_COUNT))
    {
        return;
    }

    if(0 == sample_interval_ms)
    {
        sample_interval_ms = 1;
    }

    // 第一阶段：采样阶段
    for(sample_index = 0; sample_index < sample_count; sample_index ++)
    {
        if(driver_loop_elapsed_ms >= 10)
        {
            driver_adc_loop();
            driver_loop_elapsed_ms = 0;
        }

        // 采集左电机 (Motor A) 数据
        foc_debug_l_id_ma[sample_index] = (int32)(foc_current_data.motor_a.id * 1000.0f);
        foc_debug_l_iq_ma[sample_index] = (int32)(foc_current_data.motor_a.iq * 1000.0f);
        foc_debug_l_spd_rpm[sample_index] = (int32)motor_left.motor_speed_filter;

        // 采集右电机 (Motor B) 数据
        foc_debug_r_id_ma[sample_index] = (int32)(foc_current_data.motor_b.id * 1000.0f);
        foc_debug_r_iq_ma[sample_index] = (int32)(foc_current_data.motor_b.iq * 1000.0f);
        foc_debug_r_spd_rpm[sample_index] = (int32)motor_right.motor_speed_filter;

        bus_voltage_sum_mv += (int32)(battery_value.battery_voltage * 1000.0f);

        system_delay_ms(sample_interval_ms);
        driver_loop_elapsed_ms += sample_interval_ms;
    }

    bus_voltage_avg_mv = (int32)(bus_voltage_sum_mv / sample_count);

    // 第二阶段：采样结束，停止所有电机
    motor_right_speed_ref_set(0);
    motor_left_speed_ref_set(0);

    // 第三阶段：发送日志头
    sprintf(uart0_tx_buffer, "LOG,CNT:%d,TARGET_RPM:%ld,TYPE:DUAL_MOTOR\r\n", 
            sample_count, target_speed_rpm);
    uart_write_string(UART_0, uart0_tx_buffer);

    // 第四阶段：逐行发送数据
    for(send_index = 0; send_index < sample_count; send_index ++)
    {
        // 格式：T, 左电机Id, 左电机Iq, 左速度, 右电机Id, 右电机Iq, 右速度, [母线电压]
        sprintf(uart0_tx_buffer,
                "T:%d,L_ID:%ld,L_IQ:%ld,L_SPD:%ld,R_ID:%ld,R_IQ:%ld,R_SPD:%ld",
                send_index,
                foc_debug_l_id_ma[send_index],
                foc_debug_l_iq_ma[send_index],
                foc_debug_l_spd_rpm[send_index],
                foc_debug_r_id_ma[send_index],
                foc_debug_r_iq_ma[send_index],
                foc_debug_r_spd_rpm[send_index]);
        
        uart_write_string(UART_0, uart0_tx_buffer);

        if(send_index == (sample_count - 1))
        {
            sprintf(uart0_tx_buffer, ",VBUS:%ld\r\n", bus_voltage_avg_mv);
        }
        else
        {
            sprintf(uart0_tx_buffer, "\r\n");
        }
        uart_write_string(UART_0, uart0_tx_buffer);
    }
}