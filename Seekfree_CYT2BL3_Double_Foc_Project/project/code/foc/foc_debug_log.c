#include "foc_debug_log.h"
#include "foc_adc.h"
#include "zf_common_headfile.h"

#define FOC_DEBUG_MAX_SAMPLE_COUNT   (1000)                                          //最大采样点数

// 以下 6 组数组用于“先采样后集中发送”的离线调试模式
// 单位统一为 mA / RPM，方便上位机直接画图与比对
static int32 foc_debug_ia_points_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};               //ia电流采样缓冲区
static int32 foc_debug_ib_points_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};               //ib电流采样缓冲区
static int32 foc_debug_ic_points_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};               //ic电流采样缓冲区
static int32 foc_debug_id_points_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};               //id电流采样缓冲区
static int32 foc_debug_iq_points_ma[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};               //iq电流采样缓冲区
static int32 foc_debug_speed_points_rpm[FOC_DEBUG_MAX_SAMPLE_COUNT] = {0};            //速度采样缓冲区
/********************************************************************************************************************
/*
 * 函数名称：foc_debug_capture_and_send
 * 功能说明：
 * 1) 按给定周期采样电流/速度数据，并缓存到库内部数组
 * 2) 采样结束后发送日志头与全部采样点到 UART0
 * 3) 发送前会将左电机速度目标置 0（内部已执行停车）
 *
 * 参数说明：
 * sample_count       采样点数，单位：点。建议与业务侧 SAMPLE_COUNT 一致（默认 1000）
 * sample_interval_ms 相邻采样点间隔，单位：ms。传 0 时内部按 1ms 处理
 * target_speed_rpm   本次测试目标转速，单位：RPM。用于日志头字段 TARGET_RPM
 *
 * 调用时机：
 * - 在下发目标转速后调用，用于一次完整“采样+发送”调试流程
 * - 当前在 main_cm4.c 主流程中调用
 */
void foc_debug_capture_and_send(uint16 sample_count,
                                uint16 sample_interval_ms,
                                int32 target_speed_rpm)
{
    uint16 sample_index = 0;
    uint16 send_index = 0;
    uint16 driver_loop_elapsed_ms = 0;
    int64 bus_voltage_sum_mv = 0;
    int32 bus_voltage_avg_mv = 0;
    char uart0_tx_buffer[128];

    // 参数保护：采样点数为 0 或超出静态缓存上限时直接返回
    if((0 == sample_count) || (sample_count > FOC_DEBUG_MAX_SAMPLE_COUNT))
    {
        return;
    }

    // 采样周期保护：外部传 0 时退化为 1ms，避免无延时导致采样节拍异常
    if(0 == sample_interval_ms)
    {
        sample_interval_ms = 1;
    }

    // 第一阶段：采样并缓存（不在本循环内发送串口，减少采样过程被串口阻塞）
    for(sample_index = 0; sample_index < sample_count; sample_index ++)
    {
        // 驱动维护任务固定每10ms执行一次，降低采样期间CPU占用
        if(driver_loop_elapsed_ms >= 10)
        {
            driver_adc_loop();
            driver_loop_elapsed_ms = 0;
        }

        // 电流从 A 转换为 mA，统一使用 int32 便于串口文本输出
        foc_debug_ia_points_ma[sample_index] = (int32)(foc_current_data.motor_a.ia * 1000.0f);
        foc_debug_ib_points_ma[sample_index] = (int32)(foc_current_data.motor_a.ib * 1000.0f);
        foc_debug_ic_points_ma[sample_index] = (int32)(foc_current_data.motor_a.ic * 1000.0f);
        foc_debug_id_points_ma[sample_index] = (int32)(foc_current_data.motor_a.id * 1000.0f);
        foc_debug_iq_points_ma[sample_index] = (int32)(foc_current_data.motor_a.iq * 1000.0f);
        // 速度取滤波后的机械转速（RPM）
        foc_debug_speed_points_rpm[sample_index] = (int32)motor_left.motor_speed_filter;
        // 统计采样期间母线电压均值（单位 mV）
        bus_voltage_sum_mv += (int32)(battery_value.battery_voltage * 1000.0f);

        // 维持固定采样周期
        system_delay_ms(sample_interval_ms);
        driver_loop_elapsed_ms += sample_interval_ms;
    }

    bus_voltage_avg_mv = (int32)(bus_voltage_sum_mv / sample_count);

    // 第二阶段：采样结束后停车，避免发送期间电机继续变化影响复现实验
    motor_left_speed_ref_set(0.0f);

    // 第三阶段：先发日志头，告知上位机本次数据规模与目标转速
    sprintf(uart0_tx_buffer, "LOG,CNT:%d,TARGET_RPM:%ld\r\n",
            sample_count,
            target_speed_rpm);
    uart_write_string(UART_0, uart0_tx_buffer);

    // 第四阶段：按时序索引逐条输出，便于上位机按 T 字段重建波形
    for(send_index = 0; send_index < sample_count; send_index ++)
    {
        if(send_index == (sample_count - 1))
        {
            // 仅在最后1ms对应的数据行附带平均母线电压
            sprintf(uart0_tx_buffer,
                    "T:%d,IA:%ld,IB:%ld,IC:%ld,ID:%ld,IQ:%ld,SPD:%ld,VBUS_AVG_MV:%ld\r\n",
                    send_index,
                    foc_debug_ia_points_ma[send_index],
                    foc_debug_ib_points_ma[send_index],
                    foc_debug_ic_points_ma[send_index],
                    foc_debug_id_points_ma[send_index],
                    foc_debug_iq_points_ma[send_index],
                    foc_debug_speed_points_rpm[send_index],
                    bus_voltage_avg_mv);
        }
        else
        {
            sprintf(uart0_tx_buffer,
                    "T:%d,IA:%ld,IB:%ld,IC:%ld,ID:%ld,IQ:%ld,SPD:%ld\r\n",
                    send_index,
                    foc_debug_ia_points_ma[send_index],
                    foc_debug_ib_points_ma[send_index],
                    foc_debug_ic_points_ma[send_index],
                    foc_debug_id_points_ma[send_index],
                    foc_debug_iq_points_ma[send_index],
                    foc_debug_speed_points_rpm[send_index]);
        }
        uart_write_string(UART_0, uart0_tx_buffer);
    }
}