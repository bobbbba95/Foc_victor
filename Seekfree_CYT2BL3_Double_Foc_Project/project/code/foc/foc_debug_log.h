#ifndef _FOC_DEBUG_LOG_H_
#define _FOC_DEBUG_LOG_H_

#include "zf_common_typedef.h"

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
                                int32 target_speed_rpm);

#endif