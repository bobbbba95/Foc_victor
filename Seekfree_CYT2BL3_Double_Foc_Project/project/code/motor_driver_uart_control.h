/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          motor_driver_uart_control
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2025-01-03       pudding            first version
********************************************************************************************************************/

#ifndef _motor_driver_uart_control_h_
#define _motor_driver_uart_control_h_


#include "zf_common_headfile.h"


#define MOTOR_DRIVER_UART      UART_2
#define MOTOR_DRIVER_TX        UART2_RX_P07_0 
#define MOTOR_DRIVER_RX        UART2_TX_P07_1 

typedef enum
{
    NULL_CMD,
    SET_DUTY,
    GET_SPEED,
    SET_ZERO,
    SET_ANGLE_ZERO,
    TEST_PHASE,
    GET_PHASE,
    SET_PHASE,
    GET_ENCODER,
    GET_ANGLE,
    GET_RDT_ANGLE,
    GET_PHASE_DUTY,
    GET_VOLTAGE,
    
    // ===== FOC 相关命令 =====
    SET_FOC_TORQUE_REF,     // 设置FOC力矩参考（主板下发设 Iq 目标，0xA5 0x09）
    GET_FOC_IQ,             // 持续上报 Iq 电流（字符串 GET-IQ 触发 / 上报 0xA5 0x07）
    GET_UQ,                 // 持续上报 Uq（请求 0xA5 0x08 / 上报 0xA5 0x08）

    ERROR_CMD
}control_mode_enum;

typedef enum
{
    BYTE_TYPE,
    STRING_TYPE
}control_cmd_type_enum;

typedef enum
{
    CMD_NULL            = 0,    // 无命令
    CMD_FORTHWITH       = 1,    // 即时命令 解析完成后马上响应
    CMD_LOOP            = 2,    // 循环命令 周期调用的查询响应
}refresh_state_enum;

typedef struct
{
      uint8                     send_data_buffer[7];
      
      uint8                     select_motor;
      
      int16                     receive_left_phase;
      int16                     receive_left_duty_data;

      int16                     receive_right_phase;
      int16                     receive_right_duty_data;
      
      // ====== 补充 FOC 目标参数 ======
      float                     left_id_target;
      float                     left_iq_target;
      float                     right_id_target;
      float                     right_iq_target;

      control_mode_enum         immediate_command;
      
      control_mode_enum         continue_command;
      
      refresh_state_enum        refresh_flag;
      
      control_cmd_type_enum     cmd_type;
}small_device_value_struct;

extern small_device_value_struct motor_value;

// 调试计数器：在 motor_driver_uart_control.c 中定义，
// 每解析成功一次对应功能字就 +1，可用于 UART0 调试打印或 IAR Watch
extern volatile uint32 cmd_07_recv_count;       // 0x09 下行设 Iq 解析次数（变量名保留以兼容旧调试代码）

void motor_driver_control_callback(small_device_value_struct *device_value);

void motor_driver_control_loop(small_device_value_struct *device_value);

void motor_driver_set_duty(small_device_value_struct *device_value, int left_duty, int right_duty);

void motor_driver_send_speed(small_device_value_struct *device_value, int left_speed, int right_speed);

void motor_driver_send_angle(small_device_value_struct *device_value, float left_angle, float right_angle);

void motor_driver_send_reduction_angle(small_device_value_struct *device_value, float left_angle, float right_angle);

//-------------------------------------------------------------------------------------------------------------------
// ===== FOC 通信函数 =====
//-------------------------------------------------------------------------------------------------------------------

/**
 * @brief 发送 Iq 电流（左右电机），7 字节小包
 * @note  帧格式: 0xA5 0x07 LeftIq_H LeftIq_L RightIq_H RightIq_L SUM
 *        缩放: uint16 = (int16)(iq * 100) + 3200, 量程 ±32A, 精度 0.01A
 */
void motor_driver_send_iq(small_device_value_struct *device_value, float left_iq, float right_iq);

/**
 * @brief 发送 Uq（左右电机 q 轴电压），7 字节小包
 * @note  帧格式: 0xA5 0x08 LeftUq_H LeftUq_L RightUq_H RightUq_L SUM
 *        缩放: int16 = uq * 1000, 量程 ±32.767V, 精度 0.001V
 */
void motor_driver_send_uq(small_device_value_struct *device_value, float left_uq, float right_uq);

void motor_driver_uart_init(void);

#endif
