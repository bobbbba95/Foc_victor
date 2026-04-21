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
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "foc_adc.h"
#include "foc_debug_log.h"

#define SAMPLE_COUNT             (1000)      //采样点数，不允许超过1K
#define TARGET_SPEED_RPS         (5.0f)      // 目标转速：r/s，改这里即可
#define TARGET_SPEED_RPM         (TARGET_SPEED_RPS * 60.0f)
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************

int main(void)
{
    uint16 voltage_send_elapsed_ms = 0;
    char uart0_tx_buffer[64];

    clock_init(SYSTEM_CLOCK_160M);                                              // 时钟配置及系统初始化<务必保留>
    
    debug_init();                                                               // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    interrupt_global_disable();							                         // 关闭全局中断
    
    driver_gpio_init();                                                         // 板载普通GPIO功能初始化（按键、LED）
    
    driver_adc_init();                                                          // 电池检测初始化

    motor_flash_init();                                                         // FLASH初始化 读取零点、旋转方向、极对数数据  
    
    motor_control_init();                                                       // 双电机控制初始化    
    
    interrupt_global_enable(0);							                            // 开启全局中断
    
    //motor_left_torque_ref_set(1.5f);     // 左侧电机力矩参考设置为0.5A 右侧电机在ISR中设置了速度参考 因此这里不设置力矩参考 直接进入速度闭环控制路径
    //目前采样数据为Ia,ib,ic,id,iq和速度，单位分别为mA和RPM，方便上位机直接画图与比对
    //motor_right_torque_ref_set(1.5f);    
    motor_right_speed_ref_set(500);
    motor_left_speed_ref_set(500);

    foc_debug_capture_and_send(SAMPLE_COUNT, 1, (int32)TARGET_SPEED_RPM);       

    for(;;)
    {
        driver_adc_loop();                                                      // 驱动 ADC 循环检测函数
        driver_gpio_loop();                                                     // 驱动 GPIO 循环检测函数
        driver_cmd_loop();                                                      // 驱动 控制指令 循环响应函数
        // 每100ms通过UART0输出一次电池电压和原始ADC采样值
        voltage_send_elapsed_ms += DRIVER_RESPONSE_CYCLE;
        if(voltage_send_elapsed_ms >= 100)
        {
            voltage_send_elapsed_ms = 0;
            sprintf(uart0_tx_buffer,
                    "VBAT:%.2fV, ADC_RAW:%u\r\n",
                    battery_value.battery_voltage,
                    battery_value.battery_adc_raw);
            uart_write_string(UART_0, uart0_tx_buffer);
        }

        system_delay_ms(DRIVER_RESPONSE_CYCLE);                                 // 主循环延时
    }
}

// **************************** 代码区域 ****************************
