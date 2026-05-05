
#include "motor_driver_uart_control.h"
#include "driver_gpio.h"        // 为了 BOARD_LED 宏

small_device_value_struct motor_value;

fifo_struct     motor_driver_fifo;              // 串口通讯的 FIFO 结构体

uint8           driver_fifo_buffer[128];        // FIFO 指向的缓冲数组

uint8           read_buffer[128];               // 解析时，读取数据的缓冲数组

int16           receive_enter_flag = 0;         // 回车标志位

char uart0_tx_buffer[128];

// ===== 调试计数器：可在 IAR Watch 里看，用来验证命令是否被解析 =====
volatile uint32 cmd_07_recv_count = 0;          // 成功解析 0x09（主板下发 Iq 设定）的次数，变量名保留以兼容旧调试代码



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口通讯 FIFO 擦除指定长度的数据
// 参数说明     clear_length        要擦除的长度
// 返回参数     void
// 使用示例     motor_driver_fifo_clear(1); 
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_fifo_clear(uint32 clear_length)
{
    fifo_read_buffer(&motor_driver_fifo, read_buffer, &clear_length, FIFO_READ_AND_CLEAN);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口通讯 数据包解析函数
// 参数说明     device_value        通讯参数结构体
// 参数说明     statement_buffer    一整包数据
// 返回参数     void
// 使用示例     motor_driver_control_loop(&motor_value, read_buffer); 
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_parse_statement(small_device_value_struct *device_value, uint8 *statement_buffer)
{
    if(BYTE_TYPE == device_value->cmd_type)                                     // 判断待解析内容类型是否为 字节通讯 类型
    {
        switch(statement_buffer[1])                                             // 判断 字节通讯 的功能字 
        {
            case 0x01:{                                                         // 功能字 0x01 设置占空比
  
                device_value->receive_left_duty_data   = (((int)statement_buffer[2] << 8) | (int)statement_buffer[3]);  // 左侧电机占空比拟合
                
                device_value->receive_right_duty_data  = (((int)statement_buffer[4] << 8) | (int)statement_buffer[5]);  // 右侧电机占空比拟合
                
                device_value->immediate_command        = SET_DUTY;              // 即时指令(仅响应一次)状态更新为 SET_DUTY 
                
                device_value->refresh_flag             = CMD_FORTHWITH;         // 即时指令刷新标志位修正  需要马上执行命令
            }break;
            
            case 0x02:{                                                         // 功能字 0x02 获取到速度信息
              
                device_value->continue_command         = GET_SPEED;             // 持续指令(串口持续输出)状态更新为 GET_SPEED 
                
            }break;
            
            case 0x03:{                                                         // 功能字 0x03 自矫正零点
              
                device_value->immediate_command        = SET_ZERO;              // 即时指令(仅响应一次)状态更新为 SET_ZERO 
                
            }break;
            
            case 0x04:{                                                         // 功能字 0x04 获取电机角度

                device_value->continue_command         = GET_ANGLE;             // 持续指令(串口持续输出)状态更新为 GET_ANGLE 
            }break;
            
            case 0x05:{                                                         // 功能字 0x05 获取电机减速后的角度

                device_value->continue_command         = GET_RDT_ANGLE;         // 持续指令(串口持续输出)状态更新为 GET_RDT_ANGLE 
            }break;
            
            case 0x06:{                                                         // 功能字 0x06 设置角度零点(当前角度作为 0 角度)

                device_value->immediate_command        = SET_ANGLE_ZERO;        // 即时指令(仅响应一次)状态更新为 SET_ANGLE_ZERO 
                
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行
            }break;

            // 功能字 0x07：上行专用（MCU→主板上报 Iq），驱动板不应在 RX 看到此功能字
            // 若误收到，直接忽略，留校验/帧长走默认逻辑

            case 0x08:{ 
                // 7 字节包：0xA5 0x08 LeftUq_H LeftUq_L RightUq_H RightUq_L SUM
                device_value->continue_command = GET_UQ; 
            }break;

            case 0x09: {
                // 7 字节包：0xA5 0x09 LeftIq_H LeftIq_L RightIq_H RightIq_L SUM
                // 主板→驱动板 下行电流控制指令：设置左右电机 Iq 目标
                // 反算：iq = ((uint16)bytes - 3200) / 100，量程 ±32A，精度 0.01A，目标 Id 固定为 0
                int16 l_iq = (int16)(((uint16)statement_buffer[2] << 8) | statement_buffer[3]);
                int16 r_iq = (int16)(((uint16)statement_buffer[4] << 8) | statement_buffer[5]);

                device_value->left_id_target  = 0.0f;
                device_value->right_id_target = 0.0f;
                device_value->left_iq_target  = (float)(l_iq - 3200) / 100.0f;
                device_value->right_iq_target = (float)(r_iq - 3200) / 100.0f;

                device_value->immediate_command = SET_FOC_TORQUE_REF;
                device_value->refresh_flag      = CMD_FORTHWITH;
                cmd_07_recv_count ++;           // 调试计数：记录 0x09 下行成功解析次数
            } break;
  

            // 功能字 0x10: 紧急停止/使能指令 (新增)
            case 0x10: {
                if(statement_buffer[2] == 0x01) {
                    // 使能电机
                } else {
                    // 强制进入所有 PWM 占空比为 0
                }
            } break;

            default:break;
        }
    }
    else if(STRING_TYPE == device_value->cmd_type)                              // 判断待解析内容类型是否为 字符通讯 类型
    {
        char *location1 = NULL;                                                 // 定义两个位置指针 用于解析数据
        char *location2 = NULL;
        
        char data_string[10];                                                   // 定义一个缓存数组 用于 字符串 转 数据
        
        if(statement_buffer[0] == 'S')                                          // 判断 字符 指令的首字符是否为 'S'
        {
            if(strstr((const char *)statement_buffer, "SET-DUTY"))             // 搜索 待解析数组内是否有 "SET-DUTY" 关键词  
            {                                                                   // SET-DUTY指令格式为："SET-DUTY,num_1,num_2\r\n" 其中 num_1 为左侧电机占空比， num_2 为右侧电机占空比 范围限制 -10000 ~ 10000
              
                location1 = strchr((char *)statement_buffer, ',');              // 搜索第一个逗号的位置 
                
                if(location1 != 0x00)                                           // 如果成功搜索到 则继续  否则默认将占空比设置为 0
                {
                    location1 ++;                                               // 位置自增 越过第一个逗号
                    
                    location2 = strchr(location1, ',');                         // 搜索第二个逗号的位置
                    
                    if(location2 != 0x00)                                       // 如果成功搜索到 则继续
                    {
                        memset(data_string, 0x00, 10);                          // 清空缓存数组
                      
                        memcpy(data_string, location1, location2 - location1);  // 取出两个逗号之间的数据
                        
                        device_value->receive_left_duty_data = func_str_to_int(data_string);    // 将字符串数据转为整形 第一个数据为左侧电机占空比
                        
                        device_value->receive_left_duty_data = func_limit_ab(device_value->receive_left_duty_data, -10000, 10000);  // 数据限幅
                        
                        location2 ++;                                           // 位置自增 越过第二个逗号
                        
                        location1 = strchr(location2, '\n');                    // 搜索回车的位置
                        
                        memset(data_string, 0x00, 10);                          // 清空缓存数组
                        
                        memcpy(data_string, location2, location1 - location2);  // 取出第二个逗号到回车之间的数据
                        
                        device_value->receive_right_duty_data = func_str_to_int(data_string);   // 将字符串数据转为整形 第二个数据为右侧电机占空比
                        
                        device_value->receive_right_duty_data = func_limit_ab(device_value->receive_right_duty_data, -10000, 10000);  // 数据限幅
                    }                                                                  
                    else                                                        // 未搜索到第二个逗号 则搜索回车 有一个逗号 可解析左侧占空比
                    {
                        location2 = strchr(location1, '\n');                    // 搜索回车的位置
                        
                        memset(data_string, 0x00, 10);                          // 清空缓存数组
                                                                                
                        memcpy(data_string, location1, location2 - location1);  // 取出第一个逗号到回车之间的数据
                        
                        device_value->receive_left_duty_data = func_str_to_int(data_string);    // 将字符串数据转为整形 第一个数据为左侧电机占空比
                        
                        device_value->receive_left_duty_data = func_limit_ab(device_value->receive_left_duty_data, -10000, 10000);  // 数据限幅
                      
                        device_value->receive_right_duty_data = 0;              // 没有第二个逗号 右侧占空比数据解析失败 设置占空比为 0 
                    }
                    
                }
                else
                {
                    device_value->receive_left_duty_data  = 0;                  // 没有逗号 数据指令解析异常 设置占空比为 0 
                    
                    device_value->receive_right_duty_data = 0;                  // 没有逗号 数据指令解析异常 设置占空比为 0 
                }
                
                device_value->immediate_command        = SET_DUTY;              // 即时指令(仅响应一次)状态更新为 SET_DUTY 
                        
                device_value->refresh_flag             = CMD_FORTHWITH;         // 即时指令刷新标志位修正  需要马上执行命令
            }
            else if(strstr((const char *)statement_buffer, "SET-ZERO"))        // 搜索 待解析数组内是否有 "SET-ZERO" 关键词
            {   
                device_value->immediate_command        = SET_ZERO;              // 即时指令(仅响应一次)状态更新为 SET_ZERO 
                
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            } 
            else if(strstr((const char *)statement_buffer, "SET-ANGLE-ZERO"))  // 搜索 待解析数组内是否有 "SET_ANGLE_ZERO" 关键词
            {   
                device_value->immediate_command        = SET_ANGLE_ZERO;        // 即时指令(仅响应一次)状态更新为 SET_ANGLE_ZERO
                
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            }             
            else if(strstr((const char *)statement_buffer, "SET-PHASE"))       // 搜索 待解析数组内是否有 "SET-PHASE" 关键词
            {
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            }
            else if(strstr((const char *)statement_buffer, "STOP-SEND"))       // 搜索 待解析数组内是否有 "STOP-SEND" 关键词
            {
                device_value->continue_command         = NULL_CMD;              // 持续指令(串口持续输出)状态更新为 NULL_CMD 
            } 
            else                                                                // 当前指令非协议指令 输出提示 error cmd
            {
                device_value->immediate_command        = ERROR_CMD;             // 即时指令(仅响应一次)状态更新为 ERROR_CMD 
                
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            }
        }
        else if(statement_buffer[0] == 'G')                                    // 判断 字符 指令的首字符是否为 'G'
        {
            if(strstr((const char *)statement_buffer, "GET-SPEED"))            // 搜索 待解析数组内是否有 "GET-SPEED" 关键词
            {
                device_value->continue_command         = GET_SPEED;             // 持续指令(串口持续输出)状态更新为 GET_SPEED 
            } 

            else if(strstr((const char *)statement_buffer, "GET-PHASE-DUTY"))  // 搜索 待解析数组内是否有 "GET-PHASE-DUTY" 关键词
            {                                                                   // GET-PHASE-DUTY指令格式为："GET-PHASE-DUTY,num\r\n" 其中 num 为选择的电机， 0 为左侧电机  1为右侧电机 不填 为左右互相切换
              
                location1 = strchr((char *)statement_buffer, ',');              // 搜索第一个逗号的位置 
                
                if(location1 != 0x00)                                           // 如果成功搜索到 则继续  否则默认 左右电机 互相切换
                {
                    location1 ++;                                               // 位置自增 越过第一个逗号
                    
                    location2 = strchr(location1, '\n');                        // 搜索回车的位置
                    
                    memset(data_string, 0x00, 10);                              // 清空缓存数组
                                                                                
                    memcpy(data_string, location2, location2 - location1);      // 取出第一个逗号到回车之间的数据
                    
                    device_value->select_motor = func_str_to_int(data_string);  // 将字符串数据转为整形 第一个数据为左侧电机占空比
                    
                    if(device_value->select_motor != LEFT_MOTOR)                // 数据限幅 防止用户输入异常数据  数据类型仅可选择左侧电机(O) 或者右侧电机(1)
                    {
                        device_value->select_motor = RIGHT_MOTOR;               
                    }
                }
                else
                {
                    if(device_value->select_motor == LEFT_MOTOR)                // 判断当前选择的电机类型 并且切换到另外一个电机 
                    {
                        device_value->select_motor = RIGHT_MOTOR;
                    }
                    else
                    {
                        device_value->select_motor = LEFT_MOTOR;
                    }
                }
                device_value->continue_command         = GET_PHASE_DUTY;        // 持续指令(串口持续输出)状态更新为 GET_PHASE_DUTY 
            } 
            else if(strstr((const char *)statement_buffer, "GET-ENCODER"))     // 搜索 待解析数组内是否有 "GET-ENCODER" 关键词
            {
                device_value->continue_command         = GET_ENCODER;           // 持续指令(串口持续输出)状态更新为 GET_ENCODER 
            }
            else if(strstr((const char *)statement_buffer, "GET-ANGLE"))       // 搜索 待解析数组内是否有 "GET-ANGLE" 关键词
            {
                device_value->continue_command         = GET_ANGLE;             // 持续指令(串口持续输出)状态更新为 GET_ANGLE 
            }  
            else if(strstr((const char *)statement_buffer, "GET-RDT-ANGLE"))   // 搜索 待解析数组内是否有 "GET_RDT_ANGLE" 关键词
            {
                device_value->continue_command         = GET_RDT_ANGLE;         // 持续指令(串口持续输出)状态更新为 GET_RDT_ANGLE 
            }
            
            else if(strstr((const char *)statement_buffer, "GET-VOLTAGE"))     // 搜索 待解析数组内是否有 "GET-VOLTAGE" 关键词
            {
                device_value->continue_command         = GET_VOLTAGE;           // 持续指令(串口持续输出)状态更新为 GET_VOLTAGE 
            }  
            else if(strstr((const char *)statement_buffer, "GET-IQ"))          // 持续上报 Iq 电流
            {
                device_value->continue_command         = GET_FOC_IQ;
            }
            else if(strstr((const char *)statement_buffer, "GET-UQ"))          // 持续上报 Uq
            {
                device_value->continue_command         = GET_UQ;
            }
            else                                                                // 当前指令非协议指令 输出提示 error cmd
            {   
                device_value->immediate_command        = ERROR_CMD;             // 即时指令(仅响应一次)状态更新为 ERROR_CMD 
                
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            }                                                                   
        }
        else if(statement_buffer[0] == 'T')                                     // 判断 字符 指令的首字符是否为 'T'
        {
            if(strstr((const char *)statement_buffer, "TEST-PHASE"))           // 搜索 待解析数组内是否有 "TEST-PHASE" 关键词
            {
                device_value->refresh_flag             = CMD_LOOP;              // 即时指令刷新标志位修正  无需马上执行命令
            } 
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口通讯控制 循环解析函数 
// 参数说明     device_value        通讯参数结构体
// 返回参数     void
// 使用示例     motor_driver_control_loop(&motor_value); 
// 备注信息     可放置在串口接收中断、PIT周期中断、主循环  
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_control_loop(small_device_value_struct *device_value)
{    
    uint32 read_length = 0;                                                     // 定义读取长度变量
    
    uint8  check_data = 0;                                                      // 定义数据校验变量
    
    uint8  analysis_max = 20;                                                   // 定义数据校验变量
  
    if(fifo_used(&motor_driver_fifo) >= 7)                                      // 判断 FIFO 缓冲区长度是否大于7
    {
        if(receive_enter_flag >= 1)
        {
            while(receive_enter_flag >= 1)                                      // 判断是否收到 回车 字符
            {
                receive_enter_flag --;                                          // 清除一次 回车 字符标志位
                
                read_length = fifo_used(&motor_driver_fifo);                    // 获取现在 FIFO 中的字符总长度
                
                fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_ONLY);        // 将所有数据 仅读取 出来
                
                for(int i = 0; i < read_length; i ++)                           // 查找 回车 字符位置 
                {
                    if(read_buffer[i] == '\n')                                  // 判断是否为 回车
                    {
                        read_length = i + 1;                                    // 获取到 回车的位置    
                        
                        memset(read_buffer, 0x00, 128);                         // 清空当前读出的数据
                        
                        fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_AND_CLEAN);   // 将 回车 之前的所有字符从 FIFO 缓冲区读取并且擦除
                        
                        break;                                                  // 跳出循环
                    }   
                }
                
                if(read_length > 8)
                {
                    if((read_buffer[0] == 'S' || read_buffer[0] == 'G'|| read_buffer[0] == 'T') && (read_buffer[1] == 'E' || read_buffer[1] == 'T'))
                    {
                        device_value->cmd_type = STRING_TYPE;                       // 设置通讯类型为 字符通讯
                    
                        motor_driver_parse_statement(device_value, read_buffer);    // 调用数据解析    
                    }
                }
                
                if((-- analysis_max) == 0)                                      // 允许最多一次性连续解析10条指令
                {
                    break;
                }
            }
        }
        else 
        {
            if(fifo_used(&motor_driver_fifo) >= 64)                                             // 未收到回车 但fifo占用超过 64字节 则直接仅保留最后 17 字节
            {
                read_length = fifo_used(&motor_driver_fifo) - 17;                               // 计算读取长度
                
                fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_AND_CLEAN);           // 读取无效数据
                
                memset(read_buffer, 0x00, 128);                                                 // 清空当前读出的数据
            }
            
            while(fifo_used(&motor_driver_fifo) >= 7)
            {      
                read_length = 2;
                
                fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_ONLY);// 读取前两个数据
                
                if(read_buffer[0] == 0XA5)                                                      // 判断是否为帧头
                {
                    uint8 target_length = 7;                                                    // 默认长度设为7
                    switch(read_buffer[1]) {
                        case 0x02: target_length = 7;  break; // 速度请求包（主板→驱动板 触发上报）
                        case 0x04: target_length = 7;  break; // 机械角度请求包（主板→驱动板 触发上报）
                        case 0x08: target_length = 7;  break; // Uq 请求包（主板→驱动板 触发上报）
                        case 0x09: target_length = 7;  break; // Iq 电流控制下行包（主板→驱动板 设 Iq 目标）
                        // 注：0x07 仅作驱动板→主板的 Iq 上行上报，下行不应出现
                    }
                    
                    // 【关键修复1】必须检查 FIFO 里的数据是不是已经收全了整个长包！
                    if(fifo_used(&motor_driver_fifo) >= target_length)
                    {
                        read_length = target_length;
                        fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_ONLY); // 完整读出
                        
                        // 【关键修复2】动态计算前 n-1 个字节的校验和
                        check_data = 0;
                        for(int i = 0; i < (read_length - 1); i ++) 
                        {
                            check_data += read_buffer[i];
                        }
                        
                        // 【关键修复3】动态对比最后一位校验位
                        if(check_data == read_buffer[read_length - 1]) 
                        {   
                            // 成功通过和校验 从 FIFO 读取并擦除完整的一包数据
                            fifo_read_buffer(&motor_driver_fifo, read_buffer, &read_length, FIFO_READ_AND_CLEAN);
                            
                            device_value->cmd_type = BYTE_TYPE;                                     // 设置通讯类型为 字节通讯
                            
                            motor_driver_parse_statement(device_value, read_buffer);                // 调用数据解析      
                        }
                        else
                        {
                            motor_driver_fifo_clear(1);                                             // 和校验未通过 清除帧头，往后滑动一字节寻找新帧头
                        }
                    }
                    else 
                    {
                        // 帧头是对的，但包还没收完，直接退出 while 循环，等待下一次串口接收中断
                        break; 
                    }
                }
                else
                {
                    if((read_buffer[0] == 'S' || read_buffer[0] == 'G' || read_buffer[0] == 'T') && (read_buffer[1] == 'E' || read_buffer[1] == 'T'))  // 判断是否为字符串指令
                    {
                        break;                                                                  // 字符串指令首字母不做处理
                    }
                    else
                    {
                        motor_driver_fifo_clear(1);                                             // 字节包通讯和字符串通讯协议 均不符合 清除一个缓冲区数据（状态机机制）
                    }
                }   
                
                if((-- analysis_max) == 0)
                {
                    break;
                }
            }
        }
        driver_cmd_forthwith();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送占空比信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机占空比
// 参数说明     right_speed         右侧电机占空比
// 返回参数     void
// 使用示例     motor_driver_set_duty(&motor_value, 500, 500);   // 字节包通讯则按照协议打包输出 占空比范围 -10000 ~ 10000
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_set_duty(small_device_value_struct *device_value, int left_duty, int right_duty)
{
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X01;
    device_value->send_data_buffer[2] = (uint8)((left_duty  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_duty  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_duty & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];

    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送速度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机速度
// 参数说明     right_speed         右侧电机速度
// 返回参数     void
// 使用示例     motor_driver_send_speed(&motor_value, motor_left.motor_speed_filter, motor_right.motor_speed_filter);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_speed(small_device_value_struct *device_value, int left_speed, int right_speed)
{
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X02;
    device_value->send_data_buffer[2] = (uint8)((left_speed  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_speed  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_speed & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_speed & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送角度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机角度
// 参数说明     right_speed         右侧电机角度
// 返回参数     void
// 使用示例     motor_driver_send_angle(&motor_value, left_angle, right_angle);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_angle(small_device_value_struct *device_value, float left_angle, float right_angle)
{
    int16 left_angle_int = (int16)(func_limit_ab(left_angle, -327.67f, 327.67f) * 100.0f);
    int16 righ_angle_int = (int16)(func_limit_ab(right_angle, -327.67f, 327.67f) * 100.0f);
    
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X04;
    device_value->send_data_buffer[2] = (uint8)((left_angle_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_angle_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((righ_angle_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( righ_angle_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送减速后角度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机减速后角度
// 参数说明     right_speed         右侧电机减速后角度
// 返回参数     void
// 使用示例     motor_driver_send_reduction_angle(&motor_value, left_angle, right_angle);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_reduction_angle(small_device_value_struct *device_value, float left_angle, float right_angle)
{
    uint16 left_angle_int = (uint16)(left_angle * 100.0f);
    uint16 righ_angle_int = (uint16)(right_angle * 100.0f);
    
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X05;
    device_value->send_data_buffer[2] = (uint8)((left_angle_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_angle_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((righ_angle_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( righ_angle_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口通讯控制 回调函数
// 参数说明     device_value        通讯参数结构体
// 返回参数     void
// 使用示例     motor_driver_control_callback(&motor_value);
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_control_callback(small_device_value_struct *device_value)
{
    uint8 receive_data;                                                         // 定义串口接收变量
    
    uint8 receive_max = 10;                                                     // 定义单次接收允许读取的最大数据量
    
    while(uart_query_byte(MOTOR_DRIVER_UART, &receive_data) == 1)               // 判断是否接收到串口数据
    {
        fifo_write_element(&motor_driver_fifo, receive_data);                   // 将数据写入 fifo
        
        
        if(receive_data == '\n')                                                // 若接收到特殊指令 回车：'\n'  则将回车标志位置位 用于后续解析
        {
            receive_enter_flag ++;
        }
        
        motor_driver_control_loop(device_value);                                // 调用数据解析 该函数也可在其他位置调用
        
        if((-- receive_max) == 0)
        {
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     驱动 串口通讯 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_driver_uart_init();
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_uart_init(void)
{
    fifo_init(&motor_driver_fifo, FIFO_DATA_8BIT, driver_fifo_buffer, 128);     // 初始化 fifo 结构体
      
    uart_init(MOTOR_DRIVER_UART, 460800, MOTOR_DRIVER_RX, MOTOR_DRIVER_TX);     // 与主板 MAIN_BOARD_BAUDRATE 保持一致(921600)
    // 底层 uart_query_byte 是从中断回调写入的内部 FIFO 读取数据，
    // 因此命令口对应的 RX 中断必须打开。
    uart_rx_interrupt(MOTOR_DRIVER_UART, 1);
}





// ===== FOC 通信函数实现 =====

//===================================================================================================================



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送 Iq 电流（左右电机），7 字节小包
// 帧格式      0xA5 0x07 LeftIq_H LeftIq_L RightIq_H RightIq_L SUM
// 缩放        uint16 = (int16)(iq * 100) + 3200，量程 ±32A，精度 0.01A
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_iq(small_device_value_struct *device_value, float left_iq, float right_iq)
{
    uint16 left_iq_int  = (uint16)((int16)(func_limit_ab(left_iq,  -32.0f, 32.0f) * 100.0f) + 3200);
    uint16 right_iq_int = (uint16)((int16)(func_limit_ab(right_iq, -32.0f, 32.0f) * 100.0f) + 3200);

    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0x07;
    device_value->send_data_buffer[2] = (uint8)((left_iq_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_iq_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_iq_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_iq_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] +
                                        device_value->send_data_buffer[1] +
                                        device_value->send_data_buffer[2] +
                                        device_value->send_data_buffer[3] +
                                        device_value->send_data_buffer[4] +
                                        device_value->send_data_buffer[5];

    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送 Uq（左右电机 q 轴电压），7 字节小包
// 帧格式      0xA5 0x08 LeftUq_H LeftUq_L RightUq_H RightUq_L SUM
// 缩放        int16 = uq * 1000，量程 ±32.767V，精度 0.001V
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_uq(small_device_value_struct *device_value, float left_uq, float right_uq)
{
    int16 left_uq_int  = (int16)(func_limit_ab(left_uq,  -32.767f, 32.767f) * 1000.0f);
    int16 right_uq_int = (int16)(func_limit_ab(right_uq, -32.767f, 32.767f) * 1000.0f);

    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0x08;
    device_value->send_data_buffer[2] = (uint8)((left_uq_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_uq_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_uq_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_uq_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] +
                                        device_value->send_data_buffer[1] +
                                        device_value->send_data_buffer[2] +
                                        device_value->send_data_buffer[3] +
                                        device_value->send_data_buffer[4] +
                                        device_value->send_data_buffer[5];

    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}







// //-------------------------------------------------------------------------------------------------------------------

// // 函数简介     发送轮编码器数据

// // 参数说明     device_value        通讯参数结构体

// // 参数说明     encoder_data        轮编码器数据结构体

// // 返回参数     void

// // 使用示例     motor_driver_send_wheel_encoder(&motor_value, &encoder_data);

// // 备注信息     包含左右轮编码器计数

// //-------------------------------------------------------------------------------------------------------------------

// void motor_driver_send_wheel_encoder(small_device_value_struct *device_value, wheel_encoder_data_tx *encoder_data)

// {

//     uint8 send_buffer[20];

//     uint8 check_sum = 0;

   

//     send_buffer[0] = 0xA5;                                          // 帧头

//     send_buffer[1] = 0x0D;                                          // 功能字

   

//     // 左轮编码器（4字节）

//     send_buffer[2] = (uint8)((encoder_data->left_encoder & 0xFF000000) >> 24);

//     send_buffer[3] = (uint8)((encoder_data->left_encoder & 0x00FF0000) >> 16);

//     send_buffer[4] = (uint8)((encoder_data->left_encoder & 0x0000FF00) >> 8);

//     send_buffer[5] = (uint8)(encoder_data->left_encoder & 0x000000FF);

   

//     // 右轮编码器（4字节）

//     send_buffer[6] = (uint8)((encoder_data->right_encoder & 0xFF000000) >> 24);

//     send_buffer[7] = (uint8)((encoder_data->right_encoder & 0x00FF0000) >> 16);

//     send_buffer[8] = (uint8)((encoder_data->right_encoder & 0x0000FF00) >> 8);

//     send_buffer[9] = (uint8)(encoder_data->right_encoder & 0x000000FF);

   

//     // 计算校验和

//     for(int i = 0; i < 10; i++)

//     {

//         check_sum += send_buffer[i];

//     }

//     send_buffer[10] = check_sum;

   

//     uart_write_buffer(MOTOR_DRIVER_UART, send_buffer, 11);

// }


// //-------------------------------------------------------------------------------------------------------------------
// // 函数简介     发送驱动板电机状态（双电机：三相电流 + Iq力矩量 + 角度 + 速度）
// // 参数说明     device_value        通讯参数结构体（仅用于复用其内部缓冲区，无内部状态依赖）
// // 参数说明     status              双电机状态结构体（浮点）
// // 返回参数     void
// // 使用示例     motor_status_data_tx s = { ... };
// //             motor_driver_send_motor_status(&motor_value, &s);
// // 备注信息     帧格式：0xA5 0x20 + 左(Ia,Ib,Ic,Iq,Angle,Speed) + 右(同) + sum8
// //             共 27 字节；电流/角度按 *100 缩放为 int16，转速按 RPM 直接 int16
// //-------------------------------------------------------------------------------------------------------------------
// void motor_driver_send_motor_status(small_device_value_struct *device_value, motor_status_data_tx *status)
// {
//     uint8 buf[27];
//     uint8 sum = 0;
//     uint8 i;

//     // 局部内联：浮点 -> int16(限幅+四舍五入) -> 大端写入
//     // 用宏避免函数调用开销，注意 ##__LINE__ 防止变量名冲突
//     #define WRITE_INT16_BE(idx, fval, scale)                              \
//         do {                                                              \
//             float _v = (fval) * (scale);                                  \
//             if(_v >  32767.0f) _v =  32767.0f;                            \
//             if(_v < -32768.0f) _v = -32768.0f;                            \
//             int16 _iv = (int16)(_v >= 0.0f ? (_v + 0.5f) : (_v - 0.5f));  \
//             buf[(idx)    ] = (uint8)(((uint16)_iv >> 8) & 0xFF);          \
//             buf[(idx) + 1] = (uint8)( (uint16)_iv       & 0xFF);          \
//         } while(0)

//     buf[0] = 0xA5;          // 帧头
//     buf[1] = 0x20;          // 功能字：电机状态上报

//     // 左电机：Ia, Ib, Ic, Iq, Angle(deg), Speed(rpm)
//     WRITE_INT16_BE( 2, status->left.ia,        100.0f);
//     WRITE_INT16_BE( 4, status->left.ib,        100.0f);
//     WRITE_INT16_BE( 6, status->left.ic,        100.0f);
//     WRITE_INT16_BE( 8, status->left.iq,        100.0f);
//     WRITE_INT16_BE(10, status->left.angle_deg, 100.0f);
//     WRITE_INT16_BE(12, status->left.speed_rpm,   1.0f);

//     // 右电机
//     WRITE_INT16_BE(14, status->right.ia,        100.0f);
//     WRITE_INT16_BE(16, status->right.ib,        100.0f);
//     WRITE_INT16_BE(18, status->right.ic,        100.0f);
//     WRITE_INT16_BE(20, status->right.iq,        100.0f);
//     WRITE_INT16_BE(22, status->right.angle_deg, 100.0f);
//     WRITE_INT16_BE(24, status->right.speed_rpm,   1.0f);

//     #undef WRITE_INT16_BE

//     // 计算前26字节累加校验
//     for(i = 0; i < 26; i++)
//     {
//         sum = (uint8)(sum + buf[i]);
//     }
//     buf[26] = sum;

//     // 通过 UART2 发送（MOTOR_DRIVER_UART）
//     uart_write_buffer(MOTOR_DRIVER_UART, buf, 27);

//     (void)device_value;     // 当前实现未使用 device_value，保留参数以便后续扩展
// }
