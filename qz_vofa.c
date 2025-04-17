#include "zf_common_headfile.h"

// 删除原有串口定义，改用无线库定义
#define WIRELESS_UART_INDEX        (UART_0)        // 使用无线库的 UART 定义
#define WIRELESS_UART_BUAD_RATE    (115200)        // 波特率 115200

uint8 fifo_get_data[200];                          // FIFO 输出读出缓冲区
uint8_t rxIndex = 0;                               // 接收索引

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      解析接收到的数据并获取浮点数
// 参数说明      void
// 返回参数      float          解析后的浮点数值
// 使用示例      float data = Get_Data();
// 备注信息      
//-------------------------------------------------------------------------------------------------------------------
float Get_Data(void) 
{
    uint8_t data_Start_Num = 0;                    // 记录数据位开始的地方
    uint8_t data_End_Num = 0;                      // 记录数据位结束的地方
    uint8_t data_Num = 0;                          // 记录数据位数
    uint8_t minus_Flag = 0;                        // 判断是不是负数
    float data_return = 0;                         // 解析得到的数据

    for (uint8_t i = 0; i < 200; i++)              // 查找等号和@的位置
    { 
        if (fifo_get_data[i] == '=') 
            data_Start_Num = i + 1;                // +1直接定位到数据起始位
        if (fifo_get_data[i] == '@') 
        {
            data_End_Num = i - 1;
            break;
        }
    }

    if (fifo_get_data[data_Start_Num] == '-')      // 负数处理
    { 
        data_Start_Num += 1;
        minus_Flag = 1;
    }

    data_Num = data_End_Num - data_Start_Num + 1;  // 计算数据位数

    // 数据解析逻辑（保持原有算法）
    if (data_Num == 4) 
    {
        data_return = (fifo_get_data[data_Start_Num] - 48) + 
                     (fifo_get_data[data_Start_Num + 2] - 48) * 0.1f +
                     (fifo_get_data[data_Start_Num + 3] - 48) * 0.01f;
    } 
    else if (data_Num == 5) 
    {
        data_return = (fifo_get_data[data_Start_Num] - 48) * 10 + 
                     (fifo_get_data[data_Start_Num + 1] - 48) +
                     (fifo_get_data[data_Start_Num + 3] - 48) * 0.1f + 
                     (fifo_get_data[data_Start_Num + 4] - 48) * 0.01f;
    } 
    else if (data_Num == 6) 
    {
        data_return = (fifo_get_data[data_Start_Num] - 48) * 100 + 
                     (fifo_get_data[data_Start_Num + 1] - 48) * 10 +
                     (fifo_get_data[data_Start_Num + 2] - 48) + 
                     (fifo_get_data[data_Start_Num + 4] - 48) * 0.1f +
                     (fifo_get_data[data_Start_Num + 5] - 48) * 0.01f;
    } 
    else if (data_Num == 7) 
    {
        data_return = (fifo_get_data[data_Start_Num] - 48) * 1000 + 
                     (fifo_get_data[data_Start_Num + 1] - 48) * 100 +
                     (fifo_get_data[data_Start_Num + 2] - 48) * 10 + 
                     (fifo_get_data[data_Start_Num + 3] - 48) +
                     (fifo_get_data[data_Start_Num + 5] - 48) * 0.1f + 
                     (fifo_get_data[data_Start_Num + 6] - 48) * 0.01f;
    }

    return minus_Flag ? -data_return : data_return;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      无线串口格式化输出
// 参数说明      fmt             格式化字符串
// 参数说明      ...             可变参数
// 返回参数      void
// 使用示例      wireless_printf("KP=%.2f@", 1.23f);
// 备注信息      支持流控的增强版格式化输出
//-------------------------------------------------------------------------------------------------------------------
void wireless_printf(char* fmt, ...)
{
    #define WIRELESS_TX_BUF_SIZE 128  // 根据实际需求调整缓冲区大小
    static uint8 WIRELESS_TX_BUF[WIRELESS_TX_BUF_SIZE];
    
    va_list ap;
    va_start(ap, fmt);
    vsnprintf((char*)WIRELESS_TX_BUF, WIRELESS_TX_BUF_SIZE, fmt, ap); // 安全格式化
    va_end(ap);

    // 使用流控优化发送函数
    wireless_uart_send_string((const char*)WIRELESS_TX_BUF);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介      无线模块数据接收处理
// 参数说明      void
// 返回参数      void
// 使用示例      wireless_data_handler();
// 备注信息      需要放在主循环中周期性调用
//-------------------------------------------------------------------------------------------------------------------
void wireless_data_handler(void)
{
    // 从无线模块 FIFO 读取数据
    uint32 len = wireless_uart_read_buffer(fifo_get_data + rxIndex, sizeof(fifo_get_data) - rxIndex);
    if(len > 0)
    {
        rxIndex += len;
        fifo_get_data[rxIndex] = '\0'; // 添加字符串终结符
        
        // 查找结束符 '@'
        char *end_ptr = strchr((char*)fifo_get_data, '@');
        if(end_ptr != NULL)
        {
            *end_ptr = '\0'; // 分割字符串
            
            // 调用 PID 调整函数
           // USART_PID_Adjust();
            
            // 调试输出（使用无线库发送）
            wireless_uart_send_string("Received: ");
            wireless_uart_send_string((char*)fifo_get_data);
            wireless_uart_send_string("\r\n");
            
            // 移动剩余数据到缓冲区头部
            uint8_t remain_len = rxIndex - (end_ptr - (char*)fifo_get_data) - 1;
            memmove(fifo_get_data, end_ptr + 1, remain_len);
            rxIndex = remain_len;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      PID 参数调整
// 参数说明      void
// 返回参数      void
// 使用示例      USART_PID_Adjust();
// 备注信息      
//-------------------------------------------------------------------------------------------------------------------
/*void USART_PID_Adjust(void) 
{
    float data_Get = Get_Data();

    if (fifo_get_data[0] == 'P') 
    {
        if (fifo_get_data[1] == '1') 
            zhi_pid.kp = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.kp = data_Get;
    }
    else if (fifo_get_data[0] == 'I') 
    {
        if (fifo_get_data[1] == '1') 
            Motor_Left_pid.ki = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.ki = data_Get;
    }
    else if (fifo_get_data[0] == 'D') 
    {
        if (fifo_get_data[1] == '1') 
            zhi_pid.kd = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.kd = data_Get;
    }
    
    */
    
  /*  if (fifo_get_data[0] == 'P') 
    {
        if (fifo_get_data[1] == '1') 
            Motor_Left_pid.kp = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.kp = data_Get;
    }
    else if (fifo_get_data[0] == 'I') 
    {
        if (fifo_get_data[1] == '1') 
            Motor_Left_pid.ki = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.ki = data_Get;
    }
    else if (fifo_get_data[0] == 'D') 
    {
        if (fifo_get_data[1] == '1') 
            Motor_Left_pid.kd = data_Get;
        else if (fifo_get_data[1] == '2') 
            Motor_Right_pid.kd = data_Get;
    }*/


