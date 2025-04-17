#include "zf_common_headfile.h"

// ɾ��ԭ�д��ڶ��壬�������߿ⶨ��
#define WIRELESS_UART_INDEX        (UART_0)        // ʹ�����߿�� UART ����
#define WIRELESS_UART_BUAD_RATE    (115200)        // ������ 115200

uint8 fifo_get_data[200];                          // FIFO �������������
uint8_t rxIndex = 0;                               // ��������

//-------------------------------------------------------------------------------------------------------------------
// �������      �������յ������ݲ���ȡ������
// ����˵��      void
// ���ز���      float          ������ĸ�����ֵ
// ʹ��ʾ��      float data = Get_Data();
// ��ע��Ϣ      
//-------------------------------------------------------------------------------------------------------------------
float Get_Data(void) 
{
    uint8_t data_Start_Num = 0;                    // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0;                      // ��¼����λ�����ĵط�
    uint8_t data_Num = 0;                          // ��¼����λ��
    uint8_t minus_Flag = 0;                        // �ж��ǲ��Ǹ���
    float data_return = 0;                         // �����õ�������

    for (uint8_t i = 0; i < 200; i++)              // ���ҵȺź�@��λ��
    { 
        if (fifo_get_data[i] == '=') 
            data_Start_Num = i + 1;                // +1ֱ�Ӷ�λ��������ʼλ
        if (fifo_get_data[i] == '@') 
        {
            data_End_Num = i - 1;
            break;
        }
    }

    if (fifo_get_data[data_Start_Num] == '-')      // ��������
    { 
        data_Start_Num += 1;
        minus_Flag = 1;
    }

    data_Num = data_End_Num - data_Start_Num + 1;  // ��������λ��

    // ���ݽ����߼�������ԭ���㷨��
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
// �������      ���ߴ��ڸ�ʽ�����
// ����˵��      fmt             ��ʽ���ַ���
// ����˵��      ...             �ɱ����
// ���ز���      void
// ʹ��ʾ��      wireless_printf("KP=%.2f@", 1.23f);
// ��ע��Ϣ      ֧�����ص���ǿ���ʽ�����
//-------------------------------------------------------------------------------------------------------------------
void wireless_printf(char* fmt, ...)
{
    #define WIRELESS_TX_BUF_SIZE 128  // ����ʵ�����������������С
    static uint8 WIRELESS_TX_BUF[WIRELESS_TX_BUF_SIZE];
    
    va_list ap;
    va_start(ap, fmt);
    vsnprintf((char*)WIRELESS_TX_BUF, WIRELESS_TX_BUF_SIZE, fmt, ap); // ��ȫ��ʽ��
    va_end(ap);

    // ʹ�������Ż����ͺ���
    wireless_uart_send_string((const char*)WIRELESS_TX_BUF);
}
//-------------------------------------------------------------------------------------------------------------------
// �������      ����ģ�����ݽ��մ���
// ����˵��      void
// ���ز���      void
// ʹ��ʾ��      wireless_data_handler();
// ��ע��Ϣ      ��Ҫ������ѭ���������Ե���
//-------------------------------------------------------------------------------------------------------------------
void wireless_data_handler(void)
{
    // ������ģ�� FIFO ��ȡ����
    uint32 len = wireless_uart_read_buffer(fifo_get_data + rxIndex, sizeof(fifo_get_data) - rxIndex);
    if(len > 0)
    {
        rxIndex += len;
        fifo_get_data[rxIndex] = '\0'; // ����ַ����ս��
        
        // ���ҽ����� '@'
        char *end_ptr = strchr((char*)fifo_get_data, '@');
        if(end_ptr != NULL)
        {
            *end_ptr = '\0'; // �ָ��ַ���
            
            // ���� PID ��������
           // USART_PID_Adjust();
            
            // ���������ʹ�����߿ⷢ�ͣ�
            wireless_uart_send_string("Received: ");
            wireless_uart_send_string((char*)fifo_get_data);
            wireless_uart_send_string("\r\n");
            
            // �ƶ�ʣ�����ݵ�������ͷ��
            uint8_t remain_len = rxIndex - (end_ptr - (char*)fifo_get_data) - 1;
            memmove(fifo_get_data, end_ptr + 1, remain_len);
            rxIndex = remain_len;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������      PID ��������
// ����˵��      void
// ���ز���      void
// ʹ��ʾ��      USART_PID_Adjust();
// ��ע��Ϣ      
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


