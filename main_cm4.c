
#include "zf_common_headfile.h"


/*
* �޸ļ�¼
* ����              ����                ��ע
* 2025-4-17      wanglihui            first version
********************************************************************************************************************/


int main(void)
{
   

    clock_init(SYSTEM_CLOCK_160M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();                  // ���Դ�����Ϣ��ʼ��
    debug_init();                       // ���Դ��ڳ�ʼ��
    
    
                
 
   
    mt9v03x_init();//����ͷ��ʼ��*/
    imu660ra_init (); //�����ǳ�ʼ��
    Motor_Init();
    Data_Settings();
    Speed_control_init();
    ips114_init();
    wireless_uart_init();
    keys_init();    
    pit_ms_init(PIT_CH0,5);    //0.01s   
   
    while(true)
   {    

          if(mt9v03x_finish_flag)
            {
              ImageProcess();          
              display_control();
            } 
    }
}
