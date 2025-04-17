
#include "zf_common_headfile.h"


/*
* 修改记录
* 日期              作者                备注
* 2025-4-17      wanglihui            first version
********************************************************************************************************************/


int main(void)
{
   

    clock_init(SYSTEM_CLOCK_160M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    debug_init();                       // 调试串口初始化
    
    
                
 
   
    mt9v03x_init();//摄像头初始化*/
    imu660ra_init (); //陀螺仪初始化
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
