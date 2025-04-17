#include "zf_common_headfile.h"
#include <stdbool.h>


uint8  jiezhihang_chang_flag=0;
uint8  show_flag=0;


uint32_t press_start_time = 0;
uint8_t is_counted = 0;  // 防止重复计数标志






uint32_t get_system_time(void)
{
    return p; // 返回当前时间（单位：ms）
}



void keys_init()  //按键初始化
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
    


}

#define LONG_PRESS_THRESHOLD_MS 200 // 长按时间阈值（单位：毫秒）
#define SHORT_PRESS_THRESHOLD_MS 50 // 短按最小时间阈值（单位：毫秒）

/**
 * 检测按键短按和长按。
 * 
 * @param key_index 要检测的按键索引（例如：0表示KEY1，1表示KEY2）。
 * @return 返回按键事件类型：0表示无事件，1表示短按，2表示长按。
 */
uint8_t handle_key_press(uint8_t key_index) {
    static uint32_t press_start_time[4] = {0}; // 用于存储最多4个按键的按下开始时间
    static uint8_t is_counted[4] = {0};       // 防止同一按键重复触发的标志位
    bool current_state = (gpio_get_level(key_index + KEY1) == 0); // 检查按键是否被按下

    if (current_state) {
        // 按键被按下
        if (!is_counted[key_index]) {
            press_start_time[key_index] = get_system_time(); // 记录按下的开始时间
            is_counted[key_index] = 1; // 标记此按键已被按下
        } else if (get_system_time() - press_start_time[key_index] >= LONG_PRESS_THRESHOLD_MS) {
            // 检测到长按
            is_counted[key_index] = 0; // 重置状态，防止重复触发长按
            return 2; // 返回长按事件
        }
    } else {
        // 按键被释放
        if (is_counted[key_index]) {
            uint32_t press_duration = get_system_time() - press_start_time[key_index];
            is_counted[key_index] = 0; // 重置标志位
            if (press_duration >= SHORT_PRESS_THRESHOLD_MS && press_duration < LONG_PRESS_THRESHOLD_MS) {
                // 检测到短按
                return 1; // 返回短按事件
            }
        }
    }
    return 0; // 无事件
}


void anjian_pid_change() {
    uint8_t key1_event = handle_key_press(0); // 检测按键1事件
 if (key1_event == 1) {
        // 短按逻辑
        track_configuration[TRACK_VERTICAL].kp += 0.1;
    } else if (key1_event == 2) {
        // 长按逻辑
        uint8_t previous_show_flag = show_flag; // 保存切换前的状态
        show_flag++;
        show_flag %= 2;

        // 如果状态发生了切换，清屏
        if (show_flag != previous_show_flag) {
            ips114_clear(); // 调用清屏函数
        }
    }
 
 
 
    // Existing key handling logic for other keys
    if (!gpio_get_level(KEY2)) {
        system_delay_ms(20);
        while (gpio_get_level(KEY2) == 0);
        track_configuration[TRACK_VERTICAL].kp -= 0.1;
    }
    if (!gpio_get_level(KEY3)) {
        system_delay_ms(20);
        while (gpio_get_level(KEY3) == 0);
        track_configuration[TRACK_VERTICAL].kd += 0.1;
    }
    if (!gpio_get_level(KEY4)) {
        system_delay_ms(20);
        while (gpio_get_level(KEY4) == 0);
        track_configuration[TRACK_VERTICAL].kd -= 0.1;
    }
}













// 在显示控制逻辑中添加状态跟踪
void display_control(void)
{
   

   if(show_flag == 0)
    {
      
//赛道判断专区
      
      // 显示数据（保持原有逻辑）
      /*  ips114_show_int(162, 1, Motor_Left_pid.present_value[0], 6);
        ips114_show_int(162, 16, Motor_Right_pid.present_value[0], 6);
        ips114_show_int(162, 31, current_error, 6);
      
     // ips114_show_float(162, 16, angle_type, 2,2);
     
     // ips114_show_int(162, 76, width_change, 6);
       // ips114_show_float (162, 1, weighted_variance,2,2);//方差
       // ips114_show_int(162, 16, continuous_cnt, 6);
        //ips114_show_int(162, 46, ImageParameter.OFFLine, 6);
        
        ips114_show_float (162,46,  left_slope1,2,2);
        ips114_show_float (162, 61, right_slope1,2,2);
      //  ips114_show_float (162, 61, right_slope,  2,  2);//斜率
        ips114_show_int(162, 76, Track_Line_Num, 6);*/
//赛道判断专区
        
        
     
     
  
  
  
  
        
      
 //pid显示区域     
       // 显示数据（保持原有逻辑）
        ips114_show_int(162, 1, Motor_Left_pid.present_value[0], 6);
        ips114_show_int(162, 16, Motor_Right_pid.present_value[0], 6);
       // ips114_show_int(162, 31, current_error, 6);
       // ips114_show_int(162, 46, CalculateOFFLineCenter()-PictureCentring, 6);
        ips114_show_int(162, 31,  show_flag, 6);
        
         ips114_show_int(162, 46,   track_configuration[TRACK_VERTICAL].kd , 6);
          
       // ips114_show_float(162, 46, track_configuration[TRACK_VERTICAL].kp, 2,2);//直角
       // ips114_show_float(162, 61, track_configuration[TRACK_VERTICAL].kd, 2,2);//直角*/
       // ips114_show_float(162, 61, offerror, 2,2);//直角*/
        
         //ips114_show_float (162, 46, cfg.kp,2,2);
        // ips114_show_float (162, 61, cfg.kd,2,2);
          
         ips114_show_int(162, 76, Track_Line_Num, 6);
       
       // ips114_show_int(162, 76, Motor_Left_pid.set_value[0], 6);
       // ips114_show_int(162, 91, Motor_Right_pid.set_value[0], 6);  
         
      ips114_show_gray_image(0,0,(const uint8 *)Compressed_image,80,60,80*2,60*2,ImageParameter.Threshold);  
     // ips114_show_gray_image(0,0,(const uint8 *)Compressed_image,80,60,80,60,ImageParameter.Threshold);  
     // ips114_show_gray_image(80,0,(const uint8 *)Compressed_image,80,60,80,60,0);
      
      
        ips114_draw_line (0, ImageParameter.OFFLine, 158,ImageParameter.OFFLine, RGB565_RED);
        ips114_draw_line (0, 94, 158,94, RGB565_RED);
        ips114_draw_line (0, 110, 158,110, RGB565_RED);
     // ips114_draw_line (0, 70, 158,70, RGB565_RED);
   //   ips114_draw_line (0, 90, 158,90, RGB565_RED);
      //ips114_draw_line (0, 90, 158,90, RGB565_RED);
      /* int i;
      for(i = IPSH - 1; i > 19; i-=2)
    {
        ips114_draw_point(RowAttribute[i].LeftBorder*2 , i, RGB565_GREEN);       //画左边界线
        ips114_draw_point(RowAttribute[i].RightBorder*2 , i, RGB565_GREEN);      //画右边界线   原始边界
        ips114_draw_point(RowAttribute[i].Center*2,       i, RGB565_RED);       //画中线
    }*/
      
      
    }
  
}

