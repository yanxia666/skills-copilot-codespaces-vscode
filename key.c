#include "zf_common_headfile.h"
#include <stdbool.h>


uint8  jiezhihang_chang_flag=0;
uint8  show_flag=0;


uint32_t press_start_time = 0;
uint8_t is_counted = 0;  // ��ֹ�ظ�������־






uint32_t get_system_time(void)
{
    return p; // ���ص�ǰʱ�䣨��λ��ms��
}



void keys_init()  //������ʼ��
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������
    


}

#define LONG_PRESS_THRESHOLD_MS 200 // ����ʱ����ֵ����λ�����룩
#define SHORT_PRESS_THRESHOLD_MS 50 // �̰���Сʱ����ֵ����λ�����룩

/**
 * ��ⰴ���̰��ͳ�����
 * 
 * @param key_index Ҫ���İ������������磺0��ʾKEY1��1��ʾKEY2����
 * @return ���ذ����¼����ͣ�0��ʾ���¼���1��ʾ�̰���2��ʾ������
 */
uint8_t handle_key_press(uint8_t key_index) {
    static uint32_t press_start_time[4] = {0}; // ���ڴ洢���4�������İ��¿�ʼʱ��
    static uint8_t is_counted[4] = {0};       // ��ֹͬһ�����ظ������ı�־λ
    bool current_state = (gpio_get_level(key_index + KEY1) == 0); // ��鰴���Ƿ񱻰���

    if (current_state) {
        // ����������
        if (!is_counted[key_index]) {
            press_start_time[key_index] = get_system_time(); // ��¼���µĿ�ʼʱ��
            is_counted[key_index] = 1; // ��Ǵ˰����ѱ�����
        } else if (get_system_time() - press_start_time[key_index] >= LONG_PRESS_THRESHOLD_MS) {
            // ��⵽����
            is_counted[key_index] = 0; // ����״̬����ֹ�ظ���������
            return 2; // ���س����¼�
        }
    } else {
        // �������ͷ�
        if (is_counted[key_index]) {
            uint32_t press_duration = get_system_time() - press_start_time[key_index];
            is_counted[key_index] = 0; // ���ñ�־λ
            if (press_duration >= SHORT_PRESS_THRESHOLD_MS && press_duration < LONG_PRESS_THRESHOLD_MS) {
                // ��⵽�̰�
                return 1; // ���ض̰��¼�
            }
        }
    }
    return 0; // ���¼�
}


void anjian_pid_change() {
    uint8_t key1_event = handle_key_press(0); // ��ⰴ��1�¼�
 if (key1_event == 1) {
        // �̰��߼�
        track_configuration[TRACK_VERTICAL].kp += 0.1;
    } else if (key1_event == 2) {
        // �����߼�
        uint8_t previous_show_flag = show_flag; // �����л�ǰ��״̬
        show_flag++;
        show_flag %= 2;

        // ���״̬�������л�������
        if (show_flag != previous_show_flag) {
            ips114_clear(); // ������������
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













// ����ʾ�����߼������״̬����
void display_control(void)
{
   

   if(show_flag == 0)
    {
      
//�����ж�ר��
      
      // ��ʾ���ݣ�����ԭ���߼���
      /*  ips114_show_int(162, 1, Motor_Left_pid.present_value[0], 6);
        ips114_show_int(162, 16, Motor_Right_pid.present_value[0], 6);
        ips114_show_int(162, 31, current_error, 6);
      
     // ips114_show_float(162, 16, angle_type, 2,2);
     
     // ips114_show_int(162, 76, width_change, 6);
       // ips114_show_float (162, 1, weighted_variance,2,2);//����
       // ips114_show_int(162, 16, continuous_cnt, 6);
        //ips114_show_int(162, 46, ImageParameter.OFFLine, 6);
        
        ips114_show_float (162,46,  left_slope1,2,2);
        ips114_show_float (162, 61, right_slope1,2,2);
      //  ips114_show_float (162, 61, right_slope,  2,  2);//б��
        ips114_show_int(162, 76, Track_Line_Num, 6);*/
//�����ж�ר��
        
        
     
     
  
  
  
  
        
      
 //pid��ʾ����     
       // ��ʾ���ݣ�����ԭ���߼���
        ips114_show_int(162, 1, Motor_Left_pid.present_value[0], 6);
        ips114_show_int(162, 16, Motor_Right_pid.present_value[0], 6);
       // ips114_show_int(162, 31, current_error, 6);
       // ips114_show_int(162, 46, CalculateOFFLineCenter()-PictureCentring, 6);
        ips114_show_int(162, 31,  show_flag, 6);
        
         ips114_show_int(162, 46,   track_configuration[TRACK_VERTICAL].kd , 6);
          
       // ips114_show_float(162, 46, track_configuration[TRACK_VERTICAL].kp, 2,2);//ֱ��
       // ips114_show_float(162, 61, track_configuration[TRACK_VERTICAL].kd, 2,2);//ֱ��*/
       // ips114_show_float(162, 61, offerror, 2,2);//ֱ��*/
        
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
        ips114_draw_point(RowAttribute[i].LeftBorder*2 , i, RGB565_GREEN);       //����߽���
        ips114_draw_point(RowAttribute[i].RightBorder*2 , i, RGB565_GREEN);      //���ұ߽���   ԭʼ�߽�
        ips114_draw_point(RowAttribute[i].Center*2,       i, RGB565_RED);       //������
    }*/
      
      
    }
  
}

