#include "zf_common_headfile.h"

//1 ֱ��   2 С����  3 ��ֱ��
uint8  Track_Line_Num=0;
int sum,i;
uint8 pid_set = 2;    //ѡ����ģ��pid �� λ��ʽpid
uint8_t jiezhihang_change=12;  //������������
uint8 Point_Mid; //����������ֵ
int8_t Media_line_difference=0;
Direct_pid_info Direct_pid;
int8_t piancha,last_piancha; //�����ж�·����ƫ��
Direct_pid_info zhi_pid;          //��ֱ��pid1
Direct_pid_info zhexian_pid;          //С����pid1
Direct_pid_info chuizhi_pid;       //��ֱ��pid1
Direct_pid_info angle_pid;       //�ǶȻ�PID
float GKD=0;

int current_error;


// ΢������ר��������2cm���ߣ�

#define KD_SCALE               1.5f // ΢�����͹��ԣ���ǿ΢������
// 3. ��ȷ��ʼ����ע�ⶺ�ź�C99֧�֣�
TrackPIDConfig track_configuration[] = {
    [DEFAULT] = {.kp = 3.8f, .kd = 35.0f*KD_SCALE, .base_speed = 20},  // ĩβ�����ж���
  /*  15��35��18
    3.2��35��35
    3.8  35  20���ٺܺ���
    3.8��35��45Ŀǰ�ٶ��������ߺ�ֱ��
    40.8��189.0��45���Թ�ֱ��*/
    [TRACK_VERTICAL] = {.kp = 40.8f, .kd = 189.0f*KD_SCALE, .base_speed = 20},
};
 
//------------------------- ���ƺ��� -------------------------
void Track_Control_Core(TrackType track_type) {
  
  
    static int last_error = 0;
    
    // 1. ��ȡ����ƫ��
    
    if(track_type==TRACK_VERTICAL)
    {
      current_error=CalculateOFFLineCenter()-PictureCentring;
    }else{
       current_error = ImageParameter.Det_True - PictureCentring;
    }
   
    // 2. ѡ��PID����
    TrackPIDConfig cfg = track_configuration[track_type];
    
    // 3. ���������
    float p_term = cfg.kp * current_error;
    float d_term = cfg.kd * (current_error - last_error);
    float control_output = (p_term + d_term) / 10.0f;

    
    
    // ��������޷����ؼ��޸ģ�
    #define MAX_CONTROL_OUTPUT 24.0f  // ���ݵ�����ת�ٵ���
    if(control_output > MAX_CONTROL_OUTPUT) 
        control_output = MAX_CONTROL_OUTPUT;
    else if(control_output < -MAX_CONTROL_OUTPUT)
        control_output = -MAX_CONTROL_OUTPUT;
    
    
    
    
    
    // 4. �ٶȺϳ�
    Motor_Left_pid.set_value[0] = cfg.base_speed + control_output;
    Motor_Right_pid.set_value[0] = cfg.base_speed - control_output;

   

    // 6. ����״̬
    last_error = current_error;
}



int jueduizhi(int data) //ȡ����ֵ
{
  int data_jueduizhi;
  if(data<0) return -data_jueduizhi;
  else return data_jueduizhi;

}


























// 3. ������ȷ����������ɷ���ImageProcess�л򵥶�������
void Width_Analysis(void) {
    uint8 continuous_wide_cnt = 0;
    uint8 current_width=0;
    // ɨ��ؼ��У�����30-50�У�
 
   for(uint8 y = IPSH - 1; y > 30; y--)

   {
       current_width = CountWhitePixelsInRow( y);

        if(current_width > ImageParameter.width_threshold) {
    continuous_wide_cnt++;
    if(continuous_wide_cnt > 5){ 
        ImageParameter.width_flag = 1;
        ImageParameter.width_stable_cnt = 0;
        break;
    }
} else {
    continuous_wide_cnt = 0;
    ImageParameter.width_stable_cnt++;
    ImageParameter.width_flag = 0;
}
    }
   wireless_printf("%d,%d\n",current_width,continuous_wide_cnt);
 // wireless_printf("%d,%d,%.1f,%.1f\n",current_width,continuous_wide_cnt,left_slope1,right_slope1);
   
}


  
    





// �ײ�(����5��)���в�(�м�10��)��ǰ��(����10��)����-��߽�ƽ��ֵ
void Get_RegionalWidthDiff(float *bottom, float *middle, float *front) 
{
    uint8 off = ImageParameter.OFFLine;
    int sum_b = 0, cnt_b = 0;
    int sum_m = 0, cnt_m = 0;
    int sum_f = 0, cnt_f = 0;

    // �ײ���off ~ off+4�У�����5�У���ͷǰ0-1.5cm��
    for(int y=off; y<=off+4 && y<55; y++) {
        sum_b += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_b++;
    }
    
    // �в���off+5 ~ off+14�У��м�10�У�1.5-4cm��
    for(int y=off+5; y<=off+14 && y<55; y++) {
        sum_m += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_m++;
    }
    
    // ǰ�ˣ�off+15 ~ 55�У�����10�У�4-10cm��
    for(int y=off+15; y<=55; y++) {
        sum_f += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_f++;
    }

    // ����ƽ��ֵ��������ʱ����0��
    *bottom = cnt_b ? (float)sum_b/cnt_b : 0;
    *middle = cnt_m ? (float)sum_m/cnt_m : 0;
    *front  = cnt_f ? (float)sum_f/cnt_f : 0;
}




/*
/?**?
 ? @brief ʵʱ�������б��
 ? @param dir ��ⷽ��'L'��߽�/'R'�ұ߽�
 ? @param start_row ��ʼ�кţ�����55-59��
 ? @param end_row �����кţ�����>=start_row-10��
 ? @return б��ֵ������/�У���������ʾ������չ��������ʾ��������
 */

// �ȶ��徲̬����
static int16_t get_left_border(uint8 y) { return RowAttribute[y].LeftBorder; }
static int16_t get_right_border(uint8 y) { return RowAttribute[y].RightBorder; }
float RealTime_Slope_Detection(uint8 dir, uint8 start_row, uint8 end_row) 
{
    // ����У��
    if(start_row < end_row || start_row > 59 || end_row < 5) {
        return 0.0f; // ��Ч��������0б��
    }

    // ѡ����߽�
    int16_t (*get_border)(uint8) = NULL;
    switch(dir) {
    case 'L': get_border = get_left_border; break;
    case 'R': get_border = get_right_border; break;
}

    // ��̬ѡ���������������ܼ�������
    uint8 sample_step = (start_row - end_row > 15) ? 2 : 1;
    
    // ��С���˷�����б��
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    uint8 valid_samples = 0;
    
    for(uint8 y = start_row; y >= end_row; y -= sample_step) {
        if((dir == 'L' && RowAttribute[y].IsLeftFind == 'T') || 
           (dir == 'R' && RowAttribute[y].IsRightFind == 'T')) {
            float rel_y = (float)(start_row - y);
            float border = (float)get_border(y);
            
            sum_x += rel_y;
            sum_y += border;
            sum_xy += rel_y * border;
            sum_xx += rel_y * rel_y;
            valid_samples++;
        }
    }

    // ��Ҫ����3����Ч����ܼ���б��
    if(valid_samples < 3) {
        return 0.0f;
    }

    // ����б�� k = (n��xy - ��x��y) / (n��x2 - (��x)2)
    float n = (float)valid_samples;
    float denominator = n * sum_xx - sum_x * sum_x;
    if(fabs(denominator) < 1e-5) {
        return 0.0f; // ��ֹ����0
    }
    
    return (n * sum_xy - sum_x * sum_y) / denominator;
    
    
}





 


uint32 show_count=0;
uint32 start_time=0;
uint8 TRACK_VERTICAL_delay_flag=0;
uint32 delay_time=80;
//------------------------- ԭTrack_Line�����޸Ĳ��� -------------------------
void Track_Line(void) //�����ж�
{
   // ������ȷ���
    Width_Analysis(); 
    
    // ���ݿ�ȱ�־ִ�п����߼�
    if(ImageParameter.width_flag) {
        Track_Line_Num=TRACK_VERTICAL;
    }else Track_Line_Num=DEFAULT;
    
   if(Track_Line_Num==TRACK_VERTICAL)
   {
       TRACK_VERTICAL_delay_flag=1;
   }
    if(TRACK_VERTICAL_delay_flag==1)
    {
      Track_Line_Num=TRACK_VERTICAL;
    }
    
    Track_Control_Core(Track_Line_Num);
    
    /*
    if(Track_Line_Num==TRACK_VERTICAL)
    {
     start_time=get_system_time();
     TRACK_VERTICAL_delay_flag=1;
    }
 if((Track_Line_Num!=TRACK_VERTICAL)&&TRACK_VERTICAL_delay_flag==1)
 {
   if(get_system_time()-start_time<delay_time)
   {
   Track_Control_Core(TRACK_VERTICAL);
   }
   if(get_system_time()-start_time>=delay_time)
   {
     Track_Control_Core(Track_Line_Num);
     TRACK_VERTICAL_delay_flag=0;
   }
 }
 if(Track_Line_Num==TRACK_VERTICAL&&TRACK_VERTICAL_delay_flag==1)//��������0.2sʱ 
 {
 if(get_system_time()-start_time>=delay_time)
   {
     Track_Control_Core(Track_Line_Num);
     TRACK_VERTICAL_delay_flag=0;
   }
 
 }
 
 
 if((Track_Line_Num!=TRACK_VERTICAL)&&TRACK_VERTICAL_delay_flag==0) Track_Control_Core(Track_Line_Num);
    */
  // Track_Control_Core(Track_Line_Num);
  // wireless_printf("%d\n",TRACK_VERTICAL_delay_flag);

}



//������
void fengmingqi()
{
  gpio_init(P23_7, GPO, GPIO_LOW, GPO_PUSH_PULL);              // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
  switch(Track_Line_Num) {
    case 0: BELL_OFF; break;
    case 1: BELL_ON; break;
   
   

}

}


float offLineCenter,offerror;
float left_slope1,right_slope1;
void Control()    //����5ms�ж�����
{
  
   Track_Line();      //�����ж�
   
  fengmingqi();       //������
   // ��ͼ���������е���
  //  left_slope1 = RealTime_Slope_Detection('L', 55, 35);
   // right_slope1 = RealTime_Slope_Detection('R',55, 35);
   
    
 // 2. ��ȡ�����ȼ����������ͣ�����Ĭ�ϲ��ԣ�
  //  TrackType current_track = Track_Line_Num;
   // Track_Control_Core(Track_Line_Num);
   
   
    
    
     
     Speed_Measure();   //������
     start_control();
     //Motor_Pid_Caculate(&Motor_Left_pid);
    // Motor_Pid_Caculate(&Motor_Right_pid);
     //Motor_Control(Motor_Left_pid.output, Motor_Right_pid.output);
   
   //�ý������϶˱任�����Ϊ�ж�����������д
   

}
