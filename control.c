#include "zf_common_headfile.h"

//1 直线   2 小折线  3 垂直线
uint8  Track_Line_Num=0;
int sum,i;
uint8 pid_set = 2;    //选择是模糊pid 或 位置式pid
uint8_t jiezhihang_change=12;  //截至行增加量
uint8 Point_Mid; //截至行中线值
int8_t Media_line_difference=0;
Direct_pid_info Direct_pid;
int8_t piancha,last_piancha; //用于判断路况的偏量
Direct_pid_info zhi_pid;          //长直道pid1
Direct_pid_info zhexian_pid;          //小折线pid1
Direct_pid_info chuizhi_pid;       //垂直线pid1
Direct_pid_info angle_pid;       //角度环PID
float GKD=0;

int current_error;


// 微缩赛道专属参数（2cm白线）

#define KD_SCALE               1.5f // 微缩车低惯性，增强微分作用
// 3. 正确初始化（注意逗号和C99支持）
TrackPIDConfig track_configuration[] = {
    [DEFAULT] = {.kp = 3.8f, .kd = 35.0f*KD_SCALE, .base_speed = 20},  // 末尾必须有逗号
  /*  15，35，18
    3.2，35，35
    3.8  35  20低速很好跑
    3.8，35，45目前速度最快过折线和直线
    40.8，189.0，45可以过直角*/
    [TRACK_VERTICAL] = {.kp = 40.8f, .kd = 189.0f*KD_SCALE, .base_speed = 20},
};
 
//------------------------- 控制核心 -------------------------
void Track_Control_Core(TrackType track_type) {
  
  
    static int last_error = 0;
    
    // 1. 获取赛道偏差
    
    if(track_type==TRACK_VERTICAL)
    {
      current_error=CalculateOFFLineCenter()-PictureCentring;
    }else{
       current_error = ImageParameter.Det_True - PictureCentring;
    }
   
    // 2. 选择PID参数
    TrackPIDConfig cfg = track_configuration[track_type];
    
    // 3. 计算控制量
    float p_term = cfg.kp * current_error;
    float d_term = cfg.kd * (current_error - last_error);
    float control_output = (p_term + d_term) / 10.0f;

    
    
    // 新增输出限幅（关键修改）
    #define MAX_CONTROL_OUTPUT 24.0f  // 根据电机最大转速调整
    if(control_output > MAX_CONTROL_OUTPUT) 
        control_output = MAX_CONTROL_OUTPUT;
    else if(control_output < -MAX_CONTROL_OUTPUT)
        control_output = -MAX_CONTROL_OUTPUT;
    
    
    
    
    
    // 4. 速度合成
    Motor_Left_pid.set_value[0] = cfg.base_speed + control_output;
    Motor_Right_pid.set_value[0] = cfg.base_speed - control_output;

   

    // 6. 更新状态
    last_error = current_error;
}



int jueduizhi(int data) //取绝对值
{
  int data_jueduizhi;
  if(data<0) return -data_jueduizhi;
  else return data_jueduizhi;

}


























// 3. 新增宽度分析函数（可放在ImageProcess中或单独函数）
void Width_Analysis(void) {
    uint8 continuous_wide_cnt = 0;
    uint8 current_width=0;
    // 扫描关键行（例如30-50行）
 
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


  
    





// 底部(近端5行)、中部(中间10行)、前端(顶部10行)的右-左边界平均值
void Get_RegionalWidthDiff(float *bottom, float *middle, float *front) 
{
    uint8 off = ImageParameter.OFFLine;
    int sum_b = 0, cnt_b = 0;
    int sum_m = 0, cnt_m = 0;
    int sum_f = 0, cnt_f = 0;

    // 底部：off ~ off+4行（近端5行，车头前0-1.5cm）
    for(int y=off; y<=off+4 && y<55; y++) {
        sum_b += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_b++;
    }
    
    // 中部：off+5 ~ off+14行（中间10行，1.5-4cm）
    for(int y=off+5; y<=off+14 && y<55; y++) {
        sum_m += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_m++;
    }
    
    // 前端：off+15 ~ 55行（顶部10行，4-10cm）
    for(int y=off+15; y<=55; y++) {
        sum_f += RowAttribute[y].RightBorder - RowAttribute[y].LeftBorder;
        cnt_f++;
    }

    // 计算平均值（无数据时返回0）
    *bottom = cnt_b ? (float)sum_b/cnt_b : 0;
    *middle = cnt_m ? (float)sum_m/cnt_m : 0;
    *front  = cnt_f ? (float)sum_f/cnt_f : 0;
}




/*
/?**?
 ? @brief 实时计算白线斜率
 ? @param dir 检测方向：'L'左边界/'R'右边界
 ? @param start_row 起始行号（建议55-59）
 ? @param end_row 结束行号（建议>=start_row-10）
 ? @return 斜率值（像素/行），正数表示向外扩展，负数表示向内收缩
 */

// 先定义静态函数
static int16_t get_left_border(uint8 y) { return RowAttribute[y].LeftBorder; }
static int16_t get_right_border(uint8 y) { return RowAttribute[y].RightBorder; }
float RealTime_Slope_Detection(uint8 dir, uint8 start_row, uint8 end_row) 
{
    // 参数校验
    if(start_row < end_row || start_row > 59 || end_row < 5) {
        return 0.0f; // 无效参数返回0斜率
    }

    // 选择检测边界
    int16_t (*get_border)(uint8) = NULL;
    switch(dir) {
    case 'L': get_border = get_left_border; break;
    case 'R': get_border = get_right_border; break;
}

    // 动态选择采样间隔（近端密集采样）
    uint8 sample_step = (start_row - end_row > 15) ? 2 : 1;
    
    // 最小二乘法计算斜率
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

    // 需要至少3个有效点才能计算斜率
    if(valid_samples < 3) {
        return 0.0f;
    }

    // 计算斜率 k = (nΣxy - ΣxΣy) / (nΣx2 - (Σx)2)
    float n = (float)valid_samples;
    float denominator = n * sum_xx - sum_x * sum_x;
    if(fabs(denominator) < 1e-5) {
        return 0.0f; // 防止除以0
    }
    
    return (n * sum_xy - sum_x * sum_y) / denominator;
    
    
}





 


uint32 show_count=0;
uint32 start_time=0;
uint8 TRACK_VERTICAL_delay_flag=0;
uint32 delay_time=80;
//------------------------- 原Track_Line函数修改部分 -------------------------
void Track_Line(void) //赛道判断
{
   // 新增宽度分析
    Width_Analysis(); 
    
    // 根据宽度标志执行控制逻辑
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
 if(Track_Line_Num==TRACK_VERTICAL&&TRACK_VERTICAL_delay_flag==1)//正常大于0.2s时 
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



//蜂鸣器
void fengmingqi()
{
  gpio_init(P23_7, GPO, GPIO_LOW, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式
  switch(Track_Line_Num) {
    case 0: BELL_OFF; break;
    case 1: BELL_ON; break;
   
   

}

}


float offLineCenter,offerror;
float left_slope1,right_slope1;
void Control()    //放在5ms中断里面
{
  
   Track_Line();      //赛道判断
   
  fengmingqi();       //蜂鸣器
   // 在图像处理流程中调用
  //  left_slope1 = RealTime_Slope_Detection('L', 55, 35);
   // right_slope1 = RealTime_Slope_Detection('R',55, 35);
   
    
 // 2. 获取带优先级的赛道类型（新增默认策略）
  //  TrackType current_track = Track_Line_Num;
   // Track_Control_Core(Track_Line_Num);
   
   
    
    
     
     Speed_Measure();   //编码器
     start_control();
     //Motor_Pid_Caculate(&Motor_Left_pid);
    // Motor_Pid_Caculate(&Motor_Right_pid);
     //Motor_Control(Motor_Left_pid.output, Motor_Right_pid.output);
   
   //用截至行上端变换情况作为判断条件，明天写
   

}
