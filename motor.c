#include "zf_common_headfile.h"
#include  "motor.h"

/*******************/
#define MOTOR_MAX 9000
/*************************   速度环pid    ******************************************/
Motor_pid_info Motor_Left_pid;   //电机PID
Motor_pid_info Motor_Right_pid;  //电机PID

int16  left_pulse_temp[5]  =   {0};           //左编码器读数缓冲区
int16  right_pulse_temp[5] =   {0};           //右编码器读数缓冲区
uint32 right_mc , left_mc=0;
int32 maichong=0;
uint8 right_mc_flag = 0, left_mc_flag = 0 ,maichong_flag = 0;
//uint8 right_mc_flag = 0, left_mc_flag = 0 ,maichong_flag = 0;

int16 L_outPWM=0 , R_outPWM=0;//电机速度
uint32 bianmaqi_left_speed,bianmaqi_right_speed;
uint8  suduhuan_flag=0;//控制速度环开启



// 3000速度对应 pid 0.005 0.08 0.05 

/***************电机编码器初始化*********************/
void Motor_Init(void)  //初始化1
{
 encoder_dir_init(DIRECTION_ENCODER_left, DIRECTION_ENCODER_A_left, DIRECTION_ENCODER_B_left);
 encoder_dir_init(DIRECTION_ENCODER_right, DIRECTION_ENCODER_A_right, DIRECTION_ENCODER_B_right);
// 使用示例     pwm_init(TCPWM_CH14_P00_2, 50, 1000);   // ATOM 0模块的通道7 使用P02_7引脚输出PWM  PWM频率50HZ  占空比百分之1000/PWM_DUTY_MAX*100
  //encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P7_6, TC_CH07_ENCODER_CH2_P7_7);// 
                                          

    gpio_init(PWM_RD1, GPO, GPIO_LOW, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(PWM_LD1, GPO, GPIO_LOW, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0

    //gpio_init(PWM_RD2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
   // pwm_init(PWM_R2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
   // gpio_init(PWM_LD2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    //pwm_init(PWM_L2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0

    
    
   /* pwm_stop(PWM_L1);
    pwm_stop(PWM_R1);
    pwm_set_duty(PWM_L1, 0);
    pwm_set_duty(PWM_R1, 0);
    pwm_start(PWM_L1);
    pwm_start(PWM_R1);
    
    Motor_PID_Reset(&Motor_Left_pid);
    Motor_PID_Reset(&Motor_Right_pid);*/
    
    
   
  
}

int16 range_protect_motor(int16 x,int16 max ,int16 min)
{
    if(x>=max)
        return max;
    else if(x<=min)
        return min;
    else
      return x;
}

/************************* 编码器数据 ***********************/
void Speed_Measure()
{
   
  static uint8 first_sample = 0;
    // 首次采样时清空缓冲区
    if(first_sample < 5) {
        memset(left_pulse_temp, 0, sizeof(left_pulse_temp));
        memset(right_pulse_temp, 0, sizeof(right_pulse_temp));
        first_sample++;
    }
  
  
  
    maichong_flag=1;  //开启脉冲检测
    //左编码器
    left_pulse_temp[0] = encoder_get_count(DIRECTION_ENCODER_left);                            // 采集对应编码器数据
    encoder_clear_count(DIRECTION_ENCODER_left);                                                   // 清除对应计数
    Motor_Left_pid.present_value[0] = (left_pulse_temp[0]+left_pulse_temp[1]+left_pulse_temp[2]+left_pulse_temp[3]+left_pulse_temp[4])/5;

    
    //右编码器
    right_pulse_temp[0] = encoder_get_count(DIRECTION_ENCODER_right);                           // 采集对应编码器数据
    encoder_clear_count(DIRECTION_ENCODER_right);
    Motor_Right_pid.present_value[0] = -(right_pulse_temp[0]+right_pulse_temp[1]+right_pulse_temp[2]+right_pulse_temp[3]+right_pulse_temp[4])/5;
 
    //更新编码器队列
    left_pulse_temp[4]  = left_pulse_temp[3];
    left_pulse_temp[3]  = left_pulse_temp[2];
    left_pulse_temp[2]  = left_pulse_temp[1];
    left_pulse_temp[1]  = left_pulse_temp[0];

    right_pulse_temp[4] = right_pulse_temp[3];
    right_pulse_temp[3] = right_pulse_temp[2];
    right_pulse_temp[2] = right_pulse_temp[1];
    right_pulse_temp[1] = right_pulse_temp[0];

    if(right_mc_flag)   //记脉冲
        right_mc += Motor_Right_pid.present_value[0];
    if(!right_mc_flag)
        right_mc = 0 ;

    if(left_mc_flag)    //记脉冲
        left_mc += Motor_Left_pid.present_value[0];
    if(!left_mc_flag)
        left_mc = 0 ;
    if(maichong_flag)
        maichong += (Motor_Right_pid.present_value[0] + Motor_Right_pid.present_value[0])/2;
    if(!maichong_flag)
        maichong = 0 ;

}
//1  500 0.8   
void Speed_control_init()
{
    
    Motor_Right_pid.kvff = 1;
    Motor_Right_pid.kaff = 1;
    Motor_Right_pid.kvff_param = 0.8;
    Motor_Left_pid.kvff  =1;
    Motor_Left_pid.kaff  =1;
    Motor_Left_pid.kvff_param = 0.8;
    //速度环pid  0.002   0.15  0.01   
    Motor_Right_pid.kp  =  30;//EEPROM_READ_WORD(uint32,R_Mot_P)/10.0f;       180            //获取EEPROM中的R_Mot_P
    Motor_Right_pid.ki  =  7.5;//EEPROM_READ_WORD(uint32,R_Mot_I)/100.0f;       40           //获取EEPROM中的R_Mot_D
    Motor_Right_pid.kd  =  100;//EEPROM_READ_WORD(uint32,R_Mot_D)/10.0f;    477    20        //获取EEPROM中的R_Dir_P
    Motor_Left_pid.kp   =  45;//EEPROM_READ_WORD(uint32,L_Mot_P)/10.0f;                 //获取EEPROM中的R_Mot_P        0.12
    Motor_Left_pid.ki   =  7.5;//EEPROM_READ_WORD(uint32,L_Mot_I)/100.0f;                   //获取EEPROM中的R_Mot_D 0.1702       0.1680    0.155
    Motor_Left_pid.kd   =  135;//EEPROM_READ_WORD(uint32,L_Mot_D)/10.0f;    350           //获取EEPROM中的R_Dir_P    0.01
     //45   10  100
    //30 7.5 100
    //50  6  40
  //4.5    0.6   40   36 4 40
    //8    3  1   262        
    //适配100速度  8 1.1  50      0 100 0  
    //适配200速度  1 500  0.8    5  2  62.5 
}



/***********电机PID输出计算***********/
void Motor_Pid_Caculate(Motor_pid_info *motor_info)
{
    float delta_uk = 0;//当前实际应输出的占空比，是delta_duty加last_uk
    float out_duty = 0;
    uint8 i = 0;

    for(i=9;i>0;i--)     //更新偏差队列
        motor_info->error[i] = motor_info->error[i-1];

    motor_info->error[0] = motor_info->set_value[0]-motor_info->present_value[0];

    motor_info->speed_racc = 0;   //计算电机加速度,即速度微分
    for(i=0;i<1;i++)
        motor_info->speed_racc += motor_info->present_value[i];
    for(i=1;i<2;i++)
        motor_info->speed_racc -= motor_info->present_value[i];
 
     delta_uk = (float)(motor_info->kvff * motor_info->kvff_param * (motor_info->set_value[0] - motor_info->set_value[1])
                  + motor_info->kp * (motor_info->error[0] - motor_info->error[1])
                  + motor_info->ki * motor_info->error[0]
                  + motor_info->kd * (motor_info->error[0] - 2.0f * motor_info->error[1] + motor_info->error[2])
                  - motor_info->kaff * motor_info->speed_racc * 0.10f);


       
     
     
    /*------------------------遇限削弱抗积分饱和等处理-----------------------*/
    if(motor_info->last_uk > MOTOR_MAX  /*||  motor_info->last_uk - Direct_pid.speed   > 1*/)  //范围-1000~1000
        if(delta_uk > 0)
            delta_uk = 0;//当前一时刻控制量(占空比)已经达到最大时，若现在增量为正则不累加
    if(motor_info->last_uk < -MOTOR_MAX  /*||  motor_info->last_uk - Direct_pid.speed   < -1*/)//MOTOR_DUTY_LIMIT
        if(delta_uk < 0)
            delta_uk = 0;//当前一时刻控制量(占空比)已经达到最大时，若现在增量为正则不累加
    /*------------------------遇限削弱抗积分饱和等处理结束-------------------*/

  
    
    
    
    out_duty = motor_info->last_uk + delta_uk;//当前需要输出的实际占空比

    /*-------------------------限制 实际最后总输出的幅值--------------------*/
    if (out_duty > MOTOR_MAX)
        out_duty = MOTOR_MAX;
    else if (out_duty < -9000)
        out_duty = -9000;
    /*-------------------------限制实际最后总输出的幅值结束------------------*/

    motor_info->last_uk = out_duty;               //更新上一次的实际控制量输出
    motor_info->output =(int16)(out_duty);     //当前占空比输出

    for(i = 9;i > 0;i--)  //更新速度设定值队列
        motor_info->set_value[i] = motor_info->set_value[i-1];

    for(i= 9;i>0;i--)    //更新实测速度队列
        motor_info->present_value[i] = motor_info->present_value[i-1];
}

uint32 p;
// 起跑控制参数

uint16 start_speed = 900;    // 起跑阶段固定速度
uint8 start_state = STATE_INIT;   // 初始状态



void Motor_PID_Reset(Motor_pid_info *pid)
{
    // 清零输出相关参数
    pid->output = 0;           // 立即停止输出
    pid->last_uk = 0.0f;       // 清除历史控制量
    pid->deta_uk = 0.0f;       // 重置控制增量
    pid->out_duty = 0.0f;      // 清零占空比输出
    
    // 重置运动状态参数
    pid->speed_racc = 0;       // 加速度归零
    
    // 清空数据队列（使用memset优化）
    memset(pid->present_value, 0, sizeof(pid->present_value));  // 编码器值缓存
    memset(pid->error, 0, sizeof(pid->error));                  // 误差队列
    memset(pid->set_value, 0, sizeof(pid->set_value));           // 设定值队列
    
    // 保留关键系数（kp/ki/kd等不重置）
    // 注意：前馈系数保持原值 
    // pid->kp = 0.0f;  // 禁止重置控制参数！
    // pid->ki = 0.0f;
    // pid->kd = 0.0f;
    
    // 可选：软启动保护
    //pid->kvff = pid->kvff_param;  // 恢复前馈系数默认值
}





void start_control()
{
    static uint32_t start_timestamp = 0;
    static uint32_t stop_timestamp = 0;
    
    switch(start_state){
      
        case STATE_INIT://起跑时清空原有数据
            Motor_Control(0, 0);       // 强制输出0占空比
            Motor_PID_Reset(&Motor_Left_pid);
            Motor_PID_Reset(&Motor_Right_pid);
            start_state = STATE_WAIT_STOP;
            break;
    
            
           
      
        case STATE_WAIT_STOP:
            // 检测是否完全停止
            if(abs(Motor_Left_pid.present_value[0]) < STOP_SPEED_THRESHOLD && 
               abs(Motor_Right_pid.present_value[0]) < STOP_SPEED_THRESHOLD)
            {
                // 进入起跑阶段
                start_state = STATE_STARTUP;
                start_timestamp = get_system_time(); // 记录启动时刻
            }
            break;
            
        case STATE_STARTUP: {
           // 持续输出起跑速度
           Motor_Control(start_speed, start_speed);
            
            // 持续时间检测
            if(get_system_time() - start_timestamp >= START_DURATION){
                // 切换至正常运行
                start_state = STATE_RUNNING;
            }
        }
            break;
            
        case STATE_RUNNING:
            // 正常PID控制
            Motor_Pid_Caculate(&Motor_Left_pid);
            Motor_Pid_Caculate(&Motor_Right_pid);
            Motor_Control(Motor_Left_pid.output, Motor_Right_pid.output);
            
            // 持续检测是否停止
            if(abs(Motor_Left_pid.present_value[0]) < STOP_SPEED_THRESHOLD && 
               abs(Motor_Right_pid.present_value[0]) < STOP_SPEED_THRESHOLD)
            {
                start_state = STATE_STOP_DETECT;
                stop_timestamp = get_system_time(); // 记录停止检测开始时间
            }
            break;
            
        case STATE_STOP_DETECT:
            // 维持停止状态检测
            if(abs(Motor_Left_pid.present_value[0]) >= STOP_SPEED_THRESHOLD || 
               abs(Motor_Right_pid.present_value[0]) >= STOP_SPEED_THRESHOLD)
            {
                // 如果重新运动则返回运行状态
                start_state = STATE_RUNNING;
            }
            else if(get_system_time() - stop_timestamp >= STOP_CONFIRM_TIME){
                // 确认完全停止，重置流程
                start_state = STATE_WAIT_STOP;
            }
            break;
    }
}

void Motor_Control(int L_outPWM,int R_outPWM)
{
  
   // int Speed_Diff,Motor_Speed_Left,Motor_Speed_Right;
 
    if(Motor_Left_pid.output>9000)Motor_Left_pid.output=9000;
   // if(Motor_Left_pid.output<0)  Motor_Left_pid.output=0;
    if(Motor_Left_pid.output<-9000)  Motor_Left_pid.output=-9000;
    if(Motor_Right_pid.output>9000)  Motor_Right_pid.output=9000;
    if(Motor_Right_pid.output<-9000)  Motor_Right_pid.output=-9000;
   
 
   
   /*if(SystemData.Stop==1) 
   {
   Motor_Control(0, 0);
 
   start_state = 1; // 复位状态机
   
   }*/
   
   
   
   if(L_outPWM >= 0) {
       gpio_set_level(PWM_LD1, GPIO_LOW);
     pwm_set_duty(PWM_L1, L_outPWM);
  } else {
      gpio_set_level(PWM_LD1, GPIO_HIGH);
      pwm_set_duty(PWM_L1, -L_outPWM);
  }
   
   
   if(R_outPWM >= 0) {
       gpio_set_level(PWM_RD1, GPIO_LOW);
     pwm_set_duty(PWM_R1, R_outPWM);
  } else {
      gpio_set_level(PWM_RD1, GPIO_HIGH);
      pwm_set_duty(PWM_R1, -R_outPWM);
  }
   
   
   
       
 }
 

