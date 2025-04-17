#include "zf_common_headfile.h"
#include  "motor.h"

/*******************/
#define MOTOR_MAX 9000
/*************************   �ٶȻ�pid    ******************************************/
Motor_pid_info Motor_Left_pid;   //���PID
Motor_pid_info Motor_Right_pid;  //���PID

int16  left_pulse_temp[5]  =   {0};           //�����������������
int16  right_pulse_temp[5] =   {0};           //�ұ���������������
uint32 right_mc , left_mc=0;
int32 maichong=0;
uint8 right_mc_flag = 0, left_mc_flag = 0 ,maichong_flag = 0;
//uint8 right_mc_flag = 0, left_mc_flag = 0 ,maichong_flag = 0;

int16 L_outPWM=0 , R_outPWM=0;//����ٶ�
uint32 bianmaqi_left_speed,bianmaqi_right_speed;
uint8  suduhuan_flag=0;//�����ٶȻ�����



// 3000�ٶȶ�Ӧ pid 0.005 0.08 0.05 

/***************�����������ʼ��*********************/
void Motor_Init(void)  //��ʼ��1
{
 encoder_dir_init(DIRECTION_ENCODER_left, DIRECTION_ENCODER_A_left, DIRECTION_ENCODER_B_left);
 encoder_dir_init(DIRECTION_ENCODER_right, DIRECTION_ENCODER_A_right, DIRECTION_ENCODER_B_right);
// ʹ��ʾ��     pwm_init(TCPWM_CH14_P00_2, 50, 1000);   // ATOM 0ģ���ͨ��7 ʹ��P02_7�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮1000/PWM_DUTY_MAX*100
  //encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P7_6, TC_CH07_ENCODER_CH2_P7_7);// 
                                          

    gpio_init(PWM_RD1, GPO, GPIO_LOW, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_R1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    gpio_init(PWM_LD1, GPO, GPIO_LOW, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_L1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    //gpio_init(PWM_RD2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
   // pwm_init(PWM_R2, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
   // gpio_init(PWM_LD2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    //pwm_init(PWM_L2, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    
    
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

/************************* ���������� ***********************/
void Speed_Measure()
{
   
  static uint8 first_sample = 0;
    // �״β���ʱ��ջ�����
    if(first_sample < 5) {
        memset(left_pulse_temp, 0, sizeof(left_pulse_temp));
        memset(right_pulse_temp, 0, sizeof(right_pulse_temp));
        first_sample++;
    }
  
  
  
    maichong_flag=1;  //����������
    //�������
    left_pulse_temp[0] = encoder_get_count(DIRECTION_ENCODER_left);                            // �ɼ���Ӧ����������
    encoder_clear_count(DIRECTION_ENCODER_left);                                                   // �����Ӧ����
    Motor_Left_pid.present_value[0] = (left_pulse_temp[0]+left_pulse_temp[1]+left_pulse_temp[2]+left_pulse_temp[3]+left_pulse_temp[4])/5;

    
    //�ұ�����
    right_pulse_temp[0] = encoder_get_count(DIRECTION_ENCODER_right);                           // �ɼ���Ӧ����������
    encoder_clear_count(DIRECTION_ENCODER_right);
    Motor_Right_pid.present_value[0] = -(right_pulse_temp[0]+right_pulse_temp[1]+right_pulse_temp[2]+right_pulse_temp[3]+right_pulse_temp[4])/5;
 
    //���±���������
    left_pulse_temp[4]  = left_pulse_temp[3];
    left_pulse_temp[3]  = left_pulse_temp[2];
    left_pulse_temp[2]  = left_pulse_temp[1];
    left_pulse_temp[1]  = left_pulse_temp[0];

    right_pulse_temp[4] = right_pulse_temp[3];
    right_pulse_temp[3] = right_pulse_temp[2];
    right_pulse_temp[2] = right_pulse_temp[1];
    right_pulse_temp[1] = right_pulse_temp[0];

    if(right_mc_flag)   //������
        right_mc += Motor_Right_pid.present_value[0];
    if(!right_mc_flag)
        right_mc = 0 ;

    if(left_mc_flag)    //������
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
    //�ٶȻ�pid  0.002   0.15  0.01   
    Motor_Right_pid.kp  =  30;//EEPROM_READ_WORD(uint32,R_Mot_P)/10.0f;       180            //��ȡEEPROM�е�R_Mot_P
    Motor_Right_pid.ki  =  7.5;//EEPROM_READ_WORD(uint32,R_Mot_I)/100.0f;       40           //��ȡEEPROM�е�R_Mot_D
    Motor_Right_pid.kd  =  100;//EEPROM_READ_WORD(uint32,R_Mot_D)/10.0f;    477    20        //��ȡEEPROM�е�R_Dir_P
    Motor_Left_pid.kp   =  45;//EEPROM_READ_WORD(uint32,L_Mot_P)/10.0f;                 //��ȡEEPROM�е�R_Mot_P        0.12
    Motor_Left_pid.ki   =  7.5;//EEPROM_READ_WORD(uint32,L_Mot_I)/100.0f;                   //��ȡEEPROM�е�R_Mot_D 0.1702       0.1680    0.155
    Motor_Left_pid.kd   =  135;//EEPROM_READ_WORD(uint32,L_Mot_D)/10.0f;    350           //��ȡEEPROM�е�R_Dir_P    0.01
     //45   10  100
    //30 7.5 100
    //50  6  40
  //4.5    0.6   40   36 4 40
    //8    3  1   262        
    //����100�ٶ�  8 1.1  50      0 100 0  
    //����200�ٶ�  1 500  0.8    5  2  62.5 
}



/***********���PID�������***********/
void Motor_Pid_Caculate(Motor_pid_info *motor_info)
{
    float delta_uk = 0;//��ǰʵ��Ӧ�����ռ�ձȣ���delta_duty��last_uk
    float out_duty = 0;
    uint8 i = 0;

    for(i=9;i>0;i--)     //����ƫ�����
        motor_info->error[i] = motor_info->error[i-1];

    motor_info->error[0] = motor_info->set_value[0]-motor_info->present_value[0];

    motor_info->speed_racc = 0;   //���������ٶ�,���ٶ�΢��
    for(i=0;i<1;i++)
        motor_info->speed_racc += motor_info->present_value[i];
    for(i=1;i<2;i++)
        motor_info->speed_racc -= motor_info->present_value[i];
 
     delta_uk = (float)(motor_info->kvff * motor_info->kvff_param * (motor_info->set_value[0] - motor_info->set_value[1])
                  + motor_info->kp * (motor_info->error[0] - motor_info->error[1])
                  + motor_info->ki * motor_info->error[0]
                  + motor_info->kd * (motor_info->error[0] - 2.0f * motor_info->error[1] + motor_info->error[2])
                  - motor_info->kaff * motor_info->speed_racc * 0.10f);


       
     
     
    /*------------------------�������������ֱ��͵ȴ���-----------------------*/
    if(motor_info->last_uk > MOTOR_MAX  /*||  motor_info->last_uk - Direct_pid.speed   > 1*/)  //��Χ-1000~1000
        if(delta_uk > 0)
            delta_uk = 0;//��ǰһʱ�̿�����(ռ�ձ�)�Ѿ��ﵽ���ʱ������������Ϊ�����ۼ�
    if(motor_info->last_uk < -MOTOR_MAX  /*||  motor_info->last_uk - Direct_pid.speed   < -1*/)//MOTOR_DUTY_LIMIT
        if(delta_uk < 0)
            delta_uk = 0;//��ǰһʱ�̿�����(ռ�ձ�)�Ѿ��ﵽ���ʱ������������Ϊ�����ۼ�
    /*------------------------�������������ֱ��͵ȴ������-------------------*/

  
    
    
    
    out_duty = motor_info->last_uk + delta_uk;//��ǰ��Ҫ�����ʵ��ռ�ձ�

    /*-------------------------���� ʵ�����������ķ�ֵ--------------------*/
    if (out_duty > MOTOR_MAX)
        out_duty = MOTOR_MAX;
    else if (out_duty < -9000)
        out_duty = -9000;
    /*-------------------------����ʵ�����������ķ�ֵ����------------------*/

    motor_info->last_uk = out_duty;               //������һ�ε�ʵ�ʿ��������
    motor_info->output =(int16)(out_duty);     //��ǰռ�ձ����

    for(i = 9;i > 0;i--)  //�����ٶ��趨ֵ����
        motor_info->set_value[i] = motor_info->set_value[i-1];

    for(i= 9;i>0;i--)    //����ʵ���ٶȶ���
        motor_info->present_value[i] = motor_info->present_value[i-1];
}

uint32 p;
// ���ܿ��Ʋ���

uint16 start_speed = 900;    // ���ܽ׶ι̶��ٶ�
uint8 start_state = STATE_INIT;   // ��ʼ״̬



void Motor_PID_Reset(Motor_pid_info *pid)
{
    // ���������ز���
    pid->output = 0;           // ����ֹͣ���
    pid->last_uk = 0.0f;       // �����ʷ������
    pid->deta_uk = 0.0f;       // ���ÿ�������
    pid->out_duty = 0.0f;      // ����ռ�ձ����
    
    // �����˶�״̬����
    pid->speed_racc = 0;       // ���ٶȹ���
    
    // ������ݶ��У�ʹ��memset�Ż���
    memset(pid->present_value, 0, sizeof(pid->present_value));  // ������ֵ����
    memset(pid->error, 0, sizeof(pid->error));                  // ������
    memset(pid->set_value, 0, sizeof(pid->set_value));           // �趨ֵ����
    
    // �����ؼ�ϵ����kp/ki/kd�Ȳ����ã�
    // ע�⣺ǰ��ϵ������ԭֵ 
    // pid->kp = 0.0f;  // ��ֹ���ÿ��Ʋ�����
    // pid->ki = 0.0f;
    // pid->kd = 0.0f;
    
    // ��ѡ������������
    //pid->kvff = pid->kvff_param;  // �ָ�ǰ��ϵ��Ĭ��ֵ
}





void start_control()
{
    static uint32_t start_timestamp = 0;
    static uint32_t stop_timestamp = 0;
    
    switch(start_state){
      
        case STATE_INIT://����ʱ���ԭ������
            Motor_Control(0, 0);       // ǿ�����0ռ�ձ�
            Motor_PID_Reset(&Motor_Left_pid);
            Motor_PID_Reset(&Motor_Right_pid);
            start_state = STATE_WAIT_STOP;
            break;
    
            
           
      
        case STATE_WAIT_STOP:
            // ����Ƿ���ȫֹͣ
            if(abs(Motor_Left_pid.present_value[0]) < STOP_SPEED_THRESHOLD && 
               abs(Motor_Right_pid.present_value[0]) < STOP_SPEED_THRESHOLD)
            {
                // �������ܽ׶�
                start_state = STATE_STARTUP;
                start_timestamp = get_system_time(); // ��¼����ʱ��
            }
            break;
            
        case STATE_STARTUP: {
           // ������������ٶ�
           Motor_Control(start_speed, start_speed);
            
            // ����ʱ����
            if(get_system_time() - start_timestamp >= START_DURATION){
                // �л�����������
                start_state = STATE_RUNNING;
            }
        }
            break;
            
        case STATE_RUNNING:
            // ����PID����
            Motor_Pid_Caculate(&Motor_Left_pid);
            Motor_Pid_Caculate(&Motor_Right_pid);
            Motor_Control(Motor_Left_pid.output, Motor_Right_pid.output);
            
            // ��������Ƿ�ֹͣ
            if(abs(Motor_Left_pid.present_value[0]) < STOP_SPEED_THRESHOLD && 
               abs(Motor_Right_pid.present_value[0]) < STOP_SPEED_THRESHOLD)
            {
                start_state = STATE_STOP_DETECT;
                stop_timestamp = get_system_time(); // ��¼ֹͣ��⿪ʼʱ��
            }
            break;
            
        case STATE_STOP_DETECT:
            // ά��ֹͣ״̬���
            if(abs(Motor_Left_pid.present_value[0]) >= STOP_SPEED_THRESHOLD || 
               abs(Motor_Right_pid.present_value[0]) >= STOP_SPEED_THRESHOLD)
            {
                // ��������˶��򷵻�����״̬
                start_state = STATE_RUNNING;
            }
            else if(get_system_time() - stop_timestamp >= STOP_CONFIRM_TIME){
                // ȷ����ȫֹͣ����������
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
 
   start_state = 1; // ��λ״̬��
   
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
 

