#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"

#include "zf_common_headfile.h"


 #define MAX_DUTY            (30 )                                               // ��� MAX_DUTY% ռ�ձ�

 #define PWM_L1               (TCPWM_CH50_P18_7)        //���pwm
 #define PWM_LD1              (P18_6)  
 #define PWM_R1               (TCPWM_CH13_P00_3)        //���pwm
 #define PWM_RD1              (P00_2)                 //�жϷ���io��
 
             
  /*                                                                              
#define ENCODER_QUAD1                    (TC_CH09_ENCODER)                      // �������ӿ�  
#define ENCODER_QUAD1_PHASE_A            (TC_CH09_ENCODER_CH1_P05_0)            // PHASE_A ��Ӧ������                 
#define ENCODER_QUAD1_PHASE_B            (TC_CH09_ENCODER_CH2_P05_1)            // PHASE_B ��Ӧ������                   
                                                                                
#define ENCODER_QUAD2                    (TC_CH07_ENCODER)                      // �������ӿ�
#define ENCODER_QUAD2_PHASE_A            (TC_CH07_ENCODER_CH1_P02_0)            // PHASE_A ��Ӧ������
#define ENCODER_QUAD2_PHASE_B            (TC_CH07_ENCODER_CH2_P02_1)            // PHASE_B ��Ӧ������
*/
#define DIRECTION_ENCODER_left       (TC_CH09_ENCODER)                 
#define DIRECTION_ENCODER_A_left     (TC_CH09_ENCODER_CH1_P05_0)      
#define DIRECTION_ENCODER_B_left     (TC_CH09_ENCODER_CH2_P05_1)       
#define DIRECTION_ENCODER_right      (TC_CH07_ENCODER)                 
#define DIRECTION_ENCODER_A_right    (TC_CH07_ENCODER_CH1_P02_0)       
#define DIRECTION_ENCODER_B_right    (TC_CH07_ENCODER_CH2_P02_1)       





#define START_SPEED_THRESHOLD  10  // ��������ٶ���ֵ������ʵ�����������
#define START_TIMEOUT         500  // ������ʱʱ�䣨��λ��ѭ�����ڣ�

// ���ò���������ʵ�����������
#define STOP_SPEED_THRESHOLD 5     // ֹͣ�ж��ٶ���ֵ
#define STOP_CONFIRM_TIME   1000   // ֹͣȷ��ʱ�䣨ms��
#define START_DURATION 150    // ���ܳ���ʱ�䣨���ݿ������ڵ���������10ms������1.5�룩

// �޸�״̬����ͳ�ʼ��
typedef enum {
    STATE_INIT,          // ������ʼ��״̬
    STATE_WAIT_STOP,
    STATE_STARTUP,
    STATE_RUNNING,
    STATE_STOP_DETECT
} MotorState;

 //encoder_dir_init(TC_CH7_ENCODER, TC_CH7_ENCODER_CH1_P7_6, TC_CH7_ENCODER_CH2_P7_7);// ʹ��TCPWM��ʱ��   P7_6���Ž��м���    ��������

//  ʹ��ʾ��    P7_7);// ʹ��TCPWM��ʱ��   P7_6���Ž��м���    ��������ʹ��P7_7����


//-----------------�ٶȻ�pid---------------//
typedef struct
{
  int16 output;
  int16 speed_racc;             //������ٶ�
  float last_uk;               //��һ�εĿ������������ֵ�������ռ�ձ�/10
  float  kp;
  float  ki;
  //------------    �ٶȻ���    ------------//
//  float  integral;            //ͣ��ʱΪ�㣬��motor.c�п���
  //------------    �ٶȻ���    ------------//
  float  kd;
  float  kvff;      //�ٶ�ǰ��ϵ��
  float  kaff;      //���ٶ�ϵ��
  float deta_uk;
  float out_duty;

  float kvff_param;
  int16 present_value[10];  //������ʵ�ʲ��ֵ
  int16 error[10];          //������
  int16 set_value[10];      //�ٶ��趨����
}Motor_pid_info;            //���PID�������Ϣ����






extern Motor_pid_info Motor_Left_pid;   //���PID
extern Motor_pid_info Motor_Right_pid;  //���PID
extern uint8 start_control_flag;
extern  uint8 start_state;
extern uint32 p;
extern int16 L_outPWM , R_outPWM;
//extern uint32 bianmaqi_left_speed,bianmaqi_right_speed;    //�������ٶ�
//extern uint8 left_mc_flag;
//extern uint32 right_mc , left_mc ;
//extern int32 maichong;
//extern uint32 current_left_mc,current_right_mc,previous_left_mc,previous_right_m,right_mc_difference,left_mc_difference;
//extern float left_proportion,right_proportion;
//extern uint8  suduhuan_flag;

void Motor_Init(void);
void Motor_Control(int L_outPWM, int R_outPWM);
void Speed_control_init(void);
void Speed_Measure(void);
void start_control(void);  //���ܿ���
void Motor_Pid_Caculate(Motor_pid_info *motor_info);
void Motor_PID_Reset(Motor_pid_info *pid);


#endif /* CODE_MOTOR_H_ */