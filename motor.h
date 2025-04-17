#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"

#include "zf_common_headfile.h"


 #define MAX_DUTY            (30 )                                               // 最大 MAX_DUTY% 占空比

 #define PWM_L1               (TCPWM_CH50_P18_7)        //输出pwm
 #define PWM_LD1              (P18_6)  
 #define PWM_R1               (TCPWM_CH13_P00_3)        //输出pwm
 #define PWM_RD1              (P00_2)                 //判断方向io口
 
             
  /*                                                                              
#define ENCODER_QUAD1                    (TC_CH09_ENCODER)                      // 编码器接口  
#define ENCODER_QUAD1_PHASE_A            (TC_CH09_ENCODER_CH1_P05_0)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD1_PHASE_B            (TC_CH09_ENCODER_CH2_P05_1)            // PHASE_B 对应的引脚                   
                                                                                
#define ENCODER_QUAD2                    (TC_CH07_ENCODER)                      // 编码器接口
#define ENCODER_QUAD2_PHASE_A            (TC_CH07_ENCODER_CH1_P02_0)            // PHASE_A 对应的引脚
#define ENCODER_QUAD2_PHASE_B            (TC_CH07_ENCODER_CH2_P02_1)            // PHASE_B 对应的引脚
*/
#define DIRECTION_ENCODER_left       (TC_CH09_ENCODER)                 
#define DIRECTION_ENCODER_A_left     (TC_CH09_ENCODER_CH1_P05_0)      
#define DIRECTION_ENCODER_B_left     (TC_CH09_ENCODER_CH2_P05_1)       
#define DIRECTION_ENCODER_right      (TC_CH07_ENCODER)                 
#define DIRECTION_ENCODER_A_right    (TC_CH07_ENCODER_CH1_P02_0)       
#define DIRECTION_ENCODER_B_right    (TC_CH07_ENCODER_CH2_P02_1)       





#define START_SPEED_THRESHOLD  10  // 电机启动速度阈值（根据实际情况调整）
#define START_TIMEOUT         500  // 启动超时时间（单位：循环周期）

// 配置参数（根据实际情况调整）
#define STOP_SPEED_THRESHOLD 5     // 停止判定速度阈值
#define STOP_CONFIRM_TIME   1000   // 停止确认时间（ms）
#define START_DURATION 150    // 起跑持续时间（根据控制周期调整，假设10ms周期则1.5秒）

// 修改状态定义和初始化
typedef enum {
    STATE_INIT,          // 新增初始化状态
    STATE_WAIT_STOP,
    STATE_STARTUP,
    STATE_RUNNING,
    STATE_STOP_DETECT
} MotorState;

 //encoder_dir_init(TC_CH7_ENCODER, TC_CH7_ENCODER_CH1_P7_6, TC_CH7_ENCODER_CH2_P7_7);// 使用TCPWM定时器   P7_6引脚进行计数    计数方向

//  使用示例    P7_7);// 使用TCPWM定时器   P7_6引脚进行计数    计数方向使用P7_7引脚


//-----------------速度环pid---------------//
typedef struct
{
  int16 output;
  int16 speed_racc;             //电机加速度
  float last_uk;               //上一次的控制量输出绝对值，即电机占空比/10
  float  kp;
  float  ki;
  //------------    速度积分    ------------//
//  float  integral;            //停车时为零，在motor.c中控制
  //------------    速度积分    ------------//
  float  kd;
  float  kvff;      //速度前馈系数
  float  kaff;      //加速度系数
  float deta_uk;
  float out_duty;

  float kvff_param;
  int16 present_value[10];  //编码器实际测得值
  int16 error[10];          //误差队列
  int16 set_value[10];      //速度设定队列
}Motor_pid_info;            //电机PID所需的信息类型






extern Motor_pid_info Motor_Left_pid;   //电机PID
extern Motor_pid_info Motor_Right_pid;  //电机PID
extern uint8 start_control_flag;
extern  uint8 start_state;
extern uint32 p;
extern int16 L_outPWM , R_outPWM;
//extern uint32 bianmaqi_left_speed,bianmaqi_right_speed;    //编码器速度
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
void start_control(void);  //起跑控制
void Motor_Pid_Caculate(Motor_pid_info *motor_info);
void Motor_PID_Reset(Motor_pid_info *pid);


#endif /* CODE_MOTOR_H_ */