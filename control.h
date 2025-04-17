
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

//------------------------- ���Ͷ��� -------------------------

#define BELL_ON                 gpio_set_level(P23_7, GPIO_HIGH)
#define BELL_OFF                gpio_set_level(P23_7, GPIO_LOW)

//����ö�����͵��Զ��������Ե��µģ�ָ���� TRACK_STRAIGHT = 1,����δָ��ֵ��ö������Զ�����
typedef enum {
    DEFAULT = 0,  // 0
    TRACK_VERTICAL = 1, // 3
    
} TrackType;

typedef struct {
    float kp;
    float kd;
    int base_speed;
    int max_speed;
} TrackPIDConfig;


//λ��ʽpid
typedef struct
{
    int16 output;
    int16 speed;
    int16 error[2];
    float kp;
    float ki;
    float kd;

    float base_p;
}Direct_pid_info;    //λ��ʽpid
/*ȫ�ֱ�������*/
extern  float left_slope,weighted_variance;
extern  float right_slope,variance;
extern int current_error,left_jump,right_jump,width_change;
;
extern uint8  Track_Line_Num,zhi_continuous_cnt,angle_type;
extern uint8 j,k;
extern float offLineCenter,offerror;
/*************************   ���ֽ׶�pid    *****************************************/
extern TrackPIDConfig track_configuration[];
extern TrackPIDConfig cfg;
extern uint32 show_count;
extern uint8 TRACK_VERTICAL_delay_flag;
extern float left_slope1,right_slope1;
//extern uint8 elements_flag ;
extern int8_t piancha;
extern uint8_t Point_Mid;
extern int8_t Media_line_difference;
extern uint8_t jiezhihang_change;

extern Direct_pid_info angle_pid;       //�ǶȻ�PID
extern float GKD;


extern int left_jump_cnt,right_jump_cnt;

uint8_t check_vertical_200ms(void );
void set_track_line_num(TrackType new_state);
float Calculate_Width_Diff(void);
uint8 Detect_Edge_Mutation(void);
float Calculate_Centroid_Shift(void);
uint8 Count_Lost_Border(char dir, uint8 start, uint8 end);
void Width_Analysis_Pro(void);
void fengmingqi(void);
void Get_RegionalWidthDiff(float *bottom, float *middle, float *front);
uint8 straight_line_detection();
float Calculate_Slope(uint8 is_left);
void change_pid(void);
uint8 RightAngle_Detection(void);
uint8 Right_Angle_Feature_Check(void);
void Track_Control_Core(TrackType track_type);
void Track_Line(void);//�����ж�
void zhixian_pid(void); //ֱ���ٶȵ���
void zhexian_pid_change(void);  //С�����ٶȵ���
void chuizhi_pid_change(void);  //��ֱ�ٶȵ���
int jueduizhi(int data);
void zhixian_pid(void) ;
void Control(void);
#endif

