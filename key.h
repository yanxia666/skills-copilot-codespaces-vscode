#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_typedef.h"

#include "zf_common_headfile.h"

// �����¼����Ͷ���
typedef enum {
    KEY_EVENT_NONE,         // ���¼�
    KEY_EVENT_SHORT_PRESS,  // �̰��¼�
    KEY_EVENT_LONG_PRESS    // �����¼�
} Key_Event;


// ����״̬�ṹ��
typedef struct {
    uint32_t press_timestamp;  // ����ʱ���
    bool is_pressing;          // ��ǰ��ѹ״̬
    bool long_press_triggered; // �����Ѵ�����־
} Key_State;


#define KEY1                    (P11_0)
#define KEY2                    (P22_0)
#define KEY3                    (P23_3)
#define KEY4                    (P23_4)
extern float  kp,ki,kd;
extern uint8  show_flag,show_flag1;


void handle_long_press(uint8_t key_index, uint8_t* flag);
//bool check_long_press();
void zhixian_pid_change(void);
void anjian_pid_change(void);
uint32_t get_system_time(void);
void display_control(void);
void anjian_pid_change(void);
Key_Event check_key_event(uint8_t key_index) ;
void keys_init(void );
#endif

