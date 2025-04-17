#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_typedef.h"

#include "zf_common_headfile.h"

// 按键事件类型定义
typedef enum {
    KEY_EVENT_NONE,         // 无事件
    KEY_EVENT_SHORT_PRESS,  // 短按事件
    KEY_EVENT_LONG_PRESS    // 长按事件
} Key_Event;


// 按键状态结构体
typedef struct {
    uint32_t press_timestamp;  // 按下时间戳
    bool is_pressing;          // 当前按压状态
    bool long_press_triggered; // 长按已触发标志
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

