#ifndef	_GYROSCOPE_H
#define _GYROSCOPE_H
#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

void pid_update(float error, float dt);
void get_yaw_angle(void);
void control_car(void);

extern float steering_angle;
extern float yaw_angle,real_angle,tuoluoyi_pid_error;
#endif
