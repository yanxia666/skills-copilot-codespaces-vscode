#include "Gyroscope.h"
#include "zf_common_headfile.h"
#include "zf_device_imu660ra.h"

#define PID_KP  0.5  // 比例系数
#define PID_KI  0.01 // 积分系数
#define PID_KD  0.1  // 微分系数

#define DESIRED_YAW 0.0 // 期望的偏航角（0表示直线行驶）

// IIR滤波器参数
#define IIR_A0 0.1  // 输入权重
#define IIR_B1 0.9  // 反馈权重

float tuoluoyi_pid_error = 0.0;       // 当前误差
float pid_integral = 0.0;    // 积分项
float pid_derivative = 0.0;  // 微分项
float tuoluoyi_pid_last_error = 0.0;  // 上一次的误差
float tuoluoyi_pid_output = 0.0;      // PID 输出值

float yaw_angle = 0.0;       // 当前偏航角
float filtered_yaw_angle = 0.0; // 滤波后的偏航角
float gyro_z = 0.0;          // Z 轴角速度
float dt = 0.01;             // 时间间隔（假设为 10ms）
float steering_angle;
float real_angle;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 控制器更新
// 参数说明     error          当前误差
// 参数说明     dt             时间间隔
// 返回参数     void
// 使用示例     pid_update(pid_error, dt);
// 备注信息     更新 PID 控制器的状态
//-------------------------------------------------------------------------------------------------------------------
void pid_update(float error, float dt)
{
    pid_integral += error * dt; // 积分项
    pid_derivative = (error - tuoluoyi_pid_last_error) / dt; // 微分项
    tuoluoyi_pid_output = PID_KP * error + PID_KI * pid_integral + PID_KD * pid_derivative; // PID 输出
    tuoluoyi_pid_last_error = error; // 更新上一次的误差
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取偏航角
// 参数说明     void
// 返回参数     void
// 使用示例     get_yaw_angle();
// 备注信息     通过积分陀螺仪数据获取偏航角，并进行IIR滤波
//-------------------------------------------------------------------------------------------------------------------
void get_yaw_angle() {
    static uint32_t last_time = 0;
   

    imu660ra_get_gyro(); // 获取陀螺仪数据
    gyro_z = imu660ra_gyro_z; // Z 轴角速度

    yaw_angle += gyro_z * dt; // 积分得到偏航角

    // IIR滤波
    filtered_yaw_angle = IIR_A0 * yaw_angle + IIR_B1 * filtered_yaw_angle;
    
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制小车保持直线行驶
// 参数说明     void
// 返回参数     void
// 使用示例     control_car();
// 备注信息     使用 PID 控制器调整小车的方向
//-------------------------------------------------------------------------------------------------------------------
void control_car()
{
    uint8 chushixian=0;
    get_yaw_angle(); // 获取当前偏航角
    real_angle=(int8)(yaw_angle/651*360);
    tuoluoyi_pid_error = yaw_angle - DESIRED_YAW; // 计算误差
   // pid_update(tuoluoyi_pid_error, dt); // 更新 PID 控制器
  
    // 根据 PID 输出调整小车的方向
   //  steering_angle = tuoluoyi_pid_output; // 假设 PID 输出直接映射为转向角度
    // 调用小车的转向函数（假设为 car_set_steering_angle）
    
}

