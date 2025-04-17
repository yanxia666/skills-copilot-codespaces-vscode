#include "Gyroscope.h"
#include "zf_common_headfile.h"
#include "zf_device_imu660ra.h"

#define PID_KP  0.5  // ����ϵ��
#define PID_KI  0.01 // ����ϵ��
#define PID_KD  0.1  // ΢��ϵ��

#define DESIRED_YAW 0.0 // ������ƫ���ǣ�0��ʾֱ����ʻ��

// IIR�˲�������
#define IIR_A0 0.1  // ����Ȩ��
#define IIR_B1 0.9  // ����Ȩ��

float tuoluoyi_pid_error = 0.0;       // ��ǰ���
float pid_integral = 0.0;    // ������
float pid_derivative = 0.0;  // ΢����
float tuoluoyi_pid_last_error = 0.0;  // ��һ�ε����
float tuoluoyi_pid_output = 0.0;      // PID ���ֵ

float yaw_angle = 0.0;       // ��ǰƫ����
float filtered_yaw_angle = 0.0; // �˲����ƫ����
float gyro_z = 0.0;          // Z ����ٶ�
float dt = 0.01;             // ʱ����������Ϊ 10ms��
float steering_angle;
float real_angle;
//-------------------------------------------------------------------------------------------------------------------
// �������     PID ����������
// ����˵��     error          ��ǰ���
// ����˵��     dt             ʱ����
// ���ز���     void
// ʹ��ʾ��     pid_update(pid_error, dt);
// ��ע��Ϣ     ���� PID ��������״̬
//-------------------------------------------------------------------------------------------------------------------
void pid_update(float error, float dt)
{
    pid_integral += error * dt; // ������
    pid_derivative = (error - tuoluoyi_pid_last_error) / dt; // ΢����
    tuoluoyi_pid_output = PID_KP * error + PID_KI * pid_integral + PID_KD * pid_derivative; // PID ���
    tuoluoyi_pid_last_error = error; // ������һ�ε����
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡƫ����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     get_yaw_angle();
// ��ע��Ϣ     ͨ���������������ݻ�ȡƫ���ǣ�������IIR�˲�
//-------------------------------------------------------------------------------------------------------------------
void get_yaw_angle() {
    static uint32_t last_time = 0;
   

    imu660ra_get_gyro(); // ��ȡ����������
    gyro_z = imu660ra_gyro_z; // Z ����ٶ�

    yaw_angle += gyro_z * dt; // ���ֵõ�ƫ����

    // IIR�˲�
    filtered_yaw_angle = IIR_A0 * yaw_angle + IIR_B1 * filtered_yaw_angle;
    
    
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����С������ֱ����ʻ
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     control_car();
// ��ע��Ϣ     ʹ�� PID ����������С���ķ���
//-------------------------------------------------------------------------------------------------------------------
void control_car()
{
    uint8 chushixian=0;
    get_yaw_angle(); // ��ȡ��ǰƫ����
    real_angle=(int8)(yaw_angle/651*360);
    tuoluoyi_pid_error = yaw_angle - DESIRED_YAW; // �������
   // pid_update(tuoluoyi_pid_error, dt); // ���� PID ������
  
    // ���� PID �������С���ķ���
   //  steering_angle = tuoluoyi_pid_output; // ���� PID ���ֱ��ӳ��Ϊת��Ƕ�
    // ����С����ת����������Ϊ car_set_steering_angle��
    
}

