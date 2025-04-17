/*
 * camera.h
 *
 *  Created on: 2024年9月15日
 *      Author: HONOR
 */

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#define IPSH 60                              //用于图像处理的高度
#define IPSW 80                              //用于图像处理的宽度

#define GrayScale 256                        //像素点为0~255

#define PictureCentring 39                   //图像中心

#define LimitL(L) (L = ((L < 1) ? 1 : L))    //限制幅度
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //限制幅度


#define BEEP       P23_7
#define beepopen   gpio_set_level(BEEP,1)
#define beepclose  gpio_set_level(BEEP,0)



#define FAR_ROI_START_Y  0    // 远端起始行
#define FAR_ROI_END_Y    30   // 远端结束行
#define TARGET_BRIGHT    100  // 目标亮度值(0-255)

typedef enum {          /*元素类型*/
  Straight,     ////直道
  Ramp,         //坡道
  LeftCirque,   ////左圆环
  RightCirque,  ////右圆环
  Cross_ture,   //十字
} RoadType_e;

typedef struct{
        uint8 Threshold;          //二值化阈值
        uint32 Threshold_static;  //二值化静态下限
        uint8 Threshold_detach;   //阳光算法分割阈值//40
        RoadType_e Road_type;  //元素类型
        uint8 OFFLine;            //有效处理的图像顶行
        uint8 WhiteLine;          ////双边丢边数
        int16 Miss_Left_lines;                      //左线丢失
        int16 Miss_Right_lines;                     //右线丢失
        int TowPoint;  //控制误差的前瞻
        float Det_True;
        int16 Zebra_Flag;
        int16 Ramp_Flag;
        float MU_P;
        int straight_acc;
        int variance_acc;    //用于加速的阈值方差
        int16 OFFLineBoundary;   //八领域截止行
        //左右手法则扫线数据
        int16 WhiteLine_L;        //左边丢线数
        int16 WhiteLine_R;        //右边丢线数
        int16 image_element_rings;                  /*0 :无圆环          1 :左圆环       2 :右圆环*/
        int16 image_element_rings_flag;             /*圆环进程*/
        int16 ring_big_small;                       /*0:无                     1 :大圆环       2 :小圆环*/
        
        
        
        // ...原有参数...
    uint8 width_threshold;   // 白线宽度判定阈值 
    uint8 width_stable_cnt;  // 宽度稳定计数器
    uint8 width_flag;        // 宽度状态标志 

}ImageParametertypedef;
extern ImageParametertypedef ImageParameter;//图像将用到的各个参数

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;//跳变点与类型//类型: T:正常跳变  W:无跳变点  H:大跳变:大跳变为扫线范围内未寻到跳变点且扫线范围中间黑

typedef struct {
        /*左右边边界标志    T为正常跳变边    W为无边   P为障碍类多跳边的内边  H为大跳变*///p似乎没有
      uint8 IsRightFind;      //右边是否有边标志
      uint8 IsLeftFind;       //左边是否有边标志
      int Wide;               //边界宽度
      int LeftBorder;         //左边界
      int RightBorder;        //右边界
      int Center;             //中线
      int LeftBoundary_First;  //左边界 保存第一次数据
      int RightBoundary_First; //右边界 保存第一次数据
      int LeftBoundary;        //左边界 保存最后一次数据
      int RightBoundary;       //右边界 保存最后一次数据

}RowAttributetypedef;//行的属性
extern RowAttributetypedef RowAttribute[60];          //记录单行的信息

typedef struct {
  float nowspeed;
  int MinSpeed;             //最低速度
  int MaxSpeed;             //最高速度
  int straight_speed;       //直道速度
} SpeedDatatypedef;

typedef struct {
  SpeedDatatypedef SpeedData;
  uint8_t Stop;         //停止标志

  //摄像头配置
  int exp_time; //曝光时间
  int mtv_lroffset;//摄像头左右偏置
  int mtv_gain;//摄像头增益

} SystemDatatypdef;
extern SystemDatatypdef SystemData;
extern uint8 Compressed_image[IPSH][IPSW];
extern float variance_acc;
extern int ImageScanInterval_Cross;
extern int ImageScanInterval;
extern uint8 Binarization_Image[IPSH][IPSW];//用于储存二值化后的图像


extern float D_L,D_R;



extern volatile uint8_t Emergency_Flag;


int CountWhitePixelsInRow(uint8 row) ;
void optimized_exposure_flow(void);
void enhance_far_region(void);
void weighted_exposure_adjust(void);
void dynamic_exposure_adjust(void);// 动态曝光时间调整
void ImageProcess(void);//图像总处理函数
void Cramping(void);//图像压缩函数
void Threshold_Speraration_Ostu(void);//图像二值化阈值获取及二值化
void Bin_Image_Filter(void);//去除噪点函数
static uint8 DrawBasicLines(void);//扫描基本行前五行
static void DrawProcessLines(void);//边界追逐扫描前五行外的剩余各行边线
void Eightfields_Search_Border(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline);//八领域扫线
void Right_Rings_Judgment(void);//右圆环检测
void Left_Rings_Judgment(void);//左圆环检测
void Element_Test(void);//元素识别函数
void Element_Handle(void);//元素处理总函数
void Emergency_Breaking(void);//紧急制动函数
void Prospective_error(void);//前瞻控制误差函数，获得误差值
void Speed_Control_Factor(void);//暂时没用到，作用………
float Straight_Judge(uint8 dir, uint8 start, uint8 end);
void Data_Settings(void);
float CalculateOFFLineCenter(void);
#endif /* CODE_CAMERA_H_ */
