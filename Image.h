/*
 * camera.h
 *
 *  Created on: 2024��9��15��
 *      Author: HONOR
 */

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#define IPSH 60                              //����ͼ����ĸ߶�
#define IPSW 80                              //����ͼ����Ŀ��

#define GrayScale 256                        //���ص�Ϊ0~255

#define PictureCentring 39                   //ͼ������

#define LimitL(L) (L = ((L < 1) ? 1 : L))    //���Ʒ���
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //���Ʒ���


#define BEEP       P23_7
#define beepopen   gpio_set_level(BEEP,1)
#define beepclose  gpio_set_level(BEEP,0)



#define FAR_ROI_START_Y  0    // Զ����ʼ��
#define FAR_ROI_END_Y    30   // Զ�˽�����
#define TARGET_BRIGHT    100  // Ŀ������ֵ(0-255)

typedef enum {          /*Ԫ������*/
  Straight,     ////ֱ��
  Ramp,         //�µ�
  LeftCirque,   ////��Բ��
  RightCirque,  ////��Բ��
  Cross_ture,   //ʮ��
} RoadType_e;

typedef struct{
        uint8 Threshold;          //��ֵ����ֵ
        uint32 Threshold_static;  //��ֵ����̬����
        uint8 Threshold_detach;   //�����㷨�ָ���ֵ//40
        RoadType_e Road_type;  //Ԫ������
        uint8 OFFLine;            //��Ч�����ͼ����
        uint8 WhiteLine;          ////˫�߶�����
        int16 Miss_Left_lines;                      //���߶�ʧ
        int16 Miss_Right_lines;                     //���߶�ʧ
        int TowPoint;  //��������ǰհ
        float Det_True;
        int16 Zebra_Flag;
        int16 Ramp_Flag;
        float MU_P;
        int straight_acc;
        int variance_acc;    //���ڼ��ٵ���ֵ����
        int16 OFFLineBoundary;   //�������ֹ��
        //�����ַ���ɨ������
        int16 WhiteLine_L;        //��߶�����
        int16 WhiteLine_R;        //�ұ߶�����
        int16 image_element_rings;                  /*0 :��Բ��          1 :��Բ��       2 :��Բ��*/
        int16 image_element_rings_flag;             /*Բ������*/
        int16 ring_big_small;                       /*0:��                     1 :��Բ��       2 :СԲ��*/
        
        
        
        // ...ԭ�в���...
    uint8 width_threshold;   // ���߿���ж���ֵ 
    uint8 width_stable_cnt;  // ����ȶ�������
    uint8 width_flag;        // ���״̬��־ 

}ImageParametertypedef;
extern ImageParametertypedef ImageParameter;//ͼ���õ��ĸ�������

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;//�����������//����: T:��������  W:�������  H:������:������Ϊɨ�߷�Χ��δѰ���������ɨ�߷�Χ�м��

typedef struct {
        /*���ұ߽߱��־    TΪ���������    WΪ�ޱ�   PΪ�ϰ�������ߵ��ڱ�  HΪ������*///p�ƺ�û��
      uint8 IsRightFind;      //�ұ��Ƿ��б߱�־
      uint8 IsLeftFind;       //����Ƿ��б߱�־
      int Wide;               //�߽���
      int LeftBorder;         //��߽�
      int RightBorder;        //�ұ߽�
      int Center;             //����
      int LeftBoundary_First;  //��߽� �����һ������
      int RightBoundary_First; //�ұ߽� �����һ������
      int LeftBoundary;        //��߽� �������һ������
      int RightBoundary;       //�ұ߽� �������һ������

}RowAttributetypedef;//�е�����
extern RowAttributetypedef RowAttribute[60];          //��¼���е���Ϣ

typedef struct {
  float nowspeed;
  int MinSpeed;             //����ٶ�
  int MaxSpeed;             //����ٶ�
  int straight_speed;       //ֱ���ٶ�
} SpeedDatatypedef;

typedef struct {
  SpeedDatatypedef SpeedData;
  uint8_t Stop;         //ֹͣ��־

  //����ͷ����
  int exp_time; //�ع�ʱ��
  int mtv_lroffset;//����ͷ����ƫ��
  int mtv_gain;//����ͷ����

} SystemDatatypdef;
extern SystemDatatypdef SystemData;
extern uint8 Compressed_image[IPSH][IPSW];
extern float variance_acc;
extern int ImageScanInterval_Cross;
extern int ImageScanInterval;
extern uint8 Binarization_Image[IPSH][IPSW];//���ڴ����ֵ�����ͼ��


extern float D_L,D_R;



extern volatile uint8_t Emergency_Flag;


int CountWhitePixelsInRow(uint8 row) ;
void optimized_exposure_flow(void);
void enhance_far_region(void);
void weighted_exposure_adjust(void);
void dynamic_exposure_adjust(void);// ��̬�ع�ʱ�����
void ImageProcess(void);//ͼ���ܴ�����
void Cramping(void);//ͼ��ѹ������
void Threshold_Speraration_Ostu(void);//ͼ���ֵ����ֵ��ȡ����ֵ��
void Bin_Image_Filter(void);//ȥ����㺯��
static uint8 DrawBasicLines(void);//ɨ�������ǰ����
static void DrawProcessLines(void);//�߽�׷��ɨ��ǰ�������ʣ����б���
void Eightfields_Search_Border(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline);//������ɨ��
void Right_Rings_Judgment(void);//��Բ�����
void Left_Rings_Judgment(void);//��Բ�����
void Element_Test(void);//Ԫ��ʶ����
void Element_Handle(void);//Ԫ�ش����ܺ���
void Emergency_Breaking(void);//�����ƶ�����
void Prospective_error(void);//ǰհ����������������ֵ
void Speed_Control_Factor(void);//��ʱû�õ������á�����
float Straight_Judge(uint8 dir, uint8 start, uint8 end);
void Data_Settings(void);
float CalculateOFFLineCenter(void);
#endif /* CODE_CAMERA_H_ */
