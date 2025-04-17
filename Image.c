/*
 * camera.c
 *
 *  Created on: 2024��9��15��
 *      Author: HONOR
 */
#include "zf_common_headfile.h"

uint8 Compressed_image[IPSH][IPSW];//���ڴ洢ѹ�����ͼ��
uint8 Binarization_Image[IPSH][IPSW];//���ڴ����ֵ�����ͼ��
ImageParametertypedef ImageParameter;//ͼ����������ṹ�嶨��
RowAttributetypedef RowAttribute[60]; //���ڼ�¼������Ϣ
SystemDatatypdef SystemData;

static int Ysite=0,Xsite=0;//������������꣬ǰ�к���
static int BottomBorderRight = 79,BottomBorderLeft=0,BottomCenter=0;//��59�����ұ߽�

static uint8* StoreSingleLine;                 //���浥��ͼ��
uint8 ExtenLFlag = 0;                          //�Ƿ����ӳ���־
uint8 ExtenRFlag = 0;                          //�Ƿ����ӳ���־

int ImageScanInterval_Cross;                   //270��������ʮ�ֵ�ɨ�߷�Χ
static int IntervalLow = 0, IntervalHigh = 0;  //����ߵ�ɨ������
int ImageScanInterval;                         //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval

int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //��Բ���жϵ�����������
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //��Բ���жϵ�����������
int Point_Xsite,Point_Ysite;                   //�յ��������
int Repair_Point_Xsite,Repair_Point_Ysite;     //���ߵ��������
uint8 Ring_Help_Flag = 0; //����������־
uint8 Half_Road_Wide[60] =                      //ֱ���������
{ 
  1,1,1,2,2,2,3,3,3,4,  // ǰ10�п�ȼ���
  4,4,5,5,5,6,6,6,7,7,
  //2,2,2,2,3,3,4,5,5,5,
 // 6,6,6,7,7,7,8,8,8,9,
  9,10,10,10,11,11,12,12,13,13,
  13,14,14,14,15,15,15,16,16,17,
  17,17,18,18,19,19,20,20,20,21,
  21,22,22,22,23,23,23,24,24,24,
};



//float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};
                        //10��Ȩ�ز�����������ģ�������Ӱ�죬���°�����̬�ֲ�����
float Weighting[10] = {0.67, 0.69, 0.88, 0.96, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};


int flag_cross=0;
static int TFSite = 0, left_FTSite = 0,right_FTSite = 0;              //���߼���б�ʵ�ʱ����Ҫ�õĴ���еı�����
static float DetR = 0, DetL = 0;                //��Ų���б�ʵı���
static int ytemp = 0;                           //����е���ʱ����

int flag_test_stop=1;
int flag_test=0;

float Original_imageH=MT9V03X_H;//ԭʼͼ���
float Original_imageW=MT9V03X_W;//ԭʼͼ���
float Compression_imageH=IPSH;//ѹ����ͼ���
float Compression_imageW=IPSW;//ѹ����ͼ���

/***ͼ��ѹ������***/
void Cramping(void){
    int m,n,column,line;
    const float Proportion_h=Original_imageH/Compression_imageH;//����ѹ������
    const float Proportion_w=Original_imageW/Compression_imageW;//����ѹ������
    for(m=0;m<IPSH;m++){
        column=m*Proportion_h+0.5;//��0.5����������������
    for(n=0;n<IPSW;n++){
         line=n*Proportion_w+0.5;
         Compressed_image[m][n] = mt9v03x_image[column][line];
        }

    }
}

/***�Ż���Ĵ��***/
uint8 Threshold_deal(uint8* image, uint16 col, uint16 row, uint32 pixel_threshold)//���һ������Ϊ�����㷨�ָ��õ���ֵ���ҵ����Ϊ�ֱ𱳾����������ֵ
{
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = image;
    for (i = 0; i < GrayScale; i++) {
      pixelCount[i] = 0;
      pixelPro[i] = 0;
    }//���ݳ�ʼ��
    uint32 gray_sum = 0;
    for (i = 0; i < height; i += 1) {
      for (j = 0; j < width; j += 1) {
        pixelCount[(int)data[i * width + j]]++;//ͼ�������ڸ�������ֵ�����ص��ͳ��
        gray_sum += (int)data[i * width + j];//ͼ��������ص�����ֵ�ĺ�
      }
    }
    for (i = 0; i < GrayScale; i++) {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;//�ó���������ֵ�����ص�ռͼ��ı���
    }

    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_threshold; j++) {//С�������㷨�ָ����ֵ��Ϊ����
      w0 +=
          pixelPro[j];//�������ص�ռͼ��ı���
      u0tmp += j * pixelPro[j];//��������ֵռ��ͼ�����ص�ƽ��ֵ
      w1 = 1 - w0;//�������ص�ռͼ��ı���
      u1tmp = gray_sum / pixelSum - u0tmp;//��������ֵռ��ͼ�����ص�ƽ��ֵ
      u0 = u0tmp / w0;//�������ص�ƽ��ֵ
      u1 = u1tmp / w1;//�������ص�ƽ��ֵ
      u = u0tmp + u1tmp;//ͼ������ֵ��ƽ��ֵ
      deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//���㷽��
      if (deltaTmp > deltaMax) {
        deltaMax = deltaTmp;
        threshold = (uint8)j;
      }//������ֵ�ﵽ���ʱ����ʱ����ֵ��Ϊ�����ֵ
      if (deltaTmp < deltaMax) {
        break;
      }
    }
    return threshold;
}

/***��ֵ����ֵ��ȡ����ֵ������***/
void Threshold_Speraration_Ostu(void){
  //  ImageParameter.Threshold =
      //      (uint8)Threshold_deal(Compressed_image[0],IPSW,IPSH,ImageParameter.Threshold_detach);
  ImageParameter.Threshold = Threshold_deal(Compressed_image[0], IPSW, IPSH, ImageParameter.Threshold_detach);
     //ImageParameter.Threshold =160;
          
    
    if(ImageParameter.Threshold < ImageParameter.Threshold_static)
        ImageParameter.Threshold  =  (uint8)ImageParameter.Threshold_static;//ȷ����ֵ��������ֵ��̬����
    uint8 m,n;
    uint8 ther;
    
    
     for(m = 0;m < IPSH;m++){
        for(n = 0;n < IPSW;n++){
            // ���ٱ�Ե�������ֵ˥��
            if(n <= 20 || n >= 60) // ԭ15/70�߽� �� ������������
                ther = ImageParameter.Threshold - 5; 
            else
                ther = ImageParameter.Threshold;
                
            Binarization_Image[m][n] = (Compressed_image[m][n] > ther) ? 1 : 0;
    
   /* for(m = 0;m < IPSH;m++){
        for(n = 0;n < IPSW;n++){
            if(n <= 15)
                ther = ImageParameter.Threshold - 10;//ͼ��Խ����ԽԽ����Ϊ��������ֵ�����ʵ�����
            else if ((n > 70 && n <= 75))
                ther = ImageParameter.Threshold - 10;
            else if (n >= 65)
                ther = ImageParameter.Threshold - 10;
            else
                ther = ImageParameter.Threshold;
  //��ֵ��
            if(Compressed_image[m][n] > (ther))
                Binarization_Image[m][n] = 1;//��
            else
                Binarization_Image[m][n] = 0;//��
            }
        }*/
}
     }
}
/***ȥ����㺯��***/
void Bin_Image_Filter(void) 
{
    uint16 nr; // ������
    uint16 nc; // ������

    for (nr = 1; nr < IPSH - 1; nr++) {
        for (nc = 1; nc < IPSW - 1; nc++) {
            // ����8����׵�����������4���Խǣ�
            uint8 neighbors = 
                Binarization_Image[nr-1][nc-1] +  // ����
                Binarization_Image[nr-1][nc]   +  // �� 
                Binarization_Image[nr-1][nc+1] +  // ����
                Binarization_Image[nr][nc-1]   +  // ��
                Binarization_Image[nr][nc+1]   +  // ��
                Binarization_Image[nr+1][nc-1] +  // ����
                Binarization_Image[nr+1][nc]   +  // ��
                Binarization_Image[nr+1][nc+1];   // ����

            /* �׵��������ˣ������׵�ת�ڣ� */
            if (Binarization_Image[nr][nc] == 1) 
            {
                // ����1����Χ�׵�����3����Ϊ����
                // ����2����������б�ߣ�����1��ֱ�����ڰ׵㣩
                if (neighbors < 3 && 
                   (Binarization_Image[nr-1][nc] + 
                    Binarization_Image[nr+1][nc] + 
                    Binarization_Image[nr][nc-1] + 
                    Binarization_Image[nr][nc+1]) < 1) 
                {
                    Binarization_Image[nr][nc] = 0;
                }
            }
            /* �ڵ��������ˣ������ڵ�ת�ף� */ 
            else 
            {
                // ����1����Χ�׵����5��
                // ����2�������󲹶ϵ㣨������������2���׵㣩
                if (neighbors > 5 &&
                   (Binarization_Image[nr-1][nc] + 
                    Binarization_Image[nr+1][nc] + 
                    Binarization_Image[nr][nc-1] + 
                    Binarization_Image[nr][nc+1]) > 1) 
                {
                    Binarization_Image[nr][nc] = 1;
                }
            }
        }
    }
}
/*void Bin_Image_Filter(void){
    uint16 nr; //��
    uint16 nc; //��

    for (nr = 1; nr < IPSH - 1; nr++)
    {
        for (nc = 1; nc < IPSW - 1; nc = nc + 1)
        {
            if ((Binarization_Image[nr][nc] == 0)
                    && (Binarization_Image[nr - 1][nc] + Binarization_Image[nr + 1][nc]
                      + Binarization_Image[nr][nc + 1] + Binarization_Image[nr][nc - 1] > 2))//����׵��м���ںڵ㣬����Ϊ���
            {
                Binarization_Image[nr][nc] = 1;
            }
            else if ((Binarization_Image[nr][nc] == 1)
                    && (Binarization_Image[nr - 1][nc] + Binarization_Image[nr + 1][nc]
                      + Binarization_Image[nr][nc + 1] + Binarization_Image[nr][nc - 1] < 2))//����ڵ��м���ڰ׵㣬��Ϊ���
            {
                Binarization_Image[nr][nc] = 0;
            }
        }
    }
}
*/
/***ɨ�����ߺ���***/
static uint8 DrawBasicLines(void){
    StoreSingleLine = Binarization_Image[59];//����ɨ��59��
    //��59���е�Ϊ��
    if(*(StoreSingleLine + PictureCentring) == 0) /*  ͼ��ױ��е�Ϊ�� �쳣 */
    {
        for (Xsite = 0; Xsite < PictureCentring; Xsite++)
        {
            if (*(StoreSingleLine + PictureCentring - Xsite) != 0)//���м������ɨ
                    break;
            if (*(StoreSingleLine + PictureCentring + Xsite) != 0)//���м����ұ�ɨ
                    break;
        }
        if (*(StoreSingleLine + PictureCentring - Xsite) != 0)//�����ڳ���߷�λ
            {
              BottomBorderRight = PictureCentring - Xsite + 1;//��֮��Ϊ59���ұ���
              for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)//���ұ߽�����ɨ�����
              {
                if (*(StoreSingleLine + Xsite) == 0 &&
                        *(StoreSingleLine + Xsite - 1) == 0)//���������ڵ�����߼��ҵ�//?��Ӧ��Ϊ�������?//�����м�ȫΪ��ɫ��һ·ɨ��ȥɨ�������ڵ㼴����Ϊ�Ǳ߽�
                {
                  BottomBorderLeft = Xsite;//��¼�����
                  break;
                }
                else if (Xsite == 1)//ûɨ������㣬����Ϊ������ǵ�ǰͼ�������
                {
                  BottomBorderLeft = 0;
                  break;
                }
              }
            }
        else if (*(StoreSingleLine + PictureCentring + Xsite) != 0)//�����ڳ��ұ߷�λ
        {
          BottomBorderLeft = PictureCentring + Xsite - 1;//��֮��Ϊ��߽�
          for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)//������߿�ʼ����ɨ�ұ߽�
          {
            if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite + 1) == 0)//�������ɨ�������ڵ�//?
            {
              BottomBorderRight = Xsite;//��Ϊ�ұ߽�
              break;
            }
            else if (Xsite == 78)//ûɨ�������
            {
              BottomBorderRight = 79;
              break;
            }
          }
        }
    }//��59���е�Ϊ��
/////////////////
    //��59���е�Ϊ��
    else        /*  ͼ��ױ��е�Ϊ�� ���� */
    {

        for (Xsite = PictureCentring; Xsite <78; Xsite++)//���е㿪ʼ���������ұ���
        {
          if (  *(StoreSingleLine + Xsite) == 0 &&
                  *(StoreSingleLine + Xsite + 1) == 0)//���������ڵ�
          {
            BottomBorderRight = Xsite;//��¼
            break;
          }
          else if (Xsite == 78)//ɨ�����ûɨ��
          {
            BottomBorderRight = 78;
            break;
          }
        }

        for (Xsite = PictureCentring; Xsite >0; Xsite--)//���е��������������
        {
          if (  *(StoreSingleLine + Xsite) == 0 &&
                  *(StoreSingleLine + Xsite - 1) == 0)//���������ڵ�
          {
            BottomBorderLeft = Xsite;
            break;
          }
          else if (Xsite == 1)//�����һ��ûɨ��������Ϊ�����Ϊ��߽�
          {
            BottomBorderLeft = 1;
            break;
          }
        }
    }
        BottomCenter = (BottomBorderLeft + BottomBorderRight)/2;//59���е�
        RowAttribute[59].LeftBorder = BottomBorderLeft;
        RowAttribute[59].RightBorder = BottomBorderRight;
        RowAttribute[59].Center = BottomCenter;
        RowAttribute[59].Wide = BottomBorderRight - BottomBorderLeft;//����59�п��
        RowAttribute[59].IsLeftFind = 'T';
        RowAttribute[59].IsRightFind = 'T';//59�����ұ߽���������
        //����ɨ�߻�õ�ֵ�����59�и�������

        /*ɨ�������л�����*/

               for (Ysite = 58; Ysite > 54; Ysite--)
               {
                 StoreSingleLine = Binarization_Image[Ysite];
                 for (Xsite =  RowAttribute[Ysite + 1].Center; Xsite < 78; Xsite++)//����һ�е��е�λ�ÿ�ʼɨ�ұ���
                 {
                   if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite + 1) == 0)//�������ɨ�������ڵ�
                   {
                       RowAttribute[Ysite].RightBorder = Xsite;
                       break;
                   }
                   else if (Xsite == 78)//ɨ�����ұ�Ҳûɨ��
                   {
                       RowAttribute[Ysite].RightBorder = 78;
                       break;
                   }
                 }
                 for (Xsite = RowAttribute[Ysite + 1].Center; Xsite >0; Xsite--)//����һ���е�ɨ�����
                 {
                   if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite - 1) == 0)
                   {
                       RowAttribute[Ysite].LeftBorder = Xsite;
                       break;
                   }
                   else if (Xsite == 1)
                   {
                       RowAttribute[Ysite].LeftBorder = 1;
                       break;
                   }
                 }
                 RowAttribute[Ysite].IsLeftFind = 'T';
                 RowAttribute[Ysite].IsRightFind = 'T';
                 RowAttribute[Ysite].Center =
                         (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder)/2;
                 RowAttribute[Ysite].Wide =
                         RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
                 //�������в���
               }
             return 'T'; //�����Ҫ�󡢳�ǰ���в����ܵ�����(����)����װʱע���������ͷ�ӽ�
  }//����ɨ�߽�ֹ


//********************************************���еĴ�ͳɨ��**************************************************//
//p:��Ӧ�е�ͼ������ type:L��R��  L��H:ɨ�ߵ����ҷ�Χ  Q:��¼����㼰���������
void JumpPointAndType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q){/*Ѱ������㲢ȷ��������*/
  int i = 0;                                    /*��һ��������Ҫ���ҵ����飨80���㣩
                                                  �ڶ���ɨ����߻���ɨ�ұ���
                                                  �����͵����ǿ�ʼ�ͽ�����*/
  if (type == 'L')                              //ɨ�������
  {
    for (i = H; i >= L; i--)                    //�ڵ���ɨ
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1) //����㣨�ױ�ڣ�
     // if ((*(p + i) ^ *(p + i �� 1)))
      {
        Q->point = i;                           //��¼�����
        Q->type = 'T';                          //��ȷ����
        break;
      }
      else if (i == (L + 1))                    //ɨ�����ûɨ��
      {
        if (*(p + (L + H) / 2) != 0)            //ûɨ�����м�Ϊ�׵�
        {
          Q->point = (L + H) / 2;               //��Ϊ������ǵ�ǰɨ�跶Χ���е�
          Q->type = 'W';                        //����ȷ�������м�Ϊ�ס��ޱ��У�ʮ��
          break;
        }
        else                                    //����ȷ�������м�Ϊ��
        {
          Q->point = H;                         //�������Ϊɨ�߷�Χ���ڱ�
          Q->type = 'H';                        //��Ϊ�Ǵ�����
          break;
        }
      }
    }
  }
  else if (type == 'R')                         //ɨ���ұ��ߡ�ͬ��
  {
    for (i = L; i <= H; i++)
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)
      {
        Q->point = i;
        Q->type = 'T';
        break;
      }
      else if (i == (H - 1))
      {
        if (*(p + (L + H) / 2) != 0)
        {
          Q->point = (L + H) / 2;
          Q->type = 'W';
          break;
        }
        else
        {
          Q->point = L;
          Q->type = 'H';
          break;
        }
      }
    }
  }
}




/***����׷���ȡ���б߽�***/
static void DrawProcessLines(void){
  
        
   float D_L = 0;           //�ӳ��������б��
   float D_R = 0;           //�ӳ����ұ���б��
  
    uint8 L_Found_T = 'F';   //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־:FΪδ�ҵ���TΪ�ҵ���
    uint8 Get_L_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
    uint8 R_Found_T = 'F';   //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
    uint8 Get_R_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
    
    int ytemp_W_L;           //��ס�״��󶪱���
    int ytemp_W_R;           //��ס�״��Ҷ�����
    ExtenRFlag = 0;          //��־λ����
    ExtenLFlag = 0;
    for (Ysite = 54 ; Ysite > ImageParameter.OFFLine; Ysite--){
           //̫ǰ��ͼ��ɿ��Բ��ߡ�����OFFLine�����ú��б�Ҫ

           ImageScanInterval_Cross = (54 - Ysite)/2 + 5;/*4.24���ڽ��ʮ��bug*/

           StoreSingleLine = Binarization_Image[Ysite];
           JumpPointtypedef JumpPoint[2];     // 0��1��

           /******************************ɨ�豾�е��ұ���******************************/
           /*ȷ��ɨ�߷�Χ*/
           /*if (ImageParameter.Road_type != Cross_ture)*/
           if(ImageParameter.WhiteLine < 5){//��˫�߶�������������
               IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval;//��һ���ұ��߼�ȥInterval��Ϊ����ʼɨ�����
               //����һ���ұ���-Interval�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
               IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval;//��һ���ұ��߼�Interval��Ϊ���ұ�ɨ���������
               //����һ���ұ���+Interval�ĵ������ȷ��ɨ������㣩
           }
           else{//��˫�߶������������У���Ϊʮ��
               IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval_Cross;//��ȥʮ�ֵ�Interval
               //����һ���ұ���-Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
               IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval_Cross;//����ʮ�ֵ�Interval
               //����һ���ұ���+Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
           }
           LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
           LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
           JumpPointAndType(StoreSingleLine,'R', IntervalLow, IntervalHigh,&JumpPoint[1]);//ɨ�ұ���ȷ���ұ��ߵ�����㼰������
           /******************************ɨ�豾�е��ұ���******************************/

           /******************************ɨ�豾�е������******************************/
           /*ȷ��ɨ�߷�Χ*/
           /*if (ImageParameter.Road_type != Cross_ture)*/
           if(ImageParameter.WhiteLine < 5){
               IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval;
               //����һ���ұ���-Interval�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
               IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval;
               //����һ���ұ���+Interval�ĵ������ȷ��ɨ������㣩
           }
           else{
               IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval_Cross;
               //����һ���ұ���-Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
               IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval_Cross;
               //����һ���ұ���+Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
           }
           LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
           LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
           JumpPointAndType(StoreSingleLine, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);//ɨ�����ȷ������ߵ�����㼰������
           /******************************ɨ�豾�е������******************************/

           if (JumpPoint[0].type =='W')  //������ޱ��У�ȫ��
           {
               RowAttribute[Ysite].LeftBorder = RowAttribute[Ysite + 1].LeftBorder;
               //��ô���е�����߾Ͳ�����һ�еı��ߡ�
           }
           else
           {   //������е����������T������H���
               RowAttribute[Ysite].LeftBorder = JumpPoint[0].point;//���¼
           }

           if (JumpPoint[1].type == 'W')   //�ұ����ޱ��У�ȫ��
           {
               RowAttribute[Ysite].RightBorder = RowAttribute[Ysite + 1].RightBorder;
               //��ô���е��ұ��߾Ͳ�����һ�еı��ߡ�
           }
           else
           {   //������е��ұ�������T������H���
               RowAttribute[Ysite].RightBorder = JumpPoint[1].point;
           }
           /*��¼���б�������*/
           RowAttribute[Ysite].IsLeftFind = JumpPoint[0].type;
           RowAttribute[Ysite].IsRightFind = JumpPoint[1].type;

           
           
           /*********************** �����ȶ���У�� Start ***********************/
        // �������һ�еı߽�ͻ�䣨��ֹ����������ţ�
        if(Ysite < 59) { // ��������Խ��
            // ��߽�ͻ����
            if(abs(RowAttribute[Ysite].LeftBorder - RowAttribute[Ysite+1].LeftBorder) > 5) {
                RowAttribute[Ysite].IsLeftFind = 'W'; // ���Ϊ�쳣����
            }
            // �ұ߽�ͻ����
            if(abs(RowAttribute[Ysite].RightBorder - RowAttribute[Ysite+1].RightBorder) > 5) {
                RowAttribute[Ysite].IsRightFind = 'W'; // ���Ϊ�쳣����
            }
        }
        /*********************** �����ȶ���У�� End ***********************/
           
           
           
   /************************************����ȷ��������(��H��)�ı߽�*************************************/
           if (( RowAttribute[Ysite].IsLeftFind == 'H' || RowAttribute[Ysite].IsRightFind == 'H'))
           {
               /**************************��������ߵĴ�����***************************/
               if (RowAttribute[Ysite].IsLeftFind == 'H')
               {
                 for (Xsite = (RowAttribute[Ysite].LeftBorder + 1);//�������е���߽�ǰ���¼Ϊɨ�߷�Χ��������
                         Xsite <= (RowAttribute[Ysite].RightBorder - 1);Xsite++)//��������ɨ���ұ߽�ʱ����Ϊ��ɨ�߷�Χ��������
                 {//��������ɨ��
                   if ((*(StoreSingleLine + Xsite) == 0) && (*(StoreSingleLine + Xsite + 1) != 0))
                   {
                      //������Ը�Ϊɨ����һ���׵���˵㼴Ϊ�ڰ������
                       //�����һ������ߵ��ұ��кڰ�������Ϊ���Ա���ֱ��ȡ��
                       RowAttribute[Ysite].LeftBorder = Xsite;
                       RowAttribute[Ysite].IsLeftFind = 'T';
                       break;
                   }
                   else if (*(StoreSingleLine + Xsite) != 0)//���ڴ�������߽���ȫΪ�ڵ㣬����һ�����ְ׵���ֱ������
                     break;//ֱ������������¼��߽磿��һ���ƺ�ֻ�������ǰ��һ����������
                   else if (Xsite ==(RowAttribute[Ysite].RightBorder - 1))
                   {
                       RowAttribute[Ysite].LeftBorder = Xsite;//һֱ���ұ߽綼δ���ְ׵㣬����ʱ�ұ߽��Ϊ��߽�
                       RowAttribute[Ysite].IsLeftFind = 'T';
                       break;
                   }
                 }
               }
               /**************************��������ߵĴ�����***************************/

               /**************************�����ұ��ߵĴ�����***************************/
                 if (RowAttribute[Ysite].IsRightFind == 'H')
                 {
                   for (Xsite = (RowAttribute[Ysite].RightBorder - 1);
                           Xsite >= (RowAttribute[Ysite].LeftBorder + 1); Xsite--)
                   {
                     if ((*(StoreSingleLine + Xsite) == 0) && (*(StoreSingleLine + Xsite - 1) != 0))
                     {
                         RowAttribute[Ysite].RightBorder =Xsite;
                         RowAttribute[Ysite].IsRightFind = 'T';
                         break;
                     }
                     else if (*(StoreSingleLine + Xsite) != 0)
                       break;
                     else if (Xsite == (RowAttribute[Ysite].LeftBorder + 1))
                     {
                         RowAttribute[Ysite].RightBorder = Xsite;
                         RowAttribute[Ysite].IsRightFind = 'T';
                         break;
                     }
                    }
                  }
            }
                   /**************************�����ұ��ߵĴ�����***************************/

           /*****************************����ȷ��������ı߽�******************************/


   /************************************���ޱ��н��д���****************************************************************/
           int ysite = 0;
           uint8 L_found_point = 0;
           uint8 R_found_point = 0;

               /**************************�����ұ��ߵ��ޱ���***************************/
           if (RowAttribute[Ysite].IsRightFind == 'W' && Ysite > 10 && Ysite < 50)//ʮ����ʮ��֮��
           {
             if (Get_R_line == 'F')
             {
               Get_R_line = 'T'; //һ֡ͼ��ֻ��һ��
               ytemp_W_R = Ysite + 2;//����������
               for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
               {
                 if (RowAttribute[ysite].IsRightFind =='T')
                 {
                     R_found_point++;
                 }
               }//��ǰѰ������������߽����
               if (R_found_point >8)//�������߽��г�������������ڲ���
               {       //��б�ʸ��ޱ��в�������׼
                 D_R = ((float)(RowAttribute[Ysite + R_found_point].RightBorder -
                         RowAttribute[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));//ͼ��Ϊ�����Σ�������һ��Ϊ��
                 if (D_R > 0)
                 {
                   R_Found_T ='T';
                 }
                 else
                 {
                   R_Found_T = 'F';
                   if (D_R < 0)
                   {
                       ExtenRFlag = 'F';   //�˱�־λ����ʮ�ֽǵ㲹��  ��ֹͼ����
                   }
                 }
               }
             }
             if (R_Found_T == 'T')
             {
                 RowAttribute[Ysite].RightBorder =
                         RowAttribute[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
                 //����ҵ��� ��ô�Ի�׼�����ӳ���
             }
             LimitL(RowAttribute[Ysite].RightBorder);  //�޷�
             LimitH(RowAttribute[Ysite].RightBorder);  //�޷�
           }
               /**************************�����ұ��ߵ��ޱ���***************************/

           /**************************��������ߵ��ޱ���***************************/
               if (RowAttribute[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 )
               {
                 if (Get_L_line == 'F')
                 {
                   Get_L_line = 'T';
                   ytemp_W_L = Ysite + 2;
                   for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
                   {
                     if (RowAttribute[ysite].IsLeftFind == 'T')
                       {
                         L_found_point++;
                       }
                   }
                   if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
                   {
                     D_L = ((float)(RowAttribute[Ysite + 3].LeftBorder -
                             RowAttribute[Ysite + L_found_point].LeftBorder)) /
                                     ((float)(L_found_point - 3));//����ͼ�������ɰ����Σ�������㷽ʽ��߽�б��һ��Ϊ��
                     if (D_L > 0)
                     {
                       L_Found_T = 'T';
                     }
                     else
                     {
                       L_Found_T = 'F';
                       if (D_L < 0)
                       {
                           ExtenLFlag = 'F';
                       }
                     }
                   }
                 }

                 if (L_Found_T == 'T')
                 {
                     RowAttribute[Ysite].LeftBorder =
                             RowAttribute[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
                 }

                 LimitL(RowAttribute[Ysite].LeftBorder);  //�޷�
                 LimitH(RowAttribute[Ysite].LeftBorder);  //�޷�
               }

               /**************************��������ߵ��ޱ���***************************/

   /************************************���ޱ��н��д���****************************************************************/

   /************************************����������������*************************************************/
           if (RowAttribute[Ysite].IsLeftFind == 'W' && RowAttribute[Ysite].IsRightFind == 'W')
                 {
                       ImageParameter.WhiteLine++;  //���Ҷ��ޱߣ��������ۼ�
                 }
           if (RowAttribute[Ysite].IsLeftFind == 'W'&&Ysite<55)
                {
                       ImageParameter.Miss_Left_lines++;
                }
           if (RowAttribute[Ysite].IsRightFind == 'W'&&Ysite<55)
                {
                       ImageParameter.Miss_Right_lines++;
                }
                       /*�޷�����*/
               LimitL(RowAttribute[Ysite].LeftBorder);
               LimitH(RowAttribute[Ysite].LeftBorder);
               LimitL(RowAttribute[Ysite].RightBorder);
               LimitH(RowAttribute[Ysite].RightBorder);

               RowAttribute[Ysite].Wide =
                       RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
               RowAttribute[Ysite].Center =
                       (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;

           if (RowAttribute[Ysite].Wide <= 6) //����ȷ�����Ӿ���
             {
                   ImageParameter.OFFLine = Ysite + 1;
                   break;
             }
             else if (RowAttribute[Ysite].RightBorder <= 10 || RowAttribute[Ysite].LeftBorder >= 70)
             {         //��ͼ����С��10�������ұߴﵽһ��������ʱ������ֹѲ��
                   ImageParameter.OFFLine = Ysite + 1;
                   break;
             }

   /************************************����������������*************************************************/
 
       }
       return;
    
    
    
    
    
   

    // wireless_printf("%d,%d,%d\n",D_R );
}

/***��ȡ�������ұ߽�***/
void Search_Bottom_Line(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //Ѱ����߽߱�
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            RowAttribute[Bottonline].LeftBoundary = Xsite;//��ȡ�ױ������
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < IPSW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            RowAttribute[Bottonline].RightBoundary = Xsite;//��ȡ�ױ��ұ���
            break;
        }
    }
}


void Search_Left_and_Right_Lines(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
/*  ǰ�������壺
                *   0
                * 3   1
                *   2
*/
/*Ѱ�����������*/
    uint8 Left_Rule[2][8] = {//�˸��������������Ҫʮ��������ʾ
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )//��ǰ�������ҷ������󷽣�����
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}//0.��ǰ����1.��ǰ����2.���·���3.���·�
    };
    /*Ѱ�����������*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},   (x,y)//��ǰ�������ҷ������󷽣�����
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}//0.��ǰ����1.��ǰ����2.���·���3.���·�
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;//��������������п�ʼ
    uint8 Left_Xsite = (uint8)RowAttribute[Bottonline].LeftBoundary;//��������������߽翪ʼ
    uint8 Left_Rirection = 0;//��߷���
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = (uint8)RowAttribute[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//�ұ߷���
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageParameter.OFFLineBoundary = 5;
    while (1)
    {//ѭ��ɨ������
            num++;
            if(num>400)
            {
                ImageParameter.OFFLineBoundary = Ysite;
                break;
            }//��ѭ�������İٴ�ʱ�˳�
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)//������������ڵ������ұ߽�����ʱ��ǰ��
        {//ȷ�����������ǰ�������ҷ�ͬΪ��ɫ��������������������һ�в���ֱ���˳�������Ҳ�����˲���Ѱ��Ѱ��ǰһ�е�λ�ã���λ���ɳ������Һ󷽼�����
            if (Ysite < ImageParameter.OFFLineBoundary)//������ֹ���˳�
            {
                ImageParameter.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }//����������ǰ��
        }
        /*********���Ѳ��*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary)//���ɨ��
        {//
            /*����ǰ������*///��ǰ����ʼ
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];//�������Ϊһ��ѭ���Ĺ���
            /**��ǰ������Ϊ�ںͰ׵��������������ǰ�����Ʋ��������ǰ�������ҷ�ͬʱΪ��ɫ����������������ɨ������Ϊ���������
             1.��ǰ��Ϊ�ף���ǰ��Ϊ�ڣ������Ϊ��ǰ��Ϊ��߽硣
             2.��ǰ��Ϊ�ף���ǰ��ҲΪ�ף���������ǰ���ĵ㣬���Ҵ����󷽿�ʼѰ�߽��
             3.��ǰ��Ϊ�ڣ����ҷ�Ϊ�ף���ǰ��Ϊ�ף��������Ϊ��ǰ��Ϊ��߽磬�������ڴ�ʱ��λ�����������ǰ����ʼ��
             4.��ǰ��Ϊ�ڣ����ҷ�Ϊ�ף���ǰ��Ϊ�ڣ�����ֱ�ӽ���ǰ����Ϊ����߽粢��������ǰ��Ѱ�߽�㣬
                 ����ǰ���ĵ�ʱ����ǰ����ǰ��Ϊ�ڣ������ҷ���ʼѰ�㣬����ǰ�����ƣ���ʱҲ���ܳ������ҷ�Ϊ��ɫ�������
                 �����һ�п��Խ���һ�е���ǰ����Ϊ��߽�㣬�����ڴ�ʱѰ�㷽λ��Ϊ���ҷ���ʼ����һ�е����ҷ�ͬ��Ҳ���ܳ���Ϊ��ɫ�������
             **/
            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//��ǰ������Ǻ�ɫ��ת���ҷ������ҷ����Ϊ��ɫ��ת���·�������ת����
            {
                //˳ʱ����ת90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//ǰ���ǰ�ɫ
            {
                /*��������*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//Ϊ��ɫ
                {
                    //���򲻱�  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];//��߽�������������һ��
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];//��߽����������������һ��
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0)
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;//��¼��߽�
                }
                else//Ϊ��ɫ
                {
                    // �������ı� Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];//ǰ������ǰ����Ϊ��ɫ��������ǰ���ĵ����Ѱ�߽��
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0 )
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;//���ظ���
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;//���ǰ������ǰ����Ϊ�׵㣬��������ǰ��֮������󷽿�ʼѰ��
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********�ұ�Ѳ��*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary)//�ұ�ɨ��
        {
            /*����ǰ������*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//�Ǻ�ɫ
            {
                //��ʱ����ת90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//�ǰ�ɫ
            {
                /*��������*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//Ϊ��ɫ
                {
                    //���򲻱�  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (RowAttribute[Right_Ysite].RightBoundary_First == 79 )
                        RowAttribute[Right_Ysite].RightBoundary_First = Right_Xsite;
                    RowAttribute[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//Ϊ��ɫ
                {
                    // �������ı� Right_Rirection  ��ʱ��90��
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (RowAttribute[Right_Ysite].RightBoundary_First == 79)
                        RowAttribute[Right_Ysite].RightBoundary_First = Right_Xsite;
                    RowAttribute[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }
            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80��Ϊ�˷��ڵײ��ǰ�����ɨ�����  3 && Ysite < 30
        {
            ImageParameter.OFFLineBoundary = Ysite;
            break;
        }
    }
}


/***������ɨ��***/
/***ͼ�� ����  �����***/
void Eightfields_Search_Border(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageParameter.WhiteLine_L = 0;
    ImageParameter.WhiteLine_R = 0;
    //ImageParameter.OFFLine = 1;
    /*�����±߽紦��*/
    for (int Xsite = 0; Xsite < IPSW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*�����ұ߽紦��*/
    for (int Ysite = 0; Ysite < IPSH; Ysite++)
    {
            RowAttribute[Ysite].LeftBoundary_First = 0;
            RowAttribute[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][IPSW - 1] = 0;
    }
    /********��ȡ�ײ�����*********/
    Search_Bottom_Line(imageInput, Row, Col, Bottonline);
    /********��ȡ���ұ���*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);//�������㷨ɨ���ұ���

    for (int Ysite = Bottonline; Ysite > ImageParameter.OFFLineBoundary + 1; Ysite--)
    {
        if (RowAttribute[Ysite].LeftBoundary < 3)
        {
            ImageParameter.WhiteLine_L++;
        }
        if (RowAttribute[Ysite].RightBoundary > IPSW - 3)
        {
            ImageParameter.WhiteLine_R++;
        }
    }
}

/***ֱ�����ټ��***/
float variance_acc;  //����
void Straightacc_Test(void){
    int sum = 0;
    for (Ysite = 55; Ysite > ImageParameter.OFFLine + 1; Ysite--) {
      sum += (RowAttribute[Ysite].Center - PictureCentring) *(RowAttribute[Ysite].Center - PictureCentring);//����ʵ��������������ߵķ���
    }
    variance_acc = (float)sum / (54 - ImageParameter.OFFLine);

    if ( variance_acc < ImageParameter.variance_acc && ImageParameter.OFFLine <= 10)
    {
        ImageParameter.straight_acc = 1;
    } else
        ImageParameter.straight_acc = 0;
}


/*****************ֱ���ж�******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //���ؽ��С��1��Ϊֱ��
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)//1��2��
    {
        case 1:k = (float)(RowAttribute[start].LeftBorder -
                RowAttribute[end].LeftBorder) / (start - end);
            for (i = 0; i < end - start; i++)
            {
                Err = (RowAttribute[start].LeftBorder +
                        k * i - RowAttribute[i + start].LeftBorder) *
                                (RowAttribute[start].LeftBorder + k * i -
                                        RowAttribute[i + start].LeftBorder);
                Sum += Err;
            }
            S = Sum / (end - start);
            break;
        case 2:k = (float)(RowAttribute[start].RightBorder -
                RowAttribute[end].RightBorder) / (start - end);//����б��
            for (i = 0; i < end - start; i++)
            {
                Err = (RowAttribute[start].RightBorder +
                        k * i - RowAttribute[i + start].RightBorder) *
                                (RowAttribute[start].RightBorder + k * i -
                                        RowAttribute[i + start].RightBorder);
                Sum += Err;
            }
            S = Sum / (end - start);
            break;
    }
    return S;
}


/***��Բ�����***/
void Left_Rings_Judgment(void){
    if (   ImageParameter.WhiteLine_R > 5 || ImageParameter.WhiteLine_L < 5
        || ImageParameter.OFFLine > 10 || Straight_Judge(2, 5, 55) > 1
        ||  ImageParameter.WhiteLine>4
//        || RowAttribute[52].IsLeftFind == 'W'
//        || RowAttribute[53].IsLeftFind == 'W'
//        || RowAttribute[54].IsLeftFind == 'W'
//        || RowAttribute[55].IsLeftFind == 'W'
//        || RowAttribute[56].IsLeftFind == 'W'
//        || RowAttribute[57].IsLeftFind == 'W'
//        || RowAttribute[58].IsLeftFind == 'W'
        )//�ڷ�ֱ�����ұ��߶��ߴ���3������߶���С��5���ֹ�д���10��˫�߶���������4����ǰ������ʱ��ֱ���˳���
        return;//�˳�����������ֹ���м�Ԫ�ز���Բ����
    //beepopen;
    int ring_ysite = 25;//��ⷶΧ
    uint8 Left_Less_Num = 0;//����ͳ�ƶ�������
    Left_RingsFlag_Point1_Ysite = 0;//���ڼ�¼Բ���ĵ�һ����
    Left_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite].LeftBoundary_First - RowAttribute[Ysite - 1].LeftBoundary_First > 4)//����⵽������߽������һ��
            {
                Left_RingsFlag_Point1_Ysite = Ysite;//��Ϊ��⵽Բ���ĵ�һ�����������
              //  ips200_show_int(x1[0],y[10],Left_RingsFlag_Point1_Ysite,2);
                break;
            }
        }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite + 1].LeftBoundary - RowAttribute[Ysite].LeftBoundary > 4)//����⵽��һ�п��ڱ���
            {
                Left_RingsFlag_Point2_Ysite = Ysite;//ͬ����⵽Բ���ĵ�һ�����������
              //  ips200_show_int(x1[1],y[10],Left_RingsFlag_Point1_Ysite,2);
                break;
            }
        }
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > Left_RingsFlag_Point1_Ysite - 11; Ysite--)
    {
        if(Ysite <= 0)
            break;
        if (RowAttribute[Ysite].IsLeftFind == 'W')
            Left_Less_Num++;
    }//ͳ�ƶ�������
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageParameter.OFFLine; Ysite--)
    {
        if (   RowAttribute[Ysite + 6].LeftBorder < RowAttribute[Ysite+3].LeftBorder
            && RowAttribute[Ysite + 5].LeftBorder < RowAttribute[Ysite+3].LeftBorder
            && RowAttribute[Ysite + 3].LeftBorder > RowAttribute[Ysite + 2].LeftBorder
            && RowAttribute[Ysite + 3].LeftBorder > RowAttribute[Ysite + 1].LeftBorder
            )//�Ӽ��Բ����һ��������п�ʼ����������ڲ����ߣ���ǰ�����еı߽�Ҫ������ǰ�����и����У������������ں�����߶�����˺���߽�ӦС�ڱ���
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 0)//
    {
        if(ImageParameter.Miss_Left_lines > 10)
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 1)
    {
        ImageParameter.image_element_rings = 1;
        ImageParameter.image_element_rings_flag = 1;
        ImageParameter.ring_big_small=1;
        beepopen;

    }
    Ring_Help_Flag = 0;
}

/***��Բ��ʶ��***/
void Right_Rings_Judgment(void){
    if (   ImageParameter.WhiteLine_L > 5
            || ImageParameter.WhiteLine_R < 5
            || ImageParameter.OFFLine > 10
            || Straight_Judge(1, 5, 55) > 1
            || ImageParameter.image_element_rings == 1
//            || RowAttribute[52].IsRightFind == 'W'
////            || RowAttribute[53].IsRightFind == 'W'
//            || RowAttribute[54].IsRightFind == 'W'
//            || RowAttribute[55].IsRightFind == 'W'
//            || RowAttribute[56].IsRightFind == 'W'
//            || RowAttribute[57].IsRightFind == 'W'
//            || RowAttribute[58].IsRightFind == 'W'
            )
            return;
//        beepopen;
        int ring_ysite = 25;
        uint8 Right_Less_Num = 0;
        Right_RingsFlag_Point1_Ysite = 0;
        Right_RingsFlag_Point2_Ysite = 0;
        for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite - 1].RightBoundary_First - RowAttribute[Ysite].RightBoundary_First > 4)
            {
                Right_RingsFlag_Point1_Ysite = Ysite;
                break;
            }
        }
        for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite].RightBoundary - RowAttribute[Ysite + 1].RightBoundary > 4)
            {
                Right_RingsFlag_Point2_Ysite = Ysite;
                break;
            }
        }
        for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > Right_RingsFlag_Point1_Ysite - 11; Ysite--)
        {
            if(Ysite <= 0)
                break;
            if (RowAttribute[Ysite].IsRightFind == 'W')
                Right_Less_Num++;
        }
        for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > ImageParameter.OFFLine; Ysite--)
        {
            if (   RowAttribute[Ysite + 6].RightBorder > RowAttribute[Ysite + 3].RightBorder
                && RowAttribute[Ysite + 5].RightBorder > RowAttribute[Ysite + 3].RightBorder
                && RowAttribute[Ysite + 3].RightBorder < RowAttribute[Ysite + 2].RightBorder
                && RowAttribute[Ysite + 3].RightBorder < RowAttribute[Ysite + 1].RightBorder
               )
            {
                Ring_Help_Flag = 1;
                break;
            }
        }
        if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 0 && Right_Less_Num > 3)
        {
            if(ImageParameter.Miss_Right_lines>10)
                Ring_Help_Flag = 1;
        }
        if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 1 && Right_Less_Num > 3)
        {
            ImageParameter.image_element_rings = 2;
            ImageParameter.image_element_rings_flag = 1;
            ImageParameter.ring_big_small=1;
        }
        Ring_Help_Flag = 0;
}

/*****************�µ��ж�******************/
void Element_Judgment_Ramp(void)
{
    if (ImageParameter.Ramp_Flag) return;

}

/***�����߼��***/
void Zebra_Judgment(void){
    if (ImageParameter.Zebra_Flag) return;//ֱ���˳�����

    int NUM = 0, net = 0;
    if(ImageParameter.OFFLineBoundary<20)
    {
        for (int Ysite = 40; Ysite < 53; Ysite++)
        {
            net = 0;
            for (int Xsite = RowAttribute[Ysite].LeftBoundary ; Xsite < RowAttribute[Ysite].RightBoundary; Xsite++)
            {
                if (Binarization_Image[Ysite][Xsite] == 0 && Binarization_Image[Ysite][Xsite + 1] == 1)
                {
                    net++;
                    if (net > 2)
                        NUM++;
                }
            }//����߽絽�ұ߽��������
        }
    }
    if (NUM >= 5 && ImageParameter.Zebra_Flag == 0)
    {
        ImageParameter.Zebra_Flag = 1;
    }
}

/******/
void Speed_Control_Factor(void)
{
    float SpeedGain = 0;

    SpeedGain = //   ��ʱ������ʹ�� 5.14
            (SystemData.SpeedData.nowspeed -
                    SystemData.SpeedData.MinSpeed) * 0.2 +0.5 ;
    if (SpeedGain >= 3)//ǰհ�����޷�
         SpeedGain = 3;
    else if (SpeedGain <= -1)
     SpeedGain = -1;
}


/***Ԫ��ʶ����***/
void Element_Test(void){
    Straightacc_Test();//ֱ�����ټ��
//    if(circle_count_flag < 4 && ImageParameter.image_element_rings_flag == 0
//            && ImageParameter.WhiteLine < 1 )//˫�߶�����Ϊ0��Բ��������ʱ���Բ��
//    {
       // Left_Rings_Judgment();      //��Բ�����
       // Right_Rings_Judgment();     //��Բ�����
//    }
    /*if(flag_test_stop==1)
    {
      Zebra_Judgment();//�����߼��
    }*/
}

/***��Բ������***/
void Left_Rings_Handle(void){
    /***************************************Բ�����̵ĸ����׶ε��ж�**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {

        if(RowAttribute[Ysite].IsLeftFind == 'W')
            num++;//ͳ����߶�����
        if(    RowAttribute[Ysite+3].IsLeftFind == 'W' && RowAttribute[Ysite+2].IsLeftFind == 'W'
            && RowAttribute[Ysite+1].IsLeftFind == 'W' && RowAttribute[Ysite].IsLeftFind == 'T')//����⵽Բ���������һ���յ�ʱ��ʼ׼���뻷
            break;
    }
  //  ips200_show_int(x1[5],y[9],num,2);
    //׼������
    if (ImageParameter.image_element_rings_flag == 1 && num>10)
    {
        beepopen;
        ImageParameter.image_element_rings_flag = 2;
    }
    if (ImageParameter.image_element_rings_flag == 2 && num<10)
    {
        beepopen;
        ImageParameter.image_element_rings_flag = 3;
    }
    if(ImageParameter.image_element_rings_flag == 3 && num<10)//
    {
        beepopen;
        ImageParameter.image_element_rings_flag = 5;
    }

    //����
    if(ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_R>15)//��������Բ����ƫ���Ҳඪ��
    {
        beepopen;
        ImageParameter.image_element_rings_flag = 6;
    }
        //����СԲ��
    if(ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_R<10)//��������Բ����ƫ���Ҳඪ��
    {
        beepopen;
        //Stop = 1;
        ImageParameter.image_element_rings_flag = 7;
    }
        //���� ��Բ���ж�
    if (ImageParameter.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 45; Ysite > ImageParameter.OFFLine + 7; Ysite--)
        {
            if (    RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 2].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 2].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 1].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 1].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 4].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 4].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 6].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 6].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite + 5].RightBorder
                 && RowAttribute[Ysite].RightBorder <= RowAttribute[Ysite - 5].RightBorder
               )
            {
                Point_Xsite = RowAttribute[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 22)
        {
            beepopen;
            ImageParameter.image_element_rings_flag = 8;
            //Stop = 1;
        }
    }
        //���� СԲ���ж�
    if (ImageParameter.image_element_rings_flag == 7 && ImageParameter.ring_big_small == 2)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageParameter.OFFLineBoundary + 3; Ysite--)
        {
            if (    RowAttribute[Ysite].RightBoundary < RowAttribute[Ysite + 2].RightBoundary
                 && RowAttribute[Ysite].RightBoundary < RowAttribute[Ysite - 2].RightBoundary
               )
            {
                Point_Xsite = RowAttribute[Ysite].RightBoundary;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 20)
            ImageParameter.image_element_rings_flag = 8;
    }
        //������
    if (ImageParameter.image_element_rings_flag == 8)
    {
        if (    Straight_Judge(2, ImageParameter.OFFLine+15, 50) < 1
             && ImageParameter.WhiteLine_R < 10
             && ImageParameter.OFFLine < 7)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С

            ImageParameter.image_element_rings_flag = 9;
        beepopen;
    }
        //����Բ������

    if (ImageParameter.image_element_rings_flag == 9)
    {
        int num=0;
        for (int Ysite = 50; Ysite > 10; Ysite--)
        {
            if(RowAttribute[Ysite].IsLeftFind == 'W' )
                num++;
        }
        if(num < 5)
        {
            ImageParameter.image_element_rings_flag = 0;
            ImageParameter.image_element_rings = 0;
            ImageParameter.ring_big_small = 0;
            //circle_count_flag++;
        }
    }
    /***************************************����**************************************/
           //׼������  �����
       if (   ImageParameter.image_element_rings_flag == 1
           || ImageParameter.image_element_rings_flag == 2
           || ImageParameter.image_element_rings_flag == 3
           || ImageParameter.image_element_rings_flag == 4)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
           }//����������Ӧ��Ϊ�ó�ƫ����ߣ������뻷
       }
           //����  ����
       if  ( ImageParameter.image_element_rings_flag == 5
           ||ImageParameter.image_element_rings_flag == 6
           )
       {
           int  flag_Xsite_1=0;
           int flag_Ysite_1=0;
           float Slope_Rings=0;
           for(Ysite=55;Ysite>ImageParameter.OFFLine;Ysite--)//���满��
           {
               for(Xsite=RowAttribute[Ysite].LeftBorder + 1;Xsite<RowAttribute[Ysite].RightBorder - 1;Xsite++)
               {
                   if(  Binarization_Image[Ysite][Xsite] == 1 && Binarization_Image[Ysite][Xsite + 1] == 0)//Ѱ�ҽ�����������һ����
                    {
                      flag_Ysite_1 = Ysite;
                      flag_Xsite_1 = Xsite;
                      Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                      break;
                    }
               }
               if(flag_Ysite_1 != 0)//��⵽�õ���˳�
               {
                   break;
               }
           }
           if(flag_Ysite_1 == 0)//û��⵽�õ�
           {
               for(Ysite=ImageParameter.OFFLine+1;Ysite<30;Ysite++)
               {
                   if(RowAttribute[Ysite].IsLeftFind=='T'&&RowAttribute[Ysite+1].IsLeftFind=='T'&&RowAttribute[Ysite+2].IsLeftFind=='W'
                       &&abs(RowAttribute[Ysite].LeftBorder-RowAttribute[Ysite+2].LeftBorder)>10
                     )//���������ٴμ��õ�
                   {
                       flag_Ysite_1=Ysite;
                       flag_Xsite_1=RowAttribute[flag_Ysite_1].LeftBorder;
                       ImageParameter.OFFLine=(uint8)Ysite;
                       Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);//���½ǵĵ���뻷�����ߵ�б��
                       break;
                   }

               }
           }
           //����
           if(flag_Ysite_1 != 0)
           {
               for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
               {
                   RowAttribute[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);//�����½ǿ�ʼ���뻷������
                       RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder)/2;
               }
               RowAttribute[flag_Ysite_1].RightBorder=flag_Xsite_1;
               for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
               {
                   for(Xsite=RowAttribute[Ysite+1].RightBorder-10;Xsite<RowAttribute[Ysite+1].RightBorder+2;Xsite++)
                   {
                       if(Binarization_Image[Ysite][Xsite]==1 && Binarization_Image[Ysite][Xsite+1]==0)
                       {
                           RowAttribute[Ysite].RightBorder=Xsite;
                           RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder)/2;
                           RowAttribute[Ysite].Wide=RowAttribute[Ysite].RightBorder-RowAttribute[Ysite].LeftBorder;
                           break;
                       }
                   }

                   if(RowAttribute[Ysite].Wide>8 &&RowAttribute[Ysite].RightBorder< RowAttribute[Ysite+2].RightBorder)
                   {
                       continue;
                   }
                   else
                   {
                       ImageParameter.OFFLine=Ysite+2;
                       break;
                   }
               }//��������ɨ�ߵ����ã�
           }
       }//����56�����ֹ
           //���� С���������� �󻷲���
       if (ImageParameter.image_element_rings_flag == 7)
       {
   //        for (int Ysite = 57; Ysite > ImageParameter.OFFLine+1; Ysite--)
   //        {
   //            if(ImageFlag.ring_big_small==2)
   //                RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Bend_Wide[Ysite];
   //            if(RowAttribute[Ysite].Center<=0)
   //            {
   //                RowAttribute[Ysite].Center = 0;
   //                ImageParameter.OFFLine=Ysite-1;
   //                break;
   //            }
   //        }
       }
           //��Բ������ ����
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1)    //��Բ��
       {
           Repair_Point_Xsite = 20;
           Repair_Point_Ysite = 5;
           for (int Ysite = 50; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][23] == 1 && Binarization_Image[Ysite-1][23] == 0)//28
               {
                   Repair_Point_Xsite = 23;//����Ѱ��һ������23����
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                   break;
               }//����Ѱ�㣬Ѱ�Ǹ������ұߵĹյ�
           }
           for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
           {
               RowAttribute[Ysite].RightBorder = (RowAttribute[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].RightBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //СԲ������ ����
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2)    //СԲ��
       {
           Repair_Point_Xsite = 0;
           Repair_Point_Ysite = 0;
           for (int Ysite = 55; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][15] == 1 && Binarization_Image[Ysite-1][15] == 0)
               {
                   Repair_Point_Xsite = 15;//����Ѱ��һ������15����
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                   break;
               }
           }
           for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
           {
               RowAttribute[Ysite].RightBorder = (RowAttribute[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].RightBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //�ѳ��� �����
       if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
               flag_test=1;
           }
       }//��������������ͷ
}

/***��Բ������***/
void Right_Rings_Handle(void){
    int num =0 ;
       for (int Ysite = 55; Ysite > 30; Ysite--)
       {
           if(RowAttribute[Ysite].IsRightFind == 'W')
               num++;
           if(    RowAttribute[Ysite+3].IsRightFind == 'W' && RowAttribute[Ysite+2].IsRightFind == 'W'
               && RowAttribute[Ysite+1].IsRightFind == 'W' && RowAttribute[Ysite].IsRightFind == 'T')
               break;
       }
           //׼������
       if (ImageParameter.image_element_rings_flag == 1 && num>10)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 2;
       }
       if (ImageParameter.image_element_rings_flag == 2 && num<10)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 5;
       }
           //����
       if(ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_L>15)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 6;
       }
           //����СԲ��
       if(ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_L<10)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 7;

           //����Ŀǰֱ�Ӹ��ƴ�Բ�� ��СԲ���ж�������
       }
           //���� ��Բ���ж�
       if ( ImageParameter.image_element_rings_flag == 7)
       {
           Point_Xsite = 0;
           Point_Ysite = 0;
           for (int Ysite = 45; Ysite > ImageParameter.OFFLine + 7; Ysite--)
           {
               if (    RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 2].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 2].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 1].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 1].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 4].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 4].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 5].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 5].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite + 6].LeftBorder
                    && RowAttribute[Ysite].LeftBorder >= RowAttribute[Ysite - 6].LeftBorder
                   )

               {
                           Point_Xsite = RowAttribute[Ysite].LeftBorder;
                           Point_Ysite = Ysite;
                           break;
               }
           }
           if (Point_Ysite > 22)
           {
               beepopen;
               ImageParameter.image_element_rings_flag = 8;
           }
       }
           //���� СԲ���ж�
       if (ImageParameter.ring_big_small == 2 && ImageParameter.image_element_rings_flag == 7)
       {
           Point_Xsite = 0;
           Point_Ysite = 0;
           for (int Ysite = 50; Ysite > ImageParameter.OFFLineBoundary+3; Ysite--)
           {
               if (  RowAttribute[Ysite].LeftBoundary > RowAttribute[Ysite + 2].LeftBoundary
                  && RowAttribute[Ysite].LeftBoundary > RowAttribute[Ysite - 2].LeftBoundary
                 )

               {
                         Point_Xsite = RowAttribute[Ysite].LeftBoundary;
                         Point_Ysite = Ysite;
                         break;
               }
           }
           if (Point_Ysite > 20)
           {
               beepopen;
               ImageParameter.image_element_rings_flag = 8;
           }
       }
           //������
       if (ImageParameter.image_element_rings_flag == 8)
       {

            if (   Straight_Judge(1, ImageParameter.OFFLine+15, 40) < 1
                && ImageParameter.WhiteLine_L < 10
                && ImageParameter.OFFLine < 7)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С
   //        if (    (((-8<(ring_yaw-yaw))&&((ring_yaw-yaw)<8)) ||
   //                ((-360<(ring_yaw-yaw))&&((ring_yaw-yaw)<-352)) ||
   //                ((352<(ring_yaw-yaw))&&((ring_yaw-yaw)<360)))
   //                && ImageParameter.WhiteLine_L < 10
   //                && ImageParameter.OFFLine < 7

   //            )    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С
               {
                beepopen;
                ImageParameter.image_element_rings_flag = 9;
               }
       }

       //����Բ������
       if (ImageParameter.image_element_rings_flag == 9)
       {
           int num=0;
           for (int Ysite = 50; Ysite > 10; Ysite--)
           {
               if(RowAttribute[Ysite].IsRightFind == 'W' )
                   num++;
           }
   //        if(num < 5)
           if(num < 3)
           {
               ImageParameter.image_element_rings_flag = 0;
               ImageParameter.image_element_rings = 0;
               ImageParameter.ring_big_small = 0;
               //circle_count_flag++;
           }
       }

       /***************************************����**************************************/
            //׼������  �����
       if (   ImageParameter.image_element_rings_flag == 1
           || ImageParameter.image_element_rings_flag == 2
           || ImageParameter.image_element_rings_flag == 3
           || ImageParameter.image_element_rings_flag == 4)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Road_Wide[Ysite];
           }
       }

           //����  ����
       if (   ImageParameter.image_element_rings_flag == 5
           || ImageParameter.image_element_rings_flag == 6
          )
       {
           int flag_Xsite_1=0;
           int  flag_Ysite_1=0;
           float Slope_Right_Rings = 0;
           for(Ysite=55;Ysite>ImageParameter.OFFLine;Ysite--)
           {
               for(Xsite=RowAttribute[Ysite].LeftBorder + 1;Xsite<RowAttribute[Ysite].RightBorder - 1;Xsite++)
               {
                   if(Binarization_Image[Ysite][Xsite]==1 && Binarization_Image[Ysite][Xsite+1]==0)
                   {
                       flag_Ysite_1=Ysite;
                       flag_Xsite_1=Xsite;
                       Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                       break;
                   }
               }
               if(flag_Ysite_1!=0)
               {
                 break;
               }
           }
           if(flag_Ysite_1==0)
           {
           for(Ysite=ImageParameter.OFFLine+1;Ysite<30;Ysite++)
           {
            if(RowAttribute[Ysite].IsRightFind=='T'&&RowAttribute[Ysite+1].IsRightFind=='T'&&RowAttribute[Ysite+2].IsRightFind=='W'
                  &&abs(RowAttribute[Ysite].RightBorder-RowAttribute[Ysite+2].RightBorder)>10
            )
            {
                flag_Ysite_1=Ysite;
                flag_Xsite_1=RowAttribute[flag_Ysite_1].RightBorder;
                ImageParameter.OFFLine=(uint8)Ysite;
                Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                break;
            }

           }

           }
           //����
           if(flag_Ysite_1!=0)
           {
               for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
               {
                   RowAttribute[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
                   //if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
                   //    RowAttribute[Ysite].Center=RowAttribute[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
   //              else//��Բ�����Ӱ��
                       RowAttribute[Ysite].Center=(RowAttribute[Ysite].LeftBorder+RowAttribute[Ysite].RightBorder)/2;//���
                   //if(RowAttribute[Ysite].Center>79)
                   //    RowAttribute[Ysite].Center=79;
               }
               RowAttribute[flag_Ysite_1].LeftBorder=flag_Xsite_1;
               for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
               {
                   for(Xsite=RowAttribute[Ysite+1].LeftBorder+8;Xsite>RowAttribute[Ysite+1].LeftBorder-4;Xsite--)
                   {
                       if(Binarization_Image[Ysite][Xsite]==1 && Binarization_Image[Ysite][Xsite-1]==0)
                       {
                        RowAttribute[Ysite].LeftBorder=Xsite;
                        RowAttribute[Ysite].Wide=RowAttribute[Ysite].RightBorder-RowAttribute[Ysite].LeftBorder;
                     //   if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
                     //       RowAttribute[Ysite].Center=RowAttribute[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
                      //  else//��Բ�����Ӱ��
                            RowAttribute[Ysite].Center=(RowAttribute[Ysite].LeftBorder+RowAttribute[Ysite].RightBorder)/2;//���
                      //  if(RowAttribute[Ysite].Center>79)
                      //      RowAttribute[Ysite].Center=79;
                        break;
                       }
                   }
                   if(RowAttribute[Ysite].Wide>8 && RowAttribute[Ysite].LeftBorder>  RowAttribute[Ysite+2].LeftBorder)
                   {
                       continue;
                   }
                   else
                   {
                       ImageParameter.OFFLine=Ysite+2;
                       break;
                   }
               }
           }


       }
           //���ڲ�����
       if (ImageParameter.image_element_rings_flag == 7)
       {
   //        for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
   //        {
   //            if(ImageFlag.ring_big_small==2)
   //                RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
   //            if(RowAttribute[Ysite].Center >= 79)
   //            {
   //                RowAttribute[Ysite].Center = 79;
   //                ImageParameter.OFFLine=Ysite-1;
   //                break;
   //            }
   //        }
       }

           //��Բ������ ����
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1)  //��Բ��
       {
           Repair_Point_Xsite = 60;
           Repair_Point_Ysite = 0;
           for (int Ysite = 50; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][57] == 1 && Binarization_Image[Ysite-1][57] == 0)
               {
                   Repair_Point_Xsite = 57;
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                           //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                   break;
               }
           }
           for (int Ysite = 57; Ysite >
           Repair_Point_Ysite-3; Ysite--)         //����
           {
               RowAttribute[Ysite].LeftBorder = (RowAttribute[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].LeftBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //СԲ������ ����
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2)  //СԲ��
       {
           Repair_Point_Xsite = 79;
           Repair_Point_Ysite = 0;
           for (int Ysite = 40; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][58] == 1 && Binarization_Image[Ysite-1][58] == 0)
               {
                   Repair_Point_Xsite = 58;
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                           //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                   break;
               }
           }
           for (int Ysite = 55; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
           {
               RowAttribute[Ysite].LeftBorder = (RowAttribute[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].LeftBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //�ѳ��� �����
       if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Road_Wide[Ysite];
               flag_test=1;
           }
       }
}

/***ʮ�ִ���***/
static void Get_ExtensionLine(void){
    if (ImageParameter.WhiteLine >= 8) //�����е���������8
               TFSite = 55;      //TFSite��ֵΪ55������������㲹��б�ʵ�һ������
       left_FTSite = 0;
       right_FTSite = 0;
       /********************************************��߲���********************************************/
         if (ExtenLFlag != 'F') //��ʼ���߲���
             for (Ysite = 54; Ysite >= (ImageParameter.OFFLine + 4);Ysite--)
                 {
                     StoreSingleLine = Binarization_Image[Ysite];           //�浱ǰ��
                     if (RowAttribute[Ysite].IsLeftFind =='W')
                     {
                       //**************************************************//**************************************************
                       if (RowAttribute[Ysite + 1].LeftBorder >= 70)  //��߽�̫��
                       {
                           ImageParameter.OFFLine = Ysite + 1;
                           break;                        //ֱ�����������������
                       }
                       //************************************************//*************************************************
                       while (Ysite >= (ImageParameter.OFFLine + 4))//��߽�����
                       {
                           Ysite--;
                           if (RowAttribute[Ysite].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 1].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 2].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 2].LeftBorder > 0
                             &&RowAttribute[Ysite - 2].LeftBorder < 70
                             )                            //�ޱ��е������������ж�����������
                           {
                             left_FTSite = Ysite - 2;    //�ѱ�������ĵڶ��д���FTsite
                             break;
                           }
                       }
                       /*��߽��б��*/
                       DetL =
                         ((float)(RowAttribute[left_FTSite].LeftBorder -
                                 RowAttribute[TFSite].LeftBorder)) /
                         ((float)(left_FTSite - TFSite));
                       /*���߲���*/
                       if (left_FTSite > ImageParameter.OFFLine) //���в��ɳ��������Ƶ�ͼ�񶥱�
                           for (ytemp = TFSite; ytemp >= left_FTSite; ytemp--)
                           {
                               RowAttribute[ytemp].LeftBorder = //y = k*x + b;
                                       (int)(DetL * ((float)(ytemp - TFSite))) +
                                       RowAttribute[TFSite].LeftBorder;
                           }//������߲��Ǵ�ʮ�ֶ��ߵ���ʼ�㵽���ߵ���ֹ����������
                   }//�ޱ��д���
                   else
                       TFSite = Ysite + 2; //���ɨ���˱��е���߽磬���д��������棬����б�ʣ�
                 }
}

/***����ƽ���˲�����***/
static void RouteFilter(void){
    for (Ysite = 58; Ysite >= (ImageParameter.OFFLine + 5);
         Ysite--)
    {
      if (   RowAttribute[Ysite].IsLeftFind == 'W'
           &&RowAttribute[Ysite].IsRightFind == 'W'
           &&Ysite <= 45
           &&RowAttribute[Ysite - 1].IsLeftFind == 'W'
           &&RowAttribute[Ysite - 1].IsRightFind =='W')  //��ǰ�����Ҷ��ޱ���ǰһ��Ҳ�ޱߣ�������ǰ45��   �˲�
      {
        ytemp = Ysite;
        while (ytemp >= (ImageParameter.OFFLine +5))
        {
          ytemp--;
          if (  RowAttribute[ytemp].IsLeftFind == 'T'
              &&RowAttribute[ytemp].IsRightFind == 'T')  //Ѱ�����߶������ģ��ҵ��뱾������ľͲ�����
          {
            DetR = (float)(RowAttribute[ytemp - 1].Center - RowAttribute[Ysite + 2].Center)
                    /(float)(ytemp - 1 - Ysite - 2);          //��б��
            int CenterTemp = RowAttribute[Ysite + 2].Center;
            int LineTemp = Ysite + 2;
            while (Ysite >= ytemp) {
                RowAttribute[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp)); //��б�ʲ�
              Ysite--;
            }
            break;
          }
        }
      }
      RowAttribute[Ysite].Center =
              (RowAttribute[Ysite - 1].Center + 2 * RowAttribute[Ysite].Center) /3; //��ƽ����Ӧ�û�Ƚϻ�  ��������������ƽ��
    }
}//��������˲�������ӦΪ:��ǰ��ʮ������������������Ҷ������Ҳ���Ԫ��ʱ����������˲�����


/***ǰհ��������***/
void Prospective_error(void){
    float DetTemp = 0;
    int TowPoint = 0;
    float UnitAll = 0;

    //Speed_Control_Factor();
    /*ǰհ����*/
    if(ImageParameter.image_element_rings_flag != 0)  1;/*TowPoint = circle[circle_count_flag];*/
           //?
    else TowPoint = ImageParameter.TowPoint;//25

    /*ǰհ�޷�*/
    if (TowPoint < ImageParameter.OFFLine)
            TowPoint = ImageParameter.OFFLine + 1;
    if (TowPoint >= 49) TowPoint = 49;

    if ((TowPoint - 5) >= ImageParameter.OFFLine) { //ǰհȡ�趨ǰհ���ǿ��Ӿ���  ��Ҫ���������
        for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
          DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (RowAttribute[Ysite].Center);
          UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
          DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
          UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[TowPoint].Center + DetTemp) / (UnitAll + 1);
      }
    else if (TowPoint > ImageParameter.OFFLine) {
        for (Ysite = ImageParameter.OFFLine; Ysite < TowPoint; Ysite++) {
          DetTemp += Weighting[TowPoint - Ysite - 1] * (RowAttribute[Ysite].Center);
          UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + TowPoint - ImageParameter.OFFLine); Ysite > TowPoint;
             Ysite--) {
          DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
          UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[Ysite].Center + DetTemp) / (UnitAll + 1);
      }
    else if (ImageParameter.OFFLine < 49) {
        for (Ysite = (ImageParameter.OFFLine + 3); Ysite > ImageParameter.OFFLine;
             Ysite--) {
          DetTemp += Weighting[-TowPoint + Ysite - 1] * (RowAttribute[Ysite].Center);
          UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (RowAttribute[ImageParameter.OFFLine].Center + DetTemp) / (UnitAll + 1);
      }
    else
        DetTemp = (float)ImageParameter.Det_True;//����ǳ���OFFLine>50�����������һ�ε�ƫ��ֵ

    ImageParameter.Det_True = DetTemp;//��ʱ�Ľ��������ƽ��ͼ��ƫ������������Ӧ��Ϊƽ�����߶�����ƽ�����
}


/***�����ƶ�����***/
volatile uint8_t Emergency_Flag = 0; // ����ȫ�ּ�ͣ��־

// �޸ĺ��Emergency_Breaking��������ѭ�����ã�
void Emergency_Breaking(void) {     
    static uint8_t safety_cnt = 0; // ��Ϊ��̬����
    
    if(ImageParameter.OFFLine >= 55) {
        if(safety_cnt < 5) safety_cnt++;  // ��������
        if(safety_cnt >= 3) {             // ����3֡�쳣�Ŵ���
            Emergency_Flag = 1;           // ���ñ�־����ֱ�Ӳ���Stop
        }
    } else {
        safety_cnt = 0;
        Emergency_Flag = 0;  // �Զ����
    }
}



/***��ʾ����ֵ***/
void printf_halfwide(void)
{
    for(int i = 0 ; i < 60 ; i++)
    {
        printf("%d\n",(RowAttribute[i].RightBorder - RowAttribute[i].LeftBorder)/2);
    }
}


/***Ԫ�ش���*****/
void Element_Handle(void){


    if (ImageParameter.image_element_rings == 1)
         Left_Rings_Handle();//��Բ������
    else if (ImageParameter.image_element_rings == 2)
        Right_Rings_Handle();//��Բ������


    if(ImageParameter.WhiteLine >= 8)
    {
        flag_cross=1;
        Get_ExtensionLine();//ʮ�ֲ���
    }
    else flag_cross=0;
    RouteFilter();//����ƽ���˲�������ͻȻ���ֵķ�Ԫ�ض������
   // Emergency_Breaking();//�����ƶ�  ��ʱ�ص�
}

// ������̬��ֵ�ͷָ���ֵ
uint32 Threshold_low = 30;  // ԭ40 �� 30 ��ǿ�������ж�
uint8 detach = 220;         // ԭ240 �� 220 ���ͷָ���ֵ
int MaxSpeed = 270;
int MinSpeed = 190;
int ance_acc = 70;

void Data_Settings(void){//ͼ�������ʼ��
    ImageParameter.Threshold_static = Threshold_low;//����Խ��ֵԽ�󣬷�Χ��40-80��
    ImageParameter.Threshold_detach = detach;
   // ImageScanInterval = 2;               //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
   // ImageScanInterval_Cross = 7;         //ʮ��ɨ�߷�Χ
    // ��Сɨ�߷�Χ��ʮ�ּ�ⷶΧ
    ImageScanInterval = 1;         // ԭ2 �� 1 ���ٺ���������Χ
    ImageScanInterval_Cross = 4;   // ԭ7 �� 4 խ��ʮ������������
    
    SystemData.SpeedData.MaxSpeed = MaxSpeed;
    SystemData.SpeedData.MinSpeed = MinSpeed;
    ImageParameter.TowPoint = 21;           //ǰհ//24
    ImageParameter.variance_acc = ance_acc;       //ֱ�����
    
    
     ImageParameter.width_threshold = 20;  // ����ʵ����������

}


// ��ͼ������������Ӷ�̬�ع����
void dynamic_exposure_adjust() {
    // ͳ��Զ�������綥��30�У���ƽ������
    uint32_t far_avg = 0;
    for(int y=0; y<30; y++) {
        for(int x=0; x<MT9V03X_W; x++) {
            far_avg += mt9v03x_image[y][x];
        }
    }
    far_avg /= (30 * MT9V03X_W);

    // �������ȵ����ع⣨ʾ���߼���
    if(far_avg < 80) {       // Զ�˹���
        mt9v03x_set_exposure_time(MT9V03X_EXP_TIME_DEF + 5);
    } else if(far_avg > 200) { // Զ�˹���
        mt9v03x_set_exposure_time(MT9V03X_EXP_TIME_DEF - 5);
    }
}




typedef struct {
    uint16 y_start;
    uint16 y_end;
    float  weight; // Ȩ��ϵ��
} exposure_zone;

exposure_zone zones[] = {
    {0,  30, 0.7f},  // Զ�����򣬸�Ȩ��
    {90, 120, 0.3f}  // �������򣬵�Ȩ��
};

void weighted_exposure_adjust() {
    static uint16 current_exp = MT9V03X_EXP_TIME_DEF;
    
    // 1. �����Ȩƽ������
    float total_bright = 0;
    float total_weight = 0;
    
    for(int i=0; i<sizeof(zones)/sizeof(zones[0]); i++) {
        uint32 sum = 0;
        for(int y=zones[i].y_start; y<zones[i].y_end; y++) {
            for(int x=0; x<MT9V03X_W; x++) {
                sum += mt9v03x_image[y][x];
            }
        }
        float avg = sum / (float)((zones[i].y_end-zones[i].y_start)*MT9V03X_W);
        total_bright += avg * zones[i].weight;
        total_weight += zones[i].weight;
    }
    uint8 brightness = total_bright / total_weight;

    // 2. �����ع⣨Bang-Bang���ƣ�
    if(brightness < 90) {
        current_exp += 10; // ����ʱ�������
    } else if(brightness > 150) {
        current_exp -= 20; // ����ʱ���ټ���
    } else {
        current_exp += (brightness < 110) ? 5 : -5; // ΢��
    }
    
    mt9v03x_set_exposure_time(current_exp);
}


void enhance_far_region() {
    // 1. ͳ��Զ��ֱ��ͼ
    uint8 min_val = 255, max_val = 0;
    for(int y=FAR_ROI_START_Y; y<FAR_ROI_END_Y; y++) {
        for(int x=0; x<MT9V03X_W; x++) {
            uint8 pix = mt9v03x_image[y][x];
            if(pix < min_val) min_val = pix;
            if(pix > max_val) max_val = pix;
        }
    }
    
    // 2. ��������
    float scale = 255.0f / (max_val - min_val + 1);
    for(int y=FAR_ROI_START_Y; y<FAR_ROI_END_Y; y++) {
        for(int x=0; x<MT9V03X_W; x++) {
            mt9v03x_image[y][x] = (mt9v03x_image[y][x] - min_val) * scale;
        }
    }
}

void optimized_exposure_flow() {
    if(mt9v03x_finish_flag) {
        static uint32 frame_cnt = 0;
        
        // ÿ5֡����һ���ع⣨ƽ����Ӧ�ٶ����ȶ��ԣ�
        if(frame_cnt++ % 5 == 0) {
            weighted_exposure_adjust();
        }
        
        enhance_far_region(); // ʵʱ��ǿ��ʾ
    }
}















/*** �����ֹ�����ϰ������ߺ��� ***/
float CalculateOFFLineCenter(void) 
{
  int i;
 float sum=0.0;
  
for(i=55;i>47;i--)
{
sum+=RowAttribute[i].Center;
}
    
    // ������ֹ�е�ͼ��ײ������������Ϊ59��
   
    
    // ��ֹ�������
    return sum/8;
}




/**
  * @brief ͳ��ָ���еİ�ɫ��������
  * @param row Ҫ�����кţ�0-59��
  * @return ��ɫ�������������к���Ч����-1��
  */
int CountWhitePixelsInRow(uint8 row) 
{
    // ������Ч�Լ��
    if(row >= IPSH) {
        return -1; // �����룺�кų�����Χ
    }
    
    int white_count = 0;
    uint8* line_data = Binarization_Image[row]; // ��ȡĿ��������ָ��
    
    // �����Ż���ÿ�δ���4�����أ�����CPU�ܹ�������
    const int parallel_size = IPSW - (IPSW % 4);
    
    // ��ѭ������󲿷�����
    for(int x = 0; x < parallel_size; x += 4) {
        white_count += line_data[x] + line_data[x+1] 
                      + line_data[x+2] + line_data[x+3];
    }
    
    // ����ʣ������
    for(int x = parallel_size; x < IPSW; x++) {
        white_count += line_data[x];
    }
    
    return white_count;
}









/***��ͼ������***/
void ImageProcess(void){
  // ����ƽ���������
    int avg_width = 0;
    for(int y=50; y<60; y++){
        avg_width += RowAttribute[y].Wide;
    }
    avg_width /= 10;
    
    // ��̬����ɨ�߲���
    ImageScanInterval = (avg_width < 15) ? 1 : 2; // խ����ʹ�ø�С������Χ
    
    Cramping();//ͼ��ѹ��
    ImageParameter.OFFLine = 2;//ȡͼ�񶥱ߣ�̫��ǰ��ͼ�񲻿���
    ImageParameter.WhiteLine = 0;
    for(Ysite=59;Ysite>=ImageParameter.OFFLine;Ysite--){
        RowAttribute[Ysite].IsLeftFind = 'F';
        RowAttribute[Ysite].IsRightFind = 'F';
        RowAttribute[Ysite].LeftBorder = 0;
        RowAttribute[Ysite].RightBorder = 79;
    }//�߽��ʼ��
    Threshold_Speraration_Ostu();//ͼ���ֵ����ֵ��򷨻�ȡ�����ֵ��
    Bin_Image_Filter();//ȥ�����
    DrawBasicLines();//ɨ��ǰ����
    DrawProcessLines();//ɨ��54��ǰ�ĸ��б߽�
    Emergency_Breaking();
    Eightfields_Search_Border(Binarization_Image, IPSH, IPSW, IPSH - 2);//������ɨ������Ԫ�ش���
    /***Ԫ��ʶ��***/
  //  Element_Test();//ֱ����Բ���������߼��
    /***Ԫ�ش���*****/
   // Element_Handle();//Բ����ʮ�ִ��������˲����ƶ�
    Prospective_error();//ǰհ�������
}

