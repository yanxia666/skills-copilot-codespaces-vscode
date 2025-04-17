/*
 * camera.c
 *
 *  Created on: 2024年9月15日
 *      Author: HONOR
 */
#include "zf_common_headfile.h"

uint8 Compressed_image[IPSH][IPSW];//用于存储压缩后的图像
uint8 Binarization_Image[IPSH][IPSW];//用于储存二值化后的图像
ImageParametertypedef ImageParameter;//图像各个参数结构体定义
RowAttributetypedef RowAttribute[60]; //用于记录单行信息
SystemDatatypdef SystemData;

static int Ysite=0,Xsite=0;//行坐标跟纵坐标，前行后列
static int BottomBorderRight = 79,BottomBorderLeft=0,BottomCenter=0;//第59行左右边界

static uint8* StoreSingleLine;                 //保存单行图像
uint8 ExtenLFlag = 0;                          //是否左延长标志
uint8 ExtenRFlag = 0;                          //是否右延长标志

int ImageScanInterval_Cross;                   //270°的弯道后十字的扫线范围
static int IntervalLow = 0, IntervalHigh = 0;  //定义高低扫描区间
int ImageScanInterval;                         //扫边范围    上一行的边界+-ImageScanInterval

int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //左圆环判断的两点纵坐标
int Point_Xsite,Point_Ysite;                   //拐点横纵坐标
int Repair_Point_Xsite,Repair_Point_Ysite;     //补线点横纵坐标
uint8 Ring_Help_Flag = 0; //进环辅助标志
uint8 Half_Road_Wide[60] =                      //直到赛道半宽
{ 
  1,1,1,2,2,2,3,3,3,4,  // 前10行宽度减半
  4,4,5,5,5,6,6,6,7,7,
  //2,2,2,2,3,3,4,5,5,5,
 // 6,6,6,7,7,7,8,8,8,9,
  9,10,10,10,11,11,12,12,13,13,
  13,14,14,14,15,15,15,16,16,17,
  17,17,18,18,19,19,20,20,20,21,
  21,22,22,22,23,23,23,24,24,24,
};



//float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};
                        //10行权重参数，随意更改，基本不影响，大致按照正态分布即可
float Weighting[10] = {0.67, 0.69, 0.88, 0.96, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};


int flag_cross=0;
static int TFSite = 0, left_FTSite = 0,right_FTSite = 0;              //补线计算斜率的时候需要用的存放行的变量。
static float DetR = 0, DetL = 0;                //存放补线斜率的变量
static int ytemp = 0;                           //存放行的临时变量

int flag_test_stop=1;
int flag_test=0;

float Original_imageH=MT9V03X_H;//原始图像高
float Original_imageW=MT9V03X_W;//原始图像宽
float Compression_imageH=IPSH;//压缩后图像高
float Compression_imageW=IPSW;//压缩后图像宽

/***图像压缩函数***/
void Cramping(void){
    int m,n,column,line;
    const float Proportion_h=Original_imageH/Compression_imageH;//计算压缩比例
    const float Proportion_w=Original_imageW/Compression_imageW;//计算压缩比例
    for(m=0;m<IPSH;m++){
        column=m*Proportion_h+0.5;//加0.5可以做到四舍五入
    for(n=0;n<IPSW;n++){
         line=n*Proportion_w+0.5;
         Compressed_image[m][n] = mt9v03x_image[column][line];
        }

    }
}

/***优化后的大津法***/
uint8 Threshold_deal(uint8* image, uint16 col, uint16 row, uint32 pixel_threshold)//最后一个参数为阳光算法分割获得的阈值，我的理解为分别背景与主体的阈值
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
    }//数据初始化
    uint32 gray_sum = 0;
    for (i = 0; i < height; i += 1) {
      for (j = 0; j < width; j += 1) {
        pixelCount[(int)data[i * width + j]]++;//图像中属于各个像素值的像素点的统计
        gray_sum += (int)data[i * width + j];//图像各个像素点像素值的和
      }
    }
    for (i = 0; i < GrayScale; i++) {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;//得出各个像素值的像素点占图像的比例
    }

    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_threshold; j++) {//小于阳光算法分割的阈值的为主体
      w0 +=
          pixelPro[j];//主体像素点占图像的比例
      u0tmp += j * pixelPro[j];//主体像素值占总图像像素的平均值
      w1 = 1 - w0;//背景像素点占图像的比例
      u1tmp = gray_sum / pixelSum - u0tmp;//背景像素值占总图像像素的平均值
      u0 = u0tmp / w0;//主体像素的平均值
      u1 = u1tmp / w1;//背景像素的平均值
      u = u0tmp + u1tmp;//图像像素值的平均值
      deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//计算方差
      if (deltaTmp > deltaMax) {
        deltaMax = deltaTmp;
        threshold = (uint8)j;
      }//当方差值达到最大时，此时像素值即为最佳阈值
      if (deltaTmp < deltaMax) {
        break;
      }
    }
    return threshold;
}

/***二值化阈值获取及二值化函数***/
void Threshold_Speraration_Ostu(void){
  //  ImageParameter.Threshold =
      //      (uint8)Threshold_deal(Compressed_image[0],IPSW,IPSH,ImageParameter.Threshold_detach);
  ImageParameter.Threshold = Threshold_deal(Compressed_image[0], IPSW, IPSH, ImageParameter.Threshold_detach);
     //ImageParameter.Threshold =160;
          
    
    if(ImageParameter.Threshold < ImageParameter.Threshold_static)
        ImageParameter.Threshold  =  (uint8)ImageParameter.Threshold_static;//确保阈值不低于阈值静态下限
    uint8 m,n;
    uint8 ther;
    
    
     for(m = 0;m < IPSH;m++){
        for(n = 0;n < IPSW;n++){
            // 减少边缘区域的阈值衰减
            if(n <= 20 || n >= 60) // 原15/70边界 → 扩大敏感区域
                ther = ImageParameter.Threshold - 5; 
            else
                ther = ImageParameter.Threshold;
                
            Binarization_Image[m][n] = (Compressed_image[m][n] > ther) ? 1 : 0;
    
   /* for(m = 0;m < IPSH;m++){
        for(n = 0;n < IPSW;n++){
            if(n <= 15)
                ther = ImageParameter.Threshold - 10;//图像越靠边越越可能为背景，阈值可以适当降低
            else if ((n > 70 && n <= 75))
                ther = ImageParameter.Threshold - 10;
            else if (n >= 65)
                ther = ImageParameter.Threshold - 10;
            else
                ther = ImageParameter.Threshold;
  //二值化
            if(Compressed_image[m][n] > (ther))
                Binarization_Image[m][n] = 1;//白
            else
                Binarization_Image[m][n] = 0;//黑
            }
        }*/
}
     }
}
/***去除噪点函数***/
void Bin_Image_Filter(void) 
{
    uint16 nr; // 行索引
    uint16 nc; // 列索引

    for (nr = 1; nr < IPSH - 1; nr++) {
        for (nc = 1; nc < IPSW - 1; nc++) {
            // 计算8邻域白点数量（包含4个对角）
            uint8 neighbors = 
                Binarization_Image[nr-1][nc-1] +  // 左上
                Binarization_Image[nr-1][nc]   +  // 上 
                Binarization_Image[nr-1][nc+1] +  // 右上
                Binarization_Image[nr][nc-1]   +  // 左
                Binarization_Image[nr][nc+1]   +  // 右
                Binarization_Image[nr+1][nc-1] +  // 左下
                Binarization_Image[nr+1][nc]   +  // 下
                Binarization_Image[nr+1][nc+1];   // 右下

            /* 白点噪声过滤（孤立白点转黑） */
            if (Binarization_Image[nr][nc] == 1) 
            {
                // 条件1：周围白点少于3个视为噪声
                // 条件2：避免误伤斜线（至少1个直接相邻白点）
                if (neighbors < 3 && 
                   (Binarization_Image[nr-1][nc] + 
                    Binarization_Image[nr+1][nc] + 
                    Binarization_Image[nr][nc-1] + 
                    Binarization_Image[nr][nc+1]) < 1) 
                {
                    Binarization_Image[nr][nc] = 0;
                }
            }
            /* 黑点噪声过滤（孤立黑点转白） */ 
            else 
            {
                // 条件1：周围白点多于5个
                // 条件2：避免误补断点（四邻域至少有2个白点）
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
    uint16 nr; //行
    uint16 nc; //列

    for (nr = 1; nr < IPSH - 1; nr++)
    {
        for (nc = 1; nc < IPSW - 1; nc = nc + 1)
        {
            if ((Binarization_Image[nr][nc] == 0)
                    && (Binarization_Image[nr - 1][nc] + Binarization_Image[nr + 1][nc]
                      + Binarization_Image[nr][nc + 1] + Binarization_Image[nr][nc - 1] > 2))//如果白点中间存在黑点，则视为噪点
            {
                Binarization_Image[nr][nc] = 1;
            }
            else if ((Binarization_Image[nr][nc] == 1)
                    && (Binarization_Image[nr - 1][nc] + Binarization_Image[nr + 1][nc]
                      + Binarization_Image[nr][nc + 1] + Binarization_Image[nr][nc - 1] < 2))//如果黑点中间存在白点，视为噪点
            {
                Binarization_Image[nr][nc] = 0;
            }
        }
    }
}
*/
/***扫基本线函数***/
static uint8 DrawBasicLines(void){
    StoreSingleLine = Binarization_Image[59];//单独扫描59行
    //第59行中点为黑
    if(*(StoreSingleLine + PictureCentring) == 0) /*  图像底边中点为黑 异常 */
    {
        for (Xsite = 0; Xsite < PictureCentring; Xsite++)
        {
            if (*(StoreSingleLine + PictureCentring - Xsite) != 0)//从中间往左边扫
                    break;
            if (*(StoreSingleLine + PictureCentring + Xsite) != 0)//从中间往右边扫
                    break;
        }
        if (*(StoreSingleLine + PictureCentring - Xsite) != 0)//赛道在车左边方位
            {
              BottomBorderRight = PictureCentring - Xsite + 1;//将之定为59行右边线
              for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)//从右边界往左扫左边线
              {
                if (*(StoreSingleLine + Xsite) == 0 &&
                        *(StoreSingleLine + Xsite - 1) == 0)//连续两个黑点左边线即找到//?不应该为跳变点吗?//赛道中间全为白色，一路扫过去扫到两个黑点即可认为是边界
                {
                  BottomBorderLeft = Xsite;//记录左边线
                  break;
                }
                else if (Xsite == 1)//没扫到跳变点，即认为左边线是当前图像最左边
                {
                  BottomBorderLeft = 0;
                  break;
                }
              }
            }
        else if (*(StoreSingleLine + PictureCentring + Xsite) != 0)//赛道在车右边方位
        {
          BottomBorderLeft = PictureCentring + Xsite - 1;//将之定为左边界
          for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)//从左边线开始向右扫右边界
          {
            if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite + 1) == 0)//如果连续扫到两个黑点//?
            {
              BottomBorderRight = Xsite;//记为右边界
              break;
            }
            else if (Xsite == 78)//没扫到跳变点
            {
              BottomBorderRight = 79;
              break;
            }
          }
        }
    }//第59行中点为黑
/////////////////
    //第59行中点为白
    else        /*  图像底边中点为白 正常 */
    {

        for (Xsite = PictureCentring; Xsite <78; Xsite++)//从中点开始向右搜索右边线
        {
          if (  *(StoreSingleLine + Xsite) == 0 &&
                  *(StoreSingleLine + Xsite + 1) == 0)//连续两个黑点
          {
            BottomBorderRight = Xsite;//记录
            break;
          }
          else if (Xsite == 78)//扫到最后都没扫到
          {
            BottomBorderRight = 78;
            break;
          }
        }

        for (Xsite = PictureCentring; Xsite >0; Xsite--)//从中点向左搜索左边线
        {
          if (  *(StoreSingleLine + Xsite) == 0 &&
                  *(StoreSingleLine + Xsite - 1) == 0)//连续两个黑点
          {
            BottomBorderLeft = Xsite;
            break;
          }
          else if (Xsite == 1)//到最后一列没扫到，则认为最左边为左边界
          {
            BottomBorderLeft = 1;
            break;
          }
        }
    }
        BottomCenter = (BottomBorderLeft + BottomBorderRight)/2;//59行中点
        RowAttribute[59].LeftBorder = BottomBorderLeft;
        RowAttribute[59].RightBorder = BottomBorderRight;
        RowAttribute[59].Center = BottomCenter;
        RowAttribute[59].Wide = BottomBorderRight - BottomBorderLeft;//计算59行宽度
        RowAttribute[59].IsLeftFind = 'T';
        RowAttribute[59].IsRightFind = 'T';//59行左右边界正常跳变
        //利用扫线获得的值计算第59行各个参数

        /*扫其余四行基本行*/

               for (Ysite = 58; Ysite > 54; Ysite--)
               {
                 StoreSingleLine = Binarization_Image[Ysite];
                 for (Xsite =  RowAttribute[Ysite + 1].Center; Xsite < 78; Xsite++)//从上一行的中点位置开始扫右边线
                 {
                   if (*(StoreSingleLine + Xsite) == 0 && *(StoreSingleLine + Xsite + 1) == 0)//如果连续扫到两个黑点
                   {
                       RowAttribute[Ysite].RightBorder = Xsite;
                       break;
                   }
                   else if (Xsite == 78)//扫到最右边也没扫到
                   {
                       RowAttribute[Ysite].RightBorder = 78;
                       break;
                   }
                 }
                 for (Xsite = RowAttribute[Ysite + 1].Center; Xsite >0; Xsite--)//从上一行中点扫左边线
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
                 //计算四行参数
               }
             return 'T'; //最基本要求、车前五行不可受到干扰(丢线)、安装时注意调整摄像头视角
  }//基本扫线截止


//********************************************单行的传统扫线**************************************************//
//p:对应行的图像数组 type:L左R右  L和H:扫线的左右范围  Q:记录跳变点及跳变的类型
void JumpPointAndType(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q){/*寻找跳变点并确定行类型*/
  int i = 0;                                    /*第一个参数是要查找的数组（80个点）
                                                  第二个扫左边线还是扫右边线
                                                  第三和第四是开始和结束点*/
  if (type == 'L')                              //扫描左边线
  {
    for (i = H; i >= L; i--)                    //内到外扫
    {
      if (*(p + i) == 1 && *(p + i - 1) != 1) //跳变点（白变黑）
     // if ((*(p + i) ^ *(p + i ± 1)))
      {
        Q->point = i;                           //记录左边线
        Q->type = 'T';                          //正确跳变
        break;
      }
      else if (i == (L + 1))                    //扫到最后都没扫到
      {
        if (*(p + (L + H) / 2) != 0)            //没扫到且中间为白点
        {
          Q->point = (L + H) / 2;               //认为左边线是当前扫描范围的中点
          Q->type = 'W';                        //非正确跳变且中间为白、无边行（十字
          break;
        }
        else                                    //非正确跳变且中间为黑
        {
          Q->point = H;                         //左边线认为扫线范围最内边
          Q->type = 'H';                        //认为是大跳变
          break;
        }
      }
    }
  }
  else if (type == 'R')                         //扫描右边线、同上
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




/***边线追逐获取各行边界***/
static void DrawProcessLines(void){
  
        
   float D_L = 0;           //延长线左边线斜率
   float D_R = 0;           //延长线右边线斜率
  
    uint8 L_Found_T = 'F';   //确定无边斜率的基准有边行是否被找到的标志:F为未找到，T为找到了
    uint8 Get_L_line = 'F';  //找到这一帧图像的基准左斜率
    uint8 R_Found_T = 'F';   //确定无边斜率的基准有边行是否被找到的标志
    uint8 Get_R_line = 'F';  //找到这一帧图像的基准右斜率
    
    int ytemp_W_L;           //记住首次左丢边行
    int ytemp_W_R;           //记住首次右丢边行
    ExtenRFlag = 0;          //标志位清零
    ExtenLFlag = 0;
    for (Ysite = 54 ; Ysite > ImageParameter.OFFLine; Ysite--){
           //太前的图像可靠性不高、所以OFFLine的设置很有必要

           ImageScanInterval_Cross = (54 - Ysite)/2 + 5;/*4.24用于解决十字bug*/

           StoreSingleLine = Binarization_Image[Ysite];
           JumpPointtypedef JumpPoint[2];     // 0左1右

           /******************************扫描本行的右边线******************************/
           /*确定扫边范围*/
           /*if (ImageParameter.Road_type != Cross_ture)*/
           if(ImageParameter.WhiteLine < 5){//当双边丢边数少于五行
               IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval;//上一行右边线减去Interval作为最左开始扫描的列
               //从上一行右边线-Interval的点开始（确定扫描开始点）
               IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval;//上一行右边线加Interval作为最右边扫描结束的列
               //到上一行右边线+Interval的点结束（确定扫描结束点）
           }
           else{//当双边丢边数少于五行，视为十字
               IntervalLow = RowAttribute[Ysite + 1].RightBorder - ImageScanInterval_Cross;//减去十字的Interval
               //从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
               IntervalHigh = RowAttribute[Ysite + 1].RightBorder + ImageScanInterval_Cross;//加上十字的Interval
               //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）
           }
           LimitL(IntervalLow);   //确定左扫描区间并进行限制
           LimitH(IntervalHigh);  //确定右扫描区间并进行限制
           JumpPointAndType(StoreSingleLine,'R', IntervalLow, IntervalHigh,&JumpPoint[1]);//扫右边线确认右边线的跳变点及其类型
           /******************************扫描本行的右边线******************************/

           /******************************扫描本行的左边线******************************/
           /*确定扫边范围*/
           /*if (ImageParameter.Road_type != Cross_ture)*/
           if(ImageParameter.WhiteLine < 5){
               IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval;
               //从上一行右边线-Interval的点开始（确定扫描开始点）
               IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval;
               //到上一行右边线+Interval的点结束（确定扫描结束点）
           }
           else{
               IntervalLow = RowAttribute[Ysite + 1].LeftBorder - ImageScanInterval_Cross;
               //从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
               IntervalHigh = RowAttribute[Ysite + 1].LeftBorder + ImageScanInterval_Cross;
               //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）
           }
           LimitL(IntervalLow);   //确定左扫描区间并进行限制
           LimitH(IntervalHigh);  //确定右扫描区间并进行限制
           JumpPointAndType(StoreSingleLine, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);//扫左边线确认左边线的跳变点及其类型
           /******************************扫描本行的左边线******************************/

           if (JumpPoint[0].type =='W')  //左边线无边行（全白
           {
               RowAttribute[Ysite].LeftBorder = RowAttribute[Ysite + 1].LeftBorder;
               //那么本行的左边线就采用上一行的边线。
           }
           else
           {   //如果本行的左边线属于T或者是H类别
               RowAttribute[Ysite].LeftBorder = JumpPoint[0].point;//则记录
           }

           if (JumpPoint[1].type == 'W')   //右边线无边行（全白
           {
               RowAttribute[Ysite].RightBorder = RowAttribute[Ysite + 1].RightBorder;
               //那么本行的右边线就采用上一行的边线。
           }
           else
           {   //如果本行的右边线属于T或者是H类别
               RowAttribute[Ysite].RightBorder = JumpPoint[1].point;
           }
           /*记录本行边线类型*/
           RowAttribute[Ysite].IsLeftFind = JumpPoint[0].type;
           RowAttribute[Ysite].IsRightFind = JumpPoint[1].type;

           
           
           /*********************** 新增稳定性校验 Start ***********************/
        // 检查与下一行的边界突变（防止单行跳变干扰）
        if(Ysite < 59) { // 避免数组越界
            // 左边界突变检查
            if(abs(RowAttribute[Ysite].LeftBorder - RowAttribute[Ysite+1].LeftBorder) > 5) {
                RowAttribute[Ysite].IsLeftFind = 'W'; // 标记为异常跳变
            }
            // 右边界突变检查
            if(abs(RowAttribute[Ysite].RightBorder - RowAttribute[Ysite+1].RightBorder) > 5) {
                RowAttribute[Ysite].IsRightFind = 'W'; // 标记为异常跳变
            }
        }
        /*********************** 新增稳定性校验 End ***********************/
           
           
           
   /************************************重新确定大跳变(即H类)的边界*************************************/
           if (( RowAttribute[Ysite].IsLeftFind == 'H' || RowAttribute[Ysite].IsRightFind == 'H'))
           {
               /**************************处理左边线的大跳变***************************/
               if (RowAttribute[Ysite].IsLeftFind == 'H')
               {
                 for (Xsite = (RowAttribute[Ysite].LeftBorder + 1);//大跳变行的左边界前面记录为扫线范围的最右列
                         Xsite <= (RowAttribute[Ysite].RightBorder - 1);Xsite++)//大跳变行扫描右边界时则认为是扫线范围的最左列
                 {//从左向右扫线
                   if ((*(StoreSingleLine + Xsite) == 0) && (*(StoreSingleLine + Xsite + 1) != 0))
                   {
                      //这里可以改为扫到第一个白点则此点即为黑白跳变点
                       //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
                       RowAttribute[Ysite].LeftBorder = Xsite;
                       RowAttribute[Ysite].IsLeftFind = 'T';
                       break;
                   }
                   else if (*(StoreSingleLine + Xsite) != 0)//由于大跳变左边界外全为黑点，所以一旦出现白点则直接跳出
                     break;//直接跳出？不记录左边界？这一句似乎只对于左边前第一个点有意义
                   else if (Xsite ==(RowAttribute[Ysite].RightBorder - 1))
                   {
                       RowAttribute[Ysite].LeftBorder = Xsite;//一直到右边界都未出现白点，将此时右边界记为左边界
                       RowAttribute[Ysite].IsLeftFind = 'T';
                       break;
                   }
                 }
               }
               /**************************处理左边线的大跳变***************************/

               /**************************处理右边线的大跳变***************************/
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
                   /**************************处理右边线的大跳变***************************/

           /*****************************重新确定大跳变的边界******************************/


   /************************************对无边行进行处理****************************************************************/
           int ysite = 0;
           uint8 L_found_point = 0;
           uint8 R_found_point = 0;

               /**************************处理右边线的无边行***************************/
           if (RowAttribute[Ysite].IsRightFind == 'W' && Ysite > 10 && Ysite < 50)//十到五十行之间
           {
             if (Get_R_line == 'F')
             {
               Get_R_line = 'T'; //一帧图像只跑一次
               ytemp_W_R = Ysite + 2;//往后退两行
               for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
               {
                 if (RowAttribute[ysite].IsRightFind =='T')
                 {
                     R_found_point++;
                 }
               }//往前寻找有正常跳变边界的行
               if (R_found_point >8)//当正常边界行超过八行则可用于补线
               {       //求斜率给无边行补线做基准
                 D_R = ((float)(RowAttribute[Ysite + R_found_point].RightBorder -
                         RowAttribute[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));//图像为八字形，计算结果一定为正
                 if (D_R > 0)
                 {
                   R_Found_T ='T';
                 }
                 else
                 {
                   R_Found_T = 'F';
                   if (D_R < 0)
                   {
                       ExtenRFlag = 'F';   //此标志位用于十字角点补线  防止图像误补
                   }
                 }
               }
             }
             if (R_Found_T == 'T')
             {
                 RowAttribute[Ysite].RightBorder =
                         RowAttribute[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
                 //如果找到了 那么以基准行做延长线
             }
             LimitL(RowAttribute[Ysite].RightBorder);  //限幅
             LimitH(RowAttribute[Ysite].RightBorder);  //限幅
           }
               /**************************处理右边线的无边行***************************/

           /**************************处理左边线的无边行***************************/
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
                   if (L_found_point > 8)              //找到基准斜率边  做延长线重新确定无边
                   {
                     D_L = ((float)(RowAttribute[Ysite + 3].LeftBorder -
                             RowAttribute[Ysite + L_found_point].LeftBorder)) /
                                     ((float)(L_found_point - 3));//由于图像赛道成八字形，这里计算方式左边界斜率一定为正
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

                 LimitL(RowAttribute[Ysite].LeftBorder);  //限幅
                 LimitH(RowAttribute[Ysite].LeftBorder);  //限幅
               }

               /**************************处理左边线的无边行***************************/

   /************************************对无边行进行处理****************************************************************/

   /************************************其余数据整定操作*************************************************/
           if (RowAttribute[Ysite].IsLeftFind == 'W' && RowAttribute[Ysite].IsRightFind == 'W')
                 {
                       ImageParameter.WhiteLine++;  //左右都无边，丢边数累加
                 }
           if (RowAttribute[Ysite].IsLeftFind == 'W'&&Ysite<55)
                {
                       ImageParameter.Miss_Left_lines++;
                }
           if (RowAttribute[Ysite].IsRightFind == 'W'&&Ysite<55)
                {
                       ImageParameter.Miss_Right_lines++;
                }
                       /*限幅操作*/
               LimitL(RowAttribute[Ysite].LeftBorder);
               LimitH(RowAttribute[Ysite].LeftBorder);
               LimitL(RowAttribute[Ysite].RightBorder);
               LimitH(RowAttribute[Ysite].RightBorder);

               RowAttribute[Ysite].Wide =
                       RowAttribute[Ysite].RightBorder - RowAttribute[Ysite].LeftBorder;
               RowAttribute[Ysite].Center =
                       (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;

           if (RowAttribute[Ysite].Wide <= 6) //重新确定可视距离
             {
                   ImageParameter.OFFLine = Ysite + 1;
                   break;
             }
             else if (RowAttribute[Ysite].RightBorder <= 10 || RowAttribute[Ysite].LeftBorder >= 70)
             {         //当图像宽度小于10或者左右边达到一定的限制时，则终止巡边
                   ImageParameter.OFFLine = Ysite + 1;
                   break;
             }

   /************************************其余数据整定操作*************************************************/
 
       }
       return;
    
    
    
    
    
   

    // wireless_printf("%d,%d,%d\n",D_R );
}

/***获取底线左右边界***/
void Search_Bottom_Line(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //寻找左边边界
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            RowAttribute[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < IPSW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            RowAttribute[Bottonline].RightBoundary = Xsite;//获取底边右边线
            break;
        }
    }
}


void Search_Left_and_Right_Lines(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
/*  前进方向定义：
                *   0
                * 3   1
                *   2
*/
/*寻左线坐标规则*/
    uint8 Left_Rule[2][8] = {//八个方向横纵坐标需要十六个数表示
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )//正前方，正右方，正后方，正左方
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}//0.左前方，1.右前方，2.右下方，3.左下方
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},   (x,y)//正前方，正右方，正后方，正左方
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}//0.左前方，1.右前方，2.右下方，3.左下方
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;//左边纵坐标从最底行开始
    uint8 Left_Xsite = (uint8)RowAttribute[Bottonline].LeftBoundary;//横坐标从最底行左边界开始
    uint8 Left_Rirection = 0;//左边方向
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = (uint8)RowAttribute[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//右边方向
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageParameter.OFFLineBoundary = 5;
    while (1)
    {//循环扫描爬线
            num++;
            if(num>400)
            {
                ImageParameter.OFFLineBoundary = Ysite;
                break;
            }//当循环超过四百次时退出
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)//当总纵坐标大于等于左右边界坐标时向前推
        {//确保不会出现正前方及正右方同为黑色的情况，如果出现则卡在这一行不动直至退出，这里也限制了不能寻点寻到前一行的位置，方位不可出现左右后方及正后方
            if (Ysite < ImageParameter.OFFLineBoundary)//超过截止行退出
            {
                ImageParameter.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }//总纵坐标向前推
        }
        /*********左边巡线*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary)//左边扫线
        {//
            /*计算前方坐标*///从前方开始
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];//坐标计算为一个循环的过程
            /**正前方存在为黑和白的两种情况，由于前面限制不会出现正前方和正右方同时为黑色的情况，这里的整个扫线限制为四种情况，
             1.正前方为白，左前方为黑，则可认为正前方为左边界。
             2.正前方为白，左前方也为白，则爬至左前方的点，并且从正左方开始寻边界点
             3.正前方为黑，正右方为白，右前方为白，则可以认为右前方为左边界，并且由于此时方位会回正到从正前方开始。
             4.正前方为黑，正右方为白，右前方为黑，这里直接将右前方作为了左边界并且爬到右前方寻边界点，
                 在右前方的点时由于前面右前方为黑，从正右方开始寻点，由于前面限制，此时也不能出现正右方为黑色的情况，
                 因此上一行可以将上一行的右前方作为左边界点，且由于此时寻点方位仍为正右方开始，下一行的正右方同样也不能出现为黑色的情况。
             **/
            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//正前方如果是黑色，转正右方，正右方如果为黑色，转正下方，……转正左方
            {
                //顺时针旋转90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//前方是白色
            {
                /*计算坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];//左边界纵坐标爬到下一行
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];//左边界横坐标坐标爬到下一行
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0)
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;//记录左边界
                }
                else//为白色
                {
                    // 方向发生改变 Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];//前方及左前方均为白色，爬向左前方的点继续寻边界点
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (RowAttribute[Left_Ysite].LeftBoundary_First == 0 )
                        RowAttribute[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    RowAttribute[Left_Ysite].LeftBoundary = Left_Xsite;//？重复？
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;//如果前方及左前方均为白点，则爬至左前方之后从正左方开始寻点
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********右边巡线*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageParameter.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//是黑色
            {
                //逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//是白色
            {
                /*计算坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (RowAttribute[Right_Ysite].RightBoundary_First == 79 )
                        RowAttribute[Right_Ysite].RightBoundary_First = Right_Xsite;
                    RowAttribute[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
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

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
        {
            ImageParameter.OFFLineBoundary = Ysite;
            break;
        }
    }
}


/***八领域扫线***/
/***图像 行列  最底行***/
void Eightfields_Search_Border(uint8 imageInput[IPSH][IPSW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageParameter.WhiteLine_L = 0;
    ImageParameter.WhiteLine_R = 0;
    //ImageParameter.OFFLine = 1;
    /*封上下边界处理*/
    for (int Xsite = 0; Xsite < IPSW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for (int Ysite = 0; Ysite < IPSH; Ysite++)
    {
            RowAttribute[Ysite].LeftBoundary_First = 0;
            RowAttribute[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][IPSW - 1] = 0;
    }
    /********获取底部边线*********/
    Search_Bottom_Line(imageInput, Row, Col, Bottonline);
    /********获取左右边线*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);//八领域算法扫左右边线

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

/***直道加速检测***/
float variance_acc;  //方差
void Straightacc_Test(void){
    int sum = 0;
    for (Ysite = 55; Ysite > ImageParameter.OFFLine + 1; Ysite--) {
      sum += (RowAttribute[Ysite].Center - PictureCentring) *(RowAttribute[Ysite].Center - PictureCentring);//计算实际中线与绝对中线的方差
    }
    variance_acc = (float)sum / (54 - ImageParameter.OFFLine);

    if ( variance_acc < ImageParameter.variance_acc && ImageParameter.OFFLine <= 10)
    {
        ImageParameter.straight_acc = 1;
    } else
        ImageParameter.straight_acc = 0;
}


/*****************直线判断******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //返回结果小于1即为直线
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)//1左2右
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
                RowAttribute[end].RightBorder) / (start - end);//计算斜率
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


/***左圆环检测***/
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
        )//在非直道或右边线丢线大于3或左边线丢线小于5或截止行大于10或双边丢下数大于4或最前方丢线时，直接退出。
        return;//退出本函数，防止误判即元素不是圆环。
    //beepopen;
    int ring_ysite = 25;//检测范围
    uint8 Left_Less_Num = 0;//用于统计丢线行数
    Left_RingsFlag_Point1_Ysite = 0;//用于记录圆环的第一个点
    Left_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite].LeftBoundary_First - RowAttribute[Ysite - 1].LeftBoundary_First > 4)//当检测到本行左边界大于下一行
            {
                Left_RingsFlag_Point1_Ysite = Ysite;//记为检测到圆环的第一个点的纵坐标
              //  ips200_show_int(x1[0],y[10],Left_RingsFlag_Point1_Ysite,2);
                break;
            }
        }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
        {
            if (RowAttribute[Ysite + 1].LeftBoundary - RowAttribute[Ysite].LeftBoundary > 4)//当检测到上一行宽于本行
            {
                Left_RingsFlag_Point2_Ysite = Ysite;//同样检测到圆环的第一个点的纵坐标
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
    }//统计丢线行数
    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageParameter.OFFLine; Ysite--)
    {
        if (   RowAttribute[Ysite + 6].LeftBorder < RowAttribute[Ysite+3].LeftBorder
            && RowAttribute[Ysite + 5].LeftBorder < RowAttribute[Ysite+3].LeftBorder
            && RowAttribute[Ysite + 3].LeftBorder > RowAttribute[Ysite + 2].LeftBorder
            && RowAttribute[Ysite + 3].LeftBorder > RowAttribute[Ysite + 1].LeftBorder
            )//从检测圆环第一个点的那行开始，向近处由于不丢边，往前推三行的边界要大于往前推五行跟六行，但往后推由于后面左边丢线因此后左边界应小于本行
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

/***右圆环识别***/
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

/*****************坡道判断******************/
void Element_Judgment_Ramp(void)
{
    if (ImageParameter.Ramp_Flag) return;

}

/***斑马线检测***/
void Zebra_Judgment(void){
    if (ImageParameter.Zebra_Flag) return;//直接退出函数

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
            }//从左边界到右边界检测跳变点
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

    SpeedGain = //   暂时不考虑使用 5.14
            (SystemData.SpeedData.nowspeed -
                    SystemData.SpeedData.MinSpeed) * 0.2 +0.5 ;
    if (SpeedGain >= 3)//前瞻因子限幅
         SpeedGain = 3;
    else if (SpeedGain <= -1)
     SpeedGain = -1;
}


/***元素识别函数***/
void Element_Test(void){
    Straightacc_Test();//直道加速检测
//    if(circle_count_flag < 4 && ImageParameter.image_element_rings_flag == 0
//            && ImageParameter.WhiteLine < 1 )//双边丢边数为0及圆环进程无时检测圆环
//    {
       // Left_Rings_Judgment();      //左圆环检测
       // Right_Rings_Judgment();     //右圆环检测
//    }
    /*if(flag_test_stop==1)
    {
      Zebra_Judgment();//斑马线检测
    }*/
}

/***左圆环处理***/
void Left_Rings_Handle(void){
    /***************************************圆环进程的各个阶段的判断**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {

        if(RowAttribute[Ysite].IsLeftFind == 'W')
            num++;//统计左边丢线数
        if(    RowAttribute[Ysite+3].IsLeftFind == 'W' && RowAttribute[Ysite+2].IsLeftFind == 'W'
            && RowAttribute[Ysite+1].IsLeftFind == 'W' && RowAttribute[Ysite].IsLeftFind == 'T')//当检测到圆环上面的那一个拐点时开始准备入环
            break;
    }
  //  ips200_show_int(x1[5],y[9],num,2);
    //准备进环
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

    //进环
    if(ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_R>15)//半宽处理后向圆环中偏，右侧丢线
    {
        beepopen;
        ImageParameter.image_element_rings_flag = 6;
    }
        //进环小圆环
    if(ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_R<10)//半宽处理后向圆环中偏，右侧丢线
    {
        beepopen;
        //Stop = 1;
        ImageParameter.image_element_rings_flag = 7;
    }
        //环内 大圆环判断
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
        //环内 小圆环判断
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
        //出环后
    if (ImageParameter.image_element_rings_flag == 8)
    {
        if (    Straight_Judge(2, ImageParameter.OFFLine+15, 50) < 1
             && ImageParameter.WhiteLine_R < 10
             && ImageParameter.OFFLine < 7)    //右边为直线且截止行（前瞻值）很小

            ImageParameter.image_element_rings_flag = 9;
        beepopen;
    }
        //结束圆环进程

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
    /***************************************处理**************************************/
           //准备进环  半宽处理
       if (   ImageParameter.image_element_rings_flag == 1
           || ImageParameter.image_element_rings_flag == 2
           || ImageParameter.image_element_rings_flag == 3
           || ImageParameter.image_element_rings_flag == 4)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
           }//半宽处理的作用应该为让车偏向左边，方便入环
       }
           //进环  补线
       if  ( ImageParameter.image_element_rings_flag == 5
           ||ImageParameter.image_element_rings_flag == 6
           )
       {
           int  flag_Xsite_1=0;
           int flag_Ysite_1=0;
           float Slope_Rings=0;
           for(Ysite=55;Ysite>ImageParameter.OFFLine;Ysite--)//下面弧点
           {
               for(Xsite=RowAttribute[Ysite].LeftBorder + 1;Xsite<RowAttribute[Ysite].RightBorder - 1;Xsite++)
               {
                   if(  Binarization_Image[Ysite][Xsite] == 1 && Binarization_Image[Ysite][Xsite + 1] == 0)//寻找进环的上面那一个点
                    {
                      flag_Ysite_1 = Ysite;
                      flag_Xsite_1 = Xsite;
                      Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                      break;
                    }
               }
               if(flag_Ysite_1 != 0)//检测到该点后退出
               {
                   break;
               }
           }
           if(flag_Ysite_1 == 0)//没检测到该点
           {
               for(Ysite=ImageParameter.OFFLine+1;Ysite<30;Ysite++)
               {
                   if(RowAttribute[Ysite].IsLeftFind=='T'&&RowAttribute[Ysite+1].IsLeftFind=='T'&&RowAttribute[Ysite+2].IsLeftFind=='W'
                       &&abs(RowAttribute[Ysite].LeftBorder-RowAttribute[Ysite+2].LeftBorder)>10
                     )//从上往下再次检测该点
                   {
                       flag_Ysite_1=Ysite;
                       flag_Xsite_1=RowAttribute[flag_Ysite_1].LeftBorder;
                       ImageParameter.OFFLine=(uint8)Ysite;
                       Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);//右下角的点跟入环点连线的斜率
                       break;
                   }

               }
           }
           //补线
           if(flag_Ysite_1 != 0)
           {
               for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
               {
                   RowAttribute[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);//从右下角开始向入环点拉线
                       RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder)/2;
               }
               RowAttribute[flag_Ysite_1].RightBorder=flag_Xsite_1;
               for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
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
               }//继续向上扫线的作用？
           }
       }//进程56处理截止
           //环内 小环弯道减半宽 大环不减
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
           //大圆环出环 补线
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1)    //大圆环
       {
           Repair_Point_Xsite = 20;
           Repair_Point_Ysite = 5;
           for (int Ysite = 50; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][23] == 1 && Binarization_Image[Ysite-1][23] == 0)//28
               {
                   Repair_Point_Xsite = 23;//出环寻点一定是在23列吗？
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //截止行重新规划
                   break;
               }//出环寻点，寻那个出环右边的拐点
           }
           for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
           {
               RowAttribute[Ysite].RightBorder = (RowAttribute[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].RightBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //小圆环出环 补线
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2)    //小圆环
       {
           Repair_Point_Xsite = 0;
           Repair_Point_Ysite = 0;
           for (int Ysite = 55; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][15] == 1 && Binarization_Image[Ysite-1][15] == 0)
               {
                   Repair_Point_Xsite = 15;//出环寻点一定是在15列吗？
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //截止行重新规划
                   break;
               }
           }
           for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
           {
               RowAttribute[Ysite].RightBorder = (RowAttribute[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].RightBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //已出环 半宽处理
       if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].RightBorder - Half_Road_Wide[Ysite];
               flag_test=1;
           }
       }//出环半宽处理摆正车头
}

/***右圆环处理***/
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
           //准备进环
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
           //进环
       if(ImageParameter.image_element_rings_flag == 5 && ImageParameter.WhiteLine_L>15)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 6;
       }
           //进环小圆环
       if(ImageParameter.image_element_rings_flag == 6 && ImageParameter.WhiteLine_L<10)
       {
           beepopen;
           ImageParameter.image_element_rings_flag = 7;

           //代码目前直接复制大圆环 大小圆环判断有问题
       }
           //环内 大圆环判断
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
           //环内 小圆环判断
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
           //出环后
       if (ImageParameter.image_element_rings_flag == 8)
       {

            if (   Straight_Judge(1, ImageParameter.OFFLine+15, 40) < 1
                && ImageParameter.WhiteLine_L < 10
                && ImageParameter.OFFLine < 7)    //右边为直线且截止行（前瞻值）很小
   //        if (    (((-8<(ring_yaw-yaw))&&((ring_yaw-yaw)<8)) ||
   //                ((-360<(ring_yaw-yaw))&&((ring_yaw-yaw)<-352)) ||
   //                ((352<(ring_yaw-yaw))&&((ring_yaw-yaw)<360)))
   //                && ImageParameter.WhiteLine_L < 10
   //                && ImageParameter.OFFLine < 7

   //            )    //右边为直线且截止行（前瞻值）很小
               {
                beepopen;
                ImageParameter.image_element_rings_flag = 9;
               }
       }

       //结束圆环进程
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

       /***************************************处理**************************************/
            //准备进环  半宽处理
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

           //进环  补线
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
           //补线
           if(flag_Ysite_1!=0)
           {
               for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
               {
                   RowAttribute[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
                   //if(ImageFlag.ring_big_small==2)//小圆环加半宽
                   //    RowAttribute[Ysite].Center=RowAttribute[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
   //              else//大圆环不加半宽
                       RowAttribute[Ysite].Center=(RowAttribute[Ysite].LeftBorder+RowAttribute[Ysite].RightBorder)/2;//板块
                   //if(RowAttribute[Ysite].Center>79)
                   //    RowAttribute[Ysite].Center=79;
               }
               RowAttribute[flag_Ysite_1].LeftBorder=flag_Xsite_1;
               for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
               {
                   for(Xsite=RowAttribute[Ysite+1].LeftBorder+8;Xsite>RowAttribute[Ysite+1].LeftBorder-4;Xsite--)
                   {
                       if(Binarization_Image[Ysite][Xsite]==1 && Binarization_Image[Ysite][Xsite-1]==0)
                       {
                        RowAttribute[Ysite].LeftBorder=Xsite;
                        RowAttribute[Ysite].Wide=RowAttribute[Ysite].RightBorder-RowAttribute[Ysite].LeftBorder;
                     //   if(ImageFlag.ring_big_small==2)//小圆环加半宽
                     //       RowAttribute[Ysite].Center=RowAttribute[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
                      //  else//大圆环不加半宽
                            RowAttribute[Ysite].Center=(RowAttribute[Ysite].LeftBorder+RowAttribute[Ysite].RightBorder)/2;//板块
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
           //环内不处理
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

           //大圆环出环 补线
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 1)  //大圆环
       {
           Repair_Point_Xsite = 60;
           Repair_Point_Ysite = 0;
           for (int Ysite = 50; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][57] == 1 && Binarization_Image[Ysite-1][57] == 0)
               {
                   Repair_Point_Xsite = 57;
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //截止行重新规划
                           //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                   break;
               }
           }
           for (int Ysite = 57; Ysite >
           Repair_Point_Ysite-3; Ysite--)         //补线
           {
               RowAttribute[Ysite].LeftBorder = (RowAttribute[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].LeftBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //小圆环出环 补线
       if (ImageParameter.image_element_rings_flag == 8 && ImageParameter.ring_big_small == 2)  //小圆环
       {
           Repair_Point_Xsite = 79;
           Repair_Point_Ysite = 0;
           for (int Ysite = 40; Ysite > 5; Ysite--)
           {
               if (Binarization_Image[Ysite][58] == 1 && Binarization_Image[Ysite-1][58] == 0)
               {
                   Repair_Point_Xsite = 58;
                   Repair_Point_Ysite = Ysite-1;
                   ImageParameter.OFFLine = Ysite + 1;  //截止行重新规划
                           //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                   break;
               }
           }
           for (int Ysite = 55; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
           {
               RowAttribute[Ysite].LeftBorder = (RowAttribute[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + RowAttribute[58].LeftBorder;
               RowAttribute[Ysite].Center = (RowAttribute[Ysite].RightBorder + RowAttribute[Ysite].LeftBorder) / 2;
           }
       }
           //已出环 半宽处理
       if (ImageParameter.image_element_rings_flag == 9 || ImageParameter.image_element_rings_flag == 10)
       {
           for (int Ysite = 59; Ysite > ImageParameter.OFFLine; Ysite--)
           {
               RowAttribute[Ysite].Center = RowAttribute[Ysite].LeftBorder + Half_Road_Wide[Ysite];
               flag_test=1;
           }
       }
}

/***十字处理***/
static void Get_ExtensionLine(void){
    if (ImageParameter.WhiteLine >= 8) //丢边行的数量大于8
               TFSite = 55;      //TFSite赋值为55，这个变量是算补线斜率的一个变量
       left_FTSite = 0;
       right_FTSite = 0;
       /********************************************左边补线********************************************/
         if (ExtenLFlag != 'F') //开始补线操作
             for (Ysite = 54; Ysite >= (ImageParameter.OFFLine + 4);Ysite--)
                 {
                     StoreSingleLine = Binarization_Image[Ysite];           //存当前行
                     if (RowAttribute[Ysite].IsLeftFind =='W')
                     {
                       //**************************************************//**************************************************
                       if (RowAttribute[Ysite + 1].LeftBorder >= 70)  //左边界太右
                       {
                           ImageParameter.OFFLine = Ysite + 1;
                           break;                        //直接跳出（极端情况）
                       }
                       //************************************************//*************************************************
                       while (Ysite >= (ImageParameter.OFFLine + 4))//左边界正常
                       {
                           Ysite--;
                           if (RowAttribute[Ysite].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 1].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 2].IsLeftFind == 'T'
                             &&RowAttribute[Ysite - 2].LeftBorder > 0
                             &&RowAttribute[Ysite - 2].LeftBorder < 70
                             )                            //无边行的上面连续三行都是正常边线
                           {
                             left_FTSite = Ysite - 2;    //把本行上面的第二行存入FTsite
                             break;
                           }
                       }
                       /*左边界的斜率*/
                       DetL =
                         ((float)(RowAttribute[left_FTSite].LeftBorder -
                                 RowAttribute[TFSite].LeftBorder)) /
                         ((float)(left_FTSite - TFSite));
                       /*补线操作*/
                       if (left_FTSite > ImageParameter.OFFLine) //此行不可超过所限制的图像顶边
                           for (ytemp = TFSite; ytemp >= left_FTSite; ytemp--)
                           {
                               RowAttribute[ytemp].LeftBorder = //y = k*x + b;
                                       (int)(DetL * ((float)(ytemp - TFSite))) +
                                       RowAttribute[TFSite].LeftBorder;
                           }//这个补线操是从十字丢线的起始点到丢线的终止点连接起来
                   }//无边行处理
                   else
                       TFSite = Ysite + 2; //如果扫到了本行的左边界，该行存在这里面，（算斜率）
                 }
}

/***中线平滑滤波函数***/
static void RouteFilter(void){
    for (Ysite = 58; Ysite >= (ImageParameter.OFFLine + 5);
         Ysite--)
    {
      if (   RowAttribute[Ysite].IsLeftFind == 'W'
           &&RowAttribute[Ysite].IsRightFind == 'W'
           &&Ysite <= 45
           &&RowAttribute[Ysite - 1].IsLeftFind == 'W'
           &&RowAttribute[Ysite - 1].IsRightFind =='W')  //当前行左右都无边且前一行也无边，而且在前45行   滤波
      {
        ytemp = Ysite;
        while (ytemp >= (ImageParameter.OFFLine +5))
        {
          ytemp--;
          if (  RowAttribute[ytemp].IsLeftFind == 'T'
              &&RowAttribute[ytemp].IsRightFind == 'T')  //寻找两边都正常的，找到离本行最近的就不找了
          {
            DetR = (float)(RowAttribute[ytemp - 1].Center - RowAttribute[Ysite + 2].Center)
                    /(float)(ytemp - 1 - Ysite - 2);          //算斜率
            int CenterTemp = RowAttribute[Ysite + 2].Center;
            int LineTemp = Ysite + 2;
            while (Ysite >= ytemp) {
                RowAttribute[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp)); //用斜率补
              Ysite--;
            }
            break;
          }
        }
      }
      RowAttribute[Ysite].Center =
              (RowAttribute[Ysite - 1].Center + 2 * RowAttribute[Ysite].Center) /3; //求平均，应该会比较滑  本来是上下两点平均
    }
}//这里这个滤波的作用应为:在前四十五行中如果出现了左右都丢边且不是元素时，利用这个滤波处理


/***前瞻控制误差函数***/
void Prospective_error(void){
    float DetTemp = 0;
    int TowPoint = 0;
    float UnitAll = 0;

    //Speed_Control_Factor();
    /*前瞻整定*/
    if(ImageParameter.image_element_rings_flag != 0)  1;/*TowPoint = circle[circle_count_flag];*/
           //?
    else TowPoint = ImageParameter.TowPoint;//25

    /*前瞻限幅*/
    if (TowPoint < ImageParameter.OFFLine)
            TowPoint = ImageParameter.OFFLine + 1;
    if (TowPoint >= 49) TowPoint = 49;

    if ((TowPoint - 5) >= ImageParameter.OFFLine) { //前瞻取设定前瞻还是可视距离  需要分情况讨论
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
        DetTemp = (float)ImageParameter.Det_True;//如果是出现OFFLine>50情况，保持上一次的偏差值

    ImageParameter.Det_True = DetTemp;//此时的解算出来的平均图像偏差，这里算出来的应该为平均中线而不是平均误差
}


/***紧急制动函数***/
volatile uint8_t Emergency_Flag = 0; // 新增全局急停标志

// 修改后的Emergency_Breaking函数（主循环调用）
void Emergency_Breaking(void) {     
    static uint8_t safety_cnt = 0; // 改为静态变量
    
    if(ImageParameter.OFFLine >= 55) {
        if(safety_cnt < 5) safety_cnt++;  // 消抖计数
        if(safety_cnt >= 3) {             // 连续3帧异常才触发
            Emergency_Flag = 1;           // 设置标志，不直接操作Stop
        }
    } else {
        safety_cnt = 0;
        Emergency_Flag = 0;  // 自动解除
    }
}



/***显示中线值***/
void printf_halfwide(void)
{
    for(int i = 0 ; i < 60 ; i++)
    {
        printf("%d\n",(RowAttribute[i].RightBorder - RowAttribute[i].LeftBorder)/2);
    }
}


/***元素处理*****/
void Element_Handle(void){


    if (ImageParameter.image_element_rings == 1)
         Left_Rings_Handle();//左圆环处理
    else if (ImageParameter.image_element_rings == 2)
        Right_Rings_Handle();//右圆环处理


    if(ImageParameter.WhiteLine >= 8)
    {
        flag_cross=1;
        Get_ExtensionLine();//十字补线
    }
    else flag_cross=0;
    RouteFilter();//中线平滑滤波，处理突然出现的非元素丢边情况
   // Emergency_Breaking();//紧急制动  暂时关掉
}

// 调整静态阈值和分割阈值
uint32 Threshold_low = 30;  // 原40 → 30 增强白线敏感度
uint8 detach = 220;         // 原240 → 220 降低分割阈值
int MaxSpeed = 270;
int MinSpeed = 190;
int ance_acc = 70;

void Data_Settings(void){//图像参数初始化
    ImageParameter.Threshold_static = Threshold_low;//环境越亮值越大，范围（40-80）
    ImageParameter.Threshold_detach = detach;
   // ImageScanInterval = 2;               //扫边范围    上一行的边界+-ImageScanInterval
   // ImageScanInterval_Cross = 7;         //十字扫线范围
    // 缩小扫线范围和十字检测范围
    ImageScanInterval = 1;         // 原2 → 1 减少横向搜索范围
    ImageScanInterval_Cross = 4;   // 原7 → 4 窄线十字特征更集中
    
    SystemData.SpeedData.MaxSpeed = MaxSpeed;
    SystemData.SpeedData.MinSpeed = MinSpeed;
    ImageParameter.TowPoint = 21;           //前瞻//24
    ImageParameter.variance_acc = ance_acc;       //直道检测
    
    
     ImageParameter.width_threshold = 20;  // 根据实际赛道调整

}


// 在图像处理流程中添加动态曝光计算
void dynamic_exposure_adjust() {
    // 统计远端区域（如顶部30行）的平均亮度
    uint32_t far_avg = 0;
    for(int y=0; y<30; y++) {
        for(int x=0; x<MT9V03X_W; x++) {
            far_avg += mt9v03x_image[y][x];
        }
    }
    far_avg /= (30 * MT9V03X_W);

    // 根据亮度调整曝光（示例逻辑）
    if(far_avg < 80) {       // 远端过暗
        mt9v03x_set_exposure_time(MT9V03X_EXP_TIME_DEF + 5);
    } else if(far_avg > 200) { // 远端过亮
        mt9v03x_set_exposure_time(MT9V03X_EXP_TIME_DEF - 5);
    }
}




typedef struct {
    uint16 y_start;
    uint16 y_end;
    float  weight; // 权重系数
} exposure_zone;

exposure_zone zones[] = {
    {0,  30, 0.7f},  // 远端区域，高权重
    {90, 120, 0.3f}  // 近端区域，低权重
};

void weighted_exposure_adjust() {
    static uint16 current_exp = MT9V03X_EXP_TIME_DEF;
    
    // 1. 计算加权平均亮度
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

    // 2. 调整曝光（Bang-Bang控制）
    if(brightness < 90) {
        current_exp += 10; // 过暗时大幅增加
    } else if(brightness > 150) {
        current_exp -= 20; // 过亮时快速减少
    } else {
        current_exp += (brightness < 110) ? 5 : -5; // 微调
    }
    
    mt9v03x_set_exposure_time(current_exp);
}


void enhance_far_region() {
    // 1. 统计远端直方图
    uint8 min_val = 255, max_val = 0;
    for(int y=FAR_ROI_START_Y; y<FAR_ROI_END_Y; y++) {
        for(int x=0; x<MT9V03X_W; x++) {
            uint8 pix = mt9v03x_image[y][x];
            if(pix < min_val) min_val = pix;
            if(pix > max_val) max_val = pix;
        }
    }
    
    // 2. 线性拉伸
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
        
        // 每5帧调整一次曝光（平衡响应速度与稳定性）
        if(frame_cnt++ % 5 == 0) {
            weighted_exposure_adjust();
        }
        
        enhance_far_region(); // 实时增强显示
    }
}















/*** 计算截止行以上白线中线函数 ***/
float CalculateOFFLineCenter(void) 
{
  int i;
 float sum=0.0;
  
for(i=55;i>47;i--)
{
sum+=RowAttribute[i].Center;
}
    
    // 遍历截止行到图像底部（假设最大行为59）
   
    
    // 防止除零错误
    return sum/8;
}




/**
  * @brief 统计指定行的白色像素数量
  * @param row 要检测的行号（0-59）
  * @return 白色像素数量（若行号无效返回-1）
  */
int CountWhitePixelsInRow(uint8 row) 
{
    // 参数有效性检查
    if(row >= IPSH) {
        return -1; // 错误码：行号超出范围
    }
    
    int white_count = 0;
    uint8* line_data = Binarization_Image[row]; // 获取目标行数据指针
    
    // 并行优化：每次处理4个像素（根据CPU架构调整）
    const int parallel_size = IPSW - (IPSW % 4);
    
    // 主循环处理大部分数据
    for(int x = 0; x < parallel_size; x += 4) {
        white_count += line_data[x] + line_data[x+1] 
                      + line_data[x+2] + line_data[x+3];
    }
    
    // 处理剩余像素
    for(int x = parallel_size; x < IPSW; x++) {
        white_count += line_data[x];
    }
    
    return white_count;
}









/***总图像处理函数***/
void ImageProcess(void){
  // 计算平均赛道宽度
    int avg_width = 0;
    for(int y=50; y<60; y++){
        avg_width += RowAttribute[y].Wide;
    }
    avg_width /= 10;
    
    // 动态调整扫线参数
    ImageScanInterval = (avg_width < 15) ? 1 : 2; // 窄赛道使用更小搜索范围
    
    Cramping();//图像压缩
    ImageParameter.OFFLine = 2;//取图像顶边，太靠前的图像不可信
    ImageParameter.WhiteLine = 0;
    for(Ysite=59;Ysite>=ImageParameter.OFFLine;Ysite--){
        RowAttribute[Ysite].IsLeftFind = 'F';
        RowAttribute[Ysite].IsRightFind = 'F';
        RowAttribute[Ysite].LeftBorder = 0;
        RowAttribute[Ysite].RightBorder = 79;
    }//边界初始化
    Threshold_Speraration_Ostu();//图像二值化阈值大津法获取及其二值化
    Bin_Image_Filter();//去除噪点
    DrawBasicLines();//扫描前五行
    DrawProcessLines();//扫描54行前的各行边界
    Emergency_Breaking();
    Eightfields_Search_Border(Binarization_Image, IPSH, IPSW, IPSH - 2);//八领域扫线用于元素处理
    /***元素识别***/
  //  Element_Test();//直道、圆环、斑马线检测
    /***元素处理*****/
   // Element_Handle();//圆环、十字处理、中线滤波、制动
    Prospective_error();//前瞻控制误差
}

