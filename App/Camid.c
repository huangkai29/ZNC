#include "common.h"

#define Img_Col 80 //图像宽度
#define servo_freq 50
#define servo_FTM FTM0
#define servo_CH FTM_CH0
#define servo_pwm_middle  180    // 舵机中值
#define servo_pwm_max  225      // 舵机偏转最大值
#define servo_pwm_min  135  
 
#define White_Line_Min 10   //最小赛道宽度
#define White_Line_Max 80   //最大赛道宽度
 
float KP=8;//舵机方向比例系数
float KD=0.08; //5.0;//舵机方向微分系数
int Fit_Middleline[81];
void get_centerline(uint16 img[81][61])    //  提取黑线
{
   uint8 Left_Black,Left_Black_Old;
   uint8 Right_Black,Right_Black_Old;
   uint8 Middleline;  
   uint8 Left_Black_Flag=0;
   uint8 Right_Black_Flag=0;
   uint8 i,j;
   
   for(i=60;i>1;i--)  //整幅图像的每一行都进行扫描（效率低）
   {
    for(j=40;j>1;j--)  // 从中间向左边搜索，寻找黑点
    {
      if(img[i][j]==0xfd && img[i][j-1]==0x00)
      {
        Left_Black=j;       // 找到左边黑点
        Left_Black_Old=Left_Black;  // 保存上一次的值，当下一次未找到黑点时，将上一次的黑点作为本次的黑点
        Left_Black_Flag++;  //找到左黑线则Left_Black_Flag加1 ，用于后面路径类型的判断
        break;
      }
      else
      {
        Left_Black=j;  // 未找到左边黑点
        Left_Black_Flag=0;
      }
    }
    
    for(j=41;j<80;j++)          // 从中间向右边搜索，寻找黑点
    {
      if(img[i][j]==0xfd && img[i][j+1]==0x00)
      {
        Right_Black=j;         //找到右边黑点
        Right_Black_Old=Right_Black;    // 保存上一次的值，当下一次未找到黑点时，将上一次的黑点作为本次的黑点
        Right_Black_Flag++;
        break; 
      }
      else
      {
        Right_Black=j;   //未找到右边黑点
        Right_Black_Flag=0;
      }
    }
      
    //////////////////////  道路判断    -------------///////////////////////
     ////////////          进入直道    ///////////////////
    if(Left_Black_Flag==1 && Right_Black_Flag==1)    //找到双边黑线, 车在直道上
    {
      if((Right_Black-Left_Black>=White_Line_Min)&& (Right_Black-Left_Black<=White_Line_Max)) //使黑线宽度在规定的范围内，即限定赛道的宽度，因为采集的总共有80列的，可知赛道的两边差值应在0~80之间
       {                                                                                  
         Middleline=(Left_Black + Right_Black)/2;
        Fit_Middleline[i]=Middleline;
      }
    }
    /////////////   直道结束     /////////////////////
    /////////////   进入左弯道   /////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==1)   //只找到右侧黑线，证明黑线向左弯
    {
      if((Right_Black >=White_Line_Min) && (Right_Black <=White_Line_Max))
       {
         Middleline=Right_Black/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   左弯道结束   //////////////////// 
    /////////////   进入右弯道   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //只找到左侧黑线，证明黑线向右弯
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=(Left_Black+Img_Col)/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   右弯道结束   ////////////////////
    ////////////    进入十字路口或者丢失两边黑线   /////////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==0)   //两边都没找到黑线，则舍弃该行，沿用上一行的值
    {
     
         Left_Black=Left_Black_Old;
         Right_Black=Right_Black_Old;
         Middleline=(Left_Black + Right_Black)/2;
         Fit_Middleline[i]=Middleline;
     }  
 }
}
 
//舵机控制也对应写个最简单的代码：

int Servo_Control_PWM;
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   
   for(i=55;i>35;i--)  //仅对近处的20行取平均值
    SteerSum+=Fit_Middleline[i]-Img_Col/2;//注意：Fit_Middleline[i]-Img_Col/2时对应于//Servo_PWM + servo_pwm_middle时摄像头正向安装可以正常跑，，，
 // 但是如果Img_Col/2-Fit_Middleline[i]时对应于 servo_pwm_middle-Servo_PWM 时摄像头反向安装才可以正常跑，测试便可知。
   Error=(int)(SteerSum/20);
  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //限定舵机打角范围，防止超过两个轮子所能转过的范//围，servo_pwm_max是轮子最大的转角，可调节占空比使舵机打角后轮子所能转的角度对//应的最大占空比，这样便可得出轮子所能转过的最大占空比和最小占空比。
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //偏差太小就不改变舵机角度
     Servo_PWM=servo_pwm_middle;    //使用原来舵机的值 
   
  Servo_Control_PWM=Servo_PWM;
  return Servo_PWM;
 
}