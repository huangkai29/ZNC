#include "common.h"

#define Img_Col 80 //ͼ����
#define servo_freq 50
#define servo_FTM FTM0
#define servo_CH FTM_CH0
#define servo_pwm_middle  180    // �����ֵ
#define servo_pwm_max  225      // ���ƫת���ֵ
#define servo_pwm_min  135  
 
#define White_Line_Min 10   //��С�������
#define White_Line_Max 80   //����������
 
float KP=8;//����������ϵ��
float KD=0.08; //5.0;//�������΢��ϵ��
int Fit_Middleline[81];
void get_centerline(uint16 img[81][61])    //  ��ȡ����
{
   uint8 Left_Black,Left_Black_Old;
   uint8 Right_Black,Right_Black_Old;
   uint8 Middleline;  
   uint8 Left_Black_Flag=0;
   uint8 Right_Black_Flag=0;
   uint8 i,j;
   
   for(i=60;i>1;i--)  //����ͼ���ÿһ�ж�����ɨ�裨Ч�ʵͣ�
   {
    for(j=40;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {
      if(img[i][j]==0xfd && img[i][j-1]==0x00)
      {
        Left_Black=j;       // �ҵ���ߺڵ�
        Left_Black_Old=Left_Black;  // ������һ�ε�ֵ������һ��δ�ҵ��ڵ�ʱ������һ�εĺڵ���Ϊ���εĺڵ�
        Left_Black_Flag++;  //�ҵ��������Left_Black_Flag��1 �����ں���·�����͵��ж�
        break;
      }
      else
      {
        Left_Black=j;  // δ�ҵ���ߺڵ�
        Left_Black_Flag=0;
      }
    }
    
    for(j=41;j<80;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {
      if(img[i][j]==0xfd && img[i][j+1]==0x00)
      {
        Right_Black=j;         //�ҵ��ұߺڵ�
        Right_Black_Old=Right_Black;    // ������һ�ε�ֵ������һ��δ�ҵ��ڵ�ʱ������һ�εĺڵ���Ϊ���εĺڵ�
        Right_Black_Flag++;
        break; 
      }
      else
      {
        Right_Black=j;   //δ�ҵ��ұߺڵ�
        Right_Black_Flag=0;
      }
    }
      
    //////////////////////  ��·�ж�    -------------///////////////////////
     ////////////          ����ֱ��    ///////////////////
    if(Left_Black_Flag==1 && Right_Black_Flag==1)    //�ҵ�˫�ߺ���, ����ֱ����
    {
      if((Right_Black-Left_Black>=White_Line_Min)&& (Right_Black-Left_Black<=White_Line_Max)) //ʹ���߿���ڹ涨�ķ�Χ�ڣ����޶������Ŀ�ȣ���Ϊ�ɼ����ܹ���80�еģ���֪���������߲�ֵӦ��0~80֮��
       {                                                                                  
         Middleline=(Left_Black + Right_Black)/2;
        Fit_Middleline[i]=Middleline;
      }
    }
    /////////////   ֱ������     /////////////////////
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==1)   //ֻ�ҵ��Ҳ���ߣ�֤������������
    {
      if((Right_Black >=White_Line_Min) && (Right_Black <=White_Line_Max))
       {
         Middleline=Right_Black/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   //////////////////// 
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //ֻ�ҵ������ߣ�֤������������
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=(Left_Black+Img_Col)/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   ////////////////////
    ////////////    ����ʮ��·�ڻ��߶�ʧ���ߺ���   /////////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==0)   //���߶�û�ҵ����ߣ����������У�������һ�е�ֵ
    {
     
         Left_Black=Left_Black_Old;
         Right_Black=Right_Black_Old;
         Middleline=(Left_Black + Right_Black)/2;
         Fit_Middleline[i]=Middleline;
     }  
 }
}
 
//�������Ҳ��Ӧд����򵥵Ĵ��룺

int Servo_Control_PWM;
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   
   for(i=55;i>35;i--)  //���Խ�����20��ȡƽ��ֵ
    SteerSum+=Fit_Middleline[i]-Img_Col/2;//ע�⣺Fit_Middleline[i]-Img_Col/2ʱ��Ӧ��//Servo_PWM + servo_pwm_middleʱ����ͷ����װ���������ܣ�����
 // �������Img_Col/2-Fit_Middleline[i]ʱ��Ӧ�� servo_pwm_middle-Servo_PWM ʱ����ͷ����װ�ſ��������ܣ����Ա��֪��
   Error=(int)(SteerSum/20);
  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //�޶������Ƿ�Χ����ֹ����������������ת���ķ�//Χ��servo_pwm_max����������ת�ǣ��ɵ���ռ�ձ�ʹ�����Ǻ���������ת�ĽǶȶ�//Ӧ�����ռ�ձȣ�������ɵó���������ת�������ռ�ձȺ���Сռ�ձȡ�
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //ƫ��̫С�Ͳ��ı����Ƕ�
     Servo_PWM=servo_pwm_middle;    //ʹ��ԭ�������ֵ 
   
  Servo_Control_PWM=Servo_PWM;
  return Servo_PWM;
 
}