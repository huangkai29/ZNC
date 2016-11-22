#include "common.h"
#include "include.h"
#include "math.h"

uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_W*CAMERA_H];                           //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理

//函数声明
void sendimg(uint8 *imgaddr, uint32 imgsize);          //发送图像到上位机
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void PORTD_IRQHandler();
void DMA0_IRQHandler();
void FTM_PWM_Duty(FTMn_e ftmn, FTM_CHn_e ch, uint32 duty);
void FTM_PWM_init(FTMn_e, FTM_CHn_e, uint32 freq, uint32 duty);  //初始化FTM的PWM功能并设置频率、占空比。设置通道输出占空比。同一个FTM，各通道的PWM频率是一样的，共3个FTM
void FTM_PWM_Duty(FTMn_e, FTM_CHn_e,              uint32 duty);  //设置通道占空比,占空比为 （duty * 精度） % ，如果 FTM_PRECISON 定义为 1000 ，duty = 100 ，则占空比 100*0.1%=10%
void FTM_PWM_freq(FTMn_e,            uint32 freq);               //设置FTM的频率（改频率后，需要重新配置占空比）
void port_init(PTXn_e ptxn, uint32 cfg );
void port_init_NoALT(PTXn_e ptxn, uint32 cfg);
void PIT0_IRQHandler(void);
extern void FTM_QUAD_Init(FTMn_e ftmn);         //初始化FTM 的正交解码 功能
extern int16 FTM_QUAD_get(FTMn_e ftmn);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
extern void FTM_QUAD_clean(FTMn_e ftmn);        //清 FTM 正交解码 的脉冲数
void PID_init();     //PID初始化
float PID_realize(float actspeed); //PID返回偏差值
float geterr(); //获取舵机控制量
void get_centerline(uint8 *img);    //  提取黑线
int servo_control(void);                

/*
舵机引脚：PTC1 FTM0
USART3：C17 

摄像头:
数据:B0-B7
SDA:A25
SCL:A26
CLK:A27
HRE:A28

编码器:A12(黑线) A13(白线) FTM1

电机PWM:FTM2_CH0 PTB18

bus频率设为100MHz(50MHz bus频率会太慢而导致没法及时采集图像)
volatile  */




int asas;

void  main(void)
{
  

    

    //初始化摄像头
    camera_init(imgbuff);

    //配置中断复位函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置LPTMR的中断复位函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置LPTMR的中断复位函数为 PORTA_IRQHandler
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); //设置 PIT0 的中断服务函数为 PIT0_IRQHandler

    PID_init();
    
    FTM_QUAD_Init(FTM1); //FTM1 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
    port_init_NoALT(FTM1_QDPHA,PULLUP);
    port_init_NoALT(FTM1_QDPHB,PULLUP);
    
    pit_init_ms(PIT0,150); //初始化 PIT0，定时时间
    

    

    
    //初始化舵机( >1800 往左 <1800往右 频率50-300 极限值+ - 450)
    FTM_PWM_init(FTM0, FTM_CH0,185, 1800); 

    //初始化电机
    FTM_PWM_init(FTM2, FTM_CH0,10000,0); 
  
    NVIC_EnableIRQ(PIT0_IRQn);//使能PIT0中断
    while(1)
    {
        //获取图像
        camera_get_img();                                   //摄像头获取图像

        //解压图像
        img_extract(img, imgbuff,CAMERA_SIZE); 

        //获取中线
        get_centerline(img);
        
        asas=servo_control();
        
  //     FTM_PWM_init(FTM0, FTM_CH0,185, 180+asas);
        
        //发送图像到上位机
       sendimg(img, CAMERA_W * CAMERA_H);                  //发送到上位机
        

    }
}

/*!
 *  @brief      发送图像到eSmartCameraCar上位机显示
 *  @param      imgaddr         图像地址
 *  @param      imgsize         图像占用空间大小
 *  @since      v5.0
 *  @note       不同的上位机，不同的命令，这里使用 eSmartCameraCar软件，
                如果使用其他上位机，则需要修改代码。
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
    uint8 cmd[1] = {255 };    //yy_摄像头串口调试 使用的命令

    

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
    uart_putbuff(VCAN_PORT, cmd, sizeof(cmd));    //先发送命令
    
    
}

/*!
 *  @brief      二值化图像解压（空间 换 时间 解压）
 *  @param      dst             图像解压目的地址
 *  @param      src             图像解压源地址
 *  @param      srclen          二值化图像的占用空间大小
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{

    uint8 colour[2] = {254, 0}; //0 和 1 分别对应的颜色
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8 tmpsrc=0;

    while(srclen --)
    {

        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];

    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{

    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
       
        camera_vsync();
    }

}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

float s1=0;
uint8 w[9];
void PIT0_IRQHandler(void)

{
  
  
  int16 actspeed;
  actspeed = FTM_QUAD_get(FTM1); //获取 FTM 正交解码 的脉冲数(负数表示反方向)
  
  //sprintf(w,"%d",actspeed);
  //uart_putstr(UART3,w);  
  if(1)
  {
    
 //  actspeed=(actspeed/500)/0.15*2*3.141*0.008;//M/S
 

   if(s1<=2500)
   {
     float q=PID_realize(actspeed);
     float err=geterr();
     if (err<0.035 && err>(-0.035))
       ;       
     else if (err<50 && err>(-50))
//       s1=s1+q*8;
          s1=s1+q;
     else if(err<200 && err>(-200))
       s1=s1+q;
     else 
       s1=s1+q*1.7;       
     FTM_PWM_Duty(FTM2, FTM_CH0,(int)s1);
    // FTM_PWM_init(FTM2, FTM_CH0,10000,(int)s1); 

   } 
   else if(s1>2600)
      FTM_PWM_init(FTM2, FTM_CH0,10000,0);
    
    

  }
  else
  {
    ;
  }
  FTM_QUAD_clean(FTM1); //清除正交解码
  PIT_Flag_Clear(PIT0); //清中断标志位
}



