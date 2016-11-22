#include "common.h"
#include "include.h"
#include "math.h"

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint8 img[CAMERA_W*CAMERA_H];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��

//��������
void sendimg(uint8 *imgaddr, uint32 imgsize);          //����ͼ����λ��
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void PORTD_IRQHandler();
void DMA0_IRQHandler();
void FTM_PWM_Duty(FTMn_e ftmn, FTM_CHn_e ch, uint32 duty);
void FTM_PWM_init(FTMn_e, FTM_CHn_e, uint32 freq, uint32 duty);  //��ʼ��FTM��PWM���ܲ�����Ƶ�ʡ�ռ�ձȡ�����ͨ�����ռ�ձȡ�ͬһ��FTM����ͨ����PWMƵ����һ���ģ���3��FTM
void FTM_PWM_Duty(FTMn_e, FTM_CHn_e,              uint32 duty);  //����ͨ��ռ�ձ�,ռ�ձ�Ϊ ��duty * ���ȣ� % ����� FTM_PRECISON ����Ϊ 1000 ��duty = 100 ����ռ�ձ� 100*0.1%=10%
void FTM_PWM_freq(FTMn_e,            uint32 freq);               //����FTM��Ƶ�ʣ���Ƶ�ʺ���Ҫ��������ռ�ձȣ�
void port_init(PTXn_e ptxn, uint32 cfg );
void port_init_NoALT(PTXn_e ptxn, uint32 cfg);
void PIT0_IRQHandler(void);
extern void FTM_QUAD_Init(FTMn_e ftmn);         //��ʼ��FTM ���������� ����
extern int16 FTM_QUAD_get(FTMn_e ftmn);          //��ȡFTM �������� ��������(������ʾ������)
extern void FTM_QUAD_clean(FTMn_e ftmn);        //�� FTM �������� ��������
void PID_init();     //PID��ʼ��
float PID_realize(float actspeed); //PID����ƫ��ֵ
float geterr(); //��ȡ���������
void get_centerline(uint8 *img);    //  ��ȡ����
int servo_control(void);                

/*
������ţ�PTC1 FTM0
USART3��C17 

����ͷ:
����:B0-B7
SDA:A25
SCL:A26
CLK:A27
HRE:A28

������:A12(����) A13(����) FTM1

���PWM:FTM2_CH0 PTB18

busƵ����Ϊ100MHz(50MHz busƵ�ʻ�̫��������û����ʱ�ɼ�ͼ��)
volatile  */




int asas;

void  main(void)
{
  

    

    //��ʼ������ͷ
    camera_init(imgbuff);

    //�����жϸ�λ����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); //���� PIT0 ���жϷ�����Ϊ PIT0_IRQHandler

    PID_init();
    
    FTM_QUAD_Init(FTM1); //FTM1 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
    port_init_NoALT(FTM1_QDPHA,PULLUP);
    port_init_NoALT(FTM1_QDPHB,PULLUP);
    
    pit_init_ms(PIT0,150); //��ʼ�� PIT0����ʱʱ��
    

    

    
    //��ʼ�����( >1800 ���� <1800���� Ƶ��50-300 ����ֵ+ - 450)
    FTM_PWM_init(FTM0, FTM_CH0,185, 1800); 

    //��ʼ�����
    FTM_PWM_init(FTM2, FTM_CH0,10000,0); 
  
    NVIC_EnableIRQ(PIT0_IRQn);//ʹ��PIT0�ж�
    while(1)
    {
        //��ȡͼ��
        camera_get_img();                                   //����ͷ��ȡͼ��

        //��ѹͼ��
        img_extract(img, imgbuff,CAMERA_SIZE); 

        //��ȡ����
        get_centerline(img);
        
        asas=servo_control();
        
  //     FTM_PWM_init(FTM0, FTM_CH0,185, 180+asas);
        
        //����ͼ����λ��
       sendimg(img, CAMERA_W * CAMERA_H);                  //���͵���λ��
        

    }
}

/*!
 *  @brief      ����ͼ��eSmartCameraCar��λ����ʾ
 *  @param      imgaddr         ͼ���ַ
 *  @param      imgsize         ͼ��ռ�ÿռ��С
 *  @since      v5.0
 *  @note       ��ͬ����λ������ͬ���������ʹ�� eSmartCameraCar�����
                ���ʹ��������λ��������Ҫ�޸Ĵ��롣
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
    uint8 cmd[1] = {255 };    //yy_����ͷ���ڵ��� ʹ�õ�����

    

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��
    uart_putbuff(VCAN_PORT, cmd, sizeof(cmd));    //�ȷ�������
    
    
}

/*!
 *  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
 *  @param      dst             ͼ���ѹĿ�ĵ�ַ
 *  @param      src             ͼ���ѹԴ��ַ
 *  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{

    uint8 colour[2] = {254, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{

    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
       
        camera_vsync();
    }

}

/*!
 *  @brief      DMA0�жϷ�����
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
  actspeed = FTM_QUAD_get(FTM1); //��ȡ FTM �������� ��������(������ʾ������)
  
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
  FTM_QUAD_clean(FTM1); //�����������
  PIT_Flag_Clear(PIT0); //���жϱ�־λ
}



