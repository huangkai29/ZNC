
#include "common.h"
#include "include.h"

typedef struct _pid{
float SetSpeed; //定义设定值
float ActualSpeed; //定义实际值
float err; //定义偏差值
float err_next; //定义上一个偏差值
float err_last; //定义最上前的偏差值
float Kp,Ki,Kd; //定义比例、积分、微分系数
}pid;


pid pid1;



void PID_init(){

  pid1.SetSpeed=2000;
  pid1.ActualSpeed=0.0;
  pid1.err=0.0;
  pid1.err_last=0.0;
  pid1.err_next=0.0;
  pid1.Kp=0.2;
  pid1.Ki=0.015;
  pid1.Kd=0.13;
  

}
float geterr()
{
  return pid1.err; //传出偏差值
}

float PID_realize(float actspeed){

  pid1.ActualSpeed=actspeed;
  pid1.err=pid1.SetSpeed-pid1.ActualSpeed;
  float incrementSpeed=pid1.Kp*(pid1.err-pid1.err_next)+pid1.Ki*pid1.err+pid1.Kd*(pid1.err-2*pid1.err_next+pid1.err_last);
  pid1.err_last=pid1.err_next;
  pid1.err_next=pid1.err;
  return incrementSpeed;
  
}