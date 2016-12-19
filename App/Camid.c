#include "common.h"



#define plotmid 0 //是否画中线
#define P 16 //修正数组偏差值 
#define img_top 41 //图像上部 
#define img_base 100 //图像下部 
#define img_high img_base-img_top+1 //图像高度 
#define Img_Col 160 //图像宽度

#define servo_pwm_middle  3400    // 舵机中值
#define servo_pwm_max  4200      // 舵机偏转最大值
#define servo_pwm_min  2600     //舵机偏转最小值
#define White_Line_Min 20   //最小赛道宽度
#define White_Line_Max 160   //最大赛道宽度
#define N (i-1)*Img_Col+(j-1) //二维坐标转换为一维数组对应数据  
#define Xi i-img_top+1 //实际行数转换到从0开始的行数 


 
  

float KP=28;//舵机方向比例系数
float KD=0.08; //5.0;//舵机方向微分系数
uint16 Fit_Middleline[img_high+1];

//修正数组 
int xz[60]={27+P,29+P,30+P,30+P,32+P,32+P,34+P,36+P,37+P,38+P,40+P,41+P,42+P,43+P,44+P,46+P,48+P,49+P,50+P,52+P,53+P,54+P,56+P,56+P,58+P,60+P,61+P,63+P,63+P,65+P,66+P,67+P,69+P,71+P,71+P,73+P,75+P,76+P,77+P,78+P,80+P,81+P,82+P,83+P,84+P,85+P,87+P,87+P,89+P,91+P,91+P,93+P,93+P,95+P,95+P,97+P,98+P,99+P,100+P,101+P};

int16 Left_Black[img_high+1];
int16 Right_Black[img_high+1];
int get_centerline(uint8 img[19200])    //  提取黑线
{
   uint8 tenflag=0;
   uint8 Rlost=0;
   uint8 Llost=0;
   uint8 Middleline=80;  
   int16 i,j;

   ////////////////////////////////////////搜索前三行,判断是否是有效图像/////////////////////////////
    
   for(i=img_base;i>=img_base-2;i--)  //从中线开始搜索前三行 
   {   	
    for(j=Img_Col/2;j>1;j--)  // 从中间向左边搜索，寻找黑点
    {      
      if(img[N]==254 && img[N-1]==0 )
      {     	
        Left_Black[Xi]=j;       // 找到左边黑点
        break;
      }
      else
      	Left_Black[Xi]=1;         		  
    }
    if(Left_Black[Xi]==1) //左丢线 
      	Llost++;
    
    for(j=(Img_Col/2);j<160;j++)          // 从中间向右边搜索，寻找黑点
    {      
      if(img[N]==254 && img[N+1]==0)
      {
        Right_Black[Xi]=j;         //找到右边黑点
        break; 
      }	        
	  else
	  
	  	Right_Black[Xi]=255;	  		    
    }
    if(Right_Black[Xi]==255)  //右丢线 
    	Rlost++;
    //最近三行有全黑行则舍弃 
    if(img[(i-1)*160+(80-1)]==0)
        return 1;
    if((Left_Black[Xi]==1 && Right_Black[Xi]==255) )
    	return 0;

    
    	
    else //不舍弃则补线 
    {
    	if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
	}
   
 

 }
  /////////////////////////////图像有效,继续搜索 ////////////////////////////////////////
   for(i=img_base-3;i>=img_top;i--)  //边沿寻点 
   {   
   
   	
   	uint8 oldlb=Left_Black[Xi+1];
	if(oldlb-6<=1) oldlb=8; //防止越界 
	for(j=oldlb+6;j>=oldlb-6;j--)  //从上次的点搜索 
	{    	  
	  	if(img[N]==254 && img[N-1]==0 )
	   	{     	
	    	Left_Black[Xi]=j;       
	        break;
    	} 
			    	    		
	}
	if(j==oldlb-7) //边沿寻点不成，改为朴素的选点 
	{
		
		for(j=Img_Col/2;j>1;j--)  // 从中间向左边搜索，寻找黑点
	    {      
	      if(img[N]==254 && img[N-1]==0 )
	      {     	
	        Left_Black[Xi]=j;       // 找到左边黑点
	        break;
	      }
	      else
	      	Left_Black[Xi]=1;	      		  
	    }
	    if(Left_Black[Xi]==1 && i>=img_base-10) //左丢线 
      		Llost++;
	}
	          
	uint8 oldrb=Right_Black[Xi+1];
	if(oldrb+6>=160) oldrb=153; //防止越界 
	for(j=oldrb-6;j<=oldrb+6;j++)          // 从上次的点搜索 
    {      
	 		      			 
	 	 if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         
	        break; 
	      }	 
		      
    }
    if(j==oldrb+7) //边沿寻点不成，改为朴素的选点 
    {
    	
    	for(j=(Img_Col/2);j<160;j++)          // 从中间向右边搜索，寻找黑点
	    {      
	      if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         //找到右边黑点
	        break; 
	      }	        
		  else
		  	Right_Black[Xi]=255;		  		    
	    }	
	    if(Right_Black[Xi]==255 && i>=img_base-10) //左丢线 
      		Rlost++;
	}
	
    //十字路口标记 
    uint8 Firmid=(Right_Black[img_base]-Left_Black[img_base])/2;
    if(Left_Black[Xi]==1 && Right_Black[Xi]==255  && i>=img_top+20)
    	if(img[(i-1)*160+(Firmid-1)]==0) //不是通路 
    		;
    	else
    		tenflag++;
	
	//非十字路口补线 （弯道补线） 
	else  
	{
		if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];

					
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	 
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
		
			
	}
    		
 }
 //////////////////////////////////////////赛道左右边线数组处理以及画中线 ///////////////////////////////////////
 
 	/////////////////十字路口丢失赛道///////////// 
 	uint8 n;
	uint8 LeftZJ=0,RightZJ=0; //出十字的标志 
	
		/////////////十字路口 /////////////////
		
		Fit_Middleline[img_high]=(Right_Black[img_high]+Left_Black[img_high])/2; //最后一行中线 
		
		for(n=img_high-1;n>=1;n--) 
		{
			
		 	if(tenflag>=10) 
		 	{
		 		
		 		
		 		if(!(Left_Black[n]-Left_Black[n+1]>=0 && Left_Black[n]-Left_Black[n+1]<=2) && !LeftZJ) //检测到直角，标记为出十字 
		 			LeftZJ=Left_Black[n+1]; 
									
		 		if (!(Right_Black[n]-Right_Black[n+1]<=0 && Right_Black[n]-Right_Black[n+1]>=-2) && !RightZJ)
	
		 			RightZJ=Right_Black[n+1];
	
				
		 					
		 		if(!LeftZJ && !RightZJ) //非十字区 
					Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
				
				else if(LeftZJ || RightZJ)	//进入十字区，用刚出十字的中线拟合 
					Fit_Middleline[n]=Fit_Middleline[n+1];
					
				
//				if(LeftZJ && RightZJ) //出十字路口 
//					if((Left_Black[n]>LeftZJ) && (Right_Black[n]<RightZJ) )
//					{
//						int Midd=(Right_Black[n]+Left_Black[n])/2; //当前行的拟合中线  差值在宽度以内 
//						if(Midd-Fit_Middleline[n+1]<=10 && Midd-Fit_Middleline[n+1]>=-10 && Midd<=Img_Col && Midd>=0 )
//							Fit_Middleline[n]=Midd;		
//					}
										
			}
			/////////////弯道（已经补线）和直道 ///////////////////
			else 
			{
                              
				//急弯打死 
                                
                if(Llost==0 && Rlost>=7 ) //右丢线，在入弯打死 
                    return 4;
                if(Llost>=7 && Rlost==0 ) //左丢线，在入弯打死  
                    return 3;
                           		
				int Midd=(Right_Black[n]+Left_Black[n])/2; //当前行的拟合中线  差值在宽度以内 
				if(Midd-Fit_Middleline[n+1]<=6 && Midd-Fit_Middleline[n+1]>=-6 && Midd<=Img_Col && Midd>=0 )
					Fit_Middleline[n]=Midd;							
				else			
					Fit_Middleline[n]=0;				
				
		
		}
	# if plotmid==1 	
	if(Fit_Middleline[n]!=0 && plotmid==1)  //画中线 
		img[((n+img_top-1)-1)*160+(Fit_Middleline[n]-1)]=0;	
	#endif			
		}
 		
	 

	
//	for(n=img_high;n>=1;n--) 
//		printf("%d:%d,%d\n",n,Left_Black[n],Right_Black[n]);
	
	
	  return 8;
}
 


//舵机控制
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   int Nozero=0;
   
   for(i=img_high;i>=1;i--)  //取有效行的平均值
   {
   	if(Fit_Middleline[i]!=0)
   	{
   		
   		SteerSum=SteerSum+(Fit_Middleline[i]-Img_Col/2);
   		Nozero++;
   		
   		
	}
   } 
   
   Error=-(int)(SteerSum/Nozero);

  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //限定舵机打角范围，防止超过两个轮子所能转过的范围
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if(Error==0)    //偏差太小就不改变舵机角度
     Servo_PWM=servo_pwm_middle;    //使用原来舵机的值 
   
  
   
  return Servo_PWM;

}


