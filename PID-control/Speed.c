
#include "include.h"
#include "GLOBAL.h"
#include "calculation.h"
/*
占空比20%  50
30%      83
34%     100
36%     103
40%      116
46%      133
*/
/************变量定义***************/
u8 ZHIDAO_speed=100;                  //直道速度;
u8 H_WAN_speed=100;                   //缓弯速度     
u8 J_WAN_speed=100;                   //急弯速度

u32 ZHIDAO_speed1=4300;                  //直道速度;
u32 H_WAN_speed1=3600;                   //缓弯速度     
u32 J_WAN_speed1=4100;                   //急弯速度


int Open_Robot=1;                               
u32 Ideal_speedL=4100;                     
u32 Ideal_speedR=4100;

int Last_Centererror[3];

int  Left_err[3]={0};	          //左电机速度偏差
int  Right_err[3]={0};	          //右电机速度偏差
u16 l_pulse[3]={0};                   //左编码器采样值
u16 r_pulse[3]={0};                   //右编码器采样值 
extern int Centererror;
extern u8 Load_type;                    //赛道类型
extern u8 last_Load_type;
extern u8 flag_chuwan;
// 出弯标志位        

u32 Target_speed=100; 
int AutopulseL=100;
int AutopulseR=100;
float  Motor_kp=40;                    //电机PID参数值
float  Motor_ki=6.8;  
float Motor_kd=3.8;



//float Z1=1260,W1=40,Z2=1400,W2=50,W3=50;       //  W1最低弯道速度 W2为最大弯道速度  
void Speed_init(void)
{     
      gpio_init(PORTA,4,GPO,LOW);         //电机使能
      gpio_init(PORTA,5,GPO,LOW); 
      gpio_init(PORTA,6,GPO,LOW); 
      gpio_init(PORTA,7,GPO,LOW);
      
      FTM_PWM_init(FTM0,CH1,10000,0);     //PWM1 接PTA6              
      FTM_PWM_init(FTM0,CH3,10000,0);     //PWM2 接PTA4     
      FTM_PWM_init(FTM0,CH4,10000,0);     //PWM3 接PTA7                
      FTM_PWM_init(FTM0,CH2,10000,0);     //PWM4 接PTA5         
}
void Speed_Control(void)                 //开环速度控制       
{
      //判断赛道类型
      switch(Load_type)
      {
        case 1:Target_speed=ZHIDAO_speed1;     break;         //直线道路，最大速度
        case 2:Target_speed=H_WAN_speed1;      break;         //缓弯，速度小减        
        case 3:Target_speed=J_WAN_speed1;      break;         //急弯，速度大减
        default:break;
      }              
      
      Ideal_speedL=Target_speed;
      Ideal_speedR=Target_speed;
      if(Open_Robot==1)
      {  
      
          FTM_PWM_Duty(FTM0,CH3,Ideal_speedL);    //PWM2 接PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);               //PWM1 接PTA4        
        
          FTM_PWM_Duty(FTM0,CH4,Ideal_speedR);    //PWM3 接PTA5              
          FTM_PWM_Duty(FTM0,CH2,0);                         
      }
      else
        { 
          FTM_PWM_Duty(FTM0,CH3,0);             //PWM1 接PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);             //PWM2 接PTA4          
        
          FTM_PWM_Duty(FTM0,CH4,0);             //PWM3 接PTA7              
          FTM_PWM_Duty(FTM0,CH2,0);             //PWM4 接PTA5                   
        }     
}

/**************增量式速度PID控制（**************************/
void Speed_Control2(void)
{
      /*    判断赛道类型   */
      if(Load_type==1)                    //直道，高速
      {   AutopulseL=ZHIDAO_speed; 
          AutopulseR=ZHIDAO_speed;
          if((ABS1(Last_Centererror[2])>=ABS1(Last_Centererror[1]))&&((ABS1(Last_Centererror[1])>=ABS1(Last_Centererror[0])))&&((ABS1(Last_Centererror[0])>=Centererror)))
          {
              Load_type=2;
              flag_chuwan=2;             
          }
           if((ABS1(Last_Centererror[2])<=ABS1(Last_Centererror[1]))&&((ABS1(Last_Centererror[1])<=ABS1(Last_Centererror[0])))&&((ABS1(Last_Centererror[0])<=Centererror)))
          {
              Load_type=2;
              flag_chuwan=0;             
          }
      }                
      //减速进弯
      else if(Load_type==2)    //          
      {           
         if(flag_chuwan==2)               //出弯 
         {
              if(Centererror>0)    
              {    AutopulseL=150; 
                   AutopulseR=90;
                   if(ABS1(Centererror)<=15)
                   {
                   AutopulseL=95; 
                   AutopulseR=50;
                   }
              }  
              else
              {    AutopulseL=85; 
                   AutopulseR=150;
                   if(ABS1(Centererror)<=15)
                   {
                   AutopulseL=50; 
                   AutopulseR=93;
                   }
              }   }      
         else                             //进弯 
         {
               if(Centererror>0)    
               {    AutopulseL=120; 
                    AutopulseR=80;  }  
               else
               {    AutopulseL=80; 
                    AutopulseR=120; }          
         }
      }      
      
//
//      
      else if(Load_type==3)              
      {           
         if(flag_chuwan==3)               //出弯 
         {
              if(Centererror>0)    
              {    AutopulseL=130; 
                   AutopulseR=85; 
                  if(ABS1(Centererror)>=26)//比较急的弯，减小出弯速度并让其内切
                    {
                      AutopulseL=112; 
                      AutopulseR=72;
                    }             
              }  
              else
              {    AutopulseL=85; 
                   AutopulseR=130;
                   if(ABS1(Centererror)>=26)
                    {
                      AutopulseL=72; 
                      AutopulseR=112;
                     }             
              }

          }
          else     //进弯 
          {
               if(Centererror>0)    
               {    AutopulseL=120; 
                    AutopulseR=70;    //比较急的弯，减小入弯速度并让其内切
                   if(ABS1(Centererror)>=26)
                    {
                      AutopulseL=110; 
                      AutopulseR=60;
                     }                 
               }  
               else
               {    AutopulseL=70; 
                    AutopulseR=120;   //比较急的弯，减小弯速度并让其内切
                   if(ABS1(Centererror)>=26)
                    {
                      AutopulseL=70; //入弯速度要比出弯速度低些，稳定，内切
                      AutopulseR=110;
                     }                
               }          
          }
      }            
      else  if(Load_type==4)                            //最急弯                       
      {
          if(Centererror>0)    
          {    AutopulseL=90; 
               AutopulseR=50;  }  
          else
          {    AutopulseL=50; 
               AutopulseR=90; }       
      }                        
     // AutopulseL=80;
     // AutopulseR=80;
      if(Open_Robot==1)
      {          
          Left_err[2]=Left_err[1];
          Left_err[1]=Left_err[0];  
          Left_err[0]=AutopulseL-l_pulse[0];  
          Ideal_speedL=(u16)((Motor_kp*(Left_err[0]-Left_err[1]))+(Motor_ki*Left_err[0])+(Motor_kd*(Left_err[0]-2*Left_err[1]+Left_err[2]))+Ideal_speedL);
          
          Right_err[2]=Right_err[1];
          Right_err[1]=Right_err[0]; 
          Right_err[0]=AutopulseR-r_pulse[0];      
          Ideal_speedR=(u16)((Motor_kp*(Right_err[0]-Right_err[1]))+(Motor_ki*Right_err[0])+(Motor_kd*(Right_err[0]-2*Right_err[1]+Right_err[2]))+Ideal_speedR);                   
       
       
          
         // 鲁棒控制，速度偏差太大，以最大速度加速    
          if(Left_err[0]>=12)
          {
            Ideal_speedL=8000;                  
          }
          if(Right_err[0]>=12)
          {
            Ideal_speedR=8000;                  
          }
          
          if(Ideal_speedL>=8000)
            Ideal_speedL=8000;        
          if(Ideal_speedR>=8000) 
             Ideal_speedR=8000; 
          
             
          FTM_PWM_Duty(FTM0,CH3,Ideal_speedL);    //PWM2 接PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);               //PWM1 接PTA4        
        
          FTM_PWM_Duty(FTM0,CH4,Ideal_speedR);    //PWM3 接PTA5              
          FTM_PWM_Duty(FTM0,CH2,0);              //PWM3 接PTA7               }
           
      }
     else
     {
              FTM_PWM_Duty(FTM0,CH3,0);           //PWM1 接PTA6        
              FTM_PWM_Duty(FTM0,CH1,0);           //PWM2 接PTA4          
            
              FTM_PWM_Duty(FTM0,CH4,0);           //PWM3 接PTA7              
              FTM_PWM_Duty(FTM0,CH2,0);           //PWM4 接PTA5
                         LCD_show();

              
     }    

} 
/*
void Speed_Control3()
{
   if(Load_type==1)
   {
          AutopulseL=ZHIDAO_speed; 
          AutopulseR=ZHIDAO_speed;     
   }
   if()
   {
   
   }
     
}*/