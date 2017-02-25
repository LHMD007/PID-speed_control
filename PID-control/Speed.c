
#include "include.h"
#include "GLOBAL.h"
#include "calculation.h"
/*
ռ�ձ�20%  50
30%      83
34%     100
36%     103
40%      116
46%      133
*/
/************��������***************/
u8 ZHIDAO_speed=100;                  //ֱ���ٶ�;
u8 H_WAN_speed=100;                   //�����ٶ�     
u8 J_WAN_speed=100;                   //�����ٶ�

u32 ZHIDAO_speed1=4300;                  //ֱ���ٶ�;
u32 H_WAN_speed1=3600;                   //�����ٶ�     
u32 J_WAN_speed1=4100;                   //�����ٶ�


int Open_Robot=1;                               
u32 Ideal_speedL=4100;                     
u32 Ideal_speedR=4100;

int Last_Centererror[3];

int  Left_err[3]={0};	          //�����ٶ�ƫ��
int  Right_err[3]={0};	          //�ҵ���ٶ�ƫ��
u16 l_pulse[3]={0};                   //�����������ֵ
u16 r_pulse[3]={0};                   //�ұ���������ֵ 
extern int Centererror;
extern u8 Load_type;                    //��������
extern u8 last_Load_type;
extern u8 flag_chuwan;
// �����־λ        

u32 Target_speed=100; 
int AutopulseL=100;
int AutopulseR=100;
float  Motor_kp=40;                    //���PID����ֵ
float  Motor_ki=6.8;  
float Motor_kd=3.8;



//float Z1=1260,W1=40,Z2=1400,W2=50,W3=50;       //  W1�������ٶ� W2Ϊ�������ٶ�  
void Speed_init(void)
{     
      gpio_init(PORTA,4,GPO,LOW);         //���ʹ��
      gpio_init(PORTA,5,GPO,LOW); 
      gpio_init(PORTA,6,GPO,LOW); 
      gpio_init(PORTA,7,GPO,LOW);
      
      FTM_PWM_init(FTM0,CH1,10000,0);     //PWM1 ��PTA6              
      FTM_PWM_init(FTM0,CH3,10000,0);     //PWM2 ��PTA4     
      FTM_PWM_init(FTM0,CH4,10000,0);     //PWM3 ��PTA7                
      FTM_PWM_init(FTM0,CH2,10000,0);     //PWM4 ��PTA5         
}
void Speed_Control(void)                 //�����ٶȿ���       
{
      //�ж���������
      switch(Load_type)
      {
        case 1:Target_speed=ZHIDAO_speed1;     break;         //ֱ�ߵ�·������ٶ�
        case 2:Target_speed=H_WAN_speed1;      break;         //���䣬�ٶ�С��        
        case 3:Target_speed=J_WAN_speed1;      break;         //���䣬�ٶȴ��
        default:break;
      }              
      
      Ideal_speedL=Target_speed;
      Ideal_speedR=Target_speed;
      if(Open_Robot==1)
      {  
      
          FTM_PWM_Duty(FTM0,CH3,Ideal_speedL);    //PWM2 ��PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);               //PWM1 ��PTA4        
        
          FTM_PWM_Duty(FTM0,CH4,Ideal_speedR);    //PWM3 ��PTA5              
          FTM_PWM_Duty(FTM0,CH2,0);                         
      }
      else
        { 
          FTM_PWM_Duty(FTM0,CH3,0);             //PWM1 ��PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);             //PWM2 ��PTA4          
        
          FTM_PWM_Duty(FTM0,CH4,0);             //PWM3 ��PTA7              
          FTM_PWM_Duty(FTM0,CH2,0);             //PWM4 ��PTA5                   
        }     
}

/**************����ʽ�ٶ�PID���ƣ�**************************/
void Speed_Control2(void)
{
      /*    �ж���������   */
      if(Load_type==1)                    //ֱ��������
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
      //���ٽ���
      else if(Load_type==2)    //          
      {           
         if(flag_chuwan==2)               //���� 
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
         else                             //���� 
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
         if(flag_chuwan==3)               //���� 
         {
              if(Centererror>0)    
              {    AutopulseL=130; 
                   AutopulseR=85; 
                  if(ABS1(Centererror)>=26)//�Ƚϼ����䣬��С�����ٶȲ���������
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
          else     //���� 
          {
               if(Centererror>0)    
               {    AutopulseL=120; 
                    AutopulseR=70;    //�Ƚϼ����䣬��С�����ٶȲ���������
                   if(ABS1(Centererror)>=26)
                    {
                      AutopulseL=110; 
                      AutopulseR=60;
                     }                 
               }  
               else
               {    AutopulseL=70; 
                    AutopulseR=120;   //�Ƚϼ����䣬��С���ٶȲ���������
                   if(ABS1(Centererror)>=26)
                    {
                      AutopulseL=70; //�����ٶ�Ҫ�ȳ����ٶȵ�Щ���ȶ�������
                      AutopulseR=110;
                     }                
               }          
          }
      }            
      else  if(Load_type==4)                            //���                       
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
       
       
          
         // ³�����ƣ��ٶ�ƫ��̫��������ٶȼ���    
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
          
             
          FTM_PWM_Duty(FTM0,CH3,Ideal_speedL);    //PWM2 ��PTA6        
          FTM_PWM_Duty(FTM0,CH1,0);               //PWM1 ��PTA4        
        
          FTM_PWM_Duty(FTM0,CH4,Ideal_speedR);    //PWM3 ��PTA5              
          FTM_PWM_Duty(FTM0,CH2,0);              //PWM3 ��PTA7               }
           
      }
     else
     {
              FTM_PWM_Duty(FTM0,CH3,0);           //PWM1 ��PTA6        
              FTM_PWM_Duty(FTM0,CH1,0);           //PWM2 ��PTA4          
            
              FTM_PWM_Duty(FTM0,CH4,0);           //PWM3 ��PTA7              
              FTM_PWM_Duty(FTM0,CH2,0);           //PWM4 ��PTA5
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