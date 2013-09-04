
#include "msp430f5438.h"
#include "msp430_math.h"
#include "Global.h"
#include "UCS.h"
#include "WDT.h"
#include "TimerA1.h"
#include "UART.h"

#include "I2C.h"
#include "I2C2.h"
#include "IMU.h"

#include "PWM.h"
#include "FC.h"
#include "MMA8452.h"

unsigned long nowTime,startTime;
unsigned char command;
unsigned char flag=0;

void main( void )
{
  WDT_init();
  
  UCS_init();
  
  TimerA1_init();
  
  I2C_init();
  
  I2C2_init();
  
  UART_init(UCA1,115200);
  
  PWM_init();
  
  _EINT();
  
  IMU_init();  
  
  IMU_calibrate();
  
  FC_init();
  
  UART_sendstr(UCA1,"Ready.");
  
//  WDTCTL = WDTCNTCL+ WDT_ADLY_1000;
  
  while(UCA1_GET_CHAR(&command));
  
  startTime=TimeBase;


  while(1)
  {
//    WDTCTL = WDTCNTCL+ WDT_ADLY_1000;
    
    if(nowTime!=TimeBase)
    {
      nowTime=TimeBase;
      
      if(nowTime % 5==0)//&&(nowTime < 5000+startTime))
      {
        if(!IMU_getdata())
          AHRSupdate();        
//        IMU_update();
        _NOP();
        
        if (nowTime % 10 == 0)
        {
          if(flag) 
            FC_control();
          _NOP();
        }
      } 
    }    
    if(!UCA1_GET_CHAR(&command))
    {
      if (command=='g')
      {
        flag=1;
      }
      if (command=='h')
      {
        PWM_1(0);
        PWM_2(0);
        PWM_3(0);
        PWM_4(0);
        signed long kp=0,ki=0,kd=0;
        while(UCA1_GET_CHAR(&command));        
        while(command!=' ')
        {
          kp=kp*10+command-'0';
          while(UCA1_GET_CHAR(&command));
        }
        while(UCA1_GET_CHAR(&command));
        while(command!=' ')
        {
          ki=ki*10+command-'0';
          while(UCA1_GET_CHAR(&command));
        }      
        while(UCA1_GET_CHAR(&command));
        while(command!=' ')
        {
          kd=kd*10+command-'0';
          while(UCA1_GET_CHAR(&command));
        }              
        FC_changePitchPID(kp,ki,kd);
      }
    }
      
  }
  
}
