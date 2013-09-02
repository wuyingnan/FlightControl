
#include "msp430f5438.h"
#include "msp430_math.h"
#include "Global.h"
#include "UCS.h"
#include "WDT.h"
#include "TimerA1.h"
#include "UART.h"

#include "I2C.h"
#include "IMU.h"

#include "FC.h"

unsigned long nowTime,startTime;
unsigned char command;

void main( void )
{
  WDT_init();
  
  UCS_init();
  
  TimerA1_init();
  
  I2C_init();
  
  UART_init(UCA1,115200);
  
  _EINT();
  
  IMU_init();  
  
  IMU_calibrate();
  
  FC_init();
  
//  UART_sendstr(UCA1,"Ready.");
  
//  while(UCA1_GET_CHAR(&command));
  
  startTime=TimeBase;


  while(1)
  {
    if(nowTime!=TimeBase)
    {
      nowTime=TimeBase;
      
      if(nowTime % 5==0)//&&(nowTime < 5000+startTime))
      {
        if(!IMU_getdata())
          AHRSupdate();        
//        IMU_update();
        
        if (nowTime % 10 == 0)
        {
          FC_control();
          _NOP();          
        }
      }
    }

  }
  
}
