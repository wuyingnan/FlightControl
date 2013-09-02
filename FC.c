#include <in430.h>
#include "msp430_math.h"
#include "TimerA1.h"
#include "PWM.h"
#include "IMU.h"
#include "UART.h"
#include "PID.h"

Euler_struct pos;
signed long pitchLong,rollLong,yawLong;
signed int  pitchInt,rollInt,yawInt;
PID_struct pitchPID,rollPID,yawPID;
signed int throttle=0;

unsigned char datacnt=0;
unsigned char sum=0;

void FC_init()
{
  pitchPID.myOutput = 0;
  pitchPID.myInput = &pitchLong;
  pitchPID.mySetpoint = 0;
  pitchPID.inAuto = 1;
  PID_setOutputLimits(&pitchPID, (signed long)0, (signed long)1000);
  pitchPID.SampleTime = 5;
  PID_setControllerDirection(&pitchPID, 0);
  PID_setTunings(&pitchPID, 80, 20, 20);
  if (TimeBase>pitchPID.SampleTime)
    pitchPID.lastTime = TimeBase-pitchPID.SampleTime;
  else
    pitchPID.lastTime = 0;
  
  rollPID=pitchPID;
  rollPID.myInput = &rollLong;
  
  
  yawPID.myOutput = 0;
  yawPID.myInput = &yawLong;
  yawPID.mySetpoint = 0;
  yawPID.inAuto = 1;
  PID_setOutputLimits(&yawPID, (signed long)0, (signed long)800);
  yawPID.SampleTime = 5;
  PID_setControllerDirection(&yawPID, 0);
  PID_setTunings(&yawPID, 80, 20, 20);//kp2100,ki4000
  if (TimeBase>yawPID.SampleTime)
    yawPID.lastTime = TimeBase-yawPID.SampleTime;
  else
    yawPID.lastTime = 0;
}

void FC_send(unsigned char data)
{
  datacnt++;
  sum+=data;
  while(UCA1_putchar(data));
  if (datacnt==31)
  {
    while(UCA1_putchar(sum));
    sum=0;
    datacnt=0;
  }
}

void FC_control()
{
    pos=IMU_getEuler();  
    
    FC_send(0x88);
    FC_send(0xaf);
    FC_send(0x1c);
    FC_send(BYTE1(ax_filtered));
    FC_send(BYTE0(ax_filtered));    
    FC_send(BYTE1(ay_filtered));
    FC_send(BYTE0(ay_filtered));    
    FC_send(BYTE1(az_filtered));
    FC_send(BYTE0(az_filtered));        
    FC_send(BYTE1(gx_filtered));
    FC_send(BYTE0(gx_filtered));    
    FC_send(BYTE1(gy_filtered));
    FC_send(BYTE0(gy_filtered));    
    FC_send(BYTE1(gz_filtered));
    FC_send(BYTE0(gz_filtered));            
    FC_send(0);
    FC_send(0);
    FC_send(0);    
    FC_send(0);
    FC_send(0);
    FC_send(0);
    
    pitchInt = (signed int)(pos.pitch*5730);
    rollInt  = (signed int)(pos.roll*5730);
    yawInt   = (signed int)(pos.yaw*573);
//    pitchLong = (signed long)(pos.pitch*5730);
//    rollLong  = (signed long)(pos.roll*5730);
//    yawLong   = (signed long)(pos.yaw*5730);    

    FC_send(BYTE1(pitchInt));
    FC_send(BYTE0(pitchInt));    
    FC_send(BYTE1(rollInt));
    FC_send(BYTE0(rollInt));            
    FC_send(BYTE1(yawInt));
    FC_send(BYTE0(yawInt));   
    FC_send(0xA0);    
    FC_send(0);
    FC_send(0);
    FC_send(0);
    
    
/*
    UART_sendlong(UCA1,(signed long)(pitchLong)+1000000);
    UART_sendstr(UCA1," ");
    UART_sendlong(UCA1,(signed long)(rollLong)+1000000);
    UART_sendstr(UCA1," ");
    UART_sendlong(UCA1,(signed long)(yawLong)+1000000);
    UART_sendstr(UCA1," ");*/
    
    PID_compute(&pitchPID);
    PID_compute(&rollPID);
    PID_compute(&yawPID);
    
    PWM_1(throttle+pitchPID.myOutput);
    PWM_2(throttle+pitchPID.myOutput);
    PWM_3(throttle-pitchPID.myOutput);
    PWM_4(throttle-pitchPID.myOutput);
}

void FC_emergencyStop()
{
    PWM_1(0);
    PWM_2(0);
    PWM_3(0);
    PWM_4(0);
    _DINT();
    while(1);
}