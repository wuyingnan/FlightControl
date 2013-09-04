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
signed int throttle=400;

unsigned char datacnt=0;
unsigned char sum=0;

void FC_init()
{
  pitchPID.myOutput = 0;
  pitchPID.myInput = &pitchLong;
  pitchPID.mySetpoint = 0;
  pitchPID.inAuto = 1;
  PID_setOutputLimits(&pitchPID, (signed long)-throttle*100/2, (signed long)throttle*100/2);
  pitchPID.SampleTime = 10;
  PID_setControllerDirection(&pitchPID, 0);
  PID_setTunings(&pitchPID, 750, 100, 0);
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
  PID_setOutputLimits(&yawPID, (signed long)-200, (signed long)200);
  yawPID.SampleTime = 10;
  PID_setControllerDirection(&yawPID, 0);
  PID_setTunings(&yawPID, 10, 40, 0);
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

void FC_motorFilter(unsigned long l1, unsigned long r1,unsigned long r2,unsigned long l2)
{
  static signed long lastL1,lastR1,lastR2,lastL2;
  static signed long filteredL1,filteredR1,filteredR2,filteredL2;
  if (l1-lastL1>0)
    filteredL1=(l1+lastL1)/2;
  else
    filteredL1=lastL1-(lastL1-l1)*2;
  if (l2-lastL2>0)
    filteredL2=(l2+lastL2)/2;
  else
    filteredL2=lastL2-(lastL2-l2)*2;
  if (r1-lastR1>0)
    filteredR1=(r1+lastR1)/2;
  else
    filteredR1=lastR1-(lastR1-r1)*2;
  if (r2-lastR2>0)
    filteredR2=(r2+lastR2)/2;
  else
    filteredR2=lastR2-(lastR2-r2)*2;
  if (filteredL1<0)
    filteredL1=0;
  if (filteredL2<0)
    filteredL2=0;
  if (filteredR1<0)
    filteredR1=0;
  if (filteredR2<0)
    filteredR2=0;  
  PWM_1(filteredL1);
  PWM_2(filteredR1);
  PWM_3(filteredR2);
  PWM_4(filteredL2);  
  lastL1=filteredL1;
  lastR1=filteredR1;  
  lastL2=filteredL2;  
  lastR2=filteredR2;  
  UART_sendint(UCA1,filteredL1);
  UART_sendstr(UCA1," ");  
  UART_sendint(UCA1,filteredL2);  
  UART_sendstr(UCA1," ");    
}

void FC_control()
{
    pos=IMU_getEuler();  
/*    
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
    
    

    UART_sendlong(UCA1,(signed long)(pitchLong)+1000000);
    UART_sendstr(UCA1," ");
    UART_sendlong(UCA1,(signed long)(rollLong)+1000000);
    UART_sendstr(UCA1," ");
    UART_sendlong(UCA1,(signed long)(yawLong)+1000000);
    UART_sendstr(UCA1," ");*/
    
    pitchLong = (signed long)(pos.pitch*5730);
    rollLong  = (signed long)(pos.roll*5730);
    yawLong   = (signed long)(pos.yaw*573);
    
    PID_compute(&pitchPID);
    PID_compute(&rollPID);
    PID_compute(&yawPID);
    
    FC_motorFilter(throttle+rollPID.myOutput/Accuracy,throttle+rollPID.myOutput/Accuracy,throttle-rollPID.myOutput/Accuracy,throttle-rollPID.myOutput/Accuracy);
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

void FC_changePitchPID(signed long kp,signed long ki,signed long kd)
{
  PID_setTunings(&pitchPID, kp, ki, kd);
}