#include"PID.h"
#include"TimerA1.h"

unsigned char PID_compute(PID_struct *pid)
{
   if(!pid->inAuto) return 1;
   unsigned long now = TimeBase;
   unsigned long timeChange = (now - pid->lastTime);
   if(timeChange >= pid->SampleTime)
   {
      /*Compute all the working error variables*/
      signed long input = *pid->myInput;
      signed long error = pid->mySetpoint - input;
      pid->ITerm += pid->ki * error / Accuracy;
      if(pid->ITerm > pid->outMax) pid->ITerm= pid->outMax;
      else if(pid->ITerm < pid->outMin) pid->ITerm= pid->outMin;
      signed long dInput = (input - pid->lastInput);
 
      /*Compute PID Output*/
      signed long output = (pid->kp * error)/Accuracy + pid->ITerm- (pid->kd * dInput) / Accuracy;
      
      if(output > pid->outMax) output = pid->outMax;
      else if(output < pid->outMin) output = pid->outMin;
      
      pid->myOutput = output;
	  
      /*Remember some variables for next time*/
      pid->lastInput = input;
      pid->lastTime = now;
      return 0;
   }
   else return 1;
}

void PID_setTunings(PID_struct *pid, unsigned long Kp, unsigned long Ki, unsigned long Kd)
{
   
   double SampleTimeInSec = ((double)pid->SampleTime)/1000;  
   pid->kp = Kp;
   pid->ki = (signed long)(Ki * SampleTimeInSec);
   pid->kd = (signed long)(Kd / SampleTimeInSec);
 
  if(pid->controllerDirection == REVERSE)
   {
      pid->kp = - pid->kp;
      pid->ki = - pid->ki;
      pid->kd = - pid->kd;
   }
}

void PID_setSampleTime(PID_struct *pid, unsigned int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)pid->SampleTime;
      pid->ki = (signed long)(pid->ki*ratio);
      pid->kd = (signed long)(pid->kd/ratio);
      pid->SampleTime = NewSampleTime;
   }
}

void PID_setOutputLimits(PID_struct *pid, signed long Min, signed long Max)
{
   if(Min >= Max) return;
   
   pid->outMin = Min;
   pid->outMax = Max;
 
   if(pid->inAuto)
   {
	   if(pid->myOutput > pid->outMax) pid->myOutput = pid->outMax;
	   else if(pid->myOutput < pid->outMin) pid->myOutput = pid->outMin;
	 
	   if(pid->ITerm > pid->outMax) pid->ITerm= pid->outMax;
	   else if(pid->ITerm < pid->outMin) pid->ITerm= pid->outMin;
   }
}

void PID_setMode(PID_struct *pid, unsigned char Mode)
{
    unsigned char newAuto = (Mode == AUTOMATIC);
    if(newAuto == !pid->inAuto)
    {  /*we just went from manual to auto*/
        PID_init(pid);
    }
    pid->inAuto = newAuto;
}

void PID_init(PID_struct *pid)
{
   pid->ITerm = 0;
   pid->lastInput = *pid->myInput;
   if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
   else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
}

void PID_setControllerDirection(PID_struct *pid, unsigned char Direction)
{
   if(pid->inAuto && Direction !=pid->controllerDirection)
   {
      pid->kp = -pid->kp;
      pid->ki = -pid->ki;
      pid->kd = -pid->kd;
   }   
   pid->controllerDirection = Direction;
}

