#include"msp430f5438.h"
#include"Global.h"
//调参数用
#include"UART.h"

volatile unsigned long TimeBase = 0;

void TimerA1_init()
{
    TA1CCTL0 |= CCIE;                 //TA1CCR0 允许中断
    TA1CCR0 = 4000-1;                   //设置延时时间(4MHz/4000=1KHz)
    TA1CTL |= TASSEL_2;               //SMCLK
    TA1CTL |= MC_1;
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1ISR(void)
{
  
#ifdef StepMotor_Used_
  static unsigned char StepLogic[2] = {0,0};
  unsigned char i;
#endif
  
  TimeBase ++;
  
  //下面一段调参数用,之后需要删除
#ifdef PhotoelectricEncoder_Used_  
  extern int tx;
  if(TimeBase % 10 == 0)
  {
    UART_sendint(UCA1, (unsigned int)(L_speed));
    UART_sendstr(UCA1, ", ");
  }
#endif
  
/*数码管扫描程序*/  
#ifdef DigtalTube_Used_
    P7OUT &= ~(BIT4+ BIT5+ BIT6+ BIT7);  
    switch(TimeBase % 4)
    {
    case 0:
      P7OUT |= BIT4;
      P9OUT =  Digtal[Display_Number[1]];
      if (Display_Number[0]==1)
        P9OUT &= ~BIT7;
      break;
    case 1:
      P7OUT |= BIT5;
      P9OUT =  Digtal[Display_Number[2]];
      if (Display_Number[0]==2)
        P9OUT &= ~BIT7;      
      break;
    case 2:
      P7OUT |= BIT6;
      P9OUT =  Digtal[Display_Number[3]];
      if (Display_Number[0]==3)
        P9OUT &= ~BIT7;      
      break;
    case 3:
      P7OUT |= BIT7;
      P9OUT =  Digtal[Display_Number[4]];
      if (Display_Number[0]==4)
        P9OUT &= ~BIT7;      
      break;
    }
#endif    
/*步进电机程序*/    
#ifdef StepMotor_Used_
    if (TimeBase % StepMotor_interval==0)
    {
      P3OUT = 0;
      for ( i=0; i<2; i++)
        if (StepCnt[i] != 0)
        {
            StepCnt[i]--;
            if (StepDir[i] == 0) StepLogic[i] ++;
                    else StepLogic[i] --;
            if (StepLogic[i] > 4) StepLogic[i] = 1;
            if (StepLogic[i] < 1) StepLogic[i] = 4;
            P3OUT |= 1 << (StepLogic[i]-1 + (i << 2));
        }
    }
#endif    
}
