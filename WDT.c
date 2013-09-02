#include"msp430f5438.h"
#include"Global.h"

//WDT_PWM主要为舵机使用
//如果使用WDT_PWM，一个最小WDT溢出周期为2ms/125=16us
//一个PWM周期为2ms,占空比用0~125表示，0表示0%,125表示100%
#ifdef WDT_PWM_Used_
#define WDT_CYCLE 125
  unsigned char WDT_cnt   = 0;
  unsigned char WDT_duty1 = 0;
#endif

void WDT_init()
{
#ifdef WDT_PWM_Used
  WDTCTL = WDTPW + WDTSSEL__SMCLK + WDTTMSEL + WDTIS_7;
  SFRIE1  |= WDTIE;
  
  P8DIR |= BIT7;
  P8OUT |= BIT7;
  P8DS  |= BIT7;
  
  //更多PWM口定义...
  
#else
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
#endif
}

#ifdef WDT_PWM_Used
void WDT_PWM_1(unsigned int n)
{
  WDT_duty1 = n;
}


//void WDT_PWM_2(unsigned int n)
//{
//  WDT_duty2 = n;
//}

#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR()
{
  WDT_cnt++;
  if (WDT_cnt == WDT_duty1)
  {
    P8OUT &=~BIT7;
  }
  
//  if (WDT_cnt == WDT_duty2)
//  {
//    P8OUT &=~BIT6;
//  }  
  //更多定义
  if (WDT_cnt == WDT_CYCLE)
  {
    WDT_cnt =   0;
    P8OUT   |=  BIT7;
  }
}
#endif