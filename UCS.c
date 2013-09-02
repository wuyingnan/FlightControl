#include "msp430x54x.h"

//初始化UCS
//ACLK=TX1=32,768KHz,MCLK=SMCLK=XT2=16MHz
void UCS_init(void)
{
	P7SEL |= 0x03;                            // 选择 XT1 = 32.768KHz
	P5SEL |= 0x0C;                            // 选择 XT2 = 16MHz
	UCSCTL6 &= ~(XT1OFF + XT2OFF);            // XT1 & XT2 开
	// Loop until XT1,XT2 & DCO stabilizes
	do
	{
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);// Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG;                    // Clear fault flags
	}while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	UCSCTL6 &= ~XT2DRIVE_2;                    // XT2 供电选择
	UCSCTL4 |= SELA_0 + SELM_5  + SELS_5;      // ACLK=TX1,MCLK=XT2,SMCLK=XT2
	UCSCTL5 |= DIVS__4;						   //SMCLK四分频 
}
//输出时钟
//P11.0|--> ACLK
//P11.2|--> SMCLK
//P11.1|--> MCLK
void CLK_output(void)
{
	P11DIR = BIT2 + BIT1 + BIT0;				// P11.2,1,0 输出模式
	P11SEL = BIT2 + BIT1 + BIT0;				// P11.2输出SMCLK,P11.1输出MCLK,P11.0输出ACLK
}
