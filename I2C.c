#include"msp430f5438.h"
#include"stdint.h"
#include"TimerA1.h"
#include"IMU.h"
#define I2C_RXBUF_SIZE 2
#define I2C_TXBUF_SIZE 2
#define I2CDelayTime 100

volatile unsigned char I2C_RXBUF[I2C_RXBUF_SIZE];
volatile unsigned char I2C_TXBUF[I2C_TXBUF_SIZE];
volatile unsigned char I2C_RXBUF_LEN,I2C_TXBUF_LEN;
volatile unsigned char I2C_TXBUF_R,I2C_TXBUF_W;
volatile unsigned char I2C_TXFIN,I2C_RXFIN,I2C_RXLEN,I2C_RXCNT;
volatile unsigned char* datadd;
volatile unsigned char isReading;

void I2C_init()
{
    P3SEL &= ~(BIT1+BIT2);
    P3DIR |= BIT2+BIT1;    
    P3OUT &= BIT2;
    P3OUT |= BIT1;
    // 输出9个时钟 以恢复I2C总线状态
    for( char i = 0 ; i < 9 ; i++ )
    {
      P3OUT |= BIT2;
      __delay_cycles(8000);
      P3OUT &= ~BIT2;
      __delay_cycles(8000);
    }  
  
    // Configure pins
    P3SEL     |= BIT1 + BIT2;

    UCB0CTL1 |= UCSWRST;	            //Software reset enabled
    UCB0CTL0 |= UCMODE_3  + UCMST + UCSYNC;//I2C mode, Master mode, sync
    UCB0CTL1 |= UCSSEL_2;                  //SMCLK
    UCB0BR0  =  40;                        //4M/100K=40 
    UCB0BR1  =  0;
    UCB0CTL0 &= ~UCSLA10;
    UCB0CTL1  &=~UCSWRST;
    
    UCB0IE    |= UCRXIE + UCTXIE;
}

void I2C_reset(unsigned char slaveadd)
{
    UCB0CTL1 |= UCSWRST;	            //Software reset enabled
    UCB0CTL0 |= UCMODE_3  + UCMST + UCSYNC;//I2C mode, Master mode, sync
    UCB0CTL1 |= UCSSEL_2;                  //SMCLK
    UCB0BR0  =  10;                        //4M/400K=10 
    UCB0BR1  =  0;
    UCB0CTL0 &= ~UCSLA10;
    UCB0I2CSA = slaveadd;    
    UCB0CTL1  &=~UCSWRST;
    UCB0IE    |= UCRXIE + UCTXIE;
}

unsigned char I2C_write(unsigned char slaveadd,unsigned char add,unsigned char data)
{
  static unsigned long I2CPreTime;  
  I2CPreTime = TimeBase;
  while( UCB0CTL1 & UCTXSTP )
  {
    if (I2CPreTime + I2CDelayTime < TimeBase)
    {
      I2C_init();
      UCB0CTL1 &=~UCTXSTP;
      return 1;
    }
  }
  _DINT();
  if (UCB0I2CSA != slaveadd)
    I2C_reset(slaveadd);
  UCB0CTL1 |= UCTR;                 // 写模式
  UCB0CTL1 |= UCTXSTT;              // 发送启动位  
  isReading=0;
  I2C_TXBUF_LEN=2;
  I2C_TXBUF_R=0;
  I2C_TXBUF[0]=add;
  I2C_TXBUF[1]=data;
  UCB0IE  |= UCTXIE;
  I2C_TXFIN=0;
  _EINT();
  while((!I2C_TXFIN) || (UCB0STAT&UCBUSY));
  return 0;
}

unsigned char I2C_read(unsigned char slaveadd,unsigned char regadd,unsigned char len,unsigned char *ramadd)
{
  static unsigned long I2CPreTime;  
  I2CPreTime = TimeBase;
  while( UCB0CTL1 & UCTXSTP )
  {
    if (I2CPreTime + I2CDelayTime < TimeBase)
    {
      I2C_init();
      IMU_init();        
      return 1;
    }
  }
  _DINT();
  if (UCB0I2CSA != slaveadd)
    I2C_reset(slaveadd);
  UCB0CTL1 |= UCTR;                 // 写模式
  UCB0CTL1 |= UCTXSTT;              // 发送启动位  
  isReading=1;
  I2C_TXBUF_LEN=1;
  I2C_TXBUF_R=0;
  I2C_TXBUF[0]=regadd;
  UCB0IE  |= UCTXIE;
  datadd=ramadd;
  I2C_RXFIN=0;
  I2C_RXCNT=0;
  I2C_RXLEN=len;
  _EINT();
  return 0;
}

// USCI_B0 Data ISR
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(UCB0IV)
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4:                                 // Vector  4: NACKIFG
    UCB0IFG &=  ~UCNACKIFG;
    _NOP();
    break;  
  case  6:                                 // Vector  6: STTIFG SLAVE MODE
    break;
  case  8: break;                           // Vector  8: STPIFG SLAVE MODE
  case 10:                                  // Vector 10: RXIFG    
/*    if (I2C_RXBUF_LEN == I2C_RXBUF_MAXLEN-1)
    {
      UCB0CTL1 |= UCTXSTP;              // 在接收最后一个字节之前发送停止位
    }*/
    datadd[I2C_RXCNT++] = UCB0RXBUF;	//Get data from UCB0RXBUF
    if (I2C_RXLEN-I2C_RXCNT==1)
      UCB0CTL1 |= UCTXSTP;    
    if (I2C_RXLEN-I2C_RXCNT==0)
      I2C_RXFIN=1;
    break;
  case 12:                                  // Vector 12: TXIFG
    if (I2C_TXBUF_LEN > 0)
    {
        I2C_TXBUF_LEN--;
        UCB0TXBUF = I2C_TXBUF[I2C_TXBUF_R++];	//Send data
        if (I2C_TXBUF_R == I2C_TXBUF_SIZE)
            I2C_TXBUF_R = 0;
    }
    else 
    {
      if (!isReading)
      {
        UCB0CTL1 |= UCTXSTP;
        I2C_TXFIN=1;
      }
      else
      {
        UCB0CTL1 &= ~UCTR;                // 读模式        
        UCB0CTL1 |= UCTXSTT;        
        while(UCB0CTL1 & UCTXSTT);
        if (I2C_RXLEN==1)
          UCB0CTL1 |= UCTXSTP;
      }
      UCB0IE &= ~UCTXIE;				//Turn off transmit
    }
    break;
  default: break; 
  }
}

/*
void I2C_init()
{
  P3SEL &= ~BIT2;                         // P3.2@UCB0SCL
  P3DIR |= BIT2;
  P3OUT |= BIT2;
  // 输出9个时钟 以恢复I2C总线状态
  for( uint8_t i = 0 ; i < 9 ; i++ )
  {
    P3OUT |= BIT2;
    __delay_cycles(8000);
    P3OUT &= ~BIT2;
    __delay_cycles(8000);
  }
  
  P3SEL |= (BIT1 + BIT2);                 // P3.1@UCB0SDA P3.2@UCB0SCL
  // P3.1@ISP.1 P3.2@ISP.5
  
  UCB0CTL1 |= UCSWRST;
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC ;  // I2C主机模式
  UCB0CTL1 |= UCSSEL_2;                   // 选择SMCLK
  UCB0BR0 = 40;
  UCB0BR1 = 0;
  UCB0CTL0 &= ~UCSLA10;                   // 7位地址模式
  UCB0I2CSA = 0x1e;            // ADXL345
  UCB0CTL1 &= ~UCSWRST;
}

uint8_t eeprom_readbyte( uint8_t word_addr , uint8_t *pword_value )
{
  UCB0CTL1 |= UCTR;                 // 写模式
  UCB0CTL1 |= UCTXSTT;              // 发送启动位和写控制字节

  UCB0TXBUF = word_addr;            // 发送字节地址，必须要先填充TXBUF
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化 等待一个标志位即可
  while(!(UCB0IFG & UCTXIFG))
  {
    if( UCB0IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }                        

  UCB0CTL1 &= ~UCTR;                // 读模式
  UCB0CTL1 |= UCTXSTT;              // 发送启动位和读控制字节

  while(UCB0CTL1 & UCTXSTT);        // 等待UCTXSTT=0
  // 若无应答 UCNACKIFG = 1
  UCB0CTL1 |= UCTXSTP;              // 先发送停止位

  while(!(UCB0IFG & UCRXIFG));      // 读取字节内容
  *pword_value = UCB0RXBUF;         // 读取BUF寄存器在发送停止位之后

  while( UCB0CTL1 & UCTXSTP );
  
  return 0; 
}

uint8_t eeprom_writebyte( uint8_t word_addr , uint8_t word_value )
{
  while( UCB0CTL1 & UCTXSTP );
  UCB0CTL1 |= UCTR;                 // 写模式
  UCB0CTL1 |= UCTXSTT;              // 发送启动位

  UCB0TXBUF = word_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化 等待一个标志位即可
  while(!(UCB0IFG & UCTXIFG))
  {
    if( UCB0IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }   

  UCB0TXBUF = word_value;           // 发送字节内容
  while(!(UCB0IFG & UCTXIFG));      // 等待UCTXIFG=1

  UCB0CTL1 |= UCTXSTP;
  while(UCB0CTL1 & UCTXSTP);        // 等待发送完成
  
  return 0;
}

void I2C_write(unsigned char add,unsigned char data)
{
  eeprom_writebyte(add,data);
}


uint8_t I2C_read(unsigned char add)
{
  uint8_t temp;
  eeprom_readbyte(add,&temp);
  return temp;
}
*/