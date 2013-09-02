#include "msp430x54x.h"
#include "Global.h"
#include "UART.h"
#include "FC.h"

#ifdef UCA0_Used_
unsigned char A0_RXBUF[A0_RXBUF_SIZE],A0_TXBUF[A0_TXBUF_SIZE];
unsigned char A0_RXBUF_LEN,A0_TXBUF_LEN;
unsigned char A0_RXBUF_FIFO_FULL,A0_TXBUF_FIFO_FULL;
unsigned char A0_RXBUF_R,A0_RXBUF_W;
unsigned char A0_TXBUF_R,A0_TXBUF_W;
char UCA0_GET_RXBUFLEN(void)
{
	return A0_RXBUF_LEN;
}

char UCA0_GET_TXBUFLEN(void)
{
	return A0_TXBUF_LEN;
}
char UCA0_putchar(unsigned char data)
{
	if (A0_TXBUF_LEN == A0_TXBUF_SIZE)
	{
		A0_TXBUF_FIFO_FULL = 1;
		return 1;
	}
	_DINT();
	if (A0_TXBUF_LEN == 0)
	{
		UCA0IE  |= UCTXIE;
		if (!(UCA0STAT & UCBUSY))
			UCA0IFG |= UCTXIFG;
	}
	A0_TXBUF[A0_TXBUF_W++] = data;
	if (A0_TXBUF_W == A0_TXBUF_SIZE)
		A0_TXBUF_W = 0;
	A0_TXBUF_LEN ++;
	_EINT();
	return 0;
}
char UCA0_GET_CHAR(unsigned char *ptr)
{
	if (A0_RXBUF_LEN == 0)
		return 1;
	_DINT();
	A0_RXBUF_LEN --;
	*ptr = A0_RXBUF[A0_RXBUF_R++];
	if (A0_RXBUF_R == A0_RXBUF_SIZE)
		A0_RXBUF_R = 0;
	_EINT();
	return 0;
}
#pragma vector=USCI_A0_VECTOR
__interrupt void UART0(void)
{
	switch (UCA0IV)
	{
	case 0:
		break;
	case 2:
		if (A0_RXBUF_LEN == A0_RXBUF_SIZE)
		{
			UCA0IFG &= ~UCRXIFG;		//Clear RX interrupt flag
			A0_RXBUF_FIFO_FULL = 1;		//Set FIFO full
			break;
		}
                A0_RXBUF[A0_RXBUF_W++] = UCA0RXBUF;	//Get data from UCA0RXBUF
                if (A0_RXBUF_W == A0_RXBUF_SIZE)
                    A0_RXBUF_W = 0;
                A0_RXBUF_LEN++;
		break;
	case 4:
		if (A0_TXBUF_LEN > 0)
		{
			A0_TXBUF_LEN--;
			UCA0TXBUF = A0_TXBUF[A0_TXBUF_R++];	//Send data
			if (A0_TXBUF_R == A0_TXBUF_SIZE)
				A0_TXBUF_R = 0;
		}
		else UCA0IE &= ~UCTXIE;				//Turn off transmit
		break;
	case 8:
		break;
	}
}
#endif

#ifdef UCA1_Used_
unsigned char A1_RXBUF[A1_RXBUF_SIZE],A1_TXBUF[A1_TXBUF_SIZE];
unsigned char A1_RXBUF_LEN,A1_TXBUF_LEN;
unsigned char A1_RXBUF_FIFO_FULL,A1_TXBUF_FIFO_FULL;
unsigned char A1_RXBUF_R,A1_RXBUF_W;
unsigned char A1_TXBUF_R,A1_TXBUF_W;
char UCA1_GET_RXBUFLEN(void)
{
	return A1_RXBUF_LEN;
}

char UCA1_GET_TXBUFLEN(void)
{
	return A1_TXBUF_LEN;
}
char UCA1_putchar(unsigned char data)
{
	if (A1_TXBUF_LEN == A1_TXBUF_SIZE)
	{
		A1_TXBUF_FIFO_FULL = 1;
		return 1;
	}
	_DINT();
	if (A1_TXBUF_LEN == 0)
	{
		UCA1IE  |= UCTXIE;
		if (!(UCA1STAT & UCBUSY))
			UCA1IFG |= UCTXIFG;
	}
	A1_TXBUF[A1_TXBUF_W++] = data;
	if (A1_TXBUF_W == A1_TXBUF_SIZE)
		A1_TXBUF_W = 0;
	A1_TXBUF_LEN ++;
	_EINT();
	return 0;
}
char UCA1_GET_CHAR(unsigned char *ptr)
{
	if (A1_RXBUF_LEN == 0)
		return 1;
	_DINT();
	A1_RXBUF_LEN --;
	*ptr = A1_RXBUF[A1_RXBUF_R++];
	if (A1_RXBUF_R == A1_RXBUF_SIZE)
		A1_RXBUF_R = 0;
	_EINT();
	return 0;
}
#pragma vector=USCI_A1_VECTOR
__interrupt void UART1(void)
{
	switch (UCA1IV)
	{
	case 0:
		break;
	case 2:
		if (A1_RXBUF_LEN == A1_RXBUF_SIZE)
		{
			UCA1IFG &= ~UCRXIFG;		//Clear RX interrupt flag
			A1_RXBUF_FIFO_FULL = 1;		//Set FIFO full
			break;
		}
                A1_RXBUF[A1_RXBUF_W++] = UCA1RXBUF;	//Get data from UCA0RXBUF
                
                if(A1_RXBUF[A1_RXBUF_W-1]=='s')         //StopMotor in the interrupt
                  FC_emergencyStop();
                
                if (A1_RXBUF_W == A1_RXBUF_SIZE)
                    A1_RXBUF_W = 0;
                A1_RXBUF_LEN++;
		break;
	case 4:
		if (A1_TXBUF_LEN > 0)
		{
			A1_TXBUF_LEN--;
			UCA1TXBUF = A1_TXBUF[A1_TXBUF_R++];	//Send data
			if (A1_TXBUF_R == A1_TXBUF_SIZE)
				A1_TXBUF_R = 0;
		}
		else UCA1IE &= ~UCTXIE;				//Turn off transmit
		break;
	case 8:
		break;
	}
}
#endif

#ifdef UCA2_Used_
unsigned char A2_RXBUF[A2_RXBUF_SIZE],A2_TXBUF[A2_TXBUF_SIZE];
unsigned char A2_RXBUF_LEN,A2_TXBUF_LEN;
unsigned char A2_RXBUF_FIFO_FULL,A2_TXBUF_FIFO_FULL;
unsigned char A2_RXBUF_R,A2_RXBUF_W;
unsigned char A2_TXBUF_R,A2_TXBUF_W;
char UCA2_GET_RXBUFLEN(void)
{
	return A2_RXBUF_LEN;
}

char UCA2_GET_TXBUFLEN(void)
{
	return A2_TXBUF_LEN;
}
char UCA2_putchar(unsigned char data)
{
	if (A2_TXBUF_LEN == A2_TXBUF_SIZE)
	{
		A2_TXBUF_FIFO_FULL = 1;
		return 1;
	}
	_DINT();
	if (A2_TXBUF_LEN == 0)
	{
		UCA2IE  |= UCTXIE;
		if (!(UCA2STAT & UCBUSY))
			UCA2IFG |= UCTXIFG;
	}
	A2_TXBUF[A2_TXBUF_W++] = data;
	if (A2_TXBUF_W == A2_TXBUF_SIZE)
		A2_TXBUF_W = 0;
	A2_TXBUF_LEN ++;
	_EINT();
	return 0;
}
char UCA2_GET_CHAR(unsigned char *ptr)
{
	if (A2_RXBUF_LEN == 0)
		return 1;
	_DINT();
	A2_RXBUF_LEN --;
	*ptr = A2_RXBUF[A2_RXBUF_R++];
	if (A2_RXBUF_R == A2_RXBUF_SIZE)
		A2_RXBUF_R = 0;
	_EINT();
	return 0;
}
#pragma vector=USCI_A2_VECTOR
__interrupt void UART2(void)
{
	switch (UCA2IV)
	{
	case 0:
		break;
	case 2:
		if (A2_RXBUF_LEN == A2_RXBUF_SIZE)
		{
			UCA2IFG &= ~UCRXIFG;		//Clear RX interrupt flag
			A2_RXBUF_FIFO_FULL = 1;		//Set FIFO full
			break;
		}
                A2_RXBUF[A2_RXBUF_W++] = UCA2RXBUF;	//Get data from UCA0RXBUF
                if (A2_RXBUF_W == A2_RXBUF_SIZE)
                    A2_RXBUF_W = 0;
                A2_RXBUF_LEN++;
		break;
	case 4:
		if (A2_TXBUF_LEN > 0)
		{
			A2_TXBUF_LEN--;
			UCA2TXBUF = A2_TXBUF[A2_TXBUF_R++];	//Send data
			if (A2_TXBUF_R == A2_TXBUF_SIZE)
				A2_TXBUF_R = 0;
		}
		else UCA2IE &= ~UCTXIE;				//Turn off transmit
		break;
	case 8:
		break;
	}
}
#endif

#ifdef UCA3_Used_
unsigned char A3_RXBUF[A3_RXBUF_SIZE],A3_TXBUF[A3_TXBUF_SIZE];
unsigned char A3_RXBUF_LEN,A3_TXBUF_LEN;
unsigned char A3_RXBUF_FIFO_FULL,A3_TXBUF_FIFO_FULL;
unsigned char A3_RXBUF_R,A3_RXBUF_W;
unsigned char A3_TXBUF_R,A3_TXBUF_W;
char UCA3_GET_RXBUFLEN(void)
{
	return A3_RXBUF_LEN;
}

char UCA3_GET_TXBUFLEN(void)
{
	return A3_TXBUF_LEN;
}
char UCA3_putchar(unsigned char data)
{
	if (A3_TXBUF_LEN == A3_TXBUF_SIZE)
	{
		A3_TXBUF_FIFO_FULL = 1;
		return 1;
	}
	_DINT();
	if (A3_TXBUF_LEN == 0)
	{
		UCA3IE  |= UCTXIE;
		if (!(UCA3STAT & UCBUSY))
			UCA3IFG |= UCTXIFG;
	}
	A3_TXBUF[A3_TXBUF_W++] = data;
	if (A3_TXBUF_W == A3_TXBUF_SIZE)
		A3_TXBUF_W = 0;
	A3_TXBUF_LEN ++;
	_EINT();
	return 0;
}
char UCA3_GET_CHAR(unsigned char *ptr)
{
	if (A3_RXBUF_LEN == 0)
		return 1;
	_DINT();
	A3_RXBUF_LEN --;
	*ptr = A3_RXBUF[A3_RXBUF_R++];
	if (A3_RXBUF_R == A3_RXBUF_SIZE)
		A3_RXBUF_R = 0;
	_EINT();
	return 0;
}
#pragma vector=USCI_A3_VECTOR
__interrupt void UART3(void)
{
	switch (UCA3IV)
	{
	case 0:
		break;
	case 2:
		if (A3_RXBUF_LEN == A3_RXBUF_SIZE)
		{
			UCA3IFG &= ~UCRXIFG;		//Clear RX interrupt flag
			A3_RXBUF_FIFO_FULL = 1;		//Set FIFO full
			break;
		}
                A3_RXBUF[A3_RXBUF_W++] = UCA3RXBUF;	//Get data from UCA0RXBUF
                if (A3_RXBUF_W == A3_RXBUF_SIZE)
                    A3_RXBUF_W = 0;
                A3_RXBUF_LEN++;
		break;
	case 4:
		if (A3_TXBUF_LEN > 0)
		{
			A3_TXBUF_LEN--;
			UCA3TXBUF = A3_TXBUF[A3_TXBUF_R++];	//Send data
			if (A3_TXBUF_R == A3_TXBUF_SIZE)
				A3_TXBUF_R = 0;
		}
		else UCA3IE &= ~UCTXIE;				//Turn off transmit
		break;
	case 8:
		break;
	}
}
#endif

void UART_init(unsigned char UCAx,unsigned long int baud)
{
	switch(UCAx)
	{
	case UCA0:
	{
          #ifdef UCA0_Used_
          P3SEL |= BIT4+BIT5;
	  UCA0CTL1 |= UCSWRST;
	  UCA0CTL1 |= UCSSEL_2;
	  if(baud==9600)
          {
              UCA0BR0 = 0x1a;                           // 
              UCA0BR1 = 0x00;                           //
              UCA0MCTL = UCBRS_0+UCBRF_1+UCOS16;          
          }   
          if(baud==115200)
          {
              UCA0BR0 = 0x22;                           // 32k/9600 - 3.41
              UCA0BR1 = 0x00;                           //
              UCA0MCTL = UCBRS_6+UCBRF_0;
          }
          UCA0CTL1 &= ~UCSWRST;
          UCA0IE |= UCTXIE + UCRXIE;
          #else
          break;
          #endif
	} break;
	case UCA1:
          {
            #ifdef UCA1_Used_
            P5SEL |= BIT6+BIT7;
	    UCA1CTL1 |= UCSWRST;
	    UCA1CTL1 |= UCSSEL_2;
	    if(baud==9600)
	    {
		UCA1BR0 = 0x1a;                           // 
		UCA1BR1 = 0x00;                           //
		UCA1MCTL = UCBRS_0+UCBRF_1+UCOS16;          
	    }   
	    if(baud==115200)
	    {
		UCA1BR0 = 0x22;                           // 32k/9600 - 3.41
		UCA1BR1 = 0x00;                           //
		UCA1MCTL = UCBRS_6+UCBRF_0;
	    }
	    UCA1CTL1 &= ~UCSWRST;
            UCA1IE |= UCTXIE + UCRXIE;
            #else
            break;
            #endif
	} break;
	case UCA2:
          {
            #ifdef UCA2_Used_
            P9SEL |= BIT4+BIT5;
	    UCA2CTL1 |= UCSWRST;
	    UCA2CTL1 |= UCSSEL_2;
	    if(baud==9600)
	    {
		UCA2BR0 = 0x1a;                           // 
		UCA2BR1 = 0x00;                           //
		UCA2MCTL = UCBRS_0+UCBRF_1+UCOS16;          
	    }   
	    if(baud==115200)
	    {
		UCA2BR0 = 0x22;                           // 32k/9600 - 3.41
		UCA2BR1 = 0x00;                           //
		UCA2MCTL = UCBRS_6+UCBRF_0;
	    }
	    UCA2CTL1 &= ~UCSWRST;
            UCA2IE |= UCTXIE + UCRXIE;
            #else
            break;
            #endif
	} break;
	case UCA3:
          {
            #ifdef UCA3_Used_
            P10SEL |= BIT4+BIT5;
	    UCA3CTL1 |= UCSWRST;
	    UCA3CTL1 |= UCSSEL_2;
	    if(baud==9600)
	    {
		UCA3BR0 = 0x1a;                           // 
		UCA3BR1 = 0x00;                           //
		UCA3MCTL = UCBRS_0+UCBRF_1+UCOS16;          
	    }   
	    if(baud==115200)
	    {
		UCA3BR0 = 0x22;                           // 32k/9600 - 3.41
		UCA3BR1 = 0x00;                           //
		UCA3MCTL = UCBRS_6+UCBRF_0;
	    }
	    UCA3CTL1 &= ~UCSWRST;
            UCA3IE |= UCTXIE + UCRXIE;
            #else
            break;
            #endif
	} break;
	default:break;
	}
}

void UART_sendstr(unsigned char UCAx,char *str)
{
	unsigned char i=0;
	switch(UCAx)
	{
	case UCA0:
		{
			#ifdef UCA0_Used_
			while(str[i]!=0)
			{
			while(UCA0_putchar(str[i]));
			i++;
			}
			#else
			break;
			#endif
		}
                 break;
        case UCA1:
		{
			#ifdef UCA1_Used_
			while(str[i]!=0)
			{
			while(UCA1_putchar(str[i]));
			i++;
			}
			#else
			break;
			#endif
		}
                 break;
        case UCA2:
		{
			#ifdef UCA2_Used_
			while(str[i]!=0)
			{
			while(UCA2_putchar(str[i]));
			i++;
			}
			#else
			break;
			#endif
		}
                 break;
        case UCA3:
		{
			#ifdef UCA3_Used_
			while(str[i]!=0)
			{
			while(UCA3_putchar(str[i]));
			i++;
			}
			#else
			break;
			#endif
		}
                 break;
        default:break;
	}
}

void UART_sendint(unsigned char UCAx,unsigned int data)
{
    unsigned char temp[5];
    unsigned char length=0;
    switch(UCAx)
    {
    case UCA0:
        {
            #ifdef UCA0_Used_
            if (data == 0)
              while(UCA0_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA0_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA1:
        {
            #ifdef UCA1_Used_
            if (data == 0)
              while(UCA1_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA1_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA2:
        {
            #ifdef UCA2_Used_
            if (data == 0)
              while(UCA2_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA2_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA3:
        {
            #ifdef UCA3_Used_
            if (data == 0)
              while(UCA3_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA3_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    default:break;
    }
}
void UART_sendlong(unsigned char UCAx,unsigned long data)
{
    unsigned char temp[20];
    unsigned char length=0;
    switch(UCAx)
    {
    case UCA0:
        {
            #ifdef UCA0_Used_
            if (data == 0)
              while(UCA0_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA0_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA1:
        {
            #ifdef UCA1_Used_
            if (data == 0)
              while(UCA1_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA1_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA2:
        {
            #ifdef UCA2_Used_
            if (data == 0)
              while(UCA2_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA2_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA3:
        {
            #ifdef UCA3_Used_
            if (data == 0)
              while(UCA3_putchar('0'));
            else
            {
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA3_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    default:break;
    }
}

                    
//打负号有问题，不要使用
/*                    
void UART_sendlong(unsigned char UCAx,signed long data)
{
    unsigned char temp[16];
    unsigned char length=0;
    switch(UCAx)
    {
    case UCA0:
        {
            #ifdef UCA0_Used_
            if (data == 0)
              while(UCA0_putchar('0'));
            else
            {
              if (data<0)
              {
                while(UCA0_putchar('-'));
                data=-data;
              }
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA0_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA1:
        {
            #ifdef UCA1_Used_
            if (data == 0)
              while(UCA1_putchar('0'));
            else
            {
              if (data<0)
              {
                UART_sendstr(UCA1,"-");              
                data=-data;
              }
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA1_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA2:
        {
            #ifdef UCA2_Used_
            if (data == 0)
              while(UCA2_putchar('0'));
            else
            {
              if (data<0)
              {
                while(UCA2_putchar('-'));              
                data=-data;
              }                
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA2_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    case UCA3:
        {
            #ifdef UCA3_Used_
            if (data == 0)
              while(UCA3_putchar('0'));
            else
            {
              if (data<0)
              {
                while(UCA3_putchar('-'));              
                data=-data;
              }                
              while(data!=0)
              {
                temp[length++]=data%10;
                data/=10;
              }
              for(;length!=0;)
              {
                length--;
                while(UCA3_putchar('0'+temp[length]));
              }
            }
            #else
            break;
            #endif
        }
         break;
    default:break;
    }
}

*/