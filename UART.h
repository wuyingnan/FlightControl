/*
每次可能使用时必须先在Global.h中定义UCAx_Used_进行初始化使能
每次使用时必须先调用UCAx_init();
然后才能使用对应的函数
*/
#include "Global.h"
#ifndef UART_H_
#define UART_H_
#define UCA0 0x01 //调用相关
#define UCA1 0x02
#define UCA2 0x04
#define UCA3 0x08
#define A0_RXBUF_SIZE 64  //UCA0 FIFO大小
#define A0_TXBUF_SIZE 64
#define A1_RXBUF_SIZE 64  
#define A1_TXBUF_SIZE 64
#define A2_RXBUF_SIZE 64  
#define A2_TXBUF_SIZE 64
#define A3_RXBUF_SIZE 64  
#define A3_TXBUF_SIZE 64
extern void UART_init(unsigned char UCAx,unsigned long int baud);//初始化UCAx，baud可选9600或115200
extern void UART_sendstr(unsigned char UCAx,char *str);//使用UCAx发送字符串str
extern void UART_sendint(unsigned char UCAx,unsigned int data);//使用UCAx发送unsigned int data(ASCII)
extern void UART_sendlong(unsigned char UCAx,unsigned long data);//使用UCAx发送unsigned long data(ASCII)

#define BYTE0(dwTemp)       (*(unsigned char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((unsigned char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((unsigned char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((unsigned char *)(&dwTemp) + 3))

#endif 
#ifdef UCA0_Used_
char UCA0_GET_RXBUFLEN(void);//获取RX的FIFO中字符长度
char UCA0_GET_TXBUFLEN(void);//获取TX的FIFO中字符长度
char UCA0_putchar(unsigned char data);//写一个字符到TX FIFO
char UCA0_GET_CHAR(unsigned char *ptr);//从RX FIFO中读一个字节
#endif
#ifdef UCA1_Used_
char UCA1_GET_RXBUFLEN(void);//获取RX的FIFO中字符长度
char UCA1_GET_TXBUFLEN(void);//获取TX的FIFO中字符长度
char UCA1_putchar(unsigned char data);//写一个字符到TX FIFO
char UCA1_GET_CHAR(unsigned char *ptr);//从RX FIFO中读一个字节
#endif
#ifdef UCA2_Used_
char UCA2_GET_RXBUFLEN(void);//获取RX的FIFO中字符长度
char UCA2_GET_TXBUFLEN(void);//获取TX的FIFO中字符长度
char UCA2_putchar(unsigned char data);//写一个字符到TX FIFO
char UCA2_GET_CHAR(unsigned char *ptr);//从RX FIFO中读一个字节
#endif
#ifdef UCA3_Used_
char UCA3_GET_RXBUFLEN(void);//获取RX的FIFO中字符长度
char UCA3_GET_TXBUFLEN(void);//获取TX的FIFO中字符长度
char UCA3_putchar(unsigned char data);//写一个字符到TX FIFO
char UCA3_GET_CHAR(unsigned char *ptr);//从RX FIFO中读一个字节
#endif