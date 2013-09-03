#ifndef I2C2_H_
#define I2C2_H_
#define I2C2_RXBUF_SIZE 2
#define I2C2_TXBUF_SIZE 2
extern void I2C2_init();
extern unsigned char I2C2_write(unsigned char slaveadd,unsigned char add,unsigned char data);
extern unsigned char I2C2_read(unsigned char slaveadd,unsigned char regadd,unsigned char len,unsigned char* ramadd);
extern volatile unsigned char I2C2_RXBUF[I2C2_RXBUF_SIZE];
extern volatile unsigned char I2C2_TXBUF[I2C2_TXBUF_SIZE];
extern volatile unsigned char I2C2_RXBUF_LEN,I2C2_TXBUF_LEN;
extern volatile unsigned char I2C2_TXBUF_R,I2C2_TXBUF_W;
extern volatile unsigned char I2C2_TXFIN,I2C2_RXFIN;
#endif
