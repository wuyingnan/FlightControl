#ifndef I2C_H_
#define I2C_H_
#define I2C_RXBUF_SIZE 2
#define I2C_TXBUF_SIZE 2
extern void I2C_init();
extern unsigned char I2C_write(unsigned char slaveadd,unsigned char add,unsigned char data);
extern unsigned char I2C_read(unsigned char slaveadd,unsigned char regadd,unsigned char len,unsigned char* ramadd);
extern volatile unsigned char I2C_RXBUF[I2C_RXBUF_SIZE];
extern volatile unsigned char I2C_TXBUF[I2C_TXBUF_SIZE];
extern volatile unsigned char I2C_RXBUF_LEN,I2C_TXBUF_LEN;
extern volatile unsigned char I2C_TXBUF_R,I2C_TXBUF_W;
extern volatile unsigned char I2C_TXFIN,I2C_RXFIN;
#endif
