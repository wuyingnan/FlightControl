#include "Global.h"
#ifdef MMA8452_Used_
#include "I2C.h"
#include "MMA8452.h"
MMA8452_struct MMA8452_raw;

void MMA8452_init()
{
  I2C_write(MMA8452_Address, 0x2a, 0x00);//Standby mode
  I2C_write(MMA8452_Address, 0x0e, 0x10);//HPFout, +-2g mode
  I2C_write(MMA8452_Address, 0x2b, 0x02);//HighResolution Mode
  I2C_write(MMA8452_Address, 0x2a, 0x15);//200Hz, LowNoise,Active mode  
}

void MMA8452_getData()
{
  I2C_read(MMA8452_Address, 0x01, 6, (unsigned char *)&MMA8452_raw);
}

void MMA8452_filtered()
{
  
}

#endif