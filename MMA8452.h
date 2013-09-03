#include "Global.h"
#ifdef MMA8452_Used_
#ifndef MMA8452_H_
#define MMA8452_Address 0x1c
typedef struct
{
  signed int Ax,Ay,Az;
}MMA8452_struct;

extern MMA8452_struct MMA8452_raw;

extern void MMA8452_init();
extern void MMA8452_getData();
#endif
#endif