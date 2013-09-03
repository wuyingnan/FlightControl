#include "Global.h"
#ifdef MPU6050_Used_
#ifndef MPU6050_H_
#define MPU6050_H_
#define MPU6050_Address         0x68
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	        0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	        0x3B
#define	WHO_AM_I		0x75

typedef struct{
  signed int AX,AY,AZ,TEMP,GX,GY,GZ;
}MPU6050_DataStruct;
extern MPU6050_DataStruct MPU6050_raw, MPU6050_filtered;

extern void MPU6050_init();
extern void MPU6050_getData();
extern void MPU6050_changeFormat();
extern void MPU6050_filter();
extern void MPU6050_calibrate();

#endif
#endif