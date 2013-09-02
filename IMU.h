//=====================================================================================================
// IMU.h
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
//
// See IMU.c file for description.
// 
//=====================================================================================================
#ifndef IMU_h
#define IMU_h

#define HMC5883_Address 0x1e
#define ADXL345_Address 0x53
#define L3G2440_Address 0x69

//For L3G2440
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

typedef struct{
  float pitch,roll,yaw;
}Euler_struct;

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern float q0, q1, q2, q3;	// quaternion elements representing the estimated orientation
extern signed int ax_filtered,ay_filtered,az_filtered,gx_filtered,gy_filtered,gz_filtered;

//---------------------------------------------------------------------------------------------------
// Function declaration

void IMUupdate();

void AHRSupdate();

void IMU_init();

unsigned char IMU_getdata();

void IMU_calibrate();

void IMU_update();

Euler_struct IMU_getEuler();
#endif
//=====================================================================================================
// End of file
//=====================================================================================================