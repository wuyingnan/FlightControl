//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "IMU.h"
#include "msp430_math.h"
#include "I2C.h"
#include "UART.h"
#include "TimerA1.h"
#include "MPU6050.h"

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 2.5f          	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.15f     		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f		// half the sample period
#define FilterLen 20
#define IMUDelayTime 100

//---------------------------------------------------------------------------------------------------
// Variable definitions

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error
signed int ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw;
signed int axar[FilterLen],ayar[FilterLen],azar[FilterLen];
signed long axar_sum=0,ayar_sum=0,azar_sum=0;
unsigned char axar_cnt=0,ayar_cnt=0,azar_cnt=0;
signed int ax_filtered,ay_filtered,az_filtered,gx_filtered,gy_filtered,gz_filtered;
signed int ax_bias,ay_bias,az_bias,gx_bias,gy_bias,gz_bias;
signed int mx_raw,my_raw,mz_raw;

//====================================================================================================
// Function
//====================================================================================================

void IMUupdate() 
{
    float gx, gy, gz, ax, ay, az;  
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;         
#ifdef Bias_Clear    
    static unsigned char cycle=0;
#endif
    
    gx = gx_filtered*0.00106526443603169529841533860381f;
    gy = gy_filtered*0.00106526443603169529841533860381f;
    gz = gz_filtered*0.00106526443603169529841533860381f;    
    
    ax = ax_filtered;
    ay = ay_filtered;
    az = az_filtered;    
    
    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);       
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    
    // estimated direction of gravity
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    
    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    
    // integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

#ifdef Bias_Clear
    cycle++;
    if (cycle==200)
    {
      gx_bias=
      exInt=0;
      eyInt=0;
      ezInt=0;      
    }
#endif    
}

void AHRSupdate() 
{
        float gx, gy, gz, ax, ay, az, mx, my, mz;
        float norm;
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;

        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;   
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;          

        gx = gx_filtered*0.00106526443603169529841533860381f;
        gy = gy_filtered*0.00106526443603169529841533860381f;
        gz = gz_filtered*0.00106526443603169529841533860381f;    
        
        ax = ax_filtered;
        ay = ay_filtered;
        az = az_filtered;    
        
        mx = mx_raw;
        my = my_raw;
        mz = mz_raw;
        
        // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
        norm = sqrt(mx*mx + my*my + mz*mz);          
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;         
        
        // compute reference direction of flux
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         

        bx = sqrt((hx*hx) + (hy*hy));
        bz = hz;        
    
        // estimated direction of gravity and flux (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
      
        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        
        // integral error scaled integral gain
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        // integrate quaternion rate and normalise
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
}


//====================================================================================================
//Reimplemented Functions by WuYingnan
//====================================================================================================
signed int abs(signed int a)
{
  if (a<0)
    return -a;
  else
    return a;
}

void IMU_init()
{
//HMC5883_init  
  while(I2C_write(HMC5883_Address,0x00,0x78));
  while(I2C_write(HMC5883_Address,0x01,0x40));
  while(I2C_write(HMC5883_Address,0x02,0x00));
  
//ADXL345_init
  while(I2C_write(ADXL345_Address,0x31,0x08));   //测量范围,正负2g，13位模式 分辨率4mg
  while(I2C_write(ADXL345_Address,0x2C,0x08));   //速率设定为12.5 参考pdf13页
  while(I2C_write(ADXL345_Address,0x2D,0x08));   //选择电源模式   参考pdf24页  
  
//L3G2440_init  
  while(I2C_write(L3G2440_Address,CTRL_REG1, 0x4f));   //
  while(I2C_write(L3G2440_Address,CTRL_REG2, 0x00));   //
  while(I2C_write(L3G2440_Address,CTRL_REG3, 0x08));   //
  while(I2C_write(L3G2440_Address,CTRL_REG4, 0xF0));   //+-2000dps
  while(I2C_write(L3G2440_Address,CTRL_REG5, 0x10));   //enable High Pass Filter
  
#ifdef MPU6050_Used_  
//MPU6050_init
  MPU6050_init();
#endif
}

unsigned char IMU_getdata()
{
    unsigned long IMUPreTime =TimeBase;
    
    if(IMUPreTime % 20==5)
    {
      while(!I2C_RXFIN)
      {
        if(IMUPreTime + IMUDelayTime <TimeBase)
        {
          I2C_init();
          return 1;
        }
      }      
      
      I2C_read(0x1e,0x03,2,(unsigned char *)&mx_raw);          
      while(!I2C_RXFIN)
      {
        if(IMUPreTime + IMUDelayTime <TimeBase)
        {
          I2C_init();
          return 1;
        }
      }
      
      I2C_read(0x1e,0x05,2,(unsigned char *)&mz_raw);
      mx_raw=(unsigned)mx_raw/256+((unsigned)mx_raw%256)*256;
      while(!I2C_RXFIN)
      {
        if(IMUPreTime + IMUDelayTime <TimeBase)
        {
          I2C_init();
          return 1;
        }
      }        
      I2C_read(0x1e,0x07,2,(unsigned char *)&my_raw);
      mz_raw=(unsigned)mz_raw/256+((unsigned)mz_raw%256)*256;
      
      while(!I2C_RXFIN)
      {
        if(IMUPreTime + IMUDelayTime <TimeBase)
        {
          I2C_init();
          return 1;
        }
      }        
      my_raw=(unsigned)my_raw/256+((unsigned)my_raw%256)*256;    
    }
        
    if(I2C_read(0x53,0x32,2,(unsigned char *)&ax_raw))
      return 1;
    
#ifdef MPU6050_Used_
      MPU6050_changeFormat();
      MPU6050_filter();
#endif  
    
    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }
    if(I2C_read(0x53,0x34,2,(unsigned char *)&ay_raw))
       return 1;
//    ax_raw=(unsigned)ax_raw/256+((unsigned)ax_raw%256)*256-ax_bias;
    ax_raw-=ax_bias;
    if (axar_cnt<FilterLen-1)
    {
      axar_cnt++;
      axar_sum+=ax_raw;
      axar_sum-=axar[axar_cnt];
      axar[axar_cnt]=ax_raw;
      ax_filtered=axar_sum/FilterLen;
    }
    else
    {
      axar_cnt=0;
      axar_sum+=ax_raw;
      axar_sum-=axar[FilterLen-1];
      axar[axar_cnt]=ax_raw;
      ax_filtered=axar_sum/FilterLen;
    }
    
    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }
    if(I2C_read(0x53,0x36,2,(unsigned char *)&az_raw))
       return 1;    
//    ay_raw=(unsigned)ay_raw/256+((unsigned)ay_raw%256)*256-ay_bias;
    ay_raw-=ay_bias;    
    if (ayar_cnt<FilterLen-1)
    {
      ayar_cnt++;
      ayar_sum+=ay_raw;
      ayar_sum-=ayar[ayar_cnt];
      ayar[ayar_cnt]=ay_raw;
      ay_filtered=ayar_sum/FilterLen;
    }
    else
    {
      ayar_cnt=0;
      ayar_sum+=ay_raw;
      ayar_sum-=ayar[FilterLen-1];
      ayar[ayar_cnt]=ay_raw;
      ay_filtered=ayar_sum/FilterLen;
    }    
    
    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }
    if(I2C_read(0x69,0x28|0x80,2,(unsigned char *)&gx_raw))
      return 1;    
//    az_raw=(unsigned)az_raw/256+((unsigned)az_raw%256)*256-az_bias;    
    az_raw-=az_bias;
    if (azar_cnt<FilterLen-1)
    {
      azar_cnt++;
      azar_sum+=az_raw;
      azar_sum-=azar[azar_cnt];
      azar[azar_cnt]=az_raw;
      az_filtered=azar_sum/FilterLen;
    }    
    else
    {
      azar_cnt=0;
      azar_sum+=az_raw;
      azar_sum-=azar[FilterLen-1];
      azar[azar_cnt]=az_raw;
      az_filtered=azar_sum/FilterLen;
    }

    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }
    if(I2C_read(0x69,0x2a|0x80,2,(unsigned char *)&gy_raw))
      return 1;    
    gx_raw=(unsigned)gx_raw/256+((unsigned)gx_raw%256)*256-gx_bias;

    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }
    if(I2C_read(0x69,0x2c|0x80,2,(unsigned char *)&gz_raw))
      return 1;
    gy_raw=(unsigned)gy_raw/256+((unsigned)gy_raw%256)*256-gy_bias;

    while(!I2C_RXFIN)
    {
      if(IMUPreTime + IMUDelayTime <TimeBase)
      {
        I2C_init();
        return 1;
      }
    }    
#ifdef MPU6050_Used_
    MPU6050_getData();
#endif      
      gz_raw=(unsigned)gz_raw/256+((unsigned)gz_raw%256)*256-gz_bias;

//    if (abs(gx_raw-gx_filtered)<100)
      gx_filtered=(gx_raw*3+gx_filtered)/4;
//    if (abs(gy_raw-gy_filtered)<100)    
      gy_filtered=(gy_raw*3+gy_filtered)/4;
//    if (abs(gz_raw-gz_filtered)<100)    
      gz_filtered=(gz_raw*3+gz_filtered)/4;

    /*
    UART_sendint(UCA1,ax_filtered+32768);                
    UART_sendstr(UCA1," ");          
    UART_sendint(UCA1,ay_filtered+32768);                
    UART_sendstr(UCA1," ");     
    UART_sendint(UCA1,az_filtered+32768);                
    UART_sendstr(UCA1," ");    
    UART_sendint(UCA1,gx_filtered+32768);                
    UART_sendstr(UCA1," ");       
    UART_sendint(UCA1,gy_filtered+32768);               
    UART_sendstr(UCA1," ");    
    UART_sendint(UCA1,gz_filtered+32768);                
    UART_sendstr(UCA1," ");    */
      
    return 0;
}


void IMU_calibrate()
{
  signed long ax_sum=0,ay_sum=0,az_sum=0,gx_sum=0,gy_sum=0,gz_sum=0;
  ax_bias=0;
  ay_bias=0;
  az_bias=0;
  gx_bias=0;
  gy_bias=0;
  gz_bias=0;  
  for(unsigned int i=0;i<1000;i++)
  {
    IMU_getdata();
    ax_sum+=ax_raw;
    ay_sum+=ay_raw;
    az_sum+=az_raw;
    gx_sum+=gx_raw;
    gy_sum+=gy_raw;
    gz_sum+=gz_raw;
  }
  ax_bias=ax_sum/1000;
  ay_bias=ay_sum/1000;
  az_bias=az_sum/1000-256;
  gx_bias=gx_sum/1000;
  gy_bias=gy_sum/1000;
  gz_bias=gz_sum/1000;
  
  for(unsigned int i=0;i<FilterLen;i++)
  {
    axar[i]=0;
    ayar[i]=0;
    azar[i]=256;
  }
  axar_sum=0;
  ayar_sum=0;
  azar_sum=256L*FilterLen;
  axar_cnt=0;
  ayar_cnt=0;
  azar_cnt=0;  
#ifdef MPU6050_Used_  
  MPU6050_calibrate();
#endif  
}

void IMU_update()
{
  static signed long a;
  static float roll=0.0,pitch=0.0,yaw=0.0;
  
  roll += (long)gx_filtered*2000/32768;
  pitch+= (long)gy_filtered*2000/32768;
  yaw  += (long)gz_filtered*2000/32768;
  
  a=(long)ax_filtered*(long)ax_filtered+(long)ay_filtered*(long)ay_filtered+(long)az_filtered*(long)az_filtered;
/*  
  UART_sendint(UCA1,(signed int)(roll)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(pitch)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(yaw)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gx_filtered)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gy_filtered)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gz_filtered)+32768);
  UART_sendstr(UCA1," ");*/
  UART_sendint(UCA1,ax_filtered+32767);
  UART_sendstr(UCA1," ");  
  UART_sendint(UCA1,ay_filtered+32767);
  UART_sendstr(UCA1," ");  
  UART_sendint(UCA1,az_filtered+32767);
  UART_sendstr(UCA1," ");    
}

Euler_struct IMU_getEuler()
{
    Euler_struct euler;

    euler.pitch=-asin(-2 * q1 * q3 + 2 * q0* q2);
    euler.roll =atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);
    euler.yaw  =atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1);
        
    return euler;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
