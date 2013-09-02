#include "Global.h"
#ifdef MPU6050_Used_
#include "msp430f5438.h"
#include "UCS.h"
#include "TimerA1.h"
#include "I2C.h"
#include "MPU6050.h"

#define MPU6050_FilterLen 20
MPU6050_DataStruct raw,filtered;
signed int MPU6050_ax_bias,MPU6050_ay_bias,MPU6050_az_bias,MPU6050_gx_bias,MPU6050_gy_bias,MPU6050_gz_bias;
unsigned char MPU6050_axar_cnt,MPU6050_ayar_cnt,MPU6050_azar_cnt;
signed long MPU6050_axar_sum,MPU6050_ayar_sum,MPU6050_azar_sum;
signed int MPU6050_axar[MPU6050_FilterLen],MPU6050_ayar[MPU6050_FilterLen],MPU6050_azar[MPU6050_FilterLen];

void MPU6050_init()
{
  I2C_write(MPU6050_Address,PWR_MGMT_1, 0x00);	
  delay_ms(10);
  //DEVICE_RESET_0+SLEEP_0+CYCLE_0+TEMP_DIS_0+CLKSEL_0
  //不复位，上电自动复位、不休眠、使用温度传感器、使用内部8MHz晶振
  I2C_write(MPU6050_Address,SMPLRT_DIV, 0x04);//采样速率=陀螺仪输出速度/(1+4)=200Hz
  delay_ms(10);
  I2C_write(MPU6050_Address,CONFIG, 0x03);//关闭FSYNC引脚输入,5msDLPF
  delay_ms(10);
  I2C_write(MPU6050_Address,GYRO_CONFIG, 0x18);//不自检测，FS_SEL_3：2000度/s
  delay_ms(10);
  I2C_write(MPU6050_Address,ACCEL_CONFIG, 0x01);//不自检测，AFS_SEL_2:2个重力加速度，截止频率5Hz
  delay_ms(10);
  I2C_write(MPU6050_Address,0x36, 0x80);
  I2C_write(MPU6050_Address,0x24, 0x80);
  I2C_write(MPU6050_Address,0x6a, 0x00);  
  delay_ms(100);  
}

void MPU6050_getData()
{
  I2C_read(MPU6050_Address,ACCEL_XOUT_H,14,(unsigned char*)&(raw.AX));
}

void MPU6050_changeFormat()
{
    raw.AX=(unsigned)raw.AX/256+(unsigned)raw.AX%256*256;
    raw.AY=(unsigned)raw.AY/256+(unsigned)raw.AY%256*256;
    raw.AZ=-(signed)((unsigned)raw.AZ/256+(unsigned)raw.AZ%256*256);
    raw.GX=(unsigned)raw.GX/256+(unsigned)raw.GX%256*256;
    raw.GY=(unsigned)raw.GY/256+(unsigned)raw.GY%256*256;
    raw.GZ=(unsigned)raw.GZ/256+(unsigned)raw.GZ%256*256;
}

void MPU6050_filter()
{
    raw.AX-=MPU6050_ax_bias;
    if (MPU6050_axar_cnt<MPU6050_FilterLen-1)
    {
      MPU6050_axar_cnt++;
      MPU6050_axar_sum+=raw.AX;
      MPU6050_axar_sum-=MPU6050_axar[MPU6050_axar_cnt];
      MPU6050_axar[MPU6050_axar_cnt]=raw.AX;
      filtered.AX=MPU6050_axar_sum/MPU6050_FilterLen;
    }
    else
    {
      MPU6050_axar_cnt=0;
      MPU6050_axar_sum+=raw.AX;
      MPU6050_axar_sum-=MPU6050_axar[MPU6050_FilterLen-1];
      MPU6050_axar[MPU6050_axar_cnt]=raw.AX;
      filtered.AX=MPU6050_axar_sum/MPU6050_FilterLen;
    }  
    
    raw.AY-=MPU6050_ay_bias;    
    if (MPU6050_ayar_cnt<MPU6050_FilterLen-1)
    {
      MPU6050_ayar_cnt++;
      MPU6050_ayar_sum+=raw.AY;
      MPU6050_ayar_sum-=MPU6050_ayar[MPU6050_ayar_cnt];
      MPU6050_ayar[MPU6050_ayar_cnt]=raw.AY;
      filtered.AY=MPU6050_ayar_sum/MPU6050_FilterLen;
    }
    else
    {
      MPU6050_ayar_cnt=0;
      MPU6050_ayar_sum+=raw.AY;
      MPU6050_ayar_sum-=MPU6050_ayar[MPU6050_FilterLen-1];
      MPU6050_ayar[MPU6050_ayar_cnt]=raw.AY;
      filtered.AY=MPU6050_ayar_sum/MPU6050_FilterLen;
    }      
    
    raw.AZ-=MPU6050_az_bias;
    if (MPU6050_azar_cnt<MPU6050_FilterLen-1)
    {
      MPU6050_azar_cnt++;
      MPU6050_azar_sum+=raw.AZ;
      MPU6050_azar_sum-=MPU6050_azar[MPU6050_azar_cnt];
      MPU6050_azar[MPU6050_azar_cnt]=raw.AZ;
      filtered.AZ=MPU6050_azar_sum/MPU6050_FilterLen;
    }    
    else
    {
      MPU6050_azar_cnt=0;
      MPU6050_azar_sum+=raw.AZ;
      MPU6050_azar_sum-=MPU6050_azar[MPU6050_FilterLen-1];
      MPU6050_azar[MPU6050_azar_cnt]=raw.AZ;
      filtered.AZ=MPU6050_azar_sum/MPU6050_FilterLen;
    }    
    filtered.GX=(raw.GX*3+filtered.GX)/4;
    filtered.GY=(raw.GY*3+filtered.GY)/4;
    filtered.GZ=(raw.GZ*3+filtered.GZ)/4;    
}


void MPU6050_calibrate()
{
  signed long AX_sum=0,AY_sum=0,AZ_sum=0,GX_sum=0,GY_sum=0,GZ_sum=0;
  MPU6050_ax_bias=0;
  MPU6050_ay_bias=0;
  MPU6050_az_bias=0;
  MPU6050_gx_bias=0;
  MPU6050_gy_bias=0;
  MPU6050_gz_bias=0;  
  MPU6050_getData();
  
  for(unsigned int i=0;i<1000;i++)
  {
    unsigned long IMUPreTime =TimeBase;
    while(!I2C_RXFIN);
    AX_sum+=raw.AX;
    AY_sum+=raw.AY;
    AZ_sum+=raw.AZ;
    GX_sum+=raw.GX;
    GY_sum+=raw.GY;
    GZ_sum+=raw.GZ;
    MPU6050_getData();
  }  
  MPU6050_ax_bias=AX_sum/1000;
  MPU6050_ay_bias=AY_sum/1000;
  MPU6050_az_bias=AZ_sum/1000-16384;
  MPU6050_gx_bias=GX_sum/1000;
  MPU6050_gy_bias=GY_sum/1000;
  MPU6050_gz_bias=GZ_sum/1000;
  
  for(unsigned int i=0;i<MPU6050_FilterLen;i++)
  {
    MPU6050_axar[i]=0;
    MPU6050_ayar[i]=0;
    MPU6050_azar[i]=16384;
  }
  MPU6050_axar_sum=0;
  MPU6050_ayar_sum=0;
  MPU6050_azar_sum=16384L*MPU6050_FilterLen;
  MPU6050_axar_cnt=0;
  MPU6050_ayar_cnt=0;
  MPU6050_azar_cnt=0;    
}
#endif