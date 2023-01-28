#include "mpu6050.h"
void MPU6050_Init(void)
{
	MPU_I2C_Init();
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);//001 选择 x 轴陀螺 PLL作为时钟源，以获得更高精度的时钟
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x07);//2
//	MPU_Write_Byte(MPU_CFG_REG,0x03);// 设置MPU6050	内部DLPF滤波器截止频率42HZ 滞后4.8ms
	MPU_Write_Byte(MPU_GYRO_CFG_REG,0X18);//设置陀螺仪满量程±2000°/s
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0X18);//设置加速度计满量程正负16g

}
void MPU_Set_DLPF(uint16_t lpf)
{
	uint8_t data;
	if(lpf>188)data=1;
	else if(lpf>98)data=2;
	else if(lpf>42)data=3;
	else if(lpf>20)data=4;
	else if(lpf>10)data=5;
	else data=6;
	MPU_Write_Byte(MPU_CFG_REG,data);
}
//choose sample frequency
void MPU_Set_SampleFreq(uint16_t Freq)
{
	uint8_t data;
	if(Freq>1000)
	{
		Freq=1000;
	}
	if(Freq<4)
	{
		Freq=4;
	}
	data=1000/Freq-1;
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
	MPU_Set_DLPF(Freq/2);
}


//get temperature
void MPU_Get_Temperature(float *Temperature)
{
		short temp;
	  uint8_t buff[2];	
		MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buff);    //读取温度值
    temp= (buff[0] << 8) | buff[1];	
	  *Temperature=((double) temp/340.0)+36.53;
}

//得到姿态原始数据
void MPU_Get_Gyroscope(short *gyroData)
{
	uint8_t buff[6];
	MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buff);
	  gyroData[0] = (buff[0] << 8) | buff[1];
    gyroData[1] = (buff[2] << 8) | buff[3];
    gyroData[2] = (buff[4] << 8) | buff[5];
}

//得到加速度原始数据
void MPU_Get_Accelerometer(short *accData)
{
	 uint8_t buff[6];
   MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 6, buff);
    accData[0] = (buff[0] << 8) | buff[1];
    accData[1] = (buff[2] << 8) | buff[3];
    accData[2] = (buff[4] << 8) | buff[5];
}

//获取单一数据  即高位合并低位
//param: 传感器寄存器高位地址
void MPU_Get_Single_Data(short *data,uint8_t reg)
{
	  uint8_t buff[2];	
		MPU_Read_Len(reg,2,buff);    
    *data= (buff[0] << 8) | buff[1];	
}

