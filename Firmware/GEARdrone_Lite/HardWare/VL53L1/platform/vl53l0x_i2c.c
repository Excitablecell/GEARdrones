#include "vl53l0x_i2c.h"
#include "i2c.h"
#include "main.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

//VL53L0X I2C初始化
void VL53L0X_i2c_init(void)
{
	TOF_I2C_Init();
}

//IIC写n字节数据
u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
	TOF_I2C_Start();
	TOF_I2C_WriteByte(SlaveAddress);//发写命令
	if(TOF_I2C_WaitAck()) 
	{
		TOF_I2C_Stop();//释放总线
		return 1;//没应答则退出
	}
	TOF_I2C_WriteByte(REG_Address);
	TOF_I2C_WaitAck();
	while(len--)
	{
		TOF_I2C_WriteByte(*buf++);//发送buff的数据
		TOF_I2C_WaitAck();	
	}
	TOF_I2C_Stop();//释放总线

	return 0;
	
}

//IIC读n字节数据
u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	TOF_I2C_Start();
	TOF_I2C_WriteByte(SlaveAddress);//发写命令
	if(TOF_I2C_WaitAck()) 
	{
		TOF_I2C_Stop();//释放总线
		return 1;//没应答则退出
	}
	TOF_I2C_WriteByte(REG_Address);
	TOF_I2C_WaitAck();

	TOF_I2C_Start();
	TOF_I2C_WriteByte(SlaveAddress|0x01);//发读命令
	TOF_I2C_WaitAck();
	while(len)
	{
		if(len==1)
		{
			*buf = TOF_I2C_ReadByte(0);
		}
		else
		{
			*buf = TOF_I2C_ReadByte(1);
		}
		buf++;
		len--;
	}
	TOF_I2C_Stop();//释放总线

	return 0;
	
}

//VL53L0X 写多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Write_nByte(address,index,count,pdata))
	{
	   status  = STATUS_FAIL;

	}

	return status;
}


//VL53L0X 读多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Read_nByte(address,index,count,pdata))
	{
	  status  = STATUS_FAIL;
	}

	return status;
}

//VL53L0X 写1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

//VL53L0X 写1个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_write_word(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	//将16位数据拆分成8位
	buffer[0] = (u8)(data>>8);//高八位
	buffer[1] = (u8)(data&0xff);//低八位
	
	if(index%2==1)
	{  
		//串行通信不能处理对非2字节对齐寄存器的字节
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multi(address,index,buffer,2);
	}
	
	return status;
}

//VL53L0X 写1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OK;

    u8 buffer[4];	
	
	//将32位数据拆分成8位
	buffer[0] = (u8)(data>>24);
	buffer[1] = (u8)((data&0xff0000)>>16);
	buffer[2] = (u8)((data&0xff00)>>8);
	buffer[3] = (u8)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
	
}


//VL53L0X 读1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;
	 
}

//VL53L0X 读个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	status = VL53L0X_read_multi(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;
	
}

//VL53L0X 读1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;
	
}
