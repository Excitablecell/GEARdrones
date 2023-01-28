#include "i2c.h"
#include "main.h"
#include "cmsis_os.h"
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
	 while (delay--)
	{
		;
	}
}

void TOF_I2C_Init(void)
{
	TOF_SDA_0();
	TOF_SCL_0();
	delay_us(50);
	TOF_SDA_1();
	TOF_SCL_1();
	
//	HAL_GPIO_WritePin(XSHUT_GPIO_Port,XSHUT_Pin,GPIO_PIN_RESET);
//	HAL_Delay(30);
//	HAL_GPIO_WritePin(XSHUT_GPIO_Port,XSHUT_Pin,GPIO_PIN_SET);
}

void TOF_I2C_Start(void)
{
	TOF_SDA_1();
	TOF_SCL_1();
	TOF_SDA_0();
	TOF_SCL_0();
}
void TOF_I2C_Stop(void)
{
	TOF_SDA_0();
	TOF_SCL_0();
	TOF_SCL_1();
	TOF_SDA_1();
}

void TOF_I2C_Ack(void)
{
	TOF_SCL_0();
	TOF_SDA_0();
	TOF_SCL_1();
	TOF_SCL_0();
}
void TOF_I2C_NAck(void)
{
	TOF_SCL_0();
	TOF_SDA_1();
	TOF_SCL_1();
	TOF_SCL_0();
}

//ack: 		0
//not ack:1
uint8_t TOF_I2C_WaitAck(void)
{
	uint8_t ucErrTime=0;

	TOF_SDA_1();   
	TOF_SCL_1(); 
	while(TOF_READSDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			TOF_I2C_Stop();
			return 1;
		}
	}
	TOF_SCL_0();//时钟输出0 	   
	return 0;  
}

void TOF_I2C_WriteByte(uint8_t txMsg)
{
	uint8_t i;

	TOF_SCL_0();//时钟输出0 	
	for (i = 0; i < 8; i++)
	{		
		if ((txMsg&0x80)>>7){TOF_SDA_1();}
		else{TOF_SDA_0();}
		txMsg <<= 1;	/* 左移一个bit */
		TOF_SCL_1();
		TOF_SCL_0();
	}
}


//ack: 		 1
//not ack: 0
uint8_t TOF_I2C_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t rxMsg;

	/* 读到第1个bit为数据的bit7 */
	rxMsg = 0;
	TOF_SDA_1();
	delay_us(4);
	for (i = 0; i < 8; i++)
	{
		rxMsg <<= 1;
		TOF_SCL_0();
		TOF_SCL_1();
		if (TOF_READSDA){rxMsg |= 0x01;}
	}
	if (!ack)
		TOF_I2C_NAck();//发送nACK
	else
		TOF_I2C_Ack(); //发送ACK   
	TOF_SCL_0();
	return rxMsg;
}

#define TRUE 1
#define FALSE 0

static uint8_t IIC_Start(void);				
static void IIC_Stop(void);	  		
static void IIC_Send_Byte(uint8_t txd);	
static uint8_t IIC_Read_Byte(void);
static uint8_t IIC_Wait_Ack(void); 	
static void IIC_Ack(void);				
static void IIC_NAck(void);		


//从VL53L0换成了VL53L1，磁力计和气压计还在用上面部分的软件iic，暂时还没有精力合并下面和上面的软件iic，先放在这了

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
static uint8_t IIC_Start(void)
{
	SDA_H;
	SCL_H;
	if(!SDA_read)
		return FALSE;	
	SDA_L;
	if(SDA_read)
		return FALSE;	
	SDA_L;
	return TRUE;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/
static void IIC_Stop(void)
{
	SCL_L;
	SDA_L;
	SCL_H;
	SDA_H;

}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
static uint8_t IIC_Wait_Ack(void) 	
{
	SCL_L;
	SDA_H;
	SCL_H;
	if(SDA_read)
	{
    SCL_L;
      return FALSE;
	}
	SCL_L;
	return TRUE;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
static void IIC_Ack(void)
{
	SCL_L;
	SDA_L;
	SCL_H;
	SCL_L;
}

/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/
static void IIC_NAck(void)
{
	SCL_L;
	SDA_H;
	SCL_H;
	SCL_L;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(uint8_t txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/
static void IIC_Send_Byte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
			SCL_L;
			if(SendByte&0x80) SDA_H;
			else SDA_L;
			SendByte<<=1;
			SCL_H;
    }
    SCL_L;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1串字节，ack=1时，发送ACK，ack=0，发送nACK
*******************************************************************************/
static unsigned char IIC_Read_Byte(void)  
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;
    while(i--)
    {
			ReceiveByte<<=1;
			SCL_L;
			SCL_H;
			if(SDA_read)
			{
				ReceiveByte|=0x01;
			}
    }
    SCL_L;
    return ReceiveByte;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IIC_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/
uint8_t IIC_ReadOneByte(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* data)
{
	if(!IIC_Start())
			return FALSE;
    IIC_Send_Byte(SlaveAddress); 
    if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return FALSE;
		}
    IIC_Send_Byte((uint8_t) REG_Address>>8);   
    IIC_Wait_Ack();
    IIC_Send_Byte((uint8_t) REG_Address & 0x00ff);   
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(SlaveAddress+1);
    IIC_Wait_Ack();

		*data= IIC_Read_Byte();
    IIC_NAck();
    IIC_Stop();
    return TRUE;
}


/**************************实现函数********************************************
*函数原型:		uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/
uint8_t IICreadBytes(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t len,uint8_t *data)
{
		uint8_t i = 0;
		if(!IIC_Start())
			return FALSE;
    IIC_Send_Byte(SlaveAddress); 
    if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return FALSE;
		}
    IIC_Send_Byte((uint8_t) REG_Address>>8);   
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address&0x00ff);   
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(SlaveAddress+1);
    IIC_Wait_Ack();

		for(i = 0;i<len;i++)
		{
			if(i != (len -1))
			{
				data[i]= IIC_Read_Byte();
				IIC_Ack();
			}
			else
			{
				data[i]= IIC_Read_Byte();
				IIC_NAck();
			}
		}
		IIC_Stop();
		return len;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回写入长度
*******************************************************************************/
uint8_t IICwriteBytes(uint8_t dev, uint16_t reg, uint16_t length, uint8_t* data)
{

 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);   
    IIC_Wait_Ack();
	IIC_Send_Byte(reg & 0x00ff);   
    IIC_Wait_Ack();
	for(count=0;count<length;count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	 }
	IIC_Stop();

    return length; //status == 0;
}


/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/
uint8_t IICwriteByte(uint8_t dev, uint16_t reg, uint8_t data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}


/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
 		失败为0
*******************************************************************************/
uint8_t IICwriteBit(uint8_t dev, uint16_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IIC_ReadOneByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
