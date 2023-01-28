#include "oled.h"
#include "gpio.h"
#include "oledfont.h"
 
/**********************************************
//IIC Stop
**********************************************/
void IIC_Start()
{
    OLED_SCLK_Set() ;
    OLED_SDIN_Set();
    OLED_SDIN_Clr();
    OLED_SCLK_Clr();
}
/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop()
{
    OLED_SCLK_Set() ;
    OLED_SDIN_Clr();
    OLED_SDIN_Set();
}
/**********************************************
//IIC_Wait_Ack
**********************************************/
void IIC_Wait_Ack()
{
    OLED_SCLK_Set() ;
    OLED_SCLK_Clr();
}
/**********************************************
// IIC Write byte
**********************************************/
void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m,da;
    da=IIC_Byte;
    OLED_SCLK_Clr();
    for(i=0; i<8; i++)
    {
        m=da;
        m=m&0x80;
        if(m==0x80)
        {
            OLED_SDIN_Set();
        }
        else OLED_SDIN_Clr();
        da=da<<1;
        OLED_SCLK_Set();
        OLED_SCLK_Clr();
    }
 
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    IIC_Start();
    Write_IIC_Byte(0x78);            //Slave address,SA0=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x00);			//write command
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack();
    IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    IIC_Start();
    Write_IIC_Byte(0x78);			//D/C#=0; R/W#=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x40);			//write data
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack();
    IIC_Stop();
}
/**********************************************
// OLED_WR_Byte
**********************************************/
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    if(cmd)
    {
        Write_IIC_Data(dat);
    }
    else {
        Write_IIC_Command(dat);
    }
}
/**********************************************
// OLED_Set_Pos  设置坐标
**********************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}
/**********************************************
// OLED_Display_On  开启OLED显示
**********************************************/
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**********************************************
// OLED_Display_Off  关闭OLED显示
**********************************************/
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
/**********************************************
// OLED_Clear  清屏函数
**********************************************/
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    uint8_t i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
        for(n=0; n<128; n++)OLED_WR_Byte(0,OLED_DATA);
    } //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';//得到偏移后的值
    if(x>Max_Column-1) {
        x=0;
        y=y+2;
    }
    if(SIZE ==16)
    {
        OLED_Set_Pos(x,y);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0; i<8; i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
    }
    else {
        OLED_Set_Pos(x,y+1);
        for(i=0; i<6; i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);
    }
}
//m^n函数
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}
//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size2/2)*t,y,' ');
                continue;
            } else enshow=1;
        }
        OLED_ShowChar(x+(size2/2)*t,y,temp+'0');
    }
}
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {   OLED_ShowChar(x,y,chr[j]);
        x+=8;
        if(x>120) {
            x=0;
            y+=2;
        }
        j++;
    }
}
//显示汉字
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{
    uint8_t t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0; t<32; t++)
    {
			OLED_WR_Byte(Hzk[4*no][t],OLED_DATA);//将第2*no行，第t个16进制数的内容显示到屏幕上，具体表现为屏幕上显示了一个竖排16长度的一列像素
        adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0; t<32; t++)
    {
        OLED_WR_Byte(Hzk[4*no+1][t],OLED_DATA);
        adder+=1;
    }
		OLED_Set_Pos(x,y+2);
    for(t=0; t<32; t++)
    {
        OLED_WR_Byte(Hzk[4*no+2][t],OLED_DATA);
        adder+=1;
    }
		OLED_Set_Pos(x,y+3);
    for(t=0; t<32; t++)
    {
        OLED_WR_Byte(Hzk[4*no+3][t],OLED_DATA);
        adder+=1;
    }
}
//显示时间
void OLED_ShowTime(uint8_t x,uint8_t y,uint8_t no)
{
    uint8_t t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(time[4*no][t],OLED_DATA);
        adder+=1;
    }
    OLED_Set_Pos(x,y+1);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(time[4*no+1][t],OLED_DATA);
        adder+=1;
    }
		OLED_Set_Pos(x,y+2);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(time[4*no+2][t],OLED_DATA);
        adder+=1;
    }
		OLED_Set_Pos(x,y+3);
    for(t=0; t<16; t++)
    {
        OLED_WR_Byte(time[4*no+3][t],OLED_DATA);
        adder+=1;
    }
}
#define   OLED_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define   GPIOx_OLED_PORT               GPIOB
#define   OLED_SCK_PIN                  GPIO_PIN_6
#define   OLED_SCK_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_SET)
#define   OLED_SCK_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_RESET)
#define   OLED_SCK_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SCK_PIN)
#define   OLED_SDA_PIN                  GPIO_PIN_7
#define   OLED_SDA_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define   OLED_SDA_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define   OLED_SDA_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SDA_PIN)
void OLED_GPIO_Init(void)
{ 
 GPIO_InitTypeDef  GPIO_InitStruct;  

 OLED_GPIO_CLK_ENABLE();
 GPIO_InitStruct.Pin  = OLED_SCK_PIN | OLED_SDA_PIN;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOx_OLED_PORT,&GPIO_InitStruct);
 
 OLED_SDA_OFF();
 OLED_SCK_ON();
}
//初始化SSD1306
void OLED_Init(void)
{
    OLED_GPIO_Init();
    HAL_Delay(100);
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn  oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
    OLED_Clear();
    OLED_Set_Pos(0,0);
}
void Drone_Start(){
	OLED_ShowCHinese(16,2,0);
	OLED_ShowCHinese(48,2,1);
	OLED_ShowCHinese(80,2,2);
}
void Show_Timess(uint8_t hour,uint8_t minute){
	OLED_ShowTime(4,0,hour/10);
	OLED_ShowTime(20,0,hour%10);
	OLED_ShowTime(4,4,minute/10);
	OLED_ShowTime(20,4,minute%10);
//	OLED_ShowTime(24,2,hour/10);
//	OLED_ShowTime(40,2,hour%10);
//	OLED_ShowTime(56,2,10);
//	OLED_ShowTime(72,2,minute/10);
//	OLED_ShowTime(88,2,minute%10);
}
