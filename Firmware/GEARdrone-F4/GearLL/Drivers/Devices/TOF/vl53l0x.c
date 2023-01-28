#include "vl53l0x.h"

VL53L0X_Dev_t vl53l0x_dev;//�豸I2C���ݲ���
VL53L0X_DeviceInfo_t vl53l0x_dev_info;//�豸ID�汾��Ϣ
_vl53l0x_adjust Vl53l0x_data;   //У׼����24c02��������������ϵͳ��ʼ��ʱ��24C02��ȡ���ݣ�
uint8_t AjustOK=0;//У׼��־λ

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//Ĭ��
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//�߾���
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//������
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//����
		
};

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		return Status;
	}
	return Status;
}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
static uint32_t refSpadCount;
static uint8_t  isApertureSpads;
static uint8_t  VhvSettings;
static uint8_t  PhaseCal;
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ	
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);	
}

//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
//	GPIO_InitTypeDef GPIO_Initure;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;

	pMyDevice->I2cDevAddr = VL53L0X_Addr;//I2C��ַ(�ϵ�Ĭ��0x52)
	pMyDevice->comms_type = 1;           //I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;    //I2Cͨ������
	
	VL53L0X_i2c_init();//��ʼ��IIC����
	
  vl53l0x_Addr_set(pMyDevice,0x52);//����VL53L0X������I2C��ַ
  if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_DataInit(pMyDevice);//�豸Data��ʼ��
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_StaticInit(pMyDevice);//�豸Static��ʼ��
  if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status=VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); //�ο�SPADsУ׼
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status=VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);            //�¶�У׼
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_SetReferenceSpads(dev,refSpadCount,isApertureSpads);   //�趨�ο�SPADs��У׼ֵ
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status=VL53L0X_SetRefCalibration(dev,VhvSettings,PhaseCal);             //�趨�¶�У׼ֵ
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	HAL_Delay(20);
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//��ȡ�豸ID��Ϣ
  if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		return Status;
	}
  	
	return Status;
}

//VL53L0X�����Գ���
void vl53l0x_test(void)
{   
	 while(vl53l0x_init(&vl53l0x_dev))//vl53l0x��ʼ��
	 {	 
		HAL_Delay(500);
	 }

	 while(1)
	 {
		 
		 vl53l0x_general_test(&vl53l0x_dev,Default_Mode);//Ĭ��ģʽ

	 }
}
