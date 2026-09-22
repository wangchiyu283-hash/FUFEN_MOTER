/**************************************************************
WFSensor传感器驱动文件
***************************************************************/
#include "wfsensor.h"
#include "Delay.h"
#include "myiic.h"
#include "sys.h"
#include <stdio.h>
#define		WFSensorIICDevice  0XDA			
#define		CMD_GROUP_CONVERT  0X0A			
u8 QT[5];
/********************************************************************
* Function Name  : WFSensor_ReadByte.
* Description    : IIC 写入数据
* Input          : None.
* Output         : None.
* Return         : None.
********************************************************************/
void WFSensor_WriteByte(IIC_Pin_t* iic,u8 addr,u8 Data)
{
	SCL_OUTPUT();
	iic_start(iic);
   iic_send_byte(iic, WFSensorIICDevice); // 写设备号
   iic_wait_ack(iic);
   iic_send_byte(iic, addr); // 写要写入的寄存器地址
   iic_wait_ack(iic);
   iic_send_byte(iic, Data); // 写入数据
   iic_wait_ack(iic);
   iic_stop1(iic);
	SDA_INPUT();
	SCL_INPUT();
}
/********************************************************************
* Function Name  : WFSensor_ReadByte.
* Description    : IIC 写入数据
* Input          : None.
* Output         : None.
* Return         : None.
********************************************************************/
void WFSensor_ReadContiune(IIC_Pin_t* iic,u8 addr,u8 *p,u8 Length)
{
	u8 i = 0;
//	SDA_OUTPUT();
	SCL_OUTPUT();
	iic_start(iic);
   iic_send_byte(iic, WFSensorIICDevice); // 写设备号
   iic_wait_ack(iic);
   iic_send_byte(iic, addr); // 写要读取的起始寄存器地址
   iic_wait_ack(iic);
   iic_start(iic);
   iic_send_byte(iic, WFSensorIICDevice | 0X01); // 读取数据
   iic_wait_ack(iic);
	for(i=0;i<Length;i++)
	{
		p[i]= iic_read_byte(iic);
		if(i < Length-1)
		{
			iic_ack(iic);
		}
	}
	
	iic_nack(iic);
	iic_stop1(iic);
	SDA_INPUT();
	SCL_INPUT();
}

u8 WFSensor_WaitFinish(IIC_Pin_t* iic)
{
	//delayMs(50);
	u8 i = 0;
	SCL_OUTPUT();
	iic_start(iic);
	iic_send_byte(iic, WFSensorIICDevice); //写设备号
	iic_wait_ack(iic);
	iic_send_byte(iic, 0x02); //0x02寄存器
	iic_wait_ack(iic);
	iic_start(iic);
	iic_send_byte(iic, WFSensorIICDevice | 0X01); //读取数据
	iic_wait_ack(iic);
	i = iic_read_byte(iic);
	iic_nack(iic);
	iic_stop1(iic);
	SDA_INPUT();
	SCL_INPUT();
	return i;
}

void WFSensor_indicateGroupConvert(IIC_Pin_t* iic)
{
	WFSensor_WriteByte(iic,0X30,CMD_GROUP_CONVERT);
}

void WFSensor_getTPData(IIC_Pin_t* iic)
{
	WFSensor_ReadContiune(iic ,0X06,QT,5);
}

float calculatePress(void)
{	
  float fDat;
	float Press_Data;  //unit = kpa
	float Temp_Data;   //unit = c 摄氏度
	s32 dat;//有符32位
	
	dat = QT[0];
	dat <<= 8;
	dat |= QT[1];
	dat <<= 8;
	dat |= QT[2];
	if (dat > 8388608) 
	{
		fDat = (dat - 16777216) / 8388608.0f;
	} else {
		fDat = dat / 8388608.0f;
	}
	
	Press_Data = (500 * fDat + 750) / 6.0;

	
	//温度
	dat = QT[3];
	dat = dat << 8;
	dat |= QT[4];
	if (dat > 32768) {
		Temp_Data = (dat - 65844) / 256.0f;
	} else {
		Temp_Data = (dat - 308) / 256.0f;
	}
	
	return Press_Data;
}







