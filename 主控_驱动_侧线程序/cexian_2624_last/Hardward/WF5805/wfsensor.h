#ifndef	_WFSensor_H_
#define	_WFSensor_H_
#include "stm32f10x.h"
#include "myiic.h"  // 包含IIC头文件

void WFSensor_WriteByte(IIC_Pin_t* iic,u8 addr, u8 Data);
u8 WFSensor_ReadByte(IIC_Pin_t* iic,u8 addr);
u8 WFSensor_WaitFinish(IIC_Pin_t* iic);
void WFSensor_indicateGroupConvert(IIC_Pin_t* iic);
void WFSensor_getTPData(IIC_Pin_t* iic);
void WFSensor_indicateOneByOneConvert(void);
float calculatePress(void);

void get_decData(void);

#endif //_WFSensor_H_
