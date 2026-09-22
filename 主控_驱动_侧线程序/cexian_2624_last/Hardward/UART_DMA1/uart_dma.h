#ifndef __UART3_H
#define __UART3_H
#include "stm32f10x.h"

extern uint8_t UART3TxBuffer[50];
extern uint8_t UART3RxBuffer[50];
extern int UART3_SendFree_Flag;

void UART3_Init(void);
void UART3_DMA1_SendData8Bit(uint16_t Data);
void UART3_DMA1_SendDataStream(int streamlen);

void SendSensorDataBack(void);

#endif

