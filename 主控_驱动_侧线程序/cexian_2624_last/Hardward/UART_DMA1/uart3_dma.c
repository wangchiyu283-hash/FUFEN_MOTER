#include "uart_dma.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "string.h"

int UART3_SendFree_Flag=1;//串口3发送空闲标志
uint8_t UART3TxBuffer[50]={0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
uint8_t UART3RxBuffer[50];
int RxBufferSize=50;

uint8_t RxBuffer[50];                   //需要改
int RxBufferLen=0;
extern uint32_t save[10];

u8 Tuna_Mode = 2; //1 前进 2 后退
u8 Tuna_Dir = 2; //2为左转
u8 Tuna_Pectoral = 1; //1为右鳍
u8 Tuna_Speed=0;


//========浮点数转为整数==========// 
int float_to_uint(float x, float x_min, float x_max, int bits)
{
/// Converts a float to an unsigned int, given range and number of bits ///
float span = x_max - x_min;
float offset = x_min;
	int tmp=((float)((1<<bits)-1));
	tmp=(int) ((x-offset)*((float)((1<<bits)-1))/span);
return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

////=========发送传感器数据===========//
//void SendSensorDataBack(void) {
//    if(UART1_SendFree_Flag != 1)
//        return;

//   //填充深度传感器数据
//	UART1TxBuffer[1] = 0xdd;
//	//将浮点数转换为一个无符号整数。函数的参数给出了浮点数的范围和输出的位数（16位）。
//    int press_int = float_to_uint(Pressure, -1000000, 1000000, 16);
//    int depth_int = float_to_uint(depth, -360, 360, 16);
//		//UART通信每次只发送一个字节，所以需要将16位（2字节）的整数拆分为两个8位（1字节）的部分。
//		//分别存储了压力值的高8位和低8位。
//    UART1TxBuffer[2] = press_int >> 8;    //高八位
//    UART1TxBuffer[3] = press_int & 0xff;    //低八位
//    UART1TxBuffer[4] = depth_int >> 8;
//    UART1TxBuffer[5] = depth_int & 0xff;
//    UART1TxBuffer[6] = MS58xx_TEMP & 0xff;
//		//填充数据包开头结尾
//    UART1TxBuffer[0] = 0xAA;
//    UART1TxBuffer[7] = 0xfd;
//    UART1TxBuffer[8] = 0xff;
//    UART1_DMA1_SendDataStream(9);
//}

//=====串口2 DMA1 TX通道传输数据流=====//
void UART3_DMA1_SendDataStream(int len) {
    DMA_Cmd(DMA1_Channel2, DISABLE);
    UART3_SendFree_Flag = 0;
    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA_SetCurrDataCounter(DMA1_Channel2, len);
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

//=========发送传感器数据===========//
void SendSensorDataBack(void) {
    if(UART3_SendFree_Flag != 1)
        return;

   //填充深度传感器数据     将10个24位的传感器数据放进缓冲区中
	UART3TxBuffer[1] = save[0] >> 16;    //高八位
	UART3TxBuffer[2] = save[0] >> 8;    //中间八位       严格可写成(save[0]>>8)&0xff
    UART3TxBuffer[3] = save[0] & 0xff;    //低八位        一个传感器数据
		
    UART3TxBuffer[4] = save[1] >> 16;    //高八位
	UART3TxBuffer[5] = save[1] >> 8;    //高八位
    UART3TxBuffer[6] = save[1] & 0xff;    //低八位
		
	UART3TxBuffer[7] = save[2] >> 16;    //高八位
	UART3TxBuffer[8] = save[2] >> 8;    //高八位
    UART3TxBuffer[9] = save[2] & 0xff;    //低八位
		
	UART3TxBuffer[10] = save[3] >> 16;    //高八位
	UART3TxBuffer[11] = save[3] >> 8;    //高八位
    UART3TxBuffer[12] = save[3] & 0xff;    //低八位
		
    UART3TxBuffer[13] = save[4] >> 16;    //高八位
	UART3TxBuffer[14] = save[4] >> 8;    //高八位
    UART3TxBuffer[15] = save[4] & 0xff;    //低八位
		
	UART3TxBuffer[16] = save[5] >> 16;    //高八位
	UART3TxBuffer[17] = save[5] >> 8;    //高八位
    UART3TxBuffer[18] = save[5] & 0xff;    //低八位
		
	UART3TxBuffer[19] = save[6] >> 16;    //高八位	
	UART3TxBuffer[20] = save[6] >> 8;    //高八位
    UART3TxBuffer[21] = save[6] & 0xff;    //低八位
		
	UART3TxBuffer[22] = save[7] >> 16;    //高八位
	UART3TxBuffer[23] = save[7] >> 8;    //高八位
    UART3TxBuffer[24] = save[7] & 0xff;    //低八位
				
	UART3TxBuffer[25] = save[8] >> 16;    //高八位
	UART3TxBuffer[26] = save[8] >> 8;    //高八位
    UART3TxBuffer[27] = save[8] & 0xff;    //低八位
				
	UART3TxBuffer[28] = save[9] >> 16;    //高八位
	UART3TxBuffer[29] = save[9] >> 8;    //高八位
    UART3TxBuffer[30] = save[9] & 0xff;    //低八位
		

		//填充数据包开头结尾
    UART3TxBuffer[0] = 0xAA;
    UART3TxBuffer[31] = 0xff;
    UART3_DMA1_SendDataStream(32);           //DMA控制器从内存中读取32个字节的数据，并将其传输到USART3的数据寄存器中   
    
    
    
}



//=====配置USART1和GPIO======//
void USART3_Config(void) {
	
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;  //初始化Usart结构体

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);    //使能GPIOB的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //使能USART2的时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;               //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                  //使能接收中断
    USART_Cmd(USART3, ENABLE);
}

//=====DMA1 for USART1 RX|TX 初始化配置======//
void DMA1_Config(void) {
	//初始化dma结构体，这句话是DMA_InitTypeDef个名DMA_InitStructure，之后就都用DMA_InitStructure
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //开启DMA时钟
    //发送数据
    DMA_DeInit(DMA1_Channel2);   // 设置DMA的传输通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);  // 设置DMA源：内存地址&串口数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART3TxBuffer;  // 内存基地址(要传输的变量的指针)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = 50;  // 设置DMA传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    //接收数据
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART3RxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = RxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	

    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

//=====USART1和DMA中断配置======//
void USART3_DMA1_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  	// 设置中断优先级分组

    // 配置DMA1中断源（串口发送中断）
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	//配置UART1中断源（串口接收中断）
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

//====UART1初始化====//
void UART3_Init() 
	{
    USART3_Config();//串口配置
    DMA1_Config();//dma1配置
    USART3_DMA1_NVIC_Config();

    DMA_Cmd(DMA1_Channel3, ENABLE);  //dma使能 使能UART1接收通道
    UART3_DMA1_SendDataStream(5);  //使能UART1发送通道，并发送初始应答
}


//=====USART1的接收中断处理函数====//
int len = 0;                        //存储接收到的数据长度
int CtrlPkgLen = 5;                 //控制包的长度，设置为5
int txBackIdx = 0;                  //用于跟踪发送缓冲区的索引
void USART3_IRQHandler(void) {
    if(USART_GetITStatus(USART3, USART_IT_IDLE) == RESET)
        return;

    (void)USART3->SR;
    (void)USART3->DR;
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC3);

    len = RxBufferSize - DMA_GetCurrDataCounter(DMA1_Channel3);             //DMA_GetCurrDataCounter(DMA1_Channel5)表示当前DMA传输还剩下多少数据单元没有被接收到   
    memcpy(&RxBuffer[RxBufferLen], &UART3RxBuffer, len);                    //将接收到的数据从UART2RxRuffer复制到RxBuffer
    RxBufferLen = RxBufferLen + len;
    txBackIdx = 0;
    while(RxBufferLen >= CtrlPkgLen) {
        if (RxBuffer[0] != 0xAA || RxBuffer[3] != 0xFC || RxBuffer[4] != 0xFF) {
            RxBufferLen = RxBufferLen - 1;
            memcpy(&RxBuffer[0], &RxBuffer[1], RxBufferLen);
        } else {
            UART3TxBuffer[txBackIdx] = 0xAA;
            txBackIdx++;
            switch (RxBuffer[1]) {
                case 0xDD:
                    Tuna_Mode = (RxBuffer[2] <= 2) ? RxBuffer[2] : Tuna_Mode;
                    UART3TxBuffer[txBackIdx] = 0xdd;
                    txBackIdx++;
                    UART3TxBuffer[txBackIdx] = Tuna_Mode & 0xff;
                    txBackIdx++;
                    break;
                case 0xA2:
                    Tuna_Dir = (RxBuffer[2] <= 4) ? RxBuffer[2] : Tuna_Dir;
                    UART3TxBuffer[txBackIdx] = 0xA2;
                    txBackIdx++;
                    UART3TxBuffer[txBackIdx] = Tuna_Dir & 0xff;
                    txBackIdx++;
                    break;
                case 0xA3:
                    Tuna_Pectoral = (RxBuffer[2] <= 2) ? RxBuffer[2] : Tuna_Pectoral;
                    UART3TxBuffer[txBackIdx] = 0xA3;
                    txBackIdx++;
                    UART3TxBuffer[txBackIdx] = Tuna_Pectoral & 0xff;
                    txBackIdx++;
                    break;
                case 0xA4:
                    Tuna_Speed = (RxBuffer[2] <= 5) ? RxBuffer[2] : Tuna_Speed;
                    UART3TxBuffer[txBackIdx] = 0xA4;
                    txBackIdx++;
                    UART3TxBuffer[txBackIdx] = Tuna_Speed & 0xff;
                    txBackIdx++;
                    break;
                default:
                    break;
            }
            UART3TxBuffer[txBackIdx] = 0xFC;
            txBackIdx++;
            UART3TxBuffer[txBackIdx] = 0xFF;
            txBackIdx++;
            memcpy(&RxBuffer[0], &RxBuffer[CtrlPkgLen], RxBufferLen - CtrlPkgLen);
            RxBufferLen = RxBufferLen - CtrlPkgLen;
        }
    }
    DMA_SetCurrDataCounter(DMA1_Channel3, RxBufferSize);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

//=====USART3的DMA1 TX中断处理函数====//

void DMA1_Channel2_IRQHandler(void) 
	{
	/* 判断是否为DMA发送完成中断 */
    if(DMA_GetITStatus(DMA1_IT_TC2) == RESET)
        return;
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC2);
    UART3_SendFree_Flag = 1;
}




//========单字节的方式发送串口数据==========//
/*函数说明: 利用库函数USART_SendData重新定义适用于DMA环境下的单字节发送函数*/
/*参数说明: Data 待发送的单字节数据*/
void UART3_DMA1_SendData8Bit(uint16_t Data)
{
	while(UART3_SendFree_Flag!=1){};//判断发送串口是否空闲（避免干扰DMA发送的串口数据）
	USART_SendData(USART3, Data);
		/*若调用此函数后，没有延时紧接着调用UART1_DMA1_SendDataStream()
		或UART1_DMA1_SendData8Bit(),这种情况，下面这句是不可缺少的。
		可根据是否有调用间隔，灵活调用下面一句*/
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
}


