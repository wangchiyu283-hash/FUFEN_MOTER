#include "Delay.h"
#include "sys.h"
#include "myiic.h"
#include "uart_dma.h"
#include <stdio.h>
#include "usart_serial.h"
#include "wfsensor.h"
#include "stm32f10x.h"
#include "led_PC13.h"
#define count 10

/*禁用J-tag引脚*/
void disable_jtag_pins(void)
{
    // 1. 使能AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);  // 启动 SWD
    // 2. 禁用JTAG，但保留SWD调试功能
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//实际上swd模式也会禁用
    // 这样会释放PA15, PB3, PB4作为普通IO使用
    
}


/* 定义count路IIC的引脚配置（使用正确的GPIO_Pin_X） */
IIC_Pin_t iic_pins[count] = {
    {GPIOA, GPIO_Pin_15,  GPIOB, GPIO_Pin_3},   // IIC0: PA15-SCL, PB3-SDA                            not,if disable jtag,ok
    {GPIOA, GPIO_Pin_11,  GPIOA, GPIO_Pin_12},   // IIC1: PA11-SCL, PA12-SDA                          ok
    {GPIOA, GPIO_Pin_9,  GPIOA, GPIO_Pin_10},   // IIC2: PA9-SCL, PA10-SDA                            ok
    {GPIOB, GPIO_Pin_14,  GPIOB, GPIO_Pin_15},   // IIC3: PB14-SCL, PB15-SDA                          ok
    {GPIOB, GPIO_Pin_12,  GPIOB, GPIO_Pin_13},   // IIC4: PB12-SCL, PB13-SDA                          not  Convert failed
    {GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11},  // IIC5: PB10-SCL, PB11-SDA                            not，会卡死/必须开漏输出，之前scl是推挽输出，会和复用功能冲突，ok
    {GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_1},   // IIC6: PB0-SCL, PB1-SDA（注意：SDA和SCL不能相同）      ok
    {GPIOA, GPIO_Pin_6,  GPIOA, GPIO_Pin_7},// IIC7: PA6 -SCL, PA7-SDA（注意：SDA和SCL不能相同）       ok
    {GPIOB, GPIO_Pin_6,  GPIOB, GPIO_Pin_7},// IIC8: PB6-SCL, PB7-SDA（注意：SDA和SCL不能相同）        ok
    {GPIOB, GPIO_Pin_4,  GPIOB, GPIO_Pin_5},// IIC9: PB4-SCL, PB5-SDA（注意：SDA和SCL不能相同）        not,if disable jtag,ok
};

float outTemp[10],outPress[10];
uint32_t save[10];
float wf_Press_Data;
extern int num;
extern u16 flag;
float ok_sum = 0;
static int error[10] = {0};
static int error_back[10] = {0};
int main(void)
{ 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	usart_init(36,115200);	//初始往上位机print串口
   disable_jtag_pins();

   for(int i = 0; i < count; i++)
      iic_init(iic_pins + i);
	printf("IIC initialization successful!!!\n");
	float sum1 = 0; 
   float sum2 = 0;
   float average1 [10]= {0};
   float Aver = 0;
   float all = 0;
   static float offset[10] = {0};
   static float offset_last[10] = {0};
   static float cushi[10][10] = {0};
	int isno_data[count] = {0}; 
   static float buf[10];
   static float countof = 0;
   for(int j=0;j<10;j++)
   { 
       for(int i = 0; i<count; i++)
            WFSensor_indicateGroupConvert(iic_pins + i); //发送转换命令
         delay_ms(5);      
         int i = 0;
         while(i<count){
            if(WFSensor_WaitFinish(iic_pins + i) != 0x01)
            {
               printf("IIC %d Convert failed!\r\n", i);
               isno_data[i] = 1;
               buf[i] = 0.0; 
               error[i] = 1;
            }
            ++i;
         }
         for(int i = 0; i<count; i++)
         {
            if(!isno_data[i])
            {
               WFSensor_getTPData(iic_pins + i); //读取数据
               delay_ms(5);
               wf_Press_Data = calculatePress(); //换算数据
               delay_ms(5);
   //            printf("Pressure of IIC%d : %f\r\n",i,wf_Press_Data); //串口输出原始数据
               buf[i] = wf_Press_Data;
               cushi[j][i] = buf[i];
               error[i] = 0;
            }
            else
               isno_data[i] = 0;
         }
         for(int i = 0; i < 10; i++)
         {
             sum1 = sum1 + buf[i];
         }
         for(int i = 0; i < 10; i++)
         {
             if(error[i] == 0)
             {
                ok_sum++;
                error_back[i] = error[i];
             }
             else
             {
                
                error_back[i] = error[i];
                error[i] = 0;
             }
         }
         if(ok_sum != 0.0f)
            average1[j]= sum1/(ok_sum);
         sum1 = 0;
         printf(" ok_sum:%f average:%f\r\n",ok_sum , average1[j]);
         ok_sum = 0;
            
   }
         
         for(int j = 0;j<10;j++)
         {
            sum2 = sum2 + average1 [j];
         }
         all = sum2/10.0f;
         printf ("average all : %f\r\n", all);
         sum2 = 0;
         for(int i = 0; i < 10; i++)
         {
            for(int j = 0;  j<10; j++)
            { Aver = Aver + cushi[j][i];}
            Aver = Aver/10;
            if(error_back[i] == 0)
               offset[i] = all - Aver;
            else
               offset[i] = 0.0f;
            Aver = 0; 
            printf ("offset[i] : %f    ", offset[i]);
         }
 
	while(1)
	{
       PC13_OutputLow();
      
//      GPIO_ResetBits(GPIOA, GPIO_Pin_11);
//      GPIO_ResetBits(GPIOA, GPIO_Pin_12);
//		delay_ms(20);
//      GPIO_SetBits(GPIOA, GPIO_Pin_11);
//      GPIO_SetBits(GPIOA, GPIO_Pin_12);
      for(int i = 0; i<count; i++)
         WFSensor_indicateGroupConvert(iic_pins + i); //发送转换命令
      delay_ms(5);      
      int i = 0;
      while(i<count){
         if(WFSensor_WaitFinish(iic_pins + i) != 0x01)
         {
//            printf("IIC %d Convert failed!\r\n", i);
            isno_data[i] = 1;
            buf[i] = 0.0; 
            error[i] = 1;
         }
         ++i;
      }
      for(int i = 0; i<count; i++)
      {
         if(!isno_data[i])
         {
            WFSensor_getTPData(iic_pins + i); //读取数据
            delay_ms(5);
            wf_Press_Data = calculatePress(); //换算数据
            delay_ms(5);
//            printf("Pressure of IIC%d : %f\r\n",i,wf_Press_Data); //串口输出原始数据
            buf[i] = wf_Press_Data;
            error[i] = 0;
         }
         else
            isno_data[i] = 0;
      }
      for(int i = 0; i < 10; i++)
         if(error[i] == 0)
            offset_last[i] = offset[i];
         else
         {
            error[i] = 0;
            offset_last[i] = 0.0f;
         }
      printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",countof,buf[0] + offset_last[0],buf[1] + offset_last[1]
                                                            ,buf[2] + offset_last[2],buf[3] + offset_last[3]
                                                            ,buf[4] + offset_last[4],buf[5] + offset_last[5]
                                                            ,buf[6] + offset_last[6],buf[7] + offset_last[7]
                                                            ,buf[8] + offset_last[8],buf[9] + offset_last[9],countof); 
                                                            //串口输出原始数据
//      printf("1jjjjjj\r\n");
		delay_ms(10);
      countof ++;
      
   }
}

