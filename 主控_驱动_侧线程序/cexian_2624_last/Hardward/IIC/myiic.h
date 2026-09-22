#ifndef __MYIIC_H
#define __MYIIC_H

#include "sys.h" 
#include "stm32f10x.h"

/* IIC引脚结构体 */
typedef struct {
    GPIO_TypeDef* SCL_Port;
    uint16_t SCL_Pin;
    GPIO_TypeDef* SDA_Port; 
    uint16_t SDA_Pin;
} IIC_Pin_t;

/* 引脚操作宏（通过结构体指针） */
#define IIC_SCL(iic, x)      sys_gpio_pin_set((iic)->SCL_Port, (iic)->SCL_Pin, x)
#define IIC_SDA(iic, x)      sys_gpio_pin_set((iic)->SDA_Port, (iic)->SDA_Pin, x)
#define IIC_READ_SDA(iic)    sys_gpio_pin_get((iic)->SDA_Port, (iic)->SDA_Pin)

void SDA_OUTPUT(void);
void SDA_INPUT(void);
void SCL_OUTPUT(void);
void SCL_INPUT(void);
void enable_gpio_clock(GPIO_TypeDef* GPIOx);

/* IIC所有操作函数 */
void iic_init(IIC_Pin_t* iic);            /* 初始化IIC的IO口 */
void iic_start(IIC_Pin_t* iic);           /* 发送IIC开始信号 */
void iic_stop1(IIC_Pin_t* iic);            /* 发送IIC停止信号 */
void iic_ack(IIC_Pin_t* iic);             /* IIC发送ACK信号 */
void iic_nack(IIC_Pin_t* iic);            /* IIC不发送ACK信号 */
uint8_t iic_wait_ack(IIC_Pin_t* iic);     /* IIC等待ACK信号 */
void iic_send_byte(IIC_Pin_t* iic, uint8_t txd);/* IIC发送一个字节 */
uint8_t iic_read_byte(IIC_Pin_t* iic);    /* IIC读取一个字节 */

#endif


