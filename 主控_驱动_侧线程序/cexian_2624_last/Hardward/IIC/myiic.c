#include "myiic.h"
#include "Delay.h"
#include "sys.h"

/* 时钟使能函数 */
void enable_gpio_clock(GPIO_TypeDef* GPIOx) {
    if (GPIOx == GPIOA) RCC->APB2ENR |= 1 << 2;
    else if (GPIOx == GPIOB) RCC->APB2ENR |= 1 << 3;
    else if (GPIOx == GPIOC) RCC->APB2ENR |= 1 << 4;
    else if (GPIOx == GPIOD) RCC->APB2ENR |= 1 << 5;
    else if (GPIOx == GPIOE) RCC->APB2ENR |= 1 << 6;
}

void SDA_OUTPUT(void)
{
    // 可以留空或实现具体功能
}

void SDA_INPUT(void)
{
    // 可以留空或实现具体功能
}

void SCL_OUTPUT(void)
{
    // 可以留空或实现具体功能
}

void SCL_INPUT(void)
{
    // 可以留空或实现具体功能
}

/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
void iic_init(IIC_Pin_t* iic)
{
    enable_gpio_clock(iic->SCL_Port);  /* SCL引脚时钟使能 */
    enable_gpio_clock(iic->SDA_Port);  /* SDA引脚时钟使能 */

    /* SCL引脚模式设置,开漏输出,上拉 */
    sys_gpio_set(iic->SCL_Port, iic->SCL_Pin,
                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);

    /* SDA引脚模式设置,开漏输出,上拉 */
    sys_gpio_set(iic->SDA_Port, iic->SDA_Pin,
                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_OD, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);

    iic_stop1(iic);     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
    delay_us(2);    /* 2us的延时, 读写速度在250Khz以内 */
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(IIC_Pin_t* iic)
{
    IIC_SDA(iic,1);
    IIC_SCL(iic,1);
    iic_delay();
    IIC_SDA(iic,0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC_SCL(iic,0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop1(IIC_Pin_t* iic)  // 修改函数名：iic_stop1 -> iic_stop
{
    IIC_SDA(iic,0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC_SCL(iic,1);
    iic_delay();
    IIC_SDA(iic,1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(IIC_Pin_t* iic)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(iic,1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC_SCL(iic,1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC_READ_SDA(iic))    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop1(iic);
            rack = 1;
            break;
        }
    }

    IIC_SCL(iic,0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(IIC_Pin_t* iic)
{
    IIC_SDA(iic,0);     /* SCL 0 -> 1  时 SDA = 0,表示应答 */
    iic_delay();
    IIC_SCL(iic,1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(iic,0);
    iic_delay();
    IIC_SDA(iic,1);     /* 主机释放SDA线 */
    iic_delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(IIC_Pin_t* iic)
{
    IIC_SDA(iic,1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC_SCL(iic,1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(iic,0);
    iic_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(IIC_Pin_t* iic, uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA(iic, (data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC_SCL(iic,1);
        iic_delay();
        IIC_SCL(iic,0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(iic,1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(IIC_Pin_t* iic)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(iic,1);
        iic_delay();

        if (IIC_READ_SDA(iic))
        {
            receive++;
        }
        
        IIC_SCL(iic,0);
        iic_delay();
    }

    return receive;
}


