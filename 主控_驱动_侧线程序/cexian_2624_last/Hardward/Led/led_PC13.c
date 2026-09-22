#include "stm32f10x.h"  // 如果使用标准库
// 或者直接使用寄存器地址（无库版本）

// 函数声明
void PC13_OutputLow(void);

// 函数定义：配置PC13为输出模式并持续输出低电平
void PC13_OutputLow(void) {
    // 1. 开启GPIOC时钟 (APB2ENR寄存器第4位)
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  // 标准库写法
    // 或寄存器直操作：*(volatile uint32_t*)0x40021018 |= (1 << 4);
    
    // 2. 配置PC13为推挽输出模式 (50MHz)
    // CRH寄存器控制Pin8~15，PC13是第13脚（偏移量20位）
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // 清除原有配置
    GPIOC->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1;  // 设置50MHz输出模式
    
    // 3. 持续输出低电平（死循环保持）
//    while(1) {
        // 方法1：使用BSRR寄存器复位（推荐）
        GPIOC->BSRR = GPIO_BSRR_BR13;  // 复位PC13 (低电平)
        
        // 方法2：直接操作ODR寄存器
        // GPIOC->ODR &= ~(1 << 13);
        
        // 可选：添加短暂延时减少CPU占用
        // for(volatile int i = 0; i < 1000; i++);
//    }
}
