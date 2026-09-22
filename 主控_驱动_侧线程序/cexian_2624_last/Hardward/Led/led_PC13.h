#ifndef __GPIO_CTRL_H
#define __GPIO_CTRL_H

#include "stm32f10x.h"  // 若用标准库，需包含（纯寄存器可省略）

// 函数声明：配置PC13为输出并持续低电平
void PC13_OutputLow(void);

#endif  // __GPIO_CTRL_H
