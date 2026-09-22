#ifndef __SERVO_SET_H
#define __SERVO_SET_H

#include "main.h"
#include "tim.h"
// PWM通道定义
typedef enum {
    PWM_CH1 = 0,  // PB0  - TIM3_CH3
    PWM_CH2,      // PB1  - TIM3_CH4  
    PWM_CH3,      // PB4  - TIM3_CH1
    PWM_CH4,      // PB5  - TIM3_CH2
    PWM_CH5,      // PD12 - TIM4_CH1
    PWM_CH6,      // PD13 - TIM4_CH2
    PWM_CH7,      // PD14 - TIM4_CH3
    PWM_CH8       // PD15 - TIM4_CH4
} PWM_Channel_t;

// 函数声明
void PWM_StartAll(void);
void PWM_StopAll(void);
void PWM_SetDutyCycle(PWM_Channel_t channel, float duty_cycle);
void PWM_SetDutyCycleAll(float duty_cycle);
void PWM_SetFrequency(uint32_t frequency_hz);











#endif

