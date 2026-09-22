#include "servo_set.h"

// 定时器通道映射
static const struct {
    TIM_HandleTypeDef* timer;
    uint32_t channel;
} pwm_channel_map[] = {
    {&htim3, TIM_CHANNEL_3},  // PWM_CH1 - PB0
    {&htim3, TIM_CHANNEL_4},  // PWM_CH2 - PB1
    {&htim3, TIM_CHANNEL_1},  // PWM_CH3 - PB4  
    {&htim3, TIM_CHANNEL_2},  // PWM_CH4 - PB5
    {&htim4, TIM_CHANNEL_1},  // PWM_CH5 - PD12
    {&htim4, TIM_CHANNEL_2},  // PWM_CH6 - PD13
    {&htim4, TIM_CHANNEL_3},  // PWM_CH7 - PD14
    {&htim4, TIM_CHANNEL_4}   // PWM_CH8 - PD15
};


// 启动所有PWM输出
void PWM_StartAll(void)
{
    // 启动TIM3的4个通道
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    // 启动TIM4的4个通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

// 停止所有PWM输出
void PWM_StopAll(void)
{
    // 停止TIM3的4个通道
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    
    // 停止TIM4的4个通道
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

// 设置指定通道的占空比 (0.0 - 100.0)
void PWM_SetDutyCycle(PWM_Channel_t channel, float duty_cycle)
{
    if (channel >= PWM_CH1 && channel <= PWM_CH8) {
        // 限制占空比范围
        if (duty_cycle < 0.0f) duty_cycle = 0.0f;
        if (duty_cycle > 100.0f) duty_cycle = 100.0f;
        
        TIM_HandleTypeDef* timer = pwm_channel_map[channel].timer;
        uint32_t channel_num = pwm_channel_map[channel].channel;
        
        // 计算CCR值
        uint32_t arr = timer->Instance->ARR;
        uint32_t ccr = (uint32_t)((duty_cycle / 100.0f) * arr+0.5f);
        
        // 设置捕获比较寄存器
        switch (channel_num) {
            case TIM_CHANNEL_1:
                timer->Instance->CCR1 = ccr;
                break;
            case TIM_CHANNEL_2:
                timer->Instance->CCR2 = ccr;
                break;
            case TIM_CHANNEL_3:
                timer->Instance->CCR3 = ccr;
                break;
            case TIM_CHANNEL_4:
                timer->Instance->CCR4 = ccr;
                break;
        }
    }
}


// 设置所有通道的占空比
void PWM_SetDutyCycleAll(float duty_cycle)
{
    for (PWM_Channel_t ch = PWM_CH1; ch <= PWM_CH8; ch++) {
        PWM_SetDutyCycle(ch, duty_cycle);
    }
}

// 设置PWM频率 (需要重新配置定时器)
void PWM_SetFrequency(uint32_t frequency_hz)
{
    // 注意：改变频率会影响所有通道
    // 这里以TIM3为例，TIM4需要类似处理
    
    if (frequency_hz == 0) return;
    
    // 停止PWM输出
    PWM_StopAll();
    
    // 计算新的预分频器和自动重载值
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2; // APB1定时器时钟
    uint32_t prescaler = 1;
    uint32_t arr = timer_clock / frequency_hz;
    
    // 如果ARR太大，增加预分频
    while (arr > 0xFFFF) {
        prescaler++;
        arr = timer_clock / (frequency_hz * prescaler);
    }
    
    // 配置TIM3
    htim3.Instance->PSC = prescaler - 1;
    htim3.Instance->ARR = arr - 1;
    
    // 配置TIM4
    htim4.Instance->PSC = prescaler - 1;
    htim4.Instance->ARR = arr - 1;
    
    // 重新启动PWM
    PWM_StartAll();
}


