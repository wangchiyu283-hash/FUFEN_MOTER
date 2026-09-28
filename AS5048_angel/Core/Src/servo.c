/* TIM2_CH1 (PA0) servo PWM driver. */
#include "servo.h"

void Servo_Init(void)
{
  uint32_t timer_clk = SystemCoreClock;
  uint32_t psc;

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  GPIOA->CRL &= ~(0xFu << 0);
  GPIOA->CRL |=  (0xBu << 0);                 /* PA0: AF push-pull */

  /* 330 Hz, 1 us timer tick. APB1 is currently HCLK in this project. */
  psc = timer_clk / 1000000u;
  if (psc == 0u) psc = 1u;
  TIM2->PSC = (uint16_t)(psc - 1u);
  TIM2->ARR = (uint16_t)(SERVO_PERIOD_US - 1u);
  TIM2->CCR1 = SERVO_PULSE_MID_US;
  TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
  TIM2->CCER = TIM_CCER_CC1E;
  TIM2->CR1 = TIM_CR1_ARPE;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
  if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;
  TIM2->CCR1 = pulse_us;
}

void Servo_SetAngle(int16_t angle_x100)
{
  int32_t pulse;

  if (angle_x100 > SERVO_ANGLE_MAX_X100) angle_x100 = SERVO_ANGLE_MAX_X100;
  if (angle_x100 < -SERVO_ANGLE_MAX_X100) angle_x100 = -SERVO_ANGLE_MAX_X100;

  if (angle_x100 >= 0)
  {
    pulse = (int32_t)SERVO_PULSE_MID_US +
            ((int32_t)angle_x100 *
             (SERVO_PULSE_MAX_US - SERVO_PULSE_MID_US)) /
            SERVO_ANGLE_MAX_X100;
  }
  else
  {
    pulse = (int32_t)SERVO_PULSE_MID_US +
            ((int32_t)angle_x100 *
             (SERVO_PULSE_MID_US - SERVO_PULSE_MIN_US)) /
            SERVO_ANGLE_MAX_X100;
  }
  Servo_SetPulseUs((uint16_t)pulse);
}
