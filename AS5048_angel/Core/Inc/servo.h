/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    servo.h
 * @brief   Kingmax KM1850MD 舵机驱动 —— 约 330Hz PWM，脉宽 500~2500us 对应 -90°~+90°
  *
  *   参数（来自舵机规格书）：
  *     - 工作电压：DC 5.0~6.0V（额定 5.0V）
  *     - 中性脉宽：1520us（对应 0°）
  *     - 行程：±90° @ 500us~2500us
  *
  *   本工程未引入 HAL 的 TIM 驱动（Drivers 里没有 stm32f1xx_hal_tim.c），
  *   所以这里直接用寄存器配置 TIM2 的 PWM 输出，不依赖 HAL_TIM。
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* KM1850MD 参数（脉宽单位 us；全行程端点请先空载验证） -------------------------*/
#define SERVO_PULSE_MIN_US    500u
#define SERVO_PULSE_MID_US    1520u
#define SERVO_PULSE_MAX_US    2500u
#define SERVO_PERIOD_US       3030u     /* 约 330 Hz，来自舵机参数表 */
#define SERVO_ANGLE_MAX_X100  9000     /* ±90.00°，单位 0.01° */

/* 初始化：配置 PA0 -> TIM2_CH1，输出约 330Hz PWM，先停在中性位 ------------------*/
void Servo_Init(void);

/* 直接设脉宽（us，自动钳位到 500~2500） ----------------------------------------*/
void Servo_SetPulseUs(uint16_t pulse_us);

/* 设角度（0.01°，自动钳位到 -9000~+9000，即 -90.00°~+90.00°） ------------------*/
void Servo_SetAngle(int16_t angle_x100);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
