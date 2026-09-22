/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    selftest.c
  * @brief   串口(UART)自测程序 —— 用于排查"串口收不到数据 / 全 FF"的问题
  *
  *   使用方法：
  *     1. 在 Keil 工程里排除 main.c 的编译：
  *        工程树中展开 Application/User/Core → 右键 main.c →
  *        取消勾选 "Include in Target Build"（把 √ 去掉）。
  *        （因为 main.c 和本文件都有 main()，两个一起编译会重复定义。）
  *     2. 重新编译、下载、复位。
  *     3. 打开串口助手：115200，8 数据位，1 停止位，无校验，十六进制显示。
  *
  *   判读：
  *     - 每 500ms 收到一帧：55 AA xx 66（xx 从 00 递增到 FF 循环）
  *       → UART + 接线完全正常，问题在 SPI/AS5048A 那边。
  *     - 仍是 FF/FE/FD 或没有任何变化
  *       → UART 链路本身不通，继续查跳线帽 / 接线 / COM 口 / 驱动。
  *
  *   说明：本文件使用内部 HSI 8MHz 时钟（不开 PLL、不用外部晶振），
  *         可以排除外部晶振导致程序卡死在 Error_Handler 的因素。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"

/* 私有函数原型 ---------------------------------------------------------------*/
static void SystemClock_Config(void);

/**
  * @brief  自测入口（临时替换 main）
  * @retval int
  */
int main(void)
{
  uint8_t counter = 0;
  uint8_t frame[4] = {0x55, 0xAA, 0x00, 0x66};

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  while (1)
  {
    frame[2] = counter++;          /* 递增计数，方便肉眼确认数据在刷新 */
    HAL_UART_Transmit(&huart1, frame, 4, 100);
    HAL_Delay(500);
  }
}

/**
  * @brief  内部 HSI 8MHz 时钟（不依赖外部晶振）
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while (1) {}
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    while (1) {}
  }
}

/**
  * @brief  main.c 被排除编译后，由本文件提供 Error_Handler
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
