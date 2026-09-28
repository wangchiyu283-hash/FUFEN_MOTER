/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5048a.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


//SPI_HandleTypeDef hspi1;

//UART_HandleTypeDef huart1;
				
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

//static void MX_GPIO_Init(void);
//static void MX_SPI1_Init(void);
//static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t  TXD[6] = {0};

/*
 * 固定软件零点：舵机/磁铁处于机械 0° 时的 AS5048A 原始角度计数。
 * 标定方法：临时发送/观察原始角度，机械 0° 时读出的 0~16383 数值填入此处。
 * 该值一旦填好，程序每次上电都使用同一个零点，不再执行上电采零。
 */
//#define ENCODER_ZERO_RAW  (14479u) /* 1520 us 舵机中位时标定的 AS5048A 原始值 */
#define ENCODER_ZERO_RAW  (14485u)
#define ENCODER_DIRECTION (+1) /* 若舵机正角使原始值减小，改为 -1 */

/* ---- 舵机校准扫描参数（单位 us） -------------------------------------------- */
#define CAL_PULSE_MIN_US  1180u
#define CAL_PULSE_MAX_US  1846u
#define CAL_PULSE_STEP_US 50u
#define CAL_DWELL_MS      300u

/* ---- 采样平均（每步多采几次取平均，让数据更稳） ---- */
#define SAMPLE_N         4         /* 每步采样次数 */
#define SAMPLE_GAP_MS    2         /* 相邻采样间隔 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	AS5048A_Init();
	Servo_Init();               /* 舵机先输出中性脉宽，回到 0° */

	TXD[0] = 0x55;              /* 帧头 */

	/* 舵机先回机械中立位，但零点使用上面的固定标定值 */
	Servo_SetPulseUs(SERVO_PULSE_MID_US);
	HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		static uint16_t last = 0;                     /* 上次有效原始角度 */
		static uint16_t pulse_us = CAL_PULSE_MIN_US;
		static int8_t pulse_step = 1;

		AS5048A_Status_t status = AS5048A_OK;
		int32_t acc = 0;
		uint8_t ok = 0;
		uint8_t i;

		/* 1. 直接扫描舵机控制脉宽，避免角度换算影响全行程测试。 */
		Servo_SetPulseUs(pulse_us);
		HAL_Delay(CAL_DWELL_MS);

		/* 2. 连续采 SAMPLE_N 次取平均，滤除抖动 */
		for (i = 0; i < SAMPLE_N; i++)
		{
			AS5048A_AngleResult_t r = AS5048A_ReadAngle();
			if (r.status == AS5048A_OK) { acc += r.angle; ok++; }
			else                        { status = r.status; }   /* 记录最近一次错误 */
			HAL_Delay(SAMPLE_GAP_MS);
		}
		if (ok)
		{
			last   = (uint16_t)(acc / ok);
			status = AS5048A_OK;
		}
		/* 全部采样失败则沿用 last（上一次有效角），status 保持错误类型 */
		
		int32_t rel = ((int32_t)last - (int32_t)ENCODER_ZERO_RAW) * ENCODER_DIRECTION;
		if (rel > 8192) rel -= 16384;
		else if (rel <= -8192) rel += 16384;
		int16_t meas_x100 = (int16_t)(rel * 36000 / 16384);

		/* 3. 校准帧：0x55 | 状态 | 脉宽 us(16 位) | 编码器原始角(16 位)。 */
		TXD[1] = 0xAA + (uint8_t)status;
		TXD[2] = (uint8_t)(pulse_us >> 8);
		TXD[3] = (uint8_t)(pulse_us & 0xFFu);
		TXD[4] = (uint8_t)((uint16_t)meas_x100 >> 8);
		TXD[5] = (uint8_t)(meas_x100 & 0xFFu);

		(void)HAL_UART_Transmit(&huart1, TXD, 6, 0xFFFF);

		/* 4. 到 500/2500 us 边界时反向，形成全行程脉宽三角波。 */
		if (pulse_us >= CAL_PULSE_MAX_US) pulse_step = -1;
		else if (pulse_us <= CAL_PULSE_MIN_US) pulse_step = 1;
		pulse_us = (uint16_t)((int32_t)pulse_us +
		                    pulse_step * (int32_t)CAL_PULSE_STEP_US);
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void SystemClock_Config(void)
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
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

