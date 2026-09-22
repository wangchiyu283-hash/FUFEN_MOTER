/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with PID control for electromagnetic rudder
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PID控制器结构体
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float SetPoint;    // 目标值
    float ActualPoint; // 实际值
    float Error;       // 当前误差
    float LastError;   // 上一次误差
    float PrevError2; // 前两次误差
   
    float SumError;    // 误差积分
    float Output;      // PID输出
    float OutMax;      // 输出上限
    float OutMin;      // 输出下限
} PID_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 基础配置
#define ADC_MAX_VALUE    4095    // 12位ADC最大值
#define REF_VOLTAGE      3.3f    // 参考电压3.3V
#define PWM_VOLTAGE_MAX  3.3f    // PWM映射的最大电压

// 角度映射配置（根据实际硬件调整）
#define ANGLE_MIN        0.0f    // 舵机最小角度(°)
#define ANGLE_MAX        20.0f  // 舵机最大角度(°)
#define PWM_DUTY_MIN     0.0f    // 输入PWM最小占空比(%)
#define PWM_DUTY_MAX     100.0f  // 输入PWM最大占空比(%)
#define ADC_VOLT_MIN     0.4477f    // ADC最小电压(V)1：1.52，1.69
#define ADC_VOLT_MAX     0.6127f    // ADC最大电压(V)//1.47    1.665

/*

L1  1.654  1.49     i 0.70
L2  1.56   1.725    i 0.50  
L3  0.4477  0.6127     i 0.70


R1  1.545  1.712    i 0.75  p 4.0
R2  1.66   1.49     i 0.7   p 4
R3  1.519  1.68   i 0.70  p 4 d 0.5
 


*/

// PID参数配置（需根据实际硬件调试）
#define PID_KP           4.0f    // 比例系数
#define PID_KI           0.70f    // 积分系数0.41
#define PID_KD           0.5f   // 微分系数
#define PID_OUT_MAX      100.0f  // PID输出上限
#define PID_OUT_MIN      -100.0f // PID输出下限
#define OVERSHOOT_MARGIN  0.1f   // 允许超调 0.2°

// PWM输出配置（TIM1，假设定时器ARR=99，PSC=71 → 10kHz PWM）
#define PWM_TIM_ARR      99    // TIM1自动重装值
#define PWM_CHANNEL1     TIM_CHANNEL_1 // 正转通道
#define PWM_CHANNEL2     TIM_CHANNEL_2 // 反转通道

#define ALPHA 0.2f   // 平滑系数，可调

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 限幅宏
#define LIMIT(x, max, min) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 输入捕获相关
volatile uint32_t IC_Value1 = 0;        // TIM3上升沿捕获值
volatile uint32_t IC_Value2 = 0;        // TIM3下降沿捕获值
volatile uint8_t Capture_Flag = 0;      // 捕获完成标志(0:未捕获 1:已捕上升沿 2:捕获完成)
volatile uint32_t Duty_Cycle_Raw = 0;   // 原始占空比计数值
float Duty_Cycle_Percent = 0.0f;        // 输入PWM占空比(0~100%)
float Target_Angle = 10.0f;              // 目标角度(°)
float value = 0.0f;
// ADC采样相关
uint32_t ADC_Value = 0;
float V_ADC = 0.0f;                     // ADC采样电压(V)
float Actual_Angle = 0.0f;              // 实际角度(°)
#define STABLE_COUNT 3   // 连续3次同一个值才认为稳定

static int stable_angle = 0;       // 当前稳定角度
static int last_angle = -1;        // 上一次角度
static int stable_cnt = 0;

int Get_Stable_Angle(float angle)
{
    if(angle == last_angle)
    {
        stable_cnt++;
    }
    else
    {
        stable_cnt = 1;
        last_angle = angle;
    }

    if(stable_cnt >= STABLE_COUNT)
    {
        stable_angle = angle;
    }

    return stable_angle;
}
// PID控制器
PID_TypeDef Rudder_PID = {
    .Kp = PID_KP,
    .Ki = PID_KI,
    .Kd = PID_KD,
    .SetPoint = 0.0f,
    .ActualPoint = 0.0f,
    .Error = 0.0f,
    .LastError = 0.0f,
    .PrevError2 = 0.0f,
    .SumError = 0.0f,
    .Output = 0.0f,
    .OutMax = PID_OUT_MAX,
    .OutMin = PID_OUT_MIN
};
// 全局变量定义
float Out_flag = 0;           // 角度值

float Angle = 10;           // 角度值
uint8_t rx_buffer[32];   // 接收缓冲区
uint8_t rx_index = 0;    // 缓冲区索引
uint8_t rx_complete = 0; // 接收完成标志
uint8_t rx_char;          // 单字节接收

float sine_phase = 0.0f;        // 正弦相位
const float SINE_FREQ = 2.0f;  // 正弦频率 Hz（1Hz = 1秒一周期）
const float SINE_AMPL = 7.0f;  // 幅值（度）
const float SINE_OFFSET = 10.0f; // 中心角度

//pwm tO phase of angle`
#define PHASE_MIN     0.0f
#define PHASE_MAX     (2.0f * 3.1415926f)

#define PWM_MIN       5.0f    // % 对应 0
#define PWM_MAX       95.0f   // % 对应 2π

#define PHASE_LPF_ALPHA  0.02f

static float phase = 0.0f;
static float phase_filt = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 重定向 printf 到 UART2 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
void Last_Phase(float phase_raw);
float PWM_Duty_to_Angle(float duty);
void Get_ADC_Voltage(void);                     // ADC采样并转换为电压
void Get_PWM_DutyCycle(void);                   // 计算输入PWM占空比
float PWM_Duty_to_Angle(float duty);            // PWM占空比转角度
float ADC_Volt_to_Angle(float volt);            // ADC电压转角度
void PID_Calculate(PID_TypeDef *pid);          // PID计算
void Rudder_Control(float pid_output);          // 舵机控制（正反转+占空比）
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  // 启动TIM3输入捕获（PWM占空比捕获）
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  
  // 启动TIM1 PWM输出（两路通道）
  HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL1);
  HAL_TIM_PWM_Start(&htim1, PWM_CHANNEL2);
  
  // 初始化为0占空比
  __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL2, 0);

  /* USER CODE END 2 */
  float angle_first =0;
  static long count_of = 0;
  static bool flag = false;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//     count_of++;
//     if(count_of >= 50)
//     {
//        flag = !flag;
//        count_of = 0;
//     }
//     
//     if(flag == true)
//     {
//        Angle = 3;
//     }
//     else
//        Angle = 17;
     /* ===== 正弦角度指令生成 ===== */
//    float Ts = 0.005f; // 你的控制周期 5ms
//    sine_phase += 2.0f * 3.1415926f * SINE_FREQ * Ts;
    
//    if (sine_phase > 2.0f * 3.1415926f)
//       sine_phase -= 2.0f * 3.1415926f;
    
//    Angle = SINE_OFFSET + SINE_AMPL * sin(sine_phase);

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    // 1. 读取ADC值并转换为实际角度
    Get_ADC_Voltage();
//	  printf("V_ADC= %f ",V_ADC);//    
    Actual_Angle = ADC_Volt_to_Angle(V_ADC);
//	  printf("Actual_Angle= %f \r\n",Actual_Angle);
    
    // 2. 计算输入PWM占空比并转换为目标角度
    Get_PWM_DutyCycle();
    angle_first = PWM_Duty_to_Angle(Duty_Cycle_Percent);
//    Last_Phase(angle_first);
    Target_Angle = SINE_OFFSET + SINE_AMPL * sinf(angle_first);;
    // 3. 更新PID目标值和实际值
    Rudder_PID.SetPoint = Target_Angle;
    Rudder_PID.ActualPoint = Actual_Angle;

    PID_Calculate(&Rudder_PID); 
    
    // 5. 根据PID输出控制舵机正反转和速度
    Rudder_Control(Rudder_PID.Output);
    
    
    printf("Target:%.2f,%.4f,%.2f,%.2f,%f,%f,%f\n",
        Target_Angle, Actual_Angle, Rudder_PID.Error, Rudder_PID.Output,V_ADC,Out_flag,value);

    // 控制周期（可根据需求调整，建议10~20ms）
    HAL_Delay(4);
  }
  /* USER CODE END 3 */
}



void Update_Phase(float duty)
{
    float phase_raw = PWM_Duty_to_Angle(duty);

    // 相位 unwrap（防止 2π 跳变）
    if (phase_raw - phase_filt > 3.14f)
        phase_raw -= 2.0f * 3.1415926f;
    else if (phase_raw - phase_filt < -3.14f)
        phase_raw += 2.0f * 3.1415926f;

    phase_filt += PHASE_LPF_ALPHA * (phase_raw - phase_filt);

    // wrap 回 [0, 2π)
    if (phase_filt > PHASE_MAX)
        phase_filt -= PHASE_MAX;
    if (phase_filt < PHASE_MIN)
        phase_filt += PHASE_MAX;

    phase = phase_filt;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  读取ADC值并转换为实际电压
  * @retval None
  */
void Get_ADC_Voltage(void)
{
  // 启动ADC单次转换
  HAL_ADC_Start(&hadc1);
  // 等待转换完成（超时10ms）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    // 读取ADC原始值
    ADC_Value = HAL_ADC_GetValue(&hadc1);
    // 转换为实际电压：V = (ADC值 / ADC最大值) * 参考电压
    V_ADC = (float)ADC_Value * REF_VOLTAGE / ADC_MAX_VALUE;
     
  }
  // 停止ADC
  HAL_ADC_Stop(&hadc1);
}

/**
  * @brief  计算输入PWM占空比
  * @retval None
  */
void Get_PWM_DutyCycle(void)
{
  if (Capture_Flag == 2) // 捕获完成后计算
  {
    // 获取TIM3周期（需匹配TIM3的ARR值，假设TIM3配置为400Hz，ARR=2499）
    uint32_t Period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
    
    // 计算高电平计数值（处理溢出情况）
    if (IC_Value2 > IC_Value1)
    {
      Duty_Cycle_Raw = IC_Value2 - IC_Value1;
    }
    else
    {
      Duty_Cycle_Raw = Period - IC_Value1 + IC_Value2;
    }
    
    // 计算占空比百分比
    Duty_Cycle_Percent = (float)Duty_Cycle_Raw / Period * 100.0f;
    //printf占空比
    
    // 限幅防止异常
    Duty_Cycle_Percent = LIMIT(Duty_Cycle_Percent, 100.0f, 0.0f);
    
    Capture_Flag = 0; // 重置捕获标志
  }
}

/**
  * @brief  PWM占空比转换为舵机角度（线性映射）
  * @param  duty: PWM占空比(0~100%)
  * @retval 映射后的角度(°)
  */
float PWM_Duty_to_Angle(float duty)
{
  // 线性映射：duty → angle
//  float angle = ANGLE_MIN + (duty - PWM_DUTY_MIN) * (ANGLE_MAX - ANGLE_MIN) / (PWM_DUTY_MAX - PWM_DUTY_MIN);
//  // 限幅
//   //printf角度
//  int angle_round = (int)roundf(angle);
//   
//  return LIMIT(angle_round, ANGLE_MAX, ANGLE_MIN);
   
    float p = (duty - PWM_MIN) / (PWM_MAX - PWM_MIN);
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    return PHASE_MIN + p * (PHASE_MAX - PHASE_MIN);
}

/**
  * @brief  ADC电压转换为舵机角度（线性映射）
  * @param  volt: ADC采样电压(0~3.3V)
  * @retval 映射后的角度(°)
  */
float ADC_Volt_to_Angle(float volt)
{
  // 线性映射：voltage → angle
  float angle = ANGLE_MIN + (volt - ADC_VOLT_MIN) * (ANGLE_MAX - ANGLE_MIN) / (ADC_VOLT_MAX - ADC_VOLT_MIN);
   //printf电压值
  // 限幅
  return LIMIT(angle, ANGLE_MAX, ANGLE_MIN);
}

/**
  * @brief  PID计算函数（位置式PID）
  * @param  pid: PID控制器结构体指针
  * @retval None
  */
void PID_Calculate(PID_TypeDef *pid)
{
   static float count = 0.0f;
  // 计算当前误差
  pid->Error = pid->SetPoint - pid->ActualPoint;
   

  static float last_setpoint = 0.0f;    // 用于区分指令变化
   static float LastActualPoint = 0.0f;
  const float DISTURB_E_THRESHOLD_1 = 2.0f;   // 单位：角度（或你的误差单位）
  static float disturb_detected_1 = 0;
  static float vary_set = 0;

  float pos_err = pid->ActualPoint - pid->SetPoint;
   float last_pos_err = LastActualPoint - pid->SetPoint; 
   uint8_t crossed = (pos_err * last_pos_err < 0);
   uint8_t pushing_wrong = (pid->Output * pos_err > 0);
   
  // PID核心计算
  float delta_u = pid->Kp * (pid->Error - pid->LastError) +               // 比例项
                pid->Ki * pid->Error +           // 积分项
                pid->Kd * (pid->Error - 2.0f * pid->LastError + pid->PrevError2); // 微分项

  pid->Output += delta_u;
  // === 超调方向钳制 ===
//   if (crossed && pushing_wrong)  // 目标在f方向
//   {
//      // 如果已经超过目标 + 允许超调
////      if (pid->ActualPoint < pid->SetPoint - OVERSHOOT_MARGIN)
//      {
//         // 禁止继续正向输出
////         if (pid->Output < 0)
//               pid->Output = 0;
//      }
//   }
//   else if (pid->Error < 0) // 目标在z方向
//   {
//      if (pid->ActualPoint > pid->SetPoint + OVERSHOOT_MARGIN)
//      {
//         if (pid->Output > 0)
//               pid->Output = pid->Output * 0.4;
//      }
//   }
  pid->Output = LIMIT(pid->Output, pid->OutMax, pid->OutMin);
  last_setpoint   = pid->SetPoint; // 更新目标，用于下一拍判定
  pid->PrevError2 = pid->LastError;   // e(k-2) = e(k-1)// 保存当前误差为下一次微分使用
  pid->LastError  = pid->Error;       // e(k-1) = e(k)
}

/**
  * @brief  舵机控制函数（根据PID输出控制正反转和占空比）
  * @param  pid_output: PID输出值(-100~100)
  * @retval None
  */
void Rudder_Control(float pid_output)
{
  uint16_t pwm_value = 0;
  
  // 计算PWM占空比（绝对值映射到0~PWM_TIM_ARR）
  pwm_value = (uint16_t)(fabs(pid_output) / PID_OUT_MAX * 99);
  pwm_value = LIMIT(pwm_value, 99, 0);
  value = pwm_value;
  // 根据PID输出正负控制正反转
  if (pid_output > 5.0f) // 正转（PID输出为正，误差为正：实际角度 < 目标角度）
  {
    __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL1, pwm_value);  // 正转通道输出PWM
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//     printf(" zheng_pwm_value = %d ",pwm_value);
     value = 1;
  }
  else if (pid_output < -5.0f) // 反转（PID输出为负，误差为负：实际角度 > 目标角度）
  {

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL2, pwm_value);  // 反转通道输出PWM
//     printf("fu_pwm_value = %d ",pwm_value);
     value = -1;
  }
  else // 误差过小，停止舵机
  {
    __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, PWM_CHANNEL2, 0);
//     printf(" error is so little! ");
  }
}

/**
  * @brief  TIM3输入捕获回调函数
  * @param  htim: 定时器句柄
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (Capture_Flag == 0)
    {
      // 上升沿捕获：记录高电平起始值
      IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      // 设置为下降沿捕获
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      Capture_Flag = 1; // 标记已捕获上升沿
    }
    else if (Capture_Flag == 1)
    {
      // 下降沿捕获：记录高电平结束值
      IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      // 恢复为上升沿捕获，准备下一次捕获
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      Capture_Flag = 2; // 标记捕获完成
    }
  }
}

// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)  // 确保是串口2
    {
       char c = rx_char;
        
        // 如果是数字字符
        if(rx_char >= '0' && rx_char <= '9')
        {
            // 将字符转换为数字
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = c;
            }
            
            
        }
        // 如果是特殊指令
        else if(c == '\n' || c == '\r')
        {
            rx_buffer[rx_index] = '\0';   // 字符串结束
//            printf("Received: , Angle set to: %s\n", rx_buffer);
            int value = atoi((char *) rx_buffer);

            // 限幅：只接受 0~20
            if (value >= 0 && value <= 20)
            {
                Angle = value;
            }
            rx_index = 0; // 清空缓冲区
//            printf("Reset Angle: %d\n",Angle);
        }
        else
        {
            rx_index = 0;
        }
        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
}

void Last_Phase(float phase_raw)
{

    // 相位 unwrap（防止 2π 跳变）
    if (phase_raw - phase_filt > 3.14f)
        phase_raw -= 2.0f * 3.1415926f;
    else if (phase_raw - phase_filt < -3.14f)
        phase_raw += 2.0f * 3.1415926f;

    phase_filt += PHASE_LPF_ALPHA * (phase_raw - phase_filt);

    // wrap 回 [0, 2π)
    if (phase_filt > PHASE_MAX)
        phase_filt -= PHASE_MAX;
    if (phase_filt < PHASE_MIN)
        phase_filt += PHASE_MAX;

    phase = phase_filt;
}
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
    // 错误处理：可添加LED闪烁等提示
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
