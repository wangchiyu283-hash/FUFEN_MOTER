/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "fashion_star_uart_servo.h"
#include "fashion_star_uart_servo_examples.h"
#include "gyro_get.h"
#include "servo_set.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_QUEUE_LEN    20
#define UART_MSG_MAX_LEN  256
typedef struct {
    char data[UART_MSG_MAX_LEN];
    uint16_t length;
} uart_msg_t;
#define PHASE_PWM_MIN   5.0f
#define PHASE_PWM_MAX   95.0f
#define TWO_PI          6.2831852f
#define PI              3.1415926f
float sine_freq = 1.0f;   // 正弦频率 (Hz)，你要控制的参数
float phase = 0.0f;       // 当前相位 (rad)
float Ts = 0.005f;        // 定时器周期 (秒)

typedef struct
{
    float freq ;        // Hz，0 = 停止摆动
    float phase ;       // rad
    float phase_offset;// rad，相位差
    float duty_center; // 中心占空比
    float duty_amp ;    // 摆动幅值
    uint8_t enable ;    // 1: 正弦摆动 0: 固定输出
} PWM_SineChannel_t;

PWM_SineChannel_t pwm_ch[8] = 
{
    { .freq = 1.0f, .phase_offset = PI },
    { .freq = 1.0f },
    { .freq = 1.0f, .phase_offset = PI },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f, .phase_offset = PI }
};
PWM_SineChannel_t pwm_ch_back[8] = 
{
    { .freq = 1.0f, .phase_offset = PI },
    { .freq = 1.0f },
    { .freq = 1.0f, .phase_offset = PI },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f },
    { .freq = 1.0f, .phase_offset = PI }
};
typedef enum {
    CMD_STOP        = 0x00, // 停止摆动
    CMD_SINE        = 0x01, // 设置正弦摆动
    CMD_FORWARD     = 0x10, // 前进
    CMD_TURN_LEFT   = 0x11,
    CMD_TURN_RIGHT  = 0x12,
    CMD_ROTATE_LEFT = 0x13,
    CMD_ROTATE_RIGHT= 0x14,
    CMD_SINGLE      = 0x15
} PWM_Command_t;
/*
bit0 → CH1
bit1 → CH2
bit2 → CH3
bit3 → CH4
bit4 → CH5
bit5 → CH6
bit6 → CH7
bit7 → CH8
*/
#define CH_ALL   0xFF
#define CH_LEFT  0x0F   // CH1~4
#define CH_RIGHT 0xF0   // CH5~8
// 左翼（前 → 后）
#define CH_L1  3
#define CH_L2  2
#define CH_L3  7

// 右翼（前 → 后）
#define CH_R1  0
#define CH_R2  1
#define CH_R3  4
char *chanel_of[8] = {
   "CH_R1",
   "CH_R2",
   "CH_L2",
   "CH_L1",
   "CH_R3",
   "NULL ",
   "NULL ",
   "CH_L3"
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for Print */
osThreadId_t PrintHandle;
const osThreadAttr_t Print_attributes = {
  .name = "Print",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CommandDispatch */
osThreadId_t CommandDispatchHandle;
const osThreadAttr_t CommandDispatch_attributes = {
  .name = "CommandDispatch",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PWMControl */
osThreadId_t PWMControlHandle;
const osThreadAttr_t PWMControl_attributes = {
  .name = "PWMControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GyroData */
osThreadId_t GyroDataHandle;
const osThreadAttr_t GyroData_attributes = {
  .name = "GyroData",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ServoControl */
osThreadId_t ServoControlHandle;
const osThreadAttr_t ServoControl_attributes = {
  .name = "ServoControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RadioComm */
osThreadId_t RadioCommHandle;
const osThreadAttr_t RadioComm_attributes = {
  .name = "RadioComm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BoardCom */
osThreadId_t BoardComHandle;
const osThreadAttr_t BoardCom_attributes = {
  .name = "BoardCom",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OnboardComp */
osThreadId_t OnboardCompHandle;
const osThreadAttr_t OnboardComp_attributes = {
  .name = "OnboardComp",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radio_cmd_queue */
osMessageQueueId_t radio_cmd_queueHandle;
const osMessageQueueAttr_t radio_cmd_queue_attributes = {
  .name = "radio_cmd_queue"
};
/* Definitions for gyro_data_queue */
osMessageQueueId_t gyro_data_queueHandle;
const osMessageQueueAttr_t gyro_data_queue_attributes = {
  .name = "gyro_data_queue"
};
/* Definitions for servo_cmd_queue */
osMessageQueueId_t servo_cmd_queueHandle;
const osMessageQueueAttr_t servo_cmd_queue_attributes = {
  .name = "servo_cmd_queue"
};
/* Definitions for printfQueue */
osMessageQueueId_t printfQueueHandle;
const osMessageQueueAttr_t printfQueue_attributes = {
  .name = "printfQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void async_printf(const char *format, ...);
void PWM_ParseCommand(char *buf);
osMutexId_t printfMutexHandle;
int  Angle = 350;
float Duty = 50;
char Channel_P;

osTimerId_t pwmTimerHandle;
static uint8_t pwm_toggle = 0;   // 0 → 25%，1 → 75%

osSemaphoreId_t uart_rx_semHandle;  // 用于通知任务
/* USER CODE END FunctionPrototypes */


void PrintTask(void *argument);
void CommandDispatcherTask(void *argument);
void PWMControlTask(void *argument);
void GyroDataTask(void *argument);
void ServoControlTask(void *argument);
void RadioCommTask(void *argument);
void BoardComTask(void *argument);
void OnboardCompTask(void *argument);
void PWM_TimerCallback(void *argument);
   
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of radio_cmd_queue */
  radio_cmd_queueHandle = osMessageQueueNew (16, QUEUE_MSG_SIZE, &radio_cmd_queue_attributes);

  /* creation of printfQueue */
  printfQueueHandle = osMessageQueueNew (32, sizeof(uart_msg_t), &printfQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Print */
  PrintHandle = osThreadNew(PrintTask, NULL, &Print_attributes);

  /* creation of CommandDispatch */
  CommandDispatchHandle = osThreadNew(CommandDispatcherTask, NULL, &CommandDispatch_attributes);

  /* creation of PWMControl */
  PWMControlHandle = osThreadNew(PWMControlTask, NULL, &PWMControl_attributes);

  /* creation of GyroData */
  GyroDataHandle = osThreadNew(GyroDataTask, NULL, &GyroData_attributes);

  /* creation of ServoControl */
  ServoControlHandle = osThreadNew(ServoControlTask, NULL, &ServoControl_attributes);

  /* creation of RadioComm */
  RadioCommHandle = osThreadNew(RadioCommTask, NULL, &RadioComm_attributes);

  /* creation of BoardCom */
  BoardComHandle = osThreadNew(BoardComTask, NULL, &BoardCom_attributes);

  /* creation of OnboardComp */
  OnboardCompHandle = osThreadNew(OnboardCompTask, NULL, &OnboardComp_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  printfMutexHandle = osMutexNew(NULL);
  if (printfMutexHandle == NULL) {
    Error_Handler();
  }
  uart_rx_semHandle = osSemaphoreNew(1, 0, NULL);  // 初始计数 0
  
  pwmTimerHandle = osTimerNew(
                        PWM_TimerCallback,   // 回调函数
                        osTimerPeriodic,     // 周期定时器
                        NULL,
                        NULL);

  osTimerStart(pwmTimerHandle,5);  // 5000 ms = 5 秒
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_PrintTask */
/**
  * @brief  Function implementing the Print thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_PrintTask */
void PrintTask(void *argument)
{
  /* USER CODE BEGIN PrintTask */
  /* Infinite loop */
uart_msg_t msg;
 for (;;)
   {
        if (osMessageQueueGet(printfQueueHandle, &msg, NULL, osWaitForever) == osOK)
        {
           if (osMutexAcquire(printfMutexHandle, osWaitForever) == osOK)
           {   
               HAL_UART_Transmit(&huart4, (uint8_t*)msg.data, msg.length,HAL_MAX_DELAY);
//               while(uart4_tx_busy) osDelay(1);

//               uart4_tx_busy = 1;
//               HAL_UART_Transmit_IT(&huart4, (uint8_t*)msg.data, msg.length);

//               // 等待发送完成，再继续下一条消息
//               while(uart4_tx_busy) osDelay(1);
//              // 等待上一次发送完成
               osMutexRelease(printfMutexHandle);
           }
        }
//      osDelay(100);   

    }
  /* USER CODE END PrintTask */
}

/* USER CODE BEGIN Header_CommandDispatcherTask */
/**
* @brief Function implementing the CommandDispatch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommandDispatcherTask */
void CommandDispatcherTask(void *argument)
{
  /* USER CODE BEGIN CommandDispatcherTask */
  /* Infinite loop */
   char cmd_buf[QUEUE_MSG_SIZE];  // 命令接收缓冲
   osStatus_t status;
   uint32_t tickStart = osKernelGetTickCount();

   for(;;)
   {
      memset(cmd_buf, 0, sizeof(cmd_buf));//清零
         // 处理串口4
      status = osMessageQueueGet(radio_cmd_queueHandle, cmd_buf, NULL, osWaitForever);
         if (status == osOK)
         {
//            printf("[UART4] CMD: %s\r\n", cmd_buf);
            if(cmd_buf[0] == 'S')  // 校验包头
            {
               async_printf("Angle %c,%c,%c\r\n",cmd_buf[2],cmd_buf[3],cmd_buf[4]);
               int sign = (cmd_buf[1] == '1') ? -1 : 1;
               
               Angle = (cmd_buf[2] - '0') * 100 +
                  (cmd_buf[3] - '0') * 10 +
                  (cmd_buf[4] - '0');
   
               Angle *= sign;
               async_printf("Angle %d\r\n",Angle);
            }else if(cmd_buf[0] == 'P'){
               osTimerStop(pwmTimerHandle);
               PWM_ParseCommand(cmd_buf);
               osTimerStart(pwmTimerHandle, 5);
            }else
               async_printf("CMD is error!!!\r\n");
            
         }
//      osDelay(100); 
   }
  /* USER CODE END CommandDispatcherTask */
}

/* USER CODE BEGIN Header_PWMControlTask */
/**
* @brief Function implementing the PWMControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWMControlTask */
void PWMControlTask(void *argument)
{
  /* USER CODE BEGIN PWMControlTask */
  /* Infinite loop */
   // 初始化PWM
    PWM_StartAll();
    
    // 设置初始占空比
    PWM_SetDutyCycleAll(50.0f);  // 初始为0%
    
  for(;;)
  {
//     PWM_SetDutyCycleAll(Duty);
//     PWM_SetDutyCycle((PWM_Channel_t)Channel_P,Duty);
    osDelay(100);
  }
  /* USER CODE END PWMControlTask */
}

/* USER CODE BEGIN Header_GyroDataTask */
/**
* @brief Function implementing the GyroData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroDataTask */
void GyroDataTask(void *argument)
{
  /* USER CODE BEGIN GyroDataTask */
  /* Infinite loop */
  GyroDataSet gyro_data;
  gyro_init();  // 启动DMA接收
   
  for(;;)
  {
     gyro_process_dma_data();
     
     // 步骤3: 检查是否有解析完成的新数据
     if (gyro_get_latest_data(&gyro_data)) {
         // 步骤4: 将数据发送到消息队列
        printf("gyro_data.ACC  :X:%f Y:%f Z:%f\r\n",gyro_data.acc.X,gyro_data.acc.Y,gyro_data.acc.Z);
        printf("gyro_data.gyro :X:%f Y:%f Z:%f\r\n",gyro_data.gyro.X,gyro_data.gyro.Y,gyro_data.gyro.Z);
        printf("gyro_data.angle:X:%f Y:%f Z:%f\r\n",gyro_data.angle.X,gyro_data.angle.Y,gyro_data.angle.Z);
        printf(" Real time is :%f\r\n",gyro_data.timestamp);
     }
     
    osDelay(100);
  }
  /* USER CODE END GyroDataTask */
}

/* USER CODE BEGIN Header_ServoControlTask */
/**
* @brief Function implementing the ServoControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoControlTask */
void ServoControlTask(void *argument)
{
  /* USER CODE BEGIN ServoControlTask */
  /* Infinite loop */
  //舵机控制相关的参数
  //舵机的id号
  uint8_t servo_id = 0;
  // 舵机的目标角度
  // 舵机的角度在-180-180之间，最小单位0.1
  float angle;
  uint32_t interval; //时间间隔ms
  float velocity;	   // 电机转速单位dps,。/s
  //舵机执行功率mv 默认0
  uint16_t power = 0;
  // 加速时间(单位ms)
  uint16_t t_acc;
  //减速时间
  uint16_t t_dec;
  //读取的角度
  float angle_read;
  for(;;)
  {
     
//     /* 查询舵机的角度(多圈模式) */
//FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
//	// 创建环形缓冲队列
//	const uint8_t size = 1; // 请求包content的长度
//	uint8_t ehcoServoId;
//	int32_t echoAngle;
//	
//	// 发送舵机角度请求包
//	FSUS_SendPackage_Common(usart, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, size, &servo_id,0);
//	// 接收返回的Ping
//	PackageTypeDef pkg;
//	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
//	if (statusCode == FSUS_STATUS_SUCCESS){
//		// 成功的获取到舵机角度回读数据
//		ehcoServoId = (uint8_t)pkg.content[0];
//		// 检测舵机ID是否匹配
//		if (ehcoServoId != servo_id){
//			// 反馈得到的舵机ID号不匹配
//			return FSUS_STATUS_ID_NOT_MATCH;
//		}
//		
//		// 提取舵机角度
//		echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) |  (pkg.content[3] << 16) | (pkg.content[4] << 24));
//		*angle = (float)(echoAngle / 10.0);
//	}
//  return statusCode;
//}
//     // 舵机重置多圈角度圈数
//FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id){
//	uint8_t statusCode; // 状态码

//	FSUS_SendPackage_Common(usart, FSUS_CMD_RESERT_SERVO_ANGLE_MTURN, 1, &servo_id,0);
//	// 接收返回的Ping

//	return statusCode;
//}
     
//     printf("Angle:%f\r\n",Angle);
     // 控制舵机角度，基数510
     interval = 200;
     angle = -235.0;
     if(Angle>860||Angle<350)
     {
        if(Angle>860)
           Angle = 860;
        else
           Angle = 350;
     }
     
     FSUS_SetServoAngleMTurn(servo_usart, servo_id, Angle, interval, power);
     
    osDelay(500);
  }
  /* USER CODE END ServoControlTask */
}

/* USER CODE BEGIN Header_RadioCommTask */
/**
* @brief Function implementing the RadioComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RadioCommTask */
void RadioCommTask(void *argument)
{
  /* USER CODE BEGIN RadioCommTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RadioCommTask */
}

/* USER CODE BEGIN Header_BoardComTask */
/**
* @brief Function implementing the BoardCom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BoardComTask */
void BoardComTask(void *argument)
{
  /* USER CODE BEGIN BoardComTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BoardComTask */
}

/* USER CODE BEGIN Header_OnboardCompTask */
/**
* @brief Function implementing the OnboardComp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OnboardCompTask */
void OnboardCompTask(void *argument)
{
  /* USER CODE BEGIN OnboardCompTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OnboardCompTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void parse_command(const char *cmd, uint8_t port_id)
{

 } 

 
void PWM_TimerCallback(void *argument)
{
      // 1. 相位推进
//    phase += TWO_PI * sine_freq * Ts;

//    if (phase >= TWO_PI)
//        phase -= TWO_PI;

//    // 2. 相位 → PWM
//    float duty = PHASE_PWM_MIN +
//                 (phase / TWO_PI) * (PHASE_PWM_MAX - PHASE_PWM_MIN);

//    // 3. 同步发送给所有协控
//    PWM_SetDutyCycleAll(duty);
       for (int ch = 0; ch < 8; ch++)
       {
           PWM_SineChannel_t *c = &pwm_ch[ch];

           float duty;

           if (c->enable && c->freq > 0.01f)
           {
               // 1. 相位推进
               c->phase += TWO_PI * c->freq * Ts;
               
               if (c->phase >= TWO_PI)
                   c->phase -= TWO_PI;

               // 2. 相位发送(相位差）
               float s = c->phase + c->phase_offset;
//               if (s >= TWO_PI)          有数学上的bug
//                   s -= TWO_PI;
               //处理正溢出
               while (s >= TWO_PI) {
                   s -= TWO_PI;
               }

               // 处理负溢出
               while (s < 0.0f) {
                   s += TWO_PI;
               }
               duty = PHASE_PWM_MIN +
               (s / TWO_PI) * (PHASE_PWM_MAX - PHASE_PWM_MIN);
//               printf("ch : %d,  duty: %f\r\n",ch,duty);
           }
           else
           {
               // 固定占空比（停止摆动）
               duty = PHASE_PWM_MIN +
               (c->duty_center / TWO_PI) * (PHASE_PWM_MAX - PHASE_PWM_MIN);
           }

           // 3. 限幅
           if (duty < PHASE_PWM_MIN) duty = PHASE_PWM_MIN;
           if (duty > PHASE_PWM_MAX) duty = PHASE_PWM_MAX;

           PWM_SetDutyCycle((PWM_Channel_t)ch, duty);
       }
}

// 封装的线程安全打印函数

void async_printf(const char *format, ...) {
    
    uart_msg_t msg;
    va_list args;
    char buffer[UART_MSG_MAX_LEN]= {0};   
   
    va_start(args, format);
    msg.length = vsnprintf(buffer, UART_MSG_MAX_LEN, format, args);
    va_end(args);
    
   // 确保字符串以空字符结尾
    if (msg.length >= UART_MSG_MAX_LEN - 1) {
        msg.length = UART_MSG_MAX_LEN - 2;
    }
    memcpy(msg.data, buffer, msg.length+1);
    // 将消息发送到队列，等待时间可根据实际情况调整
    osMessageQueuePut(printfQueueHandle, &msg,0, portMAX_DELAY);
}
/* 
Byte0 : 'P'              // 包头          P（0x50）  
Byte1 : CMD              // 命令类型
Byte2 : CH_MASK          // 通道掩码
Byte3 : PARAM1           // 频率 /不能发0x0A这是换行符号
Byte4 : PARAM2           // 相位 
    CMD_STOP        = 0x00, // 停止摆动
    CMD_SINE        = 0x01, // 设置正弦摆动
    CMD_FORWARD     = 0x10, // 前进
    CMD_TURN_LEFT   = 0x11,
    CMD_TURN_RIGHT  = 0x12,
    CMD_ROTATE      = 0x13  
// 统一映射
freq (Hz) = PARAM1 * 0.1f;        // 0~25.5Hz    1hz  D10 0x0A

typedef struct
{
    float freq;        // Hz，0 = 停止摆动   √
    float phase;       // rad                √
    float phase_offset;// rad，相位差        √
    float duty_center; // 中心占空比         ×
    float duty_amp;    // 摆动幅值           ×
    uint8_t enable;    // 1: 正弦摆动 0: 固定输出   √
} PWM_SineChannel_t;

    PWM_CH1 = 0,  // PB0  - TIM3_CH3    右1
    PWM_CH2,   1   // PB1  - TIM3_CH4  
    PWM_CH3,  2    // PB4  - TIM3_CH1    左1
    PWM_CH4,   3   // PB5  - TIM3_CH2    左2  
    PWM_CH5,   4   // PD12 - TIM4_CH1
    PWM_CH6,   5   // PD13 - TIM4_CH2      
    PWM_CH7,    6  // PD14 - TIM4_CH3    左3
    PWM_CH8    7   // PD15 - TIM4_CH4    左4
    
    
    #define CH_L1  3       0x08       0x8c左翼
#define CH_L2  2           0x04
#define CH_L3  7           0x80

// 右翼（前 → 后）
#define CH_R1  0           01         0x13右翼
#define CH_R2  1           02
#define CH_R3  4           10         0xff全部
   */
   
void PWM_SetForward(float freq, float strength)
{
//    float dphi = 0.5*PI; // rad
//void PWM_SetForward(float freq, float strength)
//{
//    float dphi = (1.0f + 0.4f * strength); // rad

//    // 左
//    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f,        0, 0, 0, 1};
//    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, dphi,        0, 0, 0, 1};
//    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 2.0f*dphi,  0, 0, 0, 1};

//    // 右
//    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f,        0, 0, 0, 1};
//    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, dphi,        0, 0, 0, 1};
//    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 2.0f*dphi,  0, 0, 0, 1};
//}

    float dphi = TWO_PI * strength; // rad

    async_printf("Forward dphi:%.2f\r\n",dphi);
    // 左
    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L1].phase_offset + dphi, 0, 0, 1};		//修改，线圈的初始方向只在pwm_ch中定义
    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L2].phase_offset    ,    0, 0, 1};
    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L3].phase_offset - dphi, 0, 0, 1};

    // 右
    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R1].phase_offset + dphi, 0, 0, 1};
    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R2].phase_offset    ,    0, 0, 1};
    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R3].phase_offset - dphi, 0, 0, 1};
//    // 左
//    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f,            0 + dphi, 0, 0, 1};
//    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0.0f,      PI, 0, 0, 1};
//    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0.0f, PI - dphi, 0, 0, 1};

//    // 右
//    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f,     PI+0, 0, 0, 1};
//    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0.0f,      0- dphi, 0, 0, 1};
//    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0.0f, 0- 2 * dphi, 0, 0, 1};
}

void PWM_SetTurn(float freq, int dir, float strength)
{
//    float base_dphi = 0.8f;
//    float turn_gain = 1.0f + strength;  // 1~2

//    float left_dphi  = (dir < 0) ? base_dphi * 0.2f : base_dphi * turn_gain;
//    float right_dphi = (dir < 0) ? base_dphi * turn_gain : base_dphi * 0.2f;

//    // 左
//    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0, 0.0f,              0, 0, 1};
//    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0, PI+left_dphi,         0, 0, 1};
//    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0, PI+2.0f*left_dphi,    0, 0, 1};

//    // 右
//    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0, PI+0.0f,              0, 0, 1};
//    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0, right_dphi,        0, 0, 1};
//    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0, 2.0f*right_dphi,   0, 0, 1};
   
   float dphi = TWO_PI * strength; // rad
   async_printf("SetTurn dphi:%.2f,dir:%d\r\n",dphi,dir);
		if (dir < 0) {
    // 左
    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L1].phase_offset + dphi, 0, 0, 1};
    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L2].phase_offset    ,    0, 0, 1};
    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L3].phase_offset - dphi, 0, 0, 1};
      
    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R1].phase_offset + dphi, 0, 0, 0};
    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R2].phase_offset,        0, 0, 0};
    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R3].phase_offset - dphi, 0, 0, 0};}
		else	{
    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L1].phase_offset + dphi, 0, 0, 0};
    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L2].phase_offset    ,    0, 0, 0};
    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L3].phase_offset - dphi, 0, 0, 0};
    
    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R1].phase_offset + dphi, 0, 0, 1};
    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R2].phase_offset,        0, 0, 1};
    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R3].phase_offset - dphi, 0, 0, 1};}
}
   
void PWM_SetRotate(float freq, int dir, float strength)
{
//    float dphi = 0.4f + 0.4f * strength;

//    // 左
//    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0, 0.0f,         0, 0, 1};
//    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0, PI+dphi,         0, 0, 1};
//    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0, PI+2.0f*dphi,    0, 0, 1};

//    // 右（反相）
//    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0, PI+PI,               0, 0, 1};
//    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0, PI + dphi,        0, 0, 1};
//    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0, PI + 2.0f*dphi,   0, 0, 1};
   float dphi = TWO_PI * strength; // rad
	
	async_printf("SetRotate dphi:%.2f,dir:%d\r\n",dphi,dir);	
    // 左
    pwm_ch[CH_L1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L1].phase_offset-dir*dphi, 0, 0, 1};
    pwm_ch[CH_L2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L2].phase_offset,          0, 0, 1};
    pwm_ch[CH_L3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_L3].phase_offset+dir*dphi, 0, 0, 1};

    // 右（反相）
    pwm_ch[CH_R1] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R1].phase_offset-dir*dphi, 0, 0, 1};
    pwm_ch[CH_R2] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R2].phase_offset,          0, 0, 1};
    pwm_ch[CH_R3] = (PWM_SineChannel_t){freq, 0.0f, pwm_ch_back[CH_R3].phase_offset+dir*dphi, 0, 0, 1};
}

//
void PWM_ParseCommand(char *buf)
{
    if (buf[0] != 'P') return;

    uint8_t cmd     = buf[1];
    uint8_t ch_mask = buf[2];
    uint8_t v1      = buf[3];
    uint8_t v2      = buf[4];
    async_printf("Enter %0X,%0X,%0X,%0X,%d/255\r\n",cmd,ch_mask,v1,v2,v2);
    float freq      = v1 * 0.1f;        // Hz/
    float offset    = v2 / 255.0f * TWO_PI;        // 相位差
    float level     = v2 / 255.0f;         // 0~1 归一化参数
    switch (cmd)
    {
    case CMD_STOP:// 0x00
        for (int i = 0; i < 8; i++)
        {
              if (ch_mask & (1 << i))//
              {
                 pwm_ch[i].enable = 0;
              }
        }
        break;

    case CMD_SINE:// 0x01
    //设置频率非0时正弦摆动，频率为0时保持位置；发布指令的鳍条运动，未发布指令的鳍条停止
        for (int i = 0; i < 8; i++)
        {
              if (ch_mask & (1 << i))
               {
                  pwm_ch[i] = pwm_ch_back[i];
                  
                  pwm_ch[i].freq           = freq;
                  pwm_ch[i].phase_offset   = pwm_ch[i].phase_offset + offset;
//                pwm_ch[i].phase_offset        = offset;  //修改,offset设置为绝对相位
                  pwm_ch[i].enable         = 1;
                  async_printf("ch:%d,QI:%-6s,yuan:%.3f,offset:%.3f,freq:%.3f\r\n",
                     pwm_ch[i].enable,
                     chanel_of[i],
                     pwm_ch[i].phase_offset,
                     pwm_ch[i].phase_offset -pwm_ch_back[i].phase_offset, 
                     pwm_ch[i].freq
                    );     
               }
               else			 
                  pwm_ch[i].enable         = 0; //修改，未发布指令的鳍条停止                  
        }        
        break;
        
    case CMD_SINGLE:// 0x15
    //设置频率非0时正弦摆动，频率为0时保持位置；发布指令的鳍条运动，未发布指令的鳍条按原先运动
        for (int i = 0; i < 8; i++)
        {
              if (ch_mask & (1 << i))
               {
                  pwm_ch[i] = pwm_ch_back[i];
                  
                  pwm_ch[i].freq           = freq;
                  pwm_ch[i].phase_offset   = pwm_ch[i].phase_offset + offset;
//                pwm_ch[i].phase_offset   = offset;  //修改,offset设置为绝对相位
                  pwm_ch[i].enable         = 1;
                  
                  async_printf("ch:%d,QI:%-6s,yuan:%.3f,offset:%.3f,freq:%.3f\r\n",
                     pwm_ch[i].enable,
                     chanel_of[i],
                     pwm_ch[i].phase_offset,
                     pwm_ch[i].phase_offset - pwm_ch_back[i].phase_offset, 
                     pwm_ch[i].freq
                    );     
               }
        }
        break;
        
    case CMD_FORWARD:// 0x10
        PWM_SetForward(freq, level);
        break;

    case CMD_TURN_LEFT:// 0x11
        PWM_SetTurn(freq, -1, level);
        break;

    case CMD_TURN_RIGHT:// 0x12
        PWM_SetTurn(freq, +1, level);
        break;

    case CMD_ROTATE_LEFT:// 0x13
        PWM_SetRotate(freq, -1, level);
        break;
    
    case CMD_ROTATE_RIGHT:// 0x14
        PWM_SetRotate(freq, +1, level);
        break;
    }
   
}

/* USER CODE END Application */

