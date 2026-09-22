/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5048a.h
  * @brief   AS5048A 14-bit 磁编码器 SPI 驱动（含偶校验 / 错误标志校验）
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __AS5048A_H__
#define __AS5048A_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 寄存器地址（14 位，写入命令字的 bit13:0） ----------------------------------*/
#define AS5048A_REG_NOP          0x0000u   /* 无操作（流水线占位用） */
#define AS5048A_REG_CLEAR_ERROR  0x0001u   /* 清错误标志（访问即清除） */
#define AS5048A_REG_AGC          0x3FFDu   /* 自动增益控制 / 诊断标志 */
#define AS5048A_REG_MAGNITUDE    0x3FFEu   /* 磁场强度（CORDIC 幅值） */
#define AS5048A_REG_ANGLE        0x3FFFu   /* 角度（含零位校正） */

/* 读回数据帧位掩码 -----------------------------------------------------------*/
#define AS5048A_DATA_MASK        0x3FFFu   /* bit13:0 = 14 位数据 */
#define AS5048A_EF_MASK          0x4000u   /* bit14 = 错误标志 EF */
#define AS5048A_PAR_MASK         0x8000u   /* bit15 = 偶校验位 PAR */

/* 返回状态 -------------------------------------------------------------------*/
typedef enum
{
  AS5048A_OK = 0,      /* 读取成功，数据有效 */
  AS5048A_ERR_EF,      /* 芯片报告传输错误（此帧 EF=1） */
  AS5048A_ERR_PARITY   /* 偶校验失败（此帧不可信） */
} AS5048A_Status_t;

/* 角度读取结果 ----------------------------------------------------------------*/
typedef struct
{
  uint16_t          angle;   /* 14 位原始角度，0 ~ 16383 */
  AS5048A_Status_t  status;  /* 本帧校验结果 */
} AS5048A_AngleResult_t;

/* 初始化：调用前需已完成 SPI1 与 CS 引脚(PA4)初始化 --------------------------*/
void     AS5048A_Init(void);

/* 读寄存器原始 16 位帧（含 PAR/EF/数据，不做校验） ----------------------------*/
uint16_t AS5048A_ReadRegister(uint16_t reg_addr);

/* 读角度并校验 PAR/EF，返回 角度 + 状态 --------------------------------------*/
AS5048A_AngleResult_t AS5048A_ReadAngle(void);

/* 读磁场强度（0~16383，用于判断磁铁距离是否合适） -----------------------------*/
uint16_t AS5048A_ReadMagnitude(void);

/* 读 AGC（0~255：255=磁场很弱，0=磁场很强） ----------------------------------*/
uint16_t AS5048A_ReadAGC(void);

/* 清错误标志（读取 0x0001 寄存器即清除） -------------------------------------*/
void     AS5048A_ClearError(void);

#ifdef __cplusplus
}
#endif

#endif /* __AS5048A_H__ */
