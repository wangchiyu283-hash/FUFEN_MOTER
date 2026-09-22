/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5048a.c
  * @brief   AS5048A 14-bit 磁编码器 SPI 驱动实现
  *
  *  协议要点（见 AS5048A 数据手册）：
  *   - 命令字 16 位：bit15=偶校验位，bit14=读(1)/写(0)，bit13:0=14 位地址
  *   - 读操作有 1 帧流水线延迟：发读命令时，MISO 返回的是“上一条命令”的数据，
  *     目标寄存器的数据要到下一次传输才返回。
  *   - 读回数据帧：bit15=PAR(偶校验)，bit14=EF(错误标志)，bit13:0=数据
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "as5048a.h"
#include "spi.h"

/* CS 引脚：PA4（软件 NSS） ----------------------------------------------------*/
#define AS5048A_CS_GPIO   GPIOA
#define AS5048A_CS_PIN    GPIO_PIN_4

#define AS5048A_CS_LOW()   HAL_GPIO_WritePin(AS5048A_CS_GPIO, AS5048A_CS_PIN, GPIO_PIN_RESET)
#define AS5048A_CS_HIGH()  HAL_GPIO_WritePin(AS5048A_CS_GPIO, AS5048A_CS_PIN, GPIO_PIN_SET)

/* 读 NOP 命令字（bit15=1 校验，bit14=1 读，地址 0x0000，偶校验已算好） */
#define AS5048A_CMD_NOP    0xC000u

/**
  * @brief  计算偶校验位：返回 1 表示 value 中“1”的个数为奇数，0 为偶数
  * @param  value: 待计算的值
  * @retval 0/1
  */
static uint16_t AS5048A_EvenParity(uint16_t value)
{
  uint16_t par = value;
  par ^= par >> 8;
  par ^= par >> 4;
  par ^= par >> 2;
  par ^= par >> 1;
  return par & 0x0001u;
}

/**
  * @brief  构造命令字：rw=1 读 / 0 写，addr 为 14 位寄存器地址；bit15 自动填偶校验
  * @param  rw:   1=读，0=写
  * @param  addr: 14 位寄存器地址
  * @retval 16 位命令字
  */
static uint16_t AS5048A_BuildCommand(uint8_t rw, uint16_t addr)
{
  uint16_t cmd = addr & AS5048A_DATA_MASK;

  if (rw)
  {
    cmd |= 0x4000u;                 /* bit14 = 1：读 */
  }
  /* 低 15 位若有奇数个 1，则校验位置 1，使 16 位总数保持偶数 */
  cmd |= (AS5048A_EvenParity(cmd) << 15);

  return cmd;
}

/**
  * @brief  单次 16 位 SPI 收发：拉低 CS → 收发一帧 → 拉高 CS
  * @param  tx: 要发送的 16 位数据
  * @retval 接收到的 16 位数据
  */
static uint16_t AS5048A_Transfer(uint16_t tx)
{
  uint16_t rx = 0;

  AS5048A_CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 100);
  AS5048A_CS_HIGH();

  return rx;
}

/**
  * @brief  初始化：CS 置高（芯片未选中）
  * @retval None
  */
void AS5048A_Init(void)
{
  AS5048A_CS_HIGH();
}

/**
  * @brief  读一个寄存器的原始 16 位帧（含 PAR/EF/数据，不做校验）
  * @param  reg_addr: 14 位寄存器地址
  * @retval 16 位原始帧
  */
uint16_t AS5048A_ReadRegister(uint16_t reg_addr)
{
  uint16_t cmd = AS5048A_BuildCommand(1u, reg_addr);

  AS5048A_Transfer(cmd);                    /* 第 N 帧：发读命令，MISO 是上一条命令的残留 */
  return AS5048A_Transfer(AS5048A_CMD_NOP); /* 第 N+1 帧：MISO 才是目标寄存器数据 */
}

/**
  * @brief  读角度并校验 PAR/EF
  * @param  None
  * @retval 角度 + 校验状态
  */
AS5048A_AngleResult_t AS5048A_ReadAngle(void)
{
  AS5048A_AngleResult_t res;
  uint16_t frame = AS5048A_ReadRegister(AS5048A_REG_ANGLE);

  res.angle = frame & AS5048A_DATA_MASK;

  /* 偶校验：整个 16 位帧“1”的个数应为偶数 */
  if (AS5048A_EvenParity(frame) != 0u)
  {
    res.status = AS5048A_ERR_PARITY;
  }
  else if ((frame & AS5048A_EF_MASK) != 0u)
  {
    res.status = AS5048A_ERR_EF;
  }
  else
  {
    res.status = AS5048A_OK;
  }

  return res;
}

/**
  * @brief  读磁场强度（0~16383，用于判断磁铁距离是否合适）
  * @param  None
  * @retval 14 位磁场幅值
  */
uint16_t AS5048A_ReadMagnitude(void)
{
  return AS5048A_ReadRegister(AS5048A_REG_MAGNITUDE) & AS5048A_DATA_MASK;
}

/**
  * @brief  读 AGC（0~255：255=磁场很弱，0=磁场很强）
  * @param  None
  * @retval AGC 值
  */
uint16_t AS5048A_ReadAGC(void)
{
  return AS5048A_ReadRegister(AS5048A_REG_AGC) & AS5048A_DATA_MASK;
}

/**
  * @brief  清错误标志（读取 0x0001 寄存器即清除）
  * @param  None
  * @retval None
  */
void AS5048A_ClearError(void)
{
  (void)AS5048A_ReadRegister(AS5048A_REG_CLEAR_ERROR);
}
