#ifndef __GYRO_GET_H
#define __GYRO_GET_H

#include <stdint.h>  // 添加这一行来定义 uint8_t 等标准类型
#include "stdio.h"
#include <string.h>
#include "usart.h"
#include "dma.h" 


// 传感器数据结构体
typedef struct {
    int8_t Xl, Xh, Yl, Yh, Zl, Zh;
    int8_t Tl, Th;
} SensorData;

typedef struct {
    float X, Y, Z;
    float T;
} last;

// 完整的陀螺仪数据集
typedef struct {
    last acc;
    last gyro;
    last angle;
    double timestamp;
} GyroDataSet;

// 初始化函数
void gyro_init(void);

// 处理DMA数据函数（需要定期调用）
void gyro_process_dma_data(void);

// 获取最新数据
uint8_t gyro_get_latest_data(GyroDataSet* data);













#endif

