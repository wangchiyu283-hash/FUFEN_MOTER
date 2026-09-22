#include "gyro_get.h"
#include "main.h"

// 模块内部变量
static SensorData stcAcc, stcGyro, stcAngle;
static uint32_t last_update_time = 0;
static uint8_t data_ready = 0;

// DMA接收缓冲区
#define GYRO_RX_BUFFER_SIZE 256
static uint8_t gyro_rx_buffer[GYRO_RX_BUFFER_SIZE];
static uint32_t dma_prev_pos = 0;

static void gyro_parse_byte(uint8_t data);
// 解析状态机
static struct {
    uint8_t rx_buffer[11];
    uint8_t rx_cnt;
    uint8_t data_type; // 0:等待, 1:acc收到, 2:gyro收到, 3:angle收到
} parser_state = {0};



// 初始化陀螺仪DMA接收
void gyro_init(void)
{
    printf ("gyro_Init is OK\r\n");
    // 启动DMA接收（循环模式）
    HAL_UART_Receive_DMA(&huart3, gyro_rx_buffer, GYRO_RX_BUFFER_SIZE);
    dma_prev_pos = 0;
    
    // 重置解析状态
    memset(&parser_state, 0, sizeof(parser_state));
    memset(&stcAcc, 0, sizeof(stcAcc));
    memset(&stcGyro, 0, sizeof(stcGyro));
    memset(&stcAngle, 0, sizeof(stcAngle));
    
    data_ready = 0;
    last_update_time = HAL_GetTick();
}

// 处理DMA接收的数据（需要定期调用，比如每1ms）
void gyro_process_dma_data(void)
{
    // 获取当前DMA写入位置
    uint32_t dma_current_pos = GYRO_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);//剩余数据__HAL_DMA_GET_COUNTER(huart3.hdmarx，，已写入dma_current_pos
    
    // 如果没有新数据，直接返回
    if (dma_current_pos == dma_prev_pos) {
       printf ("NO Data\r\n");
        return;
    }
    
    // 处理新接收的数据
    uint32_t data_to_process;
    if (dma_current_pos >= dma_prev_pos) {
        data_to_process = dma_current_pos - dma_prev_pos;//新入数据数量
    } else {
        // DMA缓冲区回绕
        data_to_process = (GYRO_RX_BUFFER_SIZE - dma_prev_pos) + dma_current_pos;//缓冲区数据回绕
    }
    
    // 逐个字节处理
    for (uint32_t i = 0; i < data_to_process; i++) {
        uint32_t index = (dma_prev_pos + i) % GYRO_RX_BUFFER_SIZE;
        gyro_parse_byte(gyro_rx_buffer[index]);
        if(data_ready)
        {
           break;
        }
    }
    
    dma_prev_pos = dma_current_pos;
}

// 解析单个字节
static void gyro_parse_byte(uint8_t data)
{
    parser_state.rx_buffer[parser_state.rx_cnt++] = data;
    
    // 检查帧头
    if (parser_state.rx_buffer[0] != 0x55) {
        parser_state.rx_cnt = 0;
        return;
    }
    
    // 检查数据长度
    if (parser_state.rx_cnt < 11) {
        return;
    }
    
    // 完整帧接收完成，进行解析
    switch(parser_state.rx_buffer[1]) {
        case 0x51: // 加速度数据
            memcpy(&stcAcc, &parser_state.rx_buffer[2], 8);
            parser_state.data_type = 1;
            break;
        case 0x52: // 陀螺仪数据
            memcpy(&stcGyro, &parser_state.rx_buffer[2], 8);
            parser_state.data_type = 2;
            break;
        case 0x53: // 角度数据
            memcpy(&stcAngle, &parser_state.rx_buffer[2], 8);
            parser_state.data_type = 3;
            last_update_time = HAL_GetTick();
            data_ready = 1; // 标记有新数据
            break;
    }
    
    parser_state.rx_cnt = 0;
}


// 获取最新的陀螺仪数据
uint8_t gyro_get_latest_data(GyroDataSet* data)
{
    if (data_ready) {
       data->acc.X = (float)((stcAcc.Xh << 8) | (stcAcc.Xl)) / 32768.0*16;
       data->acc.Y = (float)((stcAcc.Yh << 8) | (stcAcc.Yl)) / 32768.0*16;
       data->acc.Z = (float)((stcAcc.Zh << 8) | (stcAcc.Zl)) / 32768.0*16;
       data->gyro.X =(float)((stcGyro.Xh << 8) | (stcGyro.Xl)) / 32768.0*2000;
       data->gyro.Y =(float)((stcGyro.Yh << 8) | (stcGyro.Yl)) / 32768.0*2000;
       data->gyro.Z =(float)((stcGyro.Zh << 8) | (stcGyro.Zl)) / 32768.0*2000;
       data->angle.X = (float)((stcAngle.Xh << 8) | (stcAngle.Xl)) / 32768.0*180;
       data->angle.Y = (float)((stcAngle.Yh << 8) | (stcAngle.Yl)) / 32768.0*180;
       data->angle.Z = (float)((stcAngle.Zh << 8) | (stcAngle.Zl)) / 32768.0*180;
       data->timestamp = last_update_time;
       data_ready = 0; // 清除就绪标志
       return 1;
    }
    return 0;
}
