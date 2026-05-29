#ifndef PT_SENSOR_H
#define PT_SENSOR_H

#include "stdint.h"

// 提前声明串口句柄结构体（避免未包含HAL库头文件时编译报错）
#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct __UART_HandleTypeDef UART_HandleTypeDef;
#ifdef __cplusplus
}
#endif

/************************* 协议核心宏定义 *************************/
// 缓存大小
#define PT_BUF_MAX_LEN       256U

// 协议固定指令帧
#define PT_CMD_FRAME_TEMP  {0x55, 0x04, 0x0E, 0x6A}  // 读温度完整指令帧
#define PT_CMD_FRAME_PRESS {0x55, 0x04, 0x0D, 0x88}  // 读压力完整指令帧

/************************* 全局变量声明 *************************/
extern uint8_t pt_raw_buf[PT_BUF_MAX_LEN];
extern float g_pressure_value;

/************************* 核心函数声明 *************************/
void pt_store_raw_data(const uint8_t *data, uint16_t size);
uint8_t PT_ParseLatestPressure(void);
void PT_Send_ReadTemp_Cmd(UART_HandleTypeDef *huart);
void PT_Send_ReadPress_Cmd(UART_HandleTypeDef *huart);
void PT_ParsePressureToGlobal(uint8_t *buf, uint16_t size);

#endif // PT_SENSOR_H
