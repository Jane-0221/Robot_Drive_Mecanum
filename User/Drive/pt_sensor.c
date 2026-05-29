#include "pt_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "usart.h"
#include  "stdio.h"
/************************* 全局变量实例化 *************************/
uint8_t pt_raw_buf[PT_BUF_MAX_LEN] = {0};
float g_pressure_value = 0.0f;
static uint8_t pt_parse_buf[PT_BUF_MAX_LEN];
static volatile uint16_t pt_raw_size = 0U;
static volatile uint8_t pt_data_pending = 0U;
static const uint8_t pt_temp_cmd[] = PT_CMD_FRAME_TEMP;
static const uint8_t pt_press_cmd[] = PT_CMD_FRAME_PRESS;




/************************* 存储串口原始数据 *************************/
void pt_store_raw_data(const uint8_t *data, uint16_t size)
{
    if(data == NULL || size == 0)
    {
        return;
    }

    // 限制缓存大小，防止溢出
    uint16_t copy_len = (size > PT_BUF_MAX_LEN) ? PT_BUF_MAX_LEN : size;
    memcpy(pt_raw_buf, data, copy_len);
    pt_raw_size = copy_len;
    pt_data_pending = 1U;
}

/**
 * @brief  发送读取温度指令（HAL库）
 * @param  huart: 串口句柄指针（如&huart1、&huart2）
 * @retval 无
 * @note   指令帧：55 04 0E 6A，超时时间100ms
 */
uint8_t PT_ParseLatestPressure(void)
{
    uint16_t size;

    if (pt_data_pending == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    size = pt_raw_size;
    memcpy(pt_parse_buf, pt_raw_buf, size);
    pt_data_pending = 0U;
    taskEXIT_CRITICAL();

    PT_ParsePressureToGlobal(pt_parse_buf, size);
    return 1U;
}

void PT_Send_ReadTemp_Cmd(UART_HandleTypeDef *huart)
{
    // 空指针校验，避免传入NULL导致程序崩溃
    if(huart == NULL)
    {
        return;
    }
    
    // 读取温度的指令帧（来自pt_sensor.h的宏定义）
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return;
    }
    
    // 发送4字节指令（HAL库阻塞式发送，超时100ms）
    HAL_UART_Transmit_DMA(huart, (uint8_t *)pt_temp_cmd, sizeof(pt_temp_cmd));
}

/**
 * @brief  发送读取压力指令（HAL库）
 * @param  huart: 串口句柄指针（如&huart1、&huart2）
 * @retval 无
 * @note   指令帧：55 04 0D 88，超时时间100ms；需先发送温度指令再调用此函数
 */
void PT_Send_ReadPress_Cmd(UART_HandleTypeDef *huart)
{
    // 空指针校验
    if(huart == NULL)
    {
        return;
    }
    
    // 读取压力的指令帧（来自pt_sensor.h的宏定义）
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return;
    }
    
    // 发送4字节指令
    HAL_UART_Transmit_DMA(huart, (uint8_t *)pt_press_cmd, sizeof(pt_press_cmd));
}

void PT_ParsePressureToGlobal(uint8_t *buf, uint16_t size)
{
    // 初始化全局变量为0（解析失败时保持0.0f）
    g_pressure_value = 0.0f;

    // 参数校验：空指针/无效长度直接返回
    if(buf == NULL || size == 0 || size > 256)
    {
        return;
    }

    uint16_t i = 0;
    // 遍历缓存查找压力帧（AA 08 09开头）
    while(i < size)
    {
        // 定位帧头0xAA
        if(buf[i] != 0xAA)
        {
            i++;
            continue;
        }

        // 验证剩余长度够识别压力帧（至少3字节）
        if((i + 3) > size)
        {
            break;
        }

        // 识别压力帧特征：AA 08 09
        if(buf[i+1] == 0x08 && buf[i+2] == 0x09)
        {
            // 验证帧完整性（完整压力帧8字节）
            if((i + 8) > size)
            {
                i++;
                continue;
            }

            // 提取4字节压力原始值（小端序转大端）
            uint8_t *frame = &buf[i];
            uint32_t press_raw = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3];
            
            // 换算为实际气压值（×0.001kPa），存入全局变量
            g_pressure_value = (float)press_raw * 0.001f;
        }

        i++;
    }
}
