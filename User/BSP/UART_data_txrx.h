/**
 * @file UART_data_txrx.h
 * @author set
 * @brief 串口数据收发驱动（基于DMA+中断实现）
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __UART_DATA_TXRX__
#define __UART_DATA_TXRX__
#include "stm32h7xx.h" 
#include "stm32h7xx_hal_uart.h"
#include "main.h"
#include "usart.h"
#include "struct_typedef.h"
// ===================== 宏定义 =====================
/**
 * @brief 串口接收缓冲区大小（字节）
 * @note  固定256字节，适用于UART1/7/10的接收数据缓存
 */
#define UART_BUFFER_SIZE 256
// ===================== 数据结构体定义 =====================
/**
 * @brief 串口收发数据结构体（包含硬件句柄、缓冲区、状态）
 * @note  PACKED_STRUCT()：结构体按1字节对齐（避免编译器字节填充，保证数据紧凑）
 */
typedef PACKED_STRUCT()
{
  UART_HandleTypeDef *huart;          // 串口句柄指针（如&huart1、&huart7）
  DMA_HandleTypeDef *hdma_usart_rx;   // 接收DMA句柄指针（串口接收关联的DMA）
  DMA_HandleTypeDef *hdma_usart_tx;   // 发送DMA句柄指针（串口发送关联的DMA）

  uint8_t rev_data[UART_BUFFER_SIZE]; // 串口接收缓冲区（大小UART_BUFFER_SIZE=256字节）
  HAL_StatusTypeDef Uart_status;      // 串口通信状态（HAL_OK/HAL_ERROR等）
} transmit_data;

// ===================== 全局变量声明 =====================
extern transmit_data UART7_data;  // UART7的收发数据结构体（包含句柄、缓冲区、状态）
extern transmit_data UART10_data; // UART10的收发数据结构体
extern transmit_data UART1_data;  // UART1的收发数据结构体
// ===================== 外部函数声明（供其他文件调用） =====================
/**
 * @brief 串口全局初始化函数
 * @note  统一初始化所有串口（UART1/7/10）的DMA收发、中断等配置
 */
void uart_init(void);                                                                                                                        

/**
 * @brief 初始化单个串口的DMA收发功能
 * @param  data          串口收发数据结构体指针（存储句柄、缓冲区等）
 * @param  huart         串口句柄指针（如&huart1）
 * @param  hdma_usart_rx 接收DMA句柄指针（如&hdma_usart1_rx）
 * @param  hdma_usart_tx 发送DMA句柄指针（如&hdma_usart1_tx）
 * @retval 无
 * @note   该函数存在重复声明（下方内部函数处再次声明），实际使用以本声明为准
 */
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx); 

/**
 * @brief 串口接收中断处理函数
 * @param  uart  串口收发数据结构体指针（指定处理哪个串口的中断）
 * @retval 无
 * @note   需在串口中断服务函数中调用，处理接收完成、溢出等中断事件
 */
void UART_rx_IRQHandler(transmit_data *uart);                                                                                                

/**
 * @brief 串口发送数据（基于DMA/轮询，具体由底层实现）
 * @param  uart  串口收发数据结构体（指定发送使用的串口）
 * @param  data  待发送的数据缓冲区指针
 * @param  size  待发送数据的字节长度
 * @retval 无
 */
void UART_send_data(transmit_data uart, uint8_t data[], uint16_t size);                                                                      

// ===================== 内部函数声明（仅本文件内部使用，实际与外部函数重复） =====================
/**
 * @brief 初始化单个串口的DMA收发功能（重复声明）
 * @note  与上方外部函数声明完全一致，建议删除本声明避免重复
 */
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx);

#endif
// end of file
