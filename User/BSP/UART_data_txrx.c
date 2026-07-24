/**
 * @file UART_data_txrx.c
 * @author sethome
 * @brief 串口数据发送接受
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022 sethome
 *
 */
// #include "fifo.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "fifo.h"
#include "UART_data_txrx.h"
#include "gom_protocol.h"
#include "Sbus.h"
#include "stp23l.h"
#include "pt_sensor.h"
#include "uart_protocol.h"
#include "kvms_battery.h"
// DMA控制变量
extern DMA_HandleTypeDef hdma_uart5_rx; // 遥控器，仅用接受
extern DMA_HandleTypeDef hdma_uart7_rx; // 串口7
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart10_rx; // 串口10
extern DMA_HandleTypeDef hdma_usart10_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// 串口控制变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5; // 遥控器，可能用不到
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart10;

// 将上述串口+DMA整合，并包含缓冲区
transmit_data UART1_data;
transmit_data UART2_data;
transmit_data UART5_data;
transmit_data UART7_data;
transmit_data UART10_data;

/**
 * @brief 串口初始化
 *
 * @return * void
 */
void uart_init(void)
{
  UART_DMA_rxtx_start(&UART1_data, &huart1, &hdma_usart1_rx, &hdma_usart1_rx);
  // UART_DMA_rxtx_start(&UART2_data, &huart2, &hdma_usart2_rx, &hdma_usart2_rx);
  UART_DMA_rxtx_start(&UART5_data, &huart5, &hdma_uart5_rx, &hdma_uart5_rx);
  UART_DMA_rxtx_start(&UART7_data, &huart7, &hdma_uart7_rx, &hdma_uart7_tx);
  UART_DMA_rxtx_start(&UART10_data, &huart10, &hdma_usart10_rx, &hdma_usart10_tx);


}

/**
 * @brief DMA，串口中断启动，ヾ(?ω?`)o温馨提示，可以在头文件里用宏自定义串口缓冲区大小，
 * @param data 串口整合包指针
 * @param huart 串口指针
 * @param hdma_usart_rx 串口接受dma指针
 * @param hdma_usart_tx 串口发送dma指针
 */
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx)
{
  data->huart = huart;                 // 串口控制变量
  data->hdma_usart_rx = hdma_usart_rx; // DMA接收缓冲
  data->hdma_usart_tx = hdma_usart_tx; // DMA发送缓冲

  HAL_UARTEx_ReceiveToIdle_DMA(huart, data->rev_data, UART_BUFFER_SIZE); // 开启DMA批量数据接受
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);                        // 关闭接受过半中断
}

/**
 * @brief 串口发送数据
 *
 * @param uart 发送串口整合包
 * @param data 发送数据（数据别释放了，不然后面收不到）
 * @param size 数据大小
 */
void UART_send_data(transmit_data uart, uint8_t data[], uint16_t size)
{
  //+++++++++++++++//while(HAL_DMA_GetState(UART6_data.hdma_usart_tx) != HAL_DMA_STATE_READY)
  HAL_UART_Transmit_DMA(uart.huart, data, size); // 开启DMA批量数据发送
}

/**
 * @brief 串口接受空回调函数，用于接受不定长数据，放置数据处理函数
 *
 * @param huart 接受到数据的串口
 * @param Size 数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)//气路气压
  {
    pt_store_raw_data(UART1_data.rev_data, Size); // 存储数据
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart5) // 遥控器
  {
    store_sbus_data(UART5_data.rev_data, Size); // 收到数据进行储存
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart7) // 升降杆高度读取
  {
    store_stp23l_data(UART7_data.rev_data, Size); // 只存储，不解析
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart10) // 和主机间通信
  {
    store_uart_protocol_data(UART10_data.rev_data, Size); // 存储数据
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart3)
  {
    KvmsBattery_OnRxEvent(huart, Size);
  }
}

/**
 * @brief 串口接受完成中断回调函数，用于接受定长数据
 *
 * @param huart 接受串口号
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

// 发生错误重启串口
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart10)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart5)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart1)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart3)
  {
    KvmsBattery_RestartRx();
  }
}
// end of file
