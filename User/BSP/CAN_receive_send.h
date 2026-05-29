#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__
#include "fdcan.h"

// ===================== 类型别名定义 =====================
/**
 * @brief CAN控制器句柄类型别名（简化代码书写）
 * @note  等价于FDCAN_HandleTypeDef，仅为代码简洁性定义，无其他功能
 */
typedef FDCAN_HandleTypeDef hcan_t;

typedef struct
{
  uint32_t sample_count;
  uint32_t full_count;
  uint32_t add_fail_count;
  uint32_t last_tick;
  uint32_t last_identifier;
  uint32_t last_id_type;
  uint32_t last_free_level;
  uint32_t min_free_level;
  uint32_t last_hal_status;
  uint32_t software_queue_depth;
  uint32_t software_queue_max_depth;
  uint32_t software_enqueue_count;
  uint32_t software_dequeue_count;
  uint32_t software_drop_old_count;
  uint32_t hardware_fifo_full_count;
  uint32_t hal_fail_count;
  uint32_t invalid_param_count;
} CanTxFifoDebug_t;

extern volatile CanTxFifoDebug_t can_tx_fifo_debug[3];
extern volatile uint32_t can2_std_rx_total_debug;
extern volatile uint32_t can2_ext_rx_total_debug;
extern volatile uint32_t can2_std_rx_unknown_debug;
extern volatile uint32_t can2_std_last_id_debug;
extern volatile uint32_t can2_std_recent_id_debug[16];
extern volatile uint32_t can2_std_recent_index_debug;
extern volatile uint32_t can2_fifo0_full_debug;
extern volatile uint32_t can2_fifo0_lost_debug;
extern volatile uint32_t can2_fifo1_full_debug;
extern volatile uint32_t can2_fifo1_lost_debug;

// ===================== 函数声明 =====================
/**
 * @brief CAN总线初始化函数（占位函数）
 * @note  实际初始化由HAL库自动生成的MX_FDCANx_Init完成，本函数仅做声明占位
 */
extern void can_init(void);

void CAN_TxFifoDebug_Reset(void);
void CAN_TxFifoDebug_RecordBeforeSend(FDCAN_HandleTypeDef *hcan, const FDCAN_TxHeaderTypeDef *tx_header);
void CAN_TxFifoDebug_RecordAddResult(FDCAN_HandleTypeDef *hcan, HAL_StatusTypeDef status);
HAL_StatusTypeDef CAN_TxQueueFrame(FDCAN_HandleTypeDef *hcan, const FDCAN_TxHeaderTypeDef *tx_header, const uint8_t *data, uint32_t len);
void CAN_TxProcess(FDCAN_HandleTypeDef *hcan);

/**
 * @brief 发送CAN标准帧（11位ID）
 * @param  hcan   CAN控制器句柄指针（如&hfdcan1、&hfdcan2）
 * @param  id     CAN标准帧ID（11位，取值0~0x7FF）
 * @param  data   待发送数据缓冲区指针
 * @param  len    待发送数据字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 */
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

/**
 * @brief 发送CAN扩展帧（29位ID）
 * @param  hcan   CAN控制器句柄指针（如&hfdcan1、&hfdcan2）
 * @param  id     CAN扩展帧ID（29位，取值0~0x1FFFFFFF）
 * @param  data   待发送数据缓冲区指针
 * @param  len    待发送数据字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 */
extern uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len);

/**
 * @brief CAN数据接收函数（仅占位，不建议使用）
 * @param  hfdcan        CAN控制器句柄指针
 * @param  RXFIFO        接收FIFO编号（FDCAN_RX_FIFO0/FDCAN_RX_FIFO1）
 * @param  fdcan_RxHeader 接收帧头信息存储结构体指针
 * @param  buf           接收数据缓冲区指针
 * @retval uint8_t 始终返回0（函数逻辑不完整，仅做声明）
 * @warning 建议优先使用中断回调函数处理接收数据，而非本函数
 */
extern uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan,uint32_t RXFIFO,FDCAN_RxHeaderTypeDef *fdcan_RxHeader,uint8_t *buf);

/**
 * @brief CAN1发送电机电流控制指令
 * @note  向CAN1总线挂载的电机发送电流控制数据，实现电机转速/力矩控制
 */
extern void CAN1_send_current(void);

/**
 * @brief CAN2发送电机电流控制指令
 * @note  向CAN2总线挂载的电机发送电流控制数据，实现电机转速/力矩控制
 */
extern void CAN2_send_current(void);

/**
 * @brief CAN3发送电机电流控制指令
 * @note  向CAN3总线挂载的电机发送电流控制数据，实现电机转速/力矩控制
 */
extern void CAN3_send_current(void);

/**
 * @brief CAN错误回调函数（CAN通信出错时触发）
 * @param  hfdcan  出错的CAN控制器句柄指针
 * @note  功能：重启CAN控制器 + 重新配置过滤器，实现通信错误自恢复
 */
extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);

#endif /* __CAN_RECEIVE_SEND_H__ */
