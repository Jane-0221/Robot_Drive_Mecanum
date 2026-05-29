/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief CAN总线底层驱动（BSP层）：实现CAN帧发送、接收、中断回调等核心功能
 * @version 0.2
 * @date 2024-10-19
 * @copyright Copyright (c) 2024
 */
#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "string.h"
#include "Robstride04.h"
#include "arm.h"
#include "chassis.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "music.h"
#include "LED.h"
#include <cmsis_os2.h>
#include "iwdg.h"
#include "buzzer.h"
#include "fdcan.h"
#include "ktech_motor.h"
#include "head.h"
#include "servo_lift_control.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#define CAN_TX_BUS_COUNT 3U
#define CAN_TX_QUEUE_DEPTH 32U
#define CAN_TX_DATA_BYTES 8U

typedef struct
{
  FDCAN_TxHeaderTypeDef header;
  uint8_t data[CAN_TX_DATA_BYTES];
} CanTxQueuedFrame_t;

typedef struct
{
  CanTxQueuedFrame_t frame[CAN_TX_QUEUE_DEPTH];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} CanTxQueue_t;

static CanTxQueue_t can_tx_queue[CAN_TX_BUS_COUNT];
static volatile uint8_t can_tx_processing[CAN_TX_BUS_COUNT] = {0U};

volatile CanTxFifoDebug_t can_tx_fifo_debug[3] = {
    {.min_free_level = 0xFFFFFFFFU},
    {.min_free_level = 0xFFFFFFFFU},
    {.min_free_level = 0xFFFFFFFFU},
};

volatile uint32_t can2_std_rx_total_debug = 0U;
volatile uint32_t can2_ext_rx_total_debug = 0U;
volatile uint32_t can2_std_rx_unknown_debug = 0U;
volatile uint32_t can2_std_last_id_debug = 0U;
volatile uint32_t can2_std_recent_id_debug[16] = {0U};
volatile uint32_t can2_std_recent_index_debug = 0U;
volatile uint32_t can2_fifo0_full_debug = 0U;
volatile uint32_t can2_fifo0_lost_debug = 0U;
volatile uint32_t can2_fifo1_full_debug = 0U;
volatile uint32_t can2_fifo1_lost_debug = 0U;

static void CAN_ArmFeedbackDebug_Record(uint8_t logical_motor);

static void CAN2_StdRxDebug_Record(uint32_t identifier)
{
  uint32_t index = can2_std_recent_index_debug & 0x0FU;

  can2_std_rx_total_debug++;
  can2_std_last_id_debug = identifier;
  can2_std_recent_id_debug[index] = identifier;
  can2_std_recent_index_debug++;
}

static void CAN2_RobStrideExtFrame_Process(uint32_t identifier, uint8_t *rx_data)
{
  can2_ext_rx_total_debug++;

  uint8_t target_id = (uint8_t)((identifier >> 8) & 0xFF);
  if (target_id == 0x01)
  {
    RobStride_Motor_Analysis(&motor1, rx_data, identifier);
    CAN_ArmFeedbackDebug_Record(0U);
  }
  else if (target_id == 0x02)
  {
    RobStride_Motor_Analysis(&motor2, rx_data, identifier);
    CAN_ArmFeedbackDebug_Record(1U);
  }
  else if (target_id == 0x03)
  {
    RobStride_Motor_Analysis(&motor3, rx_data, identifier);
    CAN_ArmFeedbackDebug_Record(2U);
  }
}

static uint8_t KTech_FeedbackHasEncoder(uint8_t cmd)
{
  switch (cmd)
  {
  case KTECH_CMD_READ_STATUS2:
  case KTECH_CMD_OPENLOOP:
  case KTECH_CMD_TORQUE_CLOSED:
  case KTECH_CMD_SPEED_CLOSED:
  case KTECH_CMD_POS_MULTI1:
  case KTECH_CMD_POS_MULTI2:
  case KTECH_CMD_POS_SINGLE1:
  case KTECH_CMD_POS_SINGLE2:
  case KTECH_CMD_POS_INCREMENT1:
  case KTECH_CMD_POS_INCREMENT2:
  case KTECH_CMD_READ_ENCODER:
    return 1U;

  default:
    return 0U;
  }
}

static int32_t CAN_TxFifoDebug_GetIndex(FDCAN_HandleTypeDef *hcan)
{
  if (hcan == NULL)
  {
    return -1;
  }

  if (hcan->Instance == FDCAN1)
  {
    return 0;
  }

  if (hcan->Instance == FDCAN2)
  {
    return 1;
  }

  if (hcan->Instance == FDCAN3)
  {
    return 2;
  }

  return -1;
}

static void CAN_TxCriticalEnter(uint32_t *primask)
{
  if (primask == NULL)
  {
    return;
  }

  *primask = __get_PRIMASK();
  __disable_irq();
}

static void CAN_TxCriticalExit(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void CAN_TxDebug_RecordInvalid(FDCAN_HandleTypeDef *hcan)
{
  int32_t index = CAN_TxFifoDebug_GetIndex(hcan);

  if (index >= 0)
  {
    can_tx_fifo_debug[index].invalid_param_count++;
  }
}

static uint8_t CAN_TxProcess_Begin(uint32_t index)
{
  uint8_t can_process = 0U;
  uint32_t primask;

  if (index >= CAN_TX_BUS_COUNT)
  {
    return 0U;
  }

  CAN_TxCriticalEnter(&primask);
  if (can_tx_processing[index] == 0U)
  {
    can_tx_processing[index] = 1U;
    can_process = 1U;
  }
  CAN_TxCriticalExit(primask);

  return can_process;
}

static void CAN_TxProcess_End(uint32_t index)
{
  uint32_t primask;

  if (index >= CAN_TX_BUS_COUNT)
  {
    return;
  }

  CAN_TxCriticalEnter(&primask);
  can_tx_processing[index] = 0U;
  CAN_TxCriticalExit(primask);
}

static void CAN_ArmFeedbackDebug_Record(uint8_t logical_motor)
{
  if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
  {
    return;
  }

  arm_feedback_count_debug[logical_motor]++;
  arm_feedback_last_tick_debug[logical_motor] = HAL_GetTick();
}

void CAN_TxFifoDebug_Reset(void)
{
  uint32_t primask;

  CAN_TxCriticalEnter(&primask);
  memset(can_tx_queue, 0, sizeof(can_tx_queue));
  memset((void *)can_tx_processing, 0, sizeof(can_tx_processing));

  for (uint32_t i = 0; i < CAN_TX_BUS_COUNT; i++)
  {
    can_tx_fifo_debug[i].sample_count = 0U;
    can_tx_fifo_debug[i].full_count = 0U;
    can_tx_fifo_debug[i].add_fail_count = 0U;
    can_tx_fifo_debug[i].last_tick = 0U;
    can_tx_fifo_debug[i].last_identifier = 0U;
    can_tx_fifo_debug[i].last_id_type = 0U;
    can_tx_fifo_debug[i].last_free_level = 0U;
    can_tx_fifo_debug[i].min_free_level = 0xFFFFFFFFU;
    can_tx_fifo_debug[i].last_hal_status = (uint32_t)HAL_OK;
    can_tx_fifo_debug[i].software_queue_depth = 0U;
    can_tx_fifo_debug[i].software_queue_max_depth = 0U;
    can_tx_fifo_debug[i].software_enqueue_count = 0U;
    can_tx_fifo_debug[i].software_dequeue_count = 0U;
    can_tx_fifo_debug[i].software_drop_old_count = 0U;
    can_tx_fifo_debug[i].hardware_fifo_full_count = 0U;
    can_tx_fifo_debug[i].hal_fail_count = 0U;
    can_tx_fifo_debug[i].invalid_param_count = 0U;
  }
  CAN_TxCriticalExit(primask);
}

void CAN_TxFifoDebug_RecordBeforeSend(FDCAN_HandleTypeDef *hcan, const FDCAN_TxHeaderTypeDef *tx_header)
{
  int32_t index = CAN_TxFifoDebug_GetIndex(hcan);
  uint32_t free_level;
  volatile CanTxFifoDebug_t *debug;

  if (index < 0)
  {
    return;
  }

  free_level = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
  debug = &can_tx_fifo_debug[index];

  debug->sample_count++;
  debug->last_tick = HAL_GetTick();
  debug->last_identifier = (tx_header != NULL) ? tx_header->Identifier : 0U;
  debug->last_id_type = (tx_header != NULL) ? tx_header->IdType : 0U;
  debug->last_free_level = free_level;

  if (free_level < debug->min_free_level)
  {
    debug->min_free_level = free_level;
  }

  if (free_level == 0U)
  {
    debug->full_count++;
    debug->hardware_fifo_full_count++;
  }
}

void CAN_TxFifoDebug_RecordAddResult(FDCAN_HandleTypeDef *hcan, HAL_StatusTypeDef status)
{
  int32_t index = CAN_TxFifoDebug_GetIndex(hcan);
  volatile CanTxFifoDebug_t *debug;

  if (index < 0)
  {
    return;
  }

  debug = &can_tx_fifo_debug[index];
  debug->last_hal_status = (uint32_t)status;

  if (status != HAL_OK)
  {
    debug->add_fail_count++;
    debug->hal_fail_count++;
  }
}

/**
 * @brief CAN总线初始化函数（占位函数）
 * @note  1. 实际CAN控制器初始化由HAL库自动生成的MX_FDCANx_Init函数完成（在fdcan.c中）；
 *        2. 本函数仅做声明占位，无实际初始化逻辑，可根据需求补充自定义初始化；
 *        3. 中断配置、过滤器配置等核心初始化逻辑在HAL_FDCAN_ErrorCallback中也有兜底处理。
 */
HAL_StatusTypeDef CAN_TxQueueFrame(FDCAN_HandleTypeDef *hcan, const FDCAN_TxHeaderTypeDef *tx_header, const uint8_t *data, uint32_t len)
{
  int32_t index = CAN_TxFifoDebug_GetIndex(hcan);
  CanTxQueuedFrame_t queued_frame;
  CanTxQueue_t *queue;
  volatile CanTxFifoDebug_t *debug;
  uint32_t primask;

  if ((index < 0) || (tx_header == NULL) || (len > CAN_TX_DATA_BYTES) || ((data == NULL) && (len > 0U)))
  {
    CAN_TxDebug_RecordInvalid(hcan);
    return HAL_ERROR;
  }

  memset(&queued_frame, 0, sizeof(queued_frame));
  queued_frame.header = *tx_header;
  if ((data != NULL) && (len > 0U))
  {
    memcpy(queued_frame.data, data, len);
  }

  queue = &can_tx_queue[index];
  debug = &can_tx_fifo_debug[index];

  CAN_TxCriticalEnter(&primask);
  if (queue->count >= CAN_TX_QUEUE_DEPTH)
  {
    queue->tail = (uint8_t)((queue->tail + 1U) % CAN_TX_QUEUE_DEPTH);
    queue->count--;
    debug->software_drop_old_count++;
  }

  queue->frame[queue->head] = queued_frame;
  queue->head = (uint8_t)((queue->head + 1U) % CAN_TX_QUEUE_DEPTH);
  queue->count++;
  debug->software_enqueue_count++;
  debug->software_queue_depth = queue->count;
  if (queue->count > debug->software_queue_max_depth)
  {
    debug->software_queue_max_depth = queue->count;
  }
  CAN_TxCriticalExit(primask);

  CAN_TxProcess(hcan);

  return HAL_OK;
}

void CAN_TxProcess(FDCAN_HandleTypeDef *hcan)
{
  int32_t index = CAN_TxFifoDebug_GetIndex(hcan);
  CanTxQueuedFrame_t queued_frame;
  HAL_StatusTypeDef tx_status;
  uint32_t free_level;
  uint32_t primask;

  if (index < 0)
  {
    return;
  }

  if (CAN_TxProcess_Begin((uint32_t)index) == 0U)
  {
    return;
  }

  for (;;)
  {
    free_level = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
    if (free_level == 0U)
    {
      can_tx_fifo_debug[index].sample_count++;
      can_tx_fifo_debug[index].last_tick = HAL_GetTick();
      can_tx_fifo_debug[index].last_free_level = 0U;
      can_tx_fifo_debug[index].full_count++;
      can_tx_fifo_debug[index].hardware_fifo_full_count++;
      can_tx_fifo_debug[index].min_free_level = 0U;
      break;
    }

    CAN_TxCriticalEnter(&primask);
    if (can_tx_queue[index].count == 0U)
    {
      can_tx_fifo_debug[index].software_queue_depth = 0U;
      CAN_TxCriticalExit(primask);
      break;
    }
    queued_frame = can_tx_queue[index].frame[can_tx_queue[index].tail];
    can_tx_queue[index].tail = (uint8_t)((can_tx_queue[index].tail + 1U) % CAN_TX_QUEUE_DEPTH);
    can_tx_queue[index].count--;
    can_tx_fifo_debug[index].software_dequeue_count++;
    can_tx_fifo_debug[index].software_queue_depth = can_tx_queue[index].count;
    CAN_TxCriticalExit(primask);

    CAN_TxFifoDebug_RecordBeforeSend(hcan, &queued_frame.header);
    tx_status = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &queued_frame.header, queued_frame.data);
    CAN_TxFifoDebug_RecordAddResult(hcan, tx_status);
    if (tx_status != HAL_OK)
    {
      break;
    }
  }

  CAN_TxProcess_End((uint32_t)index);
}

void can_init(void)
{
  CAN_TxFifoDebug_Reset();

  (void)HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);
  (void)HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, 0);
  (void)HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_FIFO_EMPTY, 0);

  // 实际初始化由 HAL 库自动生成的 MX_FDCANx_Init 完成，此处仅占位
  // 若需自定义初始化（如过滤器、中断），可在此补充
}

/**
 * @brief 发送CAN标准帧（11位ID）
 * @param  hcan   CAN控制器句柄（如hfdcan1或hfdcan2）
 * @param  id     CAN标准帧ID（11位，取值0~0x7FF）
 * @param  data   待发送的数据缓冲区指针
 * @param  len    待发送数据的字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 * @note   1. 数据长度会自动映射为FDCAN标准DLC值（如len<=8时按8字节发送）；
 *         2. 发送失败会触发Error_Handler错误处理函数。
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 设置CAN标准帧ID
  TxHeader.IdType = FDCAN_STANDARD_ID;       // 帧类型：标准帧（11位ID）
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 帧类型：数据帧（非远程帧）

  // 数据长度映射（FDCAN仅支持固定DLC长度，不足则补零，超出则返回错误）
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else
    return 1; // 不支持的长度，返回错误

  // 固定配置：错误状态激活、关闭位速率切换、经典CAN格式、无发送事件、消息标记
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 将消息添加到发送FIFO队列，失败则触发错误处理
  return (CAN_TxQueueFrame(hcan, &TxHeader, data, len) == HAL_OK) ? 0U : 1U;
}

/**
 * @brief 发送CAN扩展帧（29位ID）
 * @param  hcan   CAN控制器句柄（如hfdcan1或hfdcan2）
 * @param  id     CAN扩展帧ID（29位，取值0~0x1FFFFFFF）
 * @param  data   待发送的数据缓冲区指针
 * @param  len    待发送数据的字节长度
 * @retval uint8_t 发送状态：0=成功，1=不支持的长度（非8/12/16/20/24/48/64字节）
 * @note   逻辑与标准帧发送一致，仅帧ID类型为扩展帧（29位）。
 */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;                  // 设置CAN扩展帧ID
  TxHeader.IdType = FDCAN_EXTENDED_ID;       // 帧类型：扩展帧（29位ID）
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 帧类型：数据帧

  // 数据长度映射（与标准帧逻辑一致）
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else
    return 1; // 不支持的长度，返回错误

  // 固定配置（与标准帧一致）
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  // 添加到发送FIFO队列，失败则触发错误处理
  return (CAN_TxQueueFrame(hcan, &TxHeader, data, len) == HAL_OK) ? 0U : 1U;
}

/**
 * @brief CAN数据接收函数（仅占位，不建议使用）
 * @param  hfdcan    CAN控制器句柄
 * @param  RXFIFO    接收FIFO编号（FDCAN_RX_FIFO0/FDCAN_RX_FIFO1）
 * @param  fdcan_RxHeader  接收帧头信息存储结构体指针
 * @param  buf       接收数据缓冲区指针
 * @retval uint8_t 始终返回0（原逻辑无有效返回值）
 * @warning 1. 本函数逻辑不完整，仅调用HAL_FDCAN_GetRxMessage但未处理返回值；
 *          2. 原代码中“DataLength>>16”为无效逻辑（已注释），实际无数据长度解析；
 *          3. 建议优先使用中断回调函数（HAL_FDCAN_RxFifo0Callback）处理接收数据，而非本函数。
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RXFIFO, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  // 尝试从指定FIFO读取数据，失败则直接返回0（无错误处理）
  if (HAL_FDCAN_GetRxMessage(hfdcan, RXFIFO, fdcan_RxHeader, buf) != HAL_OK)
    return 0;
  // 原代码中“DataLength>>16”是错误逻辑（DataLength无高16位有效数据），此处注释弃用
  return 0;
}

/**
 * @brief CAN接收中断回调函数（FDCAN RX FIFO0 中断）
 * @param  hfdcan    触发中断的CAN控制器句柄
 * @param  RxFifo0ITs 中断类型标志（本函数仅处理新消息中断）
 * @note   1. 循环读取FIFO0中的所有新消息，直到FIFO为空；
 *         2. 按CAN控制器（FDCAN1/FDCAN2）和帧ID分类处理不同电机的反馈数据；
 *         3. FDCAN3未在本函数中处理，可根据需求补充。
 */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
  CAN_TxProcess(hfdcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance == FDCAN2)
  {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U)
    {
      can2_fifo0_full_debug++;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
      can2_fifo0_lost_debug++;
    }
  }

  FDCAN_RxHeaderTypeDef rx_header; // 存储接收帧头信息
  uint8_t rx_data[8];              // 接收数据缓冲区（默认8字节）

  // 仅处理“FIFO0有新消息”中断，其他中断直接返回
  if ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) == 0)
    return;

  // 循环读取FIFO0中的所有消息（直到读取失败，即FIFO为空）
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
    // ========== FDCAN1 数据处理（科泰电机反馈） ==========
    if (hfdcan->Instance == FDCAN1)
    {
      // 帧ID 0x141：解析第0路科泰电机反馈数据
      if (rx_header.Identifier == 0x141)
      {
        ktech_parse_motor_fb(&motor_linkong[0], rx_data);
        if (KTech_FeedbackHasEncoder(rx_data[0]) != 0U)
        {
          Head_NotifyFeedback(0U);
        }
      }
      // 帧ID 0x142：解析第1路科泰电机反馈数据
      else if (rx_header.Identifier == 0x142)
      {
        ktech_parse_motor_fb(&motor_linkong[1], rx_data);
        if (KTech_FeedbackHasEncoder(rx_data[0]) != 0U)
        {
          Head_NotifyFeedback(1U);
        }
      }
      else if (rx_header.IdType == FDCAN_STANDARD_ID)
      {
        Servo_Lift_RxCallback(rx_header.Identifier, rx_header.IdType, rx_data);
      }
    }

    // ========== FDCAN2 数据处理（达妙/RobStride电机） ==========
    else if (hfdcan->Instance == FDCAN2)
    {
      // 标准帧（11位ID）处理逻辑
      if (rx_header.IdType == FDCAN_STANDARD_ID)
      {
        CAN2_StdRxDebug_Record(rx_header.Identifier);

        // 按帧ID分类处理
        switch (rx_header.Identifier)
        {
        case MOTOR_DAMIAO_4_ID:
        case MOTOR_DAMIAO_4_ID + POS_MODE:
          damiao_fbdata(&arm_motor[Motor4], rx_data);
          CAN_ArmFeedbackDebug_Record(3U);
          break;
        case MOTOR_DAMIAO_5_ID:
        case MOTOR_DAMIAO_5_ID + POS_MODE:
          damiao_fbdata(&arm_motor[Motor5], rx_data);
          CAN_ArmFeedbackDebug_Record(4U);
          break;
        case MOTOR_DAMIAO_6_ID:
        case MOTOR_DAMIAO_6_ID + POS_MODE:
          damiao_fbdata(&arm_motor[Motor6], rx_data);
          CAN_ArmFeedbackDebug_Record(5U);
          break;
        default:
          can2_std_rx_unknown_debug++;
          break;
        }
      }
      // 扩展帧（29位ID）处理逻辑（RobStride电机）灵足电机
      else if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        can2_ext_rx_total_debug++;

        // 从扩展帧ID中提取目标电机ID（右移8位后取低8位）
        uint8_t target_id = (uint8_t)((rx_header.Identifier >> 8) & 0xFF);
        if (target_id == 0x01) // 电机ID=0x01：解析1号RobStride电机
        {
          CAN2_RobStrideExtFrame_Process(rx_header.Identifier, rx_data);
        }
        else if (target_id == 0x02) // 电机ID=0x02：解析2号RobStride电机
        {
          CAN2_RobStrideExtFrame_Process(rx_header.Identifier, rx_data);
        }
        else if (target_id == 0x03) // 电机ID=0x03：解析3号RobStride电机
        {
          CAN2_RobStrideExtFrame_Process(rx_header.Identifier, rx_data);
        }
      }
    }

    // ========== FDCAN3 数据处理（全向轮 RobStride 电机） ==========
    else if (hfdcan->Instance == FDCAN3)
    {
      if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        Chassis_RxCallback(rx_header.Identifier, rx_data);
      }
    }
  }
}

/**
 * @brief CAN错误回调函数（CAN通信出错时触发）
 * @param  hfdcan  出错的CAN控制器句柄
 * @note   1. 功能：重启CAN控制器 + 重新配置过滤器 + 重新开启中断，实现错误自恢复；
 *         2. 过滤器配置：接收所有标准帧/扩展帧（FilterID=0，掩码=0），统一存入FIFO0；
 *         3. 全局过滤：所有未匹配过滤器的帧也存入FIFO0，避免丢帧。
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hfdcan->Instance == FDCAN2)
  {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) != 0U)
    {
      can2_fifo1_full_debug++;
    }
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
    {
      can2_fifo1_lost_debug++;
    }
  }

  if ((RxFifo1ITs & (FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL | FDCAN_IT_RX_FIFO1_MESSAGE_LOST)) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
  {
    if ((hfdcan->Instance == FDCAN2) && (rx_header.IdType == FDCAN_EXTENDED_ID))
    {
      CAN2_RobStrideExtFrame_Process(rx_header.Identifier, rx_data);
    }
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  // 步骤1：停止CAN控制器并重新初始化（错误恢复）
  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);

  // 步骤2：配置过滤器（接收所有标准帧）
  FDCAN_FilterTypeDef sFilter;
  sFilter.IdType = FDCAN_STANDARD_ID;       // 过滤器类型：标准帧
  sFilter.FilterIndex = 0;                  // 过滤器索引
  sFilter.FilterType = FDCAN_FILTER_MASK;   // 过滤模式：掩码匹配
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配帧存入FIFO0
  sFilter.FilterID1 = 0x000;                // 过滤ID=0（接收所有）
  sFilter.FilterID2 = 0x000;                // 掩码=0（接收所有）
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 步骤3：配置过滤器（接收所有扩展帧）
  sFilter.IdType = FDCAN_EXTENDED_ID;       // 过滤器类型：扩展帧
  sFilter.FilterIndex = 0;                  // 复用过滤器索引（覆盖配置）
  sFilter.FilterID1 = 0x00000000;           // 过滤ID=0（接收所有）
  sFilter.FilterID2 = 0x00000000;           // 掩码=0（接收所有）
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 步骤4：配置全局过滤规则
  // 规则：未匹配过滤器的标准帧/扩展帧都存入FIFO0，远程帧过滤
  if (hfdcan->Instance == FDCAN2)
  {
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);
  }

  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 步骤5：开启FIFO0新消息中断通知
  if (hfdcan->Instance == FDCAN2)
  {
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO1,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  }

  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0);
  if (hfdcan->Instance == FDCAN2)
  {
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_RX_FIFO0_FULL |
                                       FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                       FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                       FDCAN_IT_RX_FIFO1_FULL |
                                       FDCAN_IT_RX_FIFO1_MESSAGE_LOST,
                                   0);
  }
  
  // 步骤6：重启CAN控制器
  HAL_FDCAN_Start(hfdcan);
  CAN_TxProcess(hfdcan);
}
