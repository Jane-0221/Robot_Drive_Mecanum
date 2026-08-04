/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "music.h"
#include "stdio.h"
#include "LED.h"
#include <cmsis_os2.h>
#include "iwdg.h"
#include "buzzer.h"
#include "task.h"
#include "arm.h"
#include "head.h"
#include "servo_lift_control.h"
#include "arm_sv.h"
#include "chassis.h"
#include "Sbus.h"
#include "stp23l.h"
#include "uart_protocol.h"
#include "pt_sensor.h"
#include "usart.h"
#include "user_key.h"
#include "kvms_battery.h"
#include "rs485_lift.h"

volatile uint32_t pc_up_tx_attempt_debug = 0U;
volatile uint32_t pc_up_tx_ok_debug = 0U;
volatile uint32_t pc_up_tx_busy_debug = 0U;
volatile uint32_t pc_up_tx_error_debug = 0U;
volatile uint32_t pc_up_tx_last_tick_debug = 0U;
volatile uint32_t pc_up_tx_feedback_age_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile uint32_t pc_up_tx_feedback_tick_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile uint32_t pc_battery_tx_attempt_debug = 0U;
volatile uint32_t pc_battery_tx_ok_debug = 0U;
volatile uint32_t pc_battery_tx_busy_debug = 0U;
volatile uint32_t pc_battery_tx_error_debug = 0U;
volatile uint32_t pc_battery_tx_last_tick_debug = 0U;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEAD_STARTUP_STABILIZE_MS 1000U
#define HEAD_REENABLE_STABILIZE_MS 100U
#define HEAD_TASK_MOTOR_COUNT 2U
#define PT_PRESS_POLL_PERIOD_MS 10U
#define PC_COMM_TX_PERIOD_MS 10U
#define PC_BATTERY_TX_PERIOD_MS 1000U
#define ARM_CONTROL_TX_PERIOD_MS 1U
#define ARM_SV_CONTROL_PERIOD_MS 5U
#define LOG_TASK_IDLE_PERIOD_MS 1000U
#define BATTERY_BMS_POLL_PERIOD_MS 1000U
#define RS485_LIFT_BOOT_TEST_ENABLE 0U
#define RS485_LIFT_BOOT_TEST_START_DELAY_MS 3000U
#define RS485_LIFT_BOOT_TEST_STEP_DELAY_MS 3000U
#define RS485_LIFT_BOOT_TEST_MOVE_MM (0.0f)
#define RS485_LIFT_BOOT_TEST_RPM 240U
#define RS485_LIFT_BOOT_TEST_ACCEL_RPM 2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t color = 0;
volatile uint32_t rs485_lift_boot_test_step_debug = 0U;
volatile uint32_t rs485_lift_boot_test_submit_count_debug = 0U;
volatile uint32_t rs485_lift_boot_test_last_result_debug = 0U;
volatile uint32_t rs485_lift_boot_test_last_tick_debug = 0U;
typedef enum
{
  STACK_REMOTE_CONTROL = 0,
  STACK_ARM_MT,
  STACK_LIFT_CONTROL,
  STACK_MOTOR_CONTROL,
  STACK_HEAD,
  STACK_ARM_UPDATE,
  STACK_LOG_AND_DEBUG,
  STACK_ARM_SV,
  STACK_PC_COMM,
  STACK_BATTERY_BMS,
  STACK_RS485_LIFT,
  STACK_TASK_COUNT
} StackWatermarkIndex_t;

volatile uint32_t freertos_stack_watermark_words[STACK_TASK_COUNT] = {0U};
volatile uint32_t head_task_feedback_count_debug[HEAD_TASK_MOTOR_COUNT] = {0U};
volatile uint8_t head_task_ready_mask_debug = 0U;
volatile uint8_t head_task_control_mask_debug = 0U;
volatile uint8_t head_task_block_reason_debug[HEAD_TASK_MOTOR_COUNT] = {0U};
volatile uint8_t control_mode = CONTROL_MODE_REMOTE;
/* USER CODE END Variables */
/* Definitions for Remote_control */
osThreadId_t Remote_controlHandle;
const osThreadAttr_t Remote_control_attributes = {
    .name = "Remote_control",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Arm_MT */
osThreadId_t Arm_MTHandle;
const osThreadAttr_t Arm_MT_attributes = {
    .name = "Arm_MT",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};
/* Definitions for Lift_control */
osThreadId_t Lift_controlHandle;
const osThreadAttr_t Lift_control_attributes = {
    .name = "Lift_control",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for Motor_control */
osThreadId_t Motor_controlHandle;
const osThreadAttr_t Motor_control_attributes = {
    .name = "Motor_control",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};
/* Definitions for Head */
osThreadId_t HeadHandle;
const osThreadAttr_t Head_attributes = {
    .name = "Head",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh3,
};
/* Definitions for Arm_update */
osThreadId_t Arm_updateHandle;
const osThreadAttr_t Arm_update_attributes = {
    .name = "Arm_update",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for Log_and_debug */
osThreadId_t Log_and_debugHandle;
const osThreadAttr_t Log_and_debug_attributes = {
    .name = "Log_and_debug",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow3,
};
/* Definitions for Arm_SV */
osThreadId_t Arm_SVHandle;
const osThreadAttr_t Arm_SV_attributes = {
    .name = "Arm_SV",
    .stack_size = 512 * 4,
    /* 5 ms姿态轨迹必须能抢占RS485升降机的同步超时轮询。 */
    .priority = (osPriority_t)osPriorityHigh2,
};
/* Definitions for PC_Comm */
osThreadId_t PC_CommHandle;
const osThreadAttr_t PC_Comm_attributes = {
    .name = "PC_Comm",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Battery_BMS */
osThreadId_t Battery_BMSHandle;
const osThreadAttr_t Battery_BMS_attributes = {
    .name = "Battery_BMS",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Rs485_Lift */
osThreadId_t Rs485_LiftHandle;
const osThreadAttr_t Rs485_Lift_attributes = {
    .name = "Rs485_Lift",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Pump_Update(void);
static void UpdateTaskStackWatermarks(void);
static void Rs485LiftBootTest_Update(void);
#if (RS485_LIFT_BOOT_TEST_ENABLE != 0U)
static uint8_t SubmitRs485LiftBootTestCommand(Rs485LiftCommandId_t id, float move_mm);
#endif

/* USER CODE END FunctionPrototypes */

void Remote_control_Task(void *argument);
void Arm_MT_Task(void *argument);
void Lift_control_Task(void *argument);
void Motor_control_Task(void *argument);
void Head_Task(void *argument);
void Arm_update_Task(void *argument);
void Log_and_debug_Task(void *argument);
void Arm_SV_Task(void *argument);
void PC_Comm_Task(void *argument);
void Battery_BMS_Task(void *argument);
void Rs485_Lift_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}

static uint32_t GetStackHighWaterWords(osThreadId_t handle)
{
  if (handle == NULL)
  {
    return 0U;
  }

  return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)handle);
}

static void UpdateTaskStackWatermarks(void)
{
  freertos_stack_watermark_words[STACK_REMOTE_CONTROL] = GetStackHighWaterWords(Remote_controlHandle);
  freertos_stack_watermark_words[STACK_ARM_MT] = GetStackHighWaterWords(Arm_MTHandle);
  freertos_stack_watermark_words[STACK_LIFT_CONTROL] = GetStackHighWaterWords(Lift_controlHandle);
  freertos_stack_watermark_words[STACK_MOTOR_CONTROL] = GetStackHighWaterWords(Motor_controlHandle);
  freertos_stack_watermark_words[STACK_HEAD] = GetStackHighWaterWords(HeadHandle);
  freertos_stack_watermark_words[STACK_ARM_UPDATE] = GetStackHighWaterWords(Arm_updateHandle);
  freertos_stack_watermark_words[STACK_LOG_AND_DEBUG] = GetStackHighWaterWords(Log_and_debugHandle);
  freertos_stack_watermark_words[STACK_ARM_SV] = GetStackHighWaterWords(Arm_SVHandle);
  freertos_stack_watermark_words[STACK_PC_COMM] = GetStackHighWaterWords(PC_CommHandle);
  freertos_stack_watermark_words[STACK_BATTERY_BMS] = GetStackHighWaterWords(Battery_BMSHandle);
  freertos_stack_watermark_words[STACK_RS485_LIFT] = GetStackHighWaterWords(Rs485_LiftHandle);
}

static void Rs485LiftBootTest_Update(void)
{
#if (RS485_LIFT_BOOT_TEST_ENABLE != 0U)
  static uint32_t start_tick = 0U;
  static uint32_t step_tick = 0U;
  static uint8_t step = 0U;
  Rs485LiftStatus_t status;
  uint32_t now = osKernelGetTickCount();

  if (start_tick == 0U)
  {
    start_tick = now;
  }

  rs485_lift_boot_test_step_debug = step;

  if (step == 0U)
  {
    if ((now - start_tick) < RS485_LIFT_BOOT_TEST_START_DELAY_MS)
    {
      return;
    }
    if ((Rs485Lift_CopyStatus(&status) == 0U) ||
        (status.rx_count < 3U) ||
        (status.last_error != RS485_LIFT_ERROR_NONE))
    {
      return;
    }
    step = 1U;
  }

  if ((step != 1U) && ((now - step_tick) < RS485_LIFT_BOOT_TEST_STEP_DELAY_MS))
  {
    return;
  }

  switch (step)
  {
  case 1U:
    if (SubmitRs485LiftBootTestCommand(RS485_LIFT_CMD_STOP, 0.0f) != 0U)
    {
      step = 2U;
      step_tick = now;
    }
    break;

  case 2U:
    if (SubmitRs485LiftBootTestCommand(RS485_LIFT_CMD_SETUP, 0.0f) != 0U)
    {
      step = 3U;
      step_tick = now;
    }
    break;

  case 3U:
    if (SubmitRs485LiftBootTestCommand(RS485_LIFT_CMD_ENABLE, 0.0f) != 0U)
    {
      step = 4U;
      step_tick = now;
    }
    break;

  case 4U:
    if (SubmitRs485LiftBootTestCommand(RS485_LIFT_CMD_FORWARD, RS485_LIFT_BOOT_TEST_MOVE_MM) != 0U)
    {
      step = 5U;
      step_tick = now;
    }
    break;

  case 5U:
    if (SubmitRs485LiftBootTestCommand(RS485_LIFT_CMD_STOP, 0.0f) != 0U)
    {
      step = 7U;
      step_tick = now;
    }
    break;

  default:
    break;
  }

  rs485_lift_boot_test_step_debug = step;
#else
  rs485_lift_boot_test_step_debug = 0U;
#endif
}

#if (RS485_LIFT_BOOT_TEST_ENABLE != 0U)
static uint8_t SubmitRs485LiftBootTestCommand(Rs485LiftCommandId_t id, float move_mm)
{
  Rs485LiftCommand_t command;
  uint8_t accepted;

  Rs485Lift_SetDefaultCommand(&command, id);
  command.rpm = RS485_LIFT_BOOT_TEST_RPM;
  command.accel_rpm = RS485_LIFT_BOOT_TEST_ACCEL_RPM;
  command.move_mm = move_mm;
  if (id == RS485_LIFT_CMD_STOP)
  {
    command.flags = RS485_LIFT_FLAG_SNAP_AFTER_STOP;
  }

  accepted = Rs485Lift_SubmitCommand(&command);
  rs485_lift_boot_test_last_result_debug = accepted;
  if (accepted != 0U)
  {
    rs485_lift_boot_test_submit_count_debug++;
    rs485_lift_boot_test_last_tick_debug = osKernelGetTickCount();
  }

  return accepted;
}
#endif
/* USER CODE END 2 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Remote_control */
  Remote_controlHandle = osThreadNew(Remote_control_Task, NULL, &Remote_control_attributes);

  /* creation of Arm_MT */
  Arm_MTHandle = osThreadNew(Arm_MT_Task, NULL, &Arm_MT_attributes);

  /* creation of Lift_control */
  Lift_controlHandle = osThreadNew(Lift_control_Task, NULL, &Lift_control_attributes);

  /* creation of Motor_control */
  Motor_controlHandle = osThreadNew(Motor_control_Task, NULL, &Motor_control_attributes);

  /* creation of Head */
  HeadHandle = osThreadNew(Head_Task, NULL, &Head_attributes);

  /* creation of Arm_update */
  Arm_updateHandle = osThreadNew(Arm_update_Task, NULL, &Arm_update_attributes);

  /* creation of Log_and_debug */
  Log_and_debugHandle = osThreadNew(Log_and_debug_Task, NULL, &Log_and_debug_attributes);

  /* creation of Arm_SV */
  Arm_SVHandle = osThreadNew(Arm_SV_Task, NULL, &Arm_SV_attributes);

  /* creation of PC_Comm */
  PC_CommHandle = osThreadNew(PC_Comm_Task, NULL, &PC_Comm_attributes);

  /* creation of Battery_BMS */
  Battery_BMSHandle = osThreadNew(Battery_BMS_Task, NULL, &Battery_BMS_attributes);

  /* creation of Rs485_Lift */
  Rs485_LiftHandle = osThreadNew(Rs485_Lift_Task, NULL, &Rs485_Lift_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_Remote_control_Task */
/**
 * @brief  Function implementing the Remote_control thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Remote_control_Task */
void Remote_control_Task(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN Remote_control_Task */
  MX_USB_DEVICE_Init();
  PT_Send_ReadTemp_Cmd(&huart1);
  uint32_t pt_press_last_tick = osKernelGetTickCount() - PT_PRESS_POLL_PERIOD_MS;
  int a = 0;
  /* Infinite loop */
  for (;;)
  {     
    uint32_t tick_now = osKernelGetTickCount();
    if ((tick_now - pt_press_last_tick) >= PT_PRESS_POLL_PERIOD_MS)
    {
      pt_press_last_tick = tick_now;
      PT_Send_ReadPress_Cmd(&huart1);
    }
    SBUS_UpdateIfNew();
    // uint8_t arm_disable_active = Arm_Motor_Disable_Updata();
    if ((control_mode == CONTROL_MODE_REMOTE))
    {
      // 遥控模式
      Head_Motor_Enable_Disable_Updata();
      Pump_Control_Updata();
      Head_Motor_Control_Updata();
      Arm_Motor_Control_Updata();
      Up_Down_Motor_Control_Updata();

    }
    else if ((control_mode == CONTROL_MODE_PC))
    {
      // pc模式

      PC_Rs485_Lift_Control_Updata();
      PC_Pump_Control_Updata();
      PC_Head_Motor_Control_Updata();
      PC_Up_Down_Motor_Control_Updata();
      // if (arm_disable_active == 0U)
      // {
      // 机械臂电机控制
      PC_Arm_Motor_Control_Updata();
      PC_Motor_Command_Updata();
      // }
      if (a == 0U)
      {
        a = 1U;
        
        Arm_EnableAllMotors();
      }
    }
    //Arm_Motor_Disable_Updata();
   // Pump_Control_Updata();
    osDelay(1);
  }

  /* USER CODE END Remote_control_Task */
}

/* USER CODE BEGIN Header_Arm_MT_Task */
/**
 * @brief Function implementing the Arm_MT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_MT_Task */
void Arm_MT_Task(void *argument)
{
  /* USER CODE BEGIN Arm_MT_Task */
  uint8_t arm_save_position_latched = 0U;
  uint32_t arm_control_next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for (;;)
  {
    USER_KEY_Update();
    // Arm_CheckAndReenableDisabledMotors();
    Arm_RequestDisabledFeedback();
    if (Arm_Motor_Disable_IsActive() == 0U)
    {
      uint8_t arm_save_active = Arm_Save_Position_IsActive();

      if (arm_save_active != 0U)
      {
        if (arm_save_position_latched == 0U)
        {
          Arm_save_position();
          arm_save_position_latched = 1U;
        }
      }
      else
      {
        arm_save_position_latched = 0U;
      }
      Arm_all_tx();
    }
    else
    {
      arm_save_position_latched = 0U;
    }

    arm_control_next_tick += ARM_CONTROL_TX_PERIOD_MS;
    if (osDelayUntil(arm_control_next_tick) != osOK)
    {
      arm_control_next_tick = osKernelGetTickCount();
      osDelay(ARM_CONTROL_TX_PERIOD_MS);
    }
  }
  /* USER CODE END Arm_MT_Task */
}

/* USER CODE BEGIN Header_Lift_control_Task */
/**
 * @brief Function implementing the Lift_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Lift_control_Task */
void Lift_control_Task(void *argument)
{
  /* USER CODE BEGIN Lift_control_Task */
  /* Infinite loop */
  for (;;)
  {
  //
    PT_ParseLatestPressure();

    Servo_Lift_Update();
    Servo_Lift_GoToTarget(aim_tx_height);
    osDelay(1);
    Pump_Update(); // 更新气泵状态
  }
  /* USER CODE END Lift_control_Task */
}

/* USER CODE BEGIN Header_Motor_control_Task */
/**
 * @brief Function implementing the Motor_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Motor_control_Task */
void Motor_control_Task(void *argument)
{
  /* USER CODE BEGIN Motor_control_Task */
  /* Infinite loop */
  for (;;)
  {
    if (control_mode == CONTROL_MODE_REMOTE)
    {
      Chassis_Control_Updata();
    }
    else if (control_mode == CONTROL_MODE_PC)
    {
      PC_Chassis_Control_Updata();
    }
    else
    {
      Chassis_SetCommand(0.0f, 0.0f, 0.0f);
    }

    Chassis_Update();
    osDelay(1);
  }
  /* USER CODE END Motor_control_Task */
}

/* USER CODE BEGIN Header_Head_Task */
/**
 * @brief Function implementing the Head thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Head_Task */
void Head_Task(void *argument)
{
  /* USER CODE BEGIN Head_Task */
  uint8_t head_disable_active;
  uint8_t last_head_disable_active = 1U;
  uint8_t head_first_enable = 1U;
  uint8_t head_enable_command_sent = 0U;
  uint32_t head_stabilize_ms = HEAD_STARTUP_STABILIZE_MS;
  uint32_t head_feedback_wait_start_tick = osKernelGetTickCount();
  /* Infinite loop */
  for (;;)
  {
    uint32_t tick_now = osKernelGetTickCount();
    uint8_t head_stable;
    uint8_t motor_index;

    head_disable_active = Head_Motor_Disable_IsActive();
    Head_Lk_Data_update();

    if ((last_head_disable_active != 0U) && (head_disable_active == 0U))
    {
      head_stabilize_ms = (head_first_enable != 0U) ?
                           HEAD_STARTUP_STABILIZE_MS :
                           HEAD_REENABLE_STABILIZE_MS;
      head_first_enable = 0U;
      head_enable_command_sent = 0U;
      head_feedback_wait_start_tick = tick_now;
    }
    last_head_disable_active = head_disable_active;

    head_stable = ((tick_now - head_feedback_wait_start_tick) >= head_stabilize_ms) ? 1U : 0U;
    head_task_ready_mask_debug = 0U;
    head_task_control_mask_debug = 0U;

    for (motor_index = 0U; motor_index < HEAD_TASK_MOTOR_COUNT; motor_index++)
    {
      head_task_feedback_count_debug[motor_index] = Head_GetFeedbackCount(motor_index);

      if (Head_MotorFeedbackReady(motor_index) != 0U)
      {
        head_task_ready_mask_debug |= (uint8_t)(1U << motor_index);
      }
    }

    if (head_disable_active != 0U)
    {
      Head_RequestFeedback();
      head_task_block_reason_debug[0] = 1U;
      head_task_block_reason_debug[1] = 1U;
    }
    else if (head_stable == 0U)
    {
      Head_RequestFeedback();
      head_task_block_reason_debug[0] = 2U;
      head_task_block_reason_debug[1] = 2U;
    }
    else if (head_enable_command_sent == 0U)
    {
      Head_Motor_SendEnableCommand();
      head_enable_command_sent = 1U;
      Head_RequestFeedback();
      head_task_block_reason_debug[0] = 3U;
      head_task_block_reason_debug[1] = 3U;
    }
    else
    {
      for (motor_index = 0U; motor_index < HEAD_TASK_MOTOR_COUNT; motor_index++)
      {
        if (Head_MotorFeedbackReady(motor_index) != 0U)
        {
          Head_TxMotorByIndex(motor_index);
          head_task_control_mask_debug |= (uint8_t)(1U << motor_index);
          head_task_block_reason_debug[motor_index] = 0U;
        }
        else
        {
          Head_RequestFeedbackByIndex(motor_index);
          head_task_block_reason_debug[motor_index] = 4U;
        }

        if (motor_index == 0U)
        {
          osDelay(1);
        }
      }
    }

    head_save_home_position();
    osDelay(1);
  }
  /* USER CODE END Head_Task */
}

/* USER CODE BEGIN Header_Arm_update_Task */
/**
 * @brief Function implementing the Arm_update thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_update_Task */
void Arm_update_Task(void *argument)
{
  /* USER CODE BEGIN Arm_update_Task */
  /* Infinite loop */
  for (;;)
  {
    Arm_All_Data_update();
    osDelay(1);
  }
  /* USER CODE END Arm_update_Task */
}

/* USER CODE BEGIN Header_Log_and_debug_Task */
/**
 * @brief Function implementing the Log_and_debug thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Log_and_debug_Task */
void Log_and_debug_Task(void *argument)
{
  /* USER CODE BEGIN Log_and_debug_Task */
  LEDshowcolor(RED);
  osDelay(50);
  LEDshowcolor(BLUE);
  osDelay(50);
  LEDshowcolor(GREEN);
  osDelay(50);
  /* Infinite loop */
  for (;;)
  {
    // Music_play(melody);
    // printf("hello\n");
    // Music_play_56_nations();
    UpdateTaskStackWatermarks();
    osDelay(LOG_TASK_IDLE_PERIOD_MS);
  }
  /* USER CODE END Log_and_debug_Task */
}

/* USER CODE BEGIN Header_Arm_SV_Task */
/**
 * @brief Function implementing the Arm_SV thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Arm_SV_Task */
void Arm_SV_Task(void *argument)
{
  /* USER CODE BEGIN Arm_SV_Task */
  uint32_t next_tick = osKernelGetTickCount();

  for (;;)
  {
    ARM_SV_Tx_Rx();
    next_tick += ARM_SV_CONTROL_PERIOD_MS;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
      osDelay(ARM_SV_CONTROL_PERIOD_MS);
    }
  }
  /* USER CODE END Arm_SV_Task */
}

/* USER CODE BEGIN Header_PC_Comm_Task */
/**
 * @brief Function implementing the PC_Comm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PC_Comm_Task */
void PC_Comm_Task(void *argument)
{
  /* USER CODE BEGIN PC_Comm_Task */
  uint32_t pc_tx_last_tick = osKernelGetTickCount() - PC_COMM_TX_PERIOD_MS;
  uint32_t pc_battery_tx_last_tick = osKernelGetTickCount();
  uint8_t pc_battery_tx_pending = 0U;
  /* Infinite loop */
  for (;;)
  {
    uint32_t tick_now = osKernelGetTickCount();

    UART_Protocol_UnpackLatest(&pc_dn_data);
    Control_Mode_Updata();
    if ((tick_now - pc_battery_tx_last_tick) >= PC_BATTERY_TX_PERIOD_MS)
    {
      pc_battery_tx_last_tick = tick_now;
      pc_battery_tx_pending = 1U;
    }

    if ((tick_now - pc_tx_last_tick) >= PC_COMM_TX_PERIOD_MS)
    {
      HAL_StatusTypeDef pc_tx_status;
      uint32_t pc_status_tick = HAL_GetTick();

      pc_tx_last_tick = tick_now;
      Arm_Linzu_Data_update();
      Arm_Damiao_Data_update();
      pc_arm_tx_data();
      pc_up_tx_data();

      for (uint8_t motor_index = 0U; motor_index < ARM_LOGICAL_MOTOR_COUNT; motor_index++)
      {
        uint32_t feedback_tick = arm_feedback_last_tick_debug[motor_index];
        pc_up_tx_feedback_tick_debug[motor_index] = feedback_tick;
        pc_up_tx_feedback_age_debug[motor_index] = (feedback_tick == 0U) ? 0xFFFFFFFFU : (pc_status_tick - feedback_tick);
      }

      if (pc_battery_tx_pending != 0U)
      {
        pc_battery_tx_attempt_debug++;
        pc_battery_tx_last_tick_debug = pc_status_tick;
        pc_tx_status = send_battery_up_frame(&huart10);
        if (pc_tx_status == HAL_OK)
        {
          pc_battery_tx_ok_debug++;
          pc_battery_tx_pending = 0U;
        }
        else if (pc_tx_status == HAL_BUSY)
        {
          pc_battery_tx_busy_debug++;
        }
        else
        {
          pc_battery_tx_error_debug++;
          pc_battery_tx_pending = 0U;
        }
      }
      else
      {
        pc_up_tx_attempt_debug++;
        pc_up_tx_last_tick_debug = pc_status_tick;
        pc_tx_status = send_up_frame_usb();
        if (pc_tx_status == HAL_OK)
        {
          pc_up_tx_ok_debug++;
        }
        else if (pc_tx_status == HAL_BUSY)
        {
          pc_up_tx_busy_debug++;
        }
        else
        {
          pc_up_tx_error_debug++;
        }
      }
    }

    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(1);
  }
  /* USER CODE END PC_Comm_Task */
}

/* USER CODE BEGIN Header_Battery_BMS_Task */
/**
 * @brief Function implementing the Battery_BMS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Battery_BMS_Task */
void Battery_BMS_Task(void *argument)
{
  /* USER CODE BEGIN Battery_BMS_Task */
  uint32_t battery_next_tick;
  (void)argument;

  KvmsBattery_Init(&huart3);
  battery_next_tick = osKernelGetTickCount();

  for (;;)
  {
    KvmsBattery_Poll();
    battery_next_tick += BATTERY_BMS_POLL_PERIOD_MS;
    if (osDelayUntil(battery_next_tick) != osOK)
    {
      battery_next_tick = osKernelGetTickCount();
      osDelay(BATTERY_BMS_POLL_PERIOD_MS);
    }
  }
  /* USER CODE END Battery_BMS_Task */
}

/* USER CODE BEGIN Header_Rs485_Lift_Task */
/**
 * @brief Function implementing the Rs485_Lift thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Rs485_Lift_Task */
void Rs485_Lift_Task(void *argument)
{
  /* USER CODE BEGIN Rs485_Lift_Task */
  (void)argument;

  for (;;)
  {
    Rs485Lift_Process();
    Rs485LiftBootTest_Update();
    osDelay(1);
  }
  /* USER CODE END Rs485_Lift_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
