#include "rs485_lift.h"

#include <string.h>
#include "cmsis_os.h"

/*
 * RS485 lift controller.
 *
 * The lift drive uses Modbus RTU on USART2-RS485. This module is deliberately
 * synchronous: a task calls Rs485Lift_Process(), commands are taken from one
 * pending slot, and each Modbus request waits for its response before returning.
 *
 * Keep hardware direction control in this file. USART2 has a manual DE pin
 * (PD4), unlike the USART3 BMS port that uses HAL RS485 hardware DE.
 */

/* ----------------------------- Modbus protocol -------------------------------- */

#define RS485_LIFT_SLAVE_ADDR             1U
#define RS485_LIFT_MODBUS_FN_READ         0x03U
#define RS485_LIFT_MODBUS_FN_WRITE_ONE    0x06U
#define RS485_LIFT_MODBUS_FN_WRITE_MULTI  0x10U

/* ----------------------------- Drive registers -------------------------------- */

#define RS485_LIFT_REG_ACTUAL_RPM         0x0B00U
#define RS485_LIFT_REG_DI_MONITOR         0x0B03U
#define RS485_LIFT_REG_POSITION           0x0B07U
#define RS485_LIFT_REG_FAULT_CODE         0x0222U
#define RS485_LIFT_REG_SAVE_ENABLE        0x0C0DU
#define RS485_LIFT_REG_BUS_MODE           0x0200U
#define RS485_LIFT_REG_OPERATION_MODE     0x1003U
#define RS485_LIFT_REG_RELATIVE_POS       0x100EU
#define RS485_LIFT_REG_POSITION_SPEED     0x1019U
#define RS485_LIFT_REG_ACCEL              0x101BU
#define RS485_LIFT_REG_DECEL              0x101DU
#define RS485_LIFT_REG_PV_SPEED           0x102AU
#define RS485_LIFT_REG_MOTION_COMMAND     0x0D08U
#define RS485_LIFT_REG_FORCE_DI_ENABLE    0x0D11U
#define RS485_LIFT_REG_FORCE_DI_MASK      0x0D12U
#define RS485_LIFT_REG_DRIVE_SOFT_LIMIT   0x0A28U

/* DI bits are active low on this drive. */
#define RS485_LIFT_DI_SERVO_ON_MASK       0x04U
#define RS485_LIFT_DI_UPPER_LIMIT_MASK    0x08U
#define RS485_LIFT_DI_LOWER_LIMIT_MASK    0x10U

/* Motion command register values. */
#define RS485_LIFT_MOTION_START_REL       1U
#define RS485_LIFT_MOTION_FORWARD         16U
#define RS485_LIFT_MOTION_REVERSE         32U
#define RS485_LIFT_MOTION_STOP            256U

/* Operation modes used by the driver. */
#define RS485_LIFT_MODE_PP                1U
#define RS485_LIFT_MODE_PV                3U
#define RS485_LIFT_BUS_MODE_MODBUS        9U

/* ----------------------------- Runtime timing --------------------------------- */

#define RS485_LIFT_STATUS_POLL_MS         500U
#define RS485_LIFT_EXCHANGE_TIMEOUT_MS    40U
#define RS485_LIFT_LIMIT_TIMEOUT_MS       240000U
#define RS485_LIFT_RX_BUFFER_SIZE         32U
#define RS485_LIFT_TX_TIMEOUT_MS          20U

/* ----------------------------- RS485 DE control ------------------------------- */

#define RS485_LIFT_DE_GPIO_PORT           GPIOD
#define RS485_LIFT_DE_GPIO_PIN            GPIO_PIN_4
#define RS485_LIFT_DE_TX_LEVEL            GPIO_PIN_SET
#define RS485_LIFT_DE_RX_LEVEL            GPIO_PIN_RESET

/* ----------------------------- Mechanical scale ------------------------------- */

#define RS485_LIFT_PULSES_PER_REV         10000.0f
#define RS485_LIFT_SCREW_LEAD_MM          5.0f
#define RS485_LIFT_MOTOR_REV_PER_SCREW    2.0f
#define RS485_LIFT_NOMINAL_UNITS_PER_MM   ((RS485_LIFT_PULSES_PER_REV * RS485_LIFT_MOTOR_REV_PER_SCREW) / RS485_LIFT_SCREW_LEAD_MM)

#define RS485_LIFT_CAL_DEFAULT_DISTANCE_MM 100.0f
#define RS485_LIFT_CAL_MIN_DISTANCE_MM     1.0f
#define RS485_LIFT_CAL_TARGET_TOLERANCE_MM 1.0f
#define RS485_LIFT_CAL_POLL_MS             100U
#define RS485_LIFT_CAL_MOVE_TIMEOUT_MS     30000U
#define RS485_LIFT_CAL_STOP_RPM_THRESHOLD  3

typedef struct
{
  uint8_t zero_set;
  uint8_t top_set;
  uint8_t calibrated;
  uint8_t last_position_valid;
  int32_t zero_units;
  int32_t top_units;
  int32_t last_position_units;
  float lower_mm;
  float upper_mm;
  float calibrated_units_per_mm;
  float last_position_mm;
} Rs485LiftConfig_t;

/* ------------------------------ Module state ---------------------------------- */

static UART_HandleTypeDef *rs485_lift_huart = NULL;
static Rs485LiftStatus_t rs485_lift_status;
volatile Rs485LiftStatus_t rs485_lift_status_debug;
volatile uint32_t rs485_lift_param_debug[8];
volatile uint8_t rs485_lift_calibration_cmd_debug = RS485_LIFT_CAL_CMD_NONE;
volatile uint8_t rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_IDLE;
volatile float rs485_lift_calibration_command_mm_debug = RS485_LIFT_CAL_DEFAULT_DISTANCE_MM;
volatile float rs485_lift_calibration_actual_mm_debug = 0.0f;
volatile float rs485_lift_calibration_old_units_per_mm_debug = RS485_LIFT_NOMINAL_UNITS_PER_MM;
volatile float rs485_lift_calibration_new_units_per_mm_debug = RS485_LIFT_NOMINAL_UNITS_PER_MM;
volatile float rs485_lift_calibration_start_position_mm_debug = 0.0f;
volatile float rs485_lift_calibration_end_position_mm_debug = 0.0f;
volatile uint32_t rs485_lift_calibration_error_debug = RS485_LIFT_ERROR_NONE;

static Rs485LiftCommand_t rs485_lift_pending_command;
static volatile uint8_t rs485_lift_pending_valid = 0U;
static uint32_t rs485_lift_last_poll_tick = 0U;
static uint8_t rs485_lift_de_ready = 0U;
static uint32_t rs485_lift_calibration_start_tick = 0U;
static uint32_t rs485_lift_calibration_last_poll_tick = 0U;
static uint8_t rs485_lift_calibration_seen_motion = 0U;
static uint8_t rs485_lift_calibration_stop_requested = 0U;

/*
 * Stored calibration snapshot.
 * These values are intentionally kept in RAM for the current firmware version.
 * If persistent storage is added later, this structure is the boundary to save.
 */
static Rs485LiftConfig_t rs485_lift_config = {
  .zero_set = 1U,
  .top_set = 1U,
  .calibrated = 0U,
  .last_position_valid = 1U,
  .zero_units = -312000,
  .top_units = 39,
  .last_position_units = -5,
  .lower_mm = 0.0f,
  .upper_mm = 78.01f,
  .calibrated_units_per_mm = 0.0f,
  .last_position_mm = 77.999f,
};

/* ---------------------------- Private declarations ---------------------------- */

/* Common state and debug publishing. */
static void Rs485Lift_EnterCritical(uint32_t *primask);
static void Rs485Lift_ExitCritical(uint32_t primask);
static void Rs485Lift_PublishStatus(void);
static void Rs485Lift_SetError(uint32_t error);

/* Manual RS485 direction control for USART2 transceiver. */
static void Rs485Lift_InitDeGpio(void);
static void Rs485Lift_SetDirectionRx(void);
static void Rs485Lift_SetDirectionTx(void);
static uint8_t Rs485Lift_TransmitRequest(const uint8_t *request, uint16_t request_size);

/* Math, limits and coordinate conversion. */
static float Rs485Lift_AbsFloat(float value);
static int32_t Rs485Lift_RoundToInt32(float value);
static uint16_t Rs485Lift_ClampRpm(uint16_t rpm);
static uint16_t Rs485Lift_ClampAccel(uint16_t accel_rpm);
static float Rs485Lift_GetUnitsPerMm(void);
static int8_t Rs485Lift_GetAxisDirection(void);
static float Rs485Lift_GetTravelMm(void);
static float Rs485Lift_UnitsToHeightMm(int32_t units);
static int32_t Rs485Lift_HeightToUnits(float height_mm);
static int32_t Rs485Lift_MmToCommandUnits(float mm);
static float Rs485Lift_CommandUnitsToMm(int32_t units);
static uint8_t Rs485Lift_DiLowActive(uint16_t di_monitor, uint16_t mask);

/* Modbus RTU helpers. */
static uint16_t Rs485Lift_Crc16(const uint8_t *data, uint16_t size);
static void Rs485Lift_AddCrc(uint8_t *frame, uint16_t payload_size);
static uint8_t Rs485Lift_FindFrame(const uint8_t *bytes, uint16_t size, uint8_t expected_function,
                                   int16_t expected_byte_count, uint16_t *start, uint16_t *length);
static uint8_t Rs485Lift_Exchange(const uint8_t *request, uint16_t request_size, uint8_t expected_function,
                                  int16_t expected_byte_count, uint8_t *response, uint16_t response_max,
                                  uint16_t *response_size);

/* Register-level drive access. */
static uint8_t Rs485Lift_WriteReg16(uint16_t reg, uint16_t value);
static uint8_t Rs485Lift_ReadReg16(uint16_t reg, uint16_t *value);
static uint8_t Rs485Lift_WriteReg32LowWordFirst(uint16_t start_reg, int32_t value);
static uint8_t Rs485Lift_ReadReg32LowWordFirst(uint16_t start_reg, int32_t *value);

/* Motion profile and status helpers. */
static int16_t Rs485Lift_U16ToI16(uint16_t value);
static int32_t Rs485Lift_RpmToCommandSpeed(uint16_t rpm);
static int32_t Rs485Lift_RpmPerSecondToCommandAccel(uint16_t rpm_per_second);
static uint8_t Rs485Lift_CheckHardwareMoveDirection(float command_move_mm);
static uint8_t Rs485Lift_WriteMotionProfile(uint16_t rpm, uint16_t accel_rpm);
static uint8_t Rs485Lift_WriteRelativeMove(float move_mm, uint16_t rpm, uint16_t accel_rpm);
static uint8_t Rs485Lift_DisableDriveSoftLimits(void);
static uint8_t Rs485Lift_EnableHardwareDiLimits(void);
static uint8_t Rs485Lift_ReadStatusNow(void);

/* Calibration and height/limit helpers. */
static uint8_t Rs485Lift_SetCoordinateHeight(int32_t current_units, float height_mm);
static uint8_t Rs485Lift_SetManualLimitCoordinates(int32_t current_units, float current_height_mm,
                                                   float lower_mm, float upper_mm);
static uint8_t Rs485Lift_SetDistanceCalibration(float command_distance_mm, float actual_distance_mm,
                                                int32_t current_units);
static uint8_t Rs485Lift_SetZeroAnchoredDistanceCalibration(float command_distance_mm, float actual_distance_mm,
                                                            int32_t current_units);
static uint8_t Rs485Lift_GetTargetMoveByHeight(int32_t current_units, float target_height_mm,
                                               float *command_move_mm);
static uint8_t Rs485Lift_GetTargetMoveByDistance(int32_t current_units, float move_mm,
                                                  float *command_move_mm);
static uint8_t Rs485Lift_WaitHardwareLimit(uint16_t bit_mask);
static uint8_t Rs485Lift_InvokeFindHardwareLimits(uint16_t rpm, uint16_t accel_rpm);
static uint8_t Rs485Lift_InvokeSnapToIntegerPosition(uint16_t rpm, uint16_t accel_rpm);

/* Command queue and dispatcher. */
static void Rs485Lift_ExecuteCommand(Rs485LiftCommand_t *command);
static uint8_t Rs485Lift_TakePendingCommand(Rs485LiftCommand_t *command);

/* DAP-triggered 100 mm scale calibration. */
static void Rs485Lift_CalibrationProcess(void);
static void Rs485Lift_CalibrationStart(void);
static void Rs485Lift_CalibrationApplyActual(void);
static void Rs485Lift_CalibrationAbort(void);
static void Rs485Lift_CalibrationSetError(uint32_t error);

/* -------------------------------- Public API ---------------------------------- */

void Rs485Lift_Init(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  Rs485Lift_EnterCritical(&primask);
  rs485_lift_huart = huart;
  memset(&rs485_lift_status, 0, sizeof(rs485_lift_status));
  rs485_lift_pending_valid = 0U;
  rs485_lift_last_poll_tick = 0U;
  Rs485Lift_ExitCritical(primask);

  Rs485Lift_InitDeGpio();
  Rs485Lift_PublishStatus();
}

void Rs485Lift_Process(void)
{
  Rs485LiftCommand_t command;
  uint32_t tick_now = HAL_GetTick();

  if (Rs485Lift_TakePendingCommand(&command) != 0U)
  {
    /* Execute one accepted command to completion before polling status again. */
    rs485_lift_status.busy = 1U;
    rs485_lift_status.last_command = (uint32_t)command.command;
    rs485_lift_status.last_command_tick = tick_now;
    rs485_lift_status.command_count++;
    Rs485Lift_PublishStatus();

    Rs485Lift_ExecuteCommand(&command);

    rs485_lift_status.busy = 0U;
    rs485_lift_last_poll_tick = 0U;
    Rs485Lift_PublishStatus();
  }

  Rs485Lift_CalibrationProcess();

  tick_now = HAL_GetTick();
  if ((rs485_lift_last_poll_tick == 0U) ||
      ((tick_now - rs485_lift_last_poll_tick) >= RS485_LIFT_STATUS_POLL_MS))
  {
    /* Periodic status polling keeps the debug mirror fresh even without commands. */
    rs485_lift_last_poll_tick = tick_now;
    (void)Rs485Lift_ReadStatusNow();
  }
}

uint8_t Rs485Lift_SubmitCommand(const Rs485LiftCommand_t *command)
{
  uint32_t primask;
  uint8_t accepted = 0U;

  if ((command == NULL) || (command->command == RS485_LIFT_CMD_NONE))
  {
    return 0U;
  }

  Rs485Lift_EnterCritical(&primask);
  if (rs485_lift_pending_valid == 0U)
  {
    rs485_lift_pending_command = *command;
    rs485_lift_pending_valid = 1U;
    accepted = 1U;
  }
  Rs485Lift_ExitCritical(primask);

  if (accepted == 0U)
  {
    rs485_lift_status.command_drop_count++;
    Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
  }

  return accepted;
}

uint8_t Rs485Lift_CopyStatus(Rs485LiftStatus_t *out)
{
  uint32_t primask;

  if (out == NULL)
  {
    return 0U;
  }

  Rs485Lift_EnterCritical(&primask);
  memcpy(out, &rs485_lift_status, sizeof(*out));
  Rs485Lift_ExitCritical(primask);

  return out->valid;
}

void Rs485Lift_SetDefaultCommand(Rs485LiftCommand_t *command, Rs485LiftCommandId_t id)
{
  if (command == NULL)
  {
    return;
  }

  memset(command, 0, sizeof(*command));
  command->command = id;
  command->rpm = RS485_LIFT_DEFAULT_RPM;
  command->accel_rpm = RS485_LIFT_DEFAULT_ACCEL_RPM;
}

/* ---------------------------- State publishing -------------------------------- */

static void Rs485Lift_EnterCritical(uint32_t *primask)
{
  if (primask != NULL)
  {
    *primask = __get_PRIMASK();
  }
  __disable_irq();
}

static void Rs485Lift_ExitCritical(uint32_t primask)
{
  if ((primask & 0x1UL) == 0UL)
  {
    __enable_irq();
  }
}

static void Rs485Lift_PublishStatus(void)
{
  uint32_t primask;

  /* Merge live drive status with the current calibration/configuration snapshot. */
  rs485_lift_status.zero_set = rs485_lift_config.zero_set;
  rs485_lift_status.top_set = rs485_lift_config.top_set;
  rs485_lift_status.limits_ready = ((rs485_lift_config.zero_set != 0U) &&
                                    (rs485_lift_config.top_set != 0U)) ? 1U : 0U;
  rs485_lift_status.zero_units = rs485_lift_config.zero_units;
  rs485_lift_status.top_units = rs485_lift_config.top_units;
  rs485_lift_status.lower_mm = rs485_lift_config.lower_mm;
  rs485_lift_status.upper_mm = rs485_lift_config.upper_mm;
  rs485_lift_status.travel_mm = Rs485Lift_GetTravelMm();
  rs485_lift_status.units_per_mm = Rs485Lift_GetUnitsPerMm();
  rs485_lift_status.last_position_units = rs485_lift_config.last_position_units;
  rs485_lift_status.last_position_mm = rs485_lift_config.last_position_mm;

  Rs485Lift_EnterCritical(&primask);
  memcpy((void *)&rs485_lift_status_debug, &rs485_lift_status, sizeof(rs485_lift_status));
  Rs485Lift_ExitCritical(primask);
}

static void Rs485Lift_SetError(uint32_t error)
{
  if (error != RS485_LIFT_ERROR_NONE)
  {
    rs485_lift_status.error_count++;
  }
  rs485_lift_status.last_error = error;
  Rs485Lift_PublishStatus();
}

/* ---------------------------- RS485 direction --------------------------------- */

static void Rs485Lift_InitDeGpio(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(RS485_LIFT_DE_GPIO_PORT, RS485_LIFT_DE_GPIO_PIN, RS485_LIFT_DE_RX_LEVEL);

  GPIO_InitStruct.Pin = RS485_LIFT_DE_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_LIFT_DE_GPIO_PORT, &GPIO_InitStruct);

  rs485_lift_de_ready = 1U;
  Rs485Lift_SetDirectionRx();
}

static void Rs485Lift_SetDirectionRx(void)
{
  if (rs485_lift_de_ready != 0U)
  {
    HAL_GPIO_WritePin(RS485_LIFT_DE_GPIO_PORT, RS485_LIFT_DE_GPIO_PIN, RS485_LIFT_DE_RX_LEVEL);
  }
}

static void Rs485Lift_SetDirectionTx(void)
{
  if (rs485_lift_de_ready != 0U)
  {
    HAL_GPIO_WritePin(RS485_LIFT_DE_GPIO_PORT, RS485_LIFT_DE_GPIO_PIN, RS485_LIFT_DE_TX_LEVEL);
  }
}

static uint8_t Rs485Lift_TransmitRequest(const uint8_t *request, uint16_t request_size)
{
  HAL_StatusTypeDef status;
  uint32_t start_tick;

  /* The transceiver must be returned to RX immediately after transmit. */
  Rs485Lift_SetDirectionTx();
  status = HAL_UART_Transmit(rs485_lift_huart, (uint8_t *)request, request_size,
                             RS485_LIFT_TX_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    start_tick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(rs485_lift_huart, UART_FLAG_TC) == RESET)
    {
      if ((HAL_GetTick() - start_tick) >= RS485_LIFT_TX_TIMEOUT_MS)
      {
        status = HAL_TIMEOUT;
        break;
      }
    }
  }
  Rs485Lift_SetDirectionRx();

  if (status != HAL_OK)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_UART);
    return 0U;
  }

  return 1U;
}

/* ------------------------ Units, limits and conversion ------------------------- */

static float Rs485Lift_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int32_t Rs485Lift_RoundToInt32(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static uint16_t Rs485Lift_ClampRpm(uint16_t rpm)
{
  if (rpm == 0U)
  {
    return RS485_LIFT_DEFAULT_RPM;
  }
  if (rpm > RS485_LIFT_MAX_RPM)
  {
    return RS485_LIFT_MAX_RPM;
  }
  return rpm;
}

static uint16_t Rs485Lift_ClampAccel(uint16_t accel_rpm)
{
  if (accel_rpm == 0U)
  {
    return RS485_LIFT_DEFAULT_ACCEL_RPM;
  }
  if (accel_rpm < 10U)
  {
    return 10U;
  }
  if (accel_rpm > RS485_LIFT_MAX_ACCEL_RPM)
  {
    return RS485_LIFT_MAX_ACCEL_RPM;
  }
  return accel_rpm;
}

static float Rs485Lift_GetUnitsPerMm(void)
{
  if ((rs485_lift_config.calibrated != 0U) &&
      (rs485_lift_config.calibrated_units_per_mm > 1.0f))
  {
    return rs485_lift_config.calibrated_units_per_mm;
  }

  return RS485_LIFT_NOMINAL_UNITS_PER_MM;
}

static int8_t Rs485Lift_GetAxisDirection(void)
{
  if ((rs485_lift_config.zero_set != 0U) &&
      (rs485_lift_config.top_set != 0U) &&
      (rs485_lift_config.top_units < rs485_lift_config.zero_units))
  {
    return -1;
  }

  return 1;
}

static float Rs485Lift_GetTravelMm(void)
{
  if ((rs485_lift_config.zero_set != 0U) && (rs485_lift_config.top_set != 0U))
  {
    int32_t delta = rs485_lift_config.top_units - rs485_lift_config.zero_units;
    if (delta < 0)
    {
      delta = -delta;
    }
    return (float)delta / Rs485Lift_GetUnitsPerMm();
  }

  return Rs485Lift_AbsFloat(rs485_lift_config.upper_mm - rs485_lift_config.lower_mm);
}

static float Rs485Lift_UnitsToHeightMm(int32_t units)
{
  return rs485_lift_config.lower_mm +
         (((float)(units - rs485_lift_config.zero_units) * (float)Rs485Lift_GetAxisDirection()) /
          Rs485Lift_GetUnitsPerMm());
}

static int32_t Rs485Lift_HeightToUnits(float height_mm)
{
  float distance_mm = height_mm - rs485_lift_config.lower_mm;
  return rs485_lift_config.zero_units +
         ((int32_t)Rs485Lift_GetAxisDirection() * Rs485Lift_RoundToInt32(distance_mm * Rs485Lift_GetUnitsPerMm()));
}

static int32_t Rs485Lift_MmToCommandUnits(float mm)
{
  return Rs485Lift_RoundToInt32(mm * Rs485Lift_GetUnitsPerMm());
}

static float Rs485Lift_CommandUnitsToMm(int32_t units)
{
  return (float)units / Rs485Lift_GetUnitsPerMm();
}

static uint8_t Rs485Lift_DiLowActive(uint16_t di_monitor, uint16_t mask)
{
  return ((di_monitor & mask) == 0U) ? 1U : 0U;
}

/* ----------------------------- Modbus RTU core -------------------------------- */

static uint16_t Rs485Lift_Crc16(const uint8_t *data, uint16_t size)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < size; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static void Rs485Lift_AddCrc(uint8_t *frame, uint16_t payload_size)
{
  uint16_t crc = Rs485Lift_Crc16(frame, payload_size);

  frame[payload_size] = (uint8_t)(crc & 0xFFU);
  frame[payload_size + 1U] = (uint8_t)(crc >> 8U);
}

static uint8_t Rs485Lift_FindFrame(const uint8_t *bytes, uint16_t size, uint8_t expected_function,
                                   int16_t expected_byte_count, uint16_t *start, uint16_t *length)
{
  uint16_t pos;

  if ((bytes == NULL) || (start == NULL) || (length == NULL))
  {
    return 0U;
  }

  for (pos = 0U; pos + 5U <= size; pos++)
  {
    uint8_t function;
    uint16_t frame_len = 0U;
    uint16_t crc_calc;
    uint16_t crc_frame;

    if (bytes[pos] != RS485_LIFT_SLAVE_ADDR)
    {
      continue;
    }

    function = bytes[pos + 1U];
    if (function == (uint8_t)(expected_function | 0x80U))
    {
      frame_len = 5U;
    }
    else if ((function == RS485_LIFT_MODBUS_FN_READ) &&
             (expected_function == RS485_LIFT_MODBUS_FN_READ))
    {
      uint8_t byte_count = bytes[pos + 2U];
      if ((expected_byte_count >= 0) && (byte_count != (uint8_t)expected_byte_count))
      {
        continue;
      }
      frame_len = (uint16_t)(3U + byte_count + 2U);
    }
    else if (function == expected_function)
    {
      frame_len = 8U;
    }
    else
    {
      continue;
    }

    if ((pos + frame_len) > size)
    {
      continue;
    }

    crc_calc = Rs485Lift_Crc16(&bytes[pos], (uint16_t)(frame_len - 2U));
    crc_frame = (uint16_t)bytes[pos + frame_len - 2U] |
                ((uint16_t)bytes[pos + frame_len - 1U] << 8U);
    if (crc_calc != crc_frame)
    {
      continue;
    }

    *start = pos;
    *length = frame_len;
    return 1U;
  }

  return 0U;
}

static uint8_t Rs485Lift_Exchange(const uint8_t *request, uint16_t request_size, uint8_t expected_function,
                                  int16_t expected_byte_count, uint8_t *response, uint16_t response_max,
                                  uint16_t *response_size)
{
  uint8_t rx_buf[RS485_LIFT_RX_BUFFER_SIZE];
  uint16_t rx_size = 0U;
  uint16_t frame_start = 0U;
  uint16_t frame_len = 0U;
  uint32_t start_tick;
  uint8_t dummy;

  if ((rs485_lift_huart == NULL) || (request == NULL) || (response == NULL) ||
      (response_size == NULL))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_NOT_READY);
    return 0U;
  }

  Rs485Lift_SetDirectionRx();
  /* Drop stale bytes before starting a strict request/response exchange. */
  while (HAL_UART_Receive(rs485_lift_huart, &dummy, 1U, 1U) == HAL_OK)
  {
  }

  if (Rs485Lift_TransmitRequest(request, request_size) == 0U)
  {
    return 0U;
  }
  rs485_lift_status.tx_count++;

  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < RS485_LIFT_EXCHANGE_TIMEOUT_MS)
  {
    uint8_t byte;
    if (HAL_UART_Receive(rs485_lift_huart, &byte, 1U, 2U) == HAL_OK)
    {
      if (rx_size < sizeof(rx_buf))
      {
        rx_buf[rx_size++] = byte;
      }

      if (Rs485Lift_FindFrame(rx_buf, rx_size, expected_function, expected_byte_count,
                              &frame_start, &frame_len) != 0U)
      {
        /* Modbus exception response: slave, function|0x80, exception, CRC. */
        if (rx_buf[frame_start + 1U] == (uint8_t)(expected_function | 0x80U))
        {
          Rs485Lift_SetError(RS485_LIFT_ERROR_EXCEPTION);
          return 0U;
        }

        if (frame_len > response_max)
        {
          Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
          return 0U;
        }

        memcpy(response, &rx_buf[frame_start], frame_len);
        *response_size = frame_len;
        rs485_lift_status.rx_count++;
        Rs485Lift_SetError(RS485_LIFT_ERROR_NONE);
        return 1U;
      }
    }
  }

  if (rx_size > 0U)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_CRC);
  }
  else
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_TIMEOUT);
  }
  return 0U;
}

/* ----------------------------- Register access -------------------------------- */

static uint8_t Rs485Lift_WriteReg16(uint16_t reg, uint16_t value)
{
  uint8_t frame[8];
  uint8_t response[8];
  uint16_t response_size = 0U;

  frame[0] = RS485_LIFT_SLAVE_ADDR;
  frame[1] = RS485_LIFT_MODBUS_FN_WRITE_ONE;
  frame[2] = (uint8_t)(reg >> 8U);
  frame[3] = (uint8_t)(reg & 0xFFU);
  frame[4] = (uint8_t)(value >> 8U);
  frame[5] = (uint8_t)(value & 0xFFU);
  Rs485Lift_AddCrc(frame, 6U);

  return Rs485Lift_Exchange(frame, sizeof(frame), RS485_LIFT_MODBUS_FN_WRITE_ONE,
                            -1, response, sizeof(response), &response_size);
}

static uint8_t Rs485Lift_ReadReg16(uint16_t reg, uint16_t *value)
{
  uint8_t frame[8];
  uint8_t response[8];
  uint16_t response_size = 0U;

  if (value == NULL)
  {
    return 0U;
  }

  frame[0] = RS485_LIFT_SLAVE_ADDR;
  frame[1] = RS485_LIFT_MODBUS_FN_READ;
  frame[2] = (uint8_t)(reg >> 8U);
  frame[3] = (uint8_t)(reg & 0xFFU);
  frame[4] = 0x00U;
  frame[5] = 0x01U;
  Rs485Lift_AddCrc(frame, 6U);

  if (Rs485Lift_Exchange(frame, sizeof(frame), RS485_LIFT_MODBUS_FN_READ, 2,
                         response, sizeof(response), &response_size) == 0U)
  {
    return 0U;
  }

  *value = ((uint16_t)response[3] << 8U) | response[4];
  return 1U;
}

static uint8_t Rs485Lift_WriteReg32LowWordFirst(uint16_t start_reg, int32_t value)
{
  uint8_t frame[13];
  uint8_t response[8];
  uint16_t response_size = 0U;
  uint32_t raw = (uint32_t)value;
  uint16_t low_word = (uint16_t)(raw & 0xFFFFUL);
  uint16_t high_word = (uint16_t)((raw >> 16U) & 0xFFFFUL);

  /* This drive stores 32-bit values as low word first, then high word. */
  frame[0] = RS485_LIFT_SLAVE_ADDR;
  frame[1] = RS485_LIFT_MODBUS_FN_WRITE_MULTI;
  frame[2] = (uint8_t)(start_reg >> 8U);
  frame[3] = (uint8_t)(start_reg & 0xFFU);
  frame[4] = 0x00U;
  frame[5] = 0x02U;
  frame[6] = 0x04U;
  frame[7] = (uint8_t)(low_word >> 8U);
  frame[8] = (uint8_t)(low_word & 0xFFU);
  frame[9] = (uint8_t)(high_word >> 8U);
  frame[10] = (uint8_t)(high_word & 0xFFU);
  Rs485Lift_AddCrc(frame, 11U);

  return Rs485Lift_Exchange(frame, sizeof(frame), RS485_LIFT_MODBUS_FN_WRITE_MULTI,
                            -1, response, sizeof(response), &response_size);
}

static uint8_t Rs485Lift_ReadReg32LowWordFirst(uint16_t start_reg, int32_t *value)
{
  uint8_t frame[8];
  uint8_t response[10];
  uint16_t response_size = 0U;
  uint32_t low_word;
  uint32_t high_word;
  uint32_t raw;

  if (value == NULL)
  {
    return 0U;
  }

  frame[0] = RS485_LIFT_SLAVE_ADDR;
  frame[1] = RS485_LIFT_MODBUS_FN_READ;
  frame[2] = (uint8_t)(start_reg >> 8U);
  frame[3] = (uint8_t)(start_reg & 0xFFU);
  frame[4] = 0x00U;
  frame[5] = 0x02U;
  Rs485Lift_AddCrc(frame, 6U);

  if (Rs485Lift_Exchange(frame, sizeof(frame), RS485_LIFT_MODBUS_FN_READ, 4,
                         response, sizeof(response), &response_size) == 0U)
  {
    return 0U;
  }

  low_word = ((uint32_t)response[3] << 8U) | response[4];
  high_word = ((uint32_t)response[5] << 8U) | response[6];
  raw = low_word | (high_word << 16U);
  *value = (int32_t)raw;
  return 1U;
}

/* ---------------------------- Motion primitives ------------------------------- */

static int16_t Rs485Lift_U16ToI16(uint16_t value)
{
  return (int16_t)value;
}

static int32_t Rs485Lift_RpmToCommandSpeed(uint16_t rpm)
{
  return Rs485Lift_RoundToInt32(((float)Rs485Lift_ClampRpm(rpm) * RS485_LIFT_PULSES_PER_REV) / 60.0f);
}

static int32_t Rs485Lift_RpmPerSecondToCommandAccel(uint16_t rpm_per_second)
{
  return Rs485Lift_RoundToInt32(((float)Rs485Lift_ClampAccel(rpm_per_second) * RS485_LIFT_PULSES_PER_REV) / 60.0f);
}

static uint8_t Rs485Lift_CheckHardwareMoveDirection(float command_move_mm)
{
  uint16_t di_monitor;
  float physical_move_mm = command_move_mm * (float)Rs485Lift_GetAxisDirection();

  if (Rs485Lift_AbsFloat(command_move_mm) < 0.001f)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
  {
    return 0U;
  }
  if ((physical_move_mm > 0.0f) &&
      (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }
  if ((physical_move_mm < 0.0f) &&
      (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) != 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  return 1U;
}

static uint8_t Rs485Lift_WriteMotionProfile(uint16_t rpm, uint16_t accel_rpm)
{
  int32_t speed = Rs485Lift_RpmToCommandSpeed(rpm);
  int32_t accel = Rs485Lift_RpmPerSecondToCommandAccel(accel_rpm);

  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_PV_SPEED, speed) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_ACCEL, accel) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_DECEL, accel) == 0U)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t Rs485Lift_WriteRelativeMove(float move_mm, uint16_t rpm, uint16_t accel_rpm)
{
  int32_t position_units = Rs485Lift_MmToCommandUnits(move_mm);
  int32_t speed = Rs485Lift_RpmToCommandSpeed(rpm);
  int32_t accel = Rs485Lift_RpmPerSecondToCommandAccel(accel_rpm);

  if (Rs485Lift_CheckHardwareMoveDirection(move_mm) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PP) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_RELATIVE_POS, position_units) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_POSITION_SPEED, speed) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_ACCEL, accel) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_DECEL, accel) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_START_REL) == 0U)
  {
    return 0U;
  }

  /* Pulse the relative-start command, then clear it after the drive latches it. */
  osDelay(60U);
  return Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, 0U);
}

static uint8_t Rs485Lift_DisableDriveSoftLimits(void)
{
  return Rs485Lift_WriteReg16(RS485_LIFT_REG_DRIVE_SOFT_LIMIT, 0U);
}

static uint8_t Rs485Lift_EnableHardwareDiLimits(void)
{
  if (Rs485Lift_WriteReg16(0x0306U, 1U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(0x0307U, 0U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(0x0308U, 14U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(0x0309U, 0U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(0x030AU, 15U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(0x030BU, 0U) == 0U)
  {
    return 0U;
  }
  return Rs485Lift_WriteReg16(RS485_LIFT_REG_FORCE_DI_ENABLE, 0U);
}

/* ------------------------------ Status polling -------------------------------- */

static uint8_t Rs485Lift_ReadStatusNow(void)
{
  uint16_t actual_rpm_raw;
  uint16_t di_monitor;
  uint16_t fault_code;
  uint16_t param16;
  int32_t param32;
  int32_t position_units;
  uint8_t ok = 1U;

  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_ACTUAL_RPM, &actual_rpm_raw) == 0U)
  {
    ok = 0U;
  }
  if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &position_units) == 0U)
  {
    ok = 0U;
  }
  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
  {
    ok = 0U;
  }
  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_FAULT_CODE, &fault_code) == 0U)
  {
    ok = 0U;
  }
  rs485_lift_param_debug[0] = (Rs485Lift_ReadReg16(RS485_LIFT_REG_BUS_MODE, &param16) != 0U) ?
                              (uint32_t)param16 : 0xFFFFFFFFUL;
  rs485_lift_param_debug[1] = (Rs485Lift_ReadReg16(RS485_LIFT_REG_OPERATION_MODE, &param16) != 0U) ?
                              (uint32_t)param16 : 0xFFFFFFFFUL;
  rs485_lift_param_debug[2] = (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_PV_SPEED, &param32) != 0U) ?
                              (uint32_t)param32 : 0xFFFFFFFFUL;
  rs485_lift_param_debug[3] = (Rs485Lift_ReadReg16(RS485_LIFT_REG_MOTION_COMMAND, &param16) != 0U) ?
                              (uint32_t)param16 : 0xFFFFFFFFUL;
  rs485_lift_param_debug[4] = (Rs485Lift_ReadReg16(RS485_LIFT_REG_FORCE_DI_ENABLE, &param16) != 0U) ?
                              (uint32_t)param16 : 0xFFFFFFFFUL;
  rs485_lift_param_debug[5] = (uint32_t)di_monitor;
  rs485_lift_param_debug[6] = (uint32_t)fault_code;
  rs485_lift_param_debug[7] = (uint32_t)actual_rpm_raw;

  if (ok == 0U)
  {
    Rs485Lift_PublishStatus();
    return 0U;
  }

  rs485_lift_status.valid = 1U;
  rs485_lift_status.actual_rpm = Rs485Lift_U16ToI16(actual_rpm_raw);
  rs485_lift_status.position_units = position_units;
  rs485_lift_status.di_monitor = di_monitor;
  rs485_lift_status.fault_code = fault_code;
  rs485_lift_status.servo_on = Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_SERVO_ON_MASK);
  rs485_lift_status.upper_limit = Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK);
  rs485_lift_status.lower_limit = Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK);
  rs485_lift_status.position_mm = ((rs485_lift_config.zero_set != 0U) ?
                                   Rs485Lift_UnitsToHeightMm(position_units) : 0.0f);
  rs485_lift_status.last_update_tick = HAL_GetTick();

  /*
   * If the drive lost position reference but the mechanism is stopped, restore
   * the last known height by shifting the software coordinate frame.
   */
  if ((rs485_lift_config.last_position_valid != 0U) &&
      (Rs485Lift_AbsFloat(Rs485Lift_CommandUnitsToMm(position_units - rs485_lift_config.last_position_units)) > 20.0f) &&
      (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) < 3.0f) &&
      (rs485_lift_config.zero_set != 0U) &&
      (rs485_lift_config.top_set != 0U))
  {
    if (Rs485Lift_SetCoordinateHeight(position_units, rs485_lift_config.last_position_mm) != 0U)
    {
      rs485_lift_status.auto_restored_position = 1U;
      rs485_lift_status.position_mm = Rs485Lift_UnitsToHeightMm(position_units);
    }
  }
  else if ((rs485_lift_config.zero_set != 0U) && (rs485_lift_config.top_set != 0U))
  {
    rs485_lift_config.last_position_units = position_units;
    rs485_lift_config.last_position_mm = rs485_lift_status.position_mm;
    rs485_lift_config.last_position_valid = 1U;
  }

  Rs485Lift_PublishStatus();
  return 1U;
}

/* ------------------------- Calibration and coordinates ------------------------- */

static uint8_t Rs485Lift_SetCoordinateHeight(int32_t current_units, float height_mm)
{
  int32_t span_units;
  int32_t new_zero_units;
  int32_t new_top_units;
  float lower_mm;
  float upper_mm;

  if ((rs485_lift_config.zero_set == 0U) || (rs485_lift_config.top_set == 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  lower_mm = rs485_lift_config.lower_mm;
  upper_mm = rs485_lift_config.upper_mm;
  if (height_mm < lower_mm)
  {
    height_mm = lower_mm;
  }
  if (height_mm > upper_mm)
  {
    height_mm = upper_mm;
  }

  span_units = rs485_lift_config.top_units - rs485_lift_config.zero_units;
  new_zero_units = current_units -
                   ((int32_t)Rs485Lift_GetAxisDirection() *
                    Rs485Lift_MmToCommandUnits(height_mm - lower_mm));
  new_top_units = new_zero_units + span_units;

  rs485_lift_config.zero_units = new_zero_units;
  rs485_lift_config.top_units = new_top_units;
  rs485_lift_config.last_position_units = current_units;
  rs485_lift_config.last_position_mm = height_mm;
  rs485_lift_config.last_position_valid = 1U;
  Rs485Lift_PublishStatus();
  return 1U;
}

static uint8_t Rs485Lift_SetManualLimitCoordinates(int32_t current_units, float current_height_mm,
                                                   float lower_mm, float upper_mm)
{
  int8_t axis_direction;

  if (upper_mm <= lower_mm)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  axis_direction = Rs485Lift_GetAxisDirection();
  /* Re-anchor zero/top using the operator-provided current physical height. */
  rs485_lift_config.zero_units = current_units -
                                 ((int32_t)axis_direction *
                                  Rs485Lift_MmToCommandUnits(current_height_mm - lower_mm));
  rs485_lift_config.top_units = rs485_lift_config.zero_units +
                                ((int32_t)axis_direction *
                                 Rs485Lift_MmToCommandUnits(upper_mm - lower_mm));
  rs485_lift_config.lower_mm = lower_mm;
  rs485_lift_config.upper_mm = upper_mm;
  rs485_lift_config.zero_set = 1U;
  rs485_lift_config.top_set = 1U;
  rs485_lift_config.last_position_units = current_units;
  rs485_lift_config.last_position_mm = current_height_mm;
  rs485_lift_config.last_position_valid = 1U;
  Rs485Lift_PublishStatus();
  return 1U;
}

static uint8_t Rs485Lift_SetDistanceCalibration(float command_distance_mm, float actual_distance_mm,
                                                int32_t current_units)
{
  float old_units_per_mm;
  float new_units_per_mm;
  float current_height_mm;
  float travel_mm;
  int8_t axis_direction;

  if ((command_distance_mm <= 0.0f) || (actual_distance_mm <= 0.0f))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  old_units_per_mm = Rs485Lift_GetUnitsPerMm();
  current_height_mm = Rs485Lift_UnitsToHeightMm(current_units);
  new_units_per_mm = old_units_per_mm * (command_distance_mm / actual_distance_mm);
  if (new_units_per_mm <= 1.0f)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  rs485_lift_config.calibrated_units_per_mm = new_units_per_mm;
  rs485_lift_config.calibrated = 1U;

  /* Keep current height continuous after changing the units/mm scale. */
  if ((rs485_lift_config.zero_set != 0U) && (rs485_lift_config.top_set != 0U))
  {
    axis_direction = Rs485Lift_GetAxisDirection();
    travel_mm = rs485_lift_config.upper_mm - rs485_lift_config.lower_mm;
    rs485_lift_config.zero_units = current_units -
                                   ((int32_t)axis_direction *
                                    Rs485Lift_MmToCommandUnits(current_height_mm - rs485_lift_config.lower_mm));
    rs485_lift_config.top_units = rs485_lift_config.zero_units +
                                  ((int32_t)axis_direction * Rs485Lift_MmToCommandUnits(travel_mm));
  }

  rs485_lift_config.last_position_units = current_units;
  rs485_lift_config.last_position_mm = current_height_mm;
  rs485_lift_config.last_position_valid = 1U;
  Rs485Lift_PublishStatus();
  return 1U;
}

static uint8_t Rs485Lift_SetZeroAnchoredDistanceCalibration(float command_distance_mm, float actual_distance_mm,
                                                            int32_t current_units)
{
  float old_units_per_mm;
  float new_units_per_mm;

  if ((command_distance_mm <= 0.0f) || (actual_distance_mm <= 0.0f) ||
      (rs485_lift_config.zero_set == 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  old_units_per_mm = Rs485Lift_GetUnitsPerMm();
  new_units_per_mm = old_units_per_mm * (command_distance_mm / actual_distance_mm);
  if (new_units_per_mm <= 1.0f)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  rs485_lift_config.calibrated_units_per_mm = new_units_per_mm;
  rs485_lift_config.calibrated = 1U;
  rs485_lift_config.lower_mm = 0.0f;
  rs485_lift_config.upper_mm = actual_distance_mm;
  rs485_lift_config.top_units = current_units;
  rs485_lift_config.top_set = 1U;
  rs485_lift_config.last_position_units = current_units;
  rs485_lift_config.last_position_mm = actual_distance_mm;
  rs485_lift_config.last_position_valid = 1U;

  rs485_lift_calibration_old_units_per_mm_debug = old_units_per_mm;
  rs485_lift_calibration_new_units_per_mm_debug = new_units_per_mm;
  Rs485Lift_PublishStatus();
  return 1U;
}

/* ------------------------------ Target planning ------------------------------- */

static uint8_t Rs485Lift_GetTargetMoveByHeight(int32_t current_units, float target_height_mm,
                                               float *command_move_mm)
{
  int32_t target_units;
  int32_t move_units;

  if ((command_move_mm == NULL) ||
      (rs485_lift_config.zero_set == 0U) ||
      (rs485_lift_config.top_set == 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  target_units = Rs485Lift_HeightToUnits(target_height_mm);
  move_units = target_units - current_units;
  if ((move_units < 1) && (move_units > -1))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  *command_move_mm = Rs485Lift_CommandUnitsToMm(move_units);
  return 1U;
}

static uint8_t Rs485Lift_GetTargetMoveByDistance(int32_t current_units, float move_mm,
                                                  float *command_move_mm)
{
  if (command_move_mm == NULL)
  {
    return 0U;
  }
  (void)current_units;

  if (Rs485Lift_AbsFloat(move_mm) < 0.001f)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    return 0U;
  }

  *command_move_mm = move_mm * (float)Rs485Lift_GetAxisDirection();
  return 1U;
}

/* --------------------------- DAP calibration helper --------------------------- */

static void Rs485Lift_CalibrationSetError(uint32_t error)
{
  rs485_lift_calibration_cmd_debug = RS485_LIFT_CAL_CMD_NONE;
  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_ERROR;
  rs485_lift_calibration_error_debug = error;
  Rs485Lift_SetError(error);
}

static void Rs485Lift_CalibrationAbort(void)
{
  rs485_lift_calibration_cmd_debug = RS485_LIFT_CAL_CMD_NONE;
  (void)Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP);
  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_IDLE;
  rs485_lift_calibration_error_debug = RS485_LIFT_ERROR_NONE;
  rs485_lift_calibration_start_tick = 0U;
  rs485_lift_calibration_last_poll_tick = 0U;
  rs485_lift_calibration_seen_motion = 0U;
  rs485_lift_calibration_stop_requested = 0U;
}

static void Rs485Lift_CalibrationStart(void)
{
  float distance_mm = rs485_lift_calibration_command_mm_debug;
  int32_t current_units;
  int8_t axis_direction;
  uint16_t motion_command;

  rs485_lift_calibration_cmd_debug = RS485_LIFT_CAL_CMD_NONE;
  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_ZEROING;
  rs485_lift_calibration_error_debug = RS485_LIFT_ERROR_NONE;

  if (distance_mm < RS485_LIFT_CAL_MIN_DISTANCE_MM)
  {
    distance_mm = RS485_LIFT_CAL_DEFAULT_DISTANCE_MM;
    rs485_lift_calibration_command_mm_debug = distance_mm;
  }

  if (Rs485Lift_ReadStatusNow() == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_TIMEOUT);
    return;
  }

  if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > (float)RS485_LIFT_CAL_STOP_RPM_THRESHOLD)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_BUSY);
    return;
  }

  current_units = rs485_lift_status.position_units;
  axis_direction = Rs485Lift_GetAxisDirection();
  rs485_lift_config.zero_units = current_units;
  rs485_lift_config.top_units = current_units +
                                ((int32_t)axis_direction * Rs485Lift_MmToCommandUnits(distance_mm));
  rs485_lift_config.zero_set = 1U;
  rs485_lift_config.top_set = 1U;
  rs485_lift_config.lower_mm = 0.0f;
  rs485_lift_config.upper_mm = distance_mm;
  rs485_lift_config.last_position_units = current_units;
  rs485_lift_config.last_position_mm = 0.0f;
  rs485_lift_config.last_position_valid = 1U;
  rs485_lift_calibration_old_units_per_mm_debug = Rs485Lift_GetUnitsPerMm();
  rs485_lift_calibration_new_units_per_mm_debug = rs485_lift_calibration_old_units_per_mm_debug;
  rs485_lift_calibration_actual_mm_debug = 0.0f;
  rs485_lift_calibration_start_position_mm_debug = 0.0f;
  rs485_lift_calibration_end_position_mm_debug = 0.0f;
  Rs485Lift_PublishStatus();

  (void)Rs485Lift_DisableDriveSoftLimits();
  if (Rs485Lift_CheckHardwareMoveDirection(distance_mm * (float)axis_direction) == 0U)
  {
    Rs485Lift_CalibrationSetError((rs485_lift_status.last_error != RS485_LIFT_ERROR_NONE) ?
                                  rs485_lift_status.last_error : RS485_LIFT_ERROR_UART);
    return;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_UART);
    return;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PV) == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_UART);
    return;
  }
  if (Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_PV_SPEED,
                                       Rs485Lift_RpmToCommandSpeed(RS485_LIFT_DEFAULT_RPM)) == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_UART);
    return;
  }

  motion_command = (axis_direction >= 0) ? RS485_LIFT_MOTION_FORWARD : RS485_LIFT_MOTION_REVERSE;
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, motion_command) == 0U)
  {
    Rs485Lift_CalibrationSetError((rs485_lift_status.last_error != RS485_LIFT_ERROR_NONE) ?
                                  rs485_lift_status.last_error : RS485_LIFT_ERROR_UART);
    return;
  }

  rs485_lift_calibration_start_tick = HAL_GetTick();
  rs485_lift_calibration_last_poll_tick = 0U;
  rs485_lift_calibration_seen_motion = 0U;
  rs485_lift_calibration_stop_requested = 0U;
  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_MOVING_100MM;
}

static void Rs485Lift_CalibrationApplyActual(void)
{
  float distance_mm = rs485_lift_calibration_command_mm_debug;
  float actual_mm = rs485_lift_calibration_actual_mm_debug;

  rs485_lift_calibration_cmd_debug = RS485_LIFT_CAL_CMD_NONE;

  if (rs485_lift_calibration_state_debug != RS485_LIFT_CAL_STATE_WAIT_ACTUAL)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_BUSY);
    return;
  }
  if ((distance_mm < RS485_LIFT_CAL_MIN_DISTANCE_MM) ||
      (actual_mm < RS485_LIFT_CAL_MIN_DISTANCE_MM))
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_PARAM);
    return;
  }
  if (Rs485Lift_ReadStatusNow() == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_TIMEOUT);
    return;
  }
  if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > (float)RS485_LIFT_CAL_STOP_RPM_THRESHOLD)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_BUSY);
    return;
  }

  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_APPLYING;
  if (Rs485Lift_SetZeroAnchoredDistanceCalibration(distance_mm, actual_mm,
                                                   rs485_lift_status.position_units) == 0U)
  {
    Rs485Lift_CalibrationSetError((rs485_lift_status.last_error != RS485_LIFT_ERROR_NONE) ?
                                  rs485_lift_status.last_error : RS485_LIFT_ERROR_PARAM);
    return;
  }

  (void)Rs485Lift_ReadStatusNow();
  rs485_lift_calibration_end_position_mm_debug = rs485_lift_status.position_mm;
  rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_DONE;
}

static void Rs485Lift_CalibrationProcess(void)
{
  uint8_t command = rs485_lift_calibration_cmd_debug;
  uint32_t now;
  float target_mm;
  float position_mm;
  int16_t actual_rpm;

  if (command == RS485_LIFT_CAL_CMD_ABORT)
  {
    Rs485Lift_CalibrationAbort();
    return;
  }
  if (command == RS485_LIFT_CAL_CMD_START_100MM)
  {
    Rs485Lift_CalibrationStart();
    return;
  }
  if (command == RS485_LIFT_CAL_CMD_APPLY_ACTUAL)
  {
    Rs485Lift_CalibrationApplyActual();
    return;
  }

  if (rs485_lift_calibration_state_debug != RS485_LIFT_CAL_STATE_MOVING_100MM)
  {
    return;
  }

  now = HAL_GetTick();
  if ((rs485_lift_calibration_last_poll_tick != 0U) &&
      ((now - rs485_lift_calibration_last_poll_tick) < RS485_LIFT_CAL_POLL_MS))
  {
    return;
  }
  rs485_lift_calibration_last_poll_tick = now;

  if (Rs485Lift_ReadStatusNow() == 0U)
  {
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_TIMEOUT);
    return;
  }

  target_mm = rs485_lift_calibration_command_mm_debug;
  position_mm = rs485_lift_status.position_mm;
  actual_rpm = rs485_lift_status.actual_rpm;
  rs485_lift_calibration_end_position_mm_debug = position_mm;

  if (Rs485Lift_AbsFloat((float)actual_rpm) > (float)RS485_LIFT_CAL_STOP_RPM_THRESHOLD)
  {
    rs485_lift_calibration_seen_motion = 1U;
  }

  if ((rs485_lift_calibration_stop_requested == 0U) &&
      (position_mm >= (target_mm - RS485_LIFT_CAL_TARGET_TOLERANCE_MM)))
  {
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
    {
      Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_UART);
      return;
    }
    rs485_lift_calibration_stop_requested = 1U;
  }

  if ((rs485_lift_calibration_stop_requested != 0U) &&
      (Rs485Lift_AbsFloat((float)actual_rpm) <= (float)RS485_LIFT_CAL_STOP_RPM_THRESHOLD))
  {
    rs485_lift_calibration_state_debug = RS485_LIFT_CAL_STATE_WAIT_ACTUAL;
    return;
  }

  if (((rs485_lift_status.upper_limit != 0U) || (rs485_lift_status.lower_limit != 0U)) &&
      (position_mm < (target_mm - RS485_LIFT_CAL_TARGET_TOLERANCE_MM)))
  {
    (void)Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP);
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_LIMIT);
    return;
  }

  if ((now - rs485_lift_calibration_start_tick) >= RS485_LIFT_CAL_MOVE_TIMEOUT_MS)
  {
    (void)Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP);
    Rs485Lift_CalibrationSetError(RS485_LIFT_ERROR_TIMEOUT);
  }
}

/* --------------------------- Limit related actions ---------------------------- */

static uint8_t Rs485Lift_WaitHardwareLimit(uint16_t bit_mask)
{
  uint32_t start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < RS485_LIFT_LIMIT_TIMEOUT_MS)
  {
    uint16_t di_monitor;
    if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
    {
      return 0U;
    }
    if (Rs485Lift_DiLowActive(di_monitor, bit_mask) != 0U)
    {
      return 1U;
    }
    osDelay(80U);
  }

  Rs485Lift_SetError(RS485_LIFT_ERROR_TIMEOUT);
  return 0U;
}

static uint8_t Rs485Lift_InvokeFindHardwareLimits(uint16_t rpm, uint16_t accel_rpm)
{
  uint16_t di_monitor;
  uint16_t find_rpm = rpm;
  int32_t lower_units;
  int32_t upper_units;
  float travel_mm;
  float middle_mm;
  float move_to_middle_mm;

  if (find_rpm < 300U)
  {
    find_rpm = 300U;
  }
  if (find_rpm > 600U)
  {
    find_rpm = 600U;
  }

  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_FORCE_DI_ENABLE, 0U) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PV) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteMotionProfile(find_rpm, accel_rpm) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
  {
    return 0U;
  }
  if ((Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U) &&
      (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) != 0U))
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) == 0U)
  {
    /* Move down first until the lower hardware limit is reached. */
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_REVERSE) == 0U)
    {
      return 0U;
    }
    if (Rs485Lift_WaitHardwareLimit(RS485_LIFT_DI_LOWER_LIMIT_MASK) == 0U)
    {
      return 0U;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
    {
      return 0U;
    }
    osDelay(250U);
  }

  if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &lower_units) == 0U)
  {
    return 0U;
  }

  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U)
  {
    Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
    return 0U;
  }

  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_FORWARD) == 0U)
  {
    return 0U;
  }
  /* Then move up until the upper hardware limit is reached. */
  if (Rs485Lift_WaitHardwareLimit(RS485_LIFT_DI_UPPER_LIMIT_MASK) == 0U)
  {
    return 0U;
  }
  if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
  {
    return 0U;
  }
  osDelay(250U);

  if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &upper_units) == 0U)
  {
    return 0U;
  }

  rs485_lift_config.zero_units = lower_units;
  rs485_lift_config.top_units = upper_units;
  rs485_lift_config.zero_set = 1U;
  rs485_lift_config.top_set = 1U;
  rs485_lift_config.lower_mm = 0.0f;
  travel_mm = Rs485Lift_AbsFloat((float)(upper_units - lower_units) / Rs485Lift_GetUnitsPerMm());
  rs485_lift_config.upper_mm = travel_mm;
  rs485_lift_config.last_position_units = upper_units;
  rs485_lift_config.last_position_mm = travel_mm;
  rs485_lift_config.last_position_valid = 1U;
  Rs485Lift_PublishStatus();

  middle_mm = (float)Rs485Lift_RoundToInt32(travel_mm / 2.0f);
  move_to_middle_mm = middle_mm - travel_mm;
  if (Rs485Lift_AbsFloat(move_to_middle_mm) > 0.001f)
  {
    return Rs485Lift_WriteRelativeMove(move_to_middle_mm, RS485_LIFT_MAX_RPM, RS485_LIFT_MAX_ACCEL_RPM);
  }

  return 1U;
}

/* ------------------------------ Command dispatch ------------------------------ */

static uint8_t Rs485Lift_InvokeSnapToIntegerPosition(uint16_t rpm, uint16_t accel_rpm)
{
  int32_t current_units;
  float target_mm;
  float command_move_mm;
  uint16_t di_monitor;

  osDelay(80U);
  if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &current_units) == 0U)
  {
    return 0U;
  }
  if ((rs485_lift_config.zero_set == 0U) || (rs485_lift_config.top_set == 0U))
  {
    return 1U;
  }

  target_mm = (float)Rs485Lift_RoundToInt32(Rs485Lift_UnitsToHeightMm(current_units));
  if (Rs485Lift_GetTargetMoveByHeight(current_units, target_mm, &command_move_mm) == 0U)
  {
    return 1U;
  }
  if (Rs485Lift_AbsFloat(command_move_mm) < 0.02f)
  {
    return 1U;
  }
  if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
  {
    return 0U;
  }
  if ((Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U) &&
      (command_move_mm > 0.0f))
  {
    return 1U;
  }
  if ((Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) != 0U) &&
      (command_move_mm < 0.0f))
  {
    return 1U;
  }

  return Rs485Lift_WriteRelativeMove(command_move_mm, rpm, accel_rpm);
}

static void Rs485Lift_ExecuteCommand(Rs485LiftCommand_t *command)
{
  uint16_t rpm;
  uint16_t accel_rpm;
  uint16_t di_monitor;
  int32_t current_units;
  float command_move_mm;

  if (command == NULL)
  {
    return;
  }

  rpm = Rs485Lift_ClampRpm(command->rpm);
  accel_rpm = Rs485Lift_ClampAccel(command->accel_rpm);

  switch (command->command)
  {
  case RS485_LIFT_CMD_SETUP:
    /* Configure the drive for Modbus bus control and PV mode. */
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
    {
      return;
    }
    if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
    {
      return;
    }
    if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_SERVO_ON_MASK) != 0U)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
      return;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_SAVE_ENABLE, 1U) == 0U)
    {
      return;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_BUS_MODE, RS485_LIFT_BUS_MODE_MODBUS) == 0U)
    {
      return;
    }
    if (Rs485Lift_EnableHardwareDiLimits() == 0U)
    {
      return;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PV) == 0U)
    {
      return;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_SAVE_ENABLE, 0U) == 0U)
    {
      return;
    }
    if (Rs485Lift_WriteMotionProfile(rpm, accel_rpm) == 0U)
    {
      return;
    }
    (void)Rs485Lift_DisableDriveSoftLimits();
    break;

  case RS485_LIFT_CMD_FORWARD:
    /* Manual upward jog. Stop at upper limit; escape lower limit if needed. */
    if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
    {
      return;
    }
    if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
      return;
    }
    if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) != 0U)
    {
      /* Lower limit is the opposite direction; allow hardware to move away. */
    }
    (void)Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PV);
    (void)Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_PV_SPEED, Rs485Lift_RpmToCommandSpeed(rpm));
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_FORWARD) == 0U)
    {
      return;
    }
    break;

  case RS485_LIFT_CMD_REVERSE:
    /* Manual downward jog. Stop at lower limit; escape upper limit if needed. */
    if (Rs485Lift_ReadReg16(RS485_LIFT_REG_DI_MONITOR, &di_monitor) == 0U)
    {
      return;
    }
    if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_LOWER_LIMIT_MASK) != 0U)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
      return;
    }
    if (Rs485Lift_DiLowActive(di_monitor, RS485_LIFT_DI_UPPER_LIMIT_MASK) != 0U)
    {
      /* Upper limit is the opposite direction; allow hardware to move away. */
    }
    (void)Rs485Lift_WriteReg16(RS485_LIFT_REG_OPERATION_MODE, RS485_LIFT_MODE_PV);
    (void)Rs485Lift_WriteReg32LowWordFirst(RS485_LIFT_REG_PV_SPEED, Rs485Lift_RpmToCommandSpeed(rpm));
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_REVERSE) == 0U)
    {
      return;
    }
    break;

  case RS485_LIFT_CMD_STOP:
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
    {
      return;
    }
    if ((command->flags & RS485_LIFT_FLAG_SNAP_AFTER_STOP) != 0U)
    {
      (void)Rs485Lift_InvokeSnapToIntegerPosition(rpm, accel_rpm);
    }
    break;

  case RS485_LIFT_CMD_ENABLE:
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_FORCE_DI_ENABLE, 0U) == 0U)
    {
      return;
    }
    break;

  case RS485_LIFT_CMD_DISABLE:
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_MOTION_COMMAND, RS485_LIFT_MOTION_STOP) == 0U)
    {
      return;
    }
    if (Rs485Lift_WriteReg16(RS485_LIFT_REG_FORCE_DI_ENABLE, 0U) == 0U)
    {
      return;
    }
    break;

  case RS485_LIFT_CMD_SPEED:
    if (Rs485Lift_WriteMotionProfile(rpm, accel_rpm) == 0U)
    {
      return;
    }
    break;

  case RS485_LIFT_CMD_FIND_LIMITS:
    /* Slow full travel scan to learn lower and upper hardware limits. */
    (void)Rs485Lift_InvokeFindHardwareLimits(rpm, accel_rpm);
    break;

  case RS485_LIFT_CMD_MOVE:
    /* Relative move in millimeters. */
    if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &current_units) == 0U)
    {
      return;
    }
    if (Rs485Lift_GetTargetMoveByDistance(current_units, command->move_mm, &command_move_mm) != 0U)
    {
      (void)Rs485Lift_WriteRelativeMove(command_move_mm, rpm, accel_rpm);
    }
    break;

  case RS485_LIFT_CMD_GOTO_HEIGHT:
    /* Absolute move to a calibrated height in millimeters. */
    if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &current_units) == 0U)
    {
      return;
    }
    if (Rs485Lift_GetTargetMoveByHeight(current_units, command->target_height_mm, &command_move_mm) != 0U)
    {
      (void)Rs485Lift_WriteRelativeMove(command_move_mm, rpm, accel_rpm);
    }
    break;

  case RS485_LIFT_CMD_SET_ZERO:
  case RS485_LIFT_CMD_SET_LOWER:
    if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &current_units) != 0U)
    {
      rs485_lift_config.zero_units = current_units;
      rs485_lift_config.zero_set = 1U;
      Rs485Lift_PublishStatus();
      (void)Rs485Lift_DisableDriveSoftLimits();
    }
    break;

  case RS485_LIFT_CMD_SET_TOP:
  case RS485_LIFT_CMD_SET_UPPER:
    if (Rs485Lift_ReadReg32LowWordFirst(RS485_LIFT_REG_POSITION, &current_units) != 0U)
    {
      rs485_lift_config.top_units = current_units;
      rs485_lift_config.top_set = 1U;
      rs485_lift_config.upper_mm = Rs485Lift_GetTravelMm();
      Rs485Lift_PublishStatus();
      (void)Rs485Lift_DisableDriveSoftLimits();
    }
    break;

  case RS485_LIFT_CMD_APPLY_LIMITS:
  case RS485_LIFT_CMD_DISABLE_DRIVE_LIMITS:
    (void)Rs485Lift_DisableDriveSoftLimits();
    break;

  case RS485_LIFT_CMD_CALIBRATE_HEIGHT:
    /* Re-anchor current encoder units to a known physical height. */
    if (Rs485Lift_ReadStatusNow() == 0U)
    {
      return;
    }
    if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > 3.0f)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
      return;
    }
    (void)Rs485Lift_SetCoordinateHeight(rs485_lift_status.position_units, command->target_height_mm);
    break;

  case RS485_LIFT_CMD_SET_MANUAL_LIMITS:
    /* Operator-provided lower/upper travel range and current height. */
    if (Rs485Lift_ReadStatusNow() == 0U)
    {
      return;
    }
    if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > 3.0f)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
      return;
    }
    (void)Rs485Lift_SetManualLimitCoordinates(rs485_lift_status.position_units,
                                              command->current_height_mm,
                                              command->manual_lower_mm,
                                              command->manual_upper_mm);
    break;

  case RS485_LIFT_CMD_CALIBRATE_DISTANCE:
    /* Correct units/mm using commanded distance vs measured distance. */
    if (Rs485Lift_ReadStatusNow() == 0U)
    {
      return;
    }
    if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > 3.0f)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
      return;
    }
    (void)Rs485Lift_SetDistanceCalibration(command->command_distance_mm,
                                           command->actual_distance_mm,
                                           rs485_lift_status.position_units);
    break;

  case RS485_LIFT_CMD_RESTORE_LAST_POSITION:
    if (rs485_lift_config.last_position_valid == 0U)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_LIMIT);
      return;
    }
    if (Rs485Lift_ReadStatusNow() == 0U)
    {
      return;
    }
    if (Rs485Lift_AbsFloat((float)rs485_lift_status.actual_rpm) > 3.0f)
    {
      Rs485Lift_SetError(RS485_LIFT_ERROR_BUSY);
      return;
    }
    (void)Rs485Lift_SetCoordinateHeight(rs485_lift_status.position_units,
                                        rs485_lift_config.last_position_mm);
    break;

  case RS485_LIFT_CMD_CLEAR_LIMITS:
    rs485_lift_config.zero_set = 0U;
    rs485_lift_config.top_set = 0U;
    rs485_lift_config.last_position_valid = 0U;
    rs485_lift_config.zero_units = 0;
    rs485_lift_config.top_units = 0;
    rs485_lift_config.lower_mm = 0.0f;
    rs485_lift_config.upper_mm = 0.0f;
    Rs485Lift_PublishStatus();
    (void)Rs485Lift_DisableDriveSoftLimits();
    break;

  default:
    Rs485Lift_SetError(RS485_LIFT_ERROR_PARAM);
    break;
  }

  (void)Rs485Lift_ReadStatusNow();
}

/* Single-slot command queue. The task owns execution; callers only submit. */
static uint8_t Rs485Lift_TakePendingCommand(Rs485LiftCommand_t *command)
{
  uint32_t primask;

  if (command == NULL)
  {
    return 0U;
  }

  Rs485Lift_EnterCritical(&primask);
  if (rs485_lift_pending_valid == 0U)
  {
    Rs485Lift_ExitCritical(primask);
    return 0U;
  }

  *command = rs485_lift_pending_command;
  rs485_lift_pending_valid = 0U;
  Rs485Lift_ExitCritical(primask);

  return 1U;
}
