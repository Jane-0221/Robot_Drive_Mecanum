#ifndef RS485_LIFT_H
#define RS485_LIFT_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART2 RS485 lift control interface.
 *
 * Callers submit one command with Rs485Lift_SubmitCommand(); the
 * Rs485_Lift_Task calls Rs485Lift_Process() to execute it and periodically
 * refresh rs485_lift_status_debug for DAP/VSCode Live Watch.
 */

#define RS485_LIFT_DEFAULT_RPM        120U
#define RS485_LIFT_DEFAULT_ACCEL_RPM  3000U
#define RS485_LIFT_MAX_RPM            6000U
#define RS485_LIFT_MAX_ACCEL_RPM      60000U

#define RS485_LIFT_FLAG_SNAP_AFTER_STOP 0x01U

#define RS485_LIFT_ERROR_NONE             0x00000000UL
#define RS485_LIFT_ERROR_NOT_READY        0x00000001UL
#define RS485_LIFT_ERROR_TIMEOUT          0x00000002UL
#define RS485_LIFT_ERROR_CRC              0x00000004UL
#define RS485_LIFT_ERROR_EXCEPTION        0x00000008UL
#define RS485_LIFT_ERROR_LIMIT            0x00000010UL
#define RS485_LIFT_ERROR_PARAM            0x00000020UL
#define RS485_LIFT_ERROR_BUSY             0x00000040UL
#define RS485_LIFT_ERROR_UART             0x00000080UL

/* High-level command IDs accepted by Rs485Lift_SubmitCommand(). */
typedef enum
{
  RS485_LIFT_CMD_NONE = 0,
  RS485_LIFT_CMD_SETUP = 1,
  RS485_LIFT_CMD_FORWARD = 2,
  RS485_LIFT_CMD_REVERSE = 3,
  RS485_LIFT_CMD_STOP = 4,
  RS485_LIFT_CMD_ENABLE = 5,
  RS485_LIFT_CMD_DISABLE = 6,
  RS485_LIFT_CMD_SPEED = 7,
  RS485_LIFT_CMD_FIND_LIMITS = 8,
  RS485_LIFT_CMD_MOVE = 9,
  RS485_LIFT_CMD_GOTO_HEIGHT = 10,
  RS485_LIFT_CMD_SET_ZERO = 11,
  RS485_LIFT_CMD_SET_LOWER = 12,
  RS485_LIFT_CMD_SET_TOP = 13,
  RS485_LIFT_CMD_SET_UPPER = 14,
  RS485_LIFT_CMD_APPLY_LIMITS = 15,
  RS485_LIFT_CMD_CALIBRATE_HEIGHT = 16,
  RS485_LIFT_CMD_SET_MANUAL_LIMITS = 17,
  RS485_LIFT_CMD_CALIBRATE_DISTANCE = 18,
  RS485_LIFT_CMD_RESTORE_LAST_POSITION = 19,
  RS485_LIFT_CMD_DISABLE_DRIVE_LIMITS = 20,
  RS485_LIFT_CMD_CLEAR_LIMITS = 21
} Rs485LiftCommandId_t;

/* DAP/VSCode calibration helper commands. */
typedef enum
{
  RS485_LIFT_CAL_CMD_NONE = 0,
  RS485_LIFT_CAL_CMD_START_100MM = 1,
  RS485_LIFT_CAL_CMD_APPLY_ACTUAL = 2,
  RS485_LIFT_CAL_CMD_ABORT = 3
} Rs485LiftCalibrationCommand_t;

typedef enum
{
  RS485_LIFT_CAL_STATE_IDLE = 0,
  RS485_LIFT_CAL_STATE_ZEROING = 1,
  RS485_LIFT_CAL_STATE_MOVING_100MM = 2,
  RS485_LIFT_CAL_STATE_WAIT_ACTUAL = 3,
  RS485_LIFT_CAL_STATE_APPLYING = 4,
  RS485_LIFT_CAL_STATE_DONE = 5,
  RS485_LIFT_CAL_STATE_ERROR = 6
} Rs485LiftCalibrationState_t;

/*
 * Command payload.
 *
 * Not every field is used by every command. Use Rs485Lift_SetDefaultCommand()
 * before filling command-specific fields so rpm/accel have safe defaults.
 */
typedef struct
{
  Rs485LiftCommandId_t command;
  uint8_t flags;
  uint16_t rpm;
  uint16_t accel_rpm;
  float move_mm;
  float target_height_mm;
  float current_height_mm;
  float manual_lower_mm;
  float manual_upper_mm;
  float command_distance_mm;
  float actual_distance_mm;
} Rs485LiftCommand_t;

/* Runtime status and calibration snapshot mirrored for debugging. */
typedef struct
{
  uint8_t valid;
  uint8_t busy;
  uint8_t limits_ready;
  uint8_t zero_set;
  uint8_t top_set;
  uint8_t servo_on;
  uint8_t upper_limit;
  uint8_t lower_limit;
  uint8_t auto_restored_position;

  int16_t actual_rpm;
  int32_t position_units;
  int32_t zero_units;
  int32_t top_units;
  int32_t last_position_units;

  float position_mm;
  float lower_mm;
  float upper_mm;
  float travel_mm;
  float units_per_mm;
  float last_position_mm;

  uint16_t di_monitor;
  uint16_t fault_code;
  uint32_t last_update_tick;
  uint32_t last_command_tick;
  uint32_t last_command;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t command_count;
  uint32_t command_drop_count;
  uint32_t error_count;
  uint32_t last_error;
} Rs485LiftStatus_t;

/* Add this to VSCode Live Watch to inspect lift state while running. */
extern volatile Rs485LiftStatus_t rs485_lift_status_debug;
extern volatile uint8_t rs485_lift_calibration_cmd_debug;
extern volatile uint8_t rs485_lift_calibration_state_debug;
extern volatile float rs485_lift_calibration_command_mm_debug;
extern volatile float rs485_lift_calibration_actual_mm_debug;
extern volatile float rs485_lift_calibration_old_units_per_mm_debug;
extern volatile float rs485_lift_calibration_new_units_per_mm_debug;
extern volatile float rs485_lift_calibration_start_position_mm_debug;
extern volatile float rs485_lift_calibration_end_position_mm_debug;
extern volatile uint32_t rs485_lift_calibration_error_debug;

void Rs485Lift_Init(UART_HandleTypeDef *huart);
void Rs485Lift_Process(void);
uint8_t Rs485Lift_SubmitCommand(const Rs485LiftCommand_t *command);
uint8_t Rs485Lift_CopyStatus(Rs485LiftStatus_t *out);
void Rs485Lift_SetDefaultCommand(Rs485LiftCommand_t *command, Rs485LiftCommandId_t id);

#ifdef __cplusplus
}
#endif

#endif
