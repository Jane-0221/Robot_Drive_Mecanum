#include "servo_lift_control.h"

#include <string.h>

#include "fdcan.h"

#define SERVO_LIFT_CAN_HANDLE (&hfdcan2)
#define SERVO_LIFT_OBJ_STATUSWORD 0x6041U
#define SERVO_LIFT_OBJ_POSITION_ACTUAL 0x6064U
#define SERVO_LIFT_FEEDBACK_TIMEOUT_MS 500U
#define SERVO_LIFT_ENABLE_RETRY_INTERVAL_MS 500U
#define SERVO_LIFT_STATUS_READ_INTERVAL_MS 100U
#define SERVO_LIFT_POSITION_READ_INTERVAL_MS 100U

extern FDCAN_HandleTypeDef hfdcan2;

int16_t aim_tx_height = SERVO_LIFT_HEIGHT_MIN_MM;
int16_t lift_height_final = SERVO_LIFT_HEIGHT_MIN_MM;
uint16_t lift_current_height = SERVO_LIFT_HEIGHT_MIN_MM;
volatile Servo_Lift_Debug_t servo_lift_debug = {0};
volatile uint8_t servo_lift_debug_cmd = SERVO_LIFT_DEBUG_CMD_NONE;
volatile int32_t servo_lift_debug_arg_pulses = 0;
volatile uint8_t servo_lift_debug_last_cmd = SERVO_LIFT_DEBUG_CMD_NONE;
volatile uint8_t servo_lift_debug_last_cmd_status = 0U;
volatile uint32_t servo_lift_debug_cmd_count = 0U;

static Servo_CAN_Motor_t servo_lift_motor;
static uint8_t servo_lift_reference_valid = 0U;
static uint8_t servo_lift_target_valid = 0U;
static int32_t servo_lift_reference_position_count = 0;
static int32_t servo_lift_last_target_position_count = 0;
static int16_t servo_lift_last_target_height_mm = 0;
static uint32_t servo_lift_last_enable_retry_tick_ms = 0U;
static uint32_t servo_lift_last_status_read_tick_ms = 0U;
static uint32_t servo_lift_last_position_read_tick_ms = 0U;

static int16_t Servo_Lift_ClampHeight(int16_t height_mm)
{
    if (height_mm < SERVO_LIFT_HEIGHT_MIN_MM)
    {
        return SERVO_LIFT_HEIGHT_MIN_MM;
    }

    if (height_mm > SERVO_LIFT_HEIGHT_MAX_MM)
    {
        return SERVO_LIFT_HEIGHT_MAX_MM;
    }

    return height_mm;
}

static int32_t Servo_Lift_RoundFloatToI32(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t Servo_Lift_RoundFloatToI16(float value)
{
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int32_t Servo_Lift_HeightToPositionCount(int16_t height_mm)
{
    float delta_height_mm = (float)(height_mm - SERVO_LIFT_HEIGHT_MIN_MM);
    float delta_count = delta_height_mm * SERVO_LIFT_PULSE_PER_MM * (float)SERVO_LIFT_DIRECTION;

    return servo_lift_reference_position_count + Servo_Lift_RoundFloatToI32(delta_count);
}

static int16_t Servo_Lift_PositionCountToHeight(int32_t position_count)
{
    float delta_count = (float)(position_count - servo_lift_reference_position_count);
    float height_mm = (float)SERVO_LIFT_HEIGHT_MIN_MM;

    if (SERVO_LIFT_PULSE_PER_MM > 0.0f)
    {
        height_mm += delta_count / (SERVO_LIFT_PULSE_PER_MM * (float)SERVO_LIFT_DIRECTION);
    }

    return Servo_Lift_ClampHeight(Servo_Lift_RoundFloatToI16(height_mm));
}

static void Servo_Lift_RefreshHeightFromPosition(void)
{
    int16_t height_mm;

    if (servo_lift_reference_valid == 0U)
    {
        return;
    }

    height_mm = Servo_Lift_PositionCountToHeight(servo_lift_motor.position_actual);
    lift_height_final = height_mm;
    lift_current_height = (uint16_t)height_mm;
}

static void Servo_Lift_CaptureReference(int32_t position_count)
{
    if (servo_lift_reference_valid != 0U)
    {
        return;
    }

    servo_lift_reference_position_count = position_count;
    servo_lift_reference_valid = 1U;
    lift_height_final = SERVO_LIFT_HEIGHT_MIN_MM;
    lift_current_height = SERVO_LIFT_HEIGHT_MIN_MM;
}

static uint8_t Servo_Lift_SendSetup(void)
{
    return Servo_CAN_ISL60C_InitMinimalPosition(SERVO_LIFT_CAN_HANDLE,
                                                SERVO_LIFT_NODE_ID,
                                                SERVO_LIFT_PROFILE_VELOCITY,
                                                SERVO_LIFT_PROFILE_ACCELERATION);
}

static uint8_t Servo_Lift_SendAbsoluteTarget(int32_t target_position_count)
{
    return Servo_CAN_ISL60C_MoveAbsolutePulses(SERVO_LIFT_CAN_HANDLE,
                                               SERVO_LIFT_NODE_ID,
                                               target_position_count);
}

static void Servo_Lift_RequestStatusIfNeeded(uint32_t tick_ms)
{
    if ((tick_ms - servo_lift_last_status_read_tick_ms) < SERVO_LIFT_STATUS_READ_INTERVAL_MS)
    {
        return;
    }

    servo_lift_last_status_read_tick_ms = tick_ms;
    (void)Servo_CAN_ISL60C_ReadStatusword(SERVO_LIFT_CAN_HANDLE, SERVO_LIFT_NODE_ID);
}

static void Servo_Lift_RequestPositionIfNeeded(uint32_t tick_ms)
{
    if (servo_lift_reference_valid != 0U)
    {
        return;
    }

    if ((tick_ms - servo_lift_last_position_read_tick_ms) < SERVO_LIFT_POSITION_READ_INTERVAL_MS)
    {
        return;
    }

    servo_lift_last_position_read_tick_ms = tick_ms;
    if (Servo_CAN_ISL60C_ReadPosition(SERVO_LIFT_CAN_HANDLE, SERVO_LIFT_NODE_ID) == 0U)
    {
        servo_lift_debug.sdo_position_read_count++;
    }
}

static void Servo_Lift_RetryEnableIfNeeded(uint32_t tick_ms)
{
    if (Servo_CAN_StatusOperationEnabled(servo_lift_motor.statusword) != 0U)
    {
        return;
    }

    if ((tick_ms - servo_lift_last_enable_retry_tick_ms) < SERVO_LIFT_ENABLE_RETRY_INTERVAL_MS)
    {
        return;
    }

    servo_lift_last_enable_retry_tick_ms = tick_ms;
    servo_lift_debug.last_enable_retry_tick_ms = tick_ms;
    servo_lift_debug.enable_retry_count++;

    if (Servo_CAN_StatusFault(servo_lift_motor.statusword) != 0U)
    {
        (void)Servo_CAN_FaultReset(SERVO_LIFT_CAN_HANDLE, SERVO_LIFT_NODE_ID);
    }

    (void)Servo_Lift_SendSetup();
}

static void Servo_Lift_DebugRefresh(uint32_t tick_ms)
{
    uint32_t last_update_tick = servo_lift_motor.last_update_tick;
    uint8_t online = 0U;

    if ((last_update_tick != 0U) && ((tick_ms - last_update_tick) <= SERVO_LIFT_FEEDBACK_TIMEOUT_MS))
    {
        online = 1U;
    }

    servo_lift_motor.online = online;
    servo_lift_debug.initialized = 1U;
    servo_lift_debug.reference_valid = servo_lift_reference_valid;
    servo_lift_debug.online = online;
    servo_lift_debug.operation_enabled = Servo_CAN_StatusOperationEnabled(servo_lift_motor.statusword);
    servo_lift_debug.fault = Servo_CAN_StatusFault(servo_lift_motor.statusword);
    servo_lift_debug.target_height_mm = servo_lift_last_target_height_mm;
    servo_lift_debug.actual_height_mm = lift_height_final;
    servo_lift_debug.reference_position_count = servo_lift_reference_position_count;
    servo_lift_debug.actual_position_count = servo_lift_motor.position_actual;
    servo_lift_debug.target_position_count = servo_lift_last_target_position_count;
    servo_lift_debug.statusword = servo_lift_motor.statusword;
    servo_lift_debug.error_code = servo_lift_motor.error_code;
    servo_lift_debug.last_rx_tick_ms = last_update_tick;
}

static void Servo_Lift_DebugCommandProcess(void)
{
    uint8_t cmd = servo_lift_debug_cmd;
    uint8_t status = 0U;

    if (cmd == SERVO_LIFT_DEBUG_CMD_NONE)
    {
        return;
    }

    servo_lift_debug_cmd = SERVO_LIFT_DEBUG_CMD_NONE;

    switch ((Servo_Lift_DebugCommand_t)cmd)
    {
    case SERVO_LIFT_DEBUG_CMD_INIT:
        status = Servo_Lift_SendSetup();
        break;

    case SERVO_LIFT_DEBUG_CMD_REL_POS_ONE_REV:
        status = Servo_Lift_RelativeTurnOneCircle(1);
        break;

    case SERVO_LIFT_DEBUG_CMD_REL_NEG_ONE_REV:
        status = Servo_Lift_RelativeTurnOneCircle(-1);
        break;

    case SERVO_LIFT_DEBUG_CMD_REL_PULSES:
        status = Servo_Lift_RelativeMovePulses(servo_lift_debug_arg_pulses);
        break;

    case SERVO_LIFT_DEBUG_CMD_READ_STATUS:
        status = Servo_CAN_ISL60C_ReadStatusword(SERVO_LIFT_CAN_HANDLE, SERVO_LIFT_NODE_ID);
        break;

    case SERVO_LIFT_DEBUG_CMD_READ_POSITION:
        status = Servo_CAN_ISL60C_ReadPosition(SERVO_LIFT_CAN_HANDLE, SERVO_LIFT_NODE_ID);
        break;

    case SERVO_LIFT_DEBUG_CMD_ABS_PULSES:
        status = Servo_CAN_ISL60C_MoveAbsolutePulses(SERVO_LIFT_CAN_HANDLE,
                                                     SERVO_LIFT_NODE_ID,
                                                     servo_lift_debug_arg_pulses);
        break;

    default:
        status = 1U;
        break;
    }

    servo_lift_debug_last_cmd = cmd;
    servo_lift_debug_last_cmd_status = status;
    servo_lift_debug_cmd_count++;
    servo_lift_debug.last_tx_status = status;
}

void Servo_Lift_Init(void)
{
    uint32_t tick_ms = HAL_GetTick();

    Servo_CAN_MotorInit(&servo_lift_motor, SERVO_LIFT_NODE_ID);
    servo_lift_reference_valid = 0U;
    servo_lift_target_valid = 0U;
    servo_lift_reference_position_count = 0;
    servo_lift_last_target_position_count = 0;
    servo_lift_last_target_height_mm = SERVO_LIFT_HEIGHT_MIN_MM;
    servo_lift_last_enable_retry_tick_ms = tick_ms;
    servo_lift_last_status_read_tick_ms = tick_ms - SERVO_LIFT_STATUS_READ_INTERVAL_MS;
    servo_lift_last_position_read_tick_ms = tick_ms - SERVO_LIFT_POSITION_READ_INTERVAL_MS;
    aim_tx_height = SERVO_LIFT_HEIGHT_MIN_MM;
    lift_height_final = SERVO_LIFT_HEIGHT_MIN_MM;
    lift_current_height = SERVO_LIFT_HEIGHT_MIN_MM;
    memset((void *)&servo_lift_debug, 0, sizeof(servo_lift_debug));

    servo_lift_debug.last_tx_status = Servo_Lift_SendSetup();
    servo_lift_debug.initialized = 1U;
}

void Servo_Lift_Update(void)
{
    uint32_t tick_ms = HAL_GetTick();

    Servo_Lift_RefreshHeightFromPosition();
    Servo_Lift_DebugCommandProcess();
    Servo_Lift_RequestStatusIfNeeded(tick_ms);
    Servo_Lift_RequestPositionIfNeeded(tick_ms);
    Servo_Lift_RetryEnableIfNeeded(tick_ms);
    Servo_Lift_DebugRefresh(tick_ms);
}

void Servo_Lift_GoToTarget(int16_t target_height)
{
    int16_t clamped_height;
    int32_t target_position_count;
    uint32_t tick_ms;

    if (target_height == 0)
    {
        return;
    }

    if (servo_lift_reference_valid == 0U)
    {
        return;
    }

    if (Servo_CAN_StatusOperationEnabled(servo_lift_motor.statusword) == 0U)
    {
        return;
    }

    clamped_height = Servo_Lift_ClampHeight(target_height);
    target_position_count = Servo_Lift_HeightToPositionCount(clamped_height);

    if ((servo_lift_target_valid != 0U) &&
        (target_position_count == servo_lift_last_target_position_count) &&
        (clamped_height == servo_lift_last_target_height_mm))
    {
        return;
    }

    servo_lift_debug.last_tx_status = Servo_Lift_SendAbsoluteTarget(target_position_count);
    if (servo_lift_debug.last_tx_status == 0U)
    {
        tick_ms = HAL_GetTick();
        servo_lift_target_valid = 1U;
        servo_lift_last_target_position_count = target_position_count;
        servo_lift_last_target_height_mm = clamped_height;
        servo_lift_debug.last_target_tx_tick_ms = tick_ms;
        servo_lift_debug.target_tx_count++;
    }
}

uint8_t Servo_Lift_RelativeMovePulses(int32_t position_pulses)
{
    uint8_t tx_status;
    uint32_t tick_ms;

    tx_status = Servo_CAN_ISL60C_MoveRelativePulses(SERVO_LIFT_CAN_HANDLE,
                                                    SERVO_LIFT_NODE_ID,
                                                    position_pulses);
    servo_lift_debug.last_tx_status = tx_status;
    if (tx_status == 0U)
    {
        tick_ms = HAL_GetTick();
        servo_lift_target_valid = 1U;
        servo_lift_last_target_position_count += position_pulses;
        servo_lift_debug.last_target_tx_tick_ms = tick_ms;
        servo_lift_debug.target_tx_count++;
    }

    return tx_status;
}

uint8_t Servo_Lift_RelativeTurnOneCircle(int8_t direction)
{
    int32_t pulses = (direction < 0) ?
                     -(int32_t)SERVO_CAN_ISL60C_PULSES_PER_REV :
                     (int32_t)SERVO_CAN_ISL60C_PULSES_PER_REV;

    return Servo_Lift_RelativeMovePulses(pulses);
}

uint16_t Servo_Lift_GetHeight(void)
{
    return lift_current_height;
}

void Servo_Lift_RxCallback(uint32_t identifier, uint32_t id_type, uint8_t *data)
{
    uint8_t parsed;

    if (data == NULL)
    {
        return;
    }

    parsed = Servo_CAN_ParseFrame(&servo_lift_motor, identifier, id_type, data, 8U);
    if (parsed == 0U)
    {
        return;
    }

    servo_lift_debug.rx_count++;

    if (identifier == (SERVO_CAN_COB_TPDO1 + SERVO_LIFT_NODE_ID))
    {
        Servo_Lift_CaptureReference(servo_lift_motor.position_actual);
        Servo_Lift_RefreshHeightFromPosition();
    }
    else if ((identifier == (SERVO_CAN_COB_SDO_TX + SERVO_LIFT_NODE_ID)) &&
             (servo_lift_motor.sdo_index == SERVO_LIFT_OBJ_POSITION_ACTUAL) &&
             (servo_lift_motor.sdo_subindex == 0x00U) &&
             (servo_lift_motor.sdo_abort_code == 0U))
    {
        servo_lift_motor.position_actual = (int32_t)servo_lift_motor.sdo_data;
        Servo_Lift_CaptureReference(servo_lift_motor.position_actual);
        Servo_Lift_RefreshHeightFromPosition();
    }
    else if ((identifier == (SERVO_CAN_COB_SDO_TX + SERVO_LIFT_NODE_ID)) &&
             (servo_lift_motor.sdo_index == SERVO_LIFT_OBJ_STATUSWORD) &&
             (servo_lift_motor.sdo_subindex == 0x00U) &&
             (servo_lift_motor.sdo_abort_code == 0U))
    {
        servo_lift_motor.statusword = (uint16_t)servo_lift_motor.sdo_data;
    }
}
