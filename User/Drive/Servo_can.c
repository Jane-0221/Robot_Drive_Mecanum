#include "Servo_can.h"
#include <string.h>

#define SERVO_CAN_OBJ_CONTROLWORD            0x6040U
#define SERVO_CAN_OBJ_STATUSWORD             0x6041U
#define SERVO_CAN_OBJ_MODE_OF_OPERATION      0x6060U
#define SERVO_CAN_OBJ_MODE_DISPLAY           0x6061U
#define SERVO_CAN_OBJ_POSITION_ACTUAL        0x6064U
#define SERVO_CAN_OBJ_VELOCITY_ACTUAL        0x606CU
#define SERVO_CAN_OBJ_TARGET_TORQUE          0x6071U
#define SERVO_CAN_OBJ_TARGET_POSITION        0x607AU
#define SERVO_CAN_OBJ_TORQUE_ACTUAL          0x6077U
#define SERVO_CAN_OBJ_CURRENT_ACTUAL         0x6078U
#define SERVO_CAN_OBJ_PROFILE_VELOCITY       0x6081U
#define SERVO_CAN_OBJ_PROFILE_ACCELERATION   0x6083U
#define SERVO_CAN_OBJ_PROFILE_DECELERATION   0x6084U
#define SERVO_CAN_OBJ_HOMING_METHOD          0x6098U
#define SERVO_CAN_OBJ_HOMING_SPEED           0x6099U
#define SERVO_CAN_OBJ_TARGET_VELOCITY        0x60FFU
#define SERVO_CAN_OBJ_ERROR_CODE             0x603FU
#define SERVO_CAN_OBJ_SYNC_COB_ID            0x1005U
#define SERVO_CAN_OBJ_HEARTBEAT              0x1017U

#define SERVO_CAN_OBJ_RPDO1_COMM             0x1400U
#define SERVO_CAN_OBJ_RPDO1_MAP              0x1600U
#define SERVO_CAN_OBJ_TPDO1_COMM             0x1800U
#define SERVO_CAN_OBJ_TPDO1_MAP              0x1A00U

#define SERVO_CAN_CW_SHUTDOWN                0x0006U
#define SERVO_CAN_CW_SWITCH_ON               0x0007U
#define SERVO_CAN_CW_ENABLE_OPERATION        0x000FU
#define SERVO_CAN_CW_ABS_START               0x001FU
#define SERVO_CAN_CW_REL_START               0x005FU
#define SERVO_CAN_CW_ABS_START_IMMEDIATE     0x003FU
#define SERVO_CAN_CW_REL_START_IMMEDIATE     0x007FU
#define SERVO_CAN_CW_STOP                    0x010FU
#define SERVO_CAN_CW_QUICK_STOP              0x000BU
#define SERVO_CAN_CW_FAULT_RESET             0x0080U

#define SERVO_CAN_PDO_DISABLE_MASK           0x80000000UL

static uint32_t Servo_CAN_LenToDlc(uint8_t len)
{
    switch (len)
    {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

static void Servo_CAN_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void Servo_CAN_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFUL);
    data[1] = (uint8_t)((value >> 8) & 0xFFUL);
    data[2] = (uint8_t)((value >> 16) & 0xFFUL);
    data[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

static uint16_t Servo_CAN_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t Servo_CAN_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint8_t Servo_CAN_SdoWrite(hcan_t *hcan, uint8_t node_id, uint8_t cs, uint16_t index, uint8_t subindex, uint32_t value)
{
    uint8_t data[8] = {0U};

    data[0] = cs;
    Servo_CAN_WriteU16(&data[1], index);
    data[3] = subindex;
    Servo_CAN_WriteU32(&data[4], value);

    return Servo_CAN_SendFrame(hcan, (uint16_t)(SERVO_CAN_COB_SDO_RX + node_id), data, 8U);
}

static uint8_t Servo_CAN_ConfigPDOMapU32(hcan_t *hcan, uint8_t node_id, uint16_t base_index, uint8_t entry, uint16_t mapped_index, uint8_t subindex, uint8_t bit_len)
{
    uint32_t map = ((uint32_t)mapped_index << 16) | ((uint32_t)subindex << 8) | bit_len;
    return Servo_CAN_SdoWriteU32(hcan, node_id, base_index, entry, map);
}

static void Servo_CAN_StepDelay(void)
{
    if (SERVO_CAN_COMMAND_DELAY_MS > 0U)
    {
        HAL_Delay(SERVO_CAN_COMMAND_DELAY_MS);
    }
}

void Servo_CAN_MotorInit(Servo_CAN_Motor_t *motor, uint8_t node_id)
{
    if (motor == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->node_id = node_id;
}

uint8_t Servo_CAN_SendFrame(hcan_t *hcan, uint16_t cob_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;

    if ((hcan == NULL) || (cob_id > 0x7FFU) || (len > 8U) || ((data == NULL) && (len > 0U)))
    {
        return 1U;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.Identifier = cob_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = Servo_CAN_LenToDlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return (CAN_TxQueueFrame(hcan, &tx_header, data, len) == HAL_OK) ? 0U : 1U;
}

uint8_t Servo_CAN_NMT(hcan_t *hcan, uint8_t node_id, Servo_CAN_NmtCommand_t cmd)
{
    uint8_t data[2];

    data[0] = (uint8_t)cmd;
    data[1] = node_id;

    return Servo_CAN_SendFrame(hcan, SERVO_CAN_COB_NMT, data, 2U);
}

uint8_t Servo_CAN_StartNode(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_NMT(hcan, node_id, SERVO_CAN_NMT_START);
}

uint8_t Servo_CAN_StopNode(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_NMT(hcan, node_id, SERVO_CAN_NMT_STOP);
}

uint8_t Servo_CAN_PreOperational(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_NMT(hcan, node_id, SERVO_CAN_NMT_PRE_OPERATIONAL);
}

uint8_t Servo_CAN_ResetNode(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_NMT(hcan, node_id, SERVO_CAN_NMT_RESET_NODE);
}

uint8_t Servo_CAN_ResetCommunication(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_NMT(hcan, node_id, SERVO_CAN_NMT_RESET_COMMUNICATION);
}

uint8_t Servo_CAN_SdoRead(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex)
{
    uint8_t data[8] = {0x40U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    Servo_CAN_WriteU16(&data[1], index);
    data[3] = subindex;

    return Servo_CAN_SendFrame(hcan, (uint16_t)(SERVO_CAN_COB_SDO_RX + node_id), data, 8U);
}

uint8_t Servo_CAN_SdoWriteU8(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint8_t value)
{
    return Servo_CAN_SdoWrite(hcan, node_id, 0x2FU, index, subindex, value);
}

uint8_t Servo_CAN_SdoWriteU16(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t value)
{
    return Servo_CAN_SdoWrite(hcan, node_id, 0x2BU, index, subindex, value);
}

uint8_t Servo_CAN_SdoWriteU32(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value)
{
    return Servo_CAN_SdoWrite(hcan, node_id, 0x23U, index, subindex, value);
}

uint8_t Servo_CAN_SdoWriteI32(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value)
{
    return Servo_CAN_SdoWriteU32(hcan, node_id, index, subindex, (uint32_t)value);
}

uint8_t Servo_CAN_SetHeartbeat(hcan_t *hcan, uint8_t node_id, uint16_t heartbeat_ms)
{
    return Servo_CAN_SdoWriteU16(hcan, node_id, SERVO_CAN_OBJ_HEARTBEAT, 0x00U, heartbeat_ms);
}

uint8_t Servo_CAN_SetSyncCobId(hcan_t *hcan, uint8_t node_id, uint32_t sync_cob_id)
{
    return Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_SYNC_COB_ID, 0x00U, sync_cob_id);
}

uint8_t Servo_CAN_WriteControlword(hcan_t *hcan, uint8_t node_id, uint16_t controlword)
{
    return Servo_CAN_SdoWriteU16(hcan, node_id, SERVO_CAN_OBJ_CONTROLWORD, 0x00U, controlword);
}

uint8_t Servo_CAN_SetMode(hcan_t *hcan, uint8_t node_id, Servo_CAN_Mode_t mode)
{
    return Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_MODE_OF_OPERATION, 0x00U, (uint8_t)mode);
}

uint8_t Servo_CAN_Enable(hcan_t *hcan, uint8_t node_id)
{
    uint8_t ret = 0U;

    ret |= Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_SHUTDOWN);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_SWITCH_ON);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_ENABLE_OPERATION);

    return ret;
}

uint8_t Servo_CAN_Disable(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_SHUTDOWN);
}

uint8_t Servo_CAN_Stop(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_STOP);
}

uint8_t Servo_CAN_QuickStop(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_QUICK_STOP);
}

uint8_t Servo_CAN_FaultReset(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_FAULT_RESET);
}

uint8_t Servo_CAN_SetTargetVelocity(hcan_t *hcan, uint8_t node_id, int32_t velocity_count_per_sec)
{
    return Servo_CAN_SdoWriteI32(hcan, node_id, SERVO_CAN_OBJ_TARGET_VELOCITY, 0x00U, velocity_count_per_sec);
}

uint8_t Servo_CAN_SetTargetVelocityRPM(hcan_t *hcan, uint8_t node_id, float rpm, uint32_t pulses_per_rev)
{
    return Servo_CAN_SetTargetVelocity(hcan, node_id, Servo_CAN_RpmToCountPerSec(rpm, pulses_per_rev));
}

uint8_t Servo_CAN_SetProfileVelocity(hcan_t *hcan, uint8_t node_id, uint32_t velocity)
{
    return Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_PROFILE_VELOCITY, 0x00U, velocity);
}

uint8_t Servo_CAN_SetProfileAcceleration(hcan_t *hcan, uint8_t node_id, uint32_t acceleration)
{
    return Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_PROFILE_ACCELERATION, 0x00U, acceleration);
}

uint8_t Servo_CAN_SetProfileDeceleration(hcan_t *hcan, uint8_t node_id, uint32_t deceleration)
{
    return Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_PROFILE_DECELERATION, 0x00U, deceleration);
}

uint8_t Servo_CAN_SetTargetTorquePermille(hcan_t *hcan, uint8_t node_id, int16_t torque_permille)
{
    return Servo_CAN_SdoWriteU16(hcan, node_id, SERVO_CAN_OBJ_TARGET_TORQUE, 0x00U, (uint16_t)torque_permille);
}

uint8_t Servo_CAN_SetTargetTorquePercent(hcan_t *hcan, uint8_t node_id, float torque_percent)
{
    float scaled = torque_percent * 10.0f;
    int16_t value = (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));

    return Servo_CAN_SetTargetTorquePermille(hcan, node_id, value);
}

uint8_t Servo_CAN_SetTargetPosition(hcan_t *hcan, uint8_t node_id, int32_t position_count)
{
    return Servo_CAN_SdoWriteI32(hcan, node_id, SERVO_CAN_OBJ_TARGET_POSITION, 0x00U, position_count);
}

uint8_t Servo_CAN_SetTargetPositionDegree(hcan_t *hcan, uint8_t node_id, float degree, uint32_t pulses_per_rev)
{
    return Servo_CAN_SetTargetPosition(hcan, node_id, Servo_CAN_DegreeToCount(degree, pulses_per_rev));
}

uint8_t Servo_CAN_StartAbsolutePosition(hcan_t *hcan, uint8_t node_id, uint8_t immediate)
{
    return Servo_CAN_WriteControlword(hcan, node_id,
                                      (immediate != 0U) ? SERVO_CAN_CW_ABS_START_IMMEDIATE : SERVO_CAN_CW_ABS_START);
}

uint8_t Servo_CAN_StartRelativePosition(hcan_t *hcan, uint8_t node_id, uint8_t immediate)
{
    return Servo_CAN_WriteControlword(hcan, node_id,
                                      (immediate != 0U) ? SERVO_CAN_CW_REL_START_IMMEDIATE : SERVO_CAN_CW_REL_START);
}

uint8_t Servo_CAN_SetAbsolutePosition(hcan_t *hcan, uint8_t node_id, int32_t position_count, uint32_t velocity, uint32_t acceleration, uint8_t immediate)
{
    uint8_t ret = 0U;

    ret |= Servo_CAN_SetMode(hcan, node_id, SERVO_CAN_MODE_PROFILE_POSITION);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_SetTargetPosition(hcan, node_id, position_count);
    ret |= Servo_CAN_SetProfileVelocity(hcan, node_id, velocity);
    ret |= Servo_CAN_SetProfileAcceleration(hcan, node_id, acceleration);
    ret |= Servo_CAN_SetProfileDeceleration(hcan, node_id, acceleration);
    ret |= Servo_CAN_StartAbsolutePosition(hcan, node_id, immediate);

    return ret;
}

uint8_t Servo_CAN_SetRelativePosition(hcan_t *hcan, uint8_t node_id, int32_t position_count, uint32_t velocity, uint32_t acceleration, uint8_t immediate)
{
    uint8_t ret = 0U;

    ret |= Servo_CAN_SetMode(hcan, node_id, SERVO_CAN_MODE_PROFILE_POSITION);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_SetTargetPosition(hcan, node_id, position_count);
    ret |= Servo_CAN_SetProfileVelocity(hcan, node_id, velocity);
    ret |= Servo_CAN_SetProfileAcceleration(hcan, node_id, acceleration);
    ret |= Servo_CAN_SetProfileDeceleration(hcan, node_id, acceleration);
    ret |= Servo_CAN_StartRelativePosition(hcan, node_id, immediate);

    return ret;
}

uint8_t Servo_CAN_SetHomingMethod(hcan_t *hcan, uint8_t node_id, int8_t method)
{
    return Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_HOMING_METHOD, 0x00U, (uint8_t)method);
}

uint8_t Servo_CAN_SetHomingSpeed(hcan_t *hcan, uint8_t node_id, uint8_t subindex, uint32_t speed)
{
    if ((subindex == 0U) || (subindex > 2U))
    {
        return 1U;
    }

    return Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_HOMING_SPEED, subindex, speed);
}

uint8_t Servo_CAN_StartHoming(hcan_t *hcan, uint8_t node_id)
{
    return Servo_CAN_WriteControlword(hcan, node_id, SERVO_CAN_CW_ABS_START);
}

uint8_t Servo_CAN_SetCurrentPositionZero(hcan_t *hcan, uint8_t node_id)
{
    uint8_t ret = 0U;

    ret |= Servo_CAN_SetMode(hcan, node_id, SERVO_CAN_MODE_HOMING);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_SetHomingMethod(hcan, node_id, 0x23);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_StartHoming(hcan, node_id);

    return ret;
}

uint8_t Servo_CAN_ConfigPDO_Default(hcan_t *hcan, uint8_t node_id, uint16_t event_timer_ms)
{
    uint8_t ret = 0U;
    uint32_t cob_id;

    ret |= Servo_CAN_SetSyncCobId(hcan, node_id, 0x80000080UL);
    Servo_CAN_StepDelay();
    ret |= Servo_CAN_SetHeartbeat(hcan, node_id, 0x0320U);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_RPDO1 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_RPDO1_COMM, 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_RPDO1_COMM, 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_RPDO1_MAP, 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, SERVO_CAN_OBJ_RPDO1_MAP, 0x01U, SERVO_CAN_OBJ_TARGET_POSITION, 0x00U, 0x20U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, SERVO_CAN_OBJ_RPDO1_MAP, 0x02U, SERVO_CAN_OBJ_PROFILE_VELOCITY, 0x00U, 0x20U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_RPDO1_MAP, 0x00U, 2U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_RPDO1_COMM, 0x01U, cob_id);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_TPDO1 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_TPDO1_COMM, 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_TPDO1_COMM, 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, SERVO_CAN_OBJ_TPDO1_COMM, 0x03U, 0U);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, SERVO_CAN_OBJ_TPDO1_COMM, 0x05U, event_timer_ms);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_TPDO1_MAP, 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, SERVO_CAN_OBJ_TPDO1_MAP, 0x01U, SERVO_CAN_OBJ_POSITION_ACTUAL, 0x00U, 0x20U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, SERVO_CAN_OBJ_TPDO1_MAP, 0x02U, SERVO_CAN_OBJ_VELOCITY_ACTUAL, 0x00U, 0x20U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, SERVO_CAN_OBJ_TPDO1_MAP, 0x00U, 2U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, SERVO_CAN_OBJ_TPDO1_COMM, 0x01U, cob_id);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_RPDO2 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 1U), 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 1U), 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 1U), 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 1U), 0x01U, SERVO_CAN_OBJ_TARGET_VELOCITY, 0x00U, 0x20U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 1U), 0x02U, SERVO_CAN_OBJ_TARGET_TORQUE, 0x00U, 0x10U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 1U), 0x03U, SERVO_CAN_OBJ_CONTROLWORD, 0x00U, 0x10U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 1U), 0x00U, 3U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 1U), 0x01U, cob_id);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_TPDO2 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 1U), 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 1U), 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 1U), 0x03U, 0U);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 1U), 0x05U, event_timer_ms);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x01U, SERVO_CAN_OBJ_TORQUE_ACTUAL, 0x00U, 0x10U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x02U, SERVO_CAN_OBJ_CURRENT_ACTUAL, 0x00U, 0x10U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x03U, SERVO_CAN_OBJ_STATUSWORD, 0x00U, 0x10U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x04U, SERVO_CAN_OBJ_MODE_DISPLAY, 0x00U, 0x08U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 1U), 0x00U, 4U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 1U), 0x01U, cob_id);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_RPDO3 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 2U), 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 2U), 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 2U), 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 2U), 0x01U, SERVO_CAN_OBJ_MODE_OF_OPERATION, 0x00U, 0x08U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_MAP + 2U), 0x00U, 1U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_RPDO1_COMM + 2U), 0x01U, cob_id);
    Servo_CAN_StepDelay();

    cob_id = SERVO_CAN_COB_TPDO3 + node_id;
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 2U), 0x01U, SERVO_CAN_PDO_DISABLE_MASK | cob_id);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 2U), 0x02U, 0xFFU);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 2U), 0x03U, 0U);
    ret |= Servo_CAN_SdoWriteU16(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 2U), 0x05U, event_timer_ms);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 2U), 0x00U, 0U);
    ret |= Servo_CAN_ConfigPDOMapU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 2U), 0x01U, SERVO_CAN_OBJ_ERROR_CODE, 0x00U, 0x10U);
    ret |= Servo_CAN_SdoWriteU8(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_MAP + 2U), 0x00U, 1U);
    ret |= Servo_CAN_SdoWriteU32(hcan, node_id, (uint16_t)(SERVO_CAN_OBJ_TPDO1_COMM + 2U), 0x01U, cob_id);

    return ret;
}

uint8_t Servo_CAN_SendRPDO1(hcan_t *hcan, uint8_t node_id, int32_t target_position, uint32_t profile_velocity)
{
    uint8_t data[8];

    Servo_CAN_WriteU32(&data[0], (uint32_t)target_position);
    Servo_CAN_WriteU32(&data[4], profile_velocity);

    return Servo_CAN_SendFrame(hcan, (uint16_t)(SERVO_CAN_COB_RPDO1 + node_id), data, 8U);
}

uint8_t Servo_CAN_SendRPDO2(hcan_t *hcan, uint8_t node_id, int32_t target_velocity, int16_t target_torque, uint16_t controlword)
{
    uint8_t data[8];

    Servo_CAN_WriteU32(&data[0], (uint32_t)target_velocity);
    Servo_CAN_WriteU16(&data[4], (uint16_t)target_torque);
    Servo_CAN_WriteU16(&data[6], controlword);

    return Servo_CAN_SendFrame(hcan, (uint16_t)(SERVO_CAN_COB_RPDO2 + node_id), data, 8U);
}

uint8_t Servo_CAN_SendRPDO3(hcan_t *hcan, uint8_t node_id, int8_t mode)
{
    uint8_t data[1];

    data[0] = (uint8_t)mode;

    return Servo_CAN_SendFrame(hcan, (uint16_t)(SERVO_CAN_COB_RPDO3 + node_id), data, 1U);
}

uint8_t Servo_CAN_ParseFrame(Servo_CAN_Motor_t *motor, uint32_t identifier, uint32_t id_type, const uint8_t *data, uint8_t len)
{
    uint16_t base_id;
    uint8_t node_id;

    if ((motor == NULL) || (data == NULL) || (id_type != FDCAN_STANDARD_ID))
    {
        return 0U;
    }

    node_id = motor->node_id;

    if (identifier == (SERVO_CAN_COB_HEARTBEAT + node_id))
    {
        if (len < 1U)
        {
            return 0U;
        }
        motor->heartbeat_state = data[0];
        motor->online = 1U;
        motor->last_update_tick = HAL_GetTick();
        return 1U;
    }

    if (identifier == (SERVO_CAN_COB_EMCY + node_id))
    {
        if (len < 3U)
        {
            return 0U;
        }
        motor->emergency_error_code = Servo_CAN_ReadU16(&data[0]);
        motor->error_register = data[2];
        motor->last_update_tick = HAL_GetTick();
        return 1U;
    }

    if (identifier == (SERVO_CAN_COB_SDO_TX + node_id))
    {
        if (len < 8U)
        {
            return 0U;
        }
        motor->sdo_cs = data[0];
        motor->sdo_index = Servo_CAN_ReadU16(&data[1]);
        motor->sdo_subindex = data[3];
        motor->sdo_data = Servo_CAN_ReadU32(&data[4]);
        motor->sdo_abort_code = 0U;
        motor->sdo_size = 0U;
        motor->sdo_valid = 1U;

        if (data[0] == 0x80U)
        {
            motor->sdo_abort_code = motor->sdo_data;
        }
        else if (data[0] == 0x4FU)
        {
            motor->sdo_size = 1U;
        }
        else if (data[0] == 0x4BU)
        {
            motor->sdo_size = 2U;
        }
        else if (data[0] == 0x47U)
        {
            motor->sdo_size = 3U;
        }
        else if (data[0] == 0x43U)
        {
            motor->sdo_size = 4U;
        }
        motor->last_update_tick = HAL_GetTick();
        return 1U;
    }

    base_id = (uint16_t)(identifier - node_id);
    switch (base_id)
    {
    case SERVO_CAN_COB_TPDO1:
        if (len < 8U)
        {
            return 0U;
        }
        motor->position_actual = (int32_t)Servo_CAN_ReadU32(&data[0]);
        motor->velocity_actual = (int32_t)Servo_CAN_ReadU32(&data[4]);
        break;

    case SERVO_CAN_COB_TPDO2:
        if (len < 7U)
        {
            return 0U;
        }
        motor->torque_actual = (int16_t)Servo_CAN_ReadU16(&data[0]);
        motor->current_actual = (int16_t)Servo_CAN_ReadU16(&data[2]);
        motor->statusword = Servo_CAN_ReadU16(&data[4]);
        motor->mode_display = (int8_t)data[6];
        break;

    case SERVO_CAN_COB_TPDO3:
        if (len < 2U)
        {
            return 0U;
        }
        motor->error_code = Servo_CAN_ReadU16(&data[0]);
        break;

    default:
        return 0U;
    }

    motor->online = 1U;
    motor->last_update_tick = HAL_GetTick();
    return 1U;
}

uint8_t Servo_CAN_StatusReadyToSwitchOn(uint16_t statusword)
{
    return ((statusword & 0x006FU) == 0x0021U) ? 1U : 0U;
}

uint8_t Servo_CAN_StatusSwitchedOn(uint16_t statusword)
{
    return ((statusword & 0x006FU) == 0x0023U) ? 1U : 0U;
}

uint8_t Servo_CAN_StatusOperationEnabled(uint16_t statusword)
{
    return ((statusword & 0x006FU) == 0x0027U) ? 1U : 0U;
}

uint8_t Servo_CAN_StatusFault(uint16_t statusword)
{
    return ((statusword & 0x0008U) != 0U) ? 1U : 0U;
}

uint8_t Servo_CAN_StatusTargetReached(uint16_t statusword)
{
    return ((statusword & 0x0400U) != 0U) ? 1U : 0U;
}

int32_t Servo_CAN_DegreeToCount(float degree, uint32_t pulses_per_rev)
{
    float count;

    if (pulses_per_rev == 0U)
    {
        return 0;
    }

    count = degree * (float)pulses_per_rev / 360.0f;
    return (int32_t)((count >= 0.0f) ? (count + 0.5f) : (count - 0.5f));
}

int32_t Servo_CAN_RpmToCountPerSec(float rpm, uint32_t pulses_per_rev)
{
    float count;

    if (pulses_per_rev == 0U)
    {
        return 0;
    }

    count = rpm * (float)pulses_per_rev / 60.0f;
    return (int32_t)((count >= 0.0f) ? (count + 0.5f) : (count - 0.5f));
}
