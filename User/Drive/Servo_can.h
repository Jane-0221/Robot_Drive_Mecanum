#ifndef __SERVO_CAN_H__
#define __SERVO_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can_receive_send.h"

#ifndef SERVO_CAN_COMMAND_DELAY_MS
#define SERVO_CAN_COMMAND_DELAY_MS 10U
#endif

#define SERVO_CAN_NODE_BROADCAST 0U

#define SERVO_CAN_COB_NMT       0x000U
#define SERVO_CAN_COB_EMCY      0x080U
#define SERVO_CAN_COB_TPDO1     0x180U
#define SERVO_CAN_COB_RPDO1     0x200U
#define SERVO_CAN_COB_TPDO2     0x280U
#define SERVO_CAN_COB_RPDO2     0x300U
#define SERVO_CAN_COB_TPDO3     0x380U
#define SERVO_CAN_COB_RPDO3     0x400U
#define SERVO_CAN_COB_TPDO4     0x480U
#define SERVO_CAN_COB_RPDO4     0x500U
#define SERVO_CAN_COB_SDO_TX    0x580U
#define SERVO_CAN_COB_SDO_RX    0x600U
#define SERVO_CAN_COB_HEARTBEAT 0x700U

typedef enum
{
    SERVO_CAN_NMT_START = 0x01,
    SERVO_CAN_NMT_STOP = 0x02,
    SERVO_CAN_NMT_PRE_OPERATIONAL = 0x80,
    SERVO_CAN_NMT_RESET_NODE = 0x81,
    SERVO_CAN_NMT_RESET_COMMUNICATION = 0x82,
} Servo_CAN_NmtCommand_t;

typedef enum
{
    SERVO_CAN_MODE_PROFILE_POSITION = 1,
    SERVO_CAN_MODE_PROFILE_VELOCITY = 3,
    SERVO_CAN_MODE_PROFILE_TORQUE = 4,
    SERVO_CAN_MODE_HOMING = 6,
} Servo_CAN_Mode_t;

typedef enum
{
    SERVO_CAN_HEARTBEAT_STOPPED = 0x04,
    SERVO_CAN_HEARTBEAT_OPERATIONAL = 0x05,
    SERVO_CAN_HEARTBEAT_PRE_OPERATIONAL = 0x7F,
} Servo_CAN_HeartbeatState_t;

typedef struct
{
    uint8_t node_id;

    int32_t position_actual;
    int32_t velocity_actual;
    int16_t torque_actual;
    int16_t current_actual;
    uint16_t statusword;
    int8_t mode_display;
    uint16_t error_code;

    uint16_t emergency_error_code;
    uint8_t error_register;
    uint8_t heartbeat_state;
    uint8_t online;
    uint32_t last_update_tick;

    uint8_t sdo_valid;
    uint8_t sdo_cs;
    uint16_t sdo_index;
    uint8_t sdo_subindex;
    uint8_t sdo_size;
    uint32_t sdo_data;
    uint32_t sdo_abort_code;
} Servo_CAN_Motor_t;

void Servo_CAN_MotorInit(Servo_CAN_Motor_t *motor, uint8_t node_id);

uint8_t Servo_CAN_SendFrame(hcan_t *hcan, uint16_t cob_id, const uint8_t *data, uint8_t len);
uint8_t Servo_CAN_NMT(hcan_t *hcan, uint8_t node_id, Servo_CAN_NmtCommand_t cmd);
uint8_t Servo_CAN_StartNode(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_StopNode(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_PreOperational(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_ResetNode(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_ResetCommunication(hcan_t *hcan, uint8_t node_id);

uint8_t Servo_CAN_SdoRead(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex);
uint8_t Servo_CAN_SdoWriteU8(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint8_t value);
uint8_t Servo_CAN_SdoWriteU16(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint16_t value);
uint8_t Servo_CAN_SdoWriteU32(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value);
uint8_t Servo_CAN_SdoWriteI32(hcan_t *hcan, uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value);

uint8_t Servo_CAN_SetHeartbeat(hcan_t *hcan, uint8_t node_id, uint16_t heartbeat_ms);
uint8_t Servo_CAN_SetSyncCobId(hcan_t *hcan, uint8_t node_id, uint32_t sync_cob_id);

uint8_t Servo_CAN_WriteControlword(hcan_t *hcan, uint8_t node_id, uint16_t controlword);
uint8_t Servo_CAN_SetMode(hcan_t *hcan, uint8_t node_id, Servo_CAN_Mode_t mode);
uint8_t Servo_CAN_Enable(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_Disable(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_Stop(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_QuickStop(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_FaultReset(hcan_t *hcan, uint8_t node_id);

uint8_t Servo_CAN_SetTargetVelocity(hcan_t *hcan, uint8_t node_id, int32_t velocity_count_per_sec);
uint8_t Servo_CAN_SetTargetVelocityRPM(hcan_t *hcan, uint8_t node_id, float rpm, uint32_t pulses_per_rev);
uint8_t Servo_CAN_SetProfileVelocity(hcan_t *hcan, uint8_t node_id, uint32_t velocity);
uint8_t Servo_CAN_SetProfileAcceleration(hcan_t *hcan, uint8_t node_id, uint32_t acceleration);
uint8_t Servo_CAN_SetProfileDeceleration(hcan_t *hcan, uint8_t node_id, uint32_t deceleration);
uint8_t Servo_CAN_SetTargetTorquePermille(hcan_t *hcan, uint8_t node_id, int16_t torque_permille);
uint8_t Servo_CAN_SetTargetTorquePercent(hcan_t *hcan, uint8_t node_id, float torque_percent);
uint8_t Servo_CAN_SetTargetPosition(hcan_t *hcan, uint8_t node_id, int32_t position_count);
uint8_t Servo_CAN_SetTargetPositionDegree(hcan_t *hcan, uint8_t node_id, float degree, uint32_t pulses_per_rev);
uint8_t Servo_CAN_StartAbsolutePosition(hcan_t *hcan, uint8_t node_id, uint8_t immediate);
uint8_t Servo_CAN_StartRelativePosition(hcan_t *hcan, uint8_t node_id, uint8_t immediate);
uint8_t Servo_CAN_SetAbsolutePosition(hcan_t *hcan, uint8_t node_id, int32_t position_count, uint32_t velocity, uint32_t acceleration, uint8_t immediate);
uint8_t Servo_CAN_SetRelativePosition(hcan_t *hcan, uint8_t node_id, int32_t position_count, uint32_t velocity, uint32_t acceleration, uint8_t immediate);

uint8_t Servo_CAN_SetHomingMethod(hcan_t *hcan, uint8_t node_id, int8_t method);
uint8_t Servo_CAN_SetHomingSpeed(hcan_t *hcan, uint8_t node_id, uint8_t subindex, uint32_t speed);
uint8_t Servo_CAN_StartHoming(hcan_t *hcan, uint8_t node_id);
uint8_t Servo_CAN_SetCurrentPositionZero(hcan_t *hcan, uint8_t node_id);

uint8_t Servo_CAN_ConfigPDO_Default(hcan_t *hcan, uint8_t node_id, uint16_t event_timer_ms);
uint8_t Servo_CAN_SendRPDO1(hcan_t *hcan, uint8_t node_id, int32_t target_position, uint32_t profile_velocity);
uint8_t Servo_CAN_SendRPDO2(hcan_t *hcan, uint8_t node_id, int32_t target_velocity, int16_t target_torque, uint16_t controlword);
uint8_t Servo_CAN_SendRPDO3(hcan_t *hcan, uint8_t node_id, int8_t mode);

uint8_t Servo_CAN_ParseFrame(Servo_CAN_Motor_t *motor, uint32_t identifier, uint32_t id_type, const uint8_t *data, uint8_t len);

uint8_t Servo_CAN_StatusReadyToSwitchOn(uint16_t statusword);
uint8_t Servo_CAN_StatusSwitchedOn(uint16_t statusword);
uint8_t Servo_CAN_StatusOperationEnabled(uint16_t statusword);
uint8_t Servo_CAN_StatusFault(uint16_t statusword);
uint8_t Servo_CAN_StatusTargetReached(uint16_t statusword);

int32_t Servo_CAN_DegreeToCount(float degree, uint32_t pulses_per_rev);
int32_t Servo_CAN_RpmToCountPerSec(float rpm, uint32_t pulses_per_rev);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_CAN_H__ */
