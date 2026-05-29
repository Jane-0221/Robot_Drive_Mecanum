#ifndef __SERVO_LIFT_CONTROL_H__
#define __SERVO_LIFT_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "Servo_can.h"

#define LIFT_TARGET_HEIGHT_MIN_MM 70

#ifndef SERVO_LIFT_NODE_ID
#define SERVO_LIFT_NODE_ID 1U
#endif

#ifndef SERVO_LIFT_HEIGHT_MIN_MM
#define SERVO_LIFT_HEIGHT_MIN_MM 70
#endif

#ifndef SERVO_LIFT_HEIGHT_MAX_MM
#define SERVO_LIFT_HEIGHT_MAX_MM 700
#endif

#ifndef SERVO_LIFT_PULSE_PER_MM
#define SERVO_LIFT_PULSE_PER_MM 2000.0f
#endif

#ifndef SERVO_LIFT_DIRECTION
#define SERVO_LIFT_DIRECTION 1
#endif

#ifndef SERVO_LIFT_PROFILE_VELOCITY
#define SERVO_LIFT_PROFILE_VELOCITY 20000U
#endif

#ifndef SERVO_LIFT_PROFILE_ACCELERATION
#define SERVO_LIFT_PROFILE_ACCELERATION 10000U
#endif

#ifndef SERVO_LIFT_TPDO_EVENT_TIMER_MS
#define SERVO_LIFT_TPDO_EVENT_TIMER_MS 20U
#endif

typedef struct
{
    uint8_t initialized;
    uint8_t reference_valid;
    uint8_t online;
    uint8_t operation_enabled;
    uint8_t fault;
    uint8_t last_tx_status;
    int16_t target_height_mm;
    int16_t actual_height_mm;
    int32_t reference_position_count;
    int32_t actual_position_count;
    int32_t target_position_count;
    uint16_t statusword;
    uint16_t error_code;
    uint32_t rx_count;
    uint32_t target_tx_count;
    uint32_t enable_retry_count;
    uint32_t sdo_position_read_count;
    uint32_t last_rx_tick_ms;
    uint32_t last_target_tx_tick_ms;
    uint32_t last_enable_retry_tick_ms;
} Servo_Lift_Debug_t;

extern int16_t aim_tx_height;
extern int16_t lift_height_final;
extern uint16_t lift_current_height;
extern volatile Servo_Lift_Debug_t servo_lift_debug;

void Servo_Lift_Init(void);
void Servo_Lift_Update(void);
void Servo_Lift_GoToTarget(int16_t target_height);
uint16_t Servo_Lift_GetHeight(void);
void Servo_Lift_RxCallback(uint32_t identifier, uint32_t id_type, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_LIFT_CONTROL_H__ */
