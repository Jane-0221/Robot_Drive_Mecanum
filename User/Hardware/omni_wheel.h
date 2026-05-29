#ifndef __OMNI_WHEEL_H__
#define __OMNI_WHEEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float x_cmd;
    float y_cmd;
    float w_cmd_in;
    float w_cmd_out;
    float yaw_now;
    float yaw_target;
    float yaw_correction;
    float wheel_cmd1;
    float wheel_cmd2;
    float wheel_cmd3;
    float fb_speed1;
    float fb_speed2;
    float fb_speed3;
    uint8_t run_mode1;
    uint8_t run_mode2;
    uint8_t run_mode3;
    uint8_t pattern1;
    uint8_t pattern2;
    uint8_t pattern3;
    uint8_t ready_mask;
    uint8_t tx_slot;
    uint32_t update_tick_ms;
    uint32_t update_count;
    uint32_t tx_last_tick_ms;
    uint32_t tx_count1;
    uint32_t tx_count2;
    uint32_t tx_count3;
    uint32_t last_feedback_tick1;
    uint32_t last_feedback_tick2;
    uint32_t last_feedback_tick3;
    uint32_t enable_retry_last_tick1;
    uint32_t enable_retry_last_tick2;
    uint32_t enable_retry_last_tick3;
    uint32_t enable_retry_count1;
    uint32_t enable_retry_count2;
    uint32_t enable_retry_count3;
} Omni_Wheel_Debug_t;

extern volatile Omni_Wheel_Debug_t omni_debug;

void Omni_Wheel_Init(void);
void Omni_Wheel_Update(void);
void Omni_Wheel_RxCallback(uint32_t ext_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __OMNI_WHEEL_H__ */
