#ifndef __MECANUM_WHEEL_H__
#define __MECANUM_WHEEL_H__

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
    float wheel_cmd_lf;
    float wheel_cmd_rf;
    float wheel_cmd_rb;
    float wheel_cmd_lb;
    float wheel_raw_max_abs;
    float wheel_limit_scale;
    float fb_speed_lf;
    float fb_speed_rf;
    float fb_speed_rb;
    float fb_speed_lb;
    uint8_t run_mode_lf;
    uint8_t run_mode_rf;
    uint8_t run_mode_rb;
    uint8_t run_mode_lb;
    uint8_t pattern_lf;
    uint8_t pattern_rf;
    uint8_t pattern_rb;
    uint8_t pattern_lb;
    uint8_t ready_mask;
    uint8_t tx_slot;
    uint32_t update_tick_ms;
    uint32_t update_count;
    uint32_t tx_last_tick_ms;
    uint32_t tx_count_lf;
    uint32_t tx_count_rf;
    uint32_t tx_count_rb;
    uint32_t tx_count_lb;
    uint32_t last_feedback_tick_lf;
    uint32_t last_feedback_tick_rf;
    uint32_t last_feedback_tick_rb;
    uint32_t last_feedback_tick_lb;
    uint32_t enable_retry_last_tick_lf;
    uint32_t enable_retry_last_tick_rf;
    uint32_t enable_retry_last_tick_rb;
    uint32_t enable_retry_last_tick_lb;
    uint32_t enable_retry_count_lf;
    uint32_t enable_retry_count_rf;
    uint32_t enable_retry_count_rb;
    uint32_t enable_retry_count_lb;
} Mecanum_Wheel_Debug_t;

extern volatile Mecanum_Wheel_Debug_t mecanum_debug;
extern volatile uint32_t mecanum_single_wheel_test_request_debug;
extern volatile uint32_t mecanum_single_wheel_test_active_debug;
extern volatile uint32_t mecanum_single_wheel_test_duration_ms_debug;
extern volatile uint32_t mecanum_single_wheel_test_end_tick_debug;
extern volatile float mecanum_single_wheel_test_speed_debug;
extern volatile uint32_t mecanum_vector_test_request_debug;
extern volatile uint32_t mecanum_vector_test_active_debug;
extern volatile uint32_t mecanum_vector_test_duration_ms_debug;
extern volatile uint32_t mecanum_vector_test_end_tick_debug;
extern volatile float mecanum_vector_test_x_debug;
extern volatile float mecanum_vector_test_y_debug;
extern volatile float mecanum_vector_test_w_debug;
extern volatile uint32_t mecanum_rx_ext_total_debug;
extern volatile uint32_t mecanum_rx_last_ext_id_debug;
extern volatile uint32_t mecanum_rx_last_target_id_debug;
extern volatile uint32_t mecanum_rx_target_count_debug[16];

void Mecanum_Wheel_Init(void);
void Mecanum_Wheel_Update(void);
void Mecanum_Wheel_RxCallback(uint32_t ext_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_WHEEL_H__ */
