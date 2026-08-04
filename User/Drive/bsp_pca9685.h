#ifndef BSP_PCA9685_H
#define BSP_PCA9685_H

#include "stm32h7xx.h"
#include <stdint.h>

#define PCA9685_ADDR        0x80U
#define PCA9685_MODE1       0x00U
#define PCA9685_PRESCALE    0xFEU
#define PCA9685_LED0_ON_L   0x06U

typedef enum
{
    PCA9685_SUBMIT_ERROR = 0U,
    PCA9685_SUBMIT_ACCEPTED = 1U,
    PCA9685_SUBMIT_BUSY = 2U,
    PCA9685_SUBMIT_NOT_READY = 3U
} PCA9685_SubmitStatus_t;

typedef enum
{
    PCA9685_FRAME_RESULT_NONE = 0U,
    PCA9685_FRAME_RESULT_SUCCESS = 1U,
    PCA9685_FRAME_RESULT_ERROR = 2U
} PCA9685_FrameResult_t;

void PCA9685_Init(float freq);
uint8_t PCA9685_ServiceRecovery(void);
void PCA9685_SetDuty(uint8_t channel, float duty);
/* 兼容接口：返回1只表示已提交，实际结果必须由PCA9685_TakeFrameResult确认。 */
uint8_t PCA9685_SetDuties(uint8_t first_channel, const float *duties, uint8_t count);
/* 运行期推荐接口：单缓冲保证上一帧完成前不会覆盖正在发送的数据。 */
PCA9685_SubmitStatus_t PCA9685_SubmitDuties(uint8_t first_channel,
                                            const float *duties,
                                            uint8_t count);
/* 每个5 ms任务周期调用一次；NONE表示仍在发送或当前没有待确认帧。 */
PCA9685_FrameResult_t PCA9685_TakeFrameResult(uint32_t *duration_ms);

extern volatile uint32_t pca9685_ready_debug;
extern volatile uint32_t pca9685_i2c_bus_debug;
extern volatile uint32_t pca9685_addr_7bit_debug;
extern volatile uint32_t pca9685_last_hal_status_debug;
extern volatile uint32_t pca9685_write_error_count_debug;
extern volatile uint32_t pca9685_read_error_count_debug;
extern volatile uint32_t pca9685_consecutive_error_count_debug;
/*
 * 恢复状态：0=正常，1=等待重试，2=重置I2C2，3~7=异步恢复配置，
 * 8=等待PCA9685振荡器稳定，9~10=完成配置。ServiceRecovery返回1表示恢复完成。
 */
extern volatile uint32_t pca9685_recovery_state_debug;
extern volatile uint32_t pca9685_recovery_attempt_count_debug;
extern volatile uint32_t pca9685_recovery_success_count_debug;
extern volatile uint32_t pca9685_recovery_failure_count_debug;
extern volatile uint32_t pca9685_recovery_next_retry_tick_debug;
/*
 * 运行期发送状态。busy=1 表示一帧仍由 I2C2 中断发送；正常情况下整帧约 2~3 ms。
 * timeout_count 或 error_callback_count 增长表示总线、供电或接线需要检查。
 */
extern volatile uint32_t pca9685_async_busy_debug;
extern volatile uint32_t pca9685_async_owner_debug;
extern volatile uint32_t pca9685_async_submit_count_debug;
extern volatile uint32_t pca9685_async_completion_count_debug;
extern volatile uint32_t pca9685_async_busy_reject_count_debug;
extern volatile uint32_t pca9685_async_timeout_count_debug;
extern volatile uint32_t pca9685_async_event_callback_count_debug;
extern volatile uint32_t pca9685_async_error_callback_count_debug;
extern volatile uint32_t pca9685_async_last_duration_ms_debug;
extern volatile uint32_t pca9685_async_max_duration_ms_debug;
extern volatile uint32_t pca9685_async_last_error_code_debug;

#endif
