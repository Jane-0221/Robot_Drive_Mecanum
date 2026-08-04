#include "bsp_pca9685.h"
#include "i2c.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PCA9685_CHANNEL_COUNT 16U
#define PCA9685_ASYNC_BUFFER_SIZE (PCA9685_CHANNEL_COUNT * 4U)
#define PCA9685_MODE2         0x01U
#define PCA9685_ALL_LED_ON_L  0xFAU
#define PCA9685_I2C_TIMEOUT_MS 10U
#define PCA9685_ASYNC_TIMEOUT_MS 10U
#define PCA9685_MAX_CONSECUTIVE_ERRORS 3U
#define PCA9685_RECOVERY_RETRY_MS 500U
#define PCA9685_OSCILLATOR_STARTUP_MS 5U

typedef enum
{
    PCA9685_RECOVERY_READY = 0U,
    PCA9685_RECOVERY_WAIT_RETRY = 1U,
    PCA9685_RECOVERY_RESET_I2C = 2U,
    PCA9685_RECOVERY_WRITE_MODE1_RESET = 3U,
    PCA9685_RECOVERY_WRITE_MODE2 = 4U,
    PCA9685_RECOVERY_WRITE_SLEEP = 5U,
    PCA9685_RECOVERY_WRITE_PRESCALE = 6U,
    PCA9685_RECOVERY_WRITE_WAKE = 7U,
    PCA9685_RECOVERY_WAIT_OSCILLATOR = 8U,
    PCA9685_RECOVERY_WRITE_RESTART = 9U,
    PCA9685_RECOVERY_WRITE_ALL_OFF = 10U
} PCA9685_RecoveryState_t;

typedef enum
{
    PCA9685_ASYNC_OWNER_NONE = 0U,
    PCA9685_ASYNC_OWNER_FRAME = 1U,
    PCA9685_ASYNC_OWNER_RECOVERY = 2U
} PCA9685_AsyncOwner_t;

typedef enum
{
    PCA9685_ASYNC_COMPLETION_NONE = 0U,
    PCA9685_ASYNC_COMPLETION_SUCCESS = 1U,
    PCA9685_ASYNC_COMPLETION_ERROR = 2U
} PCA9685_AsyncCompletion_t;

volatile uint32_t pca9685_ready_debug = 0U;
volatile uint32_t pca9685_i2c_bus_debug = 2U;
volatile uint32_t pca9685_addr_7bit_debug = (uint32_t)(PCA9685_ADDR >> 1);
volatile uint32_t pca9685_last_hal_status_debug = HAL_ERROR;
volatile uint32_t pca9685_write_error_count_debug = 0U;
volatile uint32_t pca9685_read_error_count_debug = 0U;
volatile uint32_t pca9685_consecutive_error_count_debug = 0U;
volatile uint32_t pca9685_recovery_state_debug = PCA9685_RECOVERY_READY;
volatile uint32_t pca9685_recovery_attempt_count_debug = 0U;
volatile uint32_t pca9685_recovery_success_count_debug = 0U;
volatile uint32_t pca9685_recovery_failure_count_debug = 0U;
volatile uint32_t pca9685_recovery_next_retry_tick_debug = 0U;
volatile uint32_t pca9685_async_busy_debug = 0U;
volatile uint32_t pca9685_async_owner_debug = PCA9685_ASYNC_OWNER_NONE;
volatile uint32_t pca9685_async_submit_count_debug = 0U;
volatile uint32_t pca9685_async_completion_count_debug = 0U;
volatile uint32_t pca9685_async_busy_reject_count_debug = 0U;
volatile uint32_t pca9685_async_timeout_count_debug = 0U;
volatile uint32_t pca9685_async_event_callback_count_debug = 0U;
volatile uint32_t pca9685_async_error_callback_count_debug = 0U;
volatile uint32_t pca9685_async_last_duration_ms_debug = 0U;
volatile uint32_t pca9685_async_max_duration_ms_debug = 0U;
volatile uint32_t pca9685_async_last_error_code_debug = HAL_I2C_ERROR_NONE;

static I2C_HandleTypeDef *const pca9685_i2c_handle = &hi2c2;
static PCA9685_RecoveryState_t pca9685_recovery_state =
    PCA9685_RECOVERY_READY;
static uint8_t pca9685_prescale = 0U;
static uint32_t pca9685_oscillator_wait_start_tick = 0U;
static uint8_t pca9685_async_buffer[PCA9685_ASYNC_BUFFER_SIZE];
static volatile PCA9685_AsyncOwner_t pca9685_async_owner =
    PCA9685_ASYNC_OWNER_NONE;
static volatile PCA9685_AsyncCompletion_t pca9685_async_completion =
    PCA9685_ASYNC_COMPLETION_NONE;
static volatile HAL_StatusTypeDef pca9685_async_completion_status = HAL_OK;
static volatile uint32_t pca9685_async_start_tick = 0U;

static uint8_t PCA9685_TickReached(uint32_t now, uint32_t target)
{
    return (((int32_t)(now - target)) >= 0) ? 1U : 0U;
}

static void PCA9685_SetRecoveryState(PCA9685_RecoveryState_t state)
{
    pca9685_recovery_state = state;
    pca9685_recovery_state_debug = (uint32_t)state;
}

static void PCA9685_ScheduleRecovery(uint8_t count_failure)
{
    if ((count_failure != 0U) &&
        (pca9685_recovery_failure_count_debug < UINT32_MAX))
    {
        pca9685_recovery_failure_count_debug++;
    }

    pca9685_ready_debug = 0U;
    pca9685_recovery_next_retry_tick_debug =
        HAL_GetTick() + PCA9685_RECOVERY_RETRY_MS;
    PCA9685_SetRecoveryState(PCA9685_RECOVERY_WAIT_RETRY);
}

static uint8_t PCA9685_CalculatePrescale(float freq, uint8_t *prescale)
{
    float prescale_value;

    if ((prescale == NULL) ||
        (!isfinite(freq)) ||
        (freq < 24.0f) ||
        (freq > 1526.0f))
    {
        return 0U;
    }

    prescale_value = (25000000.0f / (4096.0f * freq)) - 1.0f;
    *prescale = (uint8_t)(prescale_value + 0.5f);
    return 1U;
}

static void PCA9685_RecordStatus(HAL_StatusTypeDef status, uint8_t is_write)
{
    pca9685_last_hal_status_debug = (uint32_t)status;

    if (status == HAL_OK)
    {
        pca9685_consecutive_error_count_debug = 0U;
        return;
    }

    if (is_write != 0U)
    {
        pca9685_write_error_count_debug++;
    }
    else
    {
        pca9685_read_error_count_debug++;
    }

    if (pca9685_consecutive_error_count_debug < UINT32_MAX)
    {
        pca9685_consecutive_error_count_debug++;
    }

    if ((pca9685_ready_debug != 0U) &&
        (pca9685_consecutive_error_count_debug >=
         PCA9685_MAX_CONSECUTIVE_ERRORS))
    {
        PCA9685_ScheduleRecovery(0U);
    }
}

static void PCA9685_RestoreInterruptState(uint32_t previous_primask)
{
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static void PCA9685_ResetAsyncState(void)
{
    uint32_t previous_primask = __get_PRIMASK();

    __disable_irq();
    pca9685_async_owner = PCA9685_ASYNC_OWNER_NONE;
    pca9685_async_completion = PCA9685_ASYNC_COMPLETION_NONE;
    pca9685_async_completion_status = HAL_OK;
    pca9685_async_start_tick = 0U;
    pca9685_async_busy_debug = 0U;
    pca9685_async_owner_debug = PCA9685_ASYNC_OWNER_NONE;
    PCA9685_RestoreInterruptState(previous_primask);
}

static PCA9685_SubmitStatus_t PCA9685_SubmitWrite(
    PCA9685_AsyncOwner_t owner,
    uint8_t reg,
    const uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status;
    uint32_t previous_primask;

    if ((owner == PCA9685_ASYNC_OWNER_NONE) ||
        (data == NULL) ||
        (length == 0U) ||
        (length > PCA9685_ASYNC_BUFFER_SIZE))
    {
        return PCA9685_SUBMIT_ERROR;
    }

    /*
     * 只在复制一帧和声明所有权时短暂关中断。发送过程全部由 I2C2 IRQ 完成，
     * 不会在 5 ms 舵机任务内轮询等待总线。
     */
    previous_primask = __get_PRIMASK();
    __disable_irq();
    if (pca9685_async_owner != PCA9685_ASYNC_OWNER_NONE)
    {
        PCA9685_RestoreInterruptState(previous_primask);
        if (pca9685_async_busy_reject_count_debug < UINT32_MAX)
        {
            pca9685_async_busy_reject_count_debug++;
        }
        return PCA9685_SUBMIT_BUSY;
    }

    memcpy(pca9685_async_buffer, data, length);
    pca9685_async_completion = PCA9685_ASYNC_COMPLETION_NONE;
    pca9685_async_completion_status = HAL_OK;
    pca9685_async_start_tick = HAL_GetTick();
    pca9685_async_last_error_code_debug = HAL_I2C_ERROR_NONE;
    pca9685_async_owner = owner;
    pca9685_async_owner_debug = (uint32_t)owner;
    pca9685_async_busy_debug = 1U;
    __DMB();
    PCA9685_RestoreInterruptState(previous_primask);

    status = HAL_I2C_Mem_Write_IT(pca9685_i2c_handle,
                                  PCA9685_ADDR,
                                  reg,
                                  I2C_MEMADD_SIZE_8BIT,
                                  pca9685_async_buffer,
                                  length);
    if (status == HAL_OK)
    {
        if (pca9685_async_submit_count_debug < UINT32_MAX)
        {
            pca9685_async_submit_count_debug++;
        }
        return PCA9685_SUBMIT_ACCEPTED;
    }

    previous_primask = __get_PRIMASK();
    __disable_irq();
    pca9685_async_owner = PCA9685_ASYNC_OWNER_NONE;
    pca9685_async_owner_debug = PCA9685_ASYNC_OWNER_NONE;
    pca9685_async_busy_debug = 0U;
    PCA9685_RestoreInterruptState(previous_primask);

    PCA9685_RecordStatus(status, 1U);
    return PCA9685_SUBMIT_ERROR;
}

static void PCA9685_CompleteAsyncFromIsr(PCA9685_AsyncCompletion_t completion,
                                         HAL_StatusTypeDef status,
                                         uint32_t error_code)
{
    uint32_t duration_ms;

    if ((pca9685_async_owner == PCA9685_ASYNC_OWNER_NONE) ||
        (pca9685_async_completion != PCA9685_ASYNC_COMPLETION_NONE))
    {
        return;
    }

    duration_ms = HAL_GetTick() - pca9685_async_start_tick;
    pca9685_async_last_duration_ms_debug = duration_ms;
    if (duration_ms > pca9685_async_max_duration_ms_debug)
    {
        pca9685_async_max_duration_ms_debug = duration_ms;
    }
    pca9685_async_last_error_code_debug = error_code;
    pca9685_async_completion_status = status;
    pca9685_async_busy_debug = 0U;
    if (pca9685_async_completion_count_debug < UINT32_MAX)
    {
        pca9685_async_completion_count_debug++;
    }
    __DMB();
    pca9685_async_completion = completion;
}

static void PCA9685_ServiceAsyncTimeout(uint32_t now)
{
    uint32_t duration_ms;

    if ((pca9685_async_owner == PCA9685_ASYNC_OWNER_NONE) ||
        (pca9685_async_completion != PCA9685_ASYNC_COMPLETION_NONE))
    {
        return;
    }

    duration_ms = now - pca9685_async_start_tick;
    if (duration_ms < PCA9685_ASYNC_TIMEOUT_MS)
    {
        return;
    }

    /*
     * 先屏蔽本外设中断再复查状态，避免完成中断与超时处理同时提交结果。
     * I2C2 只连接 PCA9685，因此这里复位 I2C2 不会影响 I2C1 距离传感器。
     */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
    __DMB();
    if ((pca9685_async_owner == PCA9685_ASYNC_OWNER_NONE) ||
        (pca9685_async_completion != PCA9685_ASYNC_COMPLETION_NONE))
    {
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
        return;
    }

    (void)HAL_I2C_DeInit(pca9685_i2c_handle);
    pca9685_async_last_duration_ms_debug = duration_ms;
    if (duration_ms > pca9685_async_max_duration_ms_debug)
    {
        pca9685_async_max_duration_ms_debug = duration_ms;
    }
    pca9685_async_last_error_code_debug = HAL_I2C_ERROR_TIMEOUT;
    pca9685_async_completion_status = HAL_TIMEOUT;
    pca9685_async_busy_debug = 0U;
    if (pca9685_async_timeout_count_debug < UINT32_MAX)
    {
        pca9685_async_timeout_count_debug++;
    }
    if (pca9685_async_completion_count_debug < UINT32_MAX)
    {
        pca9685_async_completion_count_debug++;
    }
    __DMB();
    pca9685_async_completion = PCA9685_ASYNC_COMPLETION_ERROR;

    /* 超时后外设已反初始化，必须走恢复流程，不能继续向失效句柄提交帧。 */
    PCA9685_ScheduleRecovery(0U);
}

static PCA9685_AsyncCompletion_t PCA9685_TakeAsyncCompletion(
    PCA9685_AsyncOwner_t expected_owner,
    HAL_StatusTypeDef *status,
    uint32_t *duration_ms)
{
    PCA9685_AsyncCompletion_t completion;
    uint32_t previous_primask = __get_PRIMASK();

    __disable_irq();
    if ((pca9685_async_owner != expected_owner) ||
        (pca9685_async_completion == PCA9685_ASYNC_COMPLETION_NONE))
    {
        PCA9685_RestoreInterruptState(previous_primask);
        return PCA9685_ASYNC_COMPLETION_NONE;
    }

    completion = pca9685_async_completion;
    if (status != NULL)
    {
        *status = pca9685_async_completion_status;
    }
    if (duration_ms != NULL)
    {
        *duration_ms = pca9685_async_last_duration_ms_debug;
    }
    pca9685_async_owner = PCA9685_ASYNC_OWNER_NONE;
    pca9685_async_owner_debug = PCA9685_ASYNC_OWNER_NONE;
    pca9685_async_completion = PCA9685_ASYNC_COMPLETION_NONE;
    PCA9685_RestoreInterruptState(previous_primask);
    return completion;
}

static HAL_StatusTypeDef PCA9685_WriteBlockRaw(uint8_t reg,
                                               const uint8_t *data,
                                               uint16_t length,
                                               uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
    {
        pca9685_last_hal_status_debug = HAL_ERROR;
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Write(pca9685_i2c_handle,
                               PCA9685_ADDR,
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data,
                               length,
                               timeout_ms);
    PCA9685_RecordStatus(status, 1U);
    return status;
}

static HAL_StatusTypeDef PCA9685_WriteBlock(uint8_t reg,
                                            const uint8_t *data,
                                            uint16_t length)
{
    if (pca9685_ready_debug == 0U)
    {
        pca9685_last_hal_status_debug = HAL_ERROR;
        return HAL_ERROR;
    }

    return PCA9685_WriteBlockRaw(reg,
                                 data,
                                 length,
                                 PCA9685_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef PCA9685_Write(uint8_t reg, uint8_t data)
{
    return PCA9685_WriteBlock(reg, &data, 1U);
}

static HAL_StatusTypeDef PCA9685_Read(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;

    if ((pca9685_ready_debug == 0U) || (data == NULL))
    {
        pca9685_last_hal_status_debug = HAL_ERROR;
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(pca9685_i2c_handle,
                              PCA9685_ADDR,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              1U,
                              PCA9685_I2C_TIMEOUT_MS);
    PCA9685_RecordStatus(status, 0U);
    return status;
}

static HAL_StatusTypeDef PCA9685_SetFreq(float freq)
{
    uint8_t old_mode;
    uint8_t sleep_mode;
    uint8_t prescale;

    if (PCA9685_CalculatePrescale(freq, &prescale) == 0U)
    {
        return HAL_ERROR;
    }

    pca9685_prescale = prescale;

    if (PCA9685_Read(PCA9685_MODE1, &old_mode) != HAL_OK)
    {
        return HAL_ERROR;
    }

    sleep_mode = (uint8_t)((old_mode & 0x7FU) | 0x10U);
    if (PCA9685_Write(PCA9685_MODE1, sleep_mode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PCA9685_Write(PCA9685_PRESCALE, prescale) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PCA9685_Write(PCA9685_MODE1, old_mode) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(PCA9685_OSCILLATOR_STARTUP_MS);

    /* RESTART + auto-increment + ALLCALL. Auto-increment enables one-frame updates. */
    return PCA9685_Write(PCA9685_MODE1, (uint8_t)(old_mode | 0xA1U));
}

void PCA9685_Init(float freq)
{
    HAL_StatusTypeDef status;
    static const uint8_t all_channels_off[4] = {0U, 0U, 0U, 0U};

    pca9685_ready_debug = 0U;
    pca9685_i2c_bus_debug = 2U;
    pca9685_addr_7bit_debug = (uint32_t)(PCA9685_ADDR >> 1);
    pca9685_last_hal_status_debug = HAL_ERROR;
    pca9685_consecutive_error_count_debug = 0U;
    pca9685_recovery_state_debug = PCA9685_RECOVERY_READY;
    pca9685_recovery_attempt_count_debug = 0U;
    pca9685_recovery_success_count_debug = 0U;
    pca9685_recovery_failure_count_debug = 0U;
    pca9685_recovery_next_retry_tick_debug = 0U;
    pca9685_async_submit_count_debug = 0U;
    pca9685_async_completion_count_debug = 0U;
    pca9685_async_busy_reject_count_debug = 0U;
    pca9685_async_timeout_count_debug = 0U;
    pca9685_async_event_callback_count_debug = 0U;
    pca9685_async_error_callback_count_debug = 0U;
    pca9685_async_last_duration_ms_debug = 0U;
    pca9685_async_max_duration_ms_debug = 0U;
    pca9685_async_last_error_code_debug = HAL_I2C_ERROR_NONE;
    pca9685_prescale = 0U;
    pca9685_oscillator_wait_start_tick = 0U;
    PCA9685_ResetAsyncState();
    PCA9685_SetRecoveryState(PCA9685_RECOVERY_READY);

    if (PCA9685_CalculatePrescale(freq, &pca9685_prescale) == 0U)
    {
        return;
    }

    /*
     * The PCA9685 is assigned exclusively to I2C2. I2C1 is reserved for the
     * arm distance sensor, so this driver must never probe or reconfigure it.
     */
    status = HAL_I2C_IsDeviceReady(pca9685_i2c_handle,
                                   PCA9685_ADDR,
                                   3U,
                                   PCA9685_I2C_TIMEOUT_MS);
    pca9685_last_hal_status_debug = (uint32_t)status;
    if (status != HAL_OK)
    {
        PCA9685_ScheduleRecovery(0U);
        return;
    }

    pca9685_ready_debug = 1U;
    if (PCA9685_Write(PCA9685_MODE1, 0x00U) != HAL_OK)
    {
        PCA9685_ScheduleRecovery(0U);
        return;
    }
    /* OUTDRV=1 and OCH=0: all channel registers take effect together on STOP. */
    if (PCA9685_Write(PCA9685_MODE2, 0x04U) != HAL_OK)
    {
        PCA9685_ScheduleRecovery(0U);
        return;
    }
    if (PCA9685_SetFreq(freq) != HAL_OK)
    {
        PCA9685_ScheduleRecovery(0U);
        return;
    }
    if (PCA9685_WriteBlock(PCA9685_ALL_LED_ON_L,
                           all_channels_off,
                           (uint16_t)sizeof(all_channels_off)) != HAL_OK)
    {
        PCA9685_ScheduleRecovery(0U);
    }
}

uint8_t PCA9685_ServiceRecovery(void)
{
    static const uint8_t all_channels_off[4] = {0U, 0U, 0U, 0U};
    uint32_t now = HAL_GetTick();
    PCA9685_AsyncCompletion_t completion;
    PCA9685_SubmitStatus_t submit_status = PCA9685_SUBMIT_ERROR;
    HAL_StatusTypeDef status = HAL_OK;

    PCA9685_ServiceAsyncTimeout(now);

    /* 恢复寄存器也异步发送，完成一个步骤后才进入下一个步骤。 */
    if (pca9685_async_owner == PCA9685_ASYNC_OWNER_RECOVERY)
    {
        completion = PCA9685_TakeAsyncCompletion(
            PCA9685_ASYNC_OWNER_RECOVERY,
            &status,
            NULL);
        if (completion == PCA9685_ASYNC_COMPLETION_NONE)
        {
            return 0U;
        }

        PCA9685_RecordStatus(status, 1U);
        if ((completion != PCA9685_ASYNC_COMPLETION_SUCCESS) ||
            (status != HAL_OK))
        {
            PCA9685_ScheduleRecovery(1U);
            return 0U;
        }

        switch (pca9685_recovery_state)
        {
        case PCA9685_RECOVERY_WRITE_MODE1_RESET:
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_MODE2);
            break;
        case PCA9685_RECOVERY_WRITE_MODE2:
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_SLEEP);
            break;
        case PCA9685_RECOVERY_WRITE_SLEEP:
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_PRESCALE);
            break;
        case PCA9685_RECOVERY_WRITE_PRESCALE:
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_WAKE);
            break;
        case PCA9685_RECOVERY_WRITE_WAKE:
            pca9685_oscillator_wait_start_tick = now;
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WAIT_OSCILLATOR);
            break;
        case PCA9685_RECOVERY_WRITE_RESTART:
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_ALL_OFF);
            break;
        case PCA9685_RECOVERY_WRITE_ALL_OFF:
            pca9685_ready_debug = 1U;
            pca9685_consecutive_error_count_debug = 0U;
            pca9685_recovery_next_retry_tick_debug = 0U;
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_READY);
            if (pca9685_recovery_success_count_debug < UINT32_MAX)
            {
                pca9685_recovery_success_count_debug++;
            }
            return 1U;
        default:
            PCA9685_ScheduleRecovery(1U);
            return 0U;
        }
    }

    if (pca9685_recovery_state == PCA9685_RECOVERY_READY)
    {
        return 0U;
    }

    /* 上层尚未领取正常帧结果时，不复位或复用同一个 I2C2 句柄。 */
    if (pca9685_async_owner != PCA9685_ASYNC_OWNER_NONE)
    {
        return 0U;
    }

    switch (pca9685_recovery_state)
    {
    case PCA9685_RECOVERY_WAIT_RETRY:
        if (PCA9685_TickReached(
                now,
                pca9685_recovery_next_retry_tick_debug) == 0U)
        {
            return 0U;
        }
        if (pca9685_recovery_attempt_count_debug < UINT32_MAX)
        {
            pca9685_recovery_attempt_count_debug++;
        }
        PCA9685_SetRecoveryState(PCA9685_RECOVERY_RESET_I2C);
        return 0U;

    case PCA9685_RECOVERY_RESET_I2C:
        status = HAL_I2C_DeInit(pca9685_i2c_handle);
        if (status == HAL_OK)
        {
            status = HAL_I2C_Init(pca9685_i2c_handle);
        }
        if (status == HAL_OK)
        {
            status = HAL_I2CEx_ConfigAnalogFilter(
                pca9685_i2c_handle,
                I2C_ANALOGFILTER_ENABLE);
        }
        if (status == HAL_OK)
        {
            status = HAL_I2CEx_ConfigDigitalFilter(pca9685_i2c_handle, 0U);
        }
        pca9685_last_hal_status_debug = (uint32_t)status;
        if (status == HAL_OK)
        {
            PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_MODE1_RESET);
            return 0U;
        }
        PCA9685_ScheduleRecovery(1U);
        return 0U;

    case PCA9685_RECOVERY_WRITE_MODE1_RESET:
    {
        static const uint8_t mode1_reset = 0x00U;
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_MODE1,
                                            &mode1_reset,
                                            1U);
        break;
    }

    case PCA9685_RECOVERY_WRITE_MODE2:
    {
        static const uint8_t mode2 = 0x04U;
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_MODE2,
                                            &mode2,
                                            1U);
        break;
    }

    case PCA9685_RECOVERY_WRITE_SLEEP:
    {
        static const uint8_t sleep_mode = 0x10U;
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_MODE1,
                                            &sleep_mode,
                                            1U);
        break;
    }

    case PCA9685_RECOVERY_WRITE_PRESCALE:
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_PRESCALE,
                                            &pca9685_prescale,
                                            1U);
        break;

    case PCA9685_RECOVERY_WRITE_WAKE:
    {
        static const uint8_t wake_mode = 0x00U;
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_MODE1,
                                            &wake_mode,
                                            1U);
        break;
    }

    case PCA9685_RECOVERY_WAIT_OSCILLATOR:
        if (PCA9685_TickReached(
                now,
                pca9685_oscillator_wait_start_tick +
                    PCA9685_OSCILLATOR_STARTUP_MS) == 0U)
        {
            return 0U;
        }
        PCA9685_SetRecoveryState(PCA9685_RECOVERY_WRITE_RESTART);
        return 0U;

    case PCA9685_RECOVERY_WRITE_RESTART:
    {
        static const uint8_t restart_mode = 0xA1U;
        submit_status = PCA9685_SubmitWrite(PCA9685_ASYNC_OWNER_RECOVERY,
                                            PCA9685_MODE1,
                                            &restart_mode,
                                            1U);
        break;
    }

    case PCA9685_RECOVERY_WRITE_ALL_OFF:
        submit_status = PCA9685_SubmitWrite(
            PCA9685_ASYNC_OWNER_RECOVERY,
            PCA9685_ALL_LED_ON_L,
            all_channels_off,
            (uint16_t)sizeof(all_channels_off));
        break;

    default:
        PCA9685_ScheduleRecovery(1U);
        return 0U;
    }

    if ((submit_status == PCA9685_SUBMIT_ACCEPTED) ||
        (submit_status == PCA9685_SUBMIT_BUSY))
    {
        return 0U;
    }

    /* 提交本身失败表示 I2C2 未进入发送状态，本轮恢复作废并延后重试。 */
    PCA9685_ScheduleRecovery(1U);
    return 0U;
}

PCA9685_FrameResult_t PCA9685_TakeFrameResult(uint32_t *duration_ms)
{
    HAL_StatusTypeDef status = HAL_OK;
    PCA9685_AsyncCompletion_t completion;

    PCA9685_ServiceAsyncTimeout(HAL_GetTick());
    completion = PCA9685_TakeAsyncCompletion(PCA9685_ASYNC_OWNER_FRAME,
                                             &status,
                                             duration_ms);
    if (completion == PCA9685_ASYNC_COMPLETION_NONE)
    {
        return PCA9685_FRAME_RESULT_NONE;
    }

    PCA9685_RecordStatus(status, 1U);
    if ((completion == PCA9685_ASYNC_COMPLETION_SUCCESS) &&
        (status == HAL_OK))
    {
        return PCA9685_FRAME_RESULT_SUCCESS;
    }
    return PCA9685_FRAME_RESULT_ERROR;
}

PCA9685_SubmitStatus_t PCA9685_SubmitDuties(uint8_t first_channel,
                                            const float *duties,
                                            uint8_t count)
{
    uint8_t data[PCA9685_ASYNC_BUFFER_SIZE];

    if ((duties == NULL) ||
        (count == 0U) ||
        (first_channel >= PCA9685_CHANNEL_COUNT) ||
        (count > (uint8_t)(PCA9685_CHANNEL_COUNT - first_channel)))
    {
        return PCA9685_SUBMIT_ERROR;
    }
    if ((pca9685_ready_debug == 0U) ||
        (pca9685_recovery_state != PCA9685_RECOVERY_READY))
    {
        return PCA9685_SUBMIT_NOT_READY;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        float duty = duties[i];
        uint16_t off;
        uint8_t data_index = (uint8_t)(i * 4U);

        if (!isfinite(duty))
        {
            return PCA9685_SUBMIT_ERROR;
        }
        if (duty < 0.0f)
        {
            duty = 0.0f;
        }
        else if (duty > 1.0f)
        {
            duty = 1.0f;
        }

        off = (uint16_t)(duty * 4096.0f + 0.5f);
        if (off > 4095U)
        {
            off = 4095U;
        }

        data[data_index] = 0U;
        data[data_index + 1U] = 0U;
        data[data_index + 2U] = (uint8_t)(off & 0xFFU);
        data[data_index + 3U] = (uint8_t)(off >> 8);
    }

    return PCA9685_SubmitWrite(
        PCA9685_ASYNC_OWNER_FRAME,
        (uint8_t)(PCA9685_LED0_ON_L + 4U * first_channel),
        data,
        (uint16_t)(count * 4U));
}

uint8_t PCA9685_SetDuties(uint8_t first_channel,
                          const float *duties,
                          uint8_t count)
{
    return (PCA9685_SubmitDuties(first_channel, duties, count) ==
            PCA9685_SUBMIT_ACCEPTED)
               ? 1U
               : 0U;
}

void PCA9685_SetDuty(uint8_t channel, float duty)
{
    (void)PCA9685_SetDuties(channel, &duty, 1U);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance != I2C2))
    {
        return;
    }

    if (pca9685_async_event_callback_count_debug < UINT32_MAX)
    {
        pca9685_async_event_callback_count_debug++;
    }
    PCA9685_CompleteAsyncFromIsr(PCA9685_ASYNC_COMPLETION_SUCCESS,
                                 HAL_OK,
                                 HAL_I2C_ERROR_NONE);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance != I2C2))
    {
        return;
    }

    if (pca9685_async_error_callback_count_debug < UINT32_MAX)
    {
        pca9685_async_error_callback_count_debug++;
    }
    PCA9685_CompleteAsyncFromIsr(PCA9685_ASYNC_COMPLETION_ERROR,
                                 HAL_ERROR,
                                 hi2c->ErrorCode);
}
