#include "user_key.h"

#include "arm.h"
#include "main.h"
#include "remote_control.h"

/* USER_KEY 硬件连接：PA15，按下接地，未按下由内部上拉保持高电平。 */
#define USER_KEY_GPIO_PORT GPIOA
#define USER_KEY_PIN GPIO_PIN_15
#define USER_KEY_PRESSED_LEVEL GPIO_PIN_RESET
#define USER_KEY_RELEASED_LEVEL GPIO_PIN_SET
#define USER_KEY_DEBOUNCE_MS 20U

/* 消抖状态：stable_state 是已确认电平，last_sample 是最近一次采样电平。 */
static GPIO_PinState user_key_stable_state = USER_KEY_RELEASED_LEVEL;
static GPIO_PinState user_key_last_sample = USER_KEY_RELEASED_LEVEL;
static uint32_t user_key_last_change_tick = 0U;
static uint8_t user_key_initialized = 0U;

/* 上电时如果按键已经按住，必须先松开一次，避免误触发手臂电机使能。 */
static uint8_t user_key_released_seen = 0U;

static void USER_KEY_HandlePressedEvent(void)
{
    (void)Arm_EnableAllMotors();
    (void)Head_Motor_ToggleByUserKey();
}

/**
 * @brief 初始化 USER_KEY GPIO 和消抖初始状态。
 */
void USER_KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_PinState initial_state;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = USER_KEY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(USER_KEY_GPIO_PORT, &GPIO_InitStruct);

    /* 用当前实际电平初始化状态机，避免初始化瞬间产生一次假按下事件。 */
    initial_state = HAL_GPIO_ReadPin(USER_KEY_GPIO_PORT, USER_KEY_PIN);
    user_key_stable_state = initial_state;
    user_key_last_sample = initial_state;
    user_key_last_change_tick = HAL_GetTick();
    user_key_released_seen = (initial_state == USER_KEY_RELEASED_LEVEL) ? 1U : 0U;
    user_key_initialized = 1U;
}

/**
 * @brief 轮询并消抖 USER_KEY，检测到一次稳定按下沿后使能所有手臂电机。
 */
void USER_KEY_Update(void)
{
    GPIO_PinState sample;
    uint32_t now_tick;

    if (user_key_initialized == 0U)
    {
        USER_KEY_Init();
        return;
    }

    sample = HAL_GPIO_ReadPin(USER_KEY_GPIO_PORT, USER_KEY_PIN);
    now_tick = HAL_GetTick();

    /* 采样电平变化后重新计时，等待电平稳定超过消抖时间。 */
    if (sample != user_key_last_sample)
    {
        user_key_last_sample = sample;
        user_key_last_change_tick = now_tick;
        return;
    }

    /* 采样已稳定，但和已确认状态相同，没有新的按下/松开事件。 */
    if (sample == user_key_stable_state)
    {
        return;
    }

    if ((now_tick - user_key_last_change_tick) < USER_KEY_DEBOUNCE_MS)
    {
        return;
    }

    /* 电平稳定超过消抖时间，确认状态切换。 */
    user_key_stable_state = sample;

    /* 松开后重新允许下一次按下触发；长按期间不会重复触发。 */
    if (user_key_stable_state == USER_KEY_RELEASED_LEVEL)
    {
        user_key_released_seen = 1U;
        return;
    }

    /* 只响应“已松开 -> 稳定按下”的边沿，每次有效按下只发送一次使能命令。 */
    if ((user_key_stable_state == USER_KEY_PRESSED_LEVEL) &&
        (user_key_released_seen != 0U))
    {
        user_key_released_seen = 0U;
        USER_KEY_HandlePressedEvent();
    }
}
