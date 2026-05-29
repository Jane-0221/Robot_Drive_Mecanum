#include "head.h"
#include "cmsis_os2.h"
#include "ktech_motor.h"

// ==================== 头部电机控制参数 ====================
#define HEAD_SINGLE_TURN_UNITS 36000U            // 单圈角度总量，单位0.01°，36000对应360°
#define HEAD_HALF_TURN_UNITS 18000               // 半圈角度总量，用于判断最短旋转方向
#define HEAD_SPEED_MIN 1U                        // 动态速度下限，单位1dps/LSB
#define HEAD_SPEED_MAX 150U                      // 动态速度上限，单位1dps/LSB
#define HEAD_SPEED_FULL_ERROR_UNITS 6000U        // 角度误差达到60°时使用最大速度
#define HEAD_MOTOR1_LIMIT_LOW 34300U             // 1号电机跨零限位下边界，343°
#define HEAD_MOTOR1_LIMIT_HIGH 2000U             // 1号电机跨零限位上边界，20°
#define HEAD_MOTOR2_LIMIT_LOW 27000U             // 2号电机跨零限位下边界，270°
#define HEAD_MOTOR2_LIMIT_HIGH 9000U             // 2号电机跨零限位上边界，90°
#define HEAD_FEEDBACK_READY_COUNT 3U             // 每个电机至少收到该次数反馈后认为反馈稳定
#define HEAD_MOTOR_COUNT 2U                      // 头部电机数量
#define HEAD_MOTOR1_INCREASE_DIR DIR_CW          // 1号电机角度增大时的旋转方向
#define HEAD_MOTOR1_DECREASE_DIR DIR_CCW         // 1号电机角度减小时的旋转方向
#define HEAD_MOTOR2_INCREASE_DIR DIR_CW          // 2号电机角度增大时的旋转方向
#define HEAD_MOTOR2_DECREASE_DIR DIR_CCW         // 2号电机角度减小时的旋转方向

// ==================== 头部电机状态缓存 ====================
KTech_Motor_t motor_linkong[2];         // 凌空电机反馈解析结构体数组，索引0/1对应1/2号电机
Head_MotorData_t head_motor_data[2];    // 头部电机控制数据数组，保存当前状态和目标指令

static volatile uint8_t head_motor_enabled = 1U;              // 头部电机控制发送使能标志：0=禁止发送，1=允许发送
static volatile uint8_t head_motor_tx_enabled[HEAD_MOTOR_COUNT] = {1U, 1U};
static volatile uint32_t head_feedback_count[2] = {0U};       // 每个电机收到有效反馈帧的累计次数

/**
 * @brief  清零头部电机反馈计数。
 * @note   电机重新使能或初始化后调用，用于重新等待反馈稳定。
 */
static void Head_ResetFeedbackReady(void)
{
    head_feedback_count[0] = 0U;
    head_feedback_count[1] = 0U;
}

static void Head_ResetMotorFeedbackReady(uint8_t motor_index)
{
    if (motor_index < HEAD_MOTOR_COUNT)
    {
        head_feedback_count[motor_index] = 0U;
    }
}

/**
 * @brief  判断两个头部电机反馈是否都已稳定。
 * @retval 1 两个电机反馈计数均达到阈值；0 至少一路未达到阈值
 */
uint8_t Head_FeedbackReady(void)
{
    return ((head_feedback_count[0] >= HEAD_FEEDBACK_READY_COUNT) &&
            (head_feedback_count[1] >= HEAD_FEEDBACK_READY_COUNT)) ? 1U : 0U;
}

/**
 * @brief  判断指定头部电机反馈是否已稳定。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 1 指定电机反馈计数达到阈值；0 索引无效或反馈未稳定
 */
uint8_t Head_MotorFeedbackReady(uint8_t motor_index)
{
    if (motor_index >= HEAD_MOTOR_COUNT)
    {
        return 0U;
    }

    return (head_feedback_count[motor_index] >= HEAD_FEEDBACK_READY_COUNT) ? 1U : 0U;
}

/**
 * @brief  获取指定头部电机的反馈计数。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 指定电机反馈计数；索引无效时返回0
 */
uint32_t Head_GetFeedbackCount(uint8_t motor_index)
{
    if (motor_index >= HEAD_MOTOR_COUNT)
    {
        return 0U;
    }

    return head_feedback_count[motor_index];
}

/**
 * @brief  通知头部模块已收到一路电机反馈。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 * @note   通常在CAN接收解析到有效KTech状态帧后调用。
 */
void Head_NotifyFeedback(uint8_t motor_index)
{
    if (motor_index < HEAD_MOTOR_COUNT)
    {
        if (head_feedback_count[motor_index] != UINT32_MAX)
        {
            head_feedback_count[motor_index]++;
        }
    }
}

/**
 * @brief  毫秒级延时封装。
 * @param  ms 延时时间，单位ms
 * @note   调度器运行后使用osDelay；调度器启动前使用HAL_Delay。
 */
static void Head_DelayMs(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        osDelay(ms);
    }
    else
    {
        HAL_Delay(ms);
    }
}

static uint8_t Head_GetMotorIdByIndex(uint8_t motor_index, uint16_t *motor_id)
{
    if (motor_id == NULL)
    {
        return 0U;
    }

    if (motor_index == 0U)
    {
        *motor_id = MOTOR_LINKONG_1_ID;
        return 1U;
    }

    if (motor_index == 1U)
    {
        *motor_id = MOTOR_LINKONG_2_ID;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  将角度归一化到单圈范围。
 * @param  angle 输入角度，单位0.01°
 * @retval 0~35999范围内的单圈角度
 */
static uint32_t Head_NormalizeAngle(uint32_t angle)
{
    return angle % HEAD_SINGLE_TURN_UNITS;
}

/**
 * @brief  将跨零单圈角度转换成以零点为中心的有符号角度。
 * @note   例如34300会转换为-1700，避免把负向小角度当成343°单圈长路径发送。
 */
static int32_t Head_AngleToSignedUnits(uint32_t angle)
{
    angle = Head_NormalizeAngle(angle);

    if (angle > HEAD_HALF_TURN_UNITS)
    {
        return (int32_t)angle - (int32_t)HEAD_SINGLE_TURN_UNITS;
    }

    return (int32_t)angle;
}

static uint8_t Head_GetMotorLimits(uint8_t motor_index, uint32_t *low_limit, uint32_t *high_limit)
{
    if (motor_index == 0U)
    {
        *low_limit = HEAD_MOTOR1_LIMIT_LOW;
        *high_limit = HEAD_MOTOR1_LIMIT_HIGH;
        return 1U;
    }

    if (motor_index == 1U)
    {
        *low_limit = HEAD_MOTOR2_LIMIT_LOW;
        *high_limit = HEAD_MOTOR2_LIMIT_HIGH;
        return 1U;
    }

    return 0U;
}

static uint8_t Head_IsAngleInAcrossZeroRange(uint32_t angle, uint32_t low_limit, uint32_t high_limit)
{
    angle = Head_NormalizeAngle(angle);

    return ((angle >= low_limit) || (angle <= high_limit)) ? 1U : 0U;
}

/**
 * @brief  对跨零点的安全角度区间进行限幅。
 * @param  angle      待限幅目标角度，单位0.01°
 * @param  low_limit  跨零区间低侧边界，例如34100表示341°
 * @param  high_limit 跨零区间高侧边界，例如2000表示20°
 * @retval 位于允许区间内的目标角度；超出区间时吸附到最近边界
 */
static uint32_t Head_ClampAcrossZero(uint32_t angle, uint32_t low_limit, uint32_t high_limit)
{
    angle = Head_NormalizeAngle(angle);

    if (Head_IsAngleInAcrossZeroRange(angle, low_limit, high_limit) != 0U)
    {
        return angle;
    }

    return ((angle - high_limit) <= (low_limit - angle)) ? high_limit : low_limit;
}

/**
 * @brief  将反馈角度从度转换为KTech单圈位置单位。
 * @param  angle_deg 当前角度，单位°
 * @retval 角度单位0.01°，范围0~35999
 */
static uint32_t Head_CurrentAngleToUnits(float angle_deg)
{
    if (angle_deg <= 0.0f)
    {
        return 0U;
    }

    return ((uint32_t)(angle_deg * 100.0f + 0.5f)) % HEAD_SINGLE_TURN_UNITS;
}

static uint32_t Head_GetSafeSendTargetAngle(uint8_t motor_index, uint32_t target_angle)
{
    uint32_t low_limit;
    uint32_t high_limit;
    uint32_t current_angle;
    uint32_t limited_target;

    if (Head_GetMotorLimits(motor_index, &low_limit, &high_limit) == 0U)
    {
        return Head_NormalizeAngle(target_angle);
    }

    limited_target = Head_ClampAcrossZero(target_angle, low_limit, high_limit);
    current_angle = Head_CurrentAngleToUnits(head_motor_data[motor_index].current_angle);

    if (Head_IsAngleInAcrossZeroRange(current_angle, low_limit, high_limit) == 0U)
    {
        return Head_ClampAcrossZero(current_angle, low_limit, high_limit);
    }

    return limited_target;
}

/**
 * @brief  计算从当前角度到目标角度的单圈最短差值。
 * @param  current_angle 当前角度，单位0.01°
 * @param  target_angle  目标角度，单位0.01°
 * @retval 带符号差值；正值表示按角度增大方向更近，负值表示按角度减小方向更近
 */
static int32_t Head_GetShortestDelta(uint32_t current_angle, uint32_t target_angle)
{
    int32_t delta = (int32_t)target_angle - (int32_t)current_angle;

    if (delta > HEAD_HALF_TURN_UNITS)
    {
        delta -= (int32_t)HEAD_SINGLE_TURN_UNITS;
    }
    else if (delta < -HEAD_HALF_TURN_UNITS)
    {
        delta += (int32_t)HEAD_SINGLE_TURN_UNITS;
    }

    return delta;
}

static int32_t Head_GetLimitedShortestDelta(uint8_t motor_index, uint32_t current_angle, uint32_t target_angle)
{
    uint32_t low_limit;
    uint32_t high_limit;
    int32_t delta;

    current_angle = Head_NormalizeAngle(current_angle);
    target_angle = Head_NormalizeAngle(target_angle);
    delta = Head_GetShortestDelta(current_angle, target_angle);

    if ((Head_GetMotorLimits(motor_index, &low_limit, &high_limit) != 0U) &&
        (Head_IsAngleInAcrossZeroRange(current_angle, low_limit, high_limit) != 0U) &&
        (Head_IsAngleInAcrossZeroRange(target_angle, low_limit, high_limit) != 0U))
    {
        if ((delta == -HEAD_HALF_TURN_UNITS) &&
            (current_angle >= low_limit) &&
            (target_angle <= high_limit))
        {
            delta = HEAD_HALF_TURN_UNITS;
        }
        else if ((delta == HEAD_HALF_TURN_UNITS) &&
                 (current_angle <= high_limit) &&
                 (target_angle >= low_limit))
        {
            delta = -HEAD_HALF_TURN_UNITS;
        }
    }

    return delta;
}

/**
 * @brief  根据目标误差计算动态速度限制。
 * @param  motor_index  电机索引
 * @param  target_angle 目标角度，单位0.01°
 * @retval 速度限制，单位1dps/LSB；误差越大速度越高
 */
static uint16_t Head_CalcDynamicSpeed(uint8_t motor_index, uint32_t target_angle)
{
    uint32_t current_angle = Head_CurrentAngleToUnits(head_motor_data[motor_index].current_angle);
    int32_t delta = Head_GetLimitedShortestDelta(motor_index, current_angle, target_angle);
    uint32_t error_units = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;

    if (error_units >= HEAD_SPEED_FULL_ERROR_UNITS)
    {
        return (uint16_t)HEAD_SPEED_MAX;
    }

    return (uint16_t)(HEAD_SPEED_MIN +
                     ((error_units * (HEAD_SPEED_MAX - HEAD_SPEED_MIN)) /
                      HEAD_SPEED_FULL_ERROR_UNITS));
}

/**
 * @brief  根据角度差值选择电机旋转方向。
 * @param  motor_index 电机索引
 * @param  delta       最短角度差值，单位0.01°
 * @retval KTech单圈位置控制方向，DIR_CW或DIR_CCW
 */
static uint8_t Head_GetDirectionForDelta(uint8_t motor_index, int32_t delta)
{
    if (delta > 0)
    {
        return (motor_index == 0U) ? HEAD_MOTOR1_INCREASE_DIR : HEAD_MOTOR2_INCREASE_DIR;
    }

    if (delta < 0)
    {
        return (motor_index == 0U) ? HEAD_MOTOR1_DECREASE_DIR : HEAD_MOTOR2_DECREASE_DIR;
    }

    return head_motor_data[motor_index].direction;
}

/**
 * @brief  按最短路径更新指定电机的方向字段。
 * @param  motor_index  电机索引
 * @param  target_angle 已限幅的目标角度，单位0.01°
 */
static void Head_UpdateShortestDirection(uint8_t motor_index, uint32_t target_angle)
{
    uint32_t current_angle = Head_CurrentAngleToUnits(head_motor_data[motor_index].current_angle);
    int32_t delta = Head_GetLimitedShortestDelta(motor_index, current_angle, target_angle);

    head_motor_data[motor_index].direction = Head_GetDirectionForDelta(motor_index, delta);
}

/**
 * @brief  初始化头部两路KTech电机并设置默认控制参数。
 */
void Head_Init()
{
    Head_ResetFeedbackReady();

    ktech_motor_init(MOTOR_LINKONG_1_ID);
    // 将电机1从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);

    // 电机1数据初始化
    head_motor_data[0].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[0].target_angle = 0;
    head_motor_data[0].max_speed = 10;


    ktech_motor_init(MOTOR_LINKONG_2_ID);
    // 将电机2从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    
    // 电机2数据初始化
    head_motor_data[1].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[1].target_angle = 0;
    head_motor_data[1].max_speed = 10;
};

/**
 * @brief  发送1号头部电机位置控制指令。
 * @note   发送前会进行目标角度限幅、最短方向计算和动态速度计算。
 */
void Head_Lk_motor1(void)
{
    uint32_t current_target = Head_GetSafeSendTargetAngle(0U, head_motor_data[0].target_angle);
    int32_t signed_target;
    uint16_t speed_limit;

    head_motor_data[0].target_angle = current_target;
    Head_UpdateShortestDirection(0U, current_target);
    speed_limit = Head_CalcDynamicSpeed(0U, current_target);
    signed_target = Head_AngleToSignedUnits(current_target);

    ktech_pos_multi2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_1_ID, 
                      signed_target, 
                      speed_limit);
}

/**
 * @brief  发送2号头部电机位置控制指令。
 * @note   发送前会进行目标角度限幅、最短方向计算和动态速度计算。
 */
void Head_Lk_motor2()
{
    uint32_t current_target = Head_GetSafeSendTargetAngle(1U, head_motor_data[1].target_angle);
    int32_t signed_target;
    uint16_t speed_limit;

    head_motor_data[1].target_angle = current_target;
    Head_UpdateShortestDirection(1U, current_target);
    speed_limit = Head_CalcDynamicSpeed(1U, current_target);
    signed_target = Head_AngleToSignedUnits(current_target);

    ktech_pos_multi2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_2_ID, 
                      signed_target, 
                      speed_limit);
}

/**
 * @brief  按索引发送单个头部电机控制指令。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 * @note   head_motor_enabled为0时不发送任何控制帧。
 */
void Head_TxMotorByIndex(uint8_t motor_index)
{
    if ((head_motor_enabled == 0U) ||
        (motor_index >= HEAD_MOTOR_COUNT) ||
        (head_motor_tx_enabled[motor_index] == 0U))
    {
        return;
    }

    if (motor_index == 0U)
    {
        Head_Lk_motor1();
    }
    else if (motor_index == 1U)
    {
        Head_Lk_motor2();
    }
}

/**
 * @brief  依次发送两路头部电机控制指令。
 * @note   两帧之间加入1ms间隔，降低CAN连续发送压力。
 */
void Head_all_tx()
{
    if (head_motor_enabled == 0U)
    {
        return;
    }

    Head_TxMotorByIndex(0U);
    Head_DelayMs(1U);
    Head_TxMotorByIndex(1U);
}

/**
 * @brief  请求两路头部电机状态2反馈。
 * @note   状态2包含温度、转矩电流/功率、速度和编码器值。
 */
void Head_RequestFeedback(void)
{
    ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    Head_DelayMs(1U);
    ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
}

/**
 * @brief  请求指定头部电机状态2反馈。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 */
void Head_RequestFeedbackByIndex(uint8_t motor_index)
{
    if (motor_index == 0U)
    {
        ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    }
    else if (motor_index == 1U)
    {
        ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    }
}

/**
 * @brief  向两路头部电机发送运行使能命令。
 * @note   仅发送电机on命令并置位发送使能，不清除反馈计数。
 */
void Head_Motor_SendEnableCommand(void)
{
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    head_motor_tx_enabled[0] = 1U;
    Head_DelayMs(1U);
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    head_motor_tx_enabled[1] = 1U;
    head_motor_enabled = 1U;
}

/**
 * @brief  使能两路头部电机。
 * @note   清除旧反馈计数后重新发送运行使能命令。
 */
void Head_Motor_Enable(void)
{
    Head_ResetFeedbackReady();
    Head_Motor_SendEnableCommand();
}

/**
 * @brief  失能两路头部电机。
 * @note   先禁止后续控制帧发送，再依次发送电机off命令。
 */
void Head_Motor_Disable(void)
{
    head_motor_enabled = 0U;
    head_motor_tx_enabled[0] = 0U;
    head_motor_tx_enabled[1] = 0U;
    ktech_motor_off(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    Head_DelayMs(1U);
    ktech_motor_off(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
}

uint8_t Head_EnableMotorByIndex(uint8_t motor_index)
{
    uint16_t motor_id;

    if (Head_GetMotorIdByIndex(motor_index, &motor_id) == 0U)
    {
        return 0U;
    }

    Head_ResetMotorFeedbackReady(motor_index);
    ktech_motor_on(CAN_HANDLE_1, motor_id);
    head_motor_tx_enabled[motor_index] = 1U;
    head_motor_enabled = 1U;

    return 1U;
}

uint8_t Head_DisableMotorByIndex(uint8_t motor_index)
{
    uint16_t motor_id;

    if (Head_GetMotorIdByIndex(motor_index, &motor_id) == 0U)
    {
        return 0U;
    }

    head_motor_tx_enabled[motor_index] = 0U;
    ktech_motor_off(CAN_HANDLE_1, motor_id);

    return 1U;
}

/**
 * @brief  保存指定电机当前位置为零点，并在新零点处保持。
 * @param  motor_index 电机索引，0对应1号电机，1对应2号电机
 * @param  motor_id    KTech电机CAN ID
 */
static void Head_SaveZeroAndHold(uint8_t motor_index, uint16_t motor_id)
{
    uint32_t current_target;
    int32_t signed_target;
    uint16_t speed_limit;

    ktech_motor_stop(CAN_HANDLE_1, motor_id);
    Head_DelayMs(1U);

    ktech_set_zero(CAN_HANDLE_1, motor_id);
    head_motor_data[motor_index].target_angle = 0U;
    head_motor_data[motor_index].current_angle = 0.0f;
    Head_DelayMs(1U);

    ktech_motor_on(CAN_HANDLE_1, motor_id);
    Head_DelayMs(1U);

    current_target = Head_GetSafeSendTargetAngle(motor_index, 0U);
    head_motor_data[motor_index].target_angle = current_target;
    Head_UpdateShortestDirection(motor_index, current_target);
    speed_limit = Head_CalcDynamicSpeed(motor_index, current_target);
    signed_target = Head_AngleToSignedUnits(current_target);
    ktech_pos_multi2(CAN_HANDLE_1,
                      motor_id,
                      signed_target,
                      speed_limit);
}

/**
 * @brief  保存两路头部电机当前位置为零点。
 * @note   每路保存零点后重新进入位置保持，最后恢复发送使能标志。
 */
void Head_save_position(void)
{
    Head_SaveZeroAndHold(0U, MOTOR_LINKONG_1_ID);
    head_motor_tx_enabled[0] = 1U;
    Head_DelayMs(1U);
    Head_SaveZeroAndHold(1U, MOTOR_LINKONG_2_ID);
    head_motor_tx_enabled[1] = 1U;
    head_motor_enabled = 1U;
}

uint8_t Head_SaveMotorZeroByIndex(uint8_t motor_index)
{
    uint16_t motor_id;

    if (Head_GetMotorIdByIndex(motor_index, &motor_id) == 0U)
    {
        return 0U;
    }

    Head_SaveZeroAndHold(motor_index, motor_id);
    head_motor_tx_enabled[motor_index] = 1U;
    head_motor_enabled = 1U;

    return 1U;
}

/**
 * @brief  从KTech反馈缓存刷新头部电机状态数据。
 * @note   编码器值按65536/圈换算为0~360°角度；速度直接使用KTech反馈的dps单位。
 */
void Head_Lk_Data_update()
{
    // 将电机1编码器值转换为角度值 (假设一圈编码器分辨率为65536)
    head_motor_data[0].current_angle = motor_linkong[0].fb.encoder / 65536.0f * 360.0f;     
    head_motor_data[0].current_velocity = motor_linkong[0].fb.speed;                 

    // 将电机2编码器值转换为角度值 
    head_motor_data[1].current_angle = motor_linkong[1].fb.encoder / 65536.0f * 360.0f;     
    head_motor_data[1].current_velocity = motor_linkong[1].fb.speed;                 
}
