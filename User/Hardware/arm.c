#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"
#include "LZ_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stdio.h"

/**
 * @brief 达妙电机状态结构体数组（6路）
 * @note  存储DM4310电机的位置、速度、电流等状态
 */
extern Motor_DM_Status DM_Status[6];

/**
 * @brief 灵足电机对象（3路）
 * @note  Robstride04协议驱动，对应灵足1/2/3号电机
 */
RobStride_Motor_t motor1; // 灵足1号电机
RobStride_Motor_t motor2; // 灵足2号电机
RobStride_Motor_t motor3; // 灵足3号电机

/**
 * @brief 灵足电机数据结构体数组（3路）
 * @note  存储灵足电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Linzu_motor_data[3];

/**
 * @brief 达妙电机数据结构体数组（3路）
 * @note  存储达妙电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Damiao_motor_data[3];

/**
 * @brief 数据回复使能标志
 * @note  浮点型：0.0f=禁用回复，非0.0f=启用回复（未在本代码中实际使用）
 */
float reply_enable = 0.0f;

#define ARM_REENABLE_CHECK_PERIOD_MS 100U
#define ARM_DAMIAO_ENABLE_STATE 2U
#define ARM_PI_RAD 3.141592654f
#define ARM_TWO_PI_RAD 6.283185307f
#define ARM_FIRST_MOTOR_TARGET_ANGLE_LIMIT_RAD ARM_PI_RAD
#define ARM_TARGET_ANGLE_LIMIT_RAD 2.617993878f
#define ARM_TARGET_CURRENT_MAX_DIFF_RAD ARM_PI_RAD
#define ARM_DISABLED_FEEDBACK_PERIOD_MS 50U
#define ARM_LINZU_ENABLE_SETTLE_MS 1U
#define ARM_LINZU_LIMIT_SPEED_MIN_RAD_S 1.0f
#define ARM_LINZU_LIMIT_SPEED_MAX_RAD_S 20.0f
#define ARM_TX_SLOT_COUNT 10U

volatile uint8_t arm_motor_disabled_mask_debug = 0U;
volatile uint32_t arm_feedback_count_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile uint32_t arm_feedback_last_tick_debug[ARM_LOGICAL_MOTOR_COUNT] = {0U};
volatile uint32_t arm_linzu_tx_attempt_debug[3] = {0U};
volatile uint32_t arm_linzu_tx_reject_debug[3] = {0U};
volatile uint32_t arm_linzu_tx_sent_debug[3] = {0U};
volatile uint32_t arm_linzu_tx_last_tick_debug[3] = {0U};
volatile float arm_linzu_tx_requested_velocity_debug[3] = {0.0f};
volatile float arm_linzu_tx_last_velocity_debug[3] = {0.0f};
volatile uint32_t arm_damiao_tx_attempt_debug[3] = {0U};
volatile uint32_t arm_damiao_tx_reject_debug[3] = {0U};
volatile uint32_t arm_damiao_tx_sent_debug[3] = {0U};
volatile uint32_t arm_damiao_tx_last_tick_debug[3] = {0U};
volatile float arm_damiao_tx_last_velocity_debug[3] = {0.0f};

static ArmMotorData_t *Arm_GetMotorDataByIndex(uint8_t logical_motor);

static uint8_t Arm_GetMotorMask(uint8_t logical_motor)
{
    if (logical_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        return 0U;
    }

    return (uint8_t)(1U << logical_motor);
}

static uint8_t Arm_MotorTxDisabledByIndex(uint8_t logical_motor)
{
    uint8_t motor_mask = Arm_GetMotorMask(logical_motor);

    if (motor_mask == 0U)
    {
        return 1U;
    }

    return ((arm_motor_disabled_mask_debug & motor_mask) != 0U) ? 1U : 0U;
}

static void Arm_SetMotorTxDisabledByIndex(uint8_t logical_motor, uint8_t disabled)
{
    uint8_t motor_mask = Arm_GetMotorMask(logical_motor);

    if (motor_mask == 0U)
    {
        return;
    }

    if (disabled != 0U)
    {
        arm_motor_disabled_mask_debug |= motor_mask;
    }
    else
    {
        arm_motor_disabled_mask_debug &= (uint8_t)(~motor_mask);
    }
}

float Arm_WrapAngleToPi(float angle)
{
    while (angle > ARM_PI_RAD)
    {
        angle -= ARM_TWO_PI_RAD;
    }

    while (angle < -ARM_PI_RAD)
    {
        angle += ARM_TWO_PI_RAD;
    }

    return angle;
}

static float Arm_GetEquivalentAngleNearCurrent(float target_angle, float current_angle)
{
    return current_angle + Arm_WrapAngleToPi(target_angle - current_angle);
}

static float Arm_GetTargetAngleLimit(uint8_t logical_motor)
{
    if (logical_motor == 0U)
    {
        return ARM_FIRST_MOTOR_TARGET_ANGLE_LIMIT_RAD;
    }

    return ARM_TARGET_ANGLE_LIMIT_RAD;
}

static float Arm_ClampTargetAngleByIndex(uint8_t logical_motor, float angle)
{
    float angle_limit = Arm_GetTargetAngleLimit(logical_motor);

    if (angle > angle_limit)
    {
        return angle_limit;
    }

    if (angle < -angle_limit)
    {
        return -angle_limit;
    }

    return angle;
}

static float Arm_LimitTargetAngle(uint8_t logical_motor, ArmMotorData_t *motor_data)
{
    motor_data->target_angle = Arm_ClampTargetAngleByIndex(logical_motor, motor_data->target_angle);
    return motor_data->target_angle;
}

static uint8_t Arm_IsAngleDiffSafe(float target_angle, float current_angle)
{
    float angle_diff;

    angle_diff = target_angle - current_angle;

    if (angle_diff < 0.0f)
    {
        angle_diff = -angle_diff;
    }

    return (angle_diff <= ARM_TARGET_CURRENT_MAX_DIFF_RAD) ? 1U : 0U;
}

static uint8_t Arm_GetSafeTargetAngle(uint8_t logical_motor, ArmMotorData_t *motor_data, float *target_angle)
{
    *target_angle = Arm_LimitTargetAngle(logical_motor, motor_data);
    return Arm_IsAngleDiffSafe(*target_angle, motor_data->current_angle);
}

static float Arm_GetLinzuLimitSpeed(float requested_speed)
{
    float limit_speed = requested_speed;

    if (limit_speed != limit_speed)
    {
        limit_speed = ARM_LINZU_LIMIT_SPEED_MIN_RAD_S;
    }

    if (limit_speed < 0.0f)
    {
        limit_speed = -limit_speed;
    }

    if (limit_speed < ARM_LINZU_LIMIT_SPEED_MIN_RAD_S)
    {
        limit_speed = ARM_LINZU_LIMIT_SPEED_MIN_RAD_S;
    }

    if (limit_speed > ARM_LINZU_LIMIT_SPEED_MAX_RAD_S)
    {
        limit_speed = ARM_LINZU_LIMIT_SPEED_MAX_RAD_S;
    }

    return limit_speed;
}

void Arm_SetPcTargetAngles(const float target_angles[ARM_LOGICAL_MOTOR_COUNT], const float target_velocities[ARM_LOGICAL_MOTOR_COUNT], uint32_t now_ms)
{
    uint8_t logical_motor;

    (void)now_ms;

    if ((target_angles == NULL) || (target_velocities == NULL))
    {
        return;
    }

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);
        float target_angle = Arm_ClampTargetAngleByIndex(logical_motor, target_angles[logical_motor]);

        if (motor_data != NULL)
        {
            motor_data->target_angle = target_angle;
            motor_data->target_velocity = target_velocities[logical_motor];
        }
    }
}

static uint8_t Arm_SendLinzuTarget(uint8_t logical_motor, RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    float target_angle;
    float motor_angle;
    float limit_speed;

    if (logical_motor < 3U)
    {
        arm_linzu_tx_attempt_debug[logical_motor]++;
        arm_linzu_tx_requested_velocity_debug[logical_motor] = motor_data->target_velocity;
    }

    target_angle = Arm_LimitTargetAngle(logical_motor, motor_data);
    motor_angle = motor->Pos_Info.Angle;
    if (logical_motor != 0U)
    {
        target_angle = Arm_GetEquivalentAngleNearCurrent(target_angle, motor_angle);
    }

    if (Arm_IsAngleDiffSafe(target_angle, motor_angle) == 0U)
    {
        if (logical_motor < 3U)
        {
            arm_linzu_tx_reject_debug[logical_motor]++;
        }
        return 0U;
    }

    limit_speed = Arm_GetLinzuLimitSpeed(motor_data->target_velocity);
    RobStride_Motor_CSP_control(motor, CAN_HANDLE_2, target_angle, limit_speed);

    if (logical_motor < 3U)
    {
        arm_linzu_tx_sent_debug[logical_motor]++;
        arm_linzu_tx_last_tick_debug[logical_motor] = HAL_GetTick();
        arm_linzu_tx_last_velocity_debug[logical_motor] = limit_speed;
    }

    return 1U;
}

static uint8_t Arm_SendDamiaoTarget(uint8_t logical_motor, uint16_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;
    uint8_t damiao_index = logical_motor - 3U;

    if (damiao_index < 3U)
    {
        arm_damiao_tx_attempt_debug[damiao_index]++;
    }

    if (Arm_GetSafeTargetAngle(logical_motor, motor_data, &target_angle) == 0U)
    {
        if (damiao_index < 3U)
        {
            arm_damiao_tx_reject_debug[damiao_index]++;
        }
        return 0U;
    }

    pos_speed_ctrl(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity);

    if (damiao_index < 3U)
    {
        arm_damiao_tx_sent_debug[damiao_index]++;
        arm_damiao_tx_last_tick_debug[damiao_index] = HAL_GetTick();
        arm_damiao_tx_last_velocity_debug[damiao_index] = motor_data->target_velocity;
    }

    return 1U;
}

// ===================== 函数定义 =====================
/**
 * @brief 机械臂初始化函数
 * @retval 无
 * @note   1. 核心功能：初始化灵足/达妙电机，配置CAN2通信；
 *         2. 灵足电机：初始化RobStride协议，设置CSP位置模式，使能电机，开启主动上报；
 *         3. 达妙电机：初始化位置模式，暂未配置具体参数；
 *         4. 所有电机均挂载在CAN2总线上。
 */
void Arm_Init()
{
    /* 灵足电机初始化（使用CAN2总线） */
    // 1号灵足电机初始化
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);                             // 初始化电机对象（ID为灵足1号）
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);                        // 读取电机参数，0X7005为参数地址
    HAL_Delay(10);                                                                       // 延时确保通信稳定
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j'); // 设置CSP位置控制模式
    // Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);                                       // 使能电机
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p'); // 设置参数0X7017，比例1.0
    HAL_Delay(10);
    // 开启主动上报（0x00=关闭，0x01=开启）
    RobStride_Motor_ProactiveEscalationSet(&motor1, CAN_HANDLE_2, 0x01);

    // 2号灵足电机初始化（逻辑2号）
    RobStride_Motor_Init(&motor2, MOTOR_LINGZU_2_ID, false);
    Get_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    // Enable_Motor(&motor2, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor2, CAN_HANDLE_2, 0x01);

    // 3号灵足电机初始化（逻辑3号）
    RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    // Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor3, CAN_HANDLE_2, 0x01);

    /* 达妙电机初始化（使用CAN2总线） */
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE); // 4号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE); // 5号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE); // 6号达妙电机初始化（位置模式）

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE); // 使能4号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE); // 使能5号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE); // 使能6号达妙电机位置模式
    set_DM_mode(Motor4, POS_MODE);
    set_DM_mode(Motor5, POS_MODE);
    set_DM_mode(Motor6, POS_MODE);

    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;
    Damiao_motor_data[0].target_velocity = 1.0f;
    Damiao_motor_data[1].target_velocity = 1.0f;
    Damiao_motor_data[2].target_velocity = 1.0f;
    // 初始化灵足电机目标参数（角度10°，速度1.0r/min）
    // Set_ZeroPos(&motor1, CAN_HANDLE_2);
    // Set_ZeroPos(&motor2, CAN_HANDLE_2);
    // Set_ZeroPos(&motor3, CAN_HANDLE_2);

    Linzu_motor_data[0].target_angle = 0.0f;
    Linzu_motor_data[1].target_angle = 0.0f;
    Linzu_motor_data[2].target_angle = 0.0f;
    Linzu_motor_data[0].target_velocity = 1.0f;
    Linzu_motor_data[1].target_velocity = 1.0f;
    Linzu_motor_data[2].target_velocity = 1.0f;

    // 初始化灵足电机位置为0

    // 初始化电机失能
    // Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
}

/**
 * @brief 控制1号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[0]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor1()
{
    (void)Arm_SendLinzuTarget(0U, &motor1, &Linzu_motor_data[0]);
}

/**
 * @brief 控制2号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[1]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor2()
{
    (void)Arm_SendLinzuTarget(1U, &motor2, &Linzu_motor_data[1]);
}

/**
 * @brief 控制3号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[2]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor3()
{
    (void)Arm_SendLinzuTarget(2U, &motor3, &Linzu_motor_data[2]);
}

/**
 * @brief 控制4号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 设置位置速度控制参数（5=位置参数，10=速度参数）；
 *         4. 固定目标位置为20（可根据需求修改）。
 */
void Arm_Damiao_motor4()
{
    (void)Arm_SendDamiaoTarget(3U, MOTOR_DAMIAO_4_ID, &Damiao_motor_data[0]);
}

/**
 * @brief 控制5号达妙电机（位置模式）
 * @retval 无
 * @note   逻辑5号达妙电机，位置参数10，速度参数1
 */
void Arm_Damiao_motor5()
{

    (void)Arm_SendDamiaoTarget(4U, MOTOR_DAMIAO_5_ID, &Damiao_motor_data[1]);
}

/**
 * @brief 控制6号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 固定目标位置为10，使用电机6的位置设定值作为控制参数；
 *         4. 速度参数固定为1。
 */
void Arm_Damiao_motor6()
{
    (void)Arm_SendDamiaoTarget(5U, MOTOR_DAMIAO_6_ID, &Damiao_motor_data[2]);
}

/**
 * @brief 更新灵足电机当前状态数据
 * @retval 无
 * @note   从motor1/2/3的RobStride协议缓存中，读取当前角度、速度，更新到Linzu_motor_data
 */
void Arm_Linzu_Data_update()
{
    Linzu_motor_data[0].current_angle = motor1.Pos_Info.Angle;
    Linzu_motor_data[1].current_angle = motor2.Pos_Info.Angle;
    Linzu_motor_data[2].current_angle = motor3.Pos_Info.Angle;
    Linzu_motor_data[0].current_velocity = motor1.Pos_Info.Speed;
    Linzu_motor_data[1].current_velocity = motor2.Pos_Info.Speed;
    Linzu_motor_data[2].current_velocity = motor3.Pos_Info.Speed;
}

void Arm_Damiao_Data_update()
{
    Damiao_motor_data[0].current_angle = arm_motor[3].para.pos;
    Damiao_motor_data[1].current_angle = arm_motor[4].para.pos;
    Damiao_motor_data[2].current_angle = arm_motor[5].para.pos;
    Damiao_motor_data[0].current_velocity = arm_motor[3].para.vel;
    Damiao_motor_data[1].current_velocity = arm_motor[4].para.vel;
    Damiao_motor_data[2].current_velocity = arm_motor[5].para.vel;
}

/**
 * @brief 批量更新所有电机状态数据
 * @retval 无
 * @note   1. 先更新灵足电机数据；
 *         2. 更新达妙电机数据。
 */
void Arm_All_Data_update()
{
    Arm_Linzu_Data_update();
    Arm_Damiao_Data_update();
}

/**
 * @brief 机械臂电机控制指令发送函数
 * @retval 无
 * @note   1. 由1ms任务周期驱动，每次只发送一个slot，避免CAN2同一时刻突发多帧；
 *         2. slot 0/2/4发送灵足1/2/3号电机；
 *         3. slot 1/3/5发送达妙4/5/6号电机；
 *         4. slot 6/7/8/9空闲，使每路目标控制保持100Hz。
 */
void Arm_all_tx()
{
    static uint8_t arm_tx_slot = 0U;
    uint8_t slot = arm_tx_slot;

    arm_tx_slot++;
    if (arm_tx_slot >= ARM_TX_SLOT_COUNT)
    {
        arm_tx_slot = 0U;
    }

    switch (slot)
    {
    case 0U:
        if (Arm_MotorTxDisabledByIndex(0U) == 0U)
        {
            Arm_Linzu_motor1();
        }
        break;

    case 1U:
        if (Arm_MotorTxDisabledByIndex(3U) == 0U)
        {
            Arm_Damiao_motor4();
        }
        break;

    case 2U:
        if (Arm_MotorTxDisabledByIndex(1U) == 0U)
        {
            Arm_Linzu_motor2();
        }
        break;

    case 3U:
        if (Arm_MotorTxDisabledByIndex(4U) == 0U)
        {
            Arm_Damiao_motor5();
        }
        break;

    case 4U:
        if (Arm_MotorTxDisabledByIndex(2U) == 0U)
        {
            Arm_Linzu_motor3();
        }
        break;

    case 5U:
        if (Arm_MotorTxDisabledByIndex(5U) == 0U)
        {
            Arm_Damiao_motor6();
        }
        break;

    default:
        break;
    }
}

static ArmMotorData_t *Arm_GetMotorDataByIndex(uint8_t logical_motor)
{
    if (logical_motor < 3U)
    {
        return &Linzu_motor_data[logical_motor];
    }

    if (logical_motor < ARM_LOGICAL_MOTOR_COUNT)
    {
        return &Damiao_motor_data[logical_motor - 3U];
    }

    return NULL;
}

static RobStride_Motor_t *Arm_GetLinzuMotorByIndex(uint8_t logical_motor)
{
    RobStride_Motor_t *linzu_motors[3] = {&motor1, &motor2, &motor3};

    if (logical_motor >= 3U)
    {
        return NULL;
    }

    return linzu_motors[logical_motor];
}

static void Arm_EnableLinzuMotorFeedback(RobStride_Motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    Enable_Motor(motor, CAN_HANDLE_2);
    osDelay(ARM_LINZU_ENABLE_SETTLE_MS);
    RobStride_Motor_ProactiveEscalationSet(motor, CAN_HANDLE_2, 0x01);
    osDelay(ARM_LINZU_ENABLE_SETTLE_MS);
}

static uint8_t Arm_GetDamiaoMotorInfoByIndex(uint8_t logical_motor, uint16_t *motor_id, uint16_t *motor_index)
{
    static const uint16_t damiao_motor_ids[3] = {
        MOTOR_DAMIAO_4_ID,
        MOTOR_DAMIAO_5_ID,
        MOTOR_DAMIAO_6_ID,
    };
    static const uint16_t damiao_motor_indices[3] = {
        Motor4,
        Motor5,
        Motor6,
    };
    uint8_t damiao_index;

    if ((logical_motor < 3U) || (logical_motor >= ARM_LOGICAL_MOTOR_COUNT))
    {
        return 0U;
    }

    damiao_index = logical_motor - 3U;
    *motor_id = damiao_motor_ids[damiao_index];
    *motor_index = damiao_motor_indices[damiao_index];
    return 1U;
}

static void Arm_ReenableLinzuMotor(uint8_t logical_motor, RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    if (motor->Pos_Info.pattern == 2U)
    {
        return;
    }

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Arm_EnableLinzuMotorFeedback(motor);
    (void)Arm_SendLinzuTarget(logical_motor, motor, motor_data);
}

static void Arm_ReenableDamiaoMotor(uint8_t logical_motor, uint16_t motor_id, uint16_t motor_index, ArmMotorData_t *motor_data)
{
    if (arm_motor[motor_index].para.state == ARM_DAMIAO_ENABLE_STATE)
    {
        return;
    }

    enable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
    set_DM_mode(motor_index, POS_MODE);
    (void)Arm_SendDamiaoTarget(logical_motor, motor_id, motor_data);
}

void Arm_CheckAndReenableDisabledMotors(void)
{
    static uint32_t last_check_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_check_tick) < ARM_REENABLE_CHECK_PERIOD_MS)
    {
        return;
    }

    last_check_tick = now_tick;

    if (Arm_MotorTxDisabledByIndex(0U) == 0U)
    {
        Arm_ReenableLinzuMotor(0U, &motor1, &Linzu_motor_data[0]);
    }
    if (Arm_MotorTxDisabledByIndex(1U) == 0U)
    {
        Arm_ReenableLinzuMotor(1U, &motor2, &Linzu_motor_data[1]);
    }
    if (Arm_MotorTxDisabledByIndex(2U) == 0U)
    {
        Arm_ReenableLinzuMotor(2U, &motor3, &Linzu_motor_data[2]);
    }

    if (Arm_MotorTxDisabledByIndex(3U) == 0U)
    {
        Arm_ReenableDamiaoMotor(3U, MOTOR_DAMIAO_4_ID, Motor4, &Damiao_motor_data[0]);
    }
    if (Arm_MotorTxDisabledByIndex(4U) == 0U)
    {
        Arm_ReenableDamiaoMotor(4U, MOTOR_DAMIAO_5_ID, Motor5, &Damiao_motor_data[1]);
    }
    if (Arm_MotorTxDisabledByIndex(5U) == 0U)
    {
        Arm_ReenableDamiaoMotor(5U, MOTOR_DAMIAO_6_ID, Motor6, &Damiao_motor_data[2]);
    }
}

void Arm_RequestDisabledFeedback(void)
{
    static uint32_t last_request_tick = 0U;
    uint32_t now_tick = HAL_GetTick();
    uint8_t logical_motor;

    if ((now_tick - last_request_tick) < ARM_DISABLED_FEEDBACK_PERIOD_MS)
    {
        return;
    }

    last_request_tick = now_tick;

    for (logical_motor = 0U; logical_motor < 3U; logical_motor++)
    {
        RobStride_Motor_t *motor;

        if (Arm_MotorTxDisabledByIndex(logical_motor) == 0U)
        {
            continue;
        }

        motor = Arm_GetLinzuMotorByIndex(logical_motor);
        if (motor == NULL)
        {
            continue;
        }

        Get_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0x7019U);
        osDelay(1U);
    }

    for (logical_motor = 3U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_MotorTxDisabledByIndex(logical_motor) == 0U)
        {
            continue;
        }

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            continue;
        }

        (void)motor_index;
        disable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        osDelay(1U);
    }
}

#define ARM_LINZU_ZERO_STOP_DELAY_MS 20U
#define ARM_LINZU_ZERO_APPLY_DELAY_MS 50U
#define ARM_LINZU_ZERO_REENABLE_DELAY_MS 20U

static void Arm_Linzu_SendStop(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_MotorStop << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SendSetZero(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_SetPosZero << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    txdata[0] = 1U;
    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SaveZeroAndHold(uint8_t logical_motor, RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    motor_data->target_angle = 0.0f;
    motor_data->target_velocity = 1.0f;

    Arm_Linzu_SendStop(motor);
    osDelay(ARM_LINZU_ZERO_STOP_DELAY_MS);
    Arm_Linzu_SendSetZero(motor);
    osDelay(ARM_LINZU_ZERO_APPLY_DELAY_MS);

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Arm_EnableLinzuMotorFeedback(motor);
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7017, motor_data->target_velocity, 'p');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    (void)Arm_SendLinzuTarget(logical_motor, motor, motor_data);
}

uint8_t Arm_AdjustMotorTargetByIndex(uint8_t logical_motor, float delta_angle)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    motor_data->target_angle = Arm_ClampTargetAngleByIndex(logical_motor, motor_data->target_angle + delta_angle);
    return 1U;
}

uint8_t Arm_EnableMotorByIndex(uint8_t logical_motor)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    if (logical_motor < 3U)
    {
        RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

        if (motor == NULL)
        {
            return 0U;
        }

        Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
        Arm_EnableLinzuMotorFeedback(motor);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
        return Arm_SendLinzuTarget(logical_motor, motor, motor_data);
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        enable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        set_DM_mode(motor_index, POS_MODE);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
        return Arm_SendDamiaoTarget(logical_motor, motor_id, motor_data);
    }
}

uint8_t Arm_EnableAllMotors(void)
{
    uint8_t logical_motor;
    uint8_t all_enabled = 1U;

    for (logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        if (Arm_EnableMotorByIndex(logical_motor) == 0U)
        {
            all_enabled = 0U;
        }

        osDelay(1);
    }

    return all_enabled;
}

uint8_t Arm_DisableMotorByIndex(uint8_t logical_motor)
{
    if (logical_motor < 3U)
    {
        RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

        if (motor == NULL)
        {
            return 0U;
        }

        Disenable_Motor(motor, CAN_HANDLE_2, 0U);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
        return 1U;
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        disable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
        Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
        return 1U;
    }
}

uint8_t Arm_SaveMotorZeroByIndex(uint8_t logical_motor)
{
    ArmMotorData_t *motor_data = Arm_GetMotorDataByIndex(logical_motor);

    if (motor_data == NULL)
    {
        return 0U;
    }

    if (logical_motor < 3U)
    {
        RobStride_Motor_t *motor = Arm_GetLinzuMotorByIndex(logical_motor);

        if (motor == NULL)
        {
            return 0U;
        }

        Arm_Linzu_SaveZeroAndHold(logical_motor, motor, motor_data);
        return 1U;
    }

    {
        uint16_t motor_id;
        uint16_t motor_index;

        if (Arm_GetDamiaoMotorInfoByIndex(logical_motor, &motor_id, &motor_index) == 0U)
        {
            return 0U;
        }

        motor_data->target_angle = 0.0f;
        CAN_Send_Save_Zero(CAN_HANDLE_2, motor_id);
        osDelay(1);
        CAN_Send_Enter(CAN_HANDLE_2, motor_id);
        osDelay(1);
        return Arm_SendDamiaoTarget(logical_motor, motor_id, motor_data);
    }
}

void Arm_save_position(void)
{
    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;

    Arm_Linzu_SaveZeroAndHold(0U, &motor1, &Linzu_motor_data[0]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(1U, &motor2, &Linzu_motor_data[1]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(2U, &motor3, &Linzu_motor_data[2]);
    osDelay(1);

    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);

    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(3U, MOTOR_DAMIAO_4_ID, &Damiao_motor_data[0]);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(4U, MOTOR_DAMIAO_5_ID, &Damiao_motor_data[1]);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);
    (void)Arm_SendDamiaoTarget(5U, MOTOR_DAMIAO_6_ID, &Damiao_motor_data[2]);
}
// void Arm_Motor_Disable_All(void)
// {
//     Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
// }
//  void Arm_Motor_Enable_All(void)
// {
//     Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
//     osDelay(1);
//     disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);

// }
