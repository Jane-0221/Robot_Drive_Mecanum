#include "remote_control.h"
#include "IMU_updata.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "UART_data_txrx.h"
#include "Sbus.h"
#include "head.h"
#include "arm.h"
#include "servo_lift_control.h"
#include "pump_control.h"
#include "arm_sv.h"
#include "uart_protocol.h" // 添加PC通信协议头文件
#include "chassis.h"
#include "mecanum_wheel.h"
#include "cmsis_os.h"
#include "Robstride04.h"
#include "rs485_lift.h"
#include <math.h>
// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50
#define ARM_MOTOR_STEP 0.001f
#define ARM_SELECTED_MOTOR_INVALID 0xFFU
#define ARM_MOTOR_CH3_ENABLE_THRESHOLD 1500U
#define ARM_MOTOR_CH3_DISABLE_THRESHOLD 500U
#define ARM_MOTOR_CH3_LATCH_NONE 0U
#define ARM_MOTOR_CH3_LATCH_ENABLE 1U
#define ARM_MOTOR_CH3_LATCH_DISABLE 2U
#define HEAD_PI 3.14159265358979323846f
#define HEAD_RAD_TO_ANGLE_UNIT (18000.0f / 3.14159265358979323846f)
#define HEAD_SINGLE_TURN_UNITS 36000L
#define HEAD_SINGLE_TURN_DEG 360.0f
#define HEAD_HALF_TURN_DEG 180.0f
#define HEAD_DEG_TO_RAD (HEAD_PI / HEAD_HALF_TURN_DEG)
#define HEAD_ANGLE_SANITY_LIMIT_DEG 1000000.0f
#define HEAD_PC_TX_ZERO_DEADBAND_DEG 1.0f
#define HEAD_MOTOR1_PC_MIN_RAD (-1700.0f * HEAD_PI / 18000.0f)
#define HEAD_MOTOR1_PC_MAX_RAD (2000.0f * HEAD_PI / 18000.0f)
#define HEAD_MOTOR2_PC_MIN_RAD (-9000.0f * HEAD_PI / 18000.0f)
#define HEAD_MOTOR2_PC_MAX_RAD (9000.0f * HEAD_PI / 18000.0f)
#define PC_ARM_MOTOR_DEBUG_STATE_NONE 0xFFU
#define PC_CHASSIS_TIMEOUT_MS 200U
#define PC_CHASSIS_LINEAR_LIMIT 1.0f
#define PC_CHASSIS_YAW_LIMIT 1.5f
#define PC_CHASSIS_SANITY_LIMIT 1000000.0f
#define CHASSIS_LIDAR_FORWARD_YAW_OFFSET_DEFAULT_DEG 0.0f
#define CHASSIS_LIDAR_YAW_OFFSET_SANITY_DEG 360.0f
#define CHASSIS_DEG_TO_RAD (HEAD_PI / HEAD_HALF_TURN_DEG)
// extern DnData_t pc_dn_data;

static const float arm_servo_pose_left[ARM_SV_COUNT] = {
    -0.101999983f,
    -0.29700008f,
    1.80600858f,
    2.40001273f,
    0.0119999889f,
    -0.0150000118f,
};

static const float arm_servo_pose_center[ARM_SV_COUNT] = {
    -0.101999983f,
    -0.29700008f,
    1.79100847f,
    1.88400912f,
    0.0119999889f,
    -0.0150000118f,
};

static const float arm_servo_pose_right[ARM_SV_COUNT] = {
    -0.101999983f,
    -0.29700008f,
    1.88700914f,
    1.30800509f,
    0.0119999889f,
    -0.0150000118f,
};

volatile uint8_t pc_arm_motor_enable_state_debug[ARM_LOGICAL_MOTOR_COUNT] = {
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
};
volatile uint8_t pc_arm_motor_last_index_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint8_t pc_arm_motor_last_enable_state_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint32_t pc_arm_motor_command_count_debug = 0U;
volatile uint8_t pc_head_motor_enable_state_debug[2] = {
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
};
volatile uint8_t pc_head_motor_last_index_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint8_t pc_head_motor_last_enable_state_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint32_t pc_head_motor_command_count_debug = 0U;
volatile PcMotorCommandDebug_t pc_motor_command_debug = {
    .latest_rx_command = {
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
    },
    .latest_rx_valid = 0U,
    .latest_rx_count = 0U,
    .latest_handled_command = {
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
    },
    .latest_handled_valid = 0U,
    .arm_motor_command = {
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
    },
    .head_motor_command = {
        PC_MOTOR_CTRL_STATE_NONE,
        PC_MOTOR_CTRL_STATE_NONE,
    },
    .arm_last_index = PC_MOTOR_CTRL_STATE_NONE,
    .arm_last_command = PC_MOTOR_CTRL_STATE_NONE,
    .arm_command_count = 0U,
    .head_last_index = PC_MOTOR_CTRL_STATE_NONE,
    .head_last_command = PC_MOTOR_CTRL_STATE_NONE,
    .head_command_count = 0U,
};
volatile ChassisRemoteSbusDebug_t chassis_remote_sbus = {0};
volatile ChassisCommandDebug_t chassis_command = {0};
volatile float chassis_lidar_yaw_offset_deg_debug = CHASSIS_LIDAR_FORWARD_YAW_OFFSET_DEFAULT_DEG;
volatile float chassis_remote_lidar_vx_debug = 0.0f;
volatile float chassis_remote_lidar_vy_debug = 0.0f;
volatile float chassis_remote_body_vx_debug = 0.0f;
volatile float chassis_remote_body_vy_debug = 0.0f;
volatile uint32_t control_mode_switch_count_debug = 0U;
volatile uint32_t control_mode_last_switch_tick_debug = 0U;
volatile uint8_t control_mode_last_request_debug = CONTROL_MODE_REMOTE;
volatile float head_pc_tx_source_deg_debug[2] = {0.0f, 0.0f};
volatile float head_pc_tx_normalized_deg_debug[2] = {0.0f, 0.0f};
volatile float head_pc_tx_rad_debug[2] = {0.0f, 0.0f};
volatile uint32_t head_pc_tx_invalid_count_debug[2] = {0U, 0U};

static volatile uint8_t arm_motor_disable_active = 0U;
static uint8_t arm_motor_disable_latched = 0U;
static volatile uint8_t head_motor_disable_active = 0U;
static volatile uint8_t head_motor_remote_disable_active = 0U;
static volatile uint8_t head_motor_user_key_disable_active = 0U;
static uint8_t head_motor_user_key_enabled_latched = 0U;
static uint8_t head_motor_enabled_latched = 1U;
static uint8_t arm_save_position = 0U;
static uint8_t arm_motor_ch3_latch[ARM_LOGICAL_MOTOR_COUNT] = {0U};
static uint8_t arm_motor_save_zero_latched = 0U;
static uint8_t head_save_zero_latched = 0U;
static uint8_t chassis_lidar_yaw_cache_valid = 0U;
static float chassis_lidar_yaw_cache_deg = CHASSIS_LIDAR_FORWARD_YAW_OFFSET_DEFAULT_DEG;
static float chassis_lidar_yaw_cache_cos = 1.0f;
static float chassis_lidar_yaw_cache_sin = 0.0f;
static uint32_t pc_lift_last_dn_rx_count = 0U;
static uint16_t pc_lift_last_target_height = 0xFFFFU;
static uint8_t pc_lift_last_target_valid = 0U;

static uint8_t sbus_match(uint16_t value, uint16_t target);
static uint8_t chassis_remote_mode_active(void);
static float chassis_sanitize_lidar_yaw_offset(float yaw_offset_deg);
static void chassis_lidar_to_body_command(float lidar_vx, float lidar_vy, float *body_vx, float *body_vy);

static void PC_Motor_Command_DebugRefresh(void)
{
    pc_motor_command_debug.latest_rx_command.target_type = pc_motor_ctrl_latest.target_type;
    pc_motor_command_debug.latest_rx_command.motor_index = pc_motor_ctrl_latest.motor_index;
    pc_motor_command_debug.latest_rx_command.command = pc_motor_ctrl_latest.command;
    pc_motor_command_debug.latest_rx_valid = pc_motor_ctrl_latest_valid;
    pc_motor_command_debug.latest_rx_count = pc_motor_ctrl_rx_count;

    for (uint8_t i = 0U; i < ARM_LOGICAL_MOTOR_COUNT; i++)
    {
        pc_motor_command_debug.arm_motor_command[i] = pc_arm_motor_enable_state_debug[i];
    }

    pc_motor_command_debug.head_motor_command[0] = pc_head_motor_enable_state_debug[0];
    pc_motor_command_debug.head_motor_command[1] = pc_head_motor_enable_state_debug[1];
    pc_motor_command_debug.arm_last_index = pc_arm_motor_last_index_debug;
    pc_motor_command_debug.arm_last_command = pc_arm_motor_last_enable_state_debug;
    pc_motor_command_debug.arm_command_count = pc_arm_motor_command_count_debug;
    pc_motor_command_debug.head_last_index = pc_head_motor_last_index_debug;
    pc_motor_command_debug.head_last_command = pc_head_motor_last_enable_state_debug;
    pc_motor_command_debug.head_command_count = pc_head_motor_command_count_debug;
}

static void PC_Motor_Command_DebugSetHandled(const PcMotorCtrl_t *command)
{
    if (command == NULL)
    {
        return;
    }

    pc_motor_command_debug.latest_handled_command.target_type = command->target_type;
    pc_motor_command_debug.latest_handled_command.motor_index = command->motor_index;
    pc_motor_command_debug.latest_handled_command.command = command->command;
    pc_motor_command_debug.latest_handled_valid = 1U;
}

static void Chassis_LiveWatch_Refresh(void)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t pc_last_tick = pc_chassis_ctrl_last_tick;

    chassis_remote_sbus.SBUS_CH = SBUS_CH;

    chassis_command.uwTick = tick_now;
    chassis_command.control_mode = control_mode;
    chassis_command.chassis_mode = chassis_mode;
    chassis_command.x = x;
    chassis_command.y = y;
    chassis_command.w = w;
    chassis_command.up_tx_data = up_tx_data;
    chassis_command.pc_rx.x = pc_chassis_ctrl_latest.x;
    chassis_command.pc_rx.y = pc_chassis_ctrl_latest.y;
    chassis_command.pc_rx.w = pc_chassis_ctrl_latest.w;
    chassis_command.pc_rx_valid = pc_chassis_ctrl_latest_valid;
    chassis_command.pc_rx_count = pc_chassis_ctrl_rx_count;
    chassis_command.pc_rx_last_tick = pc_last_tick;
    chassis_command.pc_mode_rx = pc_control_mode_latest;
    chassis_command.pc_mode_rx_valid = pc_control_mode_latest_valid;
    chassis_command.pc_mode_rx_count = pc_control_mode_rx_count;
    chassis_command.pc_mode_rx_last_tick = pc_control_mode_last_tick;
    chassis_command.control_mode_switch_count = control_mode_switch_count_debug;
    chassis_command.control_mode_last_switch_tick = control_mode_last_switch_tick_debug;

    if (pc_last_tick == 0U)
    {
        chassis_command.pc_rx_age_ms = 0xFFFFFFFFU;
        chassis_command.pc_rx_timeout = 1U;
    }
    else
    {
        chassis_command.pc_rx_age_ms = tick_now - pc_last_tick;
        chassis_command.pc_rx_timeout = (chassis_command.pc_rx_age_ms > PC_CHASSIS_TIMEOUT_MS) ? 1U : 0U;
    }
}

static uint8_t sbus_match(uint16_t value, uint16_t target)
{
    return (value >= (target - RANGE)) && (value <= (target + RANGE));
}

static uint8_t chassis_remote_mode_active(void)
{
    return (sbus_match(SBUS_CH.CH8, LOW_VALUE) &&
            sbus_match(SBUS_CH.CH6, LOW_VALUE) &&
            sbus_match(SBUS_CH.CH7, LOW_VALUE) &&
            sbus_match(SBUS_CH.CH5, MID_VALUE)) ? 1U : 0U;
}

static float sbus_axis_to_float(uint16_t value, float scale)
{
    int32_t delta = (int32_t)value - MID_VALUE;

    if ((delta > -RANGE) && (delta < RANGE))
    {
        return 0.0f;
    }

    return (float)delta / scale;
}

static uint8_t pc_chassis_float_is_valid(float value)
{
    return ((value == value) &&
            (value <= PC_CHASSIS_SANITY_LIMIT) &&
            (value >= -PC_CHASSIS_SANITY_LIMIT)) ? 1U : 0U;
}

static float pc_chassis_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float chassis_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float chassis_sanitize_lidar_yaw_offset(float yaw_offset_deg)
{
    if ((yaw_offset_deg != yaw_offset_deg) ||
        (yaw_offset_deg > CHASSIS_LIDAR_YAW_OFFSET_SANITY_DEG) ||
        (yaw_offset_deg < -CHASSIS_LIDAR_YAW_OFFSET_SANITY_DEG))
    {
        return CHASSIS_LIDAR_FORWARD_YAW_OFFSET_DEFAULT_DEG;
    }

    return yaw_offset_deg;
}

static void chassis_lidar_to_body_command(float lidar_vx, float lidar_vy, float *body_vx, float *body_vy)
{
    float yaw_offset_deg = chassis_sanitize_lidar_yaw_offset(chassis_lidar_yaw_offset_deg_debug);

    if ((chassis_lidar_yaw_cache_valid == 0U) ||
        (yaw_offset_deg != chassis_lidar_yaw_cache_deg))
    {
        float yaw_offset_rad = yaw_offset_deg * CHASSIS_DEG_TO_RAD;

        chassis_lidar_yaw_cache_cos = cosf(yaw_offset_rad);
        chassis_lidar_yaw_cache_sin = sinf(yaw_offset_rad);
        chassis_lidar_yaw_cache_deg = yaw_offset_deg;
        chassis_lidar_yaw_cache_valid = 1U;
    }

    *body_vx = lidar_vx * chassis_lidar_yaw_cache_cos -
               lidar_vy * chassis_lidar_yaw_cache_sin;
    *body_vy = lidar_vx * chassis_lidar_yaw_cache_sin +
               lidar_vy * chassis_lidar_yaw_cache_cos;
}

static void chassis_apply_command(float chassis_vx, float chassis_vy, float chassis_yaw)
{
    float linear_sum = chassis_abs_float(chassis_vx) + chassis_abs_float(chassis_vy);

    if (linear_sum > PC_CHASSIS_LINEAR_LIMIT)
    {
        float scale = PC_CHASSIS_LINEAR_LIMIT / linear_sum;
        chassis_vx *= scale;
        chassis_vy *= scale;
    }

    chassis_yaw = pc_chassis_clamp(chassis_yaw, PC_CHASSIS_YAW_LIMIT);
    up_tx_data.chassis_vx = chassis_vx;
    up_tx_data.chassis_vy = chassis_vy;
    up_tx_data.chassis_yaw = chassis_yaw;
    Chassis_SetCommand(chassis_vx, chassis_vy, chassis_yaw);
    Chassis_LiveWatch_Refresh();
}

void Control_Mode_Updata(void)
{
    uint8_t requested_mode;
    uint8_t new_mode;

    if (UART_Protocol_GetControlModeCommand(&requested_mode) == 0U)
    {
        return;
    }

    if (requested_mode == PC_CONTROL_MODE_PC)
    {
        new_mode = CONTROL_MODE_PC;
    }
    else if (requested_mode == PC_CONTROL_MODE_RC)
    {
        new_mode = CONTROL_MODE_REMOTE;
    }
    else
    {
        return;
    }

    control_mode_last_request_debug = requested_mode;

    if (control_mode != new_mode)
    {
        control_mode = new_mode;
        control_mode_switch_count_debug++;
        control_mode_last_switch_tick_debug = HAL_GetTick();

        up_tx_data.chassis_vx = 0.0f;
        up_tx_data.chassis_vy = 0.0f;
        up_tx_data.chassis_yaw = 0.0f;
        Chassis_SetCommand(0.0f, 0.0f, 0.0f);
    }

    Chassis_LiveWatch_Refresh();
}

static float head_clamp_pc_head_radian(uint8_t motor_index, float radian)
{
    float min_radian;
    float max_radian;

    if (radian != radian)
    {
        return 0.0f;
    }

    if (motor_index == 0U)
    {
        min_radian = HEAD_MOTOR1_PC_MIN_RAD;
        max_radian = HEAD_MOTOR1_PC_MAX_RAD;
    }
    else
    {
        min_radian = HEAD_MOTOR2_PC_MIN_RAD;
        max_radian = HEAD_MOTOR2_PC_MAX_RAD;
    }

    if (radian < min_radian)
    {
        return min_radian;
    }

    if (radian > max_radian)
    {
        return max_radian;
    }

    return radian;
}

static uint32_t head_radian_to_target_angle(uint8_t motor_index, float radian)
{
    float angle_unit_f;
    int32_t angle_unit;

    radian = head_clamp_pc_head_radian(motor_index, radian);

    angle_unit_f = radian * HEAD_RAD_TO_ANGLE_UNIT;
    angle_unit = (angle_unit_f >= 0.0f) ? (int32_t)(angle_unit_f + 0.5f) : (int32_t)(angle_unit_f - 0.5f);

    while (angle_unit < 0)
    {
        angle_unit += HEAD_SINGLE_TURN_UNITS;
    }

    while (angle_unit >= HEAD_SINGLE_TURN_UNITS)
    {
        angle_unit -= HEAD_SINGLE_TURN_UNITS;
    }

    return (uint32_t)angle_unit;
}

static uint8_t head_degree_is_invalid(float angle_deg)
{
    return ((angle_deg != angle_deg) ||
            (angle_deg > HEAD_ANGLE_SANITY_LIMIT_DEG) ||
            (angle_deg < -HEAD_ANGLE_SANITY_LIMIT_DEG)) ? 1U : 0U;
}

static float head_normalize_degree_for_pc_tx(float angle_deg)
{
    if (head_degree_is_invalid(angle_deg) != 0U)
    {
        return 0.0f;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += HEAD_SINGLE_TURN_DEG;
    }

    while (angle_deg >= HEAD_SINGLE_TURN_DEG)
    {
        angle_deg -= HEAD_SINGLE_TURN_DEG;
    }

    if ((angle_deg <= HEAD_PC_TX_ZERO_DEADBAND_DEG) ||
        ((HEAD_SINGLE_TURN_DEG - angle_deg) <= HEAD_PC_TX_ZERO_DEADBAND_DEG))
    {
        return 0.0f;
    }

    if (angle_deg > HEAD_HALF_TURN_DEG)
    {
        angle_deg -= HEAD_SINGLE_TURN_DEG;
    }

    return angle_deg;
}

static float head_degree_to_pc_radian(float angle_deg)
{
    angle_deg = head_normalize_degree_for_pc_tx(angle_deg);
    return angle_deg * HEAD_DEG_TO_RAD;
}

static uint8_t pc_rs485_lift_float_is_valid(float value)
{
    return ((value == value) &&
            (value <= PC_CHASSIS_SANITY_LIMIT) &&
            (value >= -PC_CHASSIS_SANITY_LIMIT)) ? 1U : 0U;
}

static Rs485LiftCommandId_t pc_rs485_lift_map_command(uint8_t pc_command)
{
    switch (pc_command)
    {
    case PC_RS485_LIFT_CMD_SETUP:
        return RS485_LIFT_CMD_SETUP;
    case PC_RS485_LIFT_CMD_FORWARD:
        return RS485_LIFT_CMD_FORWARD;
    case PC_RS485_LIFT_CMD_REVERSE:
        return RS485_LIFT_CMD_REVERSE;
    case PC_RS485_LIFT_CMD_STOP:
        return RS485_LIFT_CMD_STOP;
    case PC_RS485_LIFT_CMD_ENABLE:
        return RS485_LIFT_CMD_ENABLE;
    case PC_RS485_LIFT_CMD_DISABLE:
        return RS485_LIFT_CMD_DISABLE;
    case PC_RS485_LIFT_CMD_SPEED:
        return RS485_LIFT_CMD_SPEED;
    case PC_RS485_LIFT_CMD_FIND_LIMITS:
        return RS485_LIFT_CMD_FIND_LIMITS;
    case PC_RS485_LIFT_CMD_MOVE:
        return RS485_LIFT_CMD_MOVE;
    case PC_RS485_LIFT_CMD_GOTO_HEIGHT:
        return RS485_LIFT_CMD_GOTO_HEIGHT;
    case PC_RS485_LIFT_CMD_SET_ZERO:
        return RS485_LIFT_CMD_SET_ZERO;
    case PC_RS485_LIFT_CMD_SET_LOWER:
        return RS485_LIFT_CMD_SET_LOWER;
    case PC_RS485_LIFT_CMD_SET_TOP:
        return RS485_LIFT_CMD_SET_TOP;
    case PC_RS485_LIFT_CMD_SET_UPPER:
        return RS485_LIFT_CMD_SET_UPPER;
    case PC_RS485_LIFT_CMD_APPLY_LIMITS:
        return RS485_LIFT_CMD_APPLY_LIMITS;
    case PC_RS485_LIFT_CMD_CALIBRATE_HEIGHT:
        return RS485_LIFT_CMD_CALIBRATE_HEIGHT;
    case PC_RS485_LIFT_CMD_SET_MANUAL_LIMITS:
        return RS485_LIFT_CMD_SET_MANUAL_LIMITS;
    case PC_RS485_LIFT_CMD_CALIBRATE_DISTANCE:
        return RS485_LIFT_CMD_CALIBRATE_DISTANCE;
    case PC_RS485_LIFT_CMD_RESTORE_LAST_POSITION:
        return RS485_LIFT_CMD_RESTORE_LAST_POSITION;
    case PC_RS485_LIFT_CMD_DISABLE_DRIVE_LIMITS:
        return RS485_LIFT_CMD_DISABLE_DRIVE_LIMITS;
    case PC_RS485_LIFT_CMD_CLEAR_LIMITS:
        return RS485_LIFT_CMD_CLEAR_LIMITS;
    default:
        return RS485_LIFT_CMD_NONE;
    }
}

void remote_control_init()
{
}

static uint8_t Head_Motor_ApplyDisableRequest(uint8_t disable_request)
{
    head_motor_disable_active = disable_request;

    if (disable_request != 0U)
    {
        if (head_motor_enabled_latched != 0U)
        {
            Head_Motor_Disable();
            head_motor_enabled_latched = 0U;
        }

        return 1U;
    }

    if (head_motor_enabled_latched == 0U)
    {
        Head_Motor_Enable();
        head_motor_enabled_latched = 1U;
    }

    return 0U;
}

uint8_t Arm_Motor_Disable_Updata(void)
{
    uint8_t disable_request = (SBUS_CH.CH5 == LOW_VALUE) &&
                              (SBUS_CH.CH6 == HIGH_VALUE) &&
                              (SBUS_CH.CH7 == HIGH_VALUE) &&
                              (SBUS_CH.CH8 == HIGH_VALUE);

    arm_motor_disable_active = disable_request;

    if (disable_request == 0U)
    {
        arm_motor_disable_latched = 0U;
        return 0U;
    }

    if (arm_motor_disable_latched != 0U)
    {
        return 1U;
    }
    for (uint8_t logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        (void)Arm_DisableMotorByIndex(logical_motor);
        osDelay(1);
    }

    arm_motor_disable_latched = 1U;
    return 1U;
}

uint8_t Arm_Motor_Disable_IsActive(void)
{
    return arm_motor_disable_active;
}

uint8_t Head_Motor_Enable_Disable_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        head_motor_remote_disable_active = 1U;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        head_motor_remote_disable_active = 0U;
    }

    return Head_Motor_ApplyDisableRequest(
        ((head_motor_remote_disable_active != 0U) ||
         (head_motor_user_key_disable_active != 0U)) ? 1U : 0U);
}

uint8_t Head_Motor_ToggleByUserKey(void)
{
    if (head_motor_user_key_enabled_latched == 0U)
    {
        head_motor_user_key_disable_active = 0U;
        head_motor_user_key_enabled_latched = 1U;
    }
    else
    {
        head_motor_user_key_disable_active = 1U;
        head_motor_user_key_enabled_latched = 0U;
    }

    return Head_Motor_ApplyDisableRequest(
        ((head_motor_remote_disable_active != 0U) ||
         (head_motor_user_key_disable_active != 0U)) ? 1U : 0U);
}

uint8_t Head_Motor_Disable_IsActive(void)
{
    return head_motor_disable_active;
}

uint8_t Arm_Save_Position_IsActive(void)
{
    return arm_save_position;
}

void Pump_Control_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        pump_state = PUMP_ON;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        pump_state = PUMP_OFF;
    }
}

static uint8_t Arm_GetRemoteSelectedMotor(void)
{
    uint8_t motor_offset;

    if (chassis_remote_mode_active() != 0U)
    {
        return ARM_SELECTED_MOTOR_INVALID;
    }

    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        motor_offset = 0U;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        motor_offset = 3U;
    }
    else
    {
        return ARM_SELECTED_MOTOR_INVALID;
    }

    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        return motor_offset;

    case MID_VALUE:
        return motor_offset + 1U;

    case LOW_VALUE:
        return motor_offset + 2U;

    default:
        return ARM_SELECTED_MOTOR_INVALID;
    }
}

static void Arm_ResetMotorCh3Latches(void)
{
    uint8_t i;

    for (i = 0U; i < ARM_LOGICAL_MOTOR_COUNT; i++)
    {
        arm_motor_ch3_latch[i] = ARM_MOTOR_CH3_LATCH_NONE;
    }
}

void Arm_Motor_Control_Updata(void)
{
    uint8_t selected_motor = Arm_GetRemoteSelectedMotor();

    if (selected_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        if ((SBUS_CH.CH3 <= ARM_MOTOR_CH3_ENABLE_THRESHOLD) &&
            (SBUS_CH.CH3 >= ARM_MOTOR_CH3_DISABLE_THRESHOLD))
        {
            Arm_ResetMotorCh3Latches();
        }

        if (SBUS_CH.CH5 != HIGH_VALUE)
        {
            arm_motor_save_zero_latched = 0U;
        }

        return;
    }

    if (SBUS_CH.CH1 != 0U)
    {
        if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
        {
            (void)Arm_AdjustMotorTargetByIndex(selected_motor, ARM_MOTOR_STEP);
        }
        else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
        {
            (void)Arm_AdjustMotorTargetByIndex(selected_motor, -ARM_MOTOR_STEP);
        }
    }

    if (SBUS_CH.CH3 > ARM_MOTOR_CH3_ENABLE_THRESHOLD)
    {
        if (arm_motor_ch3_latch[selected_motor] != ARM_MOTOR_CH3_LATCH_ENABLE)
        {
            (void)Arm_EnableMotorByIndex(selected_motor);
            arm_motor_ch3_latch[selected_motor] = ARM_MOTOR_CH3_LATCH_ENABLE;
        }
    }
    else if (SBUS_CH.CH3 < ARM_MOTOR_CH3_DISABLE_THRESHOLD)
    {
        if (arm_motor_ch3_latch[selected_motor] != ARM_MOTOR_CH3_LATCH_DISABLE)
        {
            (void)Arm_DisableMotorByIndex(selected_motor);
            arm_motor_ch3_latch[selected_motor] = ARM_MOTOR_CH3_LATCH_DISABLE;
        }
    }
    else
    {
        Arm_ResetMotorCh3Latches();
    }

    if (SBUS_CH.CH5 == HIGH_VALUE)
    {
        if (arm_motor_save_zero_latched == 0U)
        {
            (void)Arm_SaveMotorZeroByIndex(selected_motor);
            arm_motor_save_zero_latched = 1U;
        }
    }
    else
    {
        arm_motor_save_zero_latched = 0U;
    }
}
void arm_save_home_position(void)
{
    if (SBUS_CH.CH5 == HIGH_VALUE && SBUS_CH.CH6 == HIGH_VALUE)
    {
        arm_save_position = 1U;
        Linzu_motor_data[0].target_angle = 0.0f;
        Linzu_motor_data[1].target_angle = 0.0f;
        Linzu_motor_data[2].target_angle = 0.0f;
        Damiao_motor_data[0].target_angle = 0.0f;
        Damiao_motor_data[1].target_angle = 0.0f;
        Damiao_motor_data[2].target_angle = 0.0f;

        return;
    }
    arm_save_position = 0U;
}
void head_save_home_position(void)
{
    if (SBUS_CH.CH5 == HIGH_VALUE && SBUS_CH.CH6 == HIGH_VALUE)
    {
        if (head_save_zero_latched == 0U)
        {
            head_motor_data[0].target_angle = 0.0f;
            head_motor_data[1].target_angle = 0.0f;
            Head_save_position();
            head_save_zero_latched = 1U;
        }

        return;
    }

    head_save_zero_latched = 0U;
}

void Head_Motor_Control_Updata(void)
{
    // 替换原有if-else if结构为switch语句
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        switch (SBUS_CH.CH6)
        {
        case HIGH_VALUE:
            // head_motor_data[0].target_angle = 0;
            // head_motor_data[1].target_angle = 0; //

            // duties_tx.duty0 = 0.12;
            //  duties_tx.duty1 = 0.12;
            //  duties_tx.duty2 = 0.12;
            //  duties_tx.duty3 = 0.12;
            //  duties_tx.duty4 = 0.12;
            //  duties_tx.duty5 = 0.12;
            // motor_radians[0] = PI / 4;
            // motor_radians[1] = PI / 3;
            // motor_radians[2] = PI / 2;
            // motor_radians[3] = PI / 2;
            // motor_radians[4] = PI / 2;
            // motor_radians[5] = PI / 2;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(0U, 0.003f);
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(0U, -0.003f);
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(1U, 0.003f);
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(1U, -0.003f);
            }

            break;

        case MID_VALUE:
            // head_motor_data[0].target_angle = 9000;
            // head_motor_data[1].target_angle = 9000; // 头部电机

            // duties_tx.duty0 = 0.075;
            // duties_tx.duty1 = 0.075;
            // duties_tx.duty2 = 0.075;
            // duties_tx.duty3 = 0.075;
            // duties_tx.duty4 = 0.075;
            // duties_tx.duty5 = 0.075;
            // motor_radians[0] = 0.0f;
            // motor_radians[1] = 0.0f;
            // motor_radians[2] = 0.0f;
            // motor_radians[3] = 0.0f;
            // motor_radians[4] = 0.0f;
            // motor_radians[5] = 0.0f;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(2U, 0.003f);
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(2U, -0.003f);
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(3U, 0.003f);
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(3U, -0.003f);
            }

            break;

        case LOW_VALUE:
            // head_motor_data[0].target_angle = 18000;
            // head_motor_data[1].target_angle = 18000;

            // duties_tx.duty0 = 0.03;
            // duties_tx.duty1 = 0.03;
            // duties_tx.duty2 = 0.03;
            // duties_tx.duty3 = 0.03;
            // duties_tx.duty4 = 0.03;
            // duties_tx.duty5 = 0.03;
            // motor_radians[0] = -PI / 4;
            // motor_radians[1] = -PI / 3;
            // motor_radians[2] = -PI / 2;
            // motor_radians[3] = -PI / 2;
            // motor_radians[4] = -PI / 4;
            // motor_radians[5] = -PI / 4;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(4U, 0.003f);
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(4U, -0.003f);
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                ARM_SV_AdjustRampTarget(5U, 0.003f);
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                ARM_SV_AdjustRampTarget(5U, -0.003f);
            }

            break;

        default:
            // 可选：处理SBUS_CH.CH6不是上述三个值的情况
            // 如果不需要特殊处理，这里可以留空
            break;
        }
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        switch (SBUS_CH.CH6)
        {
        case HIGH_VALUE: // 挥手到左边（第三张图的弧度值）
            ARM_SV_SetAllRampTargets(arm_servo_pose_left);
            break;

        case MID_VALUE: // 举手（中间姿态，第二张图的弧度值）
            ARM_SV_SetAllRampTargets(arm_servo_pose_center);
            break;

        case LOW_VALUE: // 挥手到右边（第一张图的弧度值）
            ARM_SV_SetAllRampTargets(arm_servo_pose_right);
            break;

        // 默认情况：保持当前关节角度（避免无操作时数组值异常）
        default:
            // 可选：如果需要默认姿态，可赋值为举手姿态
            // motor_radians[0] = -0.0450000018f;
            // ... 其他关节赋值
            break;
        }
    }
}

void Up_Down_Motor_Control_Updata(void)
{
    aim_tx_height = 70;
    // switch (SBUS_CH.CH7)
    // {
    // case HIGH_VALUE:
    //     aim_tx_height = LIFT_TARGET_HEIGHT_MIN_MM;
    //     break;
    // case LOW_VALUE:
    //     aim_tx_height = 700;
    //     break;
    // case MID_VALUE:
    //     aim_tx_height = 400;
    //     break;
    // default:
    //     break;
    // }
}

void Chassis_Control_Updata(void)
{
    float lidar_vx = 0.0f;
    float lidar_vy = 0.0f;
    float chassis_vx = 0.0f;
    float chassis_vy = 0.0f;
    float chassis_yaw = 0.0f;

    if (chassis_remote_mode_active() != 0U)
    {
        lidar_vx = -sbus_axis_to_float(SBUS_CH.CH2, 900.0f);
        lidar_vy = sbus_axis_to_float(SBUS_CH.CH1, 900.0f);
        chassis_yaw = -sbus_axis_to_float(SBUS_CH.CH3, 810.0f);
        chassis_lidar_to_body_command(lidar_vx, lidar_vy, &chassis_vx, &chassis_vy);
    }

    chassis_remote_lidar_vx_debug = lidar_vx;
    chassis_remote_lidar_vy_debug = lidar_vy;
    chassis_remote_body_vx_debug = chassis_vx;
    chassis_remote_body_vy_debug = chassis_vy;
    chassis_apply_command(chassis_vx, chassis_vy, chassis_yaw);
}

void PC_Chassis_Control_Updata(void)
{
    PcChassisCtrl_t pc_chassis_command;
    uint32_t last_tick;
    uint32_t tick_now = HAL_GetTick();

    if (UART_Protocol_CopyLatestChassisCtrl(&pc_chassis_command, &last_tick) == 0U)
    {
        chassis_apply_command(0.0f, 0.0f, 0.0f);
        return;
    }

    if ((tick_now - last_tick) > PC_CHASSIS_TIMEOUT_MS)
    {
        chassis_apply_command(0.0f, 0.0f, 0.0f);
        return;
    }

    if ((pc_chassis_float_is_valid(pc_chassis_command.x) == 0U) ||
        (pc_chassis_float_is_valid(pc_chassis_command.y) == 0U) ||
        (pc_chassis_float_is_valid(pc_chassis_command.w) == 0U))
    {
        chassis_apply_command(0.0f, 0.0f, 0.0f);
        return;
    }

    chassis_apply_command(
        pc_chassis_clamp(pc_chassis_command.x, PC_CHASSIS_LINEAR_LIMIT),
        pc_chassis_clamp(pc_chassis_command.y, PC_CHASSIS_LINEAR_LIMIT),
        pc_chassis_clamp(pc_chassis_command.w, PC_CHASSIS_YAW_LIMIT));
}

// PC控制函数实现

/**
 * @brief PC控制气泵更新函数
 * @note 使用PC传入的气泵状态信息进行控制
 */
void PC_Pump_Control_Updata(void)
{
    // 使用PC传入的气泵状态信息
    if (pc_dn_data.pc_pump_state == 1)
    {
        pump_state = PUMP_ON;
    }
    else if (pc_dn_data.pc_pump_state == 0)
    {
        pump_state = PUMP_OFF;
    }
}

/**
 * @brief PC控制头部电机更新函数
 * @note 使用PC传入的电机角度信息（弧度单位）进行控制
 */
void PC_Head_Motor_Control_Updata(void)
{
    // 使用PC传入的头部电机目标角度信息（弧度）
    head_motor_data[0].target_angle = head_radian_to_target_angle(0U, pc_dn_data.pc_target_head_motor_angles[0]);
    head_motor_data[1].target_angle = head_radian_to_target_angle(1U, pc_dn_data.pc_target_head_motor_angles[1]);
}

/**
 * @brief PC控制升降电机更新函数
 * @note 使用PC传入的升降目标高度信息进行控制
 */
void PC_Up_Down_Motor_Control_Updata(void)
{
    uint16_t target_height;
    uint32_t dn_rx_count_snapshot = pc_dn_rx_count;
    Rs485LiftCommand_t command;

    // 使用PC传入的升降目标高度信息（0.1mm单位）
    if ((pc_dn_data.pc_target_lift_height > 0U) &&
        (pc_dn_data.pc_target_lift_height < LIFT_TARGET_HEIGHT_MIN_MM))
    {
        aim_tx_height = LIFT_TARGET_HEIGHT_MIN_MM;
    }
    else
    {
        aim_tx_height = pc_dn_data.pc_target_lift_height;
    }

    target_height = (uint16_t)aim_tx_height;
    if (dn_rx_count_snapshot == 0U)
    {
        return;
    }
    if ((pc_lift_last_target_valid != 0U) &&
        (pc_lift_last_dn_rx_count == dn_rx_count_snapshot) &&
        (pc_lift_last_target_height == target_height))
    {
        return;
    }
    if ((pc_lift_last_target_valid != 0U) &&
        (pc_lift_last_target_height == target_height))
    {
        pc_lift_last_dn_rx_count = dn_rx_count_snapshot;
        return;
    }

    Rs485Lift_SetDefaultCommand(&command, RS485_LIFT_CMD_GOTO_HEIGHT);
    command.target_height_mm = (float)target_height;
    if (Rs485Lift_SubmitCommand(&command) != 0U)
    {
        pc_lift_last_dn_rx_count = dn_rx_count_snapshot;
        pc_lift_last_target_height = target_height;
        pc_lift_last_target_valid = 1U;
    }
}

/**
 * @brief PC控制机械臂舵机和电机更新函数
 * @note 使用PC传入的舵机和电机角度值（弧度单位）分别赋值给motor_radians数组
 *       pc_target_servo_angles[0~5]对应舵机0~5
 *       pc_target_motor_angles[0~5]对应电机0~5
 */
void PC_Rs485_Lift_Control_Updata(void)
{
    PcRs485LiftCtrl_t pc_command;
    Rs485LiftCommand_t lift_command;
    Rs485LiftCommandId_t mapped_command;

    if (UART_Protocol_GetRs485LiftCommand(&pc_command) == 0U)
    {
        return;
    }

    mapped_command = pc_rs485_lift_map_command(pc_command.command);
    if (mapped_command == RS485_LIFT_CMD_NONE)
    {
        return;
    }

    if ((pc_rs485_lift_float_is_valid(pc_command.move_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.target_height_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.current_height_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.manual_lower_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.manual_upper_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.command_distance_mm) == 0U) ||
        (pc_rs485_lift_float_is_valid(pc_command.actual_distance_mm) == 0U))
    {
        return;
    }

    Rs485Lift_SetDefaultCommand(&lift_command, mapped_command);
    lift_command.flags = pc_command.flags;
    if (pc_command.rpm != 0U)
    {
        lift_command.rpm = pc_command.rpm;
    }
    if (pc_command.accel_rpm != 0U)
    {
        lift_command.accel_rpm = pc_command.accel_rpm;
    }
    lift_command.move_mm = pc_command.move_mm;
    lift_command.target_height_mm = pc_command.target_height_mm;
    lift_command.current_height_mm = pc_command.current_height_mm;
    lift_command.manual_lower_mm = pc_command.manual_lower_mm;
    lift_command.manual_upper_mm = pc_command.manual_upper_mm;
    lift_command.command_distance_mm = pc_command.command_distance_mm;
    lift_command.actual_distance_mm = pc_command.actual_distance_mm;

    (void)Rs485Lift_SubmitCommand(&lift_command);
}

void PC_Arm_Motor_Control_Updata(void)
{
    DnData_t pc_snapshot;
    float target_motor_angles[ARM_LOGICAL_MOTOR_COUNT];
    float target_motor_velocities[ARM_LOGICAL_MOTOR_COUNT];

    if (UART_Protocol_CopyLatestDnData(&pc_snapshot) == 0U)
    {
        return;
    }

    // 将PC传入的6路舵机角度值赋值给motor_radians数组
    ARM_SV_SetAllRampTargets(pc_snapshot.pc_target_servo_angles);
    // 将PC传入的6路电机角度值赋值给电机臂目标角度
    target_motor_angles[0] = pc_snapshot.pc_target_motor_angles[0];
    target_motor_angles[1] = pc_snapshot.pc_target_motor_angles[1];
    target_motor_angles[2] = pc_snapshot.pc_target_motor_angles[2];
    target_motor_angles[3] = pc_snapshot.pc_target_motor_angles[3];
    target_motor_angles[4] = pc_snapshot.pc_target_motor_angles[4];
    target_motor_angles[5] = pc_snapshot.pc_target_motor_angles[5];
    target_motor_velocities[0] = pc_snapshot.pc_target_motor_velocities[0];
    target_motor_velocities[1] = pc_snapshot.pc_target_motor_velocities[1];
    target_motor_velocities[2] = pc_snapshot.pc_target_motor_velocities[2];
    target_motor_velocities[3] = pc_snapshot.pc_target_motor_velocities[3];
    target_motor_velocities[4] = pc_snapshot.pc_target_motor_velocities[4];
    target_motor_velocities[5] = pc_snapshot.pc_target_motor_velocities[5];
    Arm_SetPcTargetAngles(target_motor_angles, target_motor_velocities, HAL_GetTick());
}

void PC_Motor_Command_Updata(void)
{
    PcMotorCtrl_t command;

    PC_Motor_Command_DebugRefresh();

    if (UART_Protocol_GetMotorCtrlCommand(&command) == 0U)
    {
        return;
    }

    if (command.command > PC_MOTOR_CTRL_COMMAND_SAVE_ZERO)
    {
        return;
    }

    if (command.target_type == PC_MOTOR_CTRL_TARGET_ARM)
    {
        if (command.motor_index >= ARM_LOGICAL_MOTOR_COUNT)
        {
            return;
        }

        pc_arm_motor_enable_state_debug[command.motor_index] = command.command;
        pc_arm_motor_last_index_debug = command.motor_index;
        pc_arm_motor_last_enable_state_debug = command.command;
        pc_arm_motor_command_count_debug++;
        PC_Motor_Command_DebugSetHandled(&command);
        PC_Motor_Command_DebugRefresh();

        if (Arm_Motor_Disable_Updata() != 0U)
        {
            return;
        }

        if (command.command == PC_MOTOR_CTRL_COMMAND_ENABLE)
        {
            (void)Arm_EnableMotorByIndex(command.motor_index);
        }
        else if (command.command == PC_MOTOR_CTRL_COMMAND_SAVE_ZERO)
        {
            (void)Arm_SaveMotorZeroByIndex(command.motor_index);
        }
        else
        {
            (void)Arm_DisableMotorByIndex(command.motor_index);
        }

        return;
    }

    if (command.target_type == PC_MOTOR_CTRL_TARGET_HEAD)
    {
        if (command.motor_index >= 2U)
        {
            return;
        }

        pc_head_motor_enable_state_debug[command.motor_index] = command.command;
        pc_head_motor_last_index_debug = command.motor_index;
        pc_head_motor_last_enable_state_debug = command.command;
        pc_head_motor_command_count_debug++;
        PC_Motor_Command_DebugSetHandled(&command);
        PC_Motor_Command_DebugRefresh();

        if (command.command == PC_MOTOR_CTRL_COMMAND_ENABLE)
        {
            (void)Head_EnableMotorByIndex(command.motor_index);
        }
        else if (command.command == PC_MOTOR_CTRL_COMMAND_SAVE_ZERO)
        {
            (void)Head_SaveMotorZeroByIndex(command.motor_index);
        }
        else
        {
            (void)Head_DisableMotorByIndex(command.motor_index);
        }
    }
}

void pc_up_tx_data(void)
{
    up_tx_data.chassis_vx = x;
    up_tx_data.chassis_vy = y;
    up_tx_data.chassis_yaw = w;
    up_tx_data.wheel_speed_lf = mecanum_debug.fb_speed_lf;
    up_tx_data.wheel_speed_rf = mecanum_debug.fb_speed_rf;
    up_tx_data.wheel_speed_rb = mecanum_debug.fb_speed_rb;
    up_tx_data.wheel_speed_lb = mecanum_debug.fb_speed_lb;
    Chassis_LiveWatch_Refresh();
}

void pc_arm_tx_data(void)
{
    Rs485LiftStatus_t rs485_lift_status;

    up_tx_data.arm_motor_angle_1 = Arm_WrapAngleToPi(Linzu_motor_data[0].current_angle);
    up_tx_data.arm_motor_angle_2 = Arm_WrapAngleToPi(Linzu_motor_data[1].current_angle);
    up_tx_data.arm_motor_angle_3 = Arm_WrapAngleToPi(Linzu_motor_data[2].current_angle);

    up_tx_data.arm_motor_angle_4 = Damiao_motor_data[0].current_angle;
    up_tx_data.arm_motor_angle_5 = Damiao_motor_data[1].current_angle;
    up_tx_data.arm_motor_angle_6 = Damiao_motor_data[2].current_angle;
    if (Rs485Lift_CopyStatus(&rs485_lift_status) != 0U)
    {
        up_tx_data.lift_height = rs485_lift_status.position_mm;
    }
    else
    {
        up_tx_data.lift_height = lift_height_final;
    }

    head_pc_tx_source_deg_debug[0] = head_motor_data[0].current_angle;
    head_pc_tx_source_deg_debug[1] = head_motor_data[1].current_angle;
    if (head_degree_is_invalid(head_pc_tx_source_deg_debug[0]) != 0U)
    {
        head_pc_tx_invalid_count_debug[0]++;
    }
    if (head_degree_is_invalid(head_pc_tx_source_deg_debug[1]) != 0U)
    {
        head_pc_tx_invalid_count_debug[1]++;
    }

    head_pc_tx_normalized_deg_debug[0] = head_normalize_degree_for_pc_tx(head_pc_tx_source_deg_debug[0]);
    head_pc_tx_normalized_deg_debug[1] = head_normalize_degree_for_pc_tx(head_pc_tx_source_deg_debug[1]);
    head_pc_tx_rad_debug[0] = head_degree_to_pc_radian(head_pc_tx_source_deg_debug[0]);
    head_pc_tx_rad_debug[1] = head_degree_to_pc_radian(head_pc_tx_source_deg_debug[1]);
    up_tx_data.head_motor_angle_1 = head_pc_tx_rad_debug[0];
    up_tx_data.head_motor_angle_2 = head_pc_tx_rad_debug[1];
    up_tx_data.air_path_state =pump_state ;
    up_tx_data.suck_state = Check_Liquid_Sucked();
}
