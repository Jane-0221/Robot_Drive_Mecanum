#include "arm_sv.h"
#include "bsp_pca9685.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/*
 * ======================== 常用参数说明区 ========================
 * 用户动作的角度、时间、速度统一在 arm_sv_user_config.h 中使用整数修改。
 * 本文件内部仍使用弧度计算：角度(°) * PI / 180 = 弧度(rad)。
 * 不要通过修改 PWM 范围来调整普通动作的角度或速度。
 * 修改机械零位时只调整 servo_calibration[] 中对应舵机的零位占空比。
 * ================================================================
 */
#define ARM_SV_PI 3.14159265358979323846f
/* 角度制接口使用的内部换算系数，应用代码不需要再手工换算弧度。 */
#define ARM_SV_DEG_TO_RAD_SCALE (ARM_SV_PI / 180.0f)

/* PCA9685 在 50 Hz 下的全局安全占空比边界；不是每个舵机的机械零位。 */
#define ARM_SV_MIN_DUTY 0.025f
#define ARM_SV_CENTER_DUTY 0.075f
#define ARM_SV_MAX_DUTY 0.125f
#define ARM_SV_INACTIVE_DUTY 0.0f
#define ARM_SV_DUTY_SPAN (ARM_SV_MAX_DUTY - ARM_SV_CENTER_DUTY)
/* 270°舵机从中心到 135°使用完整半量程，因此 90°使用半量程的 2/3。 */
#define ARM_SV_90_DEG_DUTY_SPAN (ARM_SV_DUTY_SPAN * 2.0f / 3.0f)

/* 通用接口允许 ±135°；当前 1~4 号姿态接口按校准结果限制为 ±90°。 */
#define ARM_SV_MAX_RADIAN (ARM_SV_PI * 3.0f / 4.0f)
#define ARM_SV_MIN_RADIAN (-ARM_SV_MAX_RADIAN)
#define ARM_SV_POSE_MAX_RADIAN (ARM_SV_PI / 2.0f)
#define ARM_SV_POSE_MIN_RADIAN (-ARM_SV_POSE_MAX_RADIAN)

/* 5 ms 控制周期，异常情况下单次积分时间最多按 20 ms 计算。 */
#define ARM_SV_CONTROL_DT_S 0.005f
#define ARM_SV_MAX_DT_S 0.020f
/* 轨迹硬约束：规划器会自动延长时间，确保不超过最大速度和最大加速度。 */
#define ARM_SV_MAX_SPEED_RAD_S ARM_SV_RAMP_SPEED
#define ARM_SV_MAX_ACCEL_RAD_S2                                           \
    ((float)ARM_SV_USER_MAX_ACCEL_DEG_S2 * ARM_SV_DEG_TO_RAD_SCALE)
#define ARM_SV_POSITION_EPSILON_RAD 0.0015f
#define ARM_SV_VELOCITY_EPSILON_RAD_S 0.001f
#define ARM_SV_TARGET_CHANGE_EPSILON_RAD 0.00001f
#define ARM_SV_PWM_FRAME_PERIOD_S 0.020f
#define ARM_SV_PWM_KEEPALIVE_PERIOD_S 1.0f
#define ARM_SV_PWM_DUTY_EPSILON (0.5f / 4096.0f)
#define ARM_SV_TARGET_SNAPSHOT_ATTEMPTS 4U
#define ARM_SV_CONTROL_DEADLINE_MS 5U

#define ARM_SV_TRAJECTORY_COEFFICIENT_COUNT 6U
#define ARM_SV_TRAJECTORY_PLAN_SAMPLES 64U
#define ARM_SV_TRAJECTORY_PLAN_ITERATIONS 12U
#define ARM_SV_TRAJECTORY_MIN_DURATION_S 0.25f
#define ARM_SV_TRAJECTORY_MAX_DURATION_S 30.0f
#define ARM_SV_TRAJECTORY_LIMIT_MARGIN 1.02f
#define ARM_SV_MINIMUM_JERK_PEAK_VELOCITY 1.875f
#define ARM_SV_MINIMUM_JERK_PEAK_ACCEL 5.773503f
#define ARM_SV_ACCELERATION_EPSILON_RAD_S2 0.01f

/*
 * 运动中更换姿态时先执行同步制动段，再从静止状态进入新姿态。
 * 制动曲线使用连续的位置、速度和加速度，并限制加加速度，避免直接重规划越限。
 */
#define ARM_SV_POSE_BRAKE_MIN_DURATION_S 0.25f
#define ARM_SV_POSE_BRAKE_MAX_DURATION_S 2.0f
#define ARM_SV_POSE_BRAKE_SEARCH_STEPS 70U
#define ARM_SV_POSE_MAX_JERK_RAD_S3 (8.0f * ARM_SV_MAX_ACCEL_RAD_S2)
#define ARM_SV_POLYNOMIAL_EPSILON 0.000000001f

#define ARM_SV_PCA_FRAME_FIRST_CHANNEL 0U
#define ARM_SV_PCA_FRAME_CHANNEL_COUNT 6U
#define ARM_SV_DISABLED_PCA_CHANNEL 4U

#define ARM_SV_SIMPLE_MOTION_ENABLE 1U
#if (ARM_SV_USER_ACTION_ENABLE != 0)
#define ARM_SV_INITIAL_CONTROL_MODE ARM_SV_CONTROL_MODE_USER_ACTION
#else
#define ARM_SV_INITIAL_CONTROL_MODE ARM_SV_CONTROL_MODE_EXTERNAL_RAMP
#endif
/*
 * 每条同步姿态轨迹的最短时间来自用户配置中的MOVE_TIME_MS。
 * 实际时间仍受最大速度/加速度限制，必要时规划器会自动延长。
 */
#define ARM_SV_POSE_MIN_DURATION_S                                        \
    ((float)ARM_SV_USER_MOVE_TIME_MS * 0.001f)

/* 在线微调机械零位时使用 6 秒平滑过渡；它不用于正常姿态动作。 */
#define ARM_SV_ZERO_TRIM_COUNT ARM_SV_POSE_COUNT
#define ARM_SV_ZERO_TRIM_MOVE_MS 6000U
#define ARM_SV_ZERO_TRIM_CHANGE_EPSILON_DUTY 0.0000001f

typedef struct
{
    uint8_t pca_channel;
    float min_radian;
    float max_radian;
    float duty_at_min_radian;
    float duty_at_zero_radian;
    float duty_at_max_radian;
} ARM_SV_Calibration_t;

/* 自动动作状态；调试变量中可直接读取当前状态值。 */
typedef enum
{
    ARM_SV_USER_ACTION_WAIT_START = 0U,
    ARM_SV_USER_ACTION_MOVING_FORWARD_GRAB = 1U,
    ARM_SV_USER_ACTION_HOLDING_GRAB = 2U,
    ARM_SV_USER_ACTION_MOVING_PUT_DOWN = 3U,
    ARM_SV_USER_ACTION_DONE = 4U,
    ARM_SV_USER_ACTION_ABORTED = 5U
} ARM_SV_UserActionState_t;

typedef enum
{
    ARM_SV_POSE_MOTION_IDLE = 0U,
    ARM_SV_POSE_MOTION_MOVING = 1U,
    ARM_SV_POSE_MOTION_BRAKING = 2U
} ARM_SV_PoseMotionState_t;

#if (ARM_SV_USER_ACTION_ENABLE != 0)
#if (ARM_SV_USER_START_DELAY_MS < 0)
#error "ARM_SV_USER_START_DELAY_MS cannot be negative"
#endif
#if (ARM_SV_USER_MOVE_TIME_MS <= 0)
#error "ARM_SV_USER_MOVE_TIME_MS must be greater than zero"
#endif
#if (ARM_SV_USER_GRAB_HOLD_MS < 0)
#error "ARM_SV_USER_GRAB_HOLD_MS cannot be negative"
#endif
#if (ARM_SV_USER_MAX_SPEED_DEG_S <= 0)
#error "ARM_SV_USER_MAX_SPEED_DEG_S must be greater than zero"
#endif
#if (ARM_SV_USER_MAX_ACCEL_DEG_S2 <= 0)
#error "ARM_SV_USER_MAX_ACCEL_DEG_S2 must be greater than zero"
#endif
#if ((ARM_SV_USER_SERVO1_FORWARD_GRAB_DEG < -90) ||                  \
     (ARM_SV_USER_SERVO1_FORWARD_GRAB_DEG > 90) ||                   \
     (ARM_SV_USER_SERVO2_FORWARD_GRAB_DEG < -90) ||                  \
     (ARM_SV_USER_SERVO2_FORWARD_GRAB_DEG > 90) ||                   \
     (ARM_SV_USER_SERVO3_FORWARD_GRAB_DEG < -90) ||                  \
     (ARM_SV_USER_SERVO3_FORWARD_GRAB_DEG > 90) ||                   \
     (ARM_SV_USER_SERVO4_FORWARD_GRAB_DEG < -90) ||                  \
     (ARM_SV_USER_SERVO4_FORWARD_GRAB_DEG > 90) ||                   \
     (ARM_SV_USER_SERVO1_PUT_DOWN_DEG < -90) ||                      \
     (ARM_SV_USER_SERVO1_PUT_DOWN_DEG > 90) ||                       \
     (ARM_SV_USER_SERVO2_PUT_DOWN_DEG < -90) ||                      \
     (ARM_SV_USER_SERVO2_PUT_DOWN_DEG > 90) ||                       \
     (ARM_SV_USER_SERVO3_PUT_DOWN_DEG < -90) ||                      \
     (ARM_SV_USER_SERVO3_PUT_DOWN_DEG > 90) ||                       \
     (ARM_SV_USER_SERVO4_PUT_DOWN_DEG < -90) ||                      \
     (ARM_SV_USER_SERVO4_PUT_DOWN_DEG > 90))
#error "Forward-grab and put-down angles must stay inside -90 to +90 degrees"
#endif
#endif

/*
 * 单个校准项字段顺序：
 * {PCA通道, 最小角度, 最大角度, 最小角PWM, 0°零位PWM, 最大角PWM}。
 * 下表顺序依次为实体 1、2、3、4、6 号舵机。前四项是已经现场校准的
 * ±90°关节；最后一项是实体 6 号舵机，暂不参加当前 4 关节姿态动作。
 * 只应填写实测值。修改零位时改中间的 duty_at_zero_radian，且要同步
 * 检查两端 PWM，确保始终位于 ARM_SV_MIN_DUTY~ARM_SV_MAX_DUTY 范围内。
 */
static const ARM_SV_Calibration_t servo_calibration[ARM_SV_ACTIVE_COUNT] = {
    /* 实体1号 -> PCA9685通道0，零位占空比0.0731 */
    {0U, ARM_SV_POSE_MIN_RADIAN, ARM_SV_POSE_MAX_RADIAN,
     0.0731f - ARM_SV_90_DEG_DUTY_SPAN, 0.0731f,
     0.0731f + ARM_SV_90_DEG_DUTY_SPAN},
    /* 实体2号 -> PCA9685通道1，零位占空比0.0767 */
    {1U, ARM_SV_POSE_MIN_RADIAN, ARM_SV_POSE_MAX_RADIAN,
     0.0767f - ARM_SV_90_DEG_DUTY_SPAN, 0.0767f,
     0.0767f + ARM_SV_90_DEG_DUTY_SPAN},
    /* 实体3号 -> PCA9685通道2，零位占空比0.0750 */
    {2U, ARM_SV_POSE_MIN_RADIAN, ARM_SV_POSE_MAX_RADIAN,
     0.0750f - ARM_SV_90_DEG_DUTY_SPAN, 0.0750f,
     0.0750f + ARM_SV_90_DEG_DUTY_SPAN},
    /* 实体4号 -> PCA9685通道3，零位占空比0.0739 */
    {3U, ARM_SV_POSE_MIN_RADIAN, ARM_SV_POSE_MAX_RADIAN,
     0.0739f - ARM_SV_90_DEG_DUTY_SPAN, 0.0739f,
     0.0739f + ARM_SV_90_DEG_DUTY_SPAN},
    /* 实体6号 -> PCA9685通道5；当前保持0°，不参与1~4号姿态联动。 */
    {5U, ARM_SV_MIN_RADIAN, ARM_SV_MAX_RADIAN, 0.025f, 0.075f, 0.125f}
};

#if (ARM_SV_USER_ACTION_ENABLE != 0)
/*
 * 本次自动动作启动于放下姿态，避免烧录复位时先跳到中间位置。
 * 这里只选择启动姿态，不改变任何舵机的校准0位。
 */
static const float arm_sv_reset_pose[ARM_SV_ACTIVE_COUNT] = {
    (float)ARM_SV_USER_SERVO1_PUT_DOWN_DEG * ARM_SV_DEG_TO_RAD_SCALE,
    (float)ARM_SV_USER_SERVO2_PUT_DOWN_DEG * ARM_SV_DEG_TO_RAD_SCALE,
    (float)ARM_SV_USER_SERVO3_PUT_DOWN_DEG * ARM_SV_DEG_TO_RAD_SCALE,
    (float)ARM_SV_USER_SERVO4_PUT_DOWN_DEG * ARM_SV_DEG_TO_RAD_SCALE,
    0.0f
};
#else
/* 关闭自动动作时仍使用所有已启用舵机的校准0位。 */
static const float arm_sv_reset_pose[ARM_SV_ACTIVE_COUNT] = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};
#endif

#if (ARM_SV_USER_ACTION_ENABLE != 0)
/* 本次自动测试只使用已经校准并参与姿态轨迹的实体1~4号舵机。 */
static const int16_t arm_sv_user_forward_grab_pose_deg[ARM_SV_POSE_COUNT] = {
    ARM_SV_USER_SERVO1_FORWARD_GRAB_DEG,
    ARM_SV_USER_SERVO2_FORWARD_GRAB_DEG,
    ARM_SV_USER_SERVO3_FORWARD_GRAB_DEG,
    ARM_SV_USER_SERVO4_FORWARD_GRAB_DEG
};
static const int16_t arm_sv_user_put_down_pose_deg[ARM_SV_POSE_COUNT] = {
    ARM_SV_USER_SERVO1_PUT_DOWN_DEG,
    ARM_SV_USER_SERVO2_PUT_DOWN_DEG,
    ARM_SV_USER_SERVO3_PUT_DOWN_DEG,
    ARM_SV_USER_SERVO4_PUT_DOWN_DEG
};
#endif

volatile float motor_radians[ARM_SV_COUNT] = {0.0f};

ARM_SV_Duties_t duties_rx = {0.0f};
ARM_SV_Duties_t duties_tx = {0.0f};

volatile float arm_sv_target_radian_debug[ARM_SV_COUNT] = {0.0f};
volatile float arm_sv_position_radian_debug[ARM_SV_COUNT] = {0.0f};
volatile float arm_sv_velocity_radian_debug[ARM_SV_COUNT] = {0.0f};
volatile float arm_sv_duty_debug[ARM_SV_COUNT] = {0.0f};
volatile uint32_t arm_sv_target_sequence_debug = 0U;
volatile uint32_t arm_sv_i2c_write_count_debug = 0U;
volatile uint32_t arm_sv_i2c_error_count_debug = 0U;
volatile uint32_t arm_sv_update_count_debug = 0U;
volatile uint32_t arm_sv_last_update_interval_ms_debug = 0U;
volatile uint32_t arm_sv_max_update_interval_ms_debug = 0U;
volatile uint32_t arm_sv_deadline_miss_count_debug = 0U;
volatile uint32_t arm_sv_last_pwm_write_duration_ms_debug = 0U;
volatile uint32_t arm_sv_max_pwm_write_duration_ms_debug = 0U;
volatile uint32_t arm_sv_pwm_frame_inflight_debug = 0U;
volatile uint32_t arm_sv_pwm_retry_count_debug = 0U;
volatile uint32_t arm_sv_pwm_submit_busy_count_debug = 0U;
volatile uint32_t arm_sv_trajectory_active_debug = 0U;
volatile uint32_t arm_sv_trajectory_duration_ms_debug = 0U;
volatile uint32_t arm_sv_trajectory_replan_count_debug = 0U;
volatile uint32_t arm_sv_limit_clamp_count_debug = 0U;
volatile uint32_t arm_sv_simple_motion_active_debug = 0U;
volatile uint32_t arm_sv_simple_motion_step_debug = 0U;
volatile uint32_t arm_sv_simple_motion_servo_debug = 0U;
volatile uint32_t arm_sv_simple_motion_phase_debug = 0U;
volatile uint32_t arm_sv_simple_motion_done_debug = 0U;
volatile float arm_sv_zero_trim_target_duty[ARM_SV_COUNT] = {0.0f};
volatile float arm_sv_zero_trim_current_duty_debug[ARM_SV_COUNT] = {0.0f};
volatile uint32_t arm_sv_zero_trim_active_mask_debug = 0U;
volatile uint32_t arm_sv_zero_trim_update_count_debug = 0U;
volatile uint32_t arm_sv_zero_trim_reject_count_debug = 0U;
/*
 * DAP 可在线写入下面的姿态目标，再改变 command_sequence 提交命令。
 * 这些变量位于 RAM，复位或重新上电后会被 ARM_SV_Init() 清零。
 * 固化动作必须在固件状态机中调用 ARM_SV_SetPoseTargets()。
 */
volatile float arm_sv_pose_target_radian[ARM_SV_COUNT] = {0.0f};
volatile uint32_t arm_sv_pose_command_sequence = 0U;
volatile uint32_t arm_sv_pose_applied_sequence_debug = 0U;
volatile uint32_t arm_sv_pose_reject_count_debug = 0U;
volatile uint32_t arm_sv_pose_duration_ms_debug = 0U;
/* 姿态状态：0停止，1向目标运动，2正在为新命令同步制动。 */
volatile uint32_t arm_sv_pose_motion_state_debug = 0U;
volatile uint32_t arm_sv_pose_interrupt_count_debug = 0U;
volatile uint32_t arm_sv_pose_brake_duration_ms_debug = 0U;
volatile uint32_t arm_sv_pose_brake_fallback_count_debug = 0U;
/* 自动测试状态：0等待、1向前抓取中、2抓取保持、3放下中、4完成、5中止。 */
volatile uint32_t arm_sv_user_action_state_debug = 0U;
volatile uint32_t arm_sv_user_action_step_debug = 0U;
volatile uint32_t arm_sv_user_action_elapsed_ms_debug = 0U;
volatile uint32_t arm_sv_user_action_done_debug = 0U;
/*
 * request可由接口或DAP写入；active只由5 ms舵机任务更新。
 * pending=1表示正在等待当前轨迹停稳后切换控制权。
 */
volatile uint32_t arm_sv_control_mode_request_debug =
    (uint32_t)ARM_SV_INITIAL_CONTROL_MODE;
volatile uint32_t arm_sv_control_mode_active_debug =
    (uint32_t)ARM_SV_INITIAL_CONTROL_MODE;
volatile uint32_t arm_sv_control_mode_pending_debug = 0U;
volatile uint32_t arm_sv_control_mode_switch_count_debug = 0U;
volatile uint32_t arm_sv_control_command_reject_count_debug = 0U;
volatile uint32_t arm_sv_pca_fault_hold_active_debug = 0U;
volatile uint32_t arm_sv_pca_fault_hold_total_ms_debug = 0U;
volatile uint32_t arm_sv_pca_recovery_resync_count_debug = 0U;

static float current_duties[ARM_SV_COUNT] = {0.0f};
static float pending_duties[ARM_SV_COUNT] = {0.0f};
static float inflight_duties[ARM_SV_COUNT] = {0.0f};
static uint8_t arm_sv_pwm_frame_inflight = 0U;
static uint8_t arm_sv_pwm_retry_pending = 0U;
static float servo_position_radian[ARM_SV_COUNT] = {0.0f};
static float servo_velocity_radian_s[ARM_SV_COUNT] = {0.0f};
static float servo_acceleration_radian_s2[ARM_SV_COUNT] = {0.0f};
static float target_snapshot[ARM_SV_COUNT] = {0.0f};
static uint32_t target_snapshot_sequence = 0U;
static uint32_t arm_sv_last_tick_ms = 0U;
static uint8_t arm_sv_timing_initialized = 0U;
static float arm_sv_pwm_frame_timer_s = 0.0f;
static float arm_sv_pwm_keepalive_timer_s = 0.0f;
static float arm_sv_trajectory_coefficients[ARM_SV_ACTIVE_COUNT]
                                               [ARM_SV_TRAJECTORY_COEFFICIENT_COUNT] = {{0.0f}};
static float arm_sv_trajectory_target_radian[ARM_SV_ACTIVE_COUNT] = {0.0f};
static float arm_sv_trajectory_duration_s = 0.0f;
static uint32_t arm_sv_trajectory_start_tick_ms = 0U;
static uint32_t arm_sv_trajectory_target_sequence = 0U;
static uint8_t arm_sv_trajectory_active = 0U;
static float arm_sv_simple_motion_targets[ARM_SV_COUNT] = {0.0f};
static float arm_sv_simple_motion_velocities[ARM_SV_COUNT] = {0.0f};
static float arm_sv_simple_motion_accelerations[ARM_SV_COUNT] = {0.0f};
static uint32_t arm_sv_simple_motion_start_tick_ms = 0U;
static float arm_sv_pose_start_radian[ARM_SV_POSE_COUNT] = {0.0f};
static float arm_sv_pose_latched_target_radian[ARM_SV_POSE_COUNT] = {0.0f};
static float arm_sv_pose_pending_target_radian[ARM_SV_POSE_COUNT] = {0.0f};
static float arm_sv_pose_brake_coefficients[ARM_SV_POSE_COUNT]
                                               [ARM_SV_TRAJECTORY_COEFFICIENT_COUNT] = {{0.0f}};
static float arm_sv_pose_duration_s = ARM_SV_POSE_MIN_DURATION_S;
static uint32_t arm_sv_pose_applied_sequence = 0U;
static uint8_t arm_sv_pose_trajectory_active = 0U;
static uint8_t arm_sv_pose_pending_target_valid = 0U;
static ARM_SV_PoseMotionState_t arm_sv_pose_motion_state =
    ARM_SV_POSE_MOTION_IDLE;
static ARM_SV_UserActionState_t arm_sv_user_action_state =
    ARM_SV_USER_ACTION_WAIT_START;
static uint32_t arm_sv_user_action_start_tick_ms = 0U;
static uint32_t arm_sv_user_action_hold_start_tick_ms = 0U;
static uint32_t arm_sv_user_action_expected_sequence = 0U;
static float arm_sv_zero_trim_current_duty[ARM_SV_ZERO_TRIM_COUNT] = {0.0f};
static float arm_sv_zero_trim_start_duty[ARM_SV_ZERO_TRIM_COUNT] = {0.0f};
static float arm_sv_zero_trim_latched_target_duty[ARM_SV_ZERO_TRIM_COUNT] = {0.0f};
static uint32_t arm_sv_zero_trim_start_tick_ms[ARM_SV_ZERO_TRIM_COUNT] = {0U};
static uint8_t arm_sv_zero_trim_active[ARM_SV_ZERO_TRIM_COUNT] = {0U};
static uint8_t arm_sv_pca_fault_hold_active = 0U;
static uint32_t arm_sv_pca_fault_hold_last_tick_ms = 0U;

static uint8_t submit_pose_targets(const float radians[ARM_SV_POSE_COUNT],
                                   uint8_t user_action_owner);

static void shift_control_timeline(uint32_t elapsed_ms)
{
    if (elapsed_ms == 0U)
    {
        return;
    }

    if (arm_sv_pose_motion_state != ARM_SV_POSE_MOTION_IDLE)
    {
        arm_sv_simple_motion_start_tick_ms += elapsed_ms;
    }
    if (arm_sv_trajectory_active != 0U)
    {
        arm_sv_trajectory_start_tick_ms += elapsed_ms;
    }
    if ((arm_sv_user_action_state != ARM_SV_USER_ACTION_DONE) &&
        (arm_sv_user_action_state != ARM_SV_USER_ACTION_ABORTED))
    {
        arm_sv_user_action_start_tick_ms += elapsed_ms;
        if (arm_sv_user_action_hold_start_tick_ms != 0U)
        {
            arm_sv_user_action_hold_start_tick_ms += elapsed_ms;
        }
    }
    for (uint8_t i = 0U; i < ARM_SV_ZERO_TRIM_COUNT; i++)
    {
        if (arm_sv_zero_trim_active[i] != 0U)
        {
            arm_sv_zero_trim_start_tick_ms[i] += elapsed_ms;
        }
    }

    if (arm_sv_pca_fault_hold_total_ms_debug <= UINT32_MAX - elapsed_ms)
    {
        arm_sv_pca_fault_hold_total_ms_debug += elapsed_ms;
    }
    else
    {
        arm_sv_pca_fault_hold_total_ms_debug = UINT32_MAX;
    }
}

static void update_pca_fault_hold(uint32_t now_ms)
{
    uint8_t communication_degraded =
        ((pca9685_ready_debug == 0U) ||
         (pca9685_consecutive_error_count_debug != 0U))
            ? 1U
            : 0U;

    if (arm_sv_pca_fault_hold_active == 0U)
    {
        if (communication_degraded == 0U)
        {
            return;
        }

        arm_sv_pca_fault_hold_active = 1U;
        arm_sv_pca_fault_hold_active_debug = 1U;
        /*
         * 通信结果在本周期开始时才被确认，因此把上一个控制周期也计入冻结时间，
         * 避免恢复后轨迹比舵机真实位置提前一个5 ms步长。
         */
        shift_control_timeline(arm_sv_last_update_interval_ms_debug);
        arm_sv_pca_fault_hold_last_tick_ms = now_ms;
        return;
    }

    shift_control_timeline(now_ms - arm_sv_pca_fault_hold_last_tick_ms);
    arm_sv_pca_fault_hold_last_tick_ms = now_ms;
    if (communication_degraded == 0U)
    {
        arm_sv_pca_fault_hold_active = 0U;
        arm_sv_pca_fault_hold_active_debug = 0U;
        if (arm_sv_pca_recovery_resync_count_debug < UINT32_MAX)
        {
            arm_sv_pca_recovery_resync_count_debug++;
        }
    }

}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint8_t control_mode_is_valid(uint32_t mode)
{
    if (mode > (uint32_t)ARM_SV_CONTROL_MODE_HOLD)
    {
        return 0U;
    }
#if (ARM_SV_USER_ACTION_ENABLE == 0)
    if (mode == (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION)
    {
        return 0U;
    }
#endif
    return 1U;
}

static uint8_t control_mode_uses_pose(uint32_t mode)
{
    return ((mode == (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION) ||
            (mode == (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_POSE))
               ? 1U
               : 0U;
}

static void record_control_command_reject(void)
{
    uint32_t previous_primask = __get_PRIMASK();

    __disable_irq();
    if (arm_sv_control_command_reject_count_debug < UINT32_MAX)
    {
        arm_sv_control_command_reject_count_debug++;
    }
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static float sanitize_servo_target(uint8_t id, float value, float fallback)
{
    if ((id >= ARM_SV_ACTIVE_COUNT) || (!isfinite(value)))
    {
        return fallback;
    }
    return clamp_float(value,
                       servo_calibration[id].min_radian,
                       servo_calibration[id].max_radian);
}

static void set_duty_member(ARM_SV_Duties_t *duties, uint8_t id, float duty)
{
    if (duties == NULL)
    {
        return;
    }

    switch (id)
    {
    case 0U:
        duties->duty0 = duty;
        break;
    case 1U:
        duties->duty1 = duty;
        break;
    case 2U:
        duties->duty2 = duty;
        break;
    case 3U:
        duties->duty3 = duty;
        break;
    case 4U:
        duties->duty4 = duty;
        break;
    case 5U:
        duties->duty5 = duty;
        break;
    default:
        break;
    }
}

static void build_pca_duty_frame(
    const float logical_duties[ARM_SV_COUNT],
    float pca_duties[ARM_SV_PCA_FRAME_CHANNEL_COUNT])
{
    for (uint8_t channel = 0U; channel < ARM_SV_PCA_FRAME_CHANNEL_COUNT; channel++)
    {
        pca_duties[channel] = 0.0f;
    }

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        pca_duties[servo_calibration[i].pca_channel] = logical_duties[i];
    }

    pca_duties[ARM_SV_DISABLED_PCA_CHANNEL] = 0.0f;
}

static void interpolate_minimum_jerk(float start,
                                     float end,
                                     float phase,
                                     float duration_s,
                                     float *position,
                                     float *velocity,
                                     float *acceleration)
{
    float one_minus_phase;
    float phase_squared;
    float blend;
    float blend_derivative;
    float blend_second_derivative;

    phase = clamp_float(phase, 0.0f, 1.0f);
    one_minus_phase = 1.0f - phase;
    phase_squared = phase * phase;
    blend = phase_squared * phase *
            (10.0f + phase * (-15.0f + 6.0f * phase));
    blend_derivative = 30.0f * phase_squared *
                       one_minus_phase * one_minus_phase;
    blend_second_derivative = 60.0f * phase * one_minus_phase *
                              (1.0f - 2.0f * phase);

    *position = start + (end - start) * blend;
    *velocity = (end - start) * blend_derivative / duration_s;
    *acceleration = (end - start) * blend_second_derivative /
                    (duration_s * duration_s);
}

static void update_zero_trim(uint32_t now_ms)
{
    uint32_t active_mask = 0U;

    for (uint8_t i = 0U; i < ARM_SV_ZERO_TRIM_COUNT; i++)
    {
        const ARM_SV_Calibration_t *calibration = &servo_calibration[i];
        float min_trim = ARM_SV_MIN_DUTY - calibration->duty_at_min_radian;
        float max_trim = ARM_SV_MAX_DUTY - calibration->duty_at_max_radian;
        float requested_trim = arm_sv_zero_trim_target_duty[i];

        if (!isfinite(requested_trim))
        {
            requested_trim = arm_sv_zero_trim_latched_target_duty[i];
            arm_sv_zero_trim_target_duty[i] = requested_trim;
            arm_sv_zero_trim_reject_count_debug++;
        }
        else
        {
            float clamped_trim = clamp_float(requested_trim, min_trim, max_trim);

            if (clamped_trim != requested_trim)
            {
                requested_trim = clamped_trim;
                arm_sv_zero_trim_target_duty[i] = clamped_trim;
                arm_sv_zero_trim_reject_count_debug++;
            }
        }

        if (fabsf(requested_trim - arm_sv_zero_trim_latched_target_duty[i]) >=
            ARM_SV_ZERO_TRIM_CHANGE_EPSILON_DUTY)
        {
            arm_sv_zero_trim_start_duty[i] = arm_sv_zero_trim_current_duty[i];
            arm_sv_zero_trim_latched_target_duty[i] = requested_trim;
            arm_sv_zero_trim_start_tick_ms[i] = now_ms;
            arm_sv_zero_trim_active[i] = 1U;
            arm_sv_zero_trim_update_count_debug++;
        }

        if (arm_sv_zero_trim_active[i] != 0U)
        {
            uint32_t elapsed_ms = now_ms - arm_sv_zero_trim_start_tick_ms[i];

            if (elapsed_ms >= ARM_SV_ZERO_TRIM_MOVE_MS)
            {
                arm_sv_zero_trim_current_duty[i] =
                    arm_sv_zero_trim_latched_target_duty[i];
                arm_sv_zero_trim_active[i] = 0U;
            }
            else
            {
                float phase = (float)elapsed_ms / (float)ARM_SV_ZERO_TRIM_MOVE_MS;
                float velocity;
                float acceleration;

                interpolate_minimum_jerk(arm_sv_zero_trim_start_duty[i],
                                         arm_sv_zero_trim_latched_target_duty[i],
                                         phase,
                                         (float)ARM_SV_ZERO_TRIM_MOVE_MS * 0.001f,
                                         &arm_sv_zero_trim_current_duty[i],
                                         &velocity,
                                         &acceleration);
            }
        }

        if (arm_sv_zero_trim_active[i] != 0U)
        {
            active_mask |= (1UL << i);
        }
        arm_sv_zero_trim_current_duty_debug[i] =
            arm_sv_zero_trim_current_duty[i];
    }

    for (uint8_t i = ARM_SV_ZERO_TRIM_COUNT; i < ARM_SV_COUNT; i++)
    {
        arm_sv_zero_trim_target_duty[i] = 0.0f;
        arm_sv_zero_trim_current_duty_debug[i] = 0.0f;
    }
    arm_sv_zero_trim_active_mask_debug = active_mask;
}

static float get_control_dt_s(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms;

    if (arm_sv_timing_initialized == 0U)
    {
        arm_sv_timing_initialized = 1U;
        arm_sv_last_tick_ms = now_ms;
        arm_sv_last_update_interval_ms_debug = 0U;
        return ARM_SV_CONTROL_DT_S;
    }

    elapsed_ms = now_ms - arm_sv_last_tick_ms;
    arm_sv_last_tick_ms = now_ms;
    arm_sv_last_update_interval_ms_debug = elapsed_ms;
    if (elapsed_ms > arm_sv_max_update_interval_ms_debug)
    {
        arm_sv_max_update_interval_ms_debug = elapsed_ms;
    }
    if (elapsed_ms > ARM_SV_CONTROL_DEADLINE_MS)
    {
        arm_sv_deadline_miss_count_debug++;
    }

    if (elapsed_ms == 0U)
    {
        return ARM_SV_CONTROL_DT_S;
    }
    if (elapsed_ms > (uint32_t)(ARM_SV_MAX_DT_S * 1000.0f))
    {
        return ARM_SV_MAX_DT_S;
    }
    return (float)elapsed_ms * 0.001f;
}

static uint8_t begin_target_write(uint32_t required_mode,
                                  uint32_t *previous_primask)
{
    if (previous_primask == NULL)
    {
        return 0U;
    }

    *previous_primask = __get_PRIMASK();
    __disable_irq();
    if (arm_sv_control_mode_request_debug != required_mode)
    {
        if ((*previous_primask & 1U) == 0U)
        {
            __enable_irq();
        }
        record_control_command_reject();
        return 0U;
    }

    arm_sv_target_sequence_debug++;
    __DMB();
    return 1U;
}

static void end_target_write(uint32_t previous_primask)
{
    __DMB();
    arm_sv_target_sequence_debug++;
    __DMB();
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static void copy_target_snapshot(void)
{
    float candidate[ARM_SV_ACTIVE_COUNT];

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        target_snapshot[i] = 0.0f;
    }

    for (uint8_t attempt = 0U; attempt < ARM_SV_TARGET_SNAPSHOT_ATTEMPTS; attempt++)
    {
        uint32_t sequence_before = arm_sv_target_sequence_debug;
        uint32_t sequence_after;

        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }

        __DMB();
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            candidate[i] = motor_radians[i];
        }
        __DMB();

        sequence_after = arm_sv_target_sequence_debug;
        if ((sequence_before == sequence_after) && ((sequence_after & 1U) == 0U))
        {
            for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
            {
                target_snapshot[i] = sanitize_servo_target(i,
                                                           candidate[i],
                                                           target_snapshot[i]);
            }
            target_snapshot_sequence = sequence_after;
            return;
        }
    }
}

static float servo_radian_to_duty(uint8_t id, float radian)
{
    const ARM_SV_Calibration_t *calibration;
    float trim_duty;
    float min_duty;
    float zero_duty;
    float max_duty;
    float duty;

    if ((id >= ARM_SV_ACTIVE_COUNT) || (!isfinite(radian)))
    {
        return ARM_SV_CENTER_DUTY;
    }

    calibration = &servo_calibration[id];
    trim_duty = (id < ARM_SV_ZERO_TRIM_COUNT)
                    ? arm_sv_zero_trim_current_duty[id]
                    : 0.0f;
    min_duty = calibration->duty_at_min_radian + trim_duty;
    zero_duty = calibration->duty_at_zero_radian + trim_duty;
    max_duty = calibration->duty_at_max_radian + trim_duty;
    radian = clamp_float(radian,
                         calibration->min_radian,
                         calibration->max_radian);
    if (radian >= 0.0f)
    {
        duty = zero_duty +
               (radian / calibration->max_radian) *
                   (max_duty - zero_duty);
    }
    else
    {
        duty = zero_duty +
               (radian / (-calibration->min_radian)) *
                   (zero_duty - min_duty);
    }

    return clamp_float(duty, ARM_SV_MIN_DUTY, ARM_SV_MAX_DUTY);
}

static void build_quintic_coefficients(float start_position,
                                        float start_velocity,
                                        float start_acceleration,
                                        float end_position,
                                        float duration_s,
                                        float coefficients[ARM_SV_TRAJECTORY_COEFFICIENT_COUNT])
{
    float duration_squared = duration_s * duration_s;
    float duration_cubed = duration_squared * duration_s;
    float duration_fourth = duration_cubed * duration_s;
    float duration_fifth = duration_fourth * duration_s;
    float displacement = end_position - start_position;

    coefficients[0] = start_position;
    coefficients[1] = start_velocity;
    coefficients[2] = 0.5f * start_acceleration;
    coefficients[3] = (20.0f * displacement -
                       12.0f * start_velocity * duration_s -
                       3.0f * start_acceleration * duration_squared) /
                      (2.0f * duration_cubed);
    coefficients[4] = (-30.0f * displacement +
                       16.0f * start_velocity * duration_s +
                       3.0f * start_acceleration * duration_squared) /
                      (2.0f * duration_fourth);
    coefficients[5] = (12.0f * displacement -
                       6.0f * start_velocity * duration_s -
                       start_acceleration * duration_squared) /
                      (2.0f * duration_fifth);
}

static void evaluate_quintic(const float coefficients[ARM_SV_TRAJECTORY_COEFFICIENT_COUNT],
                             float time_s,
                             float *position,
                             float *velocity,
                             float *acceleration)
{
    *position = coefficients[0] +
                time_s * (coefficients[1] +
                          time_s * (coefficients[2] +
                                    time_s * (coefficients[3] +
                                              time_s * (coefficients[4] +
                                                        time_s * coefficients[5]))));
    *velocity = coefficients[1] +
                time_s * (2.0f * coefficients[2] +
                          time_s * (3.0f * coefficients[3] +
                                    time_s * (4.0f * coefficients[4] +
                                              time_s * 5.0f * coefficients[5])));
    *acceleration = 2.0f * coefficients[2] +
                    time_s * (6.0f * coefficients[3] +
                              time_s * (12.0f * coefficients[4] +
                                        time_s * 20.0f * coefficients[5]));
}

static void set_pose_motion_state(ARM_SV_PoseMotionState_t state)
{
    arm_sv_pose_motion_state = state;
    arm_sv_pose_motion_state_debug = (uint32_t)state;
    arm_sv_pose_trajectory_active =
        (state == ARM_SV_POSE_MOTION_IDLE) ? 0U : 1U;
}

static uint8_t pose_targets_are_equal(
    const float first[ARM_SV_POSE_COUNT],
    const float second[ARM_SV_POSE_COUNT])
{
    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        if (fabsf(first[i] - second[i]) >= ARM_SV_TARGET_CHANGE_EPSILON_RAD)
        {
            return 0U;
        }
    }
    return 1U;
}

static void start_pose_move(
    uint32_t now_ms,
    const float target_radian[ARM_SV_POSE_COUNT])
{
    float duration_s = ARM_SV_POSE_MIN_DURATION_S;
    float max_distance = 0.0f;

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        float distance;
        float velocity_duration;
        float acceleration_duration;

        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_pose_start_radian[i] = arm_sv_simple_motion_targets[i];
        arm_sv_pose_latched_target_radian[i] = target_radian[i];
        distance = fabsf(target_radian[i] - arm_sv_pose_start_radian[i]);
        velocity_duration = ARM_SV_MINIMUM_JERK_PEAK_VELOCITY *
                            distance / ARM_SV_MAX_SPEED_RAD_S;
        acceleration_duration = sqrtf(ARM_SV_MINIMUM_JERK_PEAK_ACCEL *
                                      distance /
                                      ARM_SV_MAX_ACCEL_RAD_S2);

        if (distance > max_distance)
        {
            max_distance = distance;
        }
        if (velocity_duration > duration_s)
        {
            duration_s = velocity_duration;
        }
        if (acceleration_duration > duration_s)
        {
            duration_s = acceleration_duration;
        }
    }

    arm_sv_pose_pending_target_valid = 0U;
    arm_sv_pose_duration_s = clamp_float(duration_s,
                                         ARM_SV_POSE_MIN_DURATION_S,
                                         ARM_SV_TRAJECTORY_MAX_DURATION_S);
    arm_sv_simple_motion_start_tick_ms = now_ms;
    if (max_distance < ARM_SV_POSITION_EPSILON_RAD)
    {
        for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
        {
            arm_sv_simple_motion_targets[i] = target_radian[i];
        }
        set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
        arm_sv_pose_duration_ms_debug = 0U;
        return;
    }

    set_pose_motion_state(ARM_SV_POSE_MOTION_MOVING);
    arm_sv_pose_duration_ms_debug =
        (uint32_t)(arm_sv_pose_duration_s * 1000.0f + 0.5f);
}

static void build_pose_brake_coefficients(
    float duration_s,
    float coefficients[ARM_SV_POSE_COUNT]
                      [ARM_SV_TRAJECTORY_COEFFICIENT_COUNT])
{
    float duration_squared = duration_s * duration_s;
    float duration_cubed = duration_squared * duration_s;

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        float velocity = arm_sv_simple_motion_velocities[i];
        float acceleration = arm_sv_simple_motion_accelerations[i];

        /*
         * 速度使用三次多项式，从当前速度/加速度连续过渡到0；积分后得到
         * 四次位置曲线，因此停止点由当前运动状态自然确定，不会瞬时改速度。
         */
        coefficients[i][0] = arm_sv_simple_motion_targets[i];
        coefficients[i][1] = velocity;
        coefficients[i][2] = 0.5f * acceleration;
        coefficients[i][3] = -velocity / duration_squared -
                             2.0f * acceleration / (3.0f * duration_s);
        coefficients[i][4] = velocity / (2.0f * duration_cubed) +
                             acceleration / (4.0f * duration_squared);
        coefficients[i][5] = 0.0f;
    }
}

static uint8_t pose_brake_candidate_is_valid(
    const float coefficients[ARM_SV_POSE_COUNT]
                            [ARM_SV_TRAJECTORY_COEFFICIENT_COUNT],
    float duration_s)
{
    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        const float *joint = coefficients[i];
        float max_velocity;
        float max_acceleration;
        float jerk_at_start = 6.0f * coefficients[i][3];
        float jerk_at_end = jerk_at_start +
                            24.0f * coefficients[i][4] * duration_s;
        float position;
        float velocity;
        float acceleration;
        float velocity_root_denominator;
        float acceleration_quadratic_a;
        float acceleration_quadratic_b;
        float acceleration_quadratic_c;

        for (uint8_t coefficient = 0U;
             coefficient < ARM_SV_TRAJECTORY_COEFFICIENT_COUNT;
             coefficient++)
        {
            if (!isfinite(joint[coefficient]))
            {
                return 0U;
            }
        }
        if ((joint[0] < servo_calibration[i].min_radian -
                            ARM_SV_TARGET_CHANGE_EPSILON_RAD) ||
            (joint[0] > servo_calibration[i].max_radian +
                            ARM_SV_TARGET_CHANGE_EPSILON_RAD))
        {
            return 0U;
        }

        if ((!isfinite(jerk_at_start)) || (!isfinite(jerk_at_end)) ||
            (fabsf(jerk_at_start) >
             ARM_SV_POSE_MAX_JERK_RAD_S3 * ARM_SV_TRAJECTORY_LIMIT_MARGIN) ||
            (fabsf(jerk_at_end) >
             ARM_SV_POSE_MAX_JERK_RAD_S3 * ARM_SV_TRAJECTORY_LIMIT_MARGIN))
        {
            return 0U;
        }

        evaluate_quintic(joint,
                         duration_s,
                         &position,
                         &velocity,
                         &acceleration);
        if ((!isfinite(position)) || (!isfinite(velocity)) ||
            (!isfinite(acceleration)) ||
            (position < servo_calibration[i].min_radian -
                            ARM_SV_TARGET_CHANGE_EPSILON_RAD) ||
            (position > servo_calibration[i].max_radian +
                            ARM_SV_TARGET_CHANGE_EPSILON_RAD))
        {
            return 0U;
        }

        /*
         * 制动速度可写成(1-s)^2乘以一个一次式，因此除终点外最多只有
         * 一个内部速度零点。检查该点即可严格覆盖整个位置范围，不依赖采样。
         */
        velocity_root_denominator = 2.0f * joint[1] +
                                    2.0f * joint[2] * duration_s;
        if (fabsf(velocity_root_denominator) > ARM_SV_POLYNOMIAL_EPSILON)
        {
            float root_phase = -joint[1] / velocity_root_denominator;

            if ((root_phase > 0.0f) && (root_phase < 1.0f))
            {
                evaluate_quintic(joint,
                                 root_phase * duration_s,
                                 &position,
                                 &velocity,
                                 &acceleration);
                if ((position < servo_calibration[i].min_radian -
                                    ARM_SV_TARGET_CHANGE_EPSILON_RAD) ||
                    (position > servo_calibration[i].max_radian +
                                    ARM_SV_TARGET_CHANGE_EPSILON_RAD))
                {
                    return 0U;
                }
            }
        }

        max_velocity = fmaxf(fabsf(joint[1]), fabsf(velocity));
        max_acceleration =
            fmaxf(fabsf(2.0f * joint[2]), fabsf(acceleration));

        /* 加速度极值出现在加加速度为0的位置。 */
        if (fabsf(joint[4]) > ARM_SV_POLYNOMIAL_EPSILON)
        {
            float acceleration_extreme_time = -joint[3] / (4.0f * joint[4]);

            if ((acceleration_extreme_time > 0.0f) &&
                (acceleration_extreme_time < duration_s))
            {
                evaluate_quintic(joint,
                                 acceleration_extreme_time,
                                 &position,
                                 &velocity,
                                 &acceleration);
                max_acceleration = fmaxf(max_acceleration, fabsf(acceleration));
            }
        }

        /* 速度极值出现在加速度二次式的实根处。 */
        acceleration_quadratic_a = 12.0f * joint[4];
        acceleration_quadratic_b = 6.0f * joint[3];
        acceleration_quadratic_c = 2.0f * joint[2];
        if (fabsf(acceleration_quadratic_a) > ARM_SV_POLYNOMIAL_EPSILON)
        {
            float discriminant =
                acceleration_quadratic_b * acceleration_quadratic_b -
                4.0f * acceleration_quadratic_a *
                    acceleration_quadratic_c;

            if (discriminant >= 0.0f)
            {
                float root_scale = 0.5f / acceleration_quadratic_a;
                float square_root = sqrtf(discriminant);
                float roots[2] = {
                    (-acceleration_quadratic_b - square_root) * root_scale,
                    (-acceleration_quadratic_b + square_root) * root_scale};

                for (uint8_t root = 0U; root < 2U; root++)
                {
                    if ((roots[root] > 0.0f) &&
                        (roots[root] < duration_s))
                    {
                        evaluate_quintic(joint,
                                         roots[root],
                                         &position,
                                         &velocity,
                                         &acceleration);
                        max_velocity = fmaxf(max_velocity, fabsf(velocity));
                    }
                }
            }
        }
        else if (fabsf(acceleration_quadratic_b) >
                 ARM_SV_POLYNOMIAL_EPSILON)
        {
            float root = -acceleration_quadratic_c /
                         acceleration_quadratic_b;

            if ((root > 0.0f) && (root < duration_s))
            {
                evaluate_quintic(joint,
                                 root,
                                 &position,
                                 &velocity,
                                 &acceleration);
                max_velocity = fmaxf(max_velocity, fabsf(velocity));
            }
        }

        if ((!isfinite(max_velocity)) || (!isfinite(max_acceleration)) ||
            (max_velocity >
             ARM_SV_MAX_SPEED_RAD_S * ARM_SV_TRAJECTORY_LIMIT_MARGIN) ||
            (max_acceleration >
             ARM_SV_MAX_ACCEL_RAD_S2 * ARM_SV_TRAJECTORY_LIMIT_MARGIN))
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t start_pose_brake(uint32_t now_ms)
{
    float candidate[ARM_SV_POSE_COUNT][ARM_SV_TRAJECTORY_COEFFICIENT_COUNT];

    for (uint32_t step = 0U; step <= ARM_SV_POSE_BRAKE_SEARCH_STEPS; step++)
    {
        float duration_s = ARM_SV_POSE_BRAKE_MIN_DURATION_S +
                           (ARM_SV_POSE_BRAKE_MAX_DURATION_S -
                            ARM_SV_POSE_BRAKE_MIN_DURATION_S) *
                               (float)step /
                               (float)ARM_SV_POSE_BRAKE_SEARCH_STEPS;

        build_pose_brake_coefficients(duration_s, candidate);
        if (pose_brake_candidate_is_valid(candidate, duration_s) == 0U)
        {
            continue;
        }

        for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
        {
            for (uint8_t coefficient = 0U;
                 coefficient < ARM_SV_TRAJECTORY_COEFFICIENT_COUNT;
                 coefficient++)
            {
                arm_sv_pose_brake_coefficients[i][coefficient] =
                    candidate[i][coefficient];
            }
        }
        arm_sv_pose_duration_s = duration_s;
        arm_sv_simple_motion_start_tick_ms = now_ms;
        set_pose_motion_state(ARM_SV_POSE_MOTION_BRAKING);
        arm_sv_pose_brake_duration_ms_debug =
            (uint32_t)(duration_s * 1000.0f + 0.5f);
        arm_sv_pose_duration_ms_debug = arm_sv_pose_brake_duration_ms_debug;
        if (arm_sv_pose_interrupt_count_debug < UINT32_MAX)
        {
            arm_sv_pose_interrupt_count_debug++;
        }
        return 1U;
    }

    if (arm_sv_pose_brake_fallback_count_debug < UINT32_MAX)
    {
        arm_sv_pose_brake_fallback_count_debug++;
    }
    return 0U;
}

static void update_pose_motion(uint32_t now_ms)
{
    float elapsed_s;

    if (arm_sv_pose_motion_state == ARM_SV_POSE_MOTION_IDLE)
    {
        return;
    }

    elapsed_s = (float)(now_ms - arm_sv_simple_motion_start_tick_ms) * 0.001f;
    if (arm_sv_pose_motion_state == ARM_SV_POSE_MOTION_MOVING)
    {
        if (elapsed_s >= arm_sv_pose_duration_s)
        {
            for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
            {
                arm_sv_simple_motion_targets[i] =
                    arm_sv_pose_latched_target_radian[i];
                arm_sv_simple_motion_velocities[i] = 0.0f;
                arm_sv_simple_motion_accelerations[i] = 0.0f;
            }
            set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
            arm_sv_pose_duration_ms_debug = 0U;
        }
        else
        {
            float phase = elapsed_s / arm_sv_pose_duration_s;

            for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
            {
                interpolate_minimum_jerk(
                    arm_sv_pose_start_radian[i],
                    arm_sv_pose_latched_target_radian[i],
                    phase,
                    arm_sv_pose_duration_s,
                    &arm_sv_simple_motion_targets[i],
                    &arm_sv_simple_motion_velocities[i],
                    &arm_sv_simple_motion_accelerations[i]);
            }
        }
    }
    else
    {
        float evaluation_time_s =
            (elapsed_s >= arm_sv_pose_duration_s)
                ? arm_sv_pose_duration_s
                : elapsed_s;

        for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
        {
            evaluate_quintic(arm_sv_pose_brake_coefficients[i],
                             evaluation_time_s,
                             &arm_sv_simple_motion_targets[i],
                             &arm_sv_simple_motion_velocities[i],
                             &arm_sv_simple_motion_accelerations[i]);
        }
        if (elapsed_s >= arm_sv_pose_duration_s)
        {
            for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
            {
                arm_sv_simple_motion_velocities[i] = 0.0f;
                arm_sv_simple_motion_accelerations[i] = 0.0f;
            }
            set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
            arm_sv_pose_duration_ms_debug = 0U;
        }
    }

    if ((arm_sv_pose_motion_state == ARM_SV_POSE_MOTION_IDLE) &&
        (arm_sv_pose_pending_target_valid != 0U))
    {
        start_pose_move(now_ms, arm_sv_pose_pending_target_radian);
    }
}

static void build_coordinated_coefficients(float duration_s)
{
    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        build_quintic_coefficients(servo_position_radian[i],
                                    servo_velocity_radian_s[i],
                                    servo_acceleration_radian_s2[i],
                                    arm_sv_trajectory_target_radian[i],
                                    duration_s,
                                    arm_sv_trajectory_coefficients[i]);
    }
}

static void measure_coordinated_limits(float duration_s,
                                       float *max_velocity,
                                       float *max_acceleration)
{
    *max_velocity = 0.0f;
    *max_acceleration = 0.0f;

    for (uint32_t sample = 0U; sample <= ARM_SV_TRAJECTORY_PLAN_SAMPLES; sample++)
    {
        float sample_time_s = duration_s * (float)sample /
                              (float)ARM_SV_TRAJECTORY_PLAN_SAMPLES;

        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            float position;
            float velocity;
            float acceleration;

            evaluate_quintic(arm_sv_trajectory_coefficients[i],
                             sample_time_s,
                             &position,
                             &velocity,
                             &acceleration);
            (void)position;
            if (fabsf(velocity) > *max_velocity)
            {
                *max_velocity = fabsf(velocity);
            }
            if (fabsf(acceleration) > *max_acceleration)
            {
                *max_acceleration = fabsf(acceleration);
            }
        }
    }
}

static void start_coordinated_trajectory(uint32_t now_ms)
{
    float duration_s = ARM_SV_TRAJECTORY_MIN_DURATION_S;
    uint8_t motion_required = 0U;

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        float distance;
        float velocity_duration;
        float acceleration_duration;
        float stopping_duration;

        arm_sv_trajectory_target_radian[i] = target_snapshot[i];
        servo_acceleration_radian_s2[i] = 0.0f;
        distance = fabsf(arm_sv_trajectory_target_radian[i] -
                         servo_position_radian[i]);
        velocity_duration = ARM_SV_MINIMUM_JERK_PEAK_VELOCITY * distance /
                            ARM_SV_MAX_SPEED_RAD_S;
        acceleration_duration = sqrtf(ARM_SV_MINIMUM_JERK_PEAK_ACCEL *
                                      distance /
                                      ARM_SV_MAX_ACCEL_RAD_S2);
        stopping_duration = 2.0f * fabsf(servo_velocity_radian_s[i]) /
                            ARM_SV_MAX_ACCEL_RAD_S2;

        if ((distance > ARM_SV_POSITION_EPSILON_RAD) ||
            (fabsf(servo_velocity_radian_s[i]) > ARM_SV_VELOCITY_EPSILON_RAD_S) ||
            (fabsf(servo_acceleration_radian_s2[i]) >
             ARM_SV_ACCELERATION_EPSILON_RAD_S2))
        {
            motion_required = 1U;
        }
        if (velocity_duration > duration_s)
        {
            duration_s = velocity_duration;
        }
        if (acceleration_duration > duration_s)
        {
            duration_s = acceleration_duration;
        }
        if (stopping_duration > duration_s)
        {
            duration_s = stopping_duration;
        }
    }

    arm_sv_trajectory_target_sequence = target_snapshot_sequence;
    arm_sv_trajectory_replan_count_debug++;
    if (motion_required == 0U)
    {
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            servo_position_radian[i] = arm_sv_trajectory_target_radian[i];
            servo_velocity_radian_s[i] = 0.0f;
            servo_acceleration_radian_s2[i] = 0.0f;
        }
        arm_sv_trajectory_active = 0U;
        arm_sv_trajectory_active_debug = 0U;
        arm_sv_trajectory_duration_ms_debug = 0U;
        return;
    }

    duration_s = clamp_float(duration_s,
                             ARM_SV_TRAJECTORY_MIN_DURATION_S,
                             ARM_SV_TRAJECTORY_MAX_DURATION_S);
    for (uint8_t iteration = 0U;
         iteration < ARM_SV_TRAJECTORY_PLAN_ITERATIONS;
         iteration++)
    {
        float measured_velocity;
        float measured_acceleration;
        float velocity_scale;
        float acceleration_scale;
        float duration_scale;

        build_coordinated_coefficients(duration_s);
        measure_coordinated_limits(duration_s,
                                   &measured_velocity,
                                   &measured_acceleration);
        velocity_scale = measured_velocity / ARM_SV_MAX_SPEED_RAD_S;
        acceleration_scale = sqrtf(measured_acceleration /
                                   ARM_SV_MAX_ACCEL_RAD_S2);
        duration_scale = fmaxf(velocity_scale, acceleration_scale);
        if (duration_scale <= 1.0f)
        {
            break;
        }

        duration_s = clamp_float(duration_s * duration_scale *
                                     ARM_SV_TRAJECTORY_LIMIT_MARGIN,
                                 ARM_SV_TRAJECTORY_MIN_DURATION_S,
                                 ARM_SV_TRAJECTORY_MAX_DURATION_S);
    }

    build_coordinated_coefficients(duration_s);
    arm_sv_trajectory_duration_s = duration_s;
    arm_sv_trajectory_start_tick_ms = now_ms;
    arm_sv_trajectory_active = 1U;
    arm_sv_trajectory_active_debug = 1U;
    arm_sv_trajectory_duration_ms_debug =
        (uint32_t)(duration_s * 1000.0f + 0.5f);
}

static void evaluate_coordinated_trajectory(uint32_t now_ms)
{
    float elapsed_s;
    uint8_t limit_clamped = 0U;

    if (arm_sv_trajectory_active == 0U)
    {
        return;
    }

    elapsed_s = (float)(now_ms - arm_sv_trajectory_start_tick_ms) * 0.001f;
    if (elapsed_s >= arm_sv_trajectory_duration_s)
    {
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            servo_position_radian[i] = arm_sv_trajectory_target_radian[i];
            servo_velocity_radian_s[i] = 0.0f;
            servo_acceleration_radian_s2[i] = 0.0f;
        }
        arm_sv_trajectory_active = 0U;
        arm_sv_trajectory_active_debug = 0U;
        return;
    }

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        float unclamped_position;

        evaluate_quintic(arm_sv_trajectory_coefficients[i],
                         elapsed_s,
                         &servo_position_radian[i],
                         &servo_velocity_radian_s[i],
                         &servo_acceleration_radian_s2[i]);
        unclamped_position = servo_position_radian[i];
        servo_position_radian[i] = clamp_float(servo_position_radian[i],
                                               servo_calibration[i].min_radian,
                                               servo_calibration[i].max_radian);
        if (servo_position_radian[i] != unclamped_position)
        {
            servo_velocity_radian_s[i] = 0.0f;
            servo_acceleration_radian_s2[i] = 0.0f;
            limit_clamped = 1U;
        }
    }

    if (limit_clamped != 0U)
    {
        arm_sv_limit_clamp_count_debug++;
        arm_sv_trajectory_active = 0U;
        arm_sv_trajectory_active_debug = 0U;
        arm_sv_trajectory_target_sequence = target_snapshot_sequence ^ 1U;
    }
}

float radian_to_duty_270(float radian)
{
    float duty;

    if (!isfinite(radian))
    {
        return ARM_SV_CENTER_DUTY;
    }

    radian = clamp_float(radian, ARM_SV_MIN_RADIAN, ARM_SV_MAX_RADIAN);
    duty = ARM_SV_CENTER_DUTY + (radian / ARM_SV_MAX_RADIAN) * ARM_SV_DUTY_SPAN;
    return clamp_float(duty, ARM_SV_MIN_DUTY, ARM_SV_MAX_DUTY);
}

void set_motor_radians_270(const float radians[ARM_SV_COUNT])
{
    ARM_SV_SetAllRampTargets(radians);
}

static uint8_t write_pwm_frame(void)
{
    float frame_duties[ARM_SV_COUNT];
    float pca_duties[ARM_SV_PCA_FRAME_CHANNEL_COUNT];
    PCA9685_SubmitStatus_t submit_status;
    uint32_t previous_primask;
    uint8_t was_retry;

    if (arm_sv_pwm_frame_inflight != 0U)
    {
        return 0U;
    }

    /* 固定本次提交快照，避免发送完成前后把两个控制周期的数据混在一帧中。 */
    previous_primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
    {
        frame_duties[i] = pending_duties[i];
    }
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }

    build_pca_duty_frame(frame_duties, pca_duties);
    was_retry = arm_sv_pwm_retry_pending;
    submit_status = PCA9685_SubmitDuties(ARM_SV_PCA_FRAME_FIRST_CHANNEL,
                                         pca_duties,
                                         ARM_SV_PCA_FRAME_CHANNEL_COUNT);
    if (submit_status == PCA9685_SUBMIT_ACCEPTED)
    {
        for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
        {
            inflight_duties[i] = frame_duties[i];
        }
        arm_sv_pwm_frame_inflight = 1U;
        arm_sv_pwm_frame_inflight_debug = 1U;
        arm_sv_pwm_retry_pending = 0U;
        if ((was_retry != 0U) &&
            (arm_sv_pwm_retry_count_debug < UINT32_MAX))
        {
            arm_sv_pwm_retry_count_debug++;
        }
        return 1U;
    }

    arm_sv_pwm_retry_pending = 1U;
    if (submit_status == PCA9685_SUBMIT_BUSY)
    {
        if (arm_sv_pwm_submit_busy_count_debug < UINT32_MAX)
        {
            arm_sv_pwm_submit_busy_count_debug++;
        }
    }
    else if (submit_status == PCA9685_SUBMIT_ERROR)
    {
        if (arm_sv_i2c_error_count_debug < UINT32_MAX)
        {
            arm_sv_i2c_error_count_debug++;
        }
    }
    return 0U;
}

static void process_pwm_frame_result(void)
{
    uint32_t duration_ms = 0U;
    PCA9685_FrameResult_t result = PCA9685_TakeFrameResult(&duration_ms);

    if (result == PCA9685_FRAME_RESULT_NONE)
    {
        return;
    }

    arm_sv_last_pwm_write_duration_ms_debug = duration_ms;
    if (duration_ms > arm_sv_max_pwm_write_duration_ms_debug)
    {
        arm_sv_max_pwm_write_duration_ms_debug = duration_ms;
    }

    if (arm_sv_pwm_frame_inflight == 0U)
    {
        return;
    }

    arm_sv_pwm_frame_inflight = 0U;
    arm_sv_pwm_frame_inflight_debug = 0U;
    if (result == PCA9685_FRAME_RESULT_SUCCESS)
    {
        for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
        {
            current_duties[i] = inflight_duties[i];
        }
        if (arm_sv_i2c_write_count_debug < UINT32_MAX)
        {
            arm_sv_i2c_write_count_debug++;
        }
        arm_sv_pwm_keepalive_timer_s = 0.0f;
        return;
    }

    if (arm_sv_i2c_error_count_debug < UINT32_MAX)
    {
        arm_sv_i2c_error_count_debug++;
    }
    arm_sv_pwm_retry_pending = 1U;
}

static void set_sv_duty(uint8_t id, float duty)
{
    if ((id >= ARM_SV_ACTIVE_COUNT) || (!isfinite(duty)))
    {
        return;
    }

    duty = clamp_float(duty, ARM_SV_MIN_DUTY, ARM_SV_MAX_DUTY);
    set_duty_member(&duties_tx, id, duty);
    arm_sv_duty_debug[id] = duty;

    pending_duties[id] = duty;
    (void)write_pwm_frame();
}

static void reset_trajectory(const float radians[ARM_SV_ACTIVE_COUNT])
{
    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        float safe_radian = sanitize_servo_target(i, radians[i], 0.0f);

        motor_radians[i] = safe_radian;
        servo_position_radian[i] = safe_radian;
        servo_velocity_radian_s[i] = 0.0f;
        servo_acceleration_radian_s2[i] = 0.0f;
        arm_sv_simple_motion_targets[i] = safe_radian;
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_pose_target_radian[i] = safe_radian;
        target_snapshot[i] = safe_radian;
        arm_sv_trajectory_target_radian[i] = safe_radian;
        arm_sv_target_radian_debug[i] = safe_radian;
        arm_sv_position_radian_debug[i] = safe_radian;
        arm_sv_velocity_radian_debug[i] = 0.0f;
    }

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        arm_sv_pose_start_radian[i] = servo_position_radian[i];
        arm_sv_pose_latched_target_radian[i] = servo_position_radian[i];
        arm_sv_pose_pending_target_radian[i] = servo_position_radian[i];
    }

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        motor_radians[i] = 0.0f;
        servo_position_radian[i] = 0.0f;
        servo_velocity_radian_s[i] = 0.0f;
        servo_acceleration_radian_s2[i] = 0.0f;
        arm_sv_simple_motion_targets[i] = 0.0f;
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_pose_target_radian[i] = 0.0f;
        target_snapshot[i] = 0.0f;
        arm_sv_target_radian_debug[i] = 0.0f;
        arm_sv_position_radian_debug[i] = 0.0f;
        arm_sv_velocity_radian_debug[i] = 0.0f;
    }
}

static void reset_user_action(uint32_t now_ms)
{
    arm_sv_user_action_start_tick_ms = now_ms;
    arm_sv_user_action_hold_start_tick_ms = 0U;
    arm_sv_user_action_expected_sequence = arm_sv_pose_command_sequence;
    arm_sv_user_action_step_debug = 0U;
    arm_sv_user_action_elapsed_ms_debug = 0U;

#if (ARM_SV_USER_ACTION_ENABLE != 0)
    arm_sv_user_action_state = ARM_SV_USER_ACTION_WAIT_START;
    arm_sv_user_action_done_debug = 0U;
#else
    arm_sv_user_action_state = ARM_SV_USER_ACTION_DONE;
    arm_sv_user_action_done_debug = 1U;
#endif
    arm_sv_user_action_state_debug = (uint32_t)arm_sv_user_action_state;
}

#if (ARM_SV_USER_ACTION_ENABLE != 0)
static uint8_t user_action_pose_is_idle(void)
{
    return ((arm_sv_pose_trajectory_active == 0U) &&
            (arm_sv_pose_command_sequence == arm_sv_pose_applied_sequence))
               ? 1U
               : 0U;
}

static void set_user_action_state(ARM_SV_UserActionState_t state)
{
    arm_sv_user_action_state = state;
    arm_sv_user_action_state_debug = (uint32_t)state;
    arm_sv_user_action_done_debug =
        ((state == ARM_SV_USER_ACTION_DONE) ||
         (state == ARM_SV_USER_ACTION_ABORTED))
            ? 1U
            : 0U;
}

static uint8_t submit_user_action_pose(
    const int16_t pose_degrees[ARM_SV_POSE_COUNT])
{
    float pose_radians[ARM_SV_POSE_COUNT];

    if (pose_degrees == NULL)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        pose_radians[i] =
            (float)pose_degrees[i] * ARM_SV_DEG_TO_RAD_SCALE;
    }

    if (submit_pose_targets(pose_radians, 1U) != 0U)
    {
        arm_sv_user_action_expected_sequence = arm_sv_pose_command_sequence;
        arm_sv_user_action_step_debug++;
        return 1U;
    }
    return 0U;
}

static void update_user_action(uint32_t now_ms)
{
    if (arm_sv_control_mode_active_debug !=
        (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION)
    {
        return;
    }

    if ((arm_sv_user_action_state == ARM_SV_USER_ACTION_DONE) ||
        (arm_sv_user_action_state == ARM_SV_USER_ACTION_ABORTED))
    {
        return;
    }

    /* 外部或DAP提交了新姿态时，自动动作立即停止继续发命令。 */
    if (arm_sv_pose_command_sequence != arm_sv_user_action_expected_sequence)
    {
        set_user_action_state(ARM_SV_USER_ACTION_ABORTED);
        return;
    }

    switch (arm_sv_user_action_state)
    {
    case ARM_SV_USER_ACTION_WAIT_START:
        arm_sv_user_action_elapsed_ms_debug =
            now_ms - arm_sv_user_action_start_tick_ms;
        if (arm_sv_user_action_elapsed_ms_debug >=
            (uint32_t)ARM_SV_USER_START_DELAY_MS)
        {
            if (submit_user_action_pose(arm_sv_user_forward_grab_pose_deg) != 0U)
            {
                set_user_action_state(ARM_SV_USER_ACTION_MOVING_FORWARD_GRAB);
            }
            else
            {
                set_user_action_state(ARM_SV_USER_ACTION_ABORTED);
            }
        }
        break;

    case ARM_SV_USER_ACTION_MOVING_FORWARD_GRAB:
        if (user_action_pose_is_idle() != 0U)
        {
            arm_sv_user_action_hold_start_tick_ms = now_ms;
            arm_sv_user_action_elapsed_ms_debug = 0U;
            set_user_action_state(ARM_SV_USER_ACTION_HOLDING_GRAB);
        }
        break;

    case ARM_SV_USER_ACTION_HOLDING_GRAB:
        arm_sv_user_action_elapsed_ms_debug =
            now_ms - arm_sv_user_action_hold_start_tick_ms;
        if (arm_sv_user_action_elapsed_ms_debug >=
            (uint32_t)ARM_SV_USER_GRAB_HOLD_MS)
        {
            if (submit_user_action_pose(arm_sv_user_put_down_pose_deg) != 0U)
            {
                set_user_action_state(ARM_SV_USER_ACTION_MOVING_PUT_DOWN);
            }
            else
            {
                set_user_action_state(ARM_SV_USER_ACTION_ABORTED);
            }
        }
        break;

    case ARM_SV_USER_ACTION_MOVING_PUT_DOWN:
        if (user_action_pose_is_idle() != 0U)
        {
            set_user_action_state(ARM_SV_USER_ACTION_DONE);
        }
        break;

    default:
        set_user_action_state(ARM_SV_USER_ACTION_ABORTED);
        break;
    }
}
#else
static void update_user_action(uint32_t now_ms)
{
    (void)now_ms;
}
#endif

static uint8_t active_control_trajectory_is_idle(void)
{
    if (control_mode_uses_pose(arm_sv_control_mode_active_debug) != 0U)
    {
        return (arm_sv_pose_trajectory_active == 0U) ? 1U : 0U;
    }
    if (arm_sv_control_mode_active_debug ==
        (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP)
    {
        return (arm_sv_trajectory_active == 0U) ? 1U : 0U;
    }
    return 1U;
}

static void synchronize_pose_motion_to_current_position(void)
{
    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        arm_sv_simple_motion_targets[i] = servo_position_radian[i];
        arm_sv_simple_motion_velocities[i] = servo_velocity_radian_s[i];
        arm_sv_simple_motion_accelerations[i] =
            servo_acceleration_radian_s2[i];
    }
    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        arm_sv_pose_start_radian[i] = servo_position_radian[i];
        arm_sv_pose_latched_target_radian[i] = servo_position_radian[i];
    }
    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        arm_sv_simple_motion_targets[i] = 0.0f;
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
    }

    arm_sv_pose_pending_target_valid = 0U;
    set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
    arm_sv_pose_duration_ms_debug = 0U;
}

static void apply_control_mode(uint32_t requested_mode, uint32_t now_ms)
{
#if (ARM_SV_USER_ACTION_ENABLE != 0)
    if ((arm_sv_control_mode_active_debug ==
         (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION) &&
        (requested_mode != (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION) &&
        (arm_sv_user_action_state != ARM_SV_USER_ACTION_DONE) &&
        (arm_sv_user_action_state != ARM_SV_USER_ACTION_ABORTED))
    {
        set_user_action_state(ARM_SV_USER_ACTION_ABORTED);
    }
#endif

    if (control_mode_uses_pose(requested_mode) != 0U)
    {
        synchronize_pose_motion_to_current_position();
        if (requested_mode == (uint32_t)ARM_SV_CONTROL_MODE_USER_ACTION)
        {
            /* 返回自动模式时丢弃旧的外部姿态，重新从等待阶段开始。 */
            arm_sv_pose_applied_sequence = arm_sv_pose_command_sequence;
            arm_sv_pose_applied_sequence_debug = arm_sv_pose_command_sequence;
            reset_user_action(now_ms);
        }
    }
    else
    {
        arm_sv_simple_motion_active_debug = 0U;
        arm_sv_pose_pending_target_valid = 0U;
        set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
    }

    if (requested_mode == (uint32_t)ARM_SV_CONTROL_MODE_HOLD)
    {
        arm_sv_trajectory_active = 0U;
        arm_sv_trajectory_active_debug = 0U;
        arm_sv_trajectory_duration_ms_debug = 0U;
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            servo_velocity_radian_s[i] = 0.0f;
            servo_acceleration_radian_s2[i] = 0.0f;
        }
    }

    arm_sv_control_mode_active_debug = requested_mode;
    arm_sv_control_mode_pending_debug = 0U;
    if (arm_sv_control_mode_switch_count_debug < UINT32_MAX)
    {
        arm_sv_control_mode_switch_count_debug++;
    }
}

static void update_control_mode(uint32_t now_ms)
{
    uint32_t requested_mode = arm_sv_control_mode_request_debug;

    if (control_mode_is_valid(requested_mode) == 0U)
    {
        record_control_command_reject();
        arm_sv_control_mode_request_debug = arm_sv_control_mode_active_debug;
        arm_sv_control_mode_pending_debug = 0U;
        return;
    }
    if (requested_mode == arm_sv_control_mode_active_debug)
    {
        arm_sv_control_mode_pending_debug = 0U;
        return;
    }

    arm_sv_control_mode_pending_debug = 1U;
    if (active_control_trajectory_is_idle() != 0U)
    {
        apply_control_mode(requested_mode, now_ms);
    }
}

static void update_simple_motion(void)
{
#if (ARM_SV_SIMPLE_MOTION_ENABLE != 0U)
    uint32_t now_ms = HAL_GetTick();

    if (control_mode_uses_pose(arm_sv_control_mode_active_debug) == 0U)
    {
        arm_sv_simple_motion_active_debug = 0U;
        arm_sv_simple_motion_done_debug = 1U;
        arm_sv_simple_motion_step_debug = 0U;
        arm_sv_simple_motion_servo_debug = 0U;
        arm_sv_simple_motion_phase_debug = 0U;
        return;
    }

    arm_sv_simple_motion_active_debug = 1U;

    /* 先把当前运动推进到本控制周期，再从准确的当前位置处理新命令。 */
    update_pose_motion(now_ms);

    if (arm_sv_pose_command_sequence != arm_sv_pose_applied_sequence)
    {
        float candidate[ARM_SV_POSE_COUNT];
        uint32_t sequence_before = arm_sv_pose_command_sequence;
        uint32_t sequence_after;

        __DMB();
        for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
        {
            candidate[i] = arm_sv_pose_target_radian[i];
        }
        __DMB();
        sequence_after = arm_sv_pose_command_sequence;

        if (sequence_before == sequence_after)
        {
            for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
            {
                float safe_target = sanitize_servo_target(
                    i,
                    candidate[i],
                    arm_sv_simple_motion_targets[i]);

                if ((!isfinite(candidate[i])) || (safe_target != candidate[i]))
                {
                    arm_sv_pose_reject_count_debug++;
                    arm_sv_pose_target_radian[i] = safe_target;
                }
                candidate[i] = safe_target;
                arm_sv_pose_pending_target_radian[i] = safe_target;
            }

            arm_sv_pose_applied_sequence = sequence_after;
            arm_sv_pose_applied_sequence_debug = sequence_after;
            if (arm_sv_pose_motion_state == ARM_SV_POSE_MOTION_IDLE)
            {
                start_pose_move(now_ms, candidate);
            }
            else if (arm_sv_pose_motion_state == ARM_SV_POSE_MOTION_BRAKING)
            {
                /* 已在制动时只替换待执行目标，不重新扰动当前制动曲线。 */
                arm_sv_pose_pending_target_valid = 1U;
            }
            else if (pose_targets_are_equal(candidate,
                                            arm_sv_pose_latched_target_radian) != 0U)
            {
                /* 重复提交相同目标时继续当前曲线，不产生不必要的制动。 */
                arm_sv_pose_pending_target_valid = 0U;
            }
            else
            {
                uint8_t derivatives_active = 0U;

                arm_sv_pose_pending_target_valid = 1U;
                for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
                {
                    if ((fabsf(arm_sv_simple_motion_velocities[i]) >
                         ARM_SV_VELOCITY_EPSILON_RAD_S) ||
                        (fabsf(arm_sv_simple_motion_accelerations[i]) >
                         ARM_SV_ACCELERATION_EPSILON_RAD_S2))
                    {
                        derivatives_active = 1U;
                        break;
                    }
                }

                if (derivatives_active == 0U)
                {
                    start_pose_move(now_ms, candidate);
                }
                else
                {
                    /* 失败时保留当前安全轨迹，待其结束后再执行pending目标。 */
                    (void)start_pose_brake(now_ms);
                }
            }
        }
    }

    /* 姿态动作只改变1~4号；其他已启用关节保持切换前的位置。 */
    for (uint8_t i = ARM_SV_POSE_COUNT; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_pose_target_radian[i] = arm_sv_simple_motion_targets[i];
    }
    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        arm_sv_simple_motion_targets[i] = 0.0f;
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_pose_target_radian[i] = 0.0f;
    }

    arm_sv_simple_motion_done_debug =
        (arm_sv_pose_trajectory_active == 0U) ? 1U : 0U;
    arm_sv_simple_motion_step_debug =
        (arm_sv_pose_trajectory_active != 0U) ? 1U : 2U;
    arm_sv_simple_motion_servo_debug = 0U;
    arm_sv_simple_motion_phase_debug = (uint32_t)arm_sv_pose_motion_state;
#else
    arm_sv_simple_motion_active_debug = 0U;
    arm_sv_simple_motion_done_debug = 1U;
    arm_sv_simple_motion_step_debug = 0U;
    arm_sv_simple_motion_servo_debug = 0U;
    arm_sv_simple_motion_phase_debug = 0U;
#endif
}

static void update_trajectory(float dt_s)
{
    uint32_t now_ms = HAL_GetTick();

    (void)dt_s;
    copy_target_snapshot();

    if (control_mode_uses_pose(arm_sv_control_mode_active_debug) != 0U)
    {
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            float position = arm_sv_simple_motion_targets[i];
            float velocity = arm_sv_simple_motion_velocities[i];
            float acceleration = arm_sv_simple_motion_accelerations[i];

            servo_position_radian[i] = position;
            servo_velocity_radian_s[i] = velocity;
            servo_acceleration_radian_s2[i] = acceleration;
            arm_sv_target_radian_debug[i] = position;
            arm_sv_position_radian_debug[i] = position;
            arm_sv_velocity_radian_debug[i] = velocity;
        }

        for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
        {
            servo_position_radian[i] = 0.0f;
            servo_velocity_radian_s[i] = 0.0f;
            servo_acceleration_radian_s2[i] = 0.0f;
            arm_sv_target_radian_debug[i] = 0.0f;
            arm_sv_position_radian_debug[i] = 0.0f;
            arm_sv_velocity_radian_debug[i] = 0.0f;
        }
        return;
    }

    if (arm_sv_control_mode_active_debug ==
        (uint32_t)ARM_SV_CONTROL_MODE_HOLD)
    {
        for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
        {
            arm_sv_target_radian_debug[i] = servo_position_radian[i];
            arm_sv_position_radian_debug[i] = servo_position_radian[i];
            arm_sv_velocity_radian_debug[i] = 0.0f;
        }
        return;
    }

    evaluate_coordinated_trajectory(now_ms);
    if (target_snapshot_sequence != arm_sv_trajectory_target_sequence)
    {
        start_coordinated_trajectory(now_ms);
        evaluate_coordinated_trajectory(now_ms);
    }

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        arm_sv_target_radian_debug[i] = target_snapshot[i];
        arm_sv_position_radian_debug[i] = servo_position_radian[i];
        arm_sv_velocity_radian_debug[i] = servo_velocity_radian_s[i];
    }

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        servo_position_radian[i] = 0.0f;
        servo_velocity_radian_s[i] = 0.0f;
        servo_acceleration_radian_s2[i] = 0.0f;
        arm_sv_target_radian_debug[i] = 0.0f;
        arm_sv_position_radian_debug[i] = 0.0f;
        arm_sv_velocity_radian_debug[i] = 0.0f;
    }
}

static void update_pwm_outputs(uint8_t write_frame)
{
    uint8_t output_changed = 0U;

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        float duty = servo_radian_to_duty(i, servo_position_radian[i]);

        pending_duties[i] = duty;
        set_duty_member(&duties_tx, i, duty);
        arm_sv_duty_debug[i] = duty;
        if (fabsf(duty - current_duties[i]) >= ARM_SV_PWM_DUTY_EPSILON)
        {
            output_changed = 1U;
        }
    }

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        pending_duties[i] = ARM_SV_INACTIVE_DUTY;
        set_duty_member(&duties_tx, i, ARM_SV_INACTIVE_DUTY);
        arm_sv_duty_debug[i] = ARM_SV_INACTIVE_DUTY;
    }

    if ((write_frame != 0U) &&
        ((output_changed != 0U) ||
         (arm_sv_pwm_keepalive_timer_s >= ARM_SV_PWM_KEEPALIVE_PERIOD_S) ||
         (arm_sv_pwm_retry_pending != 0U)) &&
        (arm_sv_pwm_frame_inflight == 0U))
    {
        (void)write_pwm_frame();
    }
}

static uint8_t update_pwm_frame_timer(float dt_s)
{
    arm_sv_pwm_frame_timer_s += dt_s;
    arm_sv_pwm_keepalive_timer_s += dt_s;
    if (arm_sv_pwm_frame_timer_s < ARM_SV_PWM_FRAME_PERIOD_S)
    {
        return 0U;
    }

    do
    {
        arm_sv_pwm_frame_timer_s -= ARM_SV_PWM_FRAME_PERIOD_S;
    } while (arm_sv_pwm_frame_timer_s >= ARM_SV_PWM_FRAME_PERIOD_S);
    return 1U;
}

void ARM_SV_Init(float freq)
{
    PCA9685_Init(freq);

    arm_sv_target_sequence_debug = 0U;
    arm_sv_i2c_write_count_debug = 0U;
    arm_sv_i2c_error_count_debug = 0U;
    arm_sv_update_count_debug = 0U;
    arm_sv_last_update_interval_ms_debug = 0U;
    arm_sv_max_update_interval_ms_debug = 0U;
    arm_sv_deadline_miss_count_debug = 0U;
    arm_sv_last_pwm_write_duration_ms_debug = 0U;
    arm_sv_max_pwm_write_duration_ms_debug = 0U;
    arm_sv_pwm_frame_inflight_debug = 0U;
    arm_sv_pwm_retry_count_debug = 0U;
    arm_sv_pwm_submit_busy_count_debug = 0U;
    arm_sv_trajectory_active_debug = 0U;
    arm_sv_trajectory_duration_ms_debug = 0U;
    arm_sv_trajectory_replan_count_debug = 0U;
    arm_sv_limit_clamp_count_debug = 0U;
    arm_sv_simple_motion_active_debug = 0U;
    arm_sv_simple_motion_step_debug = 0U;
    arm_sv_simple_motion_servo_debug = 0U;
    arm_sv_simple_motion_phase_debug = 0U;
    arm_sv_simple_motion_done_debug = 0U;
    arm_sv_control_mode_request_debug =
        (uint32_t)ARM_SV_INITIAL_CONTROL_MODE;
    arm_sv_control_mode_active_debug =
        (uint32_t)ARM_SV_INITIAL_CONTROL_MODE;
    arm_sv_control_mode_pending_debug = 0U;
    arm_sv_control_mode_switch_count_debug = 0U;
    arm_sv_control_command_reject_count_debug = 0U;
    arm_sv_pca_fault_hold_active_debug = 0U;
    arm_sv_pca_fault_hold_total_ms_debug = 0U;
    arm_sv_pca_recovery_resync_count_debug = 0U;
    arm_sv_pca_fault_hold_active = 0U;
    arm_sv_pca_fault_hold_last_tick_ms = 0U;
    arm_sv_zero_trim_active_mask_debug = 0U;
    arm_sv_zero_trim_update_count_debug = 0U;
    arm_sv_zero_trim_reject_count_debug = 0U;
    /* 上电时清除所有 DAP 临时姿态命令，机械臂从校准后的0°开始。 */
    arm_sv_pose_command_sequence = 0U;
    arm_sv_pose_applied_sequence_debug = 0U;
    arm_sv_pose_reject_count_debug = 0U;
    arm_sv_pose_duration_ms_debug = 0U;
    arm_sv_pose_motion_state_debug = 0U;
    arm_sv_pose_interrupt_count_debug = 0U;
    arm_sv_pose_brake_duration_ms_debug = 0U;
    arm_sv_pose_brake_fallback_count_debug = 0U;
    arm_sv_simple_motion_start_tick_ms = 0U;
    arm_sv_pose_duration_s = ARM_SV_POSE_MIN_DURATION_S;
    arm_sv_pose_applied_sequence = 0U;
    arm_sv_pose_pending_target_valid = 0U;
    set_pose_motion_state(ARM_SV_POSE_MOTION_IDLE);
    target_snapshot_sequence = 0U;
    arm_sv_trajectory_duration_s = 0.0f;
    arm_sv_trajectory_start_tick_ms = 0U;
    arm_sv_trajectory_target_sequence = 0U;
    arm_sv_trajectory_active = 0U;
    arm_sv_pwm_frame_inflight = 0U;
    arm_sv_pwm_retry_pending = 0U;

    for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
    {
        motor_radians[i] = 0.0f;
        current_duties[i] = 0.0f;
        pending_duties[i] = (i < ARM_SV_ACTIVE_COUNT)
                                ? ARM_SV_CENTER_DUTY
                                : ARM_SV_INACTIVE_DUTY;
        inflight_duties[i] = 0.0f;
        arm_sv_simple_motion_targets[i] = 0.0f;
        arm_sv_simple_motion_velocities[i] = 0.0f;
        arm_sv_simple_motion_accelerations[i] = 0.0f;
        arm_sv_zero_trim_target_duty[i] = 0.0f;
        arm_sv_zero_trim_current_duty_debug[i] = 0.0f;
        arm_sv_pose_target_radian[i] = 0.0f;
    }

    for (uint8_t i = 0U; i < ARM_SV_ZERO_TRIM_COUNT; i++)
    {
        arm_sv_zero_trim_current_duty[i] = 0.0f;
        arm_sv_zero_trim_start_duty[i] = 0.0f;
        arm_sv_zero_trim_latched_target_duty[i] = 0.0f;
        arm_sv_zero_trim_start_tick_ms[i] = 0U;
        arm_sv_zero_trim_active[i] = 0U;
    }

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        arm_sv_pose_start_radian[i] = 0.0f;
        arm_sv_pose_latched_target_radian[i] = 0.0f;
        arm_sv_pose_pending_target_radian[i] = 0.0f;
        for (uint8_t coefficient = 0U;
             coefficient < ARM_SV_TRAJECTORY_COEFFICIENT_COUNT;
             coefficient++)
        {
            arm_sv_pose_brake_coefficients[i][coefficient] = 0.0f;
        }
    }

    reset_trajectory(arm_sv_reset_pose);
    arm_sv_last_tick_ms = 0U;
    arm_sv_timing_initialized = 0U;
    arm_sv_pwm_frame_timer_s = 0.0f;
    arm_sv_pwm_keepalive_timer_s = ARM_SV_PWM_KEEPALIVE_PERIOD_S;

    update_pwm_outputs(1U);
    duties_rx = ARM_SV_GetAllDuties();
    reset_user_action(HAL_GetTick());
}

uint8_t ARM_SV_RequestControlMode(ARM_SV_ControlMode_t mode)
{
    uint32_t requested_mode = (uint32_t)mode;
    uint32_t previous_primask;

    if (control_mode_is_valid(requested_mode) == 0U)
    {
        record_control_command_reject();
        return 0U;
    }

    previous_primask = __get_PRIMASK();
    __disable_irq();
    arm_sv_control_mode_request_debug = requested_mode;
    __DMB();
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

ARM_SV_ControlMode_t ARM_SV_GetControlMode(void)
{
    return (ARM_SV_ControlMode_t)arm_sv_control_mode_active_debug;
}

void ARM_SV_SetDuty(uint8_t sv_id, float duty)
{
    set_sv_duty(sv_id, duty);
}

void ARM_SV_SetDuty0(float duty) { set_sv_duty(0U, duty); }
void ARM_SV_SetDuty1(float duty) { set_sv_duty(1U, duty); }
void ARM_SV_SetDuty2(float duty) { set_sv_duty(2U, duty); }
void ARM_SV_SetDuty3(float duty) { set_sv_duty(3U, duty); }
void ARM_SV_SetDuty4(float duty) { set_sv_duty(4U, duty); }
void ARM_SV_SetDuty5(float duty) { set_sv_duty(5U, duty); }

float ARM_SV_GetDuty(uint8_t sv_id)
{
    if (sv_id >= ARM_SV_COUNT)
    {
        return 0.0f;
    }
    return current_duties[sv_id];
}

float ARM_SV_GetDuty0(void) { return current_duties[0]; }
float ARM_SV_GetDuty1(void) { return current_duties[1]; }
float ARM_SV_GetDuty2(void) { return current_duties[2]; }
float ARM_SV_GetDuty3(void) { return current_duties[3]; }
float ARM_SV_GetDuty4(void) { return current_duties[4]; }
float ARM_SV_GetDuty5(void) { return current_duties[5]; }

void ARM_SV_SetAllDuties(const float *duties)
{
    float safe_duties[ARM_SV_COUNT];

    if (duties == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        if (!isfinite(duties[i]))
        {
            return;
        }
        safe_duties[i] = clamp_float(duties[i], ARM_SV_MIN_DUTY, ARM_SV_MAX_DUTY);
    }

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        safe_duties[i] = ARM_SV_INACTIVE_DUTY;
    }

    for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
    {
        pending_duties[i] = safe_duties[i];
    }

    for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
    {
        set_duty_member(&duties_tx, i, safe_duties[i]);
        arm_sv_duty_debug[i] = safe_duties[i];
    }

    /* current_duties 只在 I2C 完成回调被任务确认后更新。 */
    (void)write_pwm_frame();
}

ARM_SV_Duties_t ARM_SV_GetAllDuties(void)
{
    ARM_SV_Duties_t duties;

    duties.duty0 = current_duties[0];
    duties.duty1 = current_duties[1];
    duties.duty2 = current_duties[2];
    duties.duty3 = current_duties[3];
    duties.duty4 = current_duties[4];
    duties.duty5 = current_duties[5];
    return duties;
}

void ARM_SV_SetRampTarget(uint8_t sv_id, float target_radian)
{
    float safe_target;
    uint32_t previous_primask;

    if ((sv_id >= ARM_SV_ACTIVE_COUNT) || (!isfinite(target_radian)))
    {
        return;
    }
    if (arm_sv_control_mode_request_debug !=
        (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP)
    {
        record_control_command_reject();
        return;
    }

    safe_target = sanitize_servo_target(sv_id, target_radian, 0.0f);
    if (fabsf(safe_target - motor_radians[sv_id]) < ARM_SV_TARGET_CHANGE_EPSILON_RAD)
    {
        return;
    }

    if (begin_target_write((uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP,
                           &previous_primask) == 0U)
    {
        return;
    }
    if (fabsf(safe_target - motor_radians[sv_id]) >=
        ARM_SV_TARGET_CHANGE_EPSILON_RAD)
    {
        motor_radians[sv_id] = safe_target;
    }
    end_target_write(previous_primask);
}

void ARM_SV_AdjustRampTarget(uint8_t sv_id, float delta_radian)
{
    float safe_target;
    uint32_t previous_primask;

    if ((sv_id >= ARM_SV_ACTIVE_COUNT) ||
        (!isfinite(delta_radian)) ||
        (fabsf(delta_radian) < ARM_SV_TARGET_CHANGE_EPSILON_RAD))
    {
        return;
    }
    if (arm_sv_control_mode_request_debug !=
        (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP)
    {
        record_control_command_reject();
        return;
    }

    safe_target = sanitize_servo_target(sv_id,
                                        motor_radians[sv_id] + delta_radian,
                                        motor_radians[sv_id]);
    if (fabsf(safe_target - motor_radians[sv_id]) <
        ARM_SV_TARGET_CHANGE_EPSILON_RAD)
    {
        return;
    }

    if (begin_target_write((uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP,
                           &previous_primask) == 0U)
    {
        return;
    }
    safe_target = sanitize_servo_target(sv_id,
                                        motor_radians[sv_id] + delta_radian,
                                        motor_radians[sv_id]);
    if (fabsf(safe_target - motor_radians[sv_id]) >=
        ARM_SV_TARGET_CHANGE_EPSILON_RAD)
    {
        motor_radians[sv_id] = safe_target;
    }
    end_target_write(previous_primask);
}

void ARM_SV_SetAllRampTargets(const float *radians)
{
    float safe_targets[ARM_SV_COUNT];
    uint8_t changed = 0U;
    uint32_t previous_primask;

    if (radians == NULL)
    {
        return;
    }
    if (arm_sv_control_mode_request_debug !=
        (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP)
    {
        record_control_command_reject();
        return;
    }

    for (uint8_t i = 0U; i < ARM_SV_ACTIVE_COUNT; i++)
    {
        if (!isfinite(radians[i]))
        {
            return;
        }
        safe_targets[i] = sanitize_servo_target(i, radians[i], 0.0f);
        if (fabsf(safe_targets[i] - motor_radians[i]) >= ARM_SV_TARGET_CHANGE_EPSILON_RAD)
        {
            changed = 1U;
        }
    }

    for (uint8_t i = ARM_SV_ACTIVE_COUNT; i < ARM_SV_COUNT; i++)
    {
        safe_targets[i] = 0.0f;
        if (fabsf(motor_radians[i]) >= ARM_SV_TARGET_CHANGE_EPSILON_RAD)
        {
            changed = 1U;
        }
    }

    if (changed == 0U)
    {
        return;
    }

    if (begin_target_write((uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP,
                           &previous_primask) == 0U)
    {
        return;
    }
    for (uint8_t i = 0U; i < ARM_SV_COUNT; i++)
    {
        motor_radians[i] = safe_targets[i];
    }
    end_target_write(previous_primask);
}

static uint8_t submit_pose_targets(const float radians[ARM_SV_POSE_COUNT],
                                   uint8_t user_action_owner)
{
    float safe_targets[ARM_SV_POSE_COUNT];
    uint32_t previous_primask;

    if (radians == NULL)
    {
        return 0U;
    }
    if ((user_action_owner == 0U) &&
        (arm_sv_control_mode_request_debug !=
         (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_POSE))
    {
        record_control_command_reject();
        return 0U;
    }

    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        if (!isfinite(radians[i]))
        {
            return 0U;
        }
        safe_targets[i] = sanitize_servo_target(i, radians[i], 0.0f);
    }

    /*
     * 在短临界区内一次性复制4个目标，再递增序号提交，保证控制任务不会
     * 读取到一半新、一半旧的姿态。数组0~3对应实体1~4号舵机。
    */
    previous_primask = __get_PRIMASK();
    __disable_irq();
    if ((user_action_owner == 0U) &&
        (arm_sv_control_mode_request_debug !=
         (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_POSE))
    {
        if ((previous_primask & 1U) == 0U)
        {
            __enable_irq();
        }
        record_control_command_reject();
        return 0U;
    }
    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        arm_sv_pose_target_radian[i] = safe_targets[i];
    }
    for (uint8_t i = ARM_SV_POSE_COUNT; i < ARM_SV_COUNT; i++)
    {
        arm_sv_pose_target_radian[i] =
            (i < ARM_SV_ACTIVE_COUNT) ? arm_sv_simple_motion_targets[i] : 0.0f;
    }
    __DMB();
    arm_sv_pose_command_sequence++;
    __DMB();
    if ((previous_primask & 1U) == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void ARM_SV_SetPoseTargets(const float radians[ARM_SV_POSE_COUNT])
{
    (void)submit_pose_targets(radians, 0U);
}

void ARM_SV_SetPoseTargetsDeg(const float degrees[ARM_SV_POSE_COUNT])
{
    float radians[ARM_SV_POSE_COUNT];

    if (degrees == NULL)
    {
        return;
    }

    /*
     * 应用层可直接传入角度数，例如 {0, 90, 0, 45}。
     * 这里统一转换成轨迹规划器使用的弧度，最终仍会按各舵机校准范围限位。
     */
    for (uint8_t i = 0U; i < ARM_SV_POSE_COUNT; i++)
    {
        if (!isfinite(degrees[i]))
        {
            return;
        }
        radians[i] = degrees[i] * ARM_SV_DEG_TO_RAD_SCALE;
    }

    ARM_SV_SetPoseTargets(radians);
}

void ARM_SV_RampUpdate(float dt)
{
    if ((arm_sv_control_mode_active_debug ==
         (uint32_t)ARM_SV_CONTROL_MODE_EXTERNAL_RAMP) &&
        isfinite(dt) && (dt > 0.0f))
    {
        update_trajectory(dt);
    }
}

void ARM_SV_Tx_Rx(void)
{
    float dt_s = get_control_dt_s();
    uint32_t now_ms = HAL_GetTick();
    uint8_t output_due = 0U;
    uint8_t pca_recovered;

    /* 本函数由5 ms的Arm_SV_Task调用；PWM按20 ms帧周期批量刷新。 */
    arm_sv_update_count_debug++;
    pca_recovered = PCA9685_ServiceRecovery();
    process_pwm_frame_result();
    now_ms = HAL_GetTick();
    update_pca_fault_hold(now_ms);
    if (arm_sv_pca_fault_hold_active != 0U)
    {
        output_due = update_pwm_frame_timer(dt_s);
        if ((pca9685_ready_debug != 0U) &&
            ((output_due != 0U) || (arm_sv_pwm_retry_pending != 0U)))
        {
            update_pwm_outputs(1U);
        }
        duties_rx = ARM_SV_GetAllDuties();
        return;
    }

    update_control_mode(now_ms);
    update_user_action(now_ms);
    update_simple_motion();
    update_trajectory(dt_s);
    update_zero_trim(now_ms);

    output_due = update_pwm_frame_timer(dt_s);

    if (pca_recovered != 0U)
    {
        /* 恢复完成后同周期重发当前整帧，避免PCA保持全关闭状态。 */
        arm_sv_pwm_keepalive_timer_s = ARM_SV_PWM_KEEPALIVE_PERIOD_S;
        output_due = 1U;
    }
    if (arm_sv_pwm_retry_pending != 0U)
    {
        /* 失败帧在下一个5 ms周期重试，不再等待下一个20 ms PWM帧边界。 */
        output_due = 1U;
    }

    update_pwm_outputs(output_due);
    duties_rx = ARM_SV_GetAllDuties();
}
