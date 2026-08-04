#ifndef ARM_SV_H
#define ARM_SV_H

#include <stdint.h>
#include "arm_sv_user_config.h"

/*
 * 舵机编号说明：
 * 1. 软件数组下标 0~3 分别对应实体 1~4 号舵机，也是当前参与姿态联动的关节。
 * 2. 软件数组下标 4 对应替代原 5 号的实体 6 号舵机，输出到 PCA9685 通道 5。
 * 3. 软件数组下标 5 暂时保留，等新的第 6 个舵机安装后再启用。
 * 注意：代码数组使用 0 起始下标，现场标签使用 1 起始编号，不要混淆。
 */
#define ARM_SV_COUNT        6U
#define ARM_SV_ACTIVE_COUNT 5U
#define ARM_SV_POSE_COUNT   4U

/*
 * 舵机控制权必须显式选择，避免自动动作、遥控器和PC命令互相覆盖。
 * 默认模式由 ARM_SV_USER_ACTION_ENABLE 决定：自动动作开启时默认由自动动作控制，
 * 自动动作关闭时默认接受外部渐变角度命令。
 */
typedef enum
{
    ARM_SV_CONTROL_MODE_USER_ACTION = 0U,
    ARM_SV_CONTROL_MODE_EXTERNAL_POSE = 1U,
    ARM_SV_CONTROL_MODE_EXTERNAL_RAMP = 2U,
    ARM_SV_CONTROL_MODE_HOLD = 3U
} ARM_SV_ControlMode_t;

/* 用户配置使用度/秒；这里只为内部轨迹计算自动换算为rad/s。 */
#define ARM_SV_RAMP_SPEED                                                   \
    ((float)ARM_SV_USER_MAX_SPEED_DEG_S * 0.017453292519943295f)

typedef struct
{
    float duty0;
    float duty1;
    float duty2;
    float duty3;
    float duty4;
    float duty5;
} ARM_SV_Duties_t;

/*
 * 为兼容原有协议仍保留 6 个软件槽位。应用层应通过下面的角度接口修改目标，
 * 不要直接修改 PWM 占空比；直接写占空比会绕过角度限位和平滑轨迹保护。
 */
extern volatile float motor_radians[ARM_SV_COUNT];
extern ARM_SV_Duties_t duties_tx;

extern volatile float arm_sv_target_radian_debug[ARM_SV_COUNT];
extern volatile float arm_sv_position_radian_debug[ARM_SV_COUNT];
extern volatile float arm_sv_velocity_radian_debug[ARM_SV_COUNT];
extern volatile float arm_sv_duty_debug[ARM_SV_COUNT];
extern volatile uint32_t arm_sv_target_sequence_debug;
extern volatile uint32_t arm_sv_i2c_write_count_debug;
extern volatile uint32_t arm_sv_i2c_error_count_debug;
extern volatile uint32_t arm_sv_update_count_debug;
extern volatile uint32_t arm_sv_last_update_interval_ms_debug;
extern volatile uint32_t arm_sv_max_update_interval_ms_debug;
extern volatile uint32_t arm_sv_deadline_miss_count_debug;
extern volatile uint32_t arm_sv_last_pwm_write_duration_ms_debug;
extern volatile uint32_t arm_sv_max_pwm_write_duration_ms_debug;
extern volatile uint32_t arm_sv_pwm_frame_inflight_debug;
extern volatile uint32_t arm_sv_pwm_retry_count_debug;
extern volatile uint32_t arm_sv_pwm_submit_busy_count_debug;
extern volatile uint32_t arm_sv_trajectory_active_debug;
extern volatile uint32_t arm_sv_trajectory_duration_ms_debug;
extern volatile uint32_t arm_sv_trajectory_replan_count_debug;
extern volatile uint32_t arm_sv_limit_clamp_count_debug;
extern volatile uint32_t arm_sv_simple_motion_active_debug;
extern volatile uint32_t arm_sv_simple_motion_step_debug;
extern volatile uint32_t arm_sv_simple_motion_servo_debug;
extern volatile uint32_t arm_sv_simple_motion_phase_debug;
extern volatile uint32_t arm_sv_simple_motion_done_debug;
extern volatile float arm_sv_zero_trim_target_duty[ARM_SV_COUNT];
extern volatile float arm_sv_zero_trim_current_duty_debug[ARM_SV_COUNT];
extern volatile uint32_t arm_sv_zero_trim_active_mask_debug;
extern volatile uint32_t arm_sv_zero_trim_update_count_debug;
extern volatile uint32_t arm_sv_zero_trim_reject_count_debug;
extern volatile float arm_sv_pose_target_radian[ARM_SV_COUNT];
extern volatile uint32_t arm_sv_pose_command_sequence;
extern volatile uint32_t arm_sv_pose_applied_sequence_debug;
extern volatile uint32_t arm_sv_pose_reject_count_debug;
extern volatile uint32_t arm_sv_pose_duration_ms_debug;
extern volatile uint32_t arm_sv_pose_motion_state_debug;
extern volatile uint32_t arm_sv_pose_interrupt_count_debug;
extern volatile uint32_t arm_sv_pose_brake_duration_ms_debug;
extern volatile uint32_t arm_sv_pose_brake_fallback_count_debug;
/* 自动测试状态：0等待，1向前抓取中，2抓取保持，3放下中，4完成，5中止。 */
extern volatile uint32_t arm_sv_user_action_state_debug;
extern volatile uint32_t arm_sv_user_action_step_debug;
extern volatile uint32_t arm_sv_user_action_elapsed_ms_debug;
extern volatile uint32_t arm_sv_user_action_done_debug;
extern volatile uint32_t arm_sv_control_mode_request_debug;
extern volatile uint32_t arm_sv_control_mode_active_debug;
extern volatile uint32_t arm_sv_control_mode_pending_debug;
extern volatile uint32_t arm_sv_control_mode_switch_count_debug;
extern volatile uint32_t arm_sv_control_command_reject_count_debug;
extern volatile uint32_t arm_sv_pca_fault_hold_active_debug;
extern volatile uint32_t arm_sv_pca_fault_hold_total_ms_debug;
extern volatile uint32_t arm_sv_pca_recovery_resync_count_debug;

float radian_to_duty_270(float radian);
void set_motor_radians_270(const float radians[ARM_SV_COUNT]);

void ARM_SV_Init(float freq);
void ARM_SV_Tx_Rx(void);

/*
 * 请求切换舵机控制权。函数只提交请求，当前轨迹停稳后由5 ms舵机任务执行切换，
 * 因此不会在两套轨迹之间直接跳变。返回1表示请求有效，0表示模式值无效。
 */
uint8_t ARM_SV_RequestControlMode(ARM_SV_ControlMode_t mode);
ARM_SV_ControlMode_t ARM_SV_GetControlMode(void);

void ARM_SV_SetDuty(uint8_t sv_id, float duty);
void ARM_SV_SetDuty0(float duty);
void ARM_SV_SetDuty1(float duty);
void ARM_SV_SetDuty2(float duty);
void ARM_SV_SetDuty3(float duty);
void ARM_SV_SetDuty4(float duty);
void ARM_SV_SetDuty5(float duty);

float ARM_SV_GetDuty(uint8_t sv_id);
float ARM_SV_GetDuty0(void);
float ARM_SV_GetDuty1(void);
float ARM_SV_GetDuty2(void);
float ARM_SV_GetDuty3(void);
float ARM_SV_GetDuty4(void);
float ARM_SV_GetDuty5(void);

void ARM_SV_SetAllDuties(const float *duties);
ARM_SV_Duties_t ARM_SV_GetAllDuties(void);

/* 使用以下渐变接口前，必须先请求 ARM_SV_CONTROL_MODE_EXTERNAL_RAMP。 */
void ARM_SV_SetRampTarget(uint8_t sv_id, float target_radian);
void ARM_SV_AdjustRampTarget(uint8_t sv_id, float delta_radian);
void ARM_SV_SetAllRampTargets(const float *radians);
void ARM_SV_RampUpdate(float dt);

/*
 * 设置 1~4 号舵机的同步姿态目标，输入单位为弧度：
 * radians[0] = 实体 1 号，radians[1] = 实体 2 号，
 * radians[2] = 实体 3 号，radians[3] = 实体 4 号。
 * 常用角度：30°=0.523599f，45°=0.785398f，90°=1.570796f；
 * 负数表示反方向。函数只提交目标，实际运动由 5 ms 控制任务平滑完成。
 * 外部调用前必须先请求 ARM_SV_CONTROL_MODE_EXTERNAL_POSE；自动动作使用内部
 * 提交路径，不受后台遥控器或PC任务影响。
 * 运动中提交不同目标时，控制器先用受限同步制动曲线停稳，再执行最新目标；
 * 不会等待原动作全部完成，也不会直接把当前速度清零。
 */
void ARM_SV_SetPoseTargets(const float radians[ARM_SV_POSE_COUNT]);

/*
 * 推荐给动作代码使用的角度制接口。数组顺序仍为实体1、2、3、4号舵机，
 * 可直接写 float pose_deg[4] = {0, 90, 0, 45}; 无需手工换算弧度。
 * 当前1~4号关节会被安全限制在 -90°~+90°范围内。
 */
void ARM_SV_SetPoseTargetsDeg(const float degrees[ARM_SV_POSE_COUNT]);

#endif
