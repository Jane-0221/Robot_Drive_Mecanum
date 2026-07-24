#include "mecanum_wheel.h"

#include <math.h>

#include "IMU_updata.h"
#include "Robstride04.h"
#include "chassis.h"
#include "fdcan.h"
#include "main.h"
#include "pid.h"
#include "ramp_generator.h"

#define MECANUM_CAN_HANDLE (&hfdcan3)

#define MECANUM_MOTOR_LF_ID 0x01U
#define MECANUM_MOTOR_RF_ID 0x02U
#define MECANUM_MOTOR_RB_ID 0x03U
#define MECANUM_MOTOR_LB_ID 0x04U

#define MECANUM_CHASSIS_LENGTH_M 0.376f
#define MECANUM_CHASSIS_WIDTH_M  0.343f
#define MECANUM_WHEEL_RADIUS_M   0.075f
#define MECANUM_ROTATION_RADIUS_M ((MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M) * 0.5f)

/*
 * Motors 2 and 3 are mounted on the right side of the chassis, so their
 * positive motor rotation is opposite to the wheel-speed convention used by
 * the mecanum kinematics.
 */
#define MECANUM_DIR_LF 1.0f
#define MECANUM_DIR_RF -1.0f
#define MECANUM_DIR_RB -1.0f
#define MECANUM_DIR_LB 1.0f

#define MECANUM_CURRENT_LIMIT 8.0f
#define MECANUM_ACCEL_LIMIT 8.0f
#define MECANUM_SPEED_LIMIT 20.0f

#define MECANUM_RAMP_INTERVAL_MS 1U
#define MECANUM_RAMP_LINEAR_ACCEL 0.95f
#define MECANUM_RAMP_LINEAR_DECEL 1.60f
#define MECANUM_RAMP_YAW_ACCEL 0.90f
#define MECANUM_RAMP_YAW_DECEL 1.50f
#define MECANUM_RAMP_LINEAR_LIMIT 1.0f
#define MECANUM_RAMP_YAW_LIMIT 1.5f
#define MECANUM_MOTOR_TX_INTERVAL_MS 2U
#define MECANUM_MOTOR_TX_COUNT 4U
#define MECANUM_MOTOR_RUNNING_PATTERN 2U
#define MECANUM_MOTOR_FEEDBACK_TIMEOUT_MS 250U
#define MECANUM_MOTOR_ENABLE_RETRY_INTERVAL_MS 200U
#define MECANUM_MOTOR_LF_INDEX 0U
#define MECANUM_MOTOR_RF_INDEX 1U
#define MECANUM_MOTOR_RB_INDEX 2U
#define MECANUM_MOTOR_LB_INDEX 3U

#define MECANUM_YAW_HOLD_KP 1.20f
#define MECANUM_YAW_HOLD_KI 0.0f
#define MECANUM_YAW_HOLD_KD 0.04f
#define MECANUM_YAW_HOLD_LIM_OUT 0.45f
#define MECANUM_YAW_HOLD_LIM_I_OUT 0.0f
#define MECANUM_YAW_FILTER_ALPHA 0.12f
#define MECANUM_YAW_CORRECTION_LIMIT 0.45f
#define MECANUM_WHEEL_CMD_FILTER_ALPHA 0.55f
#define MECANUM_WHEEL_CMD_DEADBAND 0.05f
#define MECANUM_TRANSLATION_ENABLE_THRESHOLD 0.05f
#define MECANUM_YAW_CMD_DEADBAND 0.05f
#define MECANUM_SINGLE_WHEEL_TEST_DEFAULT_DURATION_MS 1200U
#define MECANUM_SINGLE_WHEEL_TEST_DEFAULT_SPEED 0.8f
#define MECANUM_SINGLE_WHEEL_TEST_LF 1U
#define MECANUM_SINGLE_WHEEL_TEST_RF 2U
#define MECANUM_SINGLE_WHEEL_TEST_RB 3U
#define MECANUM_SINGLE_WHEEL_TEST_LB 4U
#define MECANUM_VECTOR_TEST_DEFAULT_DURATION_MS 1000U

volatile Mecanum_Wheel_Debug_t mecanum_debug = {0};
volatile uint32_t mecanum_single_wheel_test_request_debug = 0U;
volatile uint32_t mecanum_single_wheel_test_active_debug = 0U;
volatile uint32_t mecanum_single_wheel_test_duration_ms_debug = MECANUM_SINGLE_WHEEL_TEST_DEFAULT_DURATION_MS;
volatile uint32_t mecanum_single_wheel_test_end_tick_debug = 0U;
volatile float mecanum_single_wheel_test_speed_debug = MECANUM_SINGLE_WHEEL_TEST_DEFAULT_SPEED;
volatile uint32_t mecanum_vector_test_request_debug = 0U;
volatile uint32_t mecanum_vector_test_active_debug = 0U;
volatile uint32_t mecanum_vector_test_duration_ms_debug = MECANUM_VECTOR_TEST_DEFAULT_DURATION_MS;
volatile uint32_t mecanum_vector_test_end_tick_debug = 0U;
volatile float mecanum_vector_test_x_debug = 0.0f;
volatile float mecanum_vector_test_y_debug = 0.0f;
volatile float mecanum_vector_test_w_debug = 0.0f;
volatile uint32_t mecanum_rx_ext_total_debug = 0U;
volatile uint32_t mecanum_rx_last_ext_id_debug = 0U;
volatile uint32_t mecanum_rx_last_target_id_debug = 0U;
volatile uint32_t mecanum_rx_target_count_debug[16] = {0U};

static RobStride_Motor_t mecanum_motor_lf;
static RobStride_Motor_t mecanum_motor_rf;
static RobStride_Motor_t mecanum_motor_rb;
static RobStride_Motor_t mecanum_motor_lb;
static uint8_t mecanum_initialized = 0U;
static RampGenerator mecanum_x_ramp;
static RampGenerator mecanum_y_ramp;
static RampGenerator mecanum_w_ramp;
static uint8_t mecanum_tx_slot = 0U;
static uint32_t mecanum_tx_last_tick_ms = 0U;
static volatile uint32_t mecanum_feedback_tick_ms[MECANUM_MOTOR_TX_COUNT] = {0};
static uint32_t mecanum_enable_retry_tick_ms[MECANUM_MOTOR_TX_COUNT] = {0};
static uint32_t mecanum_enable_retry_count[MECANUM_MOTOR_TX_COUNT] = {0};
static float mecanum_cmd_lf_filtered = 0.0f;
static float mecanum_cmd_rf_filtered = 0.0f;
static float mecanum_cmd_rb_filtered = 0.0f;
static float mecanum_cmd_lb_filtered = 0.0f;
static uint8_t mecanum_cmd_filter_valid = 0U;
static float mecanum_tx_cmd_lf = 0.0f;
static float mecanum_tx_cmd_rf = 0.0f;
static float mecanum_tx_cmd_rb = 0.0f;
static float mecanum_tx_cmd_lb = 0.0f;

static pid_t mecanum_yaw_hold_pid;
static float mecanum_yaw_target = 0.0f;
static float mecanum_yaw_filtered = 0.0f;
static uint8_t mecanum_yaw_lock_valid = 0U;
static uint8_t mecanum_yaw_filter_valid = 0U;

static float Mecanum_Wheel_ClampFloat(float value, float limit)
{
    if (value != value)
    {
        return 0.0f;
    }
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

static float Mecanum_Wheel_ApplyDeadband(float value, float deadband)
{
    if (value != value)
    {
        return 0.0f;
    }
    if (fabsf(value) < deadband)
    {
        return 0.0f; 
    }
    return value;
}

static float Mecanum_Wheel_GetFilteredYaw(void)
{
    float yaw_now = IMU_data.AHRS.yaw_rad_cnt;

    if (yaw_now != yaw_now)
    {
        return mecanum_yaw_filtered;
    }

    if (mecanum_yaw_filter_valid == 0U)
    {
        mecanum_yaw_filtered = yaw_now;
        mecanum_yaw_filter_valid = 1U;
    }
    else
    {
        mecanum_yaw_filtered +=
            MECANUM_YAW_FILTER_ALPHA * (yaw_now - mecanum_yaw_filtered);
    }

    return mecanum_yaw_filtered;
}

static void Mecanum_Wheel_ResetYawPid(void)
{
    mecanum_yaw_hold_pid.set = 0.0f;
    mecanum_yaw_hold_pid.err = 0.0f;
    mecanum_yaw_hold_pid.err_last = 0.0f;
    mecanum_yaw_hold_pid.diff = 0.0f;
    mecanum_yaw_hold_pid.p_out = 0.0f;
    mecanum_yaw_hold_pid.i_out = 0.0f;
    mecanum_yaw_hold_pid.d_out = 0.0f;
    mecanum_yaw_hold_pid.total_out = 0.0f;
}

static void Mecanum_Wheel_InitMotor(RobStride_Motor_t *motor, uint8_t can_id)
{
    RobStride_Motor_Init(motor, can_id, false);
    Get_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7005, Speed_control_mode, 'j');
    Enable_Motor(motor, MECANUM_CAN_HANDLE);
    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7018, MECANUM_CURRENT_LIMIT, 'p');
    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7022, MECANUM_ACCEL_LIMIT, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(motor, MECANUM_CAN_HANDLE, 0x01);
}

static void Mecanum_Wheel_ResetMotorRuntime(uint32_t tick_ms)
{
    uint8_t i;

    for (i = 0U; i < MECANUM_MOTOR_TX_COUNT; i++)
    {
        mecanum_feedback_tick_ms[i] = 0U;
        mecanum_enable_retry_tick_ms[i] = tick_ms;
        mecanum_enable_retry_count[i] = 0U;
    }
}

static uint8_t Mecanum_Wheel_FeedbackFresh(uint8_t index, uint32_t tick_ms)
{
    if (mecanum_feedback_tick_ms[index] == 0U)
    {
        return 0U;
    }

    if ((tick_ms - mecanum_feedback_tick_ms[index]) > MECANUM_MOTOR_FEEDBACK_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t Mecanum_Wheel_MotorReady(RobStride_Motor_t *motor, uint8_t index, uint32_t tick_ms)
{
    if (Mecanum_Wheel_FeedbackFresh(index, tick_ms) == 0U)
    {
        return 0U;
    }

    if (motor->Pos_Info.pattern != MECANUM_MOTOR_RUNNING_PATTERN)
    {
        return 0U;
    }

    return 1U;
}

static void Mecanum_Wheel_RetryEnableMotor(RobStride_Motor_t *motor, uint8_t index, uint32_t tick_ms)
{
    if ((tick_ms - mecanum_enable_retry_tick_ms[index]) < MECANUM_MOTOR_ENABLE_RETRY_INTERVAL_MS)
    {
        return;
    }

    mecanum_enable_retry_tick_ms[index] = tick_ms;
    mecanum_enable_retry_count[index]++;

    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7005, Speed_control_mode, 'j');
    Enable_Motor(motor, MECANUM_CAN_HANDLE);
    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7018, MECANUM_CURRENT_LIMIT, 'p');
    Set_RobStride_Motor_parameter(motor, MECANUM_CAN_HANDLE, 0x7022, MECANUM_ACCEL_LIMIT, 'p');
    RobStride_Motor_ProactiveEscalationSet(motor, MECANUM_CAN_HANDLE, 0x01);
}

static uint8_t Mecanum_Wheel_EnsureMotorReady(RobStride_Motor_t *motor, uint8_t index, uint32_t tick_ms)
{
    if (Mecanum_Wheel_MotorReady(motor, index, tick_ms) != 0U)
    {
        return 1U;
    }

    Mecanum_Wheel_RetryEnableMotor(motor, index, tick_ms);
    return 0U;
}

static uint8_t Mecanum_Wheel_GetReadyMask(uint32_t tick_ms)
{
    uint8_t ready_mask = 0U;

    if (Mecanum_Wheel_MotorReady(&mecanum_motor_lf, MECANUM_MOTOR_LF_INDEX, tick_ms) != 0U)
    {
        ready_mask |= 0x01U;
    }
    if (Mecanum_Wheel_MotorReady(&mecanum_motor_rf, MECANUM_MOTOR_RF_INDEX, tick_ms) != 0U)
    {
        ready_mask |= 0x02U;
    }
    if (Mecanum_Wheel_MotorReady(&mecanum_motor_rb, MECANUM_MOTOR_RB_INDEX, tick_ms) != 0U)
    {
        ready_mask |= 0x04U;
    }
    if (Mecanum_Wheel_MotorReady(&mecanum_motor_lb, MECANUM_MOTOR_LB_INDEX, tick_ms) != 0U)
    {
        ready_mask |= 0x08U;
    }

    return ready_mask;
}

static void Mecanum_Wheel_Limit(float *cmd_lf, float *cmd_rf, float *cmd_rb, float *cmd_lb)
{
    float max_abs = fabsf(*cmd_lf);
    float scale;

    if (fabsf(*cmd_rf) > max_abs)
    {
        max_abs = fabsf(*cmd_rf);
    }
    if (fabsf(*cmd_rb) > max_abs)
    {
        max_abs = fabsf(*cmd_rb);
    }
    if (fabsf(*cmd_lb) > max_abs)
    {
        max_abs = fabsf(*cmd_lb);
    }

    mecanum_debug.wheel_raw_max_abs = max_abs;
    mecanum_debug.wheel_limit_scale = 1.0f;

    if (max_abs <= MECANUM_SPEED_LIMIT)
    {
        return;
    }

    scale = MECANUM_SPEED_LIMIT / max_abs;
    mecanum_debug.wheel_limit_scale = scale;
    *cmd_lf *= scale;
    *cmd_rf *= scale;
    *cmd_rb *= scale;
    *cmd_lb *= scale;
}

static void Mecanum_Wheel_UpdateRamps(uint32_t tick_ms, float *x_cmd, float *y_cmd, float *w_cmd)
{
    RampGenerator_SetTarget(&mecanum_x_ramp,
                            Mecanum_Wheel_ClampFloat(x, MECANUM_RAMP_LINEAR_LIMIT));
    RampGenerator_SetTarget(&mecanum_y_ramp,
                            Mecanum_Wheel_ClampFloat(y, MECANUM_RAMP_LINEAR_LIMIT));
    RampGenerator_SetTarget(&mecanum_w_ramp,
                            Mecanum_Wheel_ClampFloat(w, MECANUM_RAMP_YAW_LIMIT));

    RampGenerator_Update(&mecanum_x_ramp, tick_ms);
    RampGenerator_Update(&mecanum_y_ramp, tick_ms);
    RampGenerator_Update(&mecanum_w_ramp, tick_ms);

    *x_cmd = RampGenerator_GetCurrent(&mecanum_x_ramp);
    *y_cmd = RampGenerator_GetCurrent(&mecanum_y_ramp);
    *w_cmd = RampGenerator_GetCurrent(&mecanum_w_ramp);
}

static void Mecanum_Wheel_ApplyStraightCorrection(float x_cmd,
                                                  float y_cmd,
                                                  float w_target,
                                                  float *w_cmd,
                                                  float *yaw_now,
                                                  float *yaw_correction)
{
    float v_cmd;

    *yaw_now = Mecanum_Wheel_GetFilteredYaw();
    *yaw_correction = 0.0f;
    v_cmd = sqrtf(x_cmd * x_cmd + y_cmd * y_cmd);

    if ((v_cmd > MECANUM_TRANSLATION_ENABLE_THRESHOLD) &&
        (fabsf(w_target) < MECANUM_YAW_CMD_DEADBAND) &&
        (fabsf(*w_cmd) < MECANUM_YAW_CMD_DEADBAND))
    {
        if (mecanum_yaw_lock_valid == 0U)
        {
            mecanum_yaw_target = *yaw_now;
            Mecanum_Wheel_ResetYawPid();
        }

        *yaw_correction = Mecanum_Wheel_ClampFloat(
            pid_cal(&mecanum_yaw_hold_pid, *yaw_now, mecanum_yaw_target),
            MECANUM_YAW_CORRECTION_LIMIT);
        *w_cmd = Mecanum_Wheel_ClampFloat(*w_cmd + *yaw_correction,
                                          MECANUM_RAMP_YAW_LIMIT);
        mecanum_yaw_lock_valid = 1U;
    }
    else
    {
        mecanum_yaw_target = *yaw_now;
        Mecanum_Wheel_ResetYawPid();
        mecanum_yaw_lock_valid = 0U;
    }
}

static void Mecanum_Wheel_CalcWheelCommands(float x_cmd,
                                            float y_cmd,
                                            float w_cmd,
                                            float *cmd_lf,
                                            float *cmd_rf,
                                            float *cmd_rb,
                                            float *cmd_lb)
{
    float rotate = MECANUM_ROTATION_RADIUS_M * w_cmd;

    *cmd_lf = (x_cmd + y_cmd + rotate) / MECANUM_WHEEL_RADIUS_M;
    *cmd_rf = (x_cmd - y_cmd - rotate) / MECANUM_WHEEL_RADIUS_M;
    *cmd_rb = (x_cmd + y_cmd - rotate) / MECANUM_WHEEL_RADIUS_M;
    *cmd_lb = (x_cmd - y_cmd + rotate) / MECANUM_WHEEL_RADIUS_M;

    *cmd_lf *= MECANUM_DIR_LF;
    *cmd_rf *= MECANUM_DIR_RF;
    *cmd_rb *= MECANUM_DIR_RB;
    *cmd_lb *= MECANUM_DIR_LB;

    Mecanum_Wheel_Limit(cmd_lf, cmd_rf, cmd_rb, cmd_lb);
}

static void Mecanum_Wheel_FilterWheelCommands(float *cmd_lf,
                                              float *cmd_rf,
                                              float *cmd_rb,
                                              float *cmd_lb)
{
    *cmd_lf = Mecanum_Wheel_ApplyDeadband(*cmd_lf, MECANUM_WHEEL_CMD_DEADBAND);
    *cmd_rf = Mecanum_Wheel_ApplyDeadband(*cmd_rf, MECANUM_WHEEL_CMD_DEADBAND);
    *cmd_rb = Mecanum_Wheel_ApplyDeadband(*cmd_rb, MECANUM_WHEEL_CMD_DEADBAND);
    *cmd_lb = Mecanum_Wheel_ApplyDeadband(*cmd_lb, MECANUM_WHEEL_CMD_DEADBAND);

    if (mecanum_cmd_filter_valid == 0U)
    {
        mecanum_cmd_lf_filtered = *cmd_lf;
        mecanum_cmd_rf_filtered = *cmd_rf;
        mecanum_cmd_rb_filtered = *cmd_rb;
        mecanum_cmd_lb_filtered = *cmd_lb;
        mecanum_cmd_filter_valid = 1U;
    }
    else
    {
        mecanum_cmd_lf_filtered += MECANUM_WHEEL_CMD_FILTER_ALPHA * (*cmd_lf - mecanum_cmd_lf_filtered);
        mecanum_cmd_rf_filtered += MECANUM_WHEEL_CMD_FILTER_ALPHA * (*cmd_rf - mecanum_cmd_rf_filtered);
        mecanum_cmd_rb_filtered += MECANUM_WHEEL_CMD_FILTER_ALPHA * (*cmd_rb - mecanum_cmd_rb_filtered);
        mecanum_cmd_lb_filtered += MECANUM_WHEEL_CMD_FILTER_ALPHA * (*cmd_lb - mecanum_cmd_lb_filtered);
    }

    mecanum_cmd_lf_filtered = Mecanum_Wheel_ApplyDeadband(mecanum_cmd_lf_filtered, MECANUM_WHEEL_CMD_DEADBAND);
    mecanum_cmd_rf_filtered = Mecanum_Wheel_ApplyDeadband(mecanum_cmd_rf_filtered, MECANUM_WHEEL_CMD_DEADBAND);
    mecanum_cmd_rb_filtered = Mecanum_Wheel_ApplyDeadband(mecanum_cmd_rb_filtered, MECANUM_WHEEL_CMD_DEADBAND);
    mecanum_cmd_lb_filtered = Mecanum_Wheel_ApplyDeadband(mecanum_cmd_lb_filtered, MECANUM_WHEEL_CMD_DEADBAND);

    *cmd_lf = mecanum_cmd_lf_filtered;
    *cmd_rf = mecanum_cmd_rf_filtered;
    *cmd_rb = mecanum_cmd_rb_filtered;
    *cmd_lb = mecanum_cmd_lb_filtered;
}

static void Mecanum_Wheel_ApplySingleWheelTest(uint32_t tick_ms,
                                               float *cmd_lf,
                                               float *cmd_rf,
                                               float *cmd_rb,
                                               float *cmd_lb)
{
    uint32_t request = mecanum_single_wheel_test_request_debug;
    uint32_t duration_ms = mecanum_single_wheel_test_duration_ms_debug;
    float speed = mecanum_single_wheel_test_speed_debug;

    if (request != 0U)
    {
        mecanum_single_wheel_test_request_debug = 0U;
        if ((request >= MECANUM_SINGLE_WHEEL_TEST_LF) &&
            (request <= MECANUM_SINGLE_WHEEL_TEST_LB))
        {
            if (duration_ms == 0U)
            {
                duration_ms = MECANUM_SINGLE_WHEEL_TEST_DEFAULT_DURATION_MS;
            }
            if (speed != speed)
            {
                speed = 0.0f;
            }
            mecanum_single_wheel_test_active_debug = request;
            mecanum_single_wheel_test_end_tick_debug = tick_ms + duration_ms;
        }
        else
        {
            mecanum_single_wheel_test_active_debug = 0U;
            mecanum_single_wheel_test_end_tick_debug = 0U;
        }
    }

    if (mecanum_single_wheel_test_active_debug == 0U)
    {
        return;
    }

    if ((int32_t)(tick_ms - mecanum_single_wheel_test_end_tick_debug) >= 0)
    {
        mecanum_single_wheel_test_active_debug = 0U;
        mecanum_single_wheel_test_end_tick_debug = 0U;
        return;
    }

    *cmd_lf = 0.0f;
    *cmd_rf = 0.0f;
    *cmd_rb = 0.0f;
    *cmd_lb = 0.0f;

    switch (mecanum_single_wheel_test_active_debug)
    {
    case MECANUM_SINGLE_WHEEL_TEST_LF:
        *cmd_lf = speed;
        break;

    case MECANUM_SINGLE_WHEEL_TEST_RF:
        *cmd_rf = speed;
        break;

    case MECANUM_SINGLE_WHEEL_TEST_RB:
        *cmd_rb = speed;
        break;

    case MECANUM_SINGLE_WHEEL_TEST_LB:
        *cmd_lb = speed;
        break;

    default:
        mecanum_single_wheel_test_active_debug = 0U;
        mecanum_single_wheel_test_end_tick_debug = 0U;
        break;
    }
}

static uint8_t Mecanum_Wheel_ApplyVectorTest(uint32_t tick_ms,
                                             float *x_cmd,
                                             float *y_cmd,
                                             float *w_cmd)
{
    uint32_t request = mecanum_vector_test_request_debug;
    uint32_t duration_ms = mecanum_vector_test_duration_ms_debug;

    if (request != 0U)
    {
        mecanum_vector_test_request_debug = 0U;
        if (duration_ms == 0U)
        {
            duration_ms = MECANUM_VECTOR_TEST_DEFAULT_DURATION_MS;
        }
        mecanum_vector_test_active_debug = request;
        mecanum_vector_test_end_tick_debug = tick_ms + duration_ms;
    }

    if (mecanum_vector_test_active_debug == 0U)
    {
        return 0U;
    }

    if ((int32_t)(tick_ms - mecanum_vector_test_end_tick_debug) >= 0)
    {
        mecanum_vector_test_active_debug = 0U;
        mecanum_vector_test_end_tick_debug = 0U;
        return 0U;
    }

    *x_cmd = mecanum_vector_test_x_debug;
    *y_cmd = mecanum_vector_test_y_debug;
    *w_cmd = mecanum_vector_test_w_debug;

    if ((*x_cmd != *x_cmd) || (*y_cmd != *y_cmd) || (*w_cmd != *w_cmd))
    {
        *x_cmd = 0.0f;
        *y_cmd = 0.0f;
        *w_cmd = 0.0f;
    }

    return 1U;
}

static void Mecanum_Wheel_TxScheduled(float cmd_lf,
                                      float cmd_rf,
                                      float cmd_rb,
                                      float cmd_lb,
                                      uint32_t tick_ms)
{
    if ((tick_ms - mecanum_tx_last_tick_ms) < MECANUM_MOTOR_TX_INTERVAL_MS)
    {
        return;
    }

    mecanum_tx_last_tick_ms = tick_ms;
    mecanum_debug.tx_last_tick_ms = tick_ms;
    mecanum_debug.tx_slot = mecanum_tx_slot;

    if (mecanum_tx_slot == 0U)
    {
        mecanum_tx_cmd_lf = cmd_lf;
        mecanum_tx_cmd_rf = cmd_rf;
        mecanum_tx_cmd_rb = cmd_rb;
        mecanum_tx_cmd_lb = cmd_lb;
    }

    switch (mecanum_tx_slot)
    {
    case 0U:
        if (Mecanum_Wheel_EnsureMotorReady(&mecanum_motor_lf, MECANUM_MOTOR_LF_INDEX, tick_ms) != 0U)
        {
            RobStride_Motor_Speed_control(&mecanum_motor_lf, MECANUM_CAN_HANDLE, mecanum_tx_cmd_lf, MECANUM_CURRENT_LIMIT, MECANUM_ACCEL_LIMIT);
            mecanum_debug.tx_count_lf++;
        }
        break;

    case 1U:
        if (Mecanum_Wheel_EnsureMotorReady(&mecanum_motor_rf, MECANUM_MOTOR_RF_INDEX, tick_ms) != 0U)
        {
            RobStride_Motor_Speed_control(&mecanum_motor_rf, MECANUM_CAN_HANDLE, mecanum_tx_cmd_rf, MECANUM_CURRENT_LIMIT, MECANUM_ACCEL_LIMIT);
            mecanum_debug.tx_count_rf++;
        }
        break;

    case 2U:
        if (Mecanum_Wheel_EnsureMotorReady(&mecanum_motor_rb, MECANUM_MOTOR_RB_INDEX, tick_ms) != 0U)
        {
            RobStride_Motor_Speed_control(&mecanum_motor_rb, MECANUM_CAN_HANDLE, mecanum_tx_cmd_rb, MECANUM_CURRENT_LIMIT, MECANUM_ACCEL_LIMIT);
            mecanum_debug.tx_count_rb++;
        }
        break;

    default:
        if (Mecanum_Wheel_EnsureMotorReady(&mecanum_motor_lb, MECANUM_MOTOR_LB_INDEX, tick_ms) != 0U)
        {
            RobStride_Motor_Speed_control(&mecanum_motor_lb, MECANUM_CAN_HANDLE, mecanum_tx_cmd_lb, MECANUM_CURRENT_LIMIT, MECANUM_ACCEL_LIMIT);
            mecanum_debug.tx_count_lb++;
        }
        break;
    }

    mecanum_tx_slot++;
    if (mecanum_tx_slot >= MECANUM_MOTOR_TX_COUNT)
    {
        mecanum_tx_slot = 0U;
    }
}

void Mecanum_Wheel_Init(void)
{
    uint32_t tick_ms;

    tick_ms = HAL_GetTick();
    mecanum_debug = (Mecanum_Wheel_Debug_t){0};
    mecanum_tx_slot = 0U;
    mecanum_tx_last_tick_ms = tick_ms - MECANUM_MOTOR_TX_INTERVAL_MS;
    mecanum_yaw_filter_valid = 0U;
    mecanum_cmd_filter_valid = 0U;
    mecanum_tx_cmd_lf = 0.0f;
    mecanum_tx_cmd_rf = 0.0f;
    mecanum_tx_cmd_rb = 0.0f;
    mecanum_tx_cmd_lb = 0.0f;
    Mecanum_Wheel_ResetMotorRuntime(tick_ms);
    RampGenerator_Init(&mecanum_x_ramp, MECANUM_RAMP_INTERVAL_MS, MECANUM_RAMP_LINEAR_ACCEL, MECANUM_RAMP_LINEAR_DECEL, MECANUM_RAMP_LINEAR_LIMIT);
    RampGenerator_Init(&mecanum_y_ramp, MECANUM_RAMP_INTERVAL_MS, MECANUM_RAMP_LINEAR_ACCEL, MECANUM_RAMP_LINEAR_DECEL, MECANUM_RAMP_LINEAR_LIMIT);
    RampGenerator_Init(&mecanum_w_ramp, MECANUM_RAMP_INTERVAL_MS, MECANUM_RAMP_YAW_ACCEL, MECANUM_RAMP_YAW_DECEL, MECANUM_RAMP_YAW_LIMIT);

    pid_set(&mecanum_yaw_hold_pid,
            MECANUM_YAW_HOLD_KP,
            MECANUM_YAW_HOLD_KI,
            MECANUM_YAW_HOLD_KD,
            MECANUM_YAW_HOLD_LIM_OUT,
            MECANUM_YAW_HOLD_LIM_I_OUT);
    mecanum_yaw_target = Mecanum_Wheel_GetFilteredYaw();
    mecanum_yaw_lock_valid = 1U;
    Mecanum_Wheel_ResetYawPid();

    Mecanum_Wheel_InitMotor(&mecanum_motor_lf, MECANUM_MOTOR_LF_ID);
    Mecanum_Wheel_InitMotor(&mecanum_motor_rf, MECANUM_MOTOR_RF_ID);
    Mecanum_Wheel_InitMotor(&mecanum_motor_rb, MECANUM_MOTOR_RB_ID);
    Mecanum_Wheel_InitMotor(&mecanum_motor_lb, MECANUM_MOTOR_LB_ID);

    mecanum_initialized = 1U;
}

void Mecanum_Wheel_Update(void)
{
    float x_cmd;
    float y_cmd;
    float w_cmd;
    float w_cmd_in;
    float yaw_now;
    float yaw_correction;
    float cmd_lf;
    float cmd_rf;
    float cmd_rb;
    float cmd_lb;
    float w_target;
    uint32_t tick_ms;
    uint8_t vector_test_active;

    if (mecanum_initialized == 0U)
    {
        return;
    }

    tick_ms = HAL_GetTick();
    w_target = w;
    Mecanum_Wheel_UpdateRamps(tick_ms, &x_cmd, &y_cmd, &w_cmd);
    vector_test_active = Mecanum_Wheel_ApplyVectorTest(tick_ms, &x_cmd, &y_cmd, &w_cmd);
    w_cmd_in = w_cmd;

    if (vector_test_active == 0U)
    {
        Mecanum_Wheel_ApplyStraightCorrection(x_cmd, y_cmd, w_target, &w_cmd, &yaw_now, &yaw_correction);
    }
    else
    {
        yaw_now = Mecanum_Wheel_GetFilteredYaw();
        yaw_correction = 0.0f;
        mecanum_yaw_target = yaw_now;
        mecanum_yaw_lock_valid = 0U;
        Mecanum_Wheel_ResetYawPid();
    }

    Mecanum_Wheel_CalcWheelCommands(x_cmd, y_cmd, w_cmd, &cmd_lf, &cmd_rf, &cmd_rb, &cmd_lb);
    Mecanum_Wheel_FilterWheelCommands(&cmd_lf, &cmd_rf, &cmd_rb, &cmd_lb);
    Mecanum_Wheel_ApplySingleWheelTest(tick_ms, &cmd_lf, &cmd_rf, &cmd_rb, &cmd_lb);

    mecanum_debug.x_cmd = x_cmd;
    mecanum_debug.y_cmd = y_cmd;
    mecanum_debug.w_cmd_in = w_cmd_in;
    mecanum_debug.w_cmd_out = w_cmd;
    mecanum_debug.yaw_now = yaw_now;
    mecanum_debug.yaw_target = mecanum_yaw_target;
    mecanum_debug.yaw_correction = yaw_correction;
    mecanum_debug.wheel_cmd_lf = cmd_lf;
    mecanum_debug.wheel_cmd_rf = cmd_rf;
    mecanum_debug.wheel_cmd_rb = cmd_rb;
    mecanum_debug.wheel_cmd_lb = cmd_lb;
    mecanum_debug.fb_speed_lf = mecanum_motor_lf.Pos_Info.Speed;
    mecanum_debug.fb_speed_rf = mecanum_motor_rf.Pos_Info.Speed;
    mecanum_debug.fb_speed_rb = mecanum_motor_rb.Pos_Info.Speed;
    mecanum_debug.fb_speed_lb = mecanum_motor_lb.Pos_Info.Speed;
    mecanum_debug.run_mode_lf = (uint8_t)mecanum_motor_lf.drw.run_mode.data;
    mecanum_debug.run_mode_rf = (uint8_t)mecanum_motor_rf.drw.run_mode.data;
    mecanum_debug.run_mode_rb = (uint8_t)mecanum_motor_rb.drw.run_mode.data;
    mecanum_debug.run_mode_lb = (uint8_t)mecanum_motor_lb.drw.run_mode.data;
    mecanum_debug.pattern_lf = mecanum_motor_lf.Pos_Info.pattern;
    mecanum_debug.pattern_rf = mecanum_motor_rf.Pos_Info.pattern;
    mecanum_debug.pattern_rb = mecanum_motor_rb.Pos_Info.pattern;
    mecanum_debug.pattern_lb = mecanum_motor_lb.Pos_Info.pattern;
    mecanum_debug.update_tick_ms = tick_ms;
    mecanum_debug.update_count++;

    Mecanum_Wheel_TxScheduled(cmd_lf, cmd_rf, cmd_rb, cmd_lb, tick_ms);

    mecanum_debug.ready_mask = Mecanum_Wheel_GetReadyMask(tick_ms);
    mecanum_debug.last_feedback_tick_lf = mecanum_feedback_tick_ms[MECANUM_MOTOR_LF_INDEX];
    mecanum_debug.last_feedback_tick_rf = mecanum_feedback_tick_ms[MECANUM_MOTOR_RF_INDEX];
    mecanum_debug.last_feedback_tick_rb = mecanum_feedback_tick_ms[MECANUM_MOTOR_RB_INDEX];
    mecanum_debug.last_feedback_tick_lb = mecanum_feedback_tick_ms[MECANUM_MOTOR_LB_INDEX];
    mecanum_debug.enable_retry_last_tick_lf = mecanum_enable_retry_tick_ms[MECANUM_MOTOR_LF_INDEX];
    mecanum_debug.enable_retry_last_tick_rf = mecanum_enable_retry_tick_ms[MECANUM_MOTOR_RF_INDEX];
    mecanum_debug.enable_retry_last_tick_rb = mecanum_enable_retry_tick_ms[MECANUM_MOTOR_RB_INDEX];
    mecanum_debug.enable_retry_last_tick_lb = mecanum_enable_retry_tick_ms[MECANUM_MOTOR_LB_INDEX];
    mecanum_debug.enable_retry_count_lf = mecanum_enable_retry_count[MECANUM_MOTOR_LF_INDEX];
    mecanum_debug.enable_retry_count_rf = mecanum_enable_retry_count[MECANUM_MOTOR_RF_INDEX];
    mecanum_debug.enable_retry_count_rb = mecanum_enable_retry_count[MECANUM_MOTOR_RB_INDEX];
    mecanum_debug.enable_retry_count_lb = mecanum_enable_retry_count[MECANUM_MOTOR_LB_INDEX];
}

void Mecanum_Wheel_RxCallback(uint32_t ext_id, uint8_t *data)
{
    uint8_t target_id;
    uint8_t communication_type;

    if (data == 0)
    {
        return;
    }

    target_id = (uint8_t)((ext_id >> 8) & 0xFFU);
    communication_type = (uint8_t)((ext_id >> 24) & 0x3FU);
    mecanum_rx_ext_total_debug++;
    mecanum_rx_last_ext_id_debug = ext_id;
    mecanum_rx_last_target_id_debug = target_id;
    mecanum_rx_target_count_debug[target_id & 0x0FU]++;

    switch (target_id)
    {
    case MECANUM_MOTOR_LF_ID:
        RobStride_Motor_Analysis(&mecanum_motor_lf, data, ext_id);
        if (communication_type == Communication_Type_MotorRequest)
        {
            mecanum_feedback_tick_ms[MECANUM_MOTOR_LF_INDEX] = HAL_GetTick();
        }
        break;
    case MECANUM_MOTOR_RF_ID:
        RobStride_Motor_Analysis(&mecanum_motor_rf, data, ext_id);
        if (communication_type == Communication_Type_MotorRequest)
        {
            mecanum_feedback_tick_ms[MECANUM_MOTOR_RF_INDEX] = HAL_GetTick();
        }
        break;
    case MECANUM_MOTOR_RB_ID:
        RobStride_Motor_Analysis(&mecanum_motor_rb, data, ext_id);
        if (communication_type == Communication_Type_MotorRequest)
        {
            mecanum_feedback_tick_ms[MECANUM_MOTOR_RB_INDEX] = HAL_GetTick();
        }
        break;
    case MECANUM_MOTOR_LB_ID:
        RobStride_Motor_Analysis(&mecanum_motor_lb, data, ext_id);
        if (communication_type == Communication_Type_MotorRequest)
        {
            mecanum_feedback_tick_ms[MECANUM_MOTOR_LB_INDEX] = HAL_GetTick();
        }
        break;
    default:
        break;
    }
}
