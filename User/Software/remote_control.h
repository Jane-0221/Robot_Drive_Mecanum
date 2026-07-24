#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"
#include "Sbus.h"
#include "uart_protocol.h"

#define CONTROL_MODE_REMOTE 0U
#define CONTROL_MODE_PC     1U

extern volatile uint8_t control_mode;

typedef struct {
    SBUS_CH_Struct SBUS_CH;
} ChassisRemoteSbusDebug_t;

typedef struct {
    uint32_t uwTick;
    uint8_t control_mode;
    uint8_t chassis_mode;
    float x;
    float y;
    float w;
    UpData_t up_tx_data;
    PcChassisCtrl_t pc_rx;
    uint8_t pc_rx_valid;
    uint32_t pc_rx_count;
    uint32_t pc_rx_last_tick;
    uint32_t pc_rx_age_ms;
    uint8_t pc_rx_timeout;
    uint8_t pc_mode_rx;
    uint8_t pc_mode_rx_valid;
    uint32_t pc_mode_rx_count;
    uint32_t pc_mode_rx_last_tick;
    uint32_t control_mode_switch_count;
    uint32_t control_mode_last_switch_tick;
} ChassisCommandDebug_t;

extern volatile ChassisRemoteSbusDebug_t chassis_remote_sbus;
extern volatile ChassisCommandDebug_t chassis_command;
extern volatile float chassis_lidar_yaw_offset_deg_debug;
extern volatile float chassis_remote_lidar_vx_debug;
extern volatile float chassis_remote_lidar_vy_debug;
extern volatile float chassis_remote_body_vx_debug;
extern volatile float chassis_remote_body_vy_debug;
extern volatile uint32_t control_mode_switch_count_debug;
extern volatile uint32_t control_mode_last_switch_tick_debug;
extern volatile uint8_t control_mode_last_request_debug;

typedef struct {
    PcMotorCtrl_t latest_rx_command;
    uint8_t latest_rx_valid;
    uint32_t latest_rx_count;
    PcMotorCtrl_t latest_handled_command;
    uint8_t latest_handled_valid;
    uint8_t arm_motor_command[6];
    uint8_t head_motor_command[2];
    uint8_t arm_last_index;
    uint8_t arm_last_command;
    uint32_t arm_command_count;
    uint8_t head_last_index;
    uint8_t head_last_command;
    uint32_t head_command_count;
} PcMotorCommandDebug_t;

/**
 * @brief 遥控控制模块初始化。
 *
 * 当前实现为空函数，保留该接口用于后续初始化遥控相关状态或外设。
 */
void remote_control_init(void);

/**
 * @brief 根据 SBUS 遥控通道更新气泵状态。
 *
 * CH8 高位打开气泵，CH8 低位关闭气泵。
 */
extern void Pump_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道更新头部电机和舵机目标角度。
 *
 * CH8 用于切换手动微调和预设姿态模式，CH6 选择当前控制档位或预设姿态。
 */
extern void Head_Motor_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道微调机械臂电机目标角度。
 *
 * CH8 低位时启用机械臂电机遥控；CH7 选择电机组，CH1/CH2 调整对应关节角度。
 */
extern void Arm_Motor_Control_Updata(void);

/**
 * @brief 检查遥控失能组合键并在首次触发时关闭机械臂电机。
 *
 * 当 CH5、CH6、CH7、CH8 同时处于高位时触发失能。该函数带锁存逻辑，
 * 同一次持续触发期间只发送一次失能指令。
 *
 * @return uint8_t 1 表示当前失能组合键有效，0 表示未触发。
 */
uint8_t Arm_Motor_Disable_Updata(void);

/**
 * @brief 获取机械臂失能组合键当前是否处于有效状态。
 *
 * @return uint8_t 1 表示失能组合键当前有效，0 表示无效。
 */
uint8_t Arm_Motor_Disable_IsActive(void);

/**
 * @brief 根据 CH8 控制头部电机使能/失能。
 *
 * CH8 高位时失能头部电机，CH8 低位时使能头部电机；中位保持上一次状态。
 *
 * @return uint8_t 1 表示头部电机处于失能请求状态，0 表示未失能。
 */
uint8_t Head_Motor_Enable_Disable_Updata(void);

/**
 * @brief PA15 USER_KEY 触发的头部电机使能/失能翻转请求。
 *
 * 与遥控器 CH8 共用同一套头部电机失能状态：任一来源请求失能时，Head_Task 都会保持失能。
 *
 * @return uint8_t 1 表示头部电机当前处于失能请求状态，0 表示未失能。
 */
uint8_t Head_Motor_ToggleByUserKey(void);

/**
 * @brief 获取头部电机失能请求当前是否有效。
 *
 * @return uint8_t 1 表示头部电机失能请求有效，0 表示无效。
 */
uint8_t Head_Motor_Disable_IsActive(void);



void arm_save_home_position(void);
void head_save_home_position(void);
uint8_t Arm_Save_Position_IsActive(void);
/**
 * @brief 根据 SBUS 遥控通道更新升降机构目标高度。
 *
 * CH7 高、中、低三档分别对应不同的升降目标高度。
 */
void Up_Down_Motor_Control_Updata(void);

/**
 * @brief 根据 SBUS 遥控通道更新底盘速度指令。
 *
 * 在指定拨杆组合下读取 CH4/CH3/CH1，转换为前后、横移和旋转速度。
 */
extern void Chassis_Control_Updata(void);

/* PC 控制接口声明 */

/**
 * @brief 根据 PC 下发数据更新气泵状态。
 */
extern void Control_Mode_Updata(void);
extern void PC_Pump_Control_Updata(void);

/**
 * @brief 根据 PC 下发的底盘速度帧更新 x/y/w。
 */
extern void PC_Chassis_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新头部电机目标角度。
 */
extern void PC_Head_Motor_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新升降机构目标高度。
 */
extern void PC_Up_Down_Motor_Control_Updata(void);
extern void PC_Rs485_Lift_Control_Updata(void);

/**
 * @brief 根据 PC 下发数据更新机械臂舵机和电机目标角度。
 */
extern void PC_Arm_Motor_Control_Updata(void);
extern void PC_Motor_Command_Updata(void);
extern volatile uint8_t pc_arm_motor_enable_state_debug[6];
extern volatile uint8_t pc_arm_motor_last_index_debug;
extern volatile uint8_t pc_arm_motor_last_enable_state_debug;
extern volatile uint32_t pc_arm_motor_command_count_debug;
extern volatile uint8_t pc_head_motor_enable_state_debug[2];
extern volatile uint8_t pc_head_motor_last_index_debug;
extern volatile uint8_t pc_head_motor_last_enable_state_debug;
extern volatile uint32_t pc_head_motor_command_count_debug;
extern volatile PcMotorCommandDebug_t pc_motor_command_debug;
extern volatile float head_pc_tx_source_deg_debug[2];
extern volatile float head_pc_tx_normalized_deg_debug[2];
extern volatile float head_pc_tx_rad_debug[2];
extern volatile uint32_t head_pc_tx_invalid_count_debug[2];

/**
 * @brief 将当前底盘速度状态写入 PC 上传数据结构。
 */
extern void pc_up_tx_data(void);
extern void pc_arm_tx_data(void);

#endif // !__REMOTE_CONTROL_H__
