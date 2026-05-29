#ifndef __ARM_H__
#define __ARM_H__

#include <stdint.h>
#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "crc_ccitt.h"

// ==================== 硬件资源统一定义 ====================
/**
 * @brief CAN2控制器句柄（机械臂所有电机均挂载在CAN2总线）
 * @note  等价于&hfdcan2，统一宏定义便于后续修改
 */
#define CAN_HANDLE_2 (&hfdcan2)

// ==================== 电机ID宏定义 ====================
/** 灵足电机ID（Robstride04协议，CAN2总线） */
#define MOTOR_LINGZU_1_ID 0x01 // 灵足1号电机ID
#define MOTOR_LINGZU_2_ID 0x02 // 灵足2号电机ID
#define MOTOR_LINGZU_3_ID 0x03 // 灵足3号电机ID

/** 达妙电机ID（DM4310协议，CAN2总线） */
#define MOTOR_DAMIAO_4_ID 0x04 // 达妙4号电机ID
#define MOTOR_DAMIAO_5_ID 0x05 // 达妙5号电机ID
#define MOTOR_DAMIAO_6_ID 0x06 // 达妙6号电机ID

#define ARM_LOGICAL_MOTOR_COUNT 6U

/** 余数电机ID（预留，未在代码中实际使用） */
#define MOTOR_YUSHU_1_ID 2 // 余数1号电机ID
#define MOTOR_YUSHU_2_ID 3 // 余数2号电机ID
// ==================== 枚举类型定义 ====================
/**
 * @brief 达妙电机状态枚举
 * @note  标识电机使能/禁用状态
 */
typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0, // 电机禁用状态
    Motor_DM_Status_ENABLE,      // 电机使能状态
} Motor_DM_Status;

// ==================== 数据结构体定义 ====================
/**
 * @brief 机械臂电机数据结构体
 * @note  存储电机的当前/目标角度、速度，单位均为弧度(rad)、弧度/秒(rad/s)
 */
typedef struct
{
    float current_angle;    // 当前角度（单位：rad）
    float current_velocity; // 当前速度（单位：rad/s）
    float target_angle;     // 目标角度（单位：rad）
    float target_velocity;  // 目标速度（单位：rad/s）
} ArmMotorData_t;

// ==================== 全局变量声明 ====================
/** 灵足电机对象（3路，Robstride04协议） */
extern RobStride_Motor_t motor1; // 灵足1号电机对象
extern RobStride_Motor_t motor2; // 灵足2号电机对象
extern RobStride_Motor_t motor3; // 灵足3号电机对象

/** 达妙电机状态数组（6路） */
extern Motor_DM_Status DM_Status[6]; // 存储每个达妙电机的使能/禁用状态

extern volatile uint8_t arm_motor_disabled_mask_debug;
extern volatile uint32_t arm_feedback_count_debug[ARM_LOGICAL_MOTOR_COUNT];
extern volatile uint32_t arm_feedback_last_tick_debug[ARM_LOGICAL_MOTOR_COUNT];
extern volatile uint32_t arm_linzu_tx_attempt_debug[3];
extern volatile uint32_t arm_linzu_tx_reject_debug[3];
extern volatile uint32_t arm_linzu_tx_sent_debug[3];
extern volatile uint32_t arm_linzu_tx_last_tick_debug[3];
extern volatile float arm_linzu_tx_requested_velocity_debug[3];
extern volatile float arm_linzu_tx_last_velocity_debug[3];
extern volatile uint32_t arm_damiao_tx_attempt_debug[3];
extern volatile uint32_t arm_damiao_tx_reject_debug[3];
extern volatile uint32_t arm_damiao_tx_sent_debug[3];
extern volatile uint32_t arm_damiao_tx_last_tick_debug[3];
extern volatile float arm_damiao_tx_last_velocity_debug[3];

/** 电机控制数据结构体数组（按品牌分类） */
extern ArmMotorData_t Linzu_motor_data[3];  // 灵足电机控制数据（3路）
extern ArmMotorData_t Damiao_motor_data[3]; // 达妙电机控制数据（3路）

// ==================== 函数声明 ====================
/**
 * @brief 机械臂初始化函数
 * @retval 无
 * @note   初始化灵足/达妙电机，配置CAN2通信
 */
void Arm_Init(void);

/** 灵足电机控制函数（按编号区分） */
void Arm_Linzu_motor1(void); // 控制1号灵足电机
void Arm_Linzu_motor2(void); // 控制2号灵足电机
void Arm_Linzu_motor3(void); // 控制3号灵足电机

/** 达妙电机控制函数（按编号区分） */
void Arm_Damiao_motor4(void); // 控制4号达妙电机
void Arm_Damiao_motor5(void); // 控制5号达妙电机
void Arm_Damiao_motor6(void); // 控制6号达妙电机

/** 电机状态数据更新函数 */
void Arm_Linzu_Data_update(void);  // 更新灵足电机当前角度/速度
void Arm_Damiao_Data_update(void); // 更新达妙电机当前角度/速度

/** 全局函数声明（跨文件调用） */
extern void Arm_All_Data_update(void); // 批量更新所有电机状态数据
extern void Arm_all_tx(void);          // 发送所有电机控制指令
extern void Arm_save_position(void);   // 保存当前位置
// extern void Arm_Motor_Disable_All(void);
// extern void Arm_Motor_Enable_All(void);
extern void Arm_CheckAndReenableDisabledMotors(void);
void Arm_RequestDisabledFeedback(void);
float Arm_WrapAngleToPi(float angle);
void Arm_SetPcTargetAngles(const float target_angles[ARM_LOGICAL_MOTOR_COUNT], const float target_velocities[ARM_LOGICAL_MOTOR_COUNT], uint32_t now_ms);
uint8_t Arm_AdjustMotorTargetByIndex(uint8_t logical_motor, float delta_angle);
uint8_t Arm_EnableMotorByIndex(uint8_t logical_motor);
uint8_t Arm_EnableAllMotors(void);
uint8_t Arm_DisableMotorByIndex(uint8_t logical_motor);
uint8_t Arm_SaveMotorZeroByIndex(uint8_t logical_motor);

#endif
