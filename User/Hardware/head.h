#ifndef __HEAD_H__
#define __HEAD_H__

/**
 * @file head.h
 * @brief 头部两路KTech连杆电机的控制接口、状态数据和硬件资源定义。
 */

#include <stdint.h>
#include "can_receive_send.h"
#include "ktech_motor.h"

// ==================== 硬件资源统一定义 ====================
/**
 * @brief CAN1控制器句柄（头部连杆电机挂载在CAN1总线）
 * @note  等价于&hfdcan1，统一宏定义便于后续修改
 */
#define CAN_HANDLE_1 (&hfdcan1)

/** 头部连杆电机ID宏定义（CAN1总线） */
#define MOTOR_LINKONG_1_ID 0x01 // 头部连杆1号电机ID
#define MOTOR_LINKONG_2_ID 0x02 // 头部连杆2号电机ID

/** 电机旋转方向宏定义 */
#define DIR_CW  0x00    // 顺时针方向
#define DIR_CCW 0x01    // 逆时针方向

// ==================== 数据结构体定义 ====================
/**
 * @brief 头部电机（连杆电机）控制数据结构体
 * @note  存储电机的当前状态、目标参数和旋转方向，适配科泰（KTech）电机协议
 */
typedef struct
{
    float current_velocity; // 当前速度（KTech状态2反馈，单位：dps）
    float current_angle;    // 当前角度（单位：°，由编码器值换算得到）
    uint8_t direction;      // 旋转方向（0=顺时针(DIR_CW)，1=逆时针(DIR_CCW)）
    uint32_t target_angle;  // 目标角度（分辨率0.01°/LSB，如18000对应180°）
    uint16_t max_speed;     // 预留最大转速限制字段（单位：1dps/LSB，当前发送使用动态速度计算）
} Head_MotorData_t;

// ==================== 全局变量声明 ====================
/**
 * @brief 头部连杆电机控制数据数组（2路）
 * @note  head_motor_data[0]对应1号连杆电机，head_motor_data[1]对应2号连杆电机
 */
extern Head_MotorData_t head_motor_data[2];

/**
 * @brief 科泰电机对象数组（2路）
 * @note  存储科泰电机的协议解析数据（反馈角度、速度、电流等），对应2个头部连杆电机
 */
extern KTech_Motor_t motor_linkong[2];

// ==================== 函数声明 ====================
/**
 * @brief 头部电机初始化函数
 * @retval 无
 * @note   初始化头部2路连杆电机，配置CAN1通信、科泰电机协议参数
 */
extern void Head_Init(void);

/**
 * @brief 控制头部1号连杆电机
 * @retval 无
 * @note   根据head_motor_data[0]的目标参数，通过CAN1发送控制指令
 */
extern void Head_Lk_motor1(void);

/**
 * @brief 头部所有电机控制指令发送函数
 * @retval 无
 * @note   批量发送头部2路连杆电机的控制指令，建议在主循环周期性调用
 */
extern void Head_all_tx(void);

/**
 * @brief 请求头部两个电机反馈编码器数据
 * @retval 无
 * @note  读取KTech状态2，反馈中包含速度和编码器位置
 */
extern void Head_RequestFeedback(void);

/**
 * @brief 请求指定头部电机反馈数据
 * @param motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 无
 */
extern void Head_RequestFeedbackByIndex(uint8_t motor_index);

/**
 * @brief 判断两路头部电机反馈是否均已达到稳定计数
 * @retval 1=两路反馈均就绪，0=至少一路未就绪
 */
extern uint8_t Head_FeedbackReady(void);

/**
 * @brief 判断指定头部电机反馈是否已达到稳定计数
 * @param motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 1=指定电机反馈就绪，0=索引无效或反馈未就绪
 */
extern uint8_t Head_MotorFeedbackReady(uint8_t motor_index);

/**
 * @brief 获取指定头部电机已收到的反馈帧计数
 * @param motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 反馈计数；索引无效时返回0
 */
extern uint32_t Head_GetFeedbackCount(uint8_t motor_index);

/**
 * @brief 通知头部模块已收到指定电机的一帧有效反馈
 * @param motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 无
 * @note  通常由CAN接收解析流程在识别到有效KTech反馈后调用
 */
extern void Head_NotifyFeedback(uint8_t motor_index);

/**
 * @brief 按索引发送单个头部电机控制指令
 * @param motor_index 电机索引，0对应1号电机，1对应2号电机
 * @retval 无
 * @note  若头部电机处于禁用状态，则不会发送控制帧
 */
extern void Head_TxMotorByIndex(uint8_t motor_index);

/**
 * @brief 使能头部两个电机
 * @retval 无
 * @note  会清零反馈就绪计数，再发送电机运行命令
 */
extern void Head_Motor_Enable(void);

/**
 * @brief 向头部两个电机发送运行命令
 * @retval 无
 * @note  仅发送运行命令并打开控制发送标志，不清零反馈计数
 */
extern void Head_Motor_SendEnableCommand(void);

extern uint8_t Head_EnableMotorByIndex(uint8_t motor_index);
extern uint8_t Head_DisableMotorByIndex(uint8_t motor_index);
extern uint8_t Head_SaveMotorZeroByIndex(uint8_t motor_index);

/**
 * @brief 失能头部两个电机
 * @retval 无
 * @note  禁止继续下发头部控制帧，并向两路电机发送关闭命令
 */
extern void Head_Motor_Disable(void);

/**
 * @brief 保存当前位置为头部电机零点
 * @retval 无
 * @note  保存后会重新使能电机，并把目标位置保持在新的零点
 */
extern void Head_save_position(void);

/**
 * @brief 更新头部连杆电机当前状态数据
 * @retval 无
 * @note   从motor_linkong数组读取科泰电机反馈数据，更新head_motor_data的current_angle/current_velocity
 *         current_angle单位为°，current_velocity单位为dps
 */
extern void Head_Lk_Data_update(void);

#endif
