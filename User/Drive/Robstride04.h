#ifndef __RobStride04_H__
#define __RobStride04_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"
#include "can_receive_send.h"



// 各种控制模式
#define move_control_mode  0  // 运控模式
#define Pos_control_mode   1  // PP位置模式
#define Speed_control_mode 2  // 速度模式
#define Elect_control_mode 3  // 电流模式
#define Set_Zero_mode      4  // 零点模式
#define CSP_control_mode   5  // CSP位置模式

// 通信类型
#define Communication_Type_Get_ID 0x00                // 获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01         // 运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02          // 用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03           // 电机使能运行
#define Communication_Type_MotorStop 0x04             // 电机停止运行
#define Communication_Type_SetPosZero 0x06            // 设置电机机械零位
#define Communication_Type_Can_ID 0x07                // 更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12          // 设置电机模式
#define Communication_Type_GetSingleParameter 0x11    // 读取单个参数
#define Communication_Type_SetSingleParameter 0x12    // 设定单个参数
#define Communication_Type_ErrorFeedback 0x15         // 故障反馈帧
#define Communication_Type_MotorDataSave 0x16         // 电机数据保存帧
#define Communication_Type_BaudRateChange 0x17        // 电机波特率修改帧，重新上电生效
#define Communication_Type_ProactiveEscalationSet 0x18 // 电机主动上报
#define Communication_Type_MotorModeSet 0x19          // 电机协议修改帧

// MIT协议命令
#define MIT_CMD_ENABLE    0xFC    // 电机使能
#define MIT_CMD_DISABLE   0xFD    // 电机失能
#define MIT_CMD_CLEAR_ERR 0xFB    // 清除错误
#define MIT_CMD_SET_ZERO  0xFE    // 设置零点
#define MIT_CMD_SET_MODE  0xFC    // 设置运行模式
#define MIT_CMD_SET_ID    0x01    // 设置电机ID
#define MIT_CMD_SET_PROTOCOL 0xFD // 设置协议类型

// MIT模式类型
enum MIT_TYPE {
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
};

// 参数读写结构体
typedef struct {
    uint16_t index;
    float data;
} data_read_write_one_t;

// 可读写参数结构体
typedef struct {
    data_read_write_one_t run_mode;        // 0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式
    data_read_write_one_t iq_ref;          // 电流模式Iq指令，范围：-90~90A
    data_read_write_one_t spd_ref;         // 转速模式转速指令，范围：-20~20rad/s
    data_read_write_one_t imit_torque;     // 转矩限制，范围：0~120Nm
    data_read_write_one_t cur_kp;          // 电流的Kp，默认值：0.05
    data_read_write_one_t cur_ki;          // 电流的Ki，默认值：0.05
    data_read_write_one_t cur_filt_gain;   // 电流滤波系数，范围：0~1.0，默认值：0.06
    data_read_write_one_t loc_ref;         // 位置模式角度指令，单位：rad
    data_read_write_one_t limit_spd;       // 位置模式速度设置，范围：0~20rad/s
    data_read_write_one_t limit_cur;       // 速度位置模式电流设置，范围：0~90A
    data_read_write_one_t mechPos;         // 负载端计圈机械角度，单位：rad（只读）
    data_read_write_one_t iqf;             // iq滤波值，范围：-90~90A（只读）
    data_read_write_one_t mechVel;         // 负载端转速，范围：-20~20rad/s（只读）
    data_read_write_one_t VBUS;            // 母线电压，单位：V（只读）
    data_read_write_one_t rotation;        // 圈数（只读）
} data_read_write_t;

// 电机位置信息结构体
typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    uint8_t pattern; // 电机模式（0复位1标定2运行）
} Motor_Pos_RobStride_Info_t;

// 电机设定值结构体
typedef struct {
    uint8_t set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_limit_speed;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set_t;

// RobStride电机主结构体
typedef struct {
    uint8_t CAN_ID;                     // CAN ID
    uint64_t Unique_ID;                 // 64位MCU唯一标识符
    uint16_t Master_CAN_ID;             // 主机ID（默认0xFD）
    
    Motor_Set_t Motor_Set_All;     // 设定值
    uint8_t error_code;                 // 错误码
    
    bool MIT_Mode;                      // MIT模式标志
    enum MIT_TYPE MIT_Type;             // MIT模式类型
    
    float output;                       // 输出值
    int32_t Can_Motor;                  // CAN电机标识
    Motor_Pos_RobStride_Info_t Pos_Info;  // 位置信息
    data_read_write_t drw;                // 电机数据
} RobStride_Motor_t;

// 参数索引列表
extern const uint16_t Index_List[];

// 邮箱变量
extern uint32_t Mailbox;

// 参数转换函数
float uint16_to_float_lz(uint16_t x, float x_min, float x_max, uint8_t bits);
uint16_t float_to_uint_lz(float x, float x_min, float x_max, uint8_t bits);
float Byte_to_float(uint8_t* bytedata);
uint8_t mapFaults(uint16_t fault16);

// 电机初始化函数
void RobStride_Motor_Init(RobStride_Motor_t* motor, uint8_t CAN_Id, bool MIT_mode);
void RobStride_Motor_Init_Offset(RobStride_Motor_t* motor, uint8_t CAN_Id, bool MIT_mode);

// 电机控制函数
void RobStride_Get_CAN_ID(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void RobStride_Motor_Analysis(RobStride_Motor_t* motor, uint8_t* DataFrame, uint32_t ID_ExtId);
void RobStride_Motor_move_control(RobStride_Motor_t* motor, hcan_t* hcan, float Torque, float Angle, float Speed, float Kp, float Kd); // 修改：增加hcan参数
void RobStride_Motor_Pos_control(RobStride_Motor_t* motor, hcan_t* hcan, float Speed, float Angle); // 修改：增加hcan参数
void RobStride_Motor_CSP_control(RobStride_Motor_t* motor, hcan_t* hcan, float Angle, float limit_spd); // 修改：增加hcan参数
void RobStride_Motor_Speed_control(RobStride_Motor_t* motor, hcan_t* hcan, float Speed, float limit_cur); // 修改：增加hcan参数
void RobStride_Motor_current_control(RobStride_Motor_t* motor, hcan_t* hcan, float current); // 修改：增加hcan参数
void RobStride_Motor_Set_Zero_control(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void Enable_Motor(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void Disenable_Motor(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t clear_error); // 修改：增加hcan参数
void Set_CAN_ID(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t Set_CAN_ID); // 修改：增加hcan参数
void Set_ZeroPos(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void Set_RobStride_Motor_parameter(RobStride_Motor_t* motor, hcan_t* hcan, uint16_t Index, float Value, char Value_mode); // 修改：增加hcan参数
void Get_RobStride_Motor_parameter(RobStride_Motor_t* motor, hcan_t* hcan, uint16_t Index); // 修改：增加hcan参数

// MIT协议相关函数
void RobStride_Motor_MIT_Enable(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void RobStride_Motor_MIT_Disable(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void RobStride_Motor_MIT_Control(RobStride_Motor_t* motor, hcan_t* hcan, float Angle, float Speed, float Kp, float Kd, float Torque); // 修改：增加hcan参数
void RobStride_Motor_MIT_PositionControl(RobStride_Motor_t* motor, hcan_t* hcan, float position_rad, float speed_rad_per_s); // 修改：增加hcan参数
void RobStride_Motor_MIT_SpeedControl(RobStride_Motor_t* motor, hcan_t* hcan, float speed_rad_per_s, float current_limit); // 修改：增加hcan参数
void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数
void RobStride_Motor_MIT_SetMotorType(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数
void RobStride_Motor_MIT_SetMotorId(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数
void RobStride_Motor_MIT_SetProtocol(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t protocol_type); // 修改：增加hcan参数

// 通用功能函数
void RobStride_Motor_MotorDataSave(RobStride_Motor_t* motor, hcan_t* hcan); // 修改：增加hcan参数
void RobStride_Motor_BaudRateChange(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数
void RobStride_Motor_MotorModeSet(RobStride_Motor_t* motor, hcan_t* hcan, uint8_t F_CMD); // 修改：增加hcan参数

// 获取函数
bool Get_MIT_Mode(RobStride_Motor_t* motor);
enum MIT_TYPE get_MIT_Type(RobStride_Motor_t* motor);

// 数据读写初始化
void data_read_write_init(data_read_write_t* drw);

#ifdef __cplusplus
}
#endif

#endif