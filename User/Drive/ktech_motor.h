#ifndef __KTECH_MOTOR_H__
#define __KTECH_MOTOR_H__

#include <stdint.h>
#include "can_receive_send.h"   // 提供 canx_send_data 函数

/* 命令字节定义 */
#define KTECH_CMD_READ_STATUS1      0x9A
#define KTECH_CMD_CLEAR_ERROR       0x9B
#define KTECH_CMD_READ_STATUS2      0x9C
#define KTECH_CMD_READ_STATUS3      0x9D
#define KTECH_CMD_MOTOR_OFF         0x80
#define KTECH_CMD_MOTOR_ON          0x88
#define KTECH_CMD_MOTOR_STOP        0x81
#define KTECH_CMD_BRAKE              0x8C
#define KTECH_CMD_OPENLOOP           0xA0
#define KTECH_CMD_TORQUE_CLOSED      0xA1
#define KTECH_CMD_SPEED_CLOSED       0xA2
#define KTECH_CMD_POS_MULTI1         0xA3
#define KTECH_CMD_POS_MULTI2         0xA4
#define KTECH_CMD_POS_SINGLE1        0xA5
#define KTECH_CMD_POS_SINGLE2        0xA6
#define KTECH_CMD_POS_INCREMENT1     0xA7
#define KTECH_CMD_POS_INCREMENT2     0xA8
#define KTECH_CMD_READ_PARAM         0xC0
#define KTECH_CMD_WRITE_PARAM        0xC1
#define KTECH_CMD_READ_ENCODER       0x90
#define KTECH_CMD_SET_ZERO           0x19
#define KTECH_CMD_READ_MULTI_ANGLE   0x92
#define KTECH_CMD_READ_SINGLE_ANGLE  0x94
#define KTECH_CMD_SET_ANGLE_RAM      0x95

/* 电机ID范围 */
#define KTECH_ID_MIN    1
#define KTECH_ID_MAX    32

/* 电机反馈数据结构 */
typedef struct {
    /* 状态1相关 */
    int8_t  temperature;      // 温度, °C
    int16_t voltage;          // 母线电压, 0.01V/LSB
    int16_t current;          // 母线电流, 0.01A/LSB
    uint8_t motorState;       // 电机状态: 0x00开启, 0x10关闭
    uint8_t errorState;       // 错误标志位

    /* 状态2相关 */
    int16_t iq_or_power;      // 转矩电流(MF/MG)或功率(MS)
    int16_t speed;            // 转速, 1dps/LSB
    uint16_t encoder;         // 编码器值 (14/15/16bit)

    /* 状态3相关 */
    int16_t iA, iB, iC;       // 三相电流 (仅MF/MG)

    /* 编码器读取相关 */
    uint16_t encoderRaw;      // 编码器原始值
    uint16_t encoderOffset;   // 编码器零偏

    /* 角度读取相关 */
    int64_t  multiAngle;      // 多圈角度, 0.01°/LSB
    uint32_t singleAngle;     // 单圈角度, 0.01°/LSB, 0~36000

    /* 参数读写相关 */
    uint8_t  paramID;         // 参数序号
    uint8_t  paramData[6];    // 参数值6字节

    /* 其他 */
    uint8_t  brakeState;      // 抱闸状态: 0断电刹车,1通电释放
} KTech_Feedback_t;

/* 电机控制数据结构（可选，用于存储用户设置） */
typedef struct {
    int16_t  power_openloop;   // 开环控制值 (MS)
    int16_t  iq_torque;        // 转矩电流 (MF/MG)
    int32_t  speed;            // 速度目标 (0.01dps/LSB)
    int32_t  position_multi;   // 多圈位置目标 (0.01°/LSB)
    uint32_t position_single;  // 单圈位置目标 (0~36000)
    int32_t  position_inc;     // 增量位置目标 (0.01°/LSB)
    uint16_t maxSpeed;         // 速度限制 (1dps/LSB)
    uint8_t  direction;        // 方向: 0顺时针,1逆时针 (单圈模式)
    uint8_t  paramID;          // 参数序号
    uint8_t  paramData[6];     // 参数值6字节
} KTech_Control_t;

/* 电机整体结构体 */
typedef struct {
    uint16_t          id;      // 电机ID 1-32
    KTech_Feedback_t  fb;      // 反馈数据
    KTech_Control_t   ctrl;    // 控制数据
} KTech_Motor_t;

/* 外部声明电机数组（索引对应ID） */
extern KTech_Motor_t ktech_motors[KTECH_ID_MAX + 1];

/* 初始化指定ID的电机 */
void ktech_motor_init(uint16_t id);

/* 发送命令函数（各命令的具体实现） */
void ktech_read_status1(hcan_t* hcan, uint16_t id);
void ktech_clear_error(hcan_t* hcan, uint16_t id);
void ktech_read_status2(hcan_t* hcan, uint16_t id);
void ktech_read_status3(hcan_t* hcan, uint16_t id);
void ktech_motor_off(hcan_t* hcan, uint16_t id);
void ktech_motor_on(hcan_t* hcan, uint16_t id);
void ktech_motor_stop(hcan_t* hcan, uint16_t id);
void ktech_brake_ctrl(hcan_t* hcan, uint16_t id, uint8_t cmd); // cmd:0刹车,1释放,0x10读取
void ktech_openloop_ctrl(hcan_t* hcan, uint16_t id, int16_t power);
void ktech_torque_ctrl(hcan_t* hcan, uint16_t id, int16_t iq);
void ktech_speed_ctrl(hcan_t* hcan, uint16_t id, int32_t speed, int16_t iq_limit);
void ktech_pos_multi1(hcan_t* hcan, uint16_t id, int32_t angle);
void ktech_pos_multi2(hcan_t* hcan, uint16_t id, int32_t angle, uint16_t max_speed);
void ktech_pos_single1(hcan_t* hcan, uint16_t id, uint8_t dir, uint32_t angle);
void ktech_pos_single2(hcan_t* hcan, uint16_t id, uint8_t dir, uint32_t angle, uint16_t max_speed);
void ktech_pos_inc1(hcan_t* hcan, uint16_t id, int32_t inc);
void ktech_pos_inc2(hcan_t* hcan, uint16_t id, int32_t inc, uint16_t max_speed);
void ktech_read_param(hcan_t* hcan, uint16_t id, uint8_t param_id);
void ktech_write_param(hcan_t* hcan, uint16_t id, uint8_t param_id, uint8_t* data); // data为6字节
void ktech_read_encoder(hcan_t* hcan, uint16_t id);
void ktech_set_zero(hcan_t* hcan, uint16_t id);
void ktech_read_multi_angle(hcan_t* hcan, uint16_t id);
void ktech_read_single_angle(hcan_t* hcan, uint16_t id);
void ktech_set_angle_ram(hcan_t* hcan, uint16_t id, int32_t angle);

/* CAN接收解析函数（在接收回调中调用） */
extern void ktech_parse_motor_fb(KTech_Motor_t* motor, uint8_t* data);

#endif /* __KTECH_MOTOR_H__ */