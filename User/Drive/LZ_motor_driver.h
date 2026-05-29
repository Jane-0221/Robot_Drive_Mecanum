#ifndef __LZ_MOTOR_DRIVER_H
#define __LZ_MOTOR_DRIVER_H

#include "main.h"
#include "stdint.h"
#include "fdcan.h"


// 默认电机和主机ID
#define DEFAULT_MOTOR_ID   1
#define DEFAULT_MASTER_ID  0xFD

// 参数范围定义
#define LZ_P_MIN -12.57f
#define LZ_P_MAX 12.57f
#define LZ_V_MIN -33.0f
#define LZ_V_MAX 33.0f
#define LZ_KP_MIN 0.0f
#define LZ_KP_MAX 500.0f
#define LZ_KD_MIN 0.0f
#define LZ_KD_MAX 5.0f
#define LZ_T_MIN -14.0f
#define LZ_T_MAX 14.0f

// 电机控制模式
typedef enum {
    LZ_MODE_MIT = 0,    // MIT模式
    LZ_MODE_POSITION,   // 位置模式
    LZ_MODE_VELOCITY,   // 速度模式
    LZ_MODE_CURRENT,    // 电流模式
    LZ_MODE_DISABLE     // 失能模式
} LZ_Mode_t;

// 电机状态结构体
typedef struct {
    uint8_t motor_id;
    float angle;        // 角度 (rad)
    float velocity;     // 速度 (rad/s)
    float torque;       // 转矩 (N.m)
    float temperature;  // 温度 (°C)
    float angle_cnt;
    float angle_last;
    int sign;
    uint32_t fault;     // 故障码
    uint8_t uid[8];     // 唯一标识符
} LZ_Motor_State_t;

// 电机控制结构体
typedef struct {
    uint8_t id;
    uint8_t master_id;
    LZ_Mode_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
    float current_limit;
    LZ_Motor_State_t state;
} LZ_Motor_t;

// 函数声明
FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus);
void lz_send_command(uint8_t can_bus, uint16_t motor_id, uint8_t *data);
void lz_enable_motor(uint8_t can_bus, uint8_t motor_id);
void lz_disable_motor(uint8_t can_bus, uint8_t motor_id);
void lz_send_mit_params(uint8_t can_bus, uint8_t motor_id, float angle, float speed, float kp, float kd, float torque);
void lz_set_zero(uint8_t can_bus, uint8_t motor_id);
void lz_clear_fault(uint8_t can_bus, uint8_t motor_id);
void lz_set_mode(uint8_t can_bus, uint8_t motor_id, uint8_t mode);
void lz_set_id(uint8_t can_bus, uint8_t motor_id, uint8_t new_id);
void lz_set_protocol(uint8_t can_bus, uint8_t motor_id, uint8_t protocol);
void lz_set_master_id(uint8_t can_bus, uint8_t motor_id, uint8_t master_id);
void lz_set_position(uint8_t can_bus, uint8_t motor_id, float target_pos, float max_speed);
void lz_set_velocity(uint8_t can_bus, uint16_t motor_id, float target_vel, float current_limit);

#endif /* __MIT_MOTOR_H */
