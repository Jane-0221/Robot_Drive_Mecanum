#ifndef __B07_HI_H__
#define __B07_HI_H__

#include <stdint.h>
#include <stdio.h>
#include "fdcan.h"          /* 定义 hcan_t 使用的 FDCAN 句柄类型 */

struct servo_state
{
    float angle;
    float speed;
};

struct servo_volcur
{
    float vol;
    float cur;
};
typedef struct
{
    float angle;    // 电机角度，单位：度
    float speed;    // 电机速度，单位：r/min
    float torque;   // 电机扭矩，单位：Nm
    uint8_t valid;  // 数据有效标志：1=有效，0=无效或未接收到反馈
} DrRobot_MotorState_t;
extern DrRobot_MotorState_t daran_motor_state[3];

typedef FDCAN_HandleTypeDef hcan_t;//
extern int8_t READ_FLAG;
extern uint8_t rx_buffer[8];
extern uint16_t can_id;
extern float motor_state[20][5];
void format_data( float *value_data, int *type_data,int length, char * str);
extern void DrRobot_ParseFbData(DrRobot_MotorState_t *motor_state, uint8_t *rx_data);
extern void reply_state(uint8_t id_num);
void preset_angle(hcan_t* hcan, uint8_t id_num, float angle, float t, float param, int mode);
void preset_speed(hcan_t* hcan, uint8_t id_num, float speed, float param, int mode);
void preset_torque(hcan_t* hcan, uint8_t id_num, float torque, float param, int mode);
void set_angle(hcan_t* hcan, uint8_t id_num, float angle, float speed, float param, int mode);
void set_angles(hcan_t* hcan, uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void step_angle(hcan_t* hcan, uint8_t id_num, float angle, float speed, float param, int mode);
void step_angles(hcan_t* hcan, uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void set_speed(hcan_t* hcan, uint8_t id_num, float speed, float param, int mode);
void set_speeds(hcan_t* hcan, uint8_t *id_list, float *speed_list, float param, float mode, size_t n);
void set_torque(hcan_t* hcan, uint8_t id_num, float torque, float param, int mode);
void set_torques(hcan_t* hcan, uint8_t *id_list, float *torque_list, float param, int mode, size_t n);
void impedance_control(hcan_t* hcan, uint8_t id_num, float pos, float vel, float tff, float kp, float kd);
void estop(hcan_t* hcan, uint8_t id_num);
void set_id(hcan_t* hcan, uint8_t id_num, int new_id);
void set_uart_baud_rate(hcan_t* hcan, uint8_t id_num, int baud_rate);
void set_can_baud_rate(hcan_t* hcan, uint8_t id_num, int baud_rate);
void set_mode(hcan_t* hcan, uint8_t id_num, int mode);
void set_zero_position(hcan_t* hcan, uint8_t id_num);
void set_GPIO_mode(hcan_t* hcan, uint8_t id_num, uint8_t mode, uint32_t param);
int8_t set_angle_range(hcan_t* hcan, uint8_t id_num, float angle_min, float angle_max);
void set_traj_mode(hcan_t* hcan, uint8_t id_num, int mode);
void write_property(hcan_t* hcan, uint8_t id_num, unsigned short param_address, int8_t param_type, float value);
uint8_t get_id(hcan_t* hcan, uint8_t id_num);
struct servo_state get_state(hcan_t* hcan, uint8_t id_num);
struct servo_volcur get_volcur(hcan_t* hcan, uint8_t id_num);
int8_t get_GPIO_mode(hcan_t* hcan, uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n);
float read_property(hcan_t* hcan, uint8_t id_num, int param_address, int param_type);
void clear_error(hcan_t* hcan, uint8_t id_num);
int8_t dump_error(hcan_t* hcan, uint8_t id_num);
void save_config(hcan_t* hcan, uint8_t id_num);
void reboot(hcan_t* hcan, uint8_t id_num);
void position_done(hcan_t* hcan, uint8_t id_num);
void positions_done(hcan_t* hcan, uint8_t *id_list, size_t n);

#endif /* __B07_HI_H__ */
