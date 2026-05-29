#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "stm32h7xx_hal.h"

#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL1   0xEE
#define FRAME_TAIL2   0xFF
#define UP_FRAME_TYPE 0x01
#define DN_FRAME_TYPE 0x02
#define PC_MOTOR_CTRL_FRAME_TYPE 0x03
#define PC_CHASSIS_CTRL_FRAME_TYPE 0x04

#define UP_DATA_LEN   56
#define DN_DATA_LEN   83
#define PC_MOTOR_CTRL_DATA_LEN 3
#define PC_CHASSIS_CTRL_DATA_LEN 12
#define UP_FRAME_LEN  64
#define DN_FRAME_LEN  91
#define PC_MOTOR_CTRL_FRAME_LEN 11
#define PC_CHASSIS_CTRL_FRAME_LEN 20

#define PC_MOTOR_CTRL_TARGET_ARM  0U
#define PC_MOTOR_CTRL_TARGET_HEAD 1U

#define PC_MOTOR_CTRL_COMMAND_DISABLE   0U
#define PC_MOTOR_CTRL_COMMAND_ENABLE    1U
#define PC_MOTOR_CTRL_COMMAND_SAVE_ZERO 2U
#define PC_MOTOR_CTRL_STATE_NONE        0xFFU

typedef struct {
    float air_path_state;
    float suck_state;
    float head_motor_angle_1;
    float head_motor_angle_2;
    float arm_motor_angle_1;
    float arm_motor_angle_2;
    float arm_motor_angle_3;
    float arm_motor_angle_4;
    float arm_motor_angle_5;
    float arm_motor_angle_6;
    float lift_height;
    float chassis_vx;
    float chassis_vy;
    float chassis_yaw;
} UpData_t;

typedef struct {
    float    pc_target_servo_angles[6];
    float    pc_target_motor_angles[6];
    float    pc_target_motor_velocities[6];
    uint8_t  pc_pump_state;
    uint16_t pc_target_lift_height;
    float    pc_target_head_motor_angles[2];
} DnData_t;

typedef struct {
    uint8_t target_type;
    uint8_t motor_index;
    uint8_t command;
} PcMotorCtrl_t;

typedef struct {
    float x;
    float y;
    float w;
} PcChassisCtrl_t;

extern DnData_t pc_dn_data;
extern uint8_t uart_protocol_raw_data[256];
extern UpData_t up_tx_data;
extern volatile PcMotorCtrl_t pc_motor_ctrl_latest;
extern volatile uint8_t pc_motor_ctrl_latest_valid;
extern volatile uint32_t pc_motor_ctrl_rx_count;
extern volatile PcChassisCtrl_t pc_chassis_ctrl_latest;
extern volatile uint8_t pc_chassis_ctrl_latest_valid;
extern volatile uint32_t pc_chassis_ctrl_rx_count;
extern volatile uint32_t pc_chassis_ctrl_last_tick;

uint16_t crc16_ccitt(uint8_t *data, uint16_t len);
void pack_up_frame(UpData_t *data, uint8_t *frame_buf);
void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data);
uint8_t UART_Protocol_UnpackLatest(DnData_t *data);
uint8_t UART_Protocol_CopyLatestDnData(DnData_t *out);
uint8_t UART_Protocol_GetMotorCtrlCommand(PcMotorCtrl_t *command);
uint8_t UART_Protocol_CopyLatestChassisCtrl(PcChassisCtrl_t *out, uint32_t *last_tick);
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len);
HAL_StatusTypeDef send_up_frame(UART_HandleTypeDef *huart);
void store_uart_protocol_data(const uint8_t *data, uint16_t size);

#endif // __UART_PROTOCOL_H
