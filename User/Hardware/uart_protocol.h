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
#define PC_RS485_LIFT_CTRL_FRAME_TYPE 0x05
#define PC_BATTERY_UP_FRAME_TYPE 0x06
#define PC_CONTROL_MODE_FRAME_TYPE 0x07

#define UP_DATA_LEN   16
#define DN_DATA_LEN   83
#define PC_MOTOR_CTRL_DATA_LEN 3
#define PC_CHASSIS_CTRL_DATA_LEN 12
#define PC_RS485_LIFT_CTRL_DATA_LEN 34
#define PC_BATTERY_UP_DATA_LEN 237
#define PC_CONTROL_MODE_DATA_LEN 1
#define UP_FRAME_LEN  24
#define DN_FRAME_LEN  91
#define PC_MOTOR_CTRL_FRAME_LEN 11
#define PC_CHASSIS_CTRL_FRAME_LEN 20
#define PC_RS485_LIFT_CTRL_FRAME_LEN 42
#define PC_BATTERY_UP_FRAME_LEN 245
#define PC_CONTROL_MODE_FRAME_LEN 9

#define PC_BATTERY_UP_PAYLOAD_VERSION 1U

#define PC_CONTROL_MODE_RC 0U
#define PC_CONTROL_MODE_PC 1U

#define PC_MOTOR_CTRL_TARGET_ARM  0U
#define PC_MOTOR_CTRL_TARGET_HEAD 1U

#define PC_MOTOR_CTRL_COMMAND_DISABLE   0U
#define PC_MOTOR_CTRL_COMMAND_ENABLE    1U
#define PC_MOTOR_CTRL_COMMAND_SAVE_ZERO 2U
#define PC_MOTOR_CTRL_STATE_NONE        0xFFU

#define PC_RS485_LIFT_FLAG_SNAP_AFTER_STOP 0x01U

#define PC_RS485_LIFT_CMD_NONE                   0U
#define PC_RS485_LIFT_CMD_SETUP                  1U
#define PC_RS485_LIFT_CMD_FORWARD                2U
#define PC_RS485_LIFT_CMD_REVERSE                3U
#define PC_RS485_LIFT_CMD_STOP                   4U
#define PC_RS485_LIFT_CMD_ENABLE                 5U
#define PC_RS485_LIFT_CMD_DISABLE                6U
#define PC_RS485_LIFT_CMD_SPEED                  7U
#define PC_RS485_LIFT_CMD_FIND_LIMITS            8U
#define PC_RS485_LIFT_CMD_MOVE                   9U
#define PC_RS485_LIFT_CMD_GOTO_HEIGHT            10U
#define PC_RS485_LIFT_CMD_SET_ZERO               11U
#define PC_RS485_LIFT_CMD_SET_LOWER              12U
#define PC_RS485_LIFT_CMD_SET_TOP                13U
#define PC_RS485_LIFT_CMD_SET_UPPER              14U
#define PC_RS485_LIFT_CMD_APPLY_LIMITS           15U
#define PC_RS485_LIFT_CMD_CALIBRATE_HEIGHT       16U
#define PC_RS485_LIFT_CMD_SET_MANUAL_LIMITS      17U
#define PC_RS485_LIFT_CMD_CALIBRATE_DISTANCE     18U
#define PC_RS485_LIFT_CMD_RESTORE_LAST_POSITION  19U
#define PC_RS485_LIFT_CMD_DISABLE_DRIVE_LIMITS   20U
#define PC_RS485_LIFT_CMD_CLEAR_LIMITS           21U

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
    float wheel_speed_lf;
    float wheel_speed_rf;
    float wheel_speed_rb;
    float wheel_speed_lb;
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

typedef struct {
    uint8_t command;
    uint8_t flags;
    uint16_t rpm;
    uint16_t accel_rpm;
    float move_mm;
    float target_height_mm;
    float current_height_mm;
    float manual_lower_mm;
    float manual_upper_mm;
    float command_distance_mm;
    float actual_distance_mm;
} PcRs485LiftCtrl_t;

extern DnData_t pc_dn_data;
extern uint8_t uart_protocol_raw_data[256];
extern UpData_t up_tx_data;
extern volatile uint32_t pc_dn_rx_count;
extern volatile PcMotorCtrl_t pc_motor_ctrl_latest;
extern volatile uint8_t pc_motor_ctrl_latest_valid;
extern volatile uint32_t pc_motor_ctrl_rx_count;
extern volatile PcChassisCtrl_t pc_chassis_ctrl_latest;
extern volatile uint8_t pc_chassis_ctrl_latest_valid;
extern volatile uint32_t pc_chassis_ctrl_rx_count;
extern volatile uint32_t pc_chassis_ctrl_last_tick;
extern volatile PcRs485LiftCtrl_t pc_rs485_lift_ctrl_latest;
extern volatile uint8_t pc_rs485_lift_ctrl_latest_valid;
extern volatile uint32_t pc_rs485_lift_ctrl_rx_count;
extern volatile uint8_t pc_control_mode_latest;
extern volatile uint8_t pc_control_mode_latest_valid;
extern volatile uint32_t pc_control_mode_rx_count;
extern volatile uint32_t pc_control_mode_last_tick;

uint16_t crc16_ccitt(uint8_t *data, uint16_t len);
void pack_up_frame(UpData_t *data, uint8_t *frame_buf);
void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data);
uint8_t UART_Protocol_UnpackLatest(DnData_t *data);
uint8_t UART_Protocol_CopyLatestDnData(DnData_t *out);
uint8_t UART_Protocol_GetControlModeCommand(uint8_t *mode);
uint8_t UART_Protocol_GetMotorCtrlCommand(PcMotorCtrl_t *command);
uint8_t UART_Protocol_CopyLatestChassisCtrl(PcChassisCtrl_t *out, uint32_t *last_tick);
uint8_t UART_Protocol_GetRs485LiftCommand(PcRs485LiftCtrl_t *command);
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len);
HAL_StatusTypeDef send_up_frame(UART_HandleTypeDef *huart);
HAL_StatusTypeDef send_up_frame_usb(void);
HAL_StatusTypeDef send_battery_up_frame(UART_HandleTypeDef *huart);
void store_uart_protocol_data(const uint8_t *data, uint16_t size);

#endif // __UART_PROTOCOL_H
