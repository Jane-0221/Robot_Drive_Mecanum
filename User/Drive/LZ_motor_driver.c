#include "LZ_motor_driver.h"
#include "CAN_receive_send.h"
#include "dm4310_drv.h"
#include "string.h"
#include "User_math.h"
FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus) {
    switch (can_bus) {
        case 0: return &hfdcan1;
        case 1: return &hfdcan2;
        case 2: return &hfdcan3;
        default: return &hfdcan1;
    }
}
/**
 * @brief LZ 电机驱动层，使用标准 CAN ID，电机 ID 占用 3 位，实际 ID 为 11 位 ID
 *        所有控制指令通过 CAN 总线发送，支持 MIT 协议和自定义协议
 */

/**
 * @brief 通用 CAN 命令发送函数
 * @param can_bus CAN 总线编号（0/1）
 * @param motor_id 电机 ID（0-7，实际 CAN ID 需左移或映射）
 * @param data 8 字节数据缓冲区
 */
void lz_send_command(uint8_t can_bus, uint16_t motor_id, uint8_t *data) {
    FDCAN_HandleTypeDef *hfdcan = Get_CanHandle(can_bus);

     canx_send_data(hfdcan, motor_id, data, 8);
}

/**
 * @brief 使能电机（命令 1）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 */
void lz_enable_motor(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    lz_send_command(can_bus, motor_id, data);

}

/**
 * @brief 失能电机（命令 2）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 */
void lz_disable_motor(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 发送 MIT 协议控制参数（命令 3）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 * @param angle 目标角度（弧度，范围 LZ_P_MIN~LZ_P_MAX）
 * @param speed 目标速度（rad/s，范围 LZ_V_MIN~LZ_V_MAX）
 * @param kp 位置刚度（范围 LZ_KP_MIN~LZ_KP_MAX）
 * @param kd 速度阻尼（范围 LZ_KD_MIN~LZ_KD_MAX）
 * @param torque 前馈转矩（范围 LZ_T_MIN~LZ_T_MAX）
 */
void lz_send_mit_params(uint8_t can_bus, uint8_t motor_id, float angle, float speed, float kp, float kd, float torque) {
    uint16_t angle_uint = float_to_uint(angle, LZ_P_MIN, LZ_P_MAX, 16);
    uint16_t speed_uint = float_to_uint(speed, LZ_V_MIN, LZ_V_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, LZ_KP_MIN, LZ_KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, LZ_KD_MIN, LZ_KD_MAX, 12);
    uint16_t torque_uint = float_to_uint(torque, LZ_T_MIN, LZ_T_MAX, 12);

    uint8_t data[8] = {
        (uint8_t)(angle_uint >> 8), 
        (uint8_t)(angle_uint & 0xFF),
        (uint8_t)((speed_uint >> 4) & 0xFF),
        (uint8_t)(((speed_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)),
        (uint8_t)(kp_uint & 0xFF),
        (uint8_t)((kd_uint >> 4) & 0xFF),
        (uint8_t)(((kd_uint & 0x0F) << 4) | ((torque_uint >> 8) & 0x0F)),
        (uint8_t)(torque_uint & 0xFF)
    };
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置电机零点（命令 4）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 */
void lz_set_zero(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 清除电机故障（命令 5）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 */
void lz_clear_fault(uint8_t can_bus, uint8_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置电机运行模式（命令 6）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 * @param mode 模式（0：MIT，1：位置，2：速度，3：电流等）
 */
void lz_set_mode(uint8_t can_bus, uint8_t motor_id, uint8_t mode) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, mode, 0xFC};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 修改电机 ID（命令 7）
 * @param can_bus CAN 总线编号
 * @param motor_id 当前电机 ID
 * @param new_id 新电机 ID（0-7）
 */
void lz_set_id(uint8_t can_bus, uint8_t motor_id, uint8_t new_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, new_id, 0xFA};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置通信协议（命令 8）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 * @param protocol 协议类型（0：MIT，1：自定义）
 */
void lz_set_protocol(uint8_t can_bus, uint8_t motor_id, uint8_t protocol) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, protocol, 0xFD};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 设置主机 ID（命令 9）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 * @param master_id 主机 ID（用于接收反馈）
 */
void lz_set_master_id(uint8_t can_bus, uint8_t motor_id, uint8_t master_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, master_id, 0x01};
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 位置控制模式（命令 10）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID
 * @param target_pos 目标位置（单位取决于配置）
 * @param pos_speed 运行速度（单位取决于配置）
 */
void lz_set_position(uint8_t can_bus, uint8_t motor_id, float target_pos, float pos_speed) {
    uint8_t data[8];
    memcpy(&data[0], &target_pos, 4);
    memcpy(&data[4], &pos_speed, 4);
    lz_send_command(can_bus, motor_id, data);
}

/**
 * @brief 速度控制模式（命令 11）
 * @param can_bus CAN 总线编号
 * @param motor_id 电机 ID（注意这里使用 uint16_t，与其他函数不同）
 * @param target_vel 目标速度
 * @param current_limit 电流限制
 */
void lz_set_velocity(uint8_t can_bus, uint16_t motor_id, float target_vel, float current_limit) {
    uint8_t data[8];
    memcpy(&data[0], &target_vel, 4);
    memcpy(&data[4], &current_limit, 4);
    lz_send_command(can_bus, motor_id, data);
}
