#include "ktech_motor.h"
#include <string.h>

/* 电机数组定义 */
KTech_Motor_t ktech_motors[KTECH_ID_MAX + 1];

void ktech_motor_init(uint16_t id)
{
    if (id >= KTECH_ID_MIN && id <= KTECH_ID_MAX) {
        ktech_motors[id].id = id;
        memset(&ktech_motors[id].fb, 0, sizeof(KTech_Feedback_t));
        memset(&ktech_motors[id].ctrl, 0, sizeof(KTech_Control_t));
    }
}

/* 底层发送包装：命令ID = 0x140 + 电机ID */
static inline void send_cmd(hcan_t* hcan, uint16_t id, uint8_t* data)
{
    uint16_t tx_id = 0x140 + id;
    canx_send_data(hcan, tx_id, data, 8);
}

/* 读取状态1和错误标志 */
void ktech_read_status1(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_STATUS1, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 清除错误标志 */
void ktech_clear_error(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_CLEAR_ERROR, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 读取状态2 */
void ktech_read_status2(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_STATUS2, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 读取状态3 */
void ktech_read_status3(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_STATUS3, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 电机关闭 */
void ktech_motor_off(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_MOTOR_OFF, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 电机运行（开启） */
void ktech_motor_on(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_MOTOR_ON, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 电机停止 */
void ktech_motor_stop(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_MOTOR_STOP, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 抱闸器控制和状态读取 */
void ktech_brake_ctrl(hcan_t* hcan, uint16_t id, uint8_t cmd)
{
    uint8_t data[8] = {KTECH_CMD_BRAKE, cmd, 0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 开环控制 (MS系列) */
void ktech_openloop_ctrl(hcan_t* hcan, uint16_t id, int16_t power)
{
    uint8_t data[8] = {KTECH_CMD_OPENLOOP, 0,0,0,
                        (uint8_t)power, (uint8_t)(power>>8), 0,0};
    send_cmd(hcan, id, data);
}

/* 转矩闭环控制 (MF/MH/MG) */
void ktech_torque_ctrl(hcan_t* hcan, uint16_t id, int16_t iq)
{
    uint8_t data[8] = {KTECH_CMD_TORQUE_CLOSED, 0,0,0,
                        (uint8_t)iq, (uint8_t)(iq>>8), 0,0};
    send_cmd(hcan, id, data);
}

/* 速度闭环控制 (带力矩限制) */
void ktech_speed_ctrl(hcan_t* hcan, uint16_t id, int32_t speed, int16_t iq_limit)
{
    uint8_t data[8] = {KTECH_CMD_SPEED_CLOSED, 0,
                        (uint8_t)iq_limit, (uint8_t)(iq_limit>>8),
                        (uint8_t)speed, (uint8_t)(speed>>8),
                        (uint8_t)(speed>>16), (uint8_t)(speed>>24)};
    send_cmd(hcan, id, data);
}

/* 多圈位置闭环控制1 */
void ktech_pos_multi1(hcan_t* hcan, uint16_t id, int32_t angle)
{
    uint8_t data[8] = {KTECH_CMD_POS_MULTI1, 0,0,0,
                        (uint8_t)angle, (uint8_t)(angle>>8),
                        (uint8_t)(angle>>16), (uint8_t)(angle>>24)};
    send_cmd(hcan, id, data);
}

/* 多圈位置闭环控制2 (带速度限制) */
void ktech_pos_multi2(hcan_t* hcan, uint16_t id, int32_t angle, uint16_t max_speed)
{
    uint8_t data[8] = {KTECH_CMD_POS_MULTI2, 0,
                        (uint8_t)max_speed, (uint8_t)(max_speed>>8),
                        (uint8_t)angle, (uint8_t)(angle>>8),
                        (uint8_t)(angle>>16), (uint8_t)(angle>>24)};
    send_cmd(hcan, id, data);
}

/* 单圈位置闭环控制1 (带方向) */
void ktech_pos_single1(hcan_t* hcan, uint16_t id, uint8_t dir, uint32_t angle)
{
    uint8_t data[8] = {KTECH_CMD_POS_SINGLE1, dir, 0,0,
                        (uint8_t)angle, (uint8_t)(angle>>8),
                        (uint8_t)(angle>>16), (uint8_t)(angle>>24)};
    send_cmd(hcan, id, data);
}

/* 单圈位置闭环控制2 (带方向和速度限制) */
void ktech_pos_single2(hcan_t* hcan, uint16_t id, uint8_t dir, uint32_t angle, uint16_t max_speed)
{
    uint8_t data[8] = {KTECH_CMD_POS_SINGLE2, dir,
                        (uint8_t)max_speed, (uint8_t)(max_speed>>8),
                        (uint8_t)angle, (uint8_t)(angle>>8),
                        (uint8_t)(angle>>16), (uint8_t)(angle>>24)};
    send_cmd(hcan, id, data);
}

/* 增量位置闭环控制1 */
void ktech_pos_inc1(hcan_t* hcan, uint16_t id, int32_t inc)
{
    uint8_t data[8] = {KTECH_CMD_POS_INCREMENT1, 0,0,0,
                        (uint8_t)inc, (uint8_t)(inc>>8),
                        (uint8_t)(inc>>16), (uint8_t)(inc>>24)};
    send_cmd(hcan, id, data);
}

/* 增量位置闭环控制2 (带速度限制) */
void ktech_pos_inc2(hcan_t* hcan, uint16_t id, int32_t inc, uint16_t max_speed)
{
    uint8_t data[8] = {KTECH_CMD_POS_INCREMENT2, 0,
                        (uint8_t)max_speed, (uint8_t)(max_speed>>8),
                        (uint8_t)inc, (uint8_t)(inc>>8),
                        (uint8_t)(inc>>16), (uint8_t)(inc>>24)};
    send_cmd(hcan, id, data);
}

/* 读取控制参数 */
void ktech_read_param(hcan_t* hcan, uint16_t id, uint8_t param_id)
{
    uint8_t data[8] = {KTECH_CMD_READ_PARAM, param_id, 0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 写入控制参数 (data指向6字节数据) */
void ktech_write_param(hcan_t* hcan, uint16_t id, uint8_t param_id, uint8_t* data)
{
    uint8_t tx_data[8] = {KTECH_CMD_WRITE_PARAM, param_id,
                           data[0], data[1], data[2], data[3], data[4], data[5]};
    send_cmd(hcan, id, tx_data);
}

/* 读取编码器数据 */
void ktech_read_encoder(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_ENCODER, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 设置当前位置到ROM作为电机零点 */
void ktech_set_zero(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_SET_ZERO, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 读取多圈角度 */
void ktech_read_multi_angle(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_MULTI_ANGLE, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 读取单圈角度 */
void ktech_read_single_angle(hcan_t* hcan, uint16_t id)
{
    uint8_t data[8] = {KTECH_CMD_READ_SINGLE_ANGLE, 0,0,0,0,0,0,0};
    send_cmd(hcan, id, data);
}

/* 设置当前位置为任意角度（写入RAM） */
void ktech_set_angle_ram(hcan_t* hcan, uint16_t id, int32_t angle)
{
    uint8_t data[8] = {KTECH_CMD_SET_ANGLE_RAM, 0,0,0,
                        (uint8_t)angle, (uint8_t)(angle>>8),
                        (uint8_t)(angle>>16), (uint8_t)(angle>>24)};
    send_cmd(hcan, id, data);
}

/* CAN接收解析函数 */
void ktech_parse_motor_fb(KTech_Motor_t* motor, uint8_t* data)
{
    if (!motor) return;
    uint8_t cmd = data[0];

    switch (cmd) {
        case KTECH_CMD_READ_STATUS1:
        case KTECH_CMD_CLEAR_ERROR:
            motor->fb.temperature = (int8_t)data[1];
            motor->fb.voltage = (int16_t)(data[2] | (data[3] << 8));
            motor->fb.current = (int16_t)(data[4] | (data[5] << 8));
            motor->fb.motorState = data[6];
            motor->fb.errorState = data[7];
            break;

        case KTECH_CMD_READ_STATUS2:
        case KTECH_CMD_OPENLOOP:
        case KTECH_CMD_TORQUE_CLOSED:
        case KTECH_CMD_SPEED_CLOSED:
        case KTECH_CMD_POS_MULTI1:
        case KTECH_CMD_POS_MULTI2:
        case KTECH_CMD_POS_SINGLE1:
        case KTECH_CMD_POS_SINGLE2:
        case KTECH_CMD_POS_INCREMENT1:
        case KTECH_CMD_POS_INCREMENT2:
            motor->fb.temperature = (int8_t)data[1];
            motor->fb.iq_or_power = (int16_t)(data[2] | (data[3] << 8));
            motor->fb.speed = (int16_t)(data[4] | (data[5] << 8));
            motor->fb.encoder = (uint16_t)(data[6] | (data[7] << 8));
            break;

        case KTECH_CMD_READ_STATUS3:
            motor->fb.temperature = (int8_t)data[1];
            motor->fb.iA = (int16_t)(data[2] | (data[3] << 8));
            motor->fb.iB = (int16_t)(data[4] | (data[5] << 8));
            motor->fb.iC = (int16_t)(data[6] | (data[7] << 8));
            break;

        case KTECH_CMD_BRAKE:
            motor->fb.brakeState = data[1];
            break;

        case KTECH_CMD_READ_PARAM:
        case KTECH_CMD_WRITE_PARAM:
            motor->fb.paramID = data[1];
            for (int i = 0; i < 6; i++) {
                motor->fb.paramData[i] = data[2 + i];
            }
            break;

        case KTECH_CMD_READ_ENCODER:
            motor->fb.encoder = (uint16_t)(data[2] | (data[3] << 8));
            motor->fb.encoderRaw = (uint16_t)(data[4] | (data[5] << 8));
            motor->fb.encoderOffset = (uint16_t)(data[6] | (data[7] << 8));
            break;

        case KTECH_CMD_SET_ZERO:
            motor->fb.encoderOffset = (uint16_t)(data[6] | (data[7] << 8));
            break;

        case KTECH_CMD_READ_MULTI_ANGLE:
            {
                uint64_t tmp = 0;
                for (int i = 0; i < 7; i++) {
                    tmp |= (uint64_t)data[1 + i] << (8 * i);
                }
                if (data[7] & 0x80) {
                    tmp |= 0xFFULL << 56;
                }
                motor->fb.multiAngle = (int64_t)tmp;
            }
            break;

        case KTECH_CMD_READ_SINGLE_ANGLE:
            motor->fb.singleAngle = (uint32_t)(data[4] | (data[5] << 8) |
                                               (data[6] << 16) | (data[7] << 24));
            break;

        default:
            break;
    }
}