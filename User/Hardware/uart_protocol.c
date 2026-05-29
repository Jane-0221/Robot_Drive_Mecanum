#include "uart_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

uint8_t uart_protocol_raw_data[256] = {0};
static uint8_t uart_protocol_parse_data[256] = {0};
static volatile uint16_t uart_protocol_raw_size = 0U;
static volatile uint8_t uart_protocol_data_pending = 0U;
static PcMotorCtrl_t pc_motor_ctrl_pending = {0U, 0U, 0U};
static volatile uint8_t pc_motor_ctrl_pending_valid = 0U;

volatile PcMotorCtrl_t pc_motor_ctrl_latest = {
    PC_MOTOR_CTRL_STATE_NONE,
    PC_MOTOR_CTRL_STATE_NONE,
    PC_MOTOR_CTRL_STATE_NONE,
};
volatile uint8_t pc_motor_ctrl_latest_valid = 0U;
volatile uint32_t pc_motor_ctrl_rx_count = 0U;

volatile PcChassisCtrl_t pc_chassis_ctrl_latest = {
    .x = 0.0f,
    .y = 0.0f,
    .w = 0.0f,
};
volatile uint8_t pc_chassis_ctrl_latest_valid = 0U;
volatile uint32_t pc_chassis_ctrl_rx_count = 0U;
volatile uint32_t pc_chassis_ctrl_last_tick = 0U;

DnData_t pc_dn_data = {
    .pc_target_servo_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pc_target_motor_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pc_target_motor_velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pc_pump_state = 0,
    .pc_target_lift_height = 0,
    .pc_target_head_motor_angles = {0.0f, 0.0f},
};

UpData_t up_tx_data = {
    .air_path_state = 0.0f,
    .suck_state = 0.0f,
    .head_motor_angle_1 = 0.0f,
    .head_motor_angle_2 = 0.0f,
    .arm_motor_angle_1 = 0.0f,
    .arm_motor_angle_2 = 0.0f,
    .arm_motor_angle_3 = 0.0f,
    .arm_motor_angle_4 = 0.0f,
    .arm_motor_angle_5 = 0.0f,
    .arm_motor_angle_6 = 0.0f,
    .lift_height = 0.0f,
    .chassis_vx = 0.0f,
    .chassis_vy = 0.0f,
    .chassis_yaw = 0.0f,
};

static void write_float_le(uint8_t *buffer, uint16_t *idx, float value)
{
    memcpy(&buffer[*idx], &value, sizeof(float));
    *idx += (uint16_t)sizeof(float);
}

static float read_float_le(const uint8_t *buffer, uint16_t *idx)
{
    float value;

    memcpy(&value, &buffer[*idx], sizeof(float));
    *idx += (uint16_t)sizeof(float);

    return value;
}

uint16_t crc16_ccitt(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void pack_up_frame(UpData_t *data, uint8_t *frame_buf)
{
    uint16_t idx = 0;

    frame_buf[idx++] = FRAME_HEADER1;
    frame_buf[idx++] = FRAME_HEADER2;
    frame_buf[idx++] = UP_FRAME_TYPE;
    frame_buf[idx++] = UP_DATA_LEN;

    write_float_le(frame_buf, &idx, data->air_path_state);
    write_float_le(frame_buf, &idx, data->suck_state);
    write_float_le(frame_buf, &idx, data->head_motor_angle_1);
    write_float_le(frame_buf, &idx, data->head_motor_angle_2);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_1);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_2);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_3);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_4);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_5);
    write_float_le(frame_buf, &idx, data->arm_motor_angle_6);
    write_float_le(frame_buf, &idx, data->lift_height);
    write_float_le(frame_buf, &idx, data->chassis_vx);
    write_float_le(frame_buf, &idx, data->chassis_vy);
    write_float_le(frame_buf, &idx, data->chassis_yaw);

    {
        uint16_t crc = crc16_ccitt(&frame_buf[2], UP_DATA_LEN + 2);
        frame_buf[idx++] = (uint8_t)(crc & 0xFF);
        frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    }

    frame_buf[idx++] = FRAME_TAIL1;
    frame_buf[idx++] = FRAME_TAIL2;
}

void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data)
{
    uint8_t idx = 4;

    for (int i = 0; i < 6; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_servo_angles[i] = value;
        idx += 4;
    }

    for (int i = 0; i < 6; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_motor_angles[i] = value;
        idx += 4;
    }

    for (int i = 0; i < 6; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_motor_velocities[i] = value;
        idx += 4;
    }

    data->pc_pump_state = frame_buf[idx++];
    data->pc_target_lift_height = (uint16_t)((frame_buf[idx] | (frame_buf[idx + 1] << 8)) / 10);
    idx += 2;

    for (int i = 0; i < 2; i++)
    {
        float value;
        uint8_t *p = (uint8_t *)&value;
        p[0] = frame_buf[idx];
        p[1] = frame_buf[idx + 1];
        p[2] = frame_buf[idx + 2];
        p[3] = frame_buf[idx + 3];
        data->pc_target_head_motor_angles[i] = value;
        idx += 4;
    }
}

static uint8_t is_valid_dn_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != DN_FRAME_TYPE) ||
        (frame_buf[3] != DN_DATA_LEN) ||
        (frame_buf[DN_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[DN_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[DN_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[DN_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], DN_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static uint8_t is_valid_pc_motor_ctrl_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != PC_MOTOR_CTRL_FRAME_TYPE) ||
        (frame_buf[3] != PC_MOTOR_CTRL_DATA_LEN) ||
        (frame_buf[PC_MOTOR_CTRL_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[PC_MOTOR_CTRL_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[PC_MOTOR_CTRL_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[PC_MOTOR_CTRL_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], PC_MOTOR_CTRL_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static uint8_t is_valid_pc_chassis_ctrl_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != PC_CHASSIS_CTRL_FRAME_TYPE) ||
        (frame_buf[3] != PC_CHASSIS_CTRL_DATA_LEN) ||
        (frame_buf[PC_CHASSIS_CTRL_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[PC_CHASSIS_CTRL_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[PC_CHASSIS_CTRL_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[PC_CHASSIS_CTRL_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], PC_CHASSIS_CTRL_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static uint8_t normalize_ascii_digit(uint8_t value)
{
    if ((value >= (uint8_t)'0') && (value <= (uint8_t)'9'))
    {
        return (uint8_t)(value - (uint8_t)'0');
    }

    return value;
}

static void unpack_pc_motor_ctrl_frame(const uint8_t *frame_buf, PcMotorCtrl_t *command)
{
    if ((frame_buf == NULL) || (command == NULL))
    {
        return;
    }

    command->target_type = normalize_ascii_digit(frame_buf[4]);
    command->motor_index = normalize_ascii_digit(frame_buf[5]);
    command->command = normalize_ascii_digit(frame_buf[6]);
}

static void unpack_pc_chassis_ctrl_frame(const uint8_t *frame_buf, PcChassisCtrl_t *command)
{
    uint16_t idx = 4U;

    if ((frame_buf == NULL) || (command == NULL))
    {
        return;
    }

    command->x = read_float_le(frame_buf, &idx);
    command->y = read_float_le(frame_buf, &idx);
    command->w = read_float_le(frame_buf, &idx);
}

uint8_t UART_Protocol_UnpackLatest(DnData_t *data)
{
    uint16_t size;
    uint16_t frame_pos = 0U;
    DnData_t dn_data_local;
    PcMotorCtrl_t motor_ctrl_command = {0U, 0U, 0U};
    PcChassisCtrl_t chassis_ctrl_command = {0.0f, 0.0f, 0.0f};
    uint8_t dn_found = 0U;
    uint8_t motor_ctrl_found = 0U;
    uint8_t chassis_ctrl_found = 0U;

    if ((data == NULL) || (uart_protocol_data_pending == 0U))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    size = uart_protocol_raw_size;
    memcpy(uart_protocol_parse_data, uart_protocol_raw_data, size);
    uart_protocol_data_pending = 0U;
    taskEXIT_CRITICAL();

    if (size < PC_MOTOR_CTRL_FRAME_LEN)
    {
        return 0U;
    }

    for (uint16_t pos = 0U; pos < size; pos++)
    {
        uint16_t remaining = (uint16_t)(size - pos);

        if ((remaining >= DN_FRAME_LEN) &&
            (is_valid_dn_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            frame_pos = pos;
            dn_found = 1U;
        }

        if ((remaining >= PC_MOTOR_CTRL_FRAME_LEN) &&
            (is_valid_pc_motor_ctrl_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            unpack_pc_motor_ctrl_frame(&uart_protocol_parse_data[pos], &motor_ctrl_command);
            motor_ctrl_found = 1U;
        }

        if ((remaining >= PC_CHASSIS_CTRL_FRAME_LEN) &&
            (is_valid_pc_chassis_ctrl_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            unpack_pc_chassis_ctrl_frame(&uart_protocol_parse_data[pos], &chassis_ctrl_command);
            chassis_ctrl_found = 1U;
        }
    }

    if (motor_ctrl_found != 0U)
    {
        taskENTER_CRITICAL();
        pc_motor_ctrl_pending = motor_ctrl_command;
        pc_motor_ctrl_pending_valid = 1U;
        pc_motor_ctrl_latest = motor_ctrl_command;
        pc_motor_ctrl_latest_valid = 1U;
        pc_motor_ctrl_rx_count++;
        taskEXIT_CRITICAL();
    }

    if (chassis_ctrl_found != 0U)
    {
        uint32_t rx_tick = HAL_GetTick();

        taskENTER_CRITICAL();
        pc_chassis_ctrl_latest = chassis_ctrl_command;
        pc_chassis_ctrl_latest_valid = 1U;
        pc_chassis_ctrl_last_tick = rx_tick;
        pc_chassis_ctrl_rx_count++;
        taskEXIT_CRITICAL();
    }

    if (dn_found != 0U)
    {
        unpack_dn_frame(&uart_protocol_parse_data[frame_pos], &dn_data_local);

        taskENTER_CRITICAL();
        *data = dn_data_local;
        taskEXIT_CRITICAL();
    }

    return ((dn_found != 0U) ||
            (motor_ctrl_found != 0U) ||
            (chassis_ctrl_found != 0U)) ? 1U : 0U;
}

uint8_t UART_Protocol_CopyLatestDnData(DnData_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = pc_dn_data;
    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t UART_Protocol_GetMotorCtrlCommand(PcMotorCtrl_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (pc_motor_ctrl_pending_valid == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *command = pc_motor_ctrl_pending;
    pc_motor_ctrl_pending_valid = 0U;
    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t UART_Protocol_CopyLatestChassisCtrl(PcChassisCtrl_t *out, uint32_t *last_tick)
{
    if ((out == NULL) || (last_tick == NULL))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (pc_chassis_ctrl_latest_valid == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *out = pc_chassis_ctrl_latest;
    *last_tick = pc_chassis_ctrl_last_tick;
    taskEXIT_CRITICAL();

    return 1U;
}

HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len)
{
    return HAL_UART_Transmit(huart, frame_buf, len, 100);
}

HAL_StatusTypeDef send_up_frame(UART_HandleTypeDef *huart)
{
    static uint8_t frame_buf[UP_FRAME_LEN];

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    pack_up_frame(&up_tx_data, frame_buf);

    return HAL_UART_Transmit_DMA(huart, frame_buf, UP_FRAME_LEN);
}

void store_uart_protocol_data(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0)
    {
        return;
    }

    uint16_t copy_size = (size > 256U) ? 256U : size;

    memcpy(uart_protocol_raw_data, data, copy_size);
    uart_protocol_raw_size = copy_size;
    uart_protocol_data_pending = 1U;
}
