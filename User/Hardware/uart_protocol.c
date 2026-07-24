#include "uart_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "kvms_battery.h"
#include "USB_VirCom.h"
#include <string.h>

uint8_t uart_protocol_raw_data[256] = {0};
static uint8_t uart_protocol_parse_data[256] = {0};
static volatile uint16_t uart_protocol_raw_size = 0U;
static volatile uint8_t uart_protocol_data_pending = 0U;
static volatile uint8_t pc_control_mode_pending = PC_CONTROL_MODE_RC;
static volatile uint8_t pc_control_mode_pending_valid = 0U;
static PcMotorCtrl_t pc_motor_ctrl_pending = {0U, 0U, 0U};
static volatile uint8_t pc_motor_ctrl_pending_valid = 0U;
static PcRs485LiftCtrl_t pc_rs485_lift_ctrl_pending = {0};
static volatile uint8_t pc_rs485_lift_ctrl_pending_valid = 0U;

volatile uint32_t pc_dn_rx_count = 0U;
volatile uint8_t pc_control_mode_latest = PC_CONTROL_MODE_RC;
volatile uint8_t pc_control_mode_latest_valid = 0U;
volatile uint32_t pc_control_mode_rx_count = 0U;
volatile uint32_t pc_control_mode_last_tick = 0U;

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

volatile PcRs485LiftCtrl_t pc_rs485_lift_ctrl_latest = {
    .command = PC_RS485_LIFT_CMD_NONE,
    .flags = 0U,
    .rpm = 0U,
    .accel_rpm = 0U,
    .move_mm = 0.0f,
    .target_height_mm = 0.0f,
    .current_height_mm = 0.0f,
    .manual_lower_mm = 0.0f,
    .manual_upper_mm = 0.0f,
    .command_distance_mm = 0.0f,
    .actual_distance_mm = 0.0f,
};
volatile uint8_t pc_rs485_lift_ctrl_latest_valid = 0U;
volatile uint32_t pc_rs485_lift_ctrl_rx_count = 0U;

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
    .wheel_speed_lf = 0.0f,
    .wheel_speed_rf = 0.0f,
    .wheel_speed_rb = 0.0f,
    .wheel_speed_lb = 0.0f,
};

static void write_float_le(uint8_t *buffer, uint16_t *idx, float value)
{
    memcpy(&buffer[*idx], &value, sizeof(float));
    *idx += (uint16_t)sizeof(float);
}

static void write_u8(uint8_t *buffer, uint16_t *idx, uint8_t value)
{
    buffer[*idx] = value;
    *idx += 1U;
}

static void write_u16_le(uint8_t *buffer, uint16_t *idx, uint16_t value)
{
    buffer[(*idx)++] = (uint8_t)(value & 0xFFU);
    buffer[(*idx)++] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_i16_le(uint8_t *buffer, uint16_t *idx, int16_t value)
{
    write_u16_le(buffer, idx, (uint16_t)value);
}

static void write_u32_le(uint8_t *buffer, uint16_t *idx, uint32_t value)
{
    buffer[(*idx)++] = (uint8_t)(value & 0xFFUL);
    buffer[(*idx)++] = (uint8_t)((value >> 8U) & 0xFFUL);
    buffer[(*idx)++] = (uint8_t)((value >> 16U) & 0xFFUL);
    buffer[(*idx)++] = (uint8_t)((value >> 24U) & 0xFFUL);
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

    write_float_le(frame_buf, &idx, data->wheel_speed_lf);
    write_float_le(frame_buf, &idx, data->wheel_speed_rf);
    write_float_le(frame_buf, &idx, data->wheel_speed_rb);
    write_float_le(frame_buf, &idx, data->wheel_speed_lb);

    {
        uint16_t crc = crc16_ccitt(&frame_buf[2], UP_DATA_LEN + 2);
        frame_buf[idx++] = (uint8_t)(crc & 0xFF);
        frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    }

    frame_buf[idx++] = FRAME_TAIL1;
    frame_buf[idx++] = FRAME_TAIL2;
}

static uint16_t battery_status_flags(const KvmsBatteryData_t *data)
{
    uint16_t flags = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    flags |= (data->charger_connected != 0U) ? (uint16_t)(1U << 0U) : 0U;
    flags |= (data->load_connected != 0U) ? (uint16_t)(1U << 1U) : 0U;
    flags |= (data->charge_mos != 0U) ? (uint16_t)(1U << 2U) : 0U;
    flags |= (data->discharge_mos != 0U) ? (uint16_t)(1U << 3U) : 0U;
    flags |= (data->precharge_mos != 0U) ? (uint16_t)(1U << 4U) : 0U;
    flags |= (data->heating_mos != 0U) ? (uint16_t)(1U << 5U) : 0U;
    flags |= (data->fan_mos != 0U) ? (uint16_t)(1U << 6U) : 0U;
    flags |= (data->current_limit_enabled != 0U) ? (uint16_t)(1U << 7U) : 0U;
    flags |= (data->rtc_valid != 0U) ? (uint16_t)(1U << 8U) : 0U;

    return flags;
}

static void pack_battery_up_frame(const KvmsBatteryData_t *data, uint8_t *frame_buf)
{
    uint16_t idx = 0U;
    uint8_t i;

    frame_buf[idx++] = FRAME_HEADER1;
    frame_buf[idx++] = FRAME_HEADER2;
    frame_buf[idx++] = PC_BATTERY_UP_FRAME_TYPE;
    frame_buf[idx++] = PC_BATTERY_UP_DATA_LEN;

    write_u8(frame_buf, &idx, PC_BATTERY_UP_PAYLOAD_VERSION);
    write_u8(frame_buf, &idx, data->valid);
    write_u8(frame_buf, &idx, data->cell_count);
    write_u8(frame_buf, &idx, data->temp_count);
    write_u8(frame_buf, &idx, data->charge_state_code);
    write_u8(frame_buf, &idx, (uint8_t)data->balance_mode);
    write_u8(frame_buf, &idx, data->wake_sources_mask);
    write_u8(frame_buf, &idx, data->max_cell_number);
    write_u8(frame_buf, &idx, data->min_cell_number);
    write_u8(frame_buf, &idx, data->max_temp_number);
    write_u8(frame_buf, &idx, data->min_temp_number);
    write_u16_le(frame_buf, &idx, battery_status_flags(data));

    write_u32_le(frame_buf, &idx, data->last_update_tick);
    write_u32_le(frame_buf, &idx, data->last_error_flags);
    write_u32_le(frame_buf, &idx, data->rx_count);
    write_u32_le(frame_buf, &idx, data->parse_ok_count);
    write_u32_le(frame_buf, &idx, data->frame_error_count);
    write_u32_le(frame_buf, &idx, data->crc_error_count);
    write_u32_le(frame_buf, &idx, data->tx_count);
    write_u32_le(frame_buf, &idx, data->tx_error_count);

    write_float_le(frame_buf, &idx, data->pack_voltage_v);
    write_float_le(frame_buf, &idx, data->current_a);
    write_float_le(frame_buf, &idx, data->soc_percent);
    write_float_le(frame_buf, &idx, data->remain_capacity_ah);
    write_float_le(frame_buf, &idx, data->power_w);
    write_float_le(frame_buf, &idx, data->charge_power_w);
    write_float_le(frame_buf, &idx, data->discharge_power_w);
    write_float_le(frame_buf, &idx, data->avg_voltage_v);

    write_u16_le(frame_buf, &idx, data->cycles);
    write_u16_le(frame_buf, &idx, data->energy_wh);
    write_u16_le(frame_buf, &idx, data->remaining_charge_minutes);
    write_float_le(frame_buf, &idx, data->current_limit_current_a);

    for (i = 0U; i < KVMS_BATTERY_CELL_MAX; i++)
    {
        write_u16_le(frame_buf, &idx, data->cell_voltage_mv[i]);
    }

    for (i = 0U; i < KVMS_BATTERY_TEMP_MAX; i++)
    {
        write_i16_le(frame_buf, &idx, data->temperatures_c[i]);
    }

    write_u16_le(frame_buf, &idx, data->max_cell_voltage_mv);
    write_u16_le(frame_buf, &idx, data->min_cell_voltage_mv);
    write_u16_le(frame_buf, &idx, data->delta_cell_voltage_mv);
    write_i16_le(frame_buf, &idx, data->max_temp_c);
    write_i16_le(frame_buf, &idx, data->min_temp_c);
    write_i16_le(frame_buf, &idx, data->delta_temp_c);
    write_i16_le(frame_buf, &idx, data->mos_temp_c);
    write_i16_le(frame_buf, &idx, data->ambient_temp_c);
    write_i16_le(frame_buf, &idx, data->heating_temp_c);

    for (i = 0U; i < KVMS_BATTERY_BALANCE_REG_COUNT; i++)
    {
        write_u16_le(frame_buf, &idx, data->balance_raw[i]);
    }

    for (i = 0U; i < KVMS_BATTERY_FAULT_REG_COUNT; i++)
    {
        write_u16_le(frame_buf, &idx, data->fault_raw[i]);
    }

    {
        uint16_t crc = crc16_ccitt(&frame_buf[2], PC_BATTERY_UP_DATA_LEN + 2U);
        frame_buf[idx++] = (uint8_t)(crc & 0xFFU);
        frame_buf[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);
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

static uint8_t is_valid_pc_rs485_lift_ctrl_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != PC_RS485_LIFT_CTRL_FRAME_TYPE) ||
        (frame_buf[3] != PC_RS485_LIFT_CTRL_DATA_LEN) ||
        (frame_buf[PC_RS485_LIFT_CTRL_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[PC_RS485_LIFT_CTRL_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[PC_RS485_LIFT_CTRL_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[PC_RS485_LIFT_CTRL_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], PC_RS485_LIFT_CTRL_DATA_LEN + 2U);

    return (crc_rx == crc_calc) ? 1U : 0U;
}

static uint8_t is_valid_pc_control_mode_frame(const uint8_t *frame_buf)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (frame_buf == NULL)
    {
        return 0U;
    }

    if ((frame_buf[0] != FRAME_HEADER1) ||
        (frame_buf[1] != FRAME_HEADER2) ||
        (frame_buf[2] != PC_CONTROL_MODE_FRAME_TYPE) ||
        (frame_buf[3] != PC_CONTROL_MODE_DATA_LEN) ||
        (frame_buf[PC_CONTROL_MODE_FRAME_LEN - 2U] != FRAME_TAIL1) ||
        (frame_buf[PC_CONTROL_MODE_FRAME_LEN - 1U] != FRAME_TAIL2))
    {
        return 0U;
    }

    crc_rx = (uint16_t)frame_buf[PC_CONTROL_MODE_DATA_LEN + 4U] |
             ((uint16_t)frame_buf[PC_CONTROL_MODE_DATA_LEN + 5U] << 8);
    crc_calc = crc16_ccitt((uint8_t *)&frame_buf[2], PC_CONTROL_MODE_DATA_LEN + 2U);

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

static uint16_t read_u16_le(const uint8_t *buffer, uint16_t *idx)
{
    uint16_t value = (uint16_t)buffer[*idx] | ((uint16_t)buffer[*idx + 1U] << 8U);
    *idx += 2U;
    return value;
}

static void unpack_pc_rs485_lift_ctrl_frame(const uint8_t *frame_buf, PcRs485LiftCtrl_t *command)
{
    uint16_t idx = 4U;

    if ((frame_buf == NULL) || (command == NULL))
    {
        return;
    }

    command->command = normalize_ascii_digit(frame_buf[idx++]);
    command->flags = frame_buf[idx++];
    command->rpm = read_u16_le(frame_buf, &idx);
    command->accel_rpm = read_u16_le(frame_buf, &idx);
    command->move_mm = read_float_le(frame_buf, &idx);
    command->target_height_mm = read_float_le(frame_buf, &idx);
    command->current_height_mm = read_float_le(frame_buf, &idx);
    command->manual_lower_mm = read_float_le(frame_buf, &idx);
    command->manual_upper_mm = read_float_le(frame_buf, &idx);
    command->command_distance_mm = read_float_le(frame_buf, &idx);
    command->actual_distance_mm = read_float_le(frame_buf, &idx);
}

uint8_t UART_Protocol_UnpackLatest(DnData_t *data)
{
    uint16_t size;
    uint16_t frame_pos = 0U;
    DnData_t dn_data_local;
    uint8_t control_mode_command = PC_CONTROL_MODE_RC;
    PcMotorCtrl_t motor_ctrl_command = {0U, 0U, 0U};
    PcChassisCtrl_t chassis_ctrl_command = {0.0f, 0.0f, 0.0f};
    PcRs485LiftCtrl_t rs485_lift_ctrl_command = {0};
    uint8_t dn_found = 0U;
    uint8_t control_mode_found = 0U;
    uint8_t motor_ctrl_found = 0U;
    uint8_t chassis_ctrl_found = 0U;
    uint8_t rs485_lift_ctrl_found = 0U;

    if ((data == NULL) || (uart_protocol_data_pending == 0U))
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    size = uart_protocol_raw_size;
    memcpy(uart_protocol_parse_data, uart_protocol_raw_data, size);
    uart_protocol_data_pending = 0U;
    taskEXIT_CRITICAL();

    if (size < PC_CONTROL_MODE_FRAME_LEN)
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

        if ((remaining >= PC_CONTROL_MODE_FRAME_LEN) &&
            (is_valid_pc_control_mode_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            uint8_t requested_mode = normalize_ascii_digit(uart_protocol_parse_data[pos + 4U]);
            if ((requested_mode == PC_CONTROL_MODE_RC) ||
                (requested_mode == PC_CONTROL_MODE_PC))
            {
                control_mode_command = requested_mode;
                control_mode_found = 1U;
            }
        }

        if ((remaining >= PC_CHASSIS_CTRL_FRAME_LEN) &&
            (is_valid_pc_chassis_ctrl_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            unpack_pc_chassis_ctrl_frame(&uart_protocol_parse_data[pos], &chassis_ctrl_command);
            chassis_ctrl_found = 1U;
        }

        if ((remaining >= PC_RS485_LIFT_CTRL_FRAME_LEN) &&
            (is_valid_pc_rs485_lift_ctrl_frame(&uart_protocol_parse_data[pos]) != 0U))
        {
            unpack_pc_rs485_lift_ctrl_frame(&uart_protocol_parse_data[pos], &rs485_lift_ctrl_command);
            rs485_lift_ctrl_found = 1U;
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

    if (control_mode_found != 0U)
    {
        uint32_t rx_tick = HAL_GetTick();

        taskENTER_CRITICAL();
        pc_control_mode_pending = control_mode_command;
        pc_control_mode_pending_valid = 1U;
        pc_control_mode_latest = control_mode_command;
        pc_control_mode_latest_valid = 1U;
        pc_control_mode_last_tick = rx_tick;
        pc_control_mode_rx_count++;
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

    if (rs485_lift_ctrl_found != 0U)
    {
        taskENTER_CRITICAL();
        pc_rs485_lift_ctrl_pending = rs485_lift_ctrl_command;
        pc_rs485_lift_ctrl_pending_valid = 1U;
        pc_rs485_lift_ctrl_latest = rs485_lift_ctrl_command;
        pc_rs485_lift_ctrl_latest_valid = 1U;
        pc_rs485_lift_ctrl_rx_count++;
        taskEXIT_CRITICAL();
    }

    if (dn_found != 0U)
    {
        unpack_dn_frame(&uart_protocol_parse_data[frame_pos], &dn_data_local);

        taskENTER_CRITICAL();
        *data = dn_data_local;
        pc_dn_rx_count++;
        taskEXIT_CRITICAL();
    }

    return ((dn_found != 0U) ||
            (control_mode_found != 0U) ||
            (motor_ctrl_found != 0U) ||
            (chassis_ctrl_found != 0U) ||
            (rs485_lift_ctrl_found != 0U)) ? 1U : 0U;
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

uint8_t UART_Protocol_GetControlModeCommand(uint8_t *mode)
{
    if (mode == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (pc_control_mode_pending_valid == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *mode = pc_control_mode_pending;
    pc_control_mode_pending_valid = 0U;
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

uint8_t UART_Protocol_GetRs485LiftCommand(PcRs485LiftCtrl_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    if (pc_rs485_lift_ctrl_pending_valid == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    *command = pc_rs485_lift_ctrl_pending;
    pc_rs485_lift_ctrl_pending_valid = 0U;
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

HAL_StatusTypeDef send_up_frame_usb(void)
{
    static uint8_t frame_buf[UP_FRAME_LEN];

    if (VirCom_TxReady() == 0U)
    {
        return HAL_BUSY;
    }

    pack_up_frame(&up_tx_data, frame_buf);

    return VirCom_try_send(frame_buf, UP_FRAME_LEN);
}

HAL_StatusTypeDef send_battery_up_frame(UART_HandleTypeDef *huart)
{
    static uint8_t frame_buf[PC_BATTERY_UP_FRAME_LEN];
    KvmsBatteryData_t battery_data;

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    if (huart->gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    (void)KvmsBattery_CopyData(&battery_data);
    pack_battery_up_frame(&battery_data, frame_buf);

    return HAL_UART_Transmit_DMA(huart, frame_buf, PC_BATTERY_UP_FRAME_LEN);
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
