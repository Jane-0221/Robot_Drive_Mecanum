#ifndef KVMS_BATTERY_H
#define KVMS_BATTERY_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * KVMS BMS Modbus map used by the chassis battery monitor.
 *
 * Request on RS485:
 *   slave 0x81, function 0x03, start register 0x0000, count 127.
 * Expected response:
 *   slave 0x51, function 0x03, byte count 254, payload 127 registers.
 */

#define KVMS_BATTERY_REQUEST_ADDR        0x81U
#define KVMS_BATTERY_RESPONSE_ADDR       0x51U
#define KVMS_BATTERY_FUNCTION_READ       0x03U
#define KVMS_BATTERY_START_REGISTER      0x0000U
#define KVMS_BATTERY_REGISTER_COUNT      127U
#define KVMS_BATTERY_RESPONSE_DATA_BYTES (KVMS_BATTERY_REGISTER_COUNT * 2U)
#define KVMS_BATTERY_RESPONSE_LENGTH     (3U + KVMS_BATTERY_RESPONSE_DATA_BYTES + 2U)
#define KVMS_BATTERY_RX_BUFFER_SIZE      260U
#define KVMS_BATTERY_CELL_MAX            48U
#define KVMS_BATTERY_TEMP_MAX            8U
#define KVMS_BATTERY_BALANCE_REG_COUNT   3U
#define KVMS_BATTERY_FAULT_REG_COUNT     7U
#define KVMS_BATTERY_CURRENT_DEADBAND_A  0.05f
#define KVMS_BATTERY_USAGE_MAX_INTERVAL_MS 5000U

#define KVMS_BATTERY_ERROR_NONE          0x00000000UL
#define KVMS_BATTERY_ERROR_SHORT_FRAME   0x00000001UL
#define KVMS_BATTERY_ERROR_LENGTH        0x00000002UL
#define KVMS_BATTERY_ERROR_ADDRESS       0x00000004UL
#define KVMS_BATTERY_ERROR_FUNCTION      0x00000008UL
#define KVMS_BATTERY_ERROR_BYTE_COUNT    0x00000010UL
#define KVMS_BATTERY_ERROR_CRC           0x00000020UL
#define KVMS_BATTERY_ERROR_EXCEPTION     0x00000040UL

/* BMS charge-state register after protocol normalization. */
typedef enum
{
  KVMS_BATTERY_CHARGE_STATE_IDLE = 0,
  KVMS_BATTERY_CHARGE_STATE_CHARGE = 1,
  KVMS_BATTERY_CHARGE_STATE_DISCHARGE = 2,
  KVMS_BATTERY_CHARGE_STATE_UNKNOWN = 255
} KvmsBatteryChargeState_t;

/* BMS balance-mode register after protocol normalization. */
typedef enum
{
  KVMS_BATTERY_BALANCE_OFF = 0,
  KVMS_BATTERY_BALANCE_PASSIVE = 1,
  KVMS_BATTERY_BALANCE_ACTIVE = 2,
  KVMS_BATTERY_BALANCE_UNKNOWN = 255
} KvmsBatteryBalanceMode_t;

/*
 * Full decoded data block.
 *
 * This structure keeps raw registers and detailed fields for application code.
 * For simple debugging, prefer kvms_battery_result or the scalar *_debug
 * mirrors below.
 */
typedef struct
{
  uint8_t valid;
  uint8_t response_addr;
  uint8_t response_function;
  uint8_t charge_state_code;
  uint8_t cell_count;
  uint8_t temp_count;
  uint8_t max_cell_number;
  uint8_t min_cell_number;
  uint8_t max_temp_number;
  uint8_t min_temp_number;
  uint8_t charger_connected;
  uint8_t load_connected;
  uint8_t charge_mos;
  uint8_t discharge_mos;
  uint8_t precharge_mos;
  uint8_t heating_mos;
  uint8_t fan_mos;
  uint8_t current_limit_enabled;
  uint8_t rtc_valid;
  uint8_t rtc_year;
  uint8_t rtc_month;
  uint8_t rtc_day;
  uint8_t rtc_hour;
  uint8_t rtc_minute;
  uint8_t rtc_second;
  uint8_t wake_sources_mask;
  uint8_t temp_valid[KVMS_BATTERY_TEMP_MAX];
  uint8_t balances[KVMS_BATTERY_CELL_MAX];

  KvmsBatteryChargeState_t charge_state;
  KvmsBatteryBalanceMode_t balance_mode;

  uint16_t life;
  uint16_t cycles;
  uint16_t energy_wh;
  uint16_t remaining_charge_minutes;
  uint16_t max_cell_voltage_mv;
  uint16_t min_cell_voltage_mv;
  uint16_t delta_cell_voltage_mv;
  uint16_t cell_voltage_mv[KVMS_BATTERY_CELL_MAX];
  uint16_t raw_registers[KVMS_BATTERY_REGISTER_COUNT];
  uint16_t balance_raw[KVMS_BATTERY_BALANCE_REG_COUNT];
  uint16_t fault_raw[KVMS_BATTERY_FAULT_REG_COUNT];

  int16_t temperatures_c[KVMS_BATTERY_TEMP_MAX];
  int16_t max_temp_c;
  int16_t min_temp_c;
  int16_t delta_temp_c;
  int16_t mos_temp_c;
  int16_t ambient_temp_c;
  int16_t heating_temp_c;
  int16_t heating_current;

  float pack_voltage_v;
  float current_a;
  float soc_percent;
  float max_cell_voltage_v;
  float min_cell_voltage_v;
  float delta_cell_voltage_v;
  float remain_capacity_ah;
  float avg_voltage_v;
  float power_w;
  float charge_power_w;
  float discharge_power_w;
  float current_limit_current_a;
  float used_energy_wh;
  float used_capacity_ah;
  float charged_energy_wh;
  float charged_capacity_ah;

  uint32_t last_update_tick;
  uint32_t usage_last_tick;
  uint32_t rx_count;
  uint32_t rx_echo_count;
  uint32_t parse_ok_count;
  uint32_t frame_error_count;
  uint32_t crc_error_count;
  uint32_t tx_count;
  uint32_t tx_error_count;
  uint32_t last_error_flags;
} KvmsBatteryData_t;

/* Compact result for dashboards, DAP Live Watch and higher-level status copy. */
typedef struct
{
  uint8_t valid;                 /* 1: last frame was parsed successfully. */
  float pack_voltage_v;          /* Pack total voltage, V. */
  float current_a;               /* Positive is discharge, negative is charge. */
  float soc_percent;             /* State of charge, %. */
  float power_w;                 /* BMS reported power, W. */
  float charge_power_w;          /* Calculated charge power, W. */
  float discharge_power_w;       /* Calculated discharge power, W. */
  float delta_cell_voltage_v;    /* Max cell voltage - min cell voltage, V. */
  float remaining_capacity_ah;   /* BMS reported remaining capacity, Ah. */
  float used_energy_wh;          /* Integrated discharge energy since boot/reset, Wh. */
  float used_capacity_ah;        /* Integrated discharge capacity since boot/reset, Ah. */
  float charged_energy_wh;       /* Integrated charge energy since boot/reset, Wh. */
  float charged_capacity_ah;     /* Integrated charge capacity since boot/reset, Ah. */
  uint32_t last_update_tick;
  uint32_t rx_count;
  uint32_t rx_echo_count;
  uint32_t parse_ok_count;
  uint32_t frame_error_count;
  uint32_t crc_error_count;
  uint32_t tx_count;
  uint32_t tx_error_count;
  uint32_t last_error_flags;
} KvmsBatteryResult_t;

/* Main watch variable. Add this one to VSCode Live Watch and expand it. */
extern volatile KvmsBatteryResult_t kvms_battery_result;

/* Scalar mirrors for tools that cannot conveniently expand structures. */
extern volatile uint8_t kvms_battery_valid_debug;
extern volatile float kvms_battery_total_voltage_v_debug;
extern volatile float kvms_battery_current_a_debug;
extern volatile float kvms_battery_soc_percent_debug;
extern volatile float kvms_battery_power_w_debug;
extern volatile float kvms_battery_charge_power_w_debug;
extern volatile float kvms_battery_discharge_power_w_debug;
extern volatile float kvms_battery_delta_voltage_v_debug;
extern volatile float kvms_battery_remaining_capacity_ah_debug;
extern volatile float kvms_battery_used_energy_wh_debug;
extern volatile float kvms_battery_used_capacity_ah_debug;
extern volatile float kvms_battery_charged_energy_wh_debug;
extern volatile float kvms_battery_charged_capacity_ah_debug;
extern volatile uint32_t kvms_battery_last_update_tick_debug;
extern volatile uint32_t kvms_battery_rx_count_debug;
extern volatile uint32_t kvms_battery_rx_echo_count_debug;
extern volatile uint32_t kvms_battery_rx_stream_overflow_count_debug;
extern volatile uint32_t kvms_battery_parse_ok_count_debug;
extern volatile uint32_t kvms_battery_frame_error_count_debug;
extern volatile uint32_t kvms_battery_crc_error_count_debug;
extern volatile uint32_t kvms_battery_tx_count_debug;
extern volatile uint32_t kvms_battery_tx_error_count_debug;
extern volatile uint32_t kvms_battery_last_error_flags_debug;
extern volatile uint32_t kvms_battery_last_poll_status_debug;
extern volatile uint32_t kvms_battery_uart_isr_debug;
extern volatile uint32_t kvms_battery_uart_gstate_debug;
extern volatile uint32_t kvms_battery_uart_rxstate_debug;
extern volatile uint32_t kvms_battery_uart_error_code_debug;
extern volatile uint32_t kvms_battery_rx_dma_remaining_debug;
extern volatile uint32_t kvms_battery_last_rx_size_debug;
extern volatile uint32_t kvms_battery_last_rx_addr_debug;
extern volatile uint32_t kvms_battery_last_rx_function_debug;
extern volatile uint32_t kvms_battery_last_rx_byte_count_debug;

void KvmsBattery_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef KvmsBattery_Poll(void);
uint8_t KvmsBattery_CopyData(KvmsBatteryData_t *out);
uint8_t KvmsBattery_CopyResult(KvmsBatteryResult_t *out);
const KvmsBatteryData_t *KvmsBattery_GetData(void);
void KvmsBattery_ResetUsage(void);
void KvmsBattery_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void KvmsBattery_RestartRx(void);

#ifdef __cplusplus
}
#endif

#endif
