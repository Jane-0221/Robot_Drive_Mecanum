#include "kvms_battery.h"

#include <stddef.h>
#include <string.h>
#include "stm32h7xx_hal.h"

/*
 * KVMS battery BMS driver over USART RS485.
 *
 * Runtime flow:
 *   Battery_BMS_Task -> KvmsBattery_Poll() sends one Modbus read request.
 *   HAL UART RX idle callback -> KvmsBattery_OnRxEvent() appends DMA bytes.
 *   KvmsBattery_ProcessPending() joins split RX chunks and parses full frames.
 *   KvmsBattery_UpdateResult() publishes compact values for VSCode Live Watch.
 *
 * Keep the receive path stream based. The BMS response is 259 bytes and may be
 * delivered by HAL_UARTEx_ReceiveToIdle_DMA() in multiple chunks.
 */

/* ----------------------------- Protocol constants ----------------------------- */

#define KVMS_BATTERY_REQUEST_LENGTH 8U
#define KVMS_BATTERY_CURRENT_OFFSET 30000
#define KVMS_BATTERY_RX_STREAM_SIZE (KVMS_BATTERY_RX_BUFFER_SIZE * 3U)

/* ------------------------------ Driver state ---------------------------------- */

static UART_HandleTypeDef *kvms_huart = NULL;
static uint8_t kvms_tx_dma_buf[KVMS_BATTERY_REQUEST_LENGTH];
static uint8_t kvms_rx_dma_buf[KVMS_BATTERY_RX_BUFFER_SIZE];
static uint8_t kvms_rx_stream_buf[KVMS_BATTERY_RX_STREAM_SIZE];
static uint8_t kvms_rx_process_buf[KVMS_BATTERY_RX_STREAM_SIZE];
static uint8_t kvms_parse_buf[KVMS_BATTERY_RX_STREAM_SIZE];
static volatile uint16_t kvms_rx_stream_size = 0U;
static volatile uint8_t kvms_rx_stream_pending = 0U;
static volatile uint32_t kvms_rx_stream_overflow_count = 0U;
static uint16_t kvms_parse_size = 0U;
static KvmsBatteryData_t kvms_data;

/* Compact result and scalar mirrors for DAP/VSCode watch windows. */
volatile KvmsBatteryResult_t kvms_battery_result;
volatile uint8_t kvms_battery_valid_debug;
volatile float kvms_battery_total_voltage_v_debug;
volatile float kvms_battery_current_a_debug;
volatile float kvms_battery_soc_percent_debug;
volatile float kvms_battery_power_w_debug;
volatile float kvms_battery_charge_power_w_debug;
volatile float kvms_battery_discharge_power_w_debug;
volatile float kvms_battery_delta_voltage_v_debug;
volatile float kvms_battery_remaining_capacity_ah_debug;
volatile float kvms_battery_used_energy_wh_debug;
volatile float kvms_battery_used_capacity_ah_debug;
volatile float kvms_battery_charged_energy_wh_debug;
volatile float kvms_battery_charged_capacity_ah_debug;
volatile uint32_t kvms_battery_last_update_tick_debug;
volatile uint32_t kvms_battery_rx_count_debug;
volatile uint32_t kvms_battery_rx_echo_count_debug;
volatile uint32_t kvms_battery_rx_stream_overflow_count_debug;
volatile uint32_t kvms_battery_parse_ok_count_debug;
volatile uint32_t kvms_battery_frame_error_count_debug;
volatile uint32_t kvms_battery_crc_error_count_debug;
volatile uint32_t kvms_battery_tx_count_debug;
volatile uint32_t kvms_battery_tx_error_count_debug;
volatile uint32_t kvms_battery_last_error_flags_debug;
volatile uint32_t kvms_battery_last_poll_status_debug;
volatile uint32_t kvms_battery_uart_isr_debug;
volatile uint32_t kvms_battery_uart_gstate_debug;
volatile uint32_t kvms_battery_uart_rxstate_debug;
volatile uint32_t kvms_battery_uart_error_code_debug;
volatile uint32_t kvms_battery_rx_dma_remaining_debug;
volatile uint32_t kvms_battery_last_rx_size_debug;
volatile uint32_t kvms_battery_last_rx_addr_debug;
volatile uint32_t kvms_battery_last_rx_function_debug;
volatile uint32_t kvms_battery_last_rx_byte_count_debug;

/* ---------------------------- Private declarations ---------------------------- */

static uint16_t KvmsBattery_Crc16(const uint8_t *data, uint16_t size);
static void KvmsBattery_BuildRequest(uint8_t frame[KVMS_BATTERY_REQUEST_LENGTH]);
static uint8_t KvmsBattery_IsRequestEcho(const uint8_t *frame, uint16_t size);
static void KvmsBattery_ProcessPending(void);
static void KvmsBattery_ProcessStream(const uint8_t *stream, uint16_t size);
static uint16_t KvmsBattery_FindResponseHeader(const uint8_t *stream, uint16_t size);
static void KvmsBattery_ProcessFrame(const uint8_t *frame, uint16_t size);
static void KvmsBattery_SetError(uint32_t error_flags);
static void KvmsBattery_DecodeRegisters(KvmsBatteryData_t *out, const uint16_t registers[KVMS_BATTERY_REGISTER_COUNT]);
static void KvmsBattery_UpdateUsage(KvmsBatteryData_t *out, uint32_t now_tick);
static void KvmsBattery_UpdateResult(const KvmsBatteryData_t *data);
static void KvmsBattery_UpdateUartDebug(void);
static uint16_t KvmsBattery_Reg(const uint16_t registers[KVMS_BATTERY_REGISTER_COUNT], uint16_t index);
static int32_t KvmsBattery_SignedOffset(uint16_t value);
static int16_t KvmsBattery_Temperature(uint16_t value, uint8_t zero_is_invalid, uint8_t *valid);
static uint8_t KvmsBattery_ToBool(uint16_t value);
static float KvmsBattery_AbsFloat(float value);
static void KvmsBattery_EnterCritical(uint32_t *primask);
static void KvmsBattery_ExitCritical(uint32_t primask);

/* -------------------------------- Public API ---------------------------------- */

void KvmsBattery_Init(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  KvmsBattery_EnterCritical(&primask);
  memset(&kvms_data, 0, sizeof(kvms_data));
  memset((void *)&kvms_battery_result, 0, sizeof(kvms_battery_result));
  kvms_huart = huart;
  kvms_rx_stream_pending = 0U;
  kvms_rx_stream_size = 0U;
  kvms_parse_size = 0U;
  kvms_rx_stream_overflow_count = 0U;
  KvmsBattery_ExitCritical(primask);

  KvmsBattery_RestartRx();
}

HAL_StatusTypeDef KvmsBattery_Poll(void)
{
  HAL_StatusTypeDef status;
  uint32_t primask;

  /* Parse bytes received by the previous request before sending a new one. */
  KvmsBattery_ProcessPending();

  if (kvms_huart == NULL)
  {
    return HAL_ERROR;
  }

  if (kvms_huart->gState != HAL_UART_STATE_READY)
  {
    status = HAL_BUSY;
  }
  else
  {
    /*
     * DMA reads the buffer after this function returns, so the request frame
     * must not live on the task stack.
     */
    KvmsBattery_BuildRequest(kvms_tx_dma_buf);
    status = HAL_UART_Transmit_DMA(kvms_huart, kvms_tx_dma_buf, sizeof(kvms_tx_dma_buf));
  }

  KvmsBattery_EnterCritical(&primask);
  kvms_battery_last_poll_status_debug = (uint32_t)status;
  if (status == HAL_OK)
  {
    kvms_data.tx_count++;
  }
  else
  {
    kvms_data.tx_error_count++;
  }
  KvmsBattery_UpdateResult(&kvms_data);
  KvmsBattery_ExitCritical(primask);

  return status;
}

uint8_t KvmsBattery_CopyData(KvmsBatteryData_t *out)
{
  uint32_t primask;

  if (out == NULL)
  {
    return 0U;
  }

  KvmsBattery_EnterCritical(&primask);
  memcpy(out, &kvms_data, sizeof(*out));
  KvmsBattery_ExitCritical(primask);

  return out->valid;
}

uint8_t KvmsBattery_CopyResult(KvmsBatteryResult_t *out)
{
  uint32_t primask;

  if (out == NULL)
  {
    return 0U;
  }

  KvmsBattery_EnterCritical(&primask);
  memcpy(out, (const void *)&kvms_battery_result, sizeof(*out));
  KvmsBattery_ExitCritical(primask);

  return out->valid;
}

const KvmsBatteryData_t *KvmsBattery_GetData(void)
{
  return &kvms_data;
}

/* Reset only boot-time integrated charge/discharge counters. */
void KvmsBattery_ResetUsage(void)
{
  uint32_t primask;

  KvmsBattery_EnterCritical(&primask);
  kvms_data.used_energy_wh = 0.0f;
  kvms_data.used_capacity_ah = 0.0f;
  kvms_data.charged_energy_wh = 0.0f;
  kvms_data.charged_capacity_ah = 0.0f;
  kvms_data.usage_last_tick = HAL_GetTick();
  KvmsBattery_UpdateResult(&kvms_data);
  KvmsBattery_ExitCritical(primask);
}

void KvmsBattery_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t copy_size;

  if ((kvms_huart == NULL) || (huart != kvms_huart))
  {
    return;
  }

  copy_size = (size > KVMS_BATTERY_RX_BUFFER_SIZE) ? KVMS_BATTERY_RX_BUFFER_SIZE : size;
  if (copy_size > 0U)
  {
    /*
     * RX idle can split one BMS response across several callbacks. Append every
     * chunk here; the task context later searches for complete frames.
     */
    if (((uint32_t)kvms_rx_stream_size + copy_size) > KVMS_BATTERY_RX_STREAM_SIZE)
    {
      kvms_rx_stream_size = 0U;
      kvms_rx_stream_overflow_count++;
      kvms_battery_rx_stream_overflow_count_debug = kvms_rx_stream_overflow_count;
    }

    memcpy(&kvms_rx_stream_buf[kvms_rx_stream_size], kvms_rx_dma_buf, copy_size);
    kvms_rx_stream_size = (uint16_t)(kvms_rx_stream_size + copy_size);
    kvms_rx_stream_pending = 1U;
    kvms_data.rx_count++;
    kvms_battery_last_rx_size_debug = copy_size;
    kvms_battery_last_rx_addr_debug = kvms_rx_dma_buf[0];
    kvms_battery_last_rx_function_debug = (copy_size > 1U) ? kvms_rx_dma_buf[1] : 0U;
    kvms_battery_last_rx_byte_count_debug = (copy_size > 2U) ? kvms_rx_dma_buf[2] : 0U;
  }

  KvmsBattery_RestartRx();
}

/* Arm USART RX-to-idle DMA for the next chunk. */
void KvmsBattery_RestartRx(void)
{
  if (kvms_huart == NULL)
  {
    return;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(kvms_huart, kvms_rx_dma_buf, sizeof(kvms_rx_dma_buf)) == HAL_OK)
  {
    if (kvms_huart->hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(kvms_huart->hdmarx, DMA_IT_HT);
    }
  }
}

/* ----------------------------- Modbus utilities ------------------------------- */

static uint16_t KvmsBattery_Crc16(const uint8_t *data, uint16_t size)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < size; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static void KvmsBattery_BuildRequest(uint8_t frame[KVMS_BATTERY_REQUEST_LENGTH])
{
  uint16_t crc;

  /* Request: 81 03 00 00 00 7F CRC_LO CRC_HI. */
  frame[0] = KVMS_BATTERY_REQUEST_ADDR;
  frame[1] = KVMS_BATTERY_FUNCTION_READ;
  frame[2] = (uint8_t)(KVMS_BATTERY_START_REGISTER >> 8U);
  frame[3] = (uint8_t)(KVMS_BATTERY_START_REGISTER & 0xFFU);
  frame[4] = (uint8_t)(KVMS_BATTERY_REGISTER_COUNT >> 8U);
  frame[5] = (uint8_t)(KVMS_BATTERY_REGISTER_COUNT & 0xFFU);

  crc = KvmsBattery_Crc16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)(crc >> 8U);
}

static uint8_t KvmsBattery_IsRequestEcho(const uint8_t *frame, uint16_t size)
{
  uint16_t crc_calc;
  uint16_t crc_frame;

  if ((frame == NULL) || (size != KVMS_BATTERY_REQUEST_LENGTH))
  {
    return 0U;
  }

  if ((frame[0] != KVMS_BATTERY_REQUEST_ADDR) ||
      (frame[1] != KVMS_BATTERY_FUNCTION_READ) ||
      (frame[2] != (uint8_t)(KVMS_BATTERY_START_REGISTER >> 8U)) ||
      (frame[3] != (uint8_t)(KVMS_BATTERY_START_REGISTER & 0xFFU)) ||
      (frame[4] != (uint8_t)(KVMS_BATTERY_REGISTER_COUNT >> 8U)) ||
      (frame[5] != (uint8_t)(KVMS_BATTERY_REGISTER_COUNT & 0xFFU)))
  {
    return 0U;
  }

  crc_calc = KvmsBattery_Crc16(frame, 6U);
  crc_frame = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8U);
  return (crc_calc == crc_frame) ? 1U : 0U;
}

/* ---------------------------- RX stream assembly ------------------------------ */

static void KvmsBattery_ProcessPending(void)
{
  uint16_t size;
  uint32_t primask;

  KvmsBattery_EnterCritical(&primask);
  if ((kvms_rx_stream_pending == 0U) || (kvms_rx_stream_size == 0U))
  {
    KvmsBattery_ExitCritical(primask);
    return;
  }

  size = kvms_rx_stream_size;
  memcpy(kvms_rx_process_buf, kvms_rx_stream_buf, size);
  kvms_rx_stream_pending = 0U;
  kvms_rx_stream_size = 0U;
  KvmsBattery_ExitCritical(primask);

  KvmsBattery_ProcessStream(kvms_rx_process_buf, size);
}

static void KvmsBattery_ProcessStream(const uint8_t *stream, uint16_t size)
{
  uint16_t pos = 0U;
  uint16_t header_pos;
  uint16_t remaining;
  uint32_t primask;

  if ((stream == NULL) || (size == 0U))
  {
    return;
  }

  if (((uint32_t)kvms_parse_size + size) > KVMS_BATTERY_RX_STREAM_SIZE)
  {
    kvms_parse_size = 0U;
    kvms_rx_stream_overflow_count++;
    kvms_battery_rx_stream_overflow_count_debug = kvms_rx_stream_overflow_count;
  }

  memcpy(&kvms_parse_buf[kvms_parse_size], stream, size);
  kvms_parse_size = (uint16_t)(kvms_parse_size + size);

  while (pos < kvms_parse_size)
  {
    if (((uint16_t)(kvms_parse_size - pos) >= KVMS_BATTERY_REQUEST_LENGTH) &&
        (KvmsBattery_IsRequestEcho(&kvms_parse_buf[pos], KVMS_BATTERY_REQUEST_LENGTH) != 0U))
    {
      KvmsBattery_EnterCritical(&primask);
      kvms_data.rx_echo_count++;
      KvmsBattery_UpdateResult(&kvms_data);
      KvmsBattery_ExitCritical(primask);
      pos = (uint16_t)(pos + KVMS_BATTERY_REQUEST_LENGTH);
      continue;
    }

    header_pos = KvmsBattery_FindResponseHeader(&kvms_parse_buf[pos], (uint16_t)(kvms_parse_size - pos));
    if (header_pos == UINT16_MAX)
    {
      /* Keep the last two bytes so a split 51 03 FE header can be found later. */
      if ((uint16_t)(kvms_parse_size - pos) > 2U)
      {
        pos = (uint16_t)(kvms_parse_size - 2U);
      }
      break;
    }

    pos = (uint16_t)(pos + header_pos);
    if ((uint16_t)(kvms_parse_size - pos) < KVMS_BATTERY_RESPONSE_LENGTH)
    {
      /* Incomplete response. Leave it in kvms_parse_buf for the next chunk. */
      break;
    }

    KvmsBattery_ProcessFrame(&kvms_parse_buf[pos], KVMS_BATTERY_RESPONSE_LENGTH);
    pos = (uint16_t)(pos + KVMS_BATTERY_RESPONSE_LENGTH);
  }

  if (pos > 0U)
  {
    remaining = (uint16_t)(kvms_parse_size - pos);
    if (remaining > 0U)
    {
      memmove(kvms_parse_buf, &kvms_parse_buf[pos], remaining);
    }
    kvms_parse_size = remaining;
  }
}

static uint16_t KvmsBattery_FindResponseHeader(const uint8_t *stream, uint16_t size)
{
  uint16_t i;

  if ((stream == NULL) || (size < 3U))
  {
    return UINT16_MAX;
  }

  for (i = 0U; i <= (uint16_t)(size - 3U); i++)
  {
    if ((stream[i] == KVMS_BATTERY_RESPONSE_ADDR) &&
        (stream[i + 1U] == KVMS_BATTERY_FUNCTION_READ) &&
        (stream[i + 2U] == KVMS_BATTERY_RESPONSE_DATA_BYTES))
    {
      return i;
    }
  }

  return UINT16_MAX;
}

/* ---------------------------- Frame validation -------------------------------- */

static void KvmsBattery_ProcessFrame(const uint8_t *frame, uint16_t size)
{
  uint16_t crc_calc;
  uint16_t crc_frame;
  uint16_t registers[KVMS_BATTERY_REGISTER_COUNT];
  uint16_t i;
  KvmsBatteryData_t decoded;
  uint32_t now_tick;
  uint32_t primask;

  if ((frame == NULL) || (size < 5U))
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_SHORT_FRAME);
    return;
  }

  if (KvmsBattery_IsRequestEcho(frame, size) != 0U)
  {
    KvmsBattery_EnterCritical(&primask);
    kvms_data.rx_echo_count++;
    KvmsBattery_UpdateResult(&kvms_data);
    KvmsBattery_ExitCritical(primask);
    return;
  }

  if (frame[0] != KVMS_BATTERY_RESPONSE_ADDR)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_ADDRESS);
    return;
  }

  if ((frame[1] & 0x80U) != 0U)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_EXCEPTION);
    return;
  }

  if (frame[1] != KVMS_BATTERY_FUNCTION_READ)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_FUNCTION);
    return;
  }

  if (size != KVMS_BATTERY_RESPONSE_LENGTH)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_LENGTH);
    return;
  }

  if (frame[2] != KVMS_BATTERY_RESPONSE_DATA_BYTES)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_BYTE_COUNT);
    return;
  }

  crc_calc = KvmsBattery_Crc16(frame, (uint16_t)(size - 2U));
  crc_frame = (uint16_t)frame[size - 2U] | ((uint16_t)frame[size - 1U] << 8U);
  if (crc_calc != crc_frame)
  {
    KvmsBattery_SetError(KVMS_BATTERY_ERROR_CRC);
    return;
  }

  for (i = 0U; i < KVMS_BATTERY_REGISTER_COUNT; i++)
  {
    /* BMS registers are big-endian 16-bit values inside the Modbus payload. */
    uint16_t data_index = (uint16_t)(3U + (i * 2U));
    registers[i] = ((uint16_t)frame[data_index] << 8U) | frame[data_index + 1U];
  }

  KvmsBattery_EnterCritical(&primask);
  memcpy(&decoded, &kvms_data, sizeof(decoded));
  KvmsBattery_ExitCritical(primask);

  KvmsBattery_DecodeRegisters(&decoded, registers);
  now_tick = HAL_GetTick();
  KvmsBattery_UpdateUsage(&decoded, now_tick);
  decoded.valid = 1U;
  decoded.response_addr = frame[0];
  decoded.response_function = frame[1];
  decoded.last_update_tick = now_tick;
  decoded.last_error_flags = KVMS_BATTERY_ERROR_NONE;
  decoded.parse_ok_count++;

  KvmsBattery_EnterCritical(&primask);
  memcpy(&kvms_data, &decoded, sizeof(kvms_data));
  KvmsBattery_UpdateResult(&kvms_data);
  KvmsBattery_ExitCritical(primask);
}

static void KvmsBattery_SetError(uint32_t error_flags)
{
  uint32_t primask;

  KvmsBattery_EnterCritical(&primask);
  kvms_data.last_error_flags = error_flags;
  if ((error_flags & KVMS_BATTERY_ERROR_CRC) != 0UL)
  {
    kvms_data.crc_error_count++;
  }
  else
  {
    kvms_data.frame_error_count++;
  }
  KvmsBattery_UpdateResult(&kvms_data);
  KvmsBattery_ExitCritical(primask);
}

/* ---------------------------- Register decoding ------------------------------- */

static void KvmsBattery_DecodeRegisters(KvmsBatteryData_t *out, const uint16_t registers[KVMS_BATTERY_REGISTER_COUNT])
{
  uint8_t i;
  uint8_t temp_valid;
  float instant_power;
  int32_t current_raw;

  memcpy(out->raw_registers, registers, sizeof(out->raw_registers));

  /* 0..47: cell voltages, mV. */
  for (i = 0U; i < KVMS_BATTERY_CELL_MAX; i++)
  {
    out->cell_voltage_mv[i] = KvmsBattery_Reg(registers, i);
  }

  /* 48..55: temperature probes, raw value is temperature + 40. */
  for (i = 0U; i < KVMS_BATTERY_TEMP_MAX; i++)
  {
    out->temperatures_c[i] = KvmsBattery_Temperature(KvmsBattery_Reg(registers, (uint16_t)(48U + i)), 0U, &temp_valid);
    out->temp_valid[i] = temp_valid;
  }

  /* 56..61: pack summary. Current is encoded as raw - 30000, in 0.1 A. */
  out->pack_voltage_v = (float)KvmsBattery_Reg(registers, 56U) * 0.1f;
  current_raw = KvmsBattery_SignedOffset(KvmsBattery_Reg(registers, 57U));
  out->current_a = (float)current_raw * 0.1f;
  out->soc_percent = (float)KvmsBattery_Reg(registers, 58U) * 0.1f;
  out->life = KvmsBattery_Reg(registers, 59U);
  out->cell_count = (uint8_t)KvmsBattery_Reg(registers, 60U);
  out->temp_count = (uint8_t)KvmsBattery_Reg(registers, 61U);

  /* 62..71: cell and temperature min/max/delta values. */
  out->max_cell_voltage_mv = KvmsBattery_Reg(registers, 62U);
  out->max_cell_voltage_v = (float)out->max_cell_voltage_mv * 0.001f;
  out->max_cell_number = (uint8_t)KvmsBattery_Reg(registers, 63U);
  out->min_cell_voltage_mv = KvmsBattery_Reg(registers, 64U);
  out->min_cell_voltage_v = (float)out->min_cell_voltage_mv * 0.001f;
  out->min_cell_number = (uint8_t)KvmsBattery_Reg(registers, 65U);
  out->delta_cell_voltage_mv = KvmsBattery_Reg(registers, 66U);
  out->delta_cell_voltage_v = (float)out->delta_cell_voltage_mv * 0.001f;

  out->max_temp_c = KvmsBattery_Temperature(KvmsBattery_Reg(registers, 67U), 0U, &temp_valid);
  out->max_temp_number = (uint8_t)KvmsBattery_Reg(registers, 68U);
  out->min_temp_c = KvmsBattery_Temperature(KvmsBattery_Reg(registers, 69U), 0U, &temp_valid);
  out->min_temp_number = (uint8_t)KvmsBattery_Reg(registers, 70U);
  out->delta_temp_c = (int16_t)KvmsBattery_Reg(registers, 71U);

  /* 72..76: charge state, connection state, remaining capacity and cycles. */
  out->charge_state_code = (uint8_t)KvmsBattery_Reg(registers, 72U);
  if (out->charge_state_code == KVMS_BATTERY_CHARGE_STATE_IDLE)
  {
    out->charge_state = KVMS_BATTERY_CHARGE_STATE_IDLE;
  }
  else if (out->charge_state_code == KVMS_BATTERY_CHARGE_STATE_CHARGE)
  {
    out->charge_state = KVMS_BATTERY_CHARGE_STATE_CHARGE;
  }
  else if (out->charge_state_code == KVMS_BATTERY_CHARGE_STATE_DISCHARGE)
  {
    out->charge_state = KVMS_BATTERY_CHARGE_STATE_DISCHARGE;
  }
  else
  {
    out->charge_state = KVMS_BATTERY_CHARGE_STATE_UNKNOWN;
  }

  out->charger_connected = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 73U));
  out->load_connected = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 74U));
  out->remain_capacity_ah = (float)KvmsBattery_Reg(registers, 75U) * 0.1f;
  out->cycles = KvmsBattery_Reg(registers, 76U);

  /* 77..81: balance mode and per-cell balance bitmap. */
  if (KvmsBattery_Reg(registers, 77U) <= KVMS_BATTERY_BALANCE_ACTIVE)
  {
    out->balance_mode = (KvmsBatteryBalanceMode_t)KvmsBattery_Reg(registers, 77U);
  }
  else
  {
    out->balance_mode = KVMS_BATTERY_BALANCE_UNKNOWN;
  }

  for (i = 0U; i < KVMS_BATTERY_BALANCE_REG_COUNT; i++)
  {
    out->balance_raw[i] = KvmsBattery_Reg(registers, (uint16_t)(79U + i));
  }

  for (i = 0U; i < KVMS_BATTERY_CELL_MAX; i++)
  {
    uint16_t balance_word = out->balance_raw[i / 16U];
    out->balances[i] = ((balance_word & (uint16_t)(1U << (i % 16U))) != 0U) ? 1U : 0U;
  }

  /* 82..89: MOS states and power/energy values. */
  out->charge_mos = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 82U));
  out->discharge_mos = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 83U));
  out->precharge_mos = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 84U));
  out->heating_mos = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 85U));
  out->fan_mos = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 86U));
  out->avg_voltage_v = (float)KvmsBattery_Reg(registers, 87U) * 0.001f;
  out->power_w = (float)KvmsBattery_Reg(registers, 88U);

  instant_power = out->pack_voltage_v * out->current_a;
  if (instant_power < 0.0f)
  {
    instant_power = -instant_power;
  }
  out->charge_power_w = ((out->charge_state == KVMS_BATTERY_CHARGE_STATE_CHARGE) || (out->current_a < -0.05f)) ? instant_power : 0.0f;
  out->discharge_power_w = ((out->charge_state == KVMS_BATTERY_CHARGE_STATE_DISCHARGE) || (out->current_a > 0.05f)) ? instant_power : 0.0f;

  out->energy_wh = KvmsBattery_Reg(registers, 89U);

  /* 90..107: extra temperatures, current limit, RTC and wake source fields. */
  out->mos_temp_c = KvmsBattery_Temperature(KvmsBattery_Reg(registers, 90U), 0U, &temp_valid);
  out->ambient_temp_c = KvmsBattery_Temperature(KvmsBattery_Reg(registers, 91U), 0U, &temp_valid);
  out->heating_temp_c = KvmsBattery_Temperature(KvmsBattery_Reg(registers, 92U), 1U, &temp_valid);
  out->heating_current = (int16_t)KvmsBattery_Reg(registers, 93U);
  out->current_limit_enabled = KvmsBattery_ToBool(KvmsBattery_Reg(registers, 95U));
  out->current_limit_current_a = (float)KvmsBattery_SignedOffset(KvmsBattery_Reg(registers, 96U)) * 0.1f;

  out->rtc_year = (uint8_t)(KvmsBattery_Reg(registers, 97U) >> 8U);
  out->rtc_month = (uint8_t)(KvmsBattery_Reg(registers, 97U) & 0xFFU);
  out->rtc_day = (uint8_t)(KvmsBattery_Reg(registers, 98U) >> 8U);
  out->rtc_hour = (uint8_t)(KvmsBattery_Reg(registers, 98U) & 0xFFU);
  out->rtc_minute = (uint8_t)(KvmsBattery_Reg(registers, 99U) >> 8U);
  out->rtc_second = (uint8_t)(KvmsBattery_Reg(registers, 99U) & 0xFFU);
  out->rtc_valid = ((out->rtc_month >= 1U) && (out->rtc_month <= 12U) &&
                    (out->rtc_day >= 1U) && (out->rtc_day <= 31U) &&
                    (out->rtc_hour <= 23U) &&
                    (out->rtc_minute <= 59U) &&
                    (out->rtc_second <= 59U)) ? 1U : 0U;

  out->remaining_charge_minutes = KvmsBattery_Reg(registers, 100U);
  out->wake_sources_mask = (uint8_t)KvmsBattery_Reg(registers, 107U);

  /* 109..115: fault words. Leave raw so higher layers can decode per manual. */
  for (i = 0U; i < KVMS_BATTERY_FAULT_REG_COUNT; i++)
  {
    out->fault_raw[i] = KvmsBattery_Reg(registers, (uint16_t)(109U + i));
  }
}

/* Integrate energy/capacity since boot or KvmsBattery_ResetUsage(). */
static void KvmsBattery_UpdateUsage(KvmsBatteryData_t *out, uint32_t now_tick)
{
  uint32_t elapsed_ms;
  float elapsed_hours;
  float current_abs_a;
  float energy_wh;
  float capacity_ah;

  if (out == NULL)
  {
    return;
  }

  /*
   * The dashboard usage values are integration results, not direct BMS registers:
   * current > 0 discharges the battery, current < 0 charges the battery.
   */
  if ((out->valid != 0U) && (out->usage_last_tick != 0U))
  {
    elapsed_ms = now_tick - out->usage_last_tick;
    if ((elapsed_ms > 0U) && (elapsed_ms <= KVMS_BATTERY_USAGE_MAX_INTERVAL_MS))
    {
      if ((out->current_a > KVMS_BATTERY_CURRENT_DEADBAND_A) ||
          (out->current_a < -KVMS_BATTERY_CURRENT_DEADBAND_A))
      {
        elapsed_hours = (float)elapsed_ms / 3600000.0f;
        current_abs_a = KvmsBattery_AbsFloat(out->current_a);
        capacity_ah = current_abs_a * elapsed_hours;
        energy_wh = out->pack_voltage_v * current_abs_a * elapsed_hours;

        if (out->current_a > 0.0f)
        {
          out->used_capacity_ah += capacity_ah;
          out->used_energy_wh += energy_wh;
        }
        else
        {
          out->charged_capacity_ah += capacity_ah;
          out->charged_energy_wh += energy_wh;
        }
      }
    }
  }

  out->usage_last_tick = now_tick;
}

/* Publish a compact watch-friendly result from the full decoded data block. */
static void KvmsBattery_UpdateResult(const KvmsBatteryData_t *data)
{
  if (data == NULL)
  {
    return;
  }

  kvms_battery_result.valid = data->valid;
  kvms_battery_result.pack_voltage_v = data->pack_voltage_v;
  kvms_battery_result.current_a = data->current_a;
  kvms_battery_result.soc_percent = data->soc_percent;
  kvms_battery_result.power_w = data->power_w;
  kvms_battery_result.charge_power_w = data->charge_power_w;
  kvms_battery_result.discharge_power_w = data->discharge_power_w;
  kvms_battery_result.delta_cell_voltage_v = data->delta_cell_voltage_v;
  kvms_battery_result.remaining_capacity_ah = data->remain_capacity_ah;
  kvms_battery_result.used_energy_wh = data->used_energy_wh;
  kvms_battery_result.used_capacity_ah = data->used_capacity_ah;
  kvms_battery_result.charged_energy_wh = data->charged_energy_wh;
  kvms_battery_result.charged_capacity_ah = data->charged_capacity_ah;
  kvms_battery_result.last_update_tick = data->last_update_tick;
  kvms_battery_result.rx_count = data->rx_count;
  kvms_battery_result.rx_echo_count = data->rx_echo_count;
  kvms_battery_result.parse_ok_count = data->parse_ok_count;
  kvms_battery_result.frame_error_count = data->frame_error_count;
  kvms_battery_result.crc_error_count = data->crc_error_count;
  kvms_battery_result.tx_count = data->tx_count;
  kvms_battery_result.tx_error_count = data->tx_error_count;
  kvms_battery_result.last_error_flags = data->last_error_flags;

  /*
   * These scalar mirrors are intentionally verbose so DAP/VSCode Live Watch can
   * display battery data without expanding the full result structure.
   */
  kvms_battery_valid_debug = data->valid;
  kvms_battery_total_voltage_v_debug = data->pack_voltage_v;
  kvms_battery_current_a_debug = data->current_a;
  kvms_battery_soc_percent_debug = data->soc_percent;
  kvms_battery_power_w_debug = data->power_w;
  kvms_battery_charge_power_w_debug = data->charge_power_w;
  kvms_battery_discharge_power_w_debug = data->discharge_power_w;
  kvms_battery_delta_voltage_v_debug = data->delta_cell_voltage_v;
  kvms_battery_remaining_capacity_ah_debug = data->remain_capacity_ah;
  kvms_battery_used_energy_wh_debug = data->used_energy_wh;
  kvms_battery_used_capacity_ah_debug = data->used_capacity_ah;
  kvms_battery_charged_energy_wh_debug = data->charged_energy_wh;
  kvms_battery_charged_capacity_ah_debug = data->charged_capacity_ah;
  kvms_battery_last_update_tick_debug = data->last_update_tick;
  kvms_battery_rx_count_debug = data->rx_count;
  kvms_battery_rx_echo_count_debug = data->rx_echo_count;
  kvms_battery_rx_stream_overflow_count_debug = kvms_rx_stream_overflow_count;
  kvms_battery_parse_ok_count_debug = data->parse_ok_count;
  kvms_battery_frame_error_count_debug = data->frame_error_count;
  kvms_battery_crc_error_count_debug = data->crc_error_count;
  kvms_battery_tx_count_debug = data->tx_count;
  kvms_battery_tx_error_count_debug = data->tx_error_count;
  kvms_battery_last_error_flags_debug = data->last_error_flags;
  KvmsBattery_UpdateUartDebug();
}

/* Snapshot useful USART/DMA status without halting the MCU. */
static void KvmsBattery_UpdateUartDebug(void)
{
  if (kvms_huart == NULL)
  {
    kvms_battery_uart_isr_debug = 0U;
    kvms_battery_uart_gstate_debug = 0U;
    kvms_battery_uart_rxstate_debug = 0U;
    kvms_battery_uart_error_code_debug = 0U;
    kvms_battery_rx_dma_remaining_debug = 0U;
    return;
  }

  kvms_battery_uart_isr_debug = kvms_huart->Instance->ISR;
  kvms_battery_uart_gstate_debug = (uint32_t)kvms_huart->gState;
  kvms_battery_uart_rxstate_debug = (uint32_t)kvms_huart->RxState;
  kvms_battery_uart_error_code_debug = kvms_huart->ErrorCode;
  if (kvms_huart->hdmarx != NULL)
  {
    kvms_battery_rx_dma_remaining_debug = ((DMA_Stream_TypeDef *)kvms_huart->hdmarx->Instance)->NDTR;
  }
  else
  {
    kvms_battery_rx_dma_remaining_debug = 0U;
  }
}

/* ------------------------------ Small helpers --------------------------------- */

static uint16_t KvmsBattery_Reg(const uint16_t registers[KVMS_BATTERY_REGISTER_COUNT], uint16_t index)
{
  if (index >= KVMS_BATTERY_REGISTER_COUNT)
  {
    return 0U;
  }

  return registers[index];
}

static int32_t KvmsBattery_SignedOffset(uint16_t value)
{
  return (int32_t)value - KVMS_BATTERY_CURRENT_OFFSET;
}

static int16_t KvmsBattery_Temperature(uint16_t value, uint8_t zero_is_invalid, uint8_t *valid)
{
  int16_t raw;

  if ((value == 0x00FFU) || (value == 0xFFFFU) || ((zero_is_invalid != 0U) && (value == 0U)))
  {
    if (valid != NULL)
    {
      *valid = 0U;
    }
    return 0;
  }

  raw = (value > 140U) ? (int16_t)((int32_t)value - 256) : (int16_t)value;
  if (valid != NULL)
  {
    *valid = 1U;
  }

  return (int16_t)(raw - 40);
}

static uint8_t KvmsBattery_ToBool(uint16_t value)
{
  return (value == 0U) ? 0U : 1U;
}

static float KvmsBattery_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void KvmsBattery_EnterCritical(uint32_t *primask)
{
  if (primask != NULL)
  {
    *primask = __get_PRIMASK();
  }
  __disable_irq();
}

static void KvmsBattery_ExitCritical(uint32_t primask)
{
  if ((primask & 0x1UL) == 0UL)
  {
    __enable_irq();
  }
}
