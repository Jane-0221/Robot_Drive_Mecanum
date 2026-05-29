#include "vl53l0x.h"

#include "cmsis_os2.h"
#include "i2c.h"

#define VL53L0X_I2C_TIMEOUT_MS 100U
#define VL53L0X_I2C_SCAN_TIMEOUT_MS 2U
#define VL53L0X_READY_POLL_TIMEOUT_MS 100U
#define VL53L0X_READY_POLL_DELAY_MS 1U
#define VL53L0X_REG_START      0x00U
#define VL53L0X_REG_RESULT     0x14U
#define VL53L0X_START_CMD      0x01U
#define VL53L0X_RESULT_READY_BIT 0x01U
#define VL53L0X_DISTANCE_OFFSET_MM 100U

volatile uint16_t vl53l0x_distance_mm = 0U;
volatile uint8_t vl53l0x_raw_data[VL53L0X_RESULT_LEN] = {0U};
volatile VL53L0X_StatusTypeDef vl53l0x_status = VL53L0X_ERROR;
volatile VL53L0X_StepTypeDef vl53l0x_error_step = VL53L0X_STEP_NONE;
volatile uint8_t vl53l0x_last_hal_status = HAL_OK;
volatile uint32_t vl53l0x_last_hal_error = 0U;
volatile uint8_t vl53l0x_scan_count = 0U;
volatile uint8_t vl53l0x_scan_addr[VL53L0X_SCAN_MAX] = {0U};
volatile uint8_t vl53l0x_scl_level = 0U;
volatile uint8_t vl53l0x_sda_level = 0U;
volatile uint8_t vl53l0x_i2c_state = 0U;
volatile uint8_t vl53l0x_i2c_busy_flag = 0U;
volatile uint32_t vl53l0x_i2c_isr = 0U;
volatile uint32_t vl53l0x_i2c_cr2 = 0U;
volatile uint8_t vl53l0x_ready_status = 0U;
volatile uint8_t vl53l0x_ready_poll_count = 0U;
volatile uint8_t vl53l0x_range_status = 0U;
volatile uint8_t vl53l0x_probe_52_status = HAL_ERROR;
volatile uint8_t vl53l0x_probe_29_status = HAL_ERROR;
volatile uint8_t vl53l0x_probe_52_shift_status = HAL_ERROR;
volatile uint8_t vl53l0x_soft_probe_52_status = HAL_ERROR;
volatile uint8_t vl53l0x_soft_probe_a4_status = HAL_ERROR;
volatile uint8_t vl53l0x_recover_count = 0U;

static VL53L0X_StatusTypeDef VL53L0X_StatusFromHal(HAL_StatusTypeDef hal_status)
{
    if (hal_status == HAL_TIMEOUT)
    {
        return VL53L0X_TIMEOUT;
    }

    return VL53L0X_ERROR;
}

static void VL53L0X_DelayMs(uint32_t delay_ms)
{
    if (delay_ms == 0U)
    {
        return;
    }

    if (osKernelGetState() == osKernelRunning)
    {
        uint32_t delay_ticks = (osKernelGetTickFreq() * delay_ms + 999U) / 1000U;

        if (delay_ticks == 0U)
        {
            delay_ticks = 1U;
        }

        (void)osDelay(delay_ticks);
    }
    else
    {
        HAL_Delay(delay_ms);
    }
}

static HAL_StatusTypeDef VL53L0X_WaitResultReady(void)
{
    HAL_StatusTypeDef hal_status = HAL_TIMEOUT;
    uint8_t result_reg;
    uint8_t status = 0U;

    vl53l0x_ready_status = 0U;
    vl53l0x_ready_poll_count = 0U;

    for (uint8_t count = 0U; count < VL53L0X_READY_POLL_TIMEOUT_MS; count++)
    {
        result_reg = VL53L0X_REG_RESULT;
        hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                             VL53L0X_I2C_HAL_ADDR,
                                             &result_reg,
                                             1U,
                                             VL53L0X_I2C_TIMEOUT_MS);
        vl53l0x_last_hal_status = hal_status;
        if (hal_status != HAL_OK)
        {
            return hal_status;
        }

        hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                            VL53L0X_I2C_HAL_ADDR,
                                            &status,
                                            1U,
                                            VL53L0X_I2C_TIMEOUT_MS);
        vl53l0x_last_hal_status = hal_status;
        if (hal_status != HAL_OK)
        {
            return hal_status;
        }

        vl53l0x_ready_status = status;
        vl53l0x_ready_poll_count = (uint8_t)(count + 1U);

        if ((status & VL53L0X_RESULT_READY_BIT) != 0U)
        {
            return HAL_OK;
        }

        VL53L0X_DelayMs(VL53L0X_READY_POLL_DELAY_MS);
    }

    return HAL_TIMEOUT;
}

static void VL53L0X_BusDelay(void)
{
    for (volatile uint32_t i = 0U; i < 2000U; i++)
    {
    }
}

static void VL53L0X_SoftSetScl(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, state);
    VL53L0X_BusDelay();
}

static void VL53L0X_SoftSetSda(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, state);
    VL53L0X_BusDelay();
}

static GPIO_PinState VL53L0X_SoftReadSda(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
}

static void VL53L0X_SoftStart(void)
{
    VL53L0X_SoftSetSda(GPIO_PIN_SET);
    VL53L0X_SoftSetScl(GPIO_PIN_SET);
    VL53L0X_SoftSetSda(GPIO_PIN_RESET);
    VL53L0X_SoftSetScl(GPIO_PIN_RESET);
}

static void VL53L0X_SoftStop(void)
{
    VL53L0X_SoftSetSda(GPIO_PIN_RESET);
    VL53L0X_SoftSetScl(GPIO_PIN_SET);
    VL53L0X_SoftSetSda(GPIO_PIN_SET);
}

static uint8_t VL53L0X_SoftWriteByte(uint8_t data)
{
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        if ((data & 0x80U) != 0U)
        {
            VL53L0X_SoftSetSda(GPIO_PIN_SET);
        }
        else
        {
            VL53L0X_SoftSetSda(GPIO_PIN_RESET);
        }

        VL53L0X_SoftSetScl(GPIO_PIN_SET);
        VL53L0X_SoftSetScl(GPIO_PIN_RESET);
        data <<= 1;
    }

    VL53L0X_SoftSetSda(GPIO_PIN_SET);
    VL53L0X_SoftSetScl(GPIO_PIN_SET);
    uint8_t ack = (VL53L0X_SoftReadSda() == GPIO_PIN_RESET) ? 1U : 0U;
    VL53L0X_SoftSetScl(GPIO_PIN_RESET);

    return ack;
}

static uint8_t VL53L0X_SoftProbeAddressByte(uint8_t address_byte)
{
    uint8_t ack;

    VL53L0X_SoftStart();
    ack = VL53L0X_SoftWriteByte(address_byte);
    VL53L0X_SoftStop();

    return (ack != 0U) ? HAL_OK : HAL_ERROR;
}

static void VL53L0X_CaptureBusDebug(void)
{
    vl53l0x_scl_level = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    vl53l0x_sda_level = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    vl53l0x_i2c_state = (uint8_t)HAL_I2C_GetState(&hi2c1);
    vl53l0x_i2c_isr = hi2c1.Instance->ISR;
    vl53l0x_i2c_cr2 = hi2c1.Instance->CR2;
    vl53l0x_i2c_busy_flag = (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET) ? 1U : 0U;
}

void VL53L0X_UpdateBusDebug(void)
{
    VL53L0X_CaptureBusDebug();

    if ((vl53l0x_i2c_busy_flag != 0U) || (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY))
    {
        vl53l0x_probe_52_status = HAL_BUSY;
        vl53l0x_probe_29_status = HAL_BUSY;
        vl53l0x_probe_52_shift_status = HAL_BUSY;
        return;
    }

    vl53l0x_probe_52_status = (uint8_t)HAL_I2C_IsDeviceReady(&hi2c1, VL53L0X_I2C_HAL_ADDR, 1U, VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_probe_29_status = (uint8_t)HAL_I2C_IsDeviceReady(&hi2c1, (0x29U << 1), 1U, VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_probe_52_shift_status = (uint8_t)HAL_I2C_IsDeviceReady(&hi2c1, VL53L0X_I2C_SHIFT_ADDR, 1U, VL53L0X_I2C_TIMEOUT_MS);
    VL53L0X_CaptureBusDebug();
}

void VL53L0X_RecoverI2C1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    VL53L0X_BusDelay();

    for (uint8_t i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        VL53L0X_BusDelay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        VL53L0X_BusDelay();

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        {
            break;
        }
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    VL53L0X_BusDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    VL53L0X_BusDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    VL53L0X_BusDelay();

    MX_I2C1_Init();
    vl53l0x_recover_count++;
    VL53L0X_UpdateBusDebug();
}

uint8_t VL53L0X_SoftProbeI2C1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    VL53L0X_SoftSetSda(GPIO_PIN_SET);
    VL53L0X_SoftSetScl(GPIO_PIN_SET);
    vl53l0x_scl_level = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    vl53l0x_sda_level = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if ((vl53l0x_scl_level == 0U) || (vl53l0x_sda_level == 0U))
    {
        vl53l0x_soft_probe_52_status = HAL_BUSY;
        vl53l0x_soft_probe_a4_status = HAL_BUSY;
        MX_I2C1_Init();
        VL53L0X_CaptureBusDebug();
        return vl53l0x_soft_probe_52_status;
    }

    vl53l0x_soft_probe_52_status = VL53L0X_SoftProbeAddressByte((uint8_t)VL53L0X_I2C_HAL_ADDR);
    vl53l0x_soft_probe_a4_status = VL53L0X_SoftProbeAddressByte((uint8_t)VL53L0X_I2C_SHIFT_ADDR);

    MX_I2C1_Init();
    VL53L0X_CaptureBusDebug();

    return vl53l0x_soft_probe_52_status;
}

uint8_t VL53L0X_ScanI2C1(void)
{
    uint8_t found_count = 0U;

    for (uint8_t i = 0U; i < VL53L0X_SCAN_MAX; i++)
    {
        vl53l0x_scan_addr[i] = 0U;
    }

    VL53L0X_CaptureBusDebug();
    if ((vl53l0x_i2c_busy_flag != 0U) || (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY))
    {
        vl53l0x_scan_count = 0U;
        return 0U;
    }

    for (uint8_t dev_addr = 1U; dev_addr < 0x7FU; dev_addr++)
    {
        HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(&hi2c1,
                                                             ((uint16_t)dev_addr << 1),
                                                             1U,
                                                             VL53L0X_I2C_SCAN_TIMEOUT_MS);

        if (hal_status == HAL_OK)
        {
            if (found_count < VL53L0X_SCAN_MAX)
            {
                vl53l0x_scan_addr[found_count] = dev_addr;
            }

            found_count++;
        }
    }

    vl53l0x_scan_count = found_count;
    return found_count;
}

VL53L0X_StatusTypeDef VL53L0X_Init(void)
{
    HAL_StatusTypeDef hal_status;

    VL53L0X_UpdateBusDebug();
    if (vl53l0x_i2c_busy_flag != 0U)
    {
        VL53L0X_RecoverI2C1();
    }

    (void)VL53L0X_ScanI2C1();

    vl53l0x_last_hal_error = 0U;
    vl53l0x_error_step = VL53L0X_STEP_DEVICE_READY;
    hal_status = HAL_I2C_IsDeviceReady(&hi2c1, VL53L0X_I2C_HAL_ADDR, 2U, VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_last_hal_status = hal_status;
    if (hal_status != HAL_OK)
    {
        vl53l0x_last_hal_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_status = (hal_status == HAL_TIMEOUT) ? VL53L0X_TIMEOUT : VL53L0X_NOT_FOUND;
        VL53L0X_UpdateBusDebug();
        if (vl53l0x_i2c_busy_flag != 0U)
        {
            VL53L0X_RecoverI2C1();
        }
        return vl53l0x_status;
    }

    VL53L0X_UpdateBusDebug();

    vl53l0x_error_step = VL53L0X_STEP_NONE;
    vl53l0x_status = VL53L0X_OK;
    return vl53l0x_status;
}

VL53L0X_StatusTypeDef VL53L0X_ReadDistance(void)
{
    HAL_StatusTypeDef hal_status;
    uint8_t start_buf[2] = {VL53L0X_REG_START, VL53L0X_START_CMD};
    uint8_t result_reg = VL53L0X_REG_RESULT;
    uint8_t read_buf[VL53L0X_RESULT_LEN];
    uint16_t distance;

    vl53l0x_error_step = VL53L0X_STEP_START_WRITE;
    hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                         VL53L0X_I2C_HAL_ADDR,
                                         start_buf,
                                         sizeof(start_buf),
                                         VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_last_hal_status = hal_status;
    if (hal_status != HAL_OK)
    {
        vl53l0x_last_hal_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_status = VL53L0X_StatusFromHal(hal_status);
        VL53L0X_UpdateBusDebug();
        if (vl53l0x_i2c_busy_flag != 0U)
        {
            VL53L0X_RecoverI2C1();
        }
        return vl53l0x_status;
    }

    vl53l0x_error_step = VL53L0X_STEP_READY_POLL;
    hal_status = VL53L0X_WaitResultReady();
    vl53l0x_last_hal_status = hal_status;
    if (hal_status != HAL_OK)
    {
        vl53l0x_last_hal_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_status = VL53L0X_StatusFromHal(hal_status);
        VL53L0X_UpdateBusDebug();
        if (vl53l0x_i2c_busy_flag != 0U)
        {
            VL53L0X_RecoverI2C1();
        }
        return vl53l0x_status;
    }

    vl53l0x_error_step = VL53L0X_STEP_RESULT_REG_WRITE;
    hal_status = HAL_I2C_Master_Transmit(&hi2c1,
                                         VL53L0X_I2C_HAL_ADDR,
                                         &result_reg,
                                         1U,
                                         VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_last_hal_status = hal_status;
    if (hal_status != HAL_OK)
    {
        vl53l0x_last_hal_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_status = VL53L0X_StatusFromHal(hal_status);
        VL53L0X_UpdateBusDebug();
        if (vl53l0x_i2c_busy_flag != 0U)
        {
            VL53L0X_RecoverI2C1();
        }
        return vl53l0x_status;
    }

    vl53l0x_error_step = VL53L0X_STEP_RESULT_READ;
    hal_status = HAL_I2C_Master_Receive(&hi2c1,
                                        VL53L0X_I2C_HAL_ADDR,
                                        read_buf,
                                        VL53L0X_RESULT_LEN,
                                        VL53L0X_I2C_TIMEOUT_MS);
    vl53l0x_last_hal_status = hal_status;
    if (hal_status != HAL_OK)
    {
        vl53l0x_last_hal_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_status = VL53L0X_StatusFromHal(hal_status);
        VL53L0X_UpdateBusDebug();
        if (vl53l0x_i2c_busy_flag != 0U)
        {
            VL53L0X_RecoverI2C1();
        }
        return vl53l0x_status;
    }

    for (uint8_t i = 0U; i < VL53L0X_RESULT_LEN; i++)
    {
        vl53l0x_raw_data[i] = read_buf[i];
    }

    distance = ((uint16_t)read_buf[10] << 8) | read_buf[11];
    vl53l0x_range_status = (uint8_t)((read_buf[0] & 0x78U) >> 3);
    vl53l0x_distance_mm = (distance > VL53L0X_DISTANCE_OFFSET_MM) ?
                           (uint16_t)(distance - VL53L0X_DISTANCE_OFFSET_MM) :
                           0U;
    vl53l0x_last_hal_error = 0U;
    vl53l0x_error_step = VL53L0X_STEP_NONE;
    vl53l0x_status = VL53L0X_OK;

    return VL53L0X_OK;
}
