#include "bsp_pca9685.h"
#include <math.h>

// 包含HAL I2C头文件，并声明外部I2C句柄（由CubeMX生成）
#include "i2c.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;   // 使用I2C2，与您的初始化对应

volatile uint8_t pca9685_ready_debug = 0U;
volatile uint8_t pca9685_i2c_bus_debug = 0U;
volatile uint8_t pca9685_addr_7bit_debug = 0U;
volatile uint32_t pca9685_last_hal_status_debug = 0U;
volatile uint32_t pca9685_write_error_count_debug = 0U;
volatile uint32_t pca9685_read_error_count_debug = 0U;

static I2C_HandleTypeDef *pca9685_i2c_handle = &hi2c2;
static uint16_t pca9685_dev_addr = PCA9685_ADDR;

// 延时函数（需外部实现）
extern void delay_us(uint32_t us);
extern void HAL_Delay(uint32_t ms);

static uint8_t PCA9685_TrySelectDeviceOnBus(I2C_HandleTypeDef *hi2c, uint8_t bus_id)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(hi2c, PCA9685_ADDR, 2, 20);
    pca9685_last_hal_status_debug = (uint32_t)status;
    if (status == HAL_OK)
    {
        pca9685_i2c_handle = hi2c;
        pca9685_dev_addr = PCA9685_ADDR;
        pca9685_i2c_bus_debug = bus_id;
        pca9685_addr_7bit_debug = (uint8_t)(PCA9685_ADDR >> 1);
        pca9685_ready_debug = 1U;
        return 1U;
    }

    for (uint8_t addr_7bit = 0x40U; addr_7bit <= 0x7FU; addr_7bit++)
    {
        uint16_t dev_addr = (uint16_t)(addr_7bit << 1);

        status = HAL_I2C_IsDeviceReady(hi2c, dev_addr, 1, 10);
        pca9685_last_hal_status_debug = (uint32_t)status;
        if (status == HAL_OK)
        {
            pca9685_i2c_handle = hi2c;
            pca9685_dev_addr = dev_addr;
            pca9685_i2c_bus_debug = bus_id;
            pca9685_addr_7bit_debug = addr_7bit;
            pca9685_ready_debug = 1U;
            return 1U;
        }
    }

    return 0U;
}

static void PCA9685_SelectDevice(void)
{
    pca9685_ready_debug = 0U;
    pca9685_i2c_bus_debug = 0U;
    pca9685_addr_7bit_debug = 0U;
    pca9685_i2c_handle = &hi2c2;
    pca9685_dev_addr = PCA9685_ADDR;

    if (PCA9685_TrySelectDeviceOnBus(&hi2c2, 2U) != 0U)
    {
        return;
    }

    if (hi2c1.Instance != I2C1)
    {
        MX_I2C1_Init();
    }

    if (PCA9685_TrySelectDeviceOnBus(&hi2c1, 1U) != 0U)
    {
        return;
    }

    pca9685_i2c_handle = &hi2c2;
    pca9685_dev_addr = PCA9685_ADDR;
    pca9685_i2c_bus_debug = 2U;
    pca9685_addr_7bit_debug = (uint8_t)(PCA9685_ADDR >> 1);
}

/******************************************************************
 * 函 数 名 称：PCA9685_Write
 * 函 数 说 明：向PCA9685写一个字节数据（硬件I2C）
 * 形    参：reg 寄存器地址，data 写入的数据
 * 返 回 值：无
 ******************************************************************/
static HAL_StatusTypeDef PCA9685_Write(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(pca9685_i2c_handle, pca9685_dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    pca9685_last_hal_status_debug = (uint32_t)status;
    if (status != HAL_OK)
    {
        pca9685_write_error_count_debug++;
    }

    return status;
}

/******************************************************************
 * 函 数 名 称：PCA9685_Read
 * 函 数 说 明：读取PCA9685一个字节数据（硬件I2C）
 * 形    参：reg 寄存器地址
 * 返 回 值：读取的数据
 ******************************************************************/
static uint8_t PCA9685_Read(uint8_t reg)
{
    HAL_StatusTypeDef status;
    uint8_t data = 0;

    status = HAL_I2C_Mem_Read(pca9685_i2c_handle, pca9685_dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    pca9685_last_hal_status_debug = (uint32_t)status;
    if (status != HAL_OK)
    {
        pca9685_read_error_count_debug++;
    }

    return data;
}

/******************************************************************
 * 函 数 名 称：PCA9685_setPWM
 * 函 数 说 明：设置指定通道的ON/OFF计数值（底层函数）
 * 形    参：channel 通道号 0~15，on ON计数值，off OFF计数值
 * 返 回 值：无
 ******************************************************************/
static HAL_StatusTypeDef PCA9685_setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t data[4];
    HAL_StatusTypeDef status;

    data[0] = on  & 0xFF;        // ON_L
    data[1] = on  >> 8;           // ON_H
    data[2] = off & 0xFF;        // OFF_L
    data[3] = off >> 8;           // OFF_H

    status = HAL_I2C_Mem_Write(pca9685_i2c_handle, pca9685_dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 4, 100);
    pca9685_last_hal_status_debug = (uint32_t)status;
    if (status != HAL_OK)
    {
        pca9685_write_error_count_debug++;
    }

    return status;
}

/******************************************************************
 * 函 数 名 称：PCA9685_setFreq
 * 函 数 说 明：设置PCA9685的PWM输出频率（内部调用）
 * 形    参：freq 目标频率（Hz）
 * 返 回 值：无
 ******************************************************************/
static void PCA9685_setFreq(float freq)
{
    uint8_t prescale, oldmode, newmode;
    double prescaleval;

    // 计算预分频值：prescale = round(25e6 / (4096 * freq)) - 1
    prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    prescale = (uint8_t)(prescaleval + 0.5);   // 四舍五入

    // 读取当前Mode1寄存器
    oldmode = PCA9685_Read(PCA9685_MODE1);

    // 设置SLEEP位，进入睡眠模式以允许修改预分频
    newmode = (oldmode & 0x7F) | 0x10;
    PCA9685_Write(PCA9685_MODE1, newmode);

    // 写入预分频值
    PCA9685_Write(PCA9685_PRESCALE, prescale);

    // 恢复原Mode1值，退出睡眠
    PCA9685_Write(PCA9685_MODE1, oldmode);

    HAL_Delay(5);   // 等待振荡器稳定
    // 开启自动递增（可选）
    PCA9685_Write(PCA9685_MODE1, oldmode | 0xA1);
}

/******************************************************************
 * 函 数 名 称：PCA9685_Init
 * 函 数 说 明：初始化PCA9685，设置PWM频率，所有通道初始占空比为0
 * 形    参：freq PWM频率（Hz）
 * 返 回 值：无
 ******************************************************************/
void PCA9685_Init(float freq)
{
    PCA9685_SelectDevice();

    // 复位Mode1寄存器（必须步骤）
    PCA9685_Write(PCA9685_MODE1, 0x00);

    // 设置PWM频率
    PCA9685_setFreq(freq);

    // 将所有通道的占空比初始化为0（OFF = 0）
    for (uint8_t i = 0; i < 16; i++)
    {
        PCA9685_setPWM(i, 0, 0);
    }
}

/******************************************************************
 * 函 数 名 称：PCA9685_SetDuty
 * 函 数 说 明：设置指定通道的PWM占空比
 * 形    参：channel 通道号 0~15
 *           duty    占空比 (0.0 ~ 1.0)
 * 返 回 值：无
 ******************************************************************/
void PCA9685_SetDuty(uint8_t channel, float duty)
{
    uint16_t off;

    // 限制duty在合法范围
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // 计算OFF计数值（12位分辨率）
    off = (uint16_t)(duty * 4096.0f + 0.5f);
    if (off > 4095) off = 4095;

    // 调用底层设置函数，ON固定为0
    PCA9685_setPWM(channel, 0, off);
}
