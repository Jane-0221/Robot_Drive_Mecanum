#include "bsp_pca9685.h"
#include <math.h>

// 包含HAL I2C头文件，并声明外部I2C句柄（由CubeMX生成）
#include "i2c.h"
extern I2C_HandleTypeDef hi2c2;   // 使用I2C2，与您的初始化对应

// 延时函数（需外部实现）
extern void delay_us(uint32_t us);
extern void HAL_Delay(uint32_t ms);

/******************************************************************
 * 函 数 名 称：PCA9685_Write
 * 函 数 说 明：向PCA9685写一个字节数据（硬件I2C）
 * 形    参：reg 寄存器地址，data 写入的数据
 * 返 回 值：无
 ******************************************************************/
static void PCA9685_Write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/******************************************************************
 * 函 数 名 称：PCA9685_Read
 * 函 数 说 明：读取PCA9685一个字节数据（硬件I2C）
 * 形    参：reg 寄存器地址
 * 返 回 值：读取的数据
 ******************************************************************/
static uint8_t PCA9685_Read(uint8_t reg)
{
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c2, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

/******************************************************************
 * 函 数 名 称：PCA9685_setPWM
 * 函 数 说 明：设置指定通道的ON/OFF计数值（底层函数）
 * 形    参：channel 通道号 0~15，on ON计数值，off OFF计数值
 * 返 回 值：无
 ******************************************************************/
static void PCA9685_setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t data[4];

    data[0] = on  & 0xFF;        // ON_L
    data[1] = on  >> 8;           // ON_H
    data[2] = off & 0xFF;        // OFF_L
    data[3] = off >> 8;           // OFF_H

    HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 4, 100);
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