#ifndef _BSP_PCA9685_H_
#define _BSP_PCA9685_H_

#include "stm32h7xx.h"
#include <stdint.h>

// PCA9685 寄存器地址
#define PCA9685_ADDR        0x80        // I2C设备地址（8位写地址）
#define PCA9685_MODE1       0x00        // Mode1寄存器
#define PCA9685_PRESCALE    0xFE        // 预分频寄存器
#define LED0_ON_L           0x06        // LED0输出ON计数值低字节

// 函数声明
void PCA9685_Init(float freq);                          // 初始化PCA9685，设置PWM频率
void PCA9685_SetDuty(uint8_t channel, float duty);      // 设置指定通道的PWM占空比 (0.0 ~ 1.0)

extern volatile uint8_t pca9685_ready_debug;
extern volatile uint8_t pca9685_i2c_bus_debug;
extern volatile uint8_t pca9685_addr_7bit_debug;
extern volatile uint32_t pca9685_last_hal_status_debug;
extern volatile uint32_t pca9685_write_error_count_debug;
extern volatile uint32_t pca9685_read_error_count_debug;

#endif
