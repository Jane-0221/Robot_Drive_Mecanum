/**
 * @file Stm32_time.h
 * @author sethome 
 * @brief
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#define __STM32_TIME_H
#ifdef __STM32_TIME_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
uint32_t Get_sys_time_ms(void); // recommend
uint32_t Get_sys_time_us(void);
float Get_sys_time_s(void);
void TIM_count_100KHz(void);
#endif
// end of file
