#include "pump_control.h"
#include "gpio.h"
#include "pt_sensor.h"
#include "stm32h7xx_hal.h"

PUMP_State pump_state = PUMP_OFF;
LIQUID_State liquid_state = LIQUID_NOT_SUCKED;

#define PRESSURE_THRESHOLD_LOW  10.0f
#define PRESSURE_THRESHOLD_HIGH 70.0f

void Pump_Init(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

    pump_state = PUMP_OFF;
}

void Pump_Update(void)
{
    switch (pump_state)
    {
    case PUMP_ON:
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, PUMP_RELAY_INACTIVE);
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, PUMP_RELAY_INACTIVE);
        break;
    case PUMP_OFF:
    default:
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, PUMP_RELAY_ACTIVE);
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, PUMP_RELAY_ACTIVE);
        break;
    }
}

LIQUID_State Check_Liquid_Sucked(void)
{
    if (g_pressure_value > PRESSURE_THRESHOLD_LOW && g_pressure_value < PRESSURE_THRESHOLD_HIGH)
    {
        liquid_state = LIQUID_SUCKED;
    }
    else
    {
        liquid_state = LIQUID_NOT_SUCKED;
    }

    return liquid_state;
}
