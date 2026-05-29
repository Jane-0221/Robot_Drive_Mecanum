#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_I2C_DEV_ADDR    0x52U
#define VL53L0X_I2C_HAL_ADDR    VL53L0X_I2C_DEV_ADDR
#define VL53L0X_I2C_SHIFT_ADDR  (VL53L0X_I2C_DEV_ADDR << 1)
#define VL53L0X_RESULT_LEN      12U
#define VL53L0X_SCAN_MAX        8U

typedef enum
{
    VL53L0X_OK = 0,
    VL53L0X_ERROR,
    VL53L0X_NOT_FOUND,
    VL53L0X_TIMEOUT
} VL53L0X_StatusTypeDef;

typedef enum
{
    VL53L0X_STEP_NONE = 0,
    VL53L0X_STEP_DEVICE_READY,
    VL53L0X_STEP_START_WRITE,
    VL53L0X_STEP_READY_POLL,
    VL53L0X_STEP_RESULT_REG_WRITE,
    VL53L0X_STEP_RESULT_READ
} VL53L0X_StepTypeDef;

extern volatile uint16_t vl53l0x_distance_mm;
extern volatile uint8_t vl53l0x_raw_data[VL53L0X_RESULT_LEN];
extern volatile VL53L0X_StatusTypeDef vl53l0x_status;
extern volatile VL53L0X_StepTypeDef vl53l0x_error_step;
extern volatile uint8_t vl53l0x_last_hal_status;
extern volatile uint32_t vl53l0x_last_hal_error;
extern volatile uint8_t vl53l0x_scan_count;
extern volatile uint8_t vl53l0x_scan_addr[VL53L0X_SCAN_MAX];
extern volatile uint8_t vl53l0x_scl_level;
extern volatile uint8_t vl53l0x_sda_level;
extern volatile uint8_t vl53l0x_i2c_state;
extern volatile uint8_t vl53l0x_i2c_busy_flag;
extern volatile uint32_t vl53l0x_i2c_isr;
extern volatile uint32_t vl53l0x_i2c_cr2;
extern volatile uint8_t vl53l0x_ready_status;
extern volatile uint8_t vl53l0x_ready_poll_count;
extern volatile uint8_t vl53l0x_range_status;
extern volatile uint8_t vl53l0x_probe_52_status;
extern volatile uint8_t vl53l0x_probe_29_status;
extern volatile uint8_t vl53l0x_probe_52_shift_status;
extern volatile uint8_t vl53l0x_soft_probe_52_status;
extern volatile uint8_t vl53l0x_soft_probe_a4_status;
extern volatile uint8_t vl53l0x_recover_count;

VL53L0X_StatusTypeDef VL53L0X_Init(void);
VL53L0X_StatusTypeDef VL53L0X_ReadDistance(void);
uint8_t VL53L0X_ScanI2C1(void);
void VL53L0X_UpdateBusDebug(void);
void VL53L0X_RecoverI2C1(void);
uint8_t VL53L0X_SoftProbeI2C1(void);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_H */
