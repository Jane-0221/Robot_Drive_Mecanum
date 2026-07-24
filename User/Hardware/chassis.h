#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHASSIS_MODE_MECANUM 1U

extern volatile uint8_t chassis_mode;

/*
 * Chassis body velocity command.
 * x: forward velocity, m/s
 * y: rightward velocity, m/s
 * w: clockwise yaw rate, rad/s
 */
extern volatile float x;
extern volatile float y;
extern volatile float w;

void Chassis_Init(void);
void Chassis_Update(void);
void Chassis_RxCallback(uint32_t ext_id, uint8_t *data);
void Chassis_SetCommand(float vx, float vy, float yaw_rate);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
