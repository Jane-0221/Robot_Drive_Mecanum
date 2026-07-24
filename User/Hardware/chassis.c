#include "chassis.h"

#include "mecanum_wheel.h"

volatile uint8_t chassis_mode = CHASSIS_MODE_MECANUM;
volatile float x = 0.0f;
volatile float y = 0.0f;
volatile float w = 0.0f;

static uint8_t chassis_mecanum_initialized = 0U;

static uint8_t Chassis_NormalizeMode(uint8_t mode)
{
    (void)mode;
    return CHASSIS_MODE_MECANUM;
}

static void Chassis_EnsureModeInitialized(uint8_t mode)
{
    (void)mode;

    if (chassis_mecanum_initialized == 0U)
    {
        Mecanum_Wheel_Init();
        chassis_mecanum_initialized = 1U;
    }
}

void Chassis_Init(void)
{
    Chassis_SetCommand(0.0f, 0.0f, 0.0f);
    chassis_mode = Chassis_NormalizeMode(chassis_mode);
    Chassis_EnsureModeInitialized(chassis_mode);
}

void Chassis_Update(void)
{
    uint8_t mode = Chassis_NormalizeMode(chassis_mode);

    if (mode != chassis_mode)
    {
        chassis_mode = mode;
    }

    Chassis_EnsureModeInitialized(mode);
    Mecanum_Wheel_Update();
}

void Chassis_RxCallback(uint32_t ext_id, uint8_t *data)
{
    uint8_t mode = Chassis_NormalizeMode(chassis_mode);

    (void)mode;
    Mecanum_Wheel_RxCallback(ext_id, data);
}

void Chassis_SetCommand(float vx, float vy, float yaw_rate)
{
    x = vx;
    y = vy;
    w = yaw_rate;
}
