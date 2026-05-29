/**
 * @file ramp.c
 * @brief 简单斜坡函数实现
 * @description 提供简洁的斜坡函数实现，方便快速集成到各种控制场景
 */
#include "ramp.h"
#include <math.h>

void ramp_init(ramp_t *ramp, float speed, float max_limit, float min_limit)
{
    ramp->current = 0.0f;
    ramp->target = 0.0f;
    ramp->speed = fabsf(speed);
    ramp->max_limit = max_limit;
    ramp->min_limit = min_limit;
}

void ramp_set_target(ramp_t *ramp, float target)
{
    ramp->target = target;
    // 限幅处理
    if (ramp->target > ramp->max_limit) {
        ramp->target = ramp->max_limit;
    }
    if (ramp->target < ramp->min_limit) {
        ramp->target = ramp->min_limit;
    }
}

void ramp_set_speed(ramp_t *ramp, float speed)
{
    ramp->speed = fabsf(speed);
}

void ramp_set_limit(ramp_t *ramp, float min, float max)
{
    ramp->min_limit = min;
    ramp->max_limit = max;
    // 钳制当前值
    if (ramp->current > ramp->max_limit) {
        ramp->current = ramp->max_limit;
    }
    if (ramp->current < ramp->min_limit) {
        ramp->current = ramp->min_limit;
    }
    // 钳制目标值
    if (ramp->target > ramp->max_limit) {
        ramp->target = ramp->max_limit;
    }
    if (ramp->target < ramp->min_limit) {
        ramp->target = ramp->min_limit;
    }
}

float ramp_calculate(ramp_t *ramp, float dt)
{
    if (dt <= 0.0f) {
        return ramp->current;
    }
    
    float error = ramp->target - ramp->current;
    
    // 判断是否已经接近目标
    if (fabsf(error) < 1e-6f) {
        ramp->current = ramp->target;
        return ramp->current;
    }
    
    // 计算步长
    float step = ramp->speed * dt;
    
    // 判断方向
    if (fabsf(error) <= step) {
        // 一步可达
        ramp->current = ramp->target;
    } else {
        // 逐步逼近
        if (error > 0) {
            ramp->current += step;
        } else {
            ramp->current -= step;
        }
    }
    
    // 限幅保护
    if (ramp->current > ramp->max_limit) {
        ramp->current = ramp->max_limit;
    }
    if (ramp->current < ramp->min_limit) {
        ramp->current = ramp->min_limit;
    }
    
    return ramp->current;
}

float ramp_get_current(const ramp_t *ramp)
{
    return ramp->current;
}

int ramp_is_reached(const ramp_t *ramp, float threshold)
{
    return (fabsf(ramp->target - ramp->current) <= threshold) ? 1 : 0;
}

void ramp_set_current(ramp_t *ramp, float value)
{
    ramp->current = value;
    // 限幅处理
    if (ramp->current > ramp->max_limit) {
        ramp->current = ramp->max_limit;
    }
    if (ramp->current < ramp->min_limit) {
        ramp->current = ramp->min_limit;
    }
}

void ramp_stop(ramp_t *ramp)
{
    ramp->current = ramp->target;
}
