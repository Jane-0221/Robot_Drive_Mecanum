/**
 * @file ramp.h
 * @brief 简单斜坡函数接口
 * @description 提供简洁的斜坡函数接口，方便快速集成到各种控制场景
 *             支持平滑过渡、限幅和可配置的速度控制
 */
#ifndef RAMP_H
#define RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 简单斜坡结构体
 * @note 提供简化版斜坡发生器，适用于只需要简单平滑过渡的场景
 */
typedef struct {
    float current;      // 当前值
    float target;      // 目标值
    float speed;       // 变化速度（单位：数值/秒）
    float max_limit;   // 最大值限制
    float min_limit;   // 最小值限制
} ramp_t;

/**
 * @brief 初始化斜坡发生器
 * @param ramp 斜坡实例指针
 * @param speed 变化速度（单位：数值/秒）
 * @param max_limit 最大值限制
 * @param min_limit 最小值限制（默认为-max_limit）
 */
void ramp_init(ramp_t *ramp, float speed, float max_limit, float min_limit);

/**
 * @brief 设置目标值
 * @param ramp 斜坡实例指针
 * @param target 目标值
 */
void ramp_set_target(ramp_t *ramp, float target);

/**
 * @brief 设置变化速度
 * @param ramp 斜坡实例指针
 * @param speed 新速度（单位：数值/秒）
 */
void ramp_set_speed(ramp_t *ramp, float speed);

/**
 * @brief 设置限幅范围
 * @param ramp 斜坡实例指针
 * @param min 最小值
 * @param max 最大值
 */
void ramp_set_limit(ramp_t *ramp, float min, float max);

/**
 * @brief 斜坡计算（固定时间步长）
 * @param ramp 斜坡实例指针
 * @param dt 时间步长（秒），通常为控制周期（如0.001表示1ms）
 * @return 当前平滑值
 */
float ramp_calculate(ramp_t *ramp, float dt);

/**
 * @brief 获取当前值
 * @param ramp 斜坡实例指针
 * @return 当前值
 */
float ramp_get_current(const ramp_t *ramp);

/**
 * @brief 判断是否达到目标值
 * @param ramp 斜坡实例指针
 * @param threshold 判定阈值
 * @return 1:达到目标 0:未达到目标
 */
int ramp_is_reached(const ramp_t *ramp, float threshold);

/**
 * @brief 强制设置当前值
 * @param ramp 斜坡实例指针
 * @param value 当前值
 */
void ramp_set_current(ramp_t *ramp, float value);

/**
 * @brief 停止斜坡（直接到达目标）
 * @param ramp 斜坡实例指针
 */
void ramp_stop(ramp_t *ramp);

#ifdef __cplusplus
}
#endif

#endif // RAMP_H
