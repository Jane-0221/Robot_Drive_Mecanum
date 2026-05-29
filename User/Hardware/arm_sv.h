#ifndef __ARM_SV_H
#define __ARM_SV_H

#include <stdint.h>
#include "ramp.h"

// ===================== 宏定义 =====================
/**
 * @brief 机械臂舵机总数
 * @note  固定为6路（对应0~5号舵机），与ARM_SV_Duties_t结构体成员一一对应
 */
#define ARM_SV_COUNT        6

/**
 * @brief 斜坡变化速度（单位：弧度/秒）
 * @note  值越大，舵机响应越快；值越小，舵机运动越平滑
 */
#define ARM_SV_RAMP_SPEED   20.0f

extern float motor_radians[6];

// ===================== 数据结构体定义 =====================
/**
 * @brief 机械臂6路舵机PWM占空比存储结构体
 * @note  每个成员存储对应编号舵机的PWM占空比，取值范围0.0~1.0（对应0%~100%）
 */
typedef struct {
    float duty0;   // 0号舵机PWM占空比（0.0~1.0）
    float duty1;   // 1号舵机PWM占空比（0.0~1.0）
    float duty2;   // 2号舵机PWM占空比（0.0~1.0）
    float duty3;   // 3号舵机PWM占空比（0.0~1.0）
    float duty4;   // 4号舵机PWM占空比（0.0~1.0）
    float duty5;   // 5号舵机PWM占空比（0.0~1.0）
} ARM_SV_Duties_t;

// ===================== 全局变量声明 =====================
/**
 * @brief 舵机占空比发送结构体（存储待设置的目标占空比）
 * @note  外部文件可直接修改该结构体成员，调用ARM_SV_Tx_Rx()生效
 */
extern ARM_SV_Duties_t duties_tx;

// ===================== 函数声明 =====================
float radian_to_duty_270(float radian);
void set_motor_radians_270(float radians[6]);
/**
 * @brief 机械臂舵机初始化函数
 * @param  freq  PWM输出频率（单位：Hz）
 * @retval 无
 * @note   1. 初始化PCA9685 PWM驱动芯片，设置输出频率；
 *         2. 舵机常用频率为50Hz（对应20ms周期）；
 *         3. 初始化所有舵机占空比为0，duties_tx结构体成员置0。
 */
void ARM_SV_Init(float freq);

/**
 * @brief 设置指定编号舵机的PWM占空比（通用接口）
 * @param  sv_id 舵机编号（0 ~ ARM_SV_COUNT-1，即0~5）
 * @param  duty  目标占空比（0.0~1.0，超出范围可能导致舵机异常）
 * @retval 无
 * @note   该接口为通用型，可通过参数控制任意一路舵机
 */
void ARM_SV_SetDuty(uint8_t sv_id, float duty);

/**
 * @brief 设置0号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty0(float duty);

/**
 * @brief 设置1号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty1(float duty);

/**
 * @brief 设置2号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty2(float duty);

/**
 * @brief 设置3号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty3(float duty);

/**
 * @brief 设置4号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty4(float duty);

/**
 * @brief 设置5号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty5(float duty);

/**
 * @brief 获取指定编号舵机的当前PWM占空比
 * @param  sv_id 舵机编号（0 ~ ARM_SV_COUNT-1，超出范围返回0.0）
 * @retval float 当前占空比（0.0~1.0）
 * @note   返回值为缓存的占空比（非实时读取硬件），与上次设置值一致
 */
float ARM_SV_GetDuty(uint8_t sv_id);

/**
 * @brief 获取0号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty0(void);

/**
 * @brief 获取1号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty1(void);

/**
 * @brief 获取2号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty2(void);

/**
 * @brief 获取3号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty3(void);

/**
 * @brief 获取4号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty4(void);

/**
 * @brief 获取5号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty5(void);

/**
 * @brief 批量设置所有舵机的PWM占空比
 * @param  duties 占空比数组指针（数组长度需≥ARM_SV_COUNT=6）
 * @retval 无
 * @note   数组索引0~5分别对应0~5号舵机，依次设置占空比
 */
void ARM_SV_SetAllDuties(const float *duties);

/**
 * @brief 批量获取所有舵机的当前PWM占空比
 * @retval ARM_SV_Duties_t 包含6路舵机占空比的结构体
 * @note   返回值为缓存的占空比，可直接读取各成员查看对应舵机占空比
 */
ARM_SV_Duties_t ARM_SV_GetAllDuties(void);

/**
 * @brief 舵机占空比收发核心函数
 * @retval 无
 * @note   1. 发送：将duties_tx结构体中的目标占空比设置到对应舵机；
 *         2. 接收：读取所有舵机当前占空比，存入duties_rx结构体（内部变量）；
 *         3. 建议在主循环中周期性调用，保证舵机状态实时更新。
 */
extern void ARM_SV_Tx_Rx(void);

/**
 * @brief 设置指定舵机的斜坡目标值
 * @param  sv_id 舵机编号（0~5）
 * @param  target_radian 目标弧度值
 * @retval 无
 * @note   使用斜坡函数使舵机平滑移动到目标角度
 */
void ARM_SV_SetRampTarget(uint8_t sv_id, float target_radian);

/**
 * @brief 设置所有舵机的斜坡目标值
 * @param  radians 包含6个目标弧度值的数组
 * @retval 无
 * @note   批量设置所有舵机的目标角度，使用斜坡函数平滑过渡
 */
void ARM_SV_SetAllRampTargets(const float *radians);

/**
 * @brief 斜坡计算更新（需要在周期循环中调用）
 * @param  dt 时间步长（秒），通常为控制周期
 * @retval 无
 * @note   更新所有启用斜坡的舵机位置
 */
void ARM_SV_RampUpdate(float dt);

#endif
