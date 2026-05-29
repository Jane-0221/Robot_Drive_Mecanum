#include "arm_sv.h"
#include "bsp_pca9685.h"
#include "stdio.h"

#define PI 3.14159265358979323846f
#define THREE_PI_OVER_FOUR (PI * 3.0f / 4.0f)
float motor_radians[6] = {0.0f};

// ===================== 斜坡相关变量 =====================
/**
 * @brief 斜坡结构体数组：每个舵机对应一个斜坡发生器
 * @note  用于实现舵机角度的平滑过渡
 */
static ramp_t servo_ramps[ARM_SV_COUNT];

/**
 * @brief 斜坡使能标志数组
 * @note  true表示启用斜坡功能，false表示直接设置（瞬时）
 */
static uint8_t ramp_enabled[ARM_SV_COUNT] = {0};

/**
 * @brief 初始化所有舵机的斜坡发生器
 * @retval 无
 */
static void init_servo_ramps(void)
{
    // 弧度范围：-3π/4 ~ +3π/4
    float max_radian = THREE_PI_OVER_FOUR;
    float min_radian = -THREE_PI_OVER_FOUR;
    
    for (int i = 0; i < ARM_SV_COUNT; i++)
    {
        ramp_init(&servo_ramps[i], ARM_SV_RAMP_SPEED, max_radian, min_radian);
        ramp_set_current(&servo_ramps[i], 0.0f);
        ramp_enabled[i] = 1;  // 默认启用斜坡
    }
}

/**
 * @brief 270°模式下弧度转占空比函数
 * @param radian 输入弧度值（范围：-3π/4 ~ +3π/4，即-135°~+135°）
 * @return 对应的占空比值（范围：0.025 ~ 0.125）
 * @note 占空比0.075对应弧度0，线性映射关系
 */
float radian_to_duty_270(float radian)
{
    // 角度范围：-135° ~ +135°（-3π/4 ~ +3π/4 弧度）
    // 占空比范围：0.025 ~ 0.125
    // 中点：弧度0对应占空比0.075

    // 限制弧度范围
    if (radian < -THREE_PI_OVER_FOUR)
    {
        radian = -THREE_PI_OVER_FOUR;
    }
    else if (radian > THREE_PI_OVER_FOUR)
    {
        radian = THREE_PI_OVER_FOUR;
    }

    // 线性映射公式：duty = 0.075 + (radian / (3π/4)) * 0.05
    float duty = 0.075f + (radian / THREE_PI_OVER_FOUR) * 0.05f;

    return duty;
}

/**
 * @brief 设置6个电机的弧度值（带斜坡处理）
 * @param radians 包含6个弧度值的数组
 * @note 每个弧度值范围：-3π/4 ~ +3π/4
 *       对所有舵机启用斜坡处理，使其平滑过渡
 */
void set_motor_radians_270(float radians[6])
{
    for (int i = 0; i < 6; i++)
    {
        // 设置斜坡目标值
        ramp_set_target(&servo_ramps[i], radians[i]);
        
        // 将弧度转换为占空比并设置
        switch (i)
        {
        case 0:
            duties_tx.duty0 = radian_to_duty_270(ramp_get_current(&servo_ramps[0]) / 2);
            break;
        case 1:
            duties_tx.duty1 = radian_to_duty_270(ramp_get_current(&servo_ramps[1]) / 2);
            break;
        case 2:
            duties_tx.duty2 = radian_to_duty_270(ramp_get_current(&servo_ramps[2]));
            break;
        case 3:
            duties_tx.duty3 = radian_to_duty_270(ramp_get_current(&servo_ramps[3]));
            break;
        case 4:
            duties_tx.duty4 = radian_to_duty_270(ramp_get_current(&servo_ramps[4]));
            break;
        case 5:
            duties_tx.duty5 = radian_to_duty_270(ramp_get_current(&servo_ramps[5]));
            break;
        }
    }
}

// ===================== 全局变量定义 =====================
/**
 * @brief 静态数组：存储每个舵机当前的PWM占空比（0~1.0对应0%~100%）
 * @note  ARM_SV_COUNT为舵机总数宏定义（本代码中为6路，对应0~5号舵机）
 */
static float current_duties[ARM_SV_COUNT];

/**
 * @brief 舵机占空比接收结构体：存储从硬件读取的当前占空比
 * @note  包含duty0~duty5共6个成员，对应0~5号舵机
 */
ARM_SV_Duties_t duties_rx;

/**
 * @brief 舵机占空比发送结构体：存储待设置的目标占空比
 * @note  包含duty0~duty5共6个成员，对应0~5号舵机
 */
ARM_SV_Duties_t duties_tx;

// ===================== 内部函数定义（仅本文件使用） =====================
/**
 * @brief 内部函数：设置单个舵机的PWM占空比
 * @param  id    舵机编号（0~ARM_SV_COUNT-1，超出范围则直接返回）
 * @param  duty  目标占空比（0.0~1.0，对应0%~100%）
 * @retval 无
 * @note   1. 先检查舵机编号有效性，避免数组越界；
 *         2. 调用PCA9685底层驱动设置占空比；
 *         3. 更新current_duties数组，记录当前实际占空比。
 */
static void set_sv_duty(uint8_t id, float duty)
{
    if (id >= ARM_SV_COUNT) // 舵机编号越界检查
        return;
    PCA9685_SetDuty(id, duty); // 调用PCA9685驱动设置指定舵机的PWM占空比
    current_duties[id] = duty; // 更新当前占空比缓存
}

// ===================== 对外接口函数定义 =====================
/**
 * @brief 机械臂舵机初始化函数
 * @param  freq  PCA9685的PWM输出频率（单位：Hz，舵机常用50Hz）
 * @retval 无
 * @note   1. 初始化duties_tx结构体，所有舵机目标占空比置0；
 *         2. 初始化PCA9685芯片（设置PWM频率）；
 *         3. 初始化current_duties数组，所有舵机当前占空比置0。
 */
void ARM_SV_Init(float freq)
{
    // 初始化发送结构体，所有舵机目标占空比默认0
    duties_tx.duty0 = 0;
    duties_tx.duty1 = 0;
    duties_tx.duty2 = 0;
    duties_tx.duty3 = 0;
    duties_tx.duty4 = 0;
    duties_tx.duty5 = 0;

    PCA9685_Init(freq); // 初始化PCA9685 PWM驱动芯片

    // 初始化当前占空比缓存数组，所有元素置0
    for (uint8_t i = 0; i < ARM_SV_COUNT; i++)
    {
        current_duties[i] = 0.0f;
    }
    
    // 初始化斜坡发生器
    init_servo_ramps();
}

/**
 * @brief 设置指定编号舵机的PWM占空比（通用接口）
 * @param  sv_id 舵机编号（0~ARM_SV_COUNT-1）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 * @note   封装内部set_sv_duty函数，对外提供统一的设置接口
 */
void ARM_SV_SetDuty(uint8_t sv_id, float duty)
{
    set_sv_duty(sv_id, duty);
}

/**
 * @brief 设置0号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty0(float duty) { set_sv_duty(0, duty); }

/**
 * @brief 设置1号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty1(float duty) { set_sv_duty(1, duty); }

/**
 * @brief 设置2号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty2(float duty) { set_sv_duty(2, duty); }

/**
 * @brief 设置3号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty3(float duty) { set_sv_duty(3, duty); }

/**
 * @brief 设置4号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty4(float duty) { set_sv_duty(4, duty); }

/**
 * @brief 设置5号舵机的PWM占空比（专用接口）
 * @param  duty  目标占空比（0.0~1.0）
 * @retval 无
 */
void ARM_SV_SetDuty5(float duty) { set_sv_duty(5, duty); }

/**
 * @brief 获取指定编号舵机的当前PWM占空比
 * @param  sv_id 舵机编号（0~ARM_SV_COUNT-1，超出范围返回0.0）
 * @retval float 当前占空比（0.0~1.0）
 * @note   通过读取current_duties缓存数组获取，无需访问硬件
 */
float ARM_SV_GetDuty(uint8_t sv_id)
{
    if (sv_id >= ARM_SV_COUNT) // 舵机编号越界检查
        return 0.0f;
    return current_duties[sv_id]; // 返回缓存的当前占空比
}

/**
 * @brief 获取0号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty0(void) { return current_duties[0]; }

/**
 * @brief 获取1号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty1(void) { return current_duties[1]; }

/**
 * @brief 获取2号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty2(void) { return current_duties[2]; }

/**
 * @brief 获取3号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty3(void) { return current_duties[3]; }

/**
 * @brief 获取4号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty4(void) { return current_duties[4]; }

/**
 * @brief 获取5号舵机的当前PWM占空比（专用接口）
 * @retval float 当前占空比（0.0~1.0）
 */
float ARM_SV_GetDuty5(void) { return current_duties[5]; }

/**
 * @brief 批量设置所有舵机的PWM占空比
 * @param  duties 占空比数组指针（数组长度需≥ARM_SV_COUNT）
 * @retval 无
 * @note   遍历数组，依次调用ARM_SV_SetDuty设置每个舵机的占空比
 */
void ARM_SV_SetAllDuties(const float *duties)
{
    for (uint8_t i = 0; i < ARM_SV_COUNT; i++)
    {
        ARM_SV_SetDuty(i, duties[i]);
    }
}

/**
 * @brief 批量获取所有舵机的当前PWM占空比
 * @retval ARM_SV_Duties_t 包含所有舵机占空比的结构体
 * @note   将current_duties数组的值赋值到结构体成员，统一返回
 */
ARM_SV_Duties_t ARM_SV_GetAllDuties(void)
{
    ARM_SV_Duties_t duties;

    // 逐个赋值当前占空比到结构体
    duties.duty0 = current_duties[0];
    duties.duty1 = current_duties[1];
    duties.duty2 = current_duties[2];
    duties.duty3 = current_duties[3];
    duties.duty4 = current_duties[4];
    duties.duty5 = current_duties[5];

    return duties;
}

/**
 * @brief 设置指定舵机的斜坡目标值
 * @param  sv_id 舵机编号（0~5）
 * @param  target_radian 目标弧度值
 * @retval 无
 * @note   使用斜坡函数使舵机平滑移动到目标角度
 */
void ARM_SV_SetRampTarget(uint8_t sv_id, float target_radian)
{
    if (sv_id >= ARM_SV_COUNT)
        return;
    
    ramp_enabled[sv_id] = 1;  // 启用斜坡
    ramp_set_target(&servo_ramps[sv_id], target_radian);
}

/**
 * @brief 设置所有舵机的斜坡目标值
 * @param  radians 包含6个目标弧度值的数组
 * @retval 无
 * @note   批量设置所有舵机的目标角度，使用斜坡函数平滑过渡
 */
void ARM_SV_SetAllRampTargets(const float *radians)
{
    for (int i = 0; i < ARM_SV_COUNT; i++)
    {
        ramp_enabled[i] = 1;  // 启用斜坡
        ramp_set_target(&servo_ramps[i], radians[i]);
    }
}

/**
 * @brief 斜坡计算更新（需要在周期循环中调用）
 * @param  dt 时间步长（秒），通常为控制周期
 * @retval 无
 * @note   更新所有启用斜坡的舵机位置
 */
void ARM_SV_RampUpdate(float dt)
{
    for (int i = 0; i < ARM_SV_COUNT; i++)
    {
        if (ramp_enabled[i])
        {
            ramp_calculate(&servo_ramps[i], dt);
        }
    }
}

/**
 * @brief 舵机占空比收发核心函数
 * @retval 无
 * @note   1. 发送（Tx）：将duties_tx结构体中的目标占空比设置到对应舵机；
 *         2. 接收（Rx）：读取所有舵机当前占空比，存入duties_rx结构体；
 *         3. 调试输出代码已注释，如需调试可取消注释查看占空比（单位：%）。
 */
void ARM_SV_Tx_Rx()
{
    // 更新斜坡计算（1ms = 0.001秒）
    ARM_SV_RampUpdate(0.001f);
    
    set_motor_radians_270(motor_radians);
    
    // 发送：将duties_tx的目标占空比设置到对应舵机
    ARM_SV_SetDuty0(duties_tx.duty0); // 设置0号舵机占空比
    ARM_SV_SetDuty1(duties_tx.duty1); // 设置1号舵机占空比
    ARM_SV_SetDuty2(duties_tx.duty2); // 设置2号舵机占空比
    ARM_SV_SetDuty3(duties_tx.duty3); // 设置3号舵机占空比
    ARM_SV_SetDuty4(duties_tx.duty4); // 设置4号舵机占空比
    ARM_SV_SetDuty5(duties_tx.duty5); // 设置5号舵机占空比

    // 接收：获取所有舵机当前占空比，存入duties_rx结构体
    duties_rx = ARM_SV_GetAllDuties();

    // 调试输出（已注释）：打印所有舵机当前占空比（乘以100转为百分比）
    // printf("===== Current PWM Duties =====\n");
    // printf("Servo 0: %.2f%%\n", duties_rx.duty0 * 100);
    // printf("Servo 1: %.2f%%\n", duties_rx.duty1 * 100);
    // printf("Servo 2: %.2f%%\n", duties_rx.duty2 * 100);
    // printf("Servo 3: %.2f%%\n", duties_rx.duty3 * 100);
    // printf("Servo 4: %.2f%%\n", duties_rx.duty4 * 100);
    // printf("Servo 5: %.2f%%\n", duties_rx.duty5 * 100);
}
