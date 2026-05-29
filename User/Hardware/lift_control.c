#include "lift_control.h" 
#include "remote_control.h" 
#include "usart.h" 
#include "UART_data_txrx.h" 
#include "stp23l.h" 
int16_t aim_tx_height = 0; // 目标高度（每帧更新）
// 全局变量定义 
LIFT_State lift_state = LIFT_STOP; // 初始状态：停止 
uint16_t lift_current_height = 0;  // 初始高度 = 0

// 高度跟踪变量 
int16_t lift_height_final = 0; // 每帧最终高度

// 私有电机控制函数 
static void Motor_Forward(void); 
static void Motor_Reverse(void); 
static void Motor_Stop(void);

/** 
  * @brief  初始化升降控制系统
  * @note   初始化为安全就绪，清空旧状态
  * @param  无 
  * @retval 无 
  */
void Lift_Init(void)
{
    Motor_Stop();
    lift_state = LIFT_STOP;
    lift_current_height = 0;
}

/** 
  * @brief  设置升降机上升状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Up(void)
{
    lift_state = LIFT_UP;
}

/** 
  * @brief  设置升降机下降状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Down(void)
{
    lift_state = LIFT_DOWN;
}

/** 
  * @brief  设置升降机停止状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Stop(void)
{
    lift_state = LIFT_STOP;
}

/** 
  * @brief  设置升降机指定状态 
  * @param  state 要设置的状态 
  * @retval 无 
  */
void Lift_SetState(LIFT_State state)
{
    lift_state = state;
}

/** 
  * @brief  获取当前状态 
  * @retval 当前状态 
  */
LIFT_State Lift_GetState(void)
{
    return lift_state;
}

/** 
  * @brief  根据当前状态更新电机控制 
  * @note   周期性调用（每10ms）来执行实际的电机控制 
  * @param  无 
  * @retval 无 
  */
void Lift_UpdateMotor(void)
{
    switch (lift_state)
    {
    case LIFT_UP:
        Motor_Forward();
        break;
    case LIFT_DOWN:
        Motor_Reverse();
        break;
    case LIFT_STOP:
    default:
        Motor_Stop();
        break;
    }
}

/** 
  * @brief  从传感器刷新高度值 
  * @note   周期性调用以从传感器更新高度，使用STP23L传感器数据 
  * @param  无 
  * @retval 无 
  */
void Lift_RefreshHeight(void)
{

    if (stp23l_data.parse_ok == 1) 
     { 
         lift_height_final = STP23L_GetFinalDistPerFrame(); // 从传感器获取高度
         STP23L_ClearOkFlag();                              // 清除标志位以保持一致性 
     } 
     
     // 使用ADC高度测量的示例（已注释）
     // lift_current_height = HAL_ADC_GetValue(&hadc1); 
     // 目前使用 STP23L 传感器进行高度测量
}

/**
 * @brief  获取前一帧高度
 * @retval 前一帧高度值
 */
uint16_t Lift_GetHeight(void)
{
    return lift_current_height;
}

/* 高度控制相关的函数实现 (简化注释) */

/**
 * @brief  电机正转控制
 * @note   IN1 = 高，IN2 = 低
 * @param  无
 * @retval 无
 */
static void Motor_Forward(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, LIFT_RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, LIFT_RELAY_OFF);
}

/**
 * @brief  电机反转控制
 * @note   IN1 = 低，IN2 = 高
 * @param  无
 * @retval 无
 */
static void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, LIFT_RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, LIFT_RELAY_ON);
}

/**
 * @brief  电机停止
 * @note   两继电器同相输出，电机停止
 * @param  无
 * @retval 无
 */
static void Motor_Stop(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, LIFT_RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, LIFT_RELAY_ON);
}

/**
 * @brief  控制到目标高度
 * @note   通过遥控设置 lift_height_final 达到目标位置，误差约5内停止
 * @param  target_height 目标高度值
 * @retval 无
 */
void Lift_GoToTarget(int16_t target_height)
{
    // 刷新高度值
    Lift_RefreshHeight();
    if (lift_height_final == 0 || target_height == 0)
    {
        return;
    }
    if (target_height < LIFT_TARGET_HEIGHT_MIN_MM)
    {
        target_height = LIFT_TARGET_HEIGHT_MIN_MM;
    }
    // 计算目标高度和当前高度的差值
    int16_t height_diff = target_height - lift_height_final;

    // 根据差值设置升降机运动状态
    if (height_diff > 5)
    {
        // 目标高度高于当前高度，向上升
        lift_state = LIFT_UP;
        return;
    }
    else if (height_diff < -5)
    {
        // 目标高度低于当前高度，向下降
        lift_state = LIFT_DOWN;
        return;
    }
    else
    {
        // 已经达到目标高度，停止
        lift_state = LIFT_STOP;
        return;
    }
}
