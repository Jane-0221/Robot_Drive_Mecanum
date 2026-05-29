#ifndef __LIFT_CONTROL_H
#define __LIFT_CONTROL_H

#include "stdint.h"
#include "main.h"

// 引脚定义
#define RELAY1_PIN          GPIO_PIN_0
#define RELAY1_PORT         GPIOA
#define RELAY2_PIN          GPIO_PIN_2
#define RELAY2_PORT         GPIOA

// 继电器状态定义
#define LIFT_RELAY_ON       GPIO_PIN_SET
#define LIFT_RELAY_OFF      GPIO_PIN_RESET
#define LIFT_TARGET_HEIGHT_MIN_MM 70

// 升降机状态
typedef enum LIFT_State {
    LIFT_UP = 0,    // 上升
    LIFT_DOWN,      // 下降
    LIFT_STOP       // 停止
} LIFT_State;

extern LIFT_State lift_state;
extern int16_t lift_height_final; // 每帧最终高度
extern uint16_t lift_current_height;   // 当前高度
extern int16_t aim_tx_height;            // 目标高度值，每帧更新

// 函数接口
void Lift_Init(void);                          // 初始化
void Lift_Up(void);                             // 设置上升
void Lift_Down(void);                           // 设置下降
void Lift_Stop(void);                           // 设置停止
void Lift_SetState(LIFT_State state);           // 设置指定状态
LIFT_State Lift_GetState(void);                  // 获取当前状态
void Lift_UpdateMotor(void);                     // 更新电机控制
void Lift_RefreshHeight(void);                   // 刷新高度
uint16_t Lift_GetHeight(void);                    // 获取当前高度
void Lift_GoToTarget(int16_t target_height);    // 控制到目标高度

#endif /* __LIFT_CONTROL_H */
