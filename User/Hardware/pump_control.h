#ifndef __PUMP_CONTROL_H
#define __PUMP_CONTROL_H

// 泵控制相关引脚定义（根据实际硬件电路修改）
#define PUMP_RELAY_PIN      GPIO_PIN_13   // 泵继电器控制引脚（实际为PE13，原注释PB3为笔误）
#define PUMP_RELAY_PORT     GPIOE         // 泵继电器引脚所属GPIO端口

#define SOLENOID_VALVE_PIN  GPIO_PIN_9    // 电磁阀控制引脚
#define SOLENOID_VALVE_PORT GPIOE         // 电磁阀引脚所属GPIO端口

// 继电器电平定义：低电平表示继电器导通（泵/阀工作），高电平表示断开（泵/阀停止）
// 注：使用前需确认硬件电路的电平匹配性，避免逻辑错误
#define PUMP_RELAY_ACTIVE   GPIO_PIN_RESET
#define PUMP_RELAY_INACTIVE GPIO_PIN_SET

// 泵工作状态枚举
typedef enum {
    PUMP_OFF = 0,   // 泵关闭状态
    PUMP_ON  = 1    // 泵开启状态
} PUMP_State;

// 药品吸取状态枚举
typedef enum {
    LIQUID_NOT_SUCKED = 0,  // 未吸到药品
    LIQUID_SUCKED     = 1   // 已成功吸到药品
} LIQUID_State;

// 全局变量声明（若需在其他文件使用boom_arm_status，需补充对应的extern声明）
extern PUMP_State pump_state;   // 当前泵的工作状态
extern LIQUID_State liquid_state;  // 当前药品吸取状态

// 函数功能声明
void Pump_Init(void);               // 初始化泵继电器和电磁阀的GPIO引脚
void Pump_Update(void);             // 泵状态更新函数：根据药品吸取状态自动控制泵的启停
LIQUID_State Check_Liquid_Sucked(void);  // 检测是否成功吸到药品，返回对应的吸取状态
#endif /* __PUMP_CONTROL_H */
