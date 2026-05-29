# 大然 DrEmpower 电机模块说明

本文档记录工程里原先“大然电机”相关代码的写法和驱动方式。当前代码已经按清理要求移除了机械臂业务层接入，只保留驱动文件：

- `User/Drive/DrEmpower_can.c`
- `User/Drive/DrEmpower_can.h`

`Makefile` 仍保留 `User/Drive/DrEmpower_can.c`，所以驱动源码还参与编译，但机械臂初始化、发送、反馈解析和上行回传中已经不再使用大然电机。

## 原来的硬件与角色

大然电机原先作为机械臂肩部 1~3 号电机的可选方案，和灵足、达妙同挂在 `CAN2` 总线上。

原业务层 ID 定义在 `User/Hardware/arm.h`：

```c
#define MOTOR_DARAN_1_ID 0x0b
#define MOTOR_DARAN_2_ID 0x0c
#define MOTOR_DARAN_3_ID 0x0d
```

当时机械臂肩部有一个品牌选择变量：

```c
typedef enum
{
    SHOULDER_TYPE_LINGZU = 0,
    SHOULDER_TYPE_DARAN = 1
} ShoulderType_t;

ShoulderType_t g_ShoulderType;
```

`g_ShoulderType == SHOULDER_TYPE_DARAN` 时，逻辑 0/1/2 号肩部电机会走大然电机分支；否则走灵足分支。

## 驱动文件接口

大然驱动在 `User/Drive/DrEmpower_can.h` 中把 STM32 FDCAN 句柄封装为：

```c
typedef FDCAN_HandleTypeDef hcan_t;
```

常用接口包括：

```c
void clear_error(hcan_t* hcan, uint8_t id_num);
void set_mode(hcan_t* hcan, uint8_t id_num, int mode);
void write_property(hcan_t* hcan, uint8_t id_num, unsigned short param_address, int8_t param_type, float value);
void set_angle(hcan_t* hcan, uint8_t id_num, float angle, float speed, float param, int mode);
void set_zero_position(hcan_t* hcan, uint8_t id_num);
void DrRobot_ParseFbData(DrRobot_MotorState_t *motor_state, uint8_t *rx_data);
```

驱动内部发送函数会把电机 ID 和命令字合成 CAN 标准帧 ID：

```c
short id_list = (id_num << 5) + cmd;
```

所以原接收侧通过 `(rx_header.Identifier >> 5) & 0x3F` 从标准帧 ID 里反推出大然电机 ID。

## 原来的初始化方式

原 `Arm_Init()` 中会先选择默认肩部电机类型，然后对大然 1~3 号电机执行初始化：

```c
g_ShoulderType = SHOULDER_TYPE_LINGZU;
// g_ShoulderType = SHOULDER_TYPE_DARAN;
```

大然初始化逻辑如下：

```c
clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);
set_mode(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 2);
write_property(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 22001, 3, 1.0f);

clear_error(CAN_HANDLE_2, MOTOR_DARAN_2_ID);
set_mode(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 2);
write_property(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 22001, 3, 1.0f);

clear_error(CAN_HANDLE_2, MOTOR_DARAN_3_ID);
set_mode(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 2);
write_property(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 22001, 3, 1.0f);
```

其中：

- `clear_error()`：清电机错误
- `set_mode(..., 2)`：进入闭环位置控制模式
- `write_property(..., 22001, 3, 1.0f)`：写入参数 22001，原代码用作比例系数配置

原先还维护一组机械臂业务层目标数据：

```c
ArmMotorData_t Daran_motor_data[3];

Daran_motor_data[0].target_angle = 0.0f;
Daran_motor_data[1].target_angle = 0.0f;
Daran_motor_data[2].target_angle = 0.0f;
Daran_motor_data[0].target_velocity = 20.0f;
Daran_motor_data[1].target_velocity = 20.0f;
Daran_motor_data[2].target_velocity = 20.0f;
```

## 原来的发送控制方式

原大然发送封装在 `Arm_SendDaranTarget()`：

```c
static uint8_t Arm_SendDaranTarget(uint8_t logical_motor, uint8_t motor_id, ArmMotorData_t *motor_data)
{
    float target_angle;

    if (Arm_GetSafeTargetAngle(logical_motor, motor_data, &target_angle) == 0U)
    {
        return 0U;
    }

    set_angle(CAN_HANDLE_2, motor_id, target_angle, motor_data->target_velocity, 10.0f, 1);
    return 1U;
}
```

这里会先复用机械臂公共安全限制：

- 目标角度限幅
- 目标角和当前角差值检查
- 第一关节和其他关节不同角度范围限制

然后调用驱动 `set_angle()`：

```c
set_angle(CAN_HANDLE_2, motor_id, target_angle, target_velocity, 10.0f, 1);
```

原来的三个发送函数只是映射逻辑电机到 CAN ID：

```c
Arm_Daran_motor1() -> ID 0x0b -> Daran_motor_data[0]
Arm_Daran_motor2() -> ID 0x0c -> Daran_motor_data[1]
Arm_Daran_motor3() -> ID 0x0d -> Daran_motor_data[2]
```

`Arm_all_tx()` 每 1ms 运行一次，通过 slot 分时发送。原逻辑里 slot 0/2/4 是肩部电机：

```c
slot 0 -> 肩部逻辑 0 -> 灵足1 或 大然1
slot 2 -> 肩部逻辑 1 -> 灵足2 或 大然2
slot 4 -> 肩部逻辑 2 -> 灵足3 或 大然3
```

所以大然目标控制实际频率约为 100Hz。

## 原来的反馈解析方式

原 `User/BSP/CAN_receive_send.c` 在 `FDCAN2` 标准帧分支中，先处理达妙 ID；如果不是达妙 4/5/6，就尝试按大然协议解析：

```c
uint8_t motor_id = (rx_header.Identifier >> 5) & 0x3F;
switch (motor_id)
{
case 11:
    DrRobot_ParseFbData(&daran_motor_state[0], rx_data);
    break;
case 12:
    DrRobot_ParseFbData(&daran_motor_state[1], rx_data);
    break;
case 13:
    break;
default:
    break;
}
```

驱动层的反馈缓存是：

```c
extern DrRobot_MotorState_t daran_motor_state[3];
```

业务层再由 `Arm_Daran_Data_update()` 同步到机械臂公共数据结构：

```c
Daran_motor_data[0].current_angle = daran_motor_state[0].angle;
Daran_motor_data[1].current_angle = daran_motor_state[1].angle;
Daran_motor_data[2].current_angle = daran_motor_state[2].angle;
Daran_motor_data[0].current_velocity = daran_motor_state[0].speed;
Daran_motor_data[1].current_velocity = daran_motor_state[1].speed;
Daran_motor_data[2].current_velocity = daran_motor_state[2].speed;
```

PC 上行数据发送前，原 `PC_Comm_Task()` 会调用：

```c
Arm_Daran_Data_update();
```

再由 `pc_arm_tx_data()` 根据 `g_ShoulderType` 选择上传大然角度或灵足角度。

## 原来的使能、失能、保存零位

大然分支曾经接入机械臂统一电机控制接口。

使能逻辑：

```c
set_mode(CAN_HANDLE_2, motor_id, 2);
osDelay(1);
Arm_SetMotorTxDisabledByIndex(logical_motor, 0U);
Arm_SendDaranTarget(logical_motor, motor_id, motor_data);
```

失能逻辑：

```c
set_mode(CAN_HANDLE_2, motor_id, 1);
Arm_SetMotorTxDisabledByIndex(logical_motor, 1U);
```

保存零位逻辑：

```c
motor_data->target_angle = 0.0f;
set_zero_position(CAN_HANDLE_2, motor_id);
osDelay(1);
set_mode(CAN_HANDLE_2, motor_id, 2);
osDelay(1);
Arm_SendDaranTarget(logical_motor, motor_id, motor_data);
```

## 当前清理后的状态

目前保留：

- `User/Drive/DrEmpower_can.c`
- `User/Drive/DrEmpower_can.h`
- `Makefile` 中 `User/Drive/DrEmpower_can.c`

目前移除：

- `arm.h` 中大然电机 ID、肩部类型枚举、大然业务数组和函数声明
- `arm.c` 中大然初始化、目标数据、发送函数、状态更新函数、使能/失能/保存零位分支
- `CAN_receive_send.c` 中 FDCAN2 大然反馈解析
- `freertos.c` 中 `Arm_Daran_Data_update()` 调用
- `remote_control.c` 中 PC 上行大然角度分支

所以当前固件不会初始化、发送控制帧、解析反馈或上报大然电机数据。后续如果需要恢复，应从保留的 `DrEmpower_can.c/.h` 驱动开始，重新接入 CAN2 初始化、ID 映射、反馈解析和机械臂目标调度。
