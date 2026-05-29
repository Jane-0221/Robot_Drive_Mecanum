# 遥控器控制逻辑说明

本文档说明当前代码中 SBUS 遥控器相关控制逻辑，主要来源于 `User/Software/remote_control.c` 和 `Core/Src/freertos.c`。

## 1. 基本通道值和死区

遥控器三档开关值：

| 名称 | 数值 | 含义 |
| --- | ---: | --- |
| `LOW_VALUE` | `353` | 低位 |
| `MID_VALUE` | `1024` | 中位 |
| `HIGH_VALUE` | `1694` | 高位 |
| `RANGE` | `50` | 摇杆死区/匹配容差 |
| `ARM_MOTOR_STEP` | `0.003f` | 机械臂微调步进，单位为弧度 |

机械臂微调摇杆逻辑：

```c
if (channel > (MID_VALUE + RANGE))
{
    motor_radians[index] += 0.003f;
}
else if (channel < (MID_VALUE - RANGE))
{
    motor_radians[index] -= 0.003f;
}
```

说明：

- `CH1/CH2/CH3` 作为摇杆输入时，中位附近 `1024 +/- 50` 不动作。
- 多数三档开关逻辑使用 `switch (SBUS_CH.CHx)` 精确匹配 `353/1024/1694`。
- 底盘控制条件使用 `sbus_match()`，即目标值 `+/- RANGE` 范围内都认为匹配。

## 2. 任务调用顺序

`Remote_control_Task` 中 `control_mode = 0`，当前默认进入遥控模式。

每个循环：

1. `update_sbus(sbus_data_buffer, &SBUS_CH)`
2. `Arm_Motor_Disable_Updata()`
3. 遥控模式下依次调用：
   - `Pump_Control_Updata()`
   - `Head_Motor_Control_Updata()`
   - `Arm_Motor_Control_Updata()`
   - `Up_Down_Motor_Control_Updata()`
4. `osDelay(1)`

`Motor_control_Task` 独立循环：

1. `update_sbus(sbus_data_buffer, &SBUS_CH)`
2. `Chassis_Control_Updata()`
3. `Omni_Wheel_Update()`
4. `osDelay(1)`

`Arm_MT_Task` 中只有 `Arm_Motor_Disable_IsActive() == 0U` 时才执行 `Arm_all_tx()`。

## 3. 机械臂禁用保护

函数：`Arm_Motor_Disable_Updata()`

触发条件：

| CH5 | CH6 | CH7 | CH8 | 动作 |
| --- | --- | --- | --- | --- |
| HIGH | HIGH | HIGH | HIGH | 禁用机械臂电机 |

触发后：

- `arm_motor_disable_active = 1`
- 灵足肩部电机：依次调用 `Disenable_Motor(&motor1/2/3, CAN_HANDLE_2, 0U)`
- 大然肩部电机：依次调用 `set_mode(..., 1)`
- 达妙 4/5/6 电机：依次调用 `disable_motor_mode(..., POS_MODE)`
- 使用 `arm_motor_disable_latched` 做锁存，避免同一次持续触发中重复发送禁用命令。
- 任一通道离开全高组合后，锁存清零。

## 4. 气泵控制

函数：`Pump_Control_Updata()`

| CH8 | 动作 |
| --- | --- |
| HIGH | `pump_state = PUMP_ON` |
| LOW | `pump_state = PUMP_OFF` |
| 其他 | 不改变当前气泵状态 |

## 5. 头部和机械臂姿态控制

函数：`Head_Motor_Control_Updata()`

### CH8 高位

当 `CH8 == HIGH_VALUE` 时，`CH6` 选择头部角度，并且 `CH1/CH2` 同时微调一对 `motor_radians[]`。

| CH6 | 头部目标角度 | CH1 控制 | CH2 控制 |
| --- | --- | --- | --- |
| HIGH | `Daran_motor_data[0] = 0`, `Daran_motor_data[1] = 0` | `motor_radians[0]` | `motor_radians[1]` |
| MID | `Daran_motor_data[0] = 90`, `Daran_motor_data[1] = 90` | `motor_radians[2]` | `motor_radians[3]` |
| LOW | `Daran_motor_data[0] = 180`, `Daran_motor_data[1] = 180` | `motor_radians[4]` | `motor_radians[5]` |
| 其他 | 不动作 | 不动作 | 不动作 |

微调幅度固定为每个控制周期 `+/-0.003f`。

### CH8 低位

当 `CH8 == LOW_VALUE` 时，当前 `Head_Motor_Control_Updata()` 仍按 `CH6` 写入固定机械臂姿态：

| CH6 | `motor_radians[0]` | `[1]` | `[2]` | `[3]` | `[4]` | `[5]` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| HIGH | `-0.101999983` | `-0.29700008` | `1.80600858` | `2.40001273` | `0.0119999889` | `-0.0150000118` |
| MID | `-0.101999983` | `-0.29700008` | `1.79100847` | `1.88400912` | `0.0119999889` | `-0.0150000118` |
| LOW | `-0.101999983` | `-0.29700008` | `1.88700914` | `1.30800509` | `0.0119999889` | `-0.0150000118` |
| 其他 | 保持当前值 | 保持当前值 | 保持当前值 | 保持当前值 | 保持当前值 | 保持当前值 |

## 6. CH8 低位机械臂手动微调

函数：`Arm_Motor_Control_Updata()`

触发条件：

- 只有 `CH8 == LOW_VALUE` 时生效。

控制映射：

| CH7 | CH1 控制 | CH2 控制 |
| --- | --- | --- |
| HIGH | `motor_radians[0]` | `motor_radians[1]` |
| MID | `motor_radians[2]` | `motor_radians[3]` |
| LOW | `motor_radians[4]` | `motor_radians[5]` |
| 其他 | 不动作 | 不动作 |

注意：

- `Remote_control_Task` 的调用顺序是先 `Head_Motor_Control_Updata()`，后 `Arm_Motor_Control_Updata()`。
- 因此在 `CH8 == LOW_VALUE` 且 `CH6` 精确等于高/中/低任一档时，代码会先写入 CH6 固定姿态，再叠加本周期的 `CH7 + CH1/CH2` 微调。
- 下一周期如果 `CH6` 仍处于高/中/低档，固定姿态会再次写入，所以 CH8 低位下的手动微调不一定能持续累积。

## 7. 升降控制

函数：`Up_Down_Motor_Control_Updata()`

| CH7 | `aim_tx_height` |
| --- | ---: |
| HIGH | `100` |
| MID | `400` |
| LOW | `700` |
| 其他 | 保持当前目标高度 |

说明：

- 该函数不检查 `CH8`，所以 CH7 在任何 CH8 档位都会控制升降目标高度。
- 当 `CH8 == LOW_VALUE` 时，CH7 同时被 `Arm_Motor_Control_Updata()` 用作机械臂电机组选档。

## 8. 底盘控制

函数：`Chassis_Control_Updata()`

底盘只有在以下通道同时匹配低位时才响应：

| CH5 | CH6 | CH7 | CH8 |
| --- | --- | --- | --- |
| LOW +/- RANGE | LOW +/- RANGE | LOW +/- RANGE | LOW +/- RANGE |

满足条件时：

```c
chassis_vx  = -((float)SBUS_CH.CH2 - MID_VALUE) / 300.0f;
chassis_vy  =  ((float)SBUS_CH.CH1 - MID_VALUE) / 300.0f;
chassis_yaw =  ((float)SBUS_CH.CH3 - MID_VALUE) / 200.0f;
```

不满足条件时：

```c
chassis_vx = 0.0f;
chassis_vy = 0.0f;
chassis_yaw = 0.0f;
```

随后写入：

- `up_tx_data.chassis_vx`
- `up_tx_data.chassis_vy`
- `up_tx_data.chassis_yaw`
- 全局变量 `x/y/w`

注意：

- 底盘触发条件包含 `CH7 == LOW` 和 `CH8 == LOW`。
- 该组合下，CH1/CH2 同时也是机械臂 4/5 号电机微调输入，因此当前代码允许底盘和机械臂 4/5 微调同时响应。

## 9. PC 模式补充

`Remote_control_Task` 中存在 PC 模式分支，但当前 `control_mode` 是局部变量并初始化为 `0`，默认不会进入 PC 模式。

如果后续代码将 `control_mode` 改为 `1`：

- 气泵由 `pc_dn_data.pc_pump_state` 控制。
- 头部角度由 `pc_dn_data.pc_target_motor_angles[0/1]` 控制。
- 升降高度由 `pc_dn_data.pc_target_lift_height` 控制。
- 机械臂舵机角度由 `pc_dn_data.pc_target_servo_angles[0..5]` 直接写入 `motor_radians[0..5]`。
- PC 机械臂控制会被机械臂禁用状态拦截：`arm_disable_active == 0U` 时才执行。

