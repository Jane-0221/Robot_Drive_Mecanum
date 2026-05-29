# VL53L0X I2C 手臂测距模块说明

本文档记录工程里原先那一路 IIC/I2C 手臂测距模块的写法和驱动方式。当前代码已经按清理要求移除了业务接入，只保留驱动文件：

- `User/Drive/vl53l0x.c`
- `User/Drive/vl53l0x.h`

也就是说，驱动源码仍会参与编译，但系统启动和任务循环里已经不会主动初始化或读取这个测距模块。

## 硬件与工程配置

这一路测距模块使用 `VL53L0X`，挂在 `I2C1` 上：

- `PB8`：`I2C1_SCL`
- `PB9`：`I2C1_SDA`
- CubeMX 生成初始化函数：`MX_I2C1_Init()`
- I2C 句柄：`hi2c1`

地址定义在 `User/Drive/vl53l0x.h`：

```c
#define VL53L0X_I2C_DEV_ADDR    0x52U
#define VL53L0X_I2C_HAL_ADDR    VL53L0X_I2C_DEV_ADDR
#define VL53L0X_I2C_SHIFT_ADDR  (VL53L0X_I2C_DEV_ADDR << 1)
```

这里 `0x52` 是传给 STM32 HAL 的 8 位地址，等价于常见 7 位地址 `0x29 << 1`。`VL53L0X_I2C_SHIFT_ADDR` 是调试时用来探测“是否误左移两次”的地址。

## 原来的接入方式

旧版本在 `Core/Src/main.c` 中初始化 I2C1：

```c
MX_I2C2_Init();
MX_I2C1_Init();
```

其中 `I2C2` 给其他设备使用，`I2C1` 是这一路 VL53L0X 手臂测距。

旧版本在 `Core/Src/freertos.c` 中 include 驱动头文件：

```c
#include "vl53l0x.h"
```

并定义了开关和启动软探测参数：

```c
#define VL53L0X_COMM_ENABLE 1U
#define VL53L0X_SOFT_PROBE_ON_BOOT 0U
#define VL53L0X_SOFT_PROBE_DELAY_MS 500U
```

原来驱动逻辑挂在 `Arm_MT_Task()` 里。任务启动时先计算读取周期：

```c
uint32_t vl53l0x_last_tick = osKernelGetTickCount();
uint32_t vl53l0x_period_ticks = osKernelGetTickFreq() / 100U;
```

`osKernelGetTickFreq() / 100U` 表示 100Hz，也就是约 10ms 读一次。然后可选做软件 I2C 探测，最后初始化 VL53L0X：

```c
#if VL53L0X_SOFT_PROBE_ON_BOOT
osDelay(VL53L0X_SOFT_PROBE_DELAY_MS);
(void)VL53L0X_SoftProbeI2C1();
#endif

(void)VL53L0X_Init();
```

循环里按周期读取距离：

```c
uint32_t tick_now = osKernelGetTickCount();
if ((tick_now - vl53l0x_last_tick) >= vl53l0x_period_ticks)
{
    vl53l0x_last_tick = tick_now;
    (void)VL53L0X_ReadDistance();
}
```

读取结果保存在驱动全局变量：

```c
volatile uint16_t vl53l0x_distance_mm;
```

注意：这一路 I2C VL53L0X 原先主要更新 `vl53l0x_distance_mm` 和一组调试变量。之前上行协议里的 `arm_distance` 不是从 `vl53l0x_distance_mm` 直接打包出去的，而是另一条 UART8/STP23L 手臂测距链路维护的 `arm_distance_final`。

## 驱动读取流程

核心读取函数是：

```c
VL53L0X_StatusTypeDef VL53L0X_ReadDistance(void);
```

它的流程如下：

1. 向寄存器 `0x00` 写启动命令 `0x01`。
2. 轮询结果寄存器 `0x14`，等待 bit0 置位。
3. 再写一次结果寄存器地址 `0x14`。
4. 从设备连续读取 `12` 字节结果。
5. 从结果缓冲区第 10、11 字节组合距离值。
6. 距离值减去固定补偿 `100mm`，写入 `vl53l0x_distance_mm`。

相关定义在 `vl53l0x.c`：

```c
#define VL53L0X_REG_START        0x00U
#define VL53L0X_REG_RESULT       0x14U
#define VL53L0X_START_CMD        0x01U
#define VL53L0X_RESULT_READY_BIT 0x01U
#define VL53L0X_DISTANCE_OFFSET_MM 100U
```

距离计算逻辑：

```c
distance = ((uint16_t)read_buf[10] << 8) | read_buf[11];
vl53l0x_distance_mm = (distance > VL53L0X_DISTANCE_OFFSET_MM) ?
                       (uint16_t)(distance - VL53L0X_DISTANCE_OFFSET_MM) :
                       0U;
```

测距状态也会记录：

```c
vl53l0x_range_status = (uint8_t)((read_buf[0] & 0x78U) >> 3);
```

## 初始化与调试

初始化函数：

```c
VL53L0X_StatusTypeDef VL53L0X_Init(void);
```

主要做三件事：

1. 调用 `VL53L0X_UpdateBusDebug()` 采集 I2C 总线状态。
2. 如果发现 `I2C_FLAG_BUSY`，调用 `VL53L0X_RecoverI2C1()` 恢复总线。
3. 扫描 I2C1，并用 `HAL_I2C_IsDeviceReady()` 确认 `0x52` 地址设备存在。

驱动里保留了一组调试变量，方便在线观察：

- `vl53l0x_status`：驱动状态，`OK/ERROR/NOT_FOUND/TIMEOUT`
- `vl53l0x_error_step`：出错步骤，例如启动写、ready 轮询、结果读取
- `vl53l0x_last_hal_status`：最近一次 HAL 返回值
- `vl53l0x_last_hal_error`：最近一次 HAL I2C error
- `vl53l0x_scan_count` / `vl53l0x_scan_addr[]`：I2C 扫描结果
- `vl53l0x_scl_level` / `vl53l0x_sda_level`：PB8/PB9 电平
- `vl53l0x_i2c_state` / `vl53l0x_i2c_busy_flag`：HAL I2C 状态和 BUSY 标志
- `vl53l0x_range_status`：VL53L0X 返回的测距状态位

## 总线恢复和软探测

`VL53L0X_RecoverI2C1()` 用于 I2C1 BUSY 或 SDA 被拉低时恢复总线。做法是：

1. `HAL_I2C_DeInit(&hi2c1)`
2. 把 PB8/PB9 临时配置成开漏 GPIO 输出
3. 拉动 SCL 9 个周期，尝试释放从设备
4. 手动产生 STOP 条件
5. 调用 `MX_I2C1_Init()` 恢复硬件 I2C
6. 更新调试变量

`VL53L0X_SoftProbeI2C1()` 用 GPIO 模拟 I2C 起始、地址发送和停止，用于确认设备是否 ACK。旧代码里默认不开启启动软探测：

```c
#define VL53L0X_SOFT_PROBE_ON_BOOT 0U
```

如果调试硬件连线或地址问题，可以临时打开它。

## 当前若要重新启用

当前工程已经清掉业务接入。如果以后要恢复这一路 VL53L0X，需要至少做这些事：

1. 在 `Core/Src/main.c` 恢复 `MX_I2C1_Init()`，并确保它在 `VL53L0X_Init()` 之前执行。
2. 在需要的任务文件中 include：

```c
#include "vl53l0x.h"
```

3. 启动时调用：

```c
(void)VL53L0X_Init();
```

4. 周期调用：

```c
(void)VL53L0X_ReadDistance();
```

5. 从 `vl53l0x_distance_mm` 读取距离，单位是 mm。

建议不要把读取放在高频控制关键路径里，因为 `VL53L0X_ReadDistance()` 使用阻塞式 HAL I2C，并且 ready 轮询最长可能等到 `100ms`。

## 当前清理后的状态

目前保留：

- `User/Drive/vl53l0x.c/.h`
- `Core/Src/i2c.c` 和 `Core/Inc/i2c.h` 里的 `I2C1` 生成代码
- `Makefile` 中的 `User/Drive/vl53l0x.c`

目前移除：

- `main.c` 中的 `MX_I2C1_Init()`
- `freertos.c` 中的 `vl53l0x.h`
- `freertos.c` 中的 `VL53L0X_Init()`
- `freertos.c` 中的周期 `VL53L0X_ReadDistance()`

所以当前固件不会主动驱动这个 I2C 手臂测距模块，但以后需要时可以直接按上面的流程接回。
