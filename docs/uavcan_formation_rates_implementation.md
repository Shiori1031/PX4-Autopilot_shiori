# UAVCAN 编队速率控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队速率控制功能，用于三机刚性连接编队飞行系统。中央主飞控读取遥控器输入，计算左右从飞控的角速率与推力指令，通过 UAVCAN 总线广播；左右从飞控按自身编队位置筛选消息，在 Offboard body-rate 模式下直接执行。

当前实现面向 `cuav_7-nano_default`，相关功能已经通过板级配置编进固件。

核心机制如下：

```text
主机 roll 输入
  -> 左右从机 pitch_rate 反向变化
  -> 左右机升力产生差异
  -> 编队整体获得滚转力矩
```

## 2. 当前实际使用的文件

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 主机发送器 | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | 订阅 `manual_control_setpoint`，内部解算左右从机速率指令，通过 UAVCAN 广播 |
| UAVCAN 接收器 | `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}` | 接收编队 `ArrayCommand`，直接发布 `offboard_control_mode` 和 `vehicle_rates_setpoint` |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c` | 定义当前实际生效的编队参数 |
| 启动接入 | `src/drivers/uavcan/uavcan_main.cpp`、`src/drivers/uavcan/sensors/sensor_bridge.cpp` | 根据参数和板级开关自动初始化发送器/接收器 |
| 板级配置 | `boards/cuav/7-nano/default.px4board` | 开启 `CONFIG_UAVCAN_FORMATION_RATES_SENDER` 与 `CONFIG_UAVCAN_SENSOR_FORMATION_RATES` |

说明：此前预留的 `FormationRatesSetpoint.msg` 已不再使用，当前主从链路统一走 UAVCAN `uavcan::equipment::actuator::ArrayCommand`。

## 3. 传输格式

### 3.1 UAVCAN 消息载体

复用标准 `uavcan::equipment::actuator::ArrayCommand`，通过自定义 `actuator_id` 编码数据：

| Actuator ID | 含义 | 类型 | 当前发送端是否使用 |
| ----------- | ---- | ---- | ------------------ |
| 100 | Roll command / roll rate command tx | `COMMAND_TYPE_SPEED` | 是 |
| 101 | Pitch rate | `COMMAND_TYPE_SPEED` | 是 |
| 102 | Yaw rate | `COMMAND_TYPE_SPEED` | 是 |
| 103 | Thrust_X | `COMMAND_TYPE_UNITLESS` | 是 |
| 104 | Thrust_Y | `COMMAND_TYPE_UNITLESS` | 否，接收端保留支持 |
| 105 | Thrust_Z | `COMMAND_TYPE_UNITLESS` | 否，接收端保留支持 |
| 110 | Formation position (`1=左`, `2=右`) | `COMMAND_TYPE_UNITLESS` | 是 |

发送频率为 200 Hz。

### 3.2 路由方式

当前实现不是按节点 ID 单播，而是：

```text
主机广播左右两帧 ArrayCommand
  -> 每个从机接收
  -> 按 FORM_POSITION 与消息里的 formation_position 匹配
  -> 只有匹配的一侧执行
```

因此，真正决定左右机分流的是 `formation_position` 字段和从机本地参数 `FORM_POSITION`。

## 4. 控制映射关系

机械结构前提：三机通过刚性连杆连接，连接处允许相对俯仰，但不允许相对滚转。

因此：
- 从机通过 pitch 通道承担编队滚转的主要控制作用。
- 从机 roll 通道不直接照搬主机姿态，而是在接收端结合从机自身 roll 做闭环补偿。
- 主机 pitch 用于三机同步俯仰。
- 主机 yaw 用于差速推力与协调偏航。

### 4.1 左机完整指令

```text
roll_cmd_tx  = manual.roll
roll_rate_sp = FORM_ROLL_COMP * (roll_cmd_tx - self_roll)    // 接收端执行
pitch_rate   = manual.roll * FORM_R2P_GAIN + manual.pitch * FORM_PITCH_SYNC
yaw_rate     = manual.yaw  + (-manual.roll * FORM_YAW_K)
thrust_x     = (manual.throttle + 1) / 2 + max(manual.yaw, 0) * FORM_THR_DIFF
```

### 4.2 右机完整指令

```text
roll_cmd_tx  = manual.roll
roll_rate_sp = FORM_ROLL_COMP * (roll_cmd_tx - self_roll)    // 接收端执行
pitch_rate   = -manual.roll * FORM_R2P_GAIN + manual.pitch * FORM_PITCH_SYNC
yaw_rate     = manual.yaw  + (manual.roll * FORM_YAW_K)
thrust_x     = (manual.throttle + 1) / 2 + max(-manual.yaw, 0) * FORM_THR_DIFF
```

### 4.3 物理意义

- 主机向右滚：左机抬头、右机低头，左右升力差使编队右滚。
- 主机向左滚：左机低头、右机抬头，左右升力差使编队左滚。
- 主机俯仰输入：左右机同向叠加到 `pitch_rate`，用于三机同步抬头/低头。
- 主机偏航输入：外侧机增加推力，同时叠加协调偏航项，降低侧滑。

## 5. 代码流程

### 5.1 主机端：formation_rates_sender

初始化时：
- 查找 `FORM_R2P_GAIN`、`FORM_YAW_K`、`FORM_THR_DIFF`、`FORM_PITCH_SYNC`
- 启动 200 Hz 定时器

周期执行时：
1. 读取 `manual_control_setpoint`
2. 如果遥控器数据超过 500 ms 未更新，则停止发送
3. 读取 `vehicle_status`
4. 仅在固定翼模式或过渡模式下继续工作
5. 计算左机指令并广播
6. 计算右机指令并广播

### 5.2 从机端：formation_rates

初始化时：
- 查找 `FORM_FOLLOWER_EN`、`FORM_POSITION`、`FORM_ROLL_COMP`
- 注册 `ArrayCommand` 订阅回调

接收回调时：
1. 检查 `FORM_FOLLOWER_EN`，未开启则直接返回
2. 读取 `vehicle_attitude`，提取从机自身 `self_roll`
3. 遍历消息中的各个 `Command`，按 `actuator_id` 解码
4. 检查消息中的 `formation_position` 是否与本机 `FORM_POSITION` 一致
5. 发布 `offboard_control_mode`，其中 `body_rate=true`
6. 发布 `vehicle_rates_setpoint`
7. 由固定翼速率控制器继续完成执行

### 5.3 下游控制链

```text
FormationRatesBridge
  -> offboard_control_mode(body_rate=true)
  -> vehicle_rates_setpoint
  -> Commander 进入/维持 Offboard body-rate 控制链
  -> FixedwingRateControl 执行角速率控制
```

Offboard 丢失保护由 PX4 原生参数 `COM_OF_LOSS_T` 控制，当前实现没有额外的编队私有超时参数。

## 6. 实际生效参数

### 6.1 主机参数（中央机，Node ID=1）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 本机节点 ID，主机固定为 `1` |
| `UAVCAN_PUB_FORM` | 发送器使能，设为 `1` |
| `FORM_R2P_GAIN` | Roll 到 Pitch 的核心映射增益 |
| `FORM_YAW_K` | 偏航耦合系数 |
| `FORM_THR_DIFF` | 差速推力系数 |
| `FORM_PITCH_SYNC` | 俯仰同步系数 |

### 6.2 从机参数（左/右机）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 左机为 `2`，右机为 `3` |
| `UAVCAN_SUB_FORM` | 接收器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 从机内部处理开关，设为 `1` |
| `FORM_POSITION` | 左机设 `1`，右机设 `2` |
| `FORM_ROLL_COMP` | 从机 roll 补偿增益 |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 推荐配置示例

主机：

```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_FORM 1
param set FORM_R2P_GAIN 2.0
param set FORM_YAW_K 0.3
param set FORM_THR_DIFF 0.05
param set FORM_PITCH_SYNC 0.1
param save
reboot
```

左机：

```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 2
param set UAVCAN_SUB_FORM 1
param set FORM_FOLLOWER_EN 1
param set FORM_POSITION 1
param set FORM_ROLL_COMP 2.0
param set COM_OF_LOSS_T 1.0
param save
reboot
```

右机：

```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 3
param set UAVCAN_SUB_FORM 1
param set FORM_FOLLOWER_EN 1
param set FORM_POSITION 2
param set FORM_ROLL_COMP 2.0
param set COM_OF_LOSS_T 1.0
param save
reboot
```

从机进入 Offboard 后，接收器会持续发送 body-rate 控制心跳。

## 7. 7-nano 相关接入

`cuav_7-nano_default` 已开启以下板级配置：

```text
CONFIG_UAVCAN_FORMATION_RATES_SENDER=y
CONFIG_UAVCAN_SENSOR_FORMATION_RATES=y
```

因此只要参数打开，UAVCAN 驱动启动后会自动完成发送器和接收器初始化，不需要额外手动启动独立模块。

## 8. 验证方法

### 8.1 查看 UAVCAN 状态

```bash
uavcan status
```

预期可见主机与左右从机节点在线。

### 8.2 主机侧检查

```bash
uavcan status
```

确认 UAVCAN 驱动正常工作，主机进入固定翼模式后会按 200 Hz 广播编队指令。

### 8.3 从机侧检查接收与执行

```bash
listener offboard_control_mode
listener vehicle_rates_setpoint
listener vehicle_status
```

预期现象：
- `offboard_control_mode.body_rate = true`
- `vehicle_rates_setpoint` 中的 `pitch` 随主机 `roll` 产生左右相反变化
- `vehicle_status.nav_state` 为 Offboard

### 8.4 地面联调建议

1. 先只接通一侧从机，验证 `FORM_POSITION` 是否正确。
2. 移动主机横滚杆，确认左右从机升降舵响应方向相反。
3. 再验证俯仰同步和偏航差速是否符合预期。
4. 最后再进行三机联动测试。

## 9. 当前实现边界

当前实现刻意保持精简，只保留已接入并实际使用的内容：
- 不再使用额外的 `FormationRatesSetpoint` uORB 消息。
- 不再保留未落地的主机使能预留参数。
- 不再保留未实际执行的私有 `FORM_TIMEOUT` 逻辑。
- 左右机分流完全依赖 `formation_position` 与 `FORM_POSITION`，而不是额外的节点 ID 参数。
