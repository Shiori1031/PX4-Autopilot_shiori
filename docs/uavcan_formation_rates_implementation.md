# UAVCAN 编队速率控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队速率控制功能，用于三机刚性连接编队飞行系统。中央主飞控读取遥控器输入，不再在主机上拆解左右从机指令，而是通过一条自定义 DroneCAN 消息广播 `throttle / yaw / roll / pitch / flags`；左右从飞控按各自 `FORM_POSITION` 在本地完成控制解算，然后在 Offboard body-rate 模式下直接执行。

当前实现面向 `cuav_7-nano_default`，相关功能已经通过板级配置编进固件。

核心机制如下：

```text
主机遥控器四通道输入
  -> 广播一条 ControlInput
  -> 左右从机分别按 FORM_POSITION 本地解算
  -> 发布 offboard_control_mode + vehicle_rates_setpoint
```

## 2. 当前实际使用的文件

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 自定义消息 | `src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20040.ControlInput.uavcan` | 定义编队控制广播消息 `throttle/yaw/roll/pitch/flags` |
| 主机发送器 | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | 订阅 `manual_control_setpoint`，广播 `ControlInput` |
| UAVCAN 接收器 | `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}` | 接收 `ControlInput`，在从机本地完成控制解算并发布 `offboard_control_mode` 和 `vehicle_rates_setpoint` |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c` | 定义当前实际生效的编队参数 |
| 启动接入 | `src/drivers/uavcan/uavcan_main.cpp`、`src/drivers/uavcan/sensors/sensor_bridge.cpp` | 根据参数和板级开关自动初始化发送器/接收器 |
| 板级配置 | `boards/cuav/7-nano/default.px4board` | 开启 `CONFIG_UAVCAN_FORMATION_RATES_SENDER` 与 `CONFIG_UAVCAN_SENSOR_FORMATION_RATES` |

## 3. 传输格式

### 3.1 UAVCAN 消息载体

当前实现使用自定义 DroneCAN 消息 `dronecan::formation::ControlInput`，字段如下：

| 字段 | 类型 | 含义 | 取值范围 |
| ---- | ---- | ---- | ---- |
| `throttle` | `float16` | 主机油门输入 | `[-1, 1]` |
| `yaw` | `float16` | 主机偏航输入 | `[-1, 1]` |
| `roll` | `float16` | 主机滚转输入 | `[-1, 1]` |
| `pitch` | `float16` | 主机俯仰输入 | `[-1, 1]` |
| `flags` | `uint8` | 状态位 | 位掩码 |

发送频率为 300 Hz。
```c++
src/drivers/uavcan/formation_rates_sender.hpp
static constexpr unsigned MAX_RATE_HZ = 300; // 发布频率上限
```

### 3.2 取值范围来源

主机发送端读取的是 PX4 的 `manual_control_setpoint`。这个主题本身就约定 `roll/pitch/yaw/throttle` 为 `[-1, 1]` 归一化量，因此：

```cpp
src/drivers/uavcan/formation_rates_sender.cpp
_manual_sub.copy(&manual);
```
这里的 `copy()` 只是把当前 uORB 数据拷贝出来，不会在这里重新归一化；如果上游是 PX4 标准遥控器输入链路，那么拷出来时已经是 `[-1, 1]`。发送端里再用 `math::constrain(..., -1.0f, 1.0f)`，只是做一层保险限制。

### 3.3 传输方式

```text
主机广播一条 ControlInput
  -> 左右从机都接收
  -> 每个从机按本机 FORM_POSITION 判断自己是左机还是右机
  -> 用同一份代码、本地按符号完成左右差异化解算
```

因此，左右从机分流依赖的是本地参数 `FORM_POSITION`，不是消息里的额外位置字段。

## 4. 控制映射关系

机械结构前提：三机通过刚性连杆连接，连接处允许相对俯仰，但不允许相对滚转。

因此：
- 从机通过 `pitch` 通道承担编队滚转的主要控制作用。
- 从机 `roll` 通道用于自稳和滚转补偿。
- 主机 `pitch` 用于三机同步俯仰，同时从机保留自身 pitch 自稳补偿。
- 主机 `yaw` 用于左右从机同向偏航，同时从机保留自身 yaw 阻尼补偿，外侧从机增加油门。

### 4.1 从机本地解算

设：
- 左机 `side_sign = +1`
- 右机 `side_sign = -1`

则从机本地解算为：

```text
roll_rate_sp = FORM_ROLL_COMP * (manual.roll - self_roll)
pitch_rate   = side_sign * manual.roll * FORM_R2P_GAIN
             + manual.pitch * FORM_PITCH_SYNC
             - self_pitch * FORM_PITCH_COMP
yaw_rate     = manual.yaw * FORM_YAW_SYNC - self_yaw_rate * FORM_YAW_COMP
base_thrust  = (manual.throttle + 1) / 2
outer_boost  = left ? max(manual.yaw, 0) : max(-manual.yaw, 0)
thrust_x     = base_thrust + outer_boost * FORM_YAW_K
```

### 4.2 物理意义

- 主机向右滚：左机抬头、右机低头，左右升力差使编队右滚。
- 主机向左滚：左机低头、右机抬头，左右升力差使编队左滚。
- 主机俯仰输入：左右机同向叠加到 `pitch_rate`，用于三机同步抬头/低头。
- 主机右偏航：左右从机方向舵同向右偏，左机作为外侧机增加油门。
- 主机左偏航：左右从机方向舵同向左偏，右机作为外侧机增加油门。
- 主机无明显滚转输入时：`FORM_ROLL_COMP` 仍会利用 `self_roll` 做自稳补偿。

## 5. 代码流程

### 5.1 主机端：formation_rates_sender

初始化时：
- 启动 300 Hz 定时器

周期执行时：
1. 读取 `manual_control_setpoint`
2. 如果遥控器数据超过 500 ms 未更新，或 `valid=false`，则停止发送
3. 检查四个输入是否为有限值
4. 读取 `vehicle_status`
5. 仅在固定翼模式或过渡模式下继续工作
6. 将 `throttle/yaw/roll/pitch` 限幅到 `[-1, 1]`
7. 打包 `flags` 并广播单条 `ControlInput`

### 5.2 从机端：formation_rates

初始化时：
- 查找 `FORM_FOLLOWER_EN`、`FORM_POSITION`、`FORM_R2P_GAIN`、`FORM_YAW_K`、`FORM_PITCH_SYNC`、`FORM_PITCH_COMP`、`FORM_YAW_SYNC`、`FORM_YAW_COMP`、`FORM_ROLL_COMP`
- 注册 `ControlInput` 订阅回调

接收回调时：
1. 检查 `FORM_FOLLOWER_EN`，未开启则直接返回
2. 检查 `flags` 里的 `FLAG_VALID`
3. 检查 `FORM_POSITION` 是否为左机或右机
4. 读取 `vehicle_attitude` 和 `vehicle_angular_velocity`，提取从机自身姿态与偏航角速度
5. 按 `FORM_POSITION` 计算 `side_sign` 和外侧机油门增量
6. 发布 `offboard_control_mode`，其中 `body_rate=true`
7. 发布 `vehicle_rates_setpoint`
8. 由固定翼速率控制器继续完成执行

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

### 6.2 从机参数（左/右机）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 左机为 `2`，右机为 `3` |
| `UAVCAN_SUB_FORM` | 接收器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 从机内部处理开关，设为 `1` |
| `FORM_POSITION` | 左机设 `1`，右机设 `2` |
| `FORM_R2P_GAIN` | 滚转到俯仰映射增益 |
| `FORM_YAW_K` | 偏航时外侧从机油门增益 |
| `FORM_PITCH_SYNC` | 俯仰同步系数 |
| `FORM_PITCH_COMP` | pitch 自稳补偿系数 |
| `FORM_YAW_SYNC` | 偏航同步系数 |
| `FORM_YAW_COMP` | yaw 自稳阻尼系数 |
| `FORM_ROLL_COMP` | roll 自稳补偿系数 |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 推荐配置示例

主机：

```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_FORM 1
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
param set FORM_R2P_GAIN 2.0
param set FORM_YAW_K 0.3
param set FORM_PITCH_SYNC 0.1
param set FORM_PITCH_COMP 1.0
param set FORM_YAW_SYNC 1.0
param set FORM_YAW_COMP 0.3
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
param set FORM_R2P_GAIN 2.0
param set FORM_YAW_K 0.3
param set FORM_PITCH_SYNC 0.1
param set FORM_PITCH_COMP 1.0
param set FORM_YAW_SYNC 1.0
param set FORM_YAW_COMP 0.3
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

## 8. 当前版本已经清理掉的旧内容

- 不再使用 `uavcan::equipment::actuator::ArrayCommand` 进行编队控制传输。
- 不再通过消息中的 `formation_position` 做左右机分流。
- 不再保留 `FORM_THR_DIFF` 参数。
- 不再保留额外的 `FormationRatesSetpoint` 中间消息。
