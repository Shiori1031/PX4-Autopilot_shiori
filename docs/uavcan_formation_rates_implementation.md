# UAVCAN 编队速率控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队速率控制功能，用于三机刚性连接编队飞行系统。中央主飞控读取遥控器输入，同时广播主机实际姿态角与机体系角速度；左右从飞控按各自 `FORM_POSITION` 在本地完成控制解算，然后在 Offboard body-rate 模式下执行。

当前实现面向 `make cuav_7-nano_default`，相关功能已经通过板级配置编进固件。

核心机制：

```text
从机先保持自身原有飞行模式（如自稳/自主）
  -> 持续接收主机 CAN 指令并发布 offboard_control_mode 心跳
  -> 只有当从机被人工切入 Offboard 后
  -> 才开始发布 vehicle_rates_setpoint，由 CAN 链路接管控制权
```

## 2. 当前实际使用的文件

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 自定义消息 | `src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20040.ControlInput.uavcan` | 定义编队控制广播消息 `throttle/yaw/roll/pitch/att_roll/att_pitch/att_yaw/p/q/r/flags` |
| 主机发送器 | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | 订阅 `manual_control_setpoint`、`vehicle_attitude`、`vehicle_angular_velocity`，广播 `ControlInput` |
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
| `att_roll` | `float16` | 主机滚转姿态角 | `rad` |
| `att_pitch` | `float16` | 主机俯仰姿态角 | `rad` |
| `att_yaw` | `float16` | 主机偏航姿态角 | `rad` |
| `p` | `float16` | 主机机体系 x 轴角速度 | `rad/s` |
| `q` | `float16` | 主机机体系 y 轴角速度 | `rad/s` |
| `r` | `float16` | 主机机体系 z 轴角速度 | `rad/s` |
| `flags` | `uint8` | 状态位 | 位掩码 |

当前发送频率为 300 Hz：

```cpp
// src/drivers/uavcan/formation_rates_sender.hpp
static constexpr unsigned MAX_RATE_HZ = 300;
```

### 3.2 取值范围来源

主机发送端读取的是 PX4 的 `manual_control_setpoint`。这个主题本身约定 `roll/pitch/yaw/throttle` 为 `[-1, 1]` 归一化量，因此：

```cpp
// src/drivers/uavcan/formation_rates_sender.cpp
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

## 4. 当前控制映射关系

控制方式: 以“上一版遥控器直接控制逻辑”为主控制通道，同时叠加“主机姿态/角速度显式前馈 + 相对姿态误差反馈 + 相对速率误差阻尼”作为辅助修正通道

因此：
- 从机 `roll / pitch / yaw` 速率设定值由两部分共同决定，但当前以遥控器直接控制部分为主：
  1. 主机实际姿态角、主机 `p/q/r`、从机自身姿态角和从机自身角速度构成的相对姿态辅助修正部分。
  2. 改为相对姿态 PID 控制前的上一版“遥控器直接控制”部分。
- `FORM_R2P_GAIN` 当前承担 `roll -> pitch` 的耦合作用，在遥控器主控支路中把主机滚转输入映射为左右从机反向俯仰。
- `FORM_YAW_K` 当前同时承担两部分作用：从机侧偏航时外侧机附加油门，以及主机侧偏航时同步加速。
- 从机控制链只有在自身飞行模式已经切入 Offboard 时才真正接管。

### 4.1 从机本地解算

- 左机 `side_sign = +1`
- 右机 `side_sign = -1`

定义：
- 主机姿态角：`leader_roll / leader_pitch / leader_yaw`
- 主机角速度：`leader_p / leader_q / leader_r`
- 从机姿态角：`self_roll / self_pitch / self_yaw`
- 从机角速度：`self_roll_rate / self_pitch_rate / self_yaw_rate`

期望相对姿态为：（初始安装角度，直接设置为0）

```text
roll_rel_des  = FORM_ROLL_OFS
pitch_rel_des = FORM_PITCH_OFS * side_sign
yaw_rel_des   = FORM_YAW_OFS
```

相对误差为：

```text
e_roll  = (leader_roll  + roll_rel_des)  - self_roll
e_pitch = (leader_pitch + pitch_rel_des) - self_pitch
e_yaw   = wrap_pi((leader_yaw + yaw_rel_des) - self_yaw)
```

从机速率设定值先分别计算两部分：

```text
relative_roll_corr  = FORM_ROLL_FF * leader_p
                    + FORM_ROLL_KP * e_roll
                    + FORM_ROLL_KD * (leader_p - self_roll_rate)

relative_pitch_corr = FORM_PITCH_FF * leader_q
                    + FORM_PITCH_KP * e_pitch
                    + FORM_PITCH_KD * (leader_q - self_pitch_rate)

relative_yaw_corr   = FORM_YAW_FF * leader_r
                    + FORM_YAW_KP * e_yaw
                    + FORM_YAW_KD * (leader_r - self_yaw_rate)

stick_roll_sp     = FORM_ROLL_COMP * (roll - self_roll)
stick_pitch_sp    = side_sign * roll * FORM_R2P_GAIN
                  + pitch * FORM_PITCH_SYNC
                  - self_pitch * FORM_PITCH_COMP
stick_yaw_sp      = yaw * FORM_YAW_SYNC
                  - self_yaw_rate * FORM_YAW_COMP

roll_rate_sp      = 1.0 * stick_roll_sp  + 0.3  * relative_roll_corr
pitch_rate_sp     = 1.0 * stick_pitch_sp + 0.15 * relative_pitch_corr
yaw_rate_sp       = 1.0 * stick_yaw_sp   + 0.3  * relative_yaw_corr

base_thrust   = (throttle + 1) / 2
outer_boost   = left ? max(yaw, 0) : max(-yaw, 0)
thrust_x      = base_thrust + outer_boost * FORM_YAW_K
```

说明：
- 当前三轴辅助修正权重是代码内固定常量，未做成参数。
- 当前实现中 `roll/yaw` 辅助权重为 `0.3`，`pitch` 辅助权重为 `0.15`。
- `FORM_ROLL_KD / FORM_PITCH_KD / FORM_YAW_KD` 默认值当前均为 `0.0`，也就是默认保留 D 项代码但实际不参与控制。

主机侧还会在固定翼速率控制输出阶段增加一项偏航同步加速：

```text
master_thrust = thrust_body[0] + 0.5 * abs(manual.yaw) * FORM_YAW_K
```

其中这项逻辑只在 `FORM_FOLLOWER_EN == 0` 且 `UAVCAN_PUB_FORM == 1` 时生效，也就是主机身份并实际开启编队广播时生效；从机身份下不在 `FixedwingRateControl.cpp` 里执行这一步。

随后对 `roll / pitch / yaw` 速率设定值进行限幅：

```text
roll  ∈ [-FORM_ROLL_RMAX, +FORM_ROLL_RMAX]
pitch ∈ [-FORM_PTCH_RMAX, +FORM_PTCH_RMAX]
yaw   ∈ [-FORM_YAW_RMAX,  +FORM_YAW_RMAX]
```

### 4.2 物理意义

- 主机向右滚时，左右从机会根据主机真实滚转姿态与从机自身滚转姿态的相对误差生成辅助滚转修正；同时 `FORM_R2P_GAIN` 会在遥控器主控支路里让左机抬头、右机低头，形成整体右滚。
- 主机向左滚时，左右从机相反动作，形成整体左滚。
- 主机 `p/q/r` 通过 `FORM_ROLL_FF / FORM_PITCH_FF / FORM_YAW_FF` 显式进入前馈项，提高从机跟随速度。
- 主机与从机的姿态差、角速度差分别通过 `KP / KD` 项形成反馈与阻尼，其中 `KD` 作用在相对速率误差上。
- 上一版“遥控器直接控制”部分仍作为主控通道保留；当前从机控制不是纯相对姿态控制，而是“遥控器主控 + 相对姿态辅助修正”的叠加结构。
- 当前辅助修正支路中，`pitch` 轴权重最小，为 `0.15`；`roll/yaw` 辅助权重为 `0.3`，因此编队修正对俯仰的介入最弱。
- 主机右偏航时，左右从机方向舵同向右偏，左机作为外侧机增加油门。
- 主机左偏航时，左右从机方向舵同向左偏，右机作为外侧机增加油门。
- 主机自身在左右偏航时也会按 `0.5 * abs(manual.yaw) * FORM_YAW_K` 同步增加油门，用于三机整体偏航阶段的共同推力补偿。
- 如果 `FORM_ROLL_OFS / FORM_PITCH_OFS / FORM_YAW_OFS` 全设为 `0`，则代表默认没有相对安装偏角或编队姿态偏角。

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
7. 读取主机 `vehicle_attitude` 和 `vehicle_angular_velocity`
8. 将主机欧拉角 `att_roll/att_pitch/att_yaw` 与 `p/q/r` 一并打包到 `ControlInput`
9. 设置 `flags` 并广播单条 `ControlInput`

### 5.2 从机端：formation_rates

初始化时：
- 查找 `FORM_FOLLOWER_EN`、`FORM_POSITION`、`FORM_R2P_GAIN`、`FORM_YAW_K`
- 查找相对姿态参数 `FORM_ROLL_OFS / FORM_PITCH_OFS / FORM_YAW_OFS`
- 查找前馈/反馈阻尼参数 `FORM_ROLL_FF / KP / KD`、`FORM_PITCH_FF / KP / KD`、`FORM_YAW_FF / KP / KD`
- 查找限幅参数 `FORM_ROLL_RMAX / FORM_PTCH_RMAX / FORM_YAW_RMAX`
- 注册 `ControlInput` 订阅回调

接收回调时：
1. 仅在 `parameter_update` 到达时刷新参数
2. 检查 `FORM_FOLLOWER_EN`，未开启则直接返回
3. 检查 `flags` 里的 `FLAG_VALID`
4. 检查 `FORM_POSITION` 是否为左机或右机
5. 读取从机自身 `vehicle_attitude`、`vehicle_angular_velocity` 和 `vehicle_status`
6. 计算本机 `self_roll/self_pitch/self_yaw` 与 `self_roll_rate/self_pitch_rate/self_yaw_rate`
7. 计算 `side_sign` 与偏航外侧机油门增量
8. 发布 `offboard_control_mode`，其中 `body_rate=true`
9. 如果当前不是 Offboard，则直接退出这次回调，不发布 `vehicle_rates_setpoint`
10. 如果当前已经是 Offboard，则继续按“遥控器主控 + 相对姿态辅助修正”逻辑生成 `vehicle_rates_setpoint`
11. 由固定翼速率控制器继续完成执行

### 5.3 下游控制链

```text
FormationRatesBridge
  -> offboard_control_mode(body_rate=true)
  -> vehicle_rates_setpoint
  -> Commander 进入/维持 Offboard body-rate 控制链
  -> FixedwingRateControl 执行角速率控制
  -> 主机在 FixedwingRateControl 内按 manual.yaw 叠加偏航同步加速
```

Offboard 丢失保护由 PX4 原生参数 `COM_OF_LOSS_T` 控制，当前实现没有额外的编队私有超时参数。

## 6. 实际生效参数

### 6.1 主机参数（中央机，Node ID=1）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 本机节点 ID，主机固定为 `1` |
| `UAVCAN_PUB_FORM` | 发送器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 主机应设为 `0`，用于区分主机/从机身份 |
| `FORM_YAW_K` | 主机偏航同步加速系数，同时也是从机外侧偏航增油系数 |

### 6.2 从机参数（左/右机）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 左机为 `2`，右机为 `3` |
| `UAVCAN_SUB_FORM` | 接收器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 从机内部处理开关，设为 `1` |
| `FORM_POSITION` | 左机设 `1`，右机设 `2` |
| `FORM_R2P_GAIN` | 遥控器滚转输入到俯仰通道的耦合增益 |
| `FORM_YAW_K` | 偏航时外侧从机油门增益 |
| `FORM_ROLL_OFS` | 相对滚转偏角 |
| `FORM_PITCH_OFS` | 相对俯仰偏角幅值，左右机内部自动取反 |
| `FORM_YAW_OFS` | 相对偏航偏角 |
| `FORM_ROLL_FF` | 滚转 leader 角速度显式前馈增益 |
| `FORM_ROLL_KP` | 滚转相对姿态反馈比例增益 |
| `FORM_ROLL_KD` | 滚转相对速率阻尼增益 |
| `FORM_PITCH_FF` | 俯仰 leader 角速度显式前馈增益 |
| `FORM_PITCH_KP` | 俯仰相对姿态反馈比例增益 |
| `FORM_PITCH_KD` | 俯仰相对速率阻尼增益 |
| `FORM_YAW_FF` | 偏航 leader 角速度显式前馈增益 |
| `FORM_YAW_KP` | 偏航相对姿态反馈比例增益 |
| `FORM_YAW_KD` | 偏航相对速率阻尼增益 |
| `FORM_ROLL_RMAX` | 从机滚转角速度限幅 |
| `FORM_PTCH_RMAX` | 从机俯仰角速度限幅 |
| `FORM_YAW_RMAX` | 从机偏航角速度限幅 |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 预留叠加参数

当前代码里以下参数已经重新参与融合控制中的“上一版遥控器直接控制”部分：

| 参数 | 当前状态 |
| ---- | ---- |
| `FORM_PITCH_SYNC` | 当前已用于上一版遥控器直接控制中的俯仰同步项 |
| `FORM_PITCH_COMP` | 当前已用于上一版遥控器直接控制中的 pitch 自稳补偿 |
| `FORM_YAW_SYNC` | 当前已用于上一版遥控器直接控制中的 yaw 同步项 |
| `FORM_YAW_COMP` | 当前已用于上一版遥控器直接控制中的 yaw 阻尼补偿 |
| `FORM_ROLL_COMP` | 当前已用于上一版遥控器直接控制中的 roll 自稳补偿 |

### 6.4 推荐配置示例

主机：

```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_FORM 1
param set FORM_FOLLOWER_EN 0
param set FORM_YAW_K 0.3
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
param set FORM_ROLL_FF 0.5
param set FORM_PITCH_FF 0.5
param set FORM_YAW_FF 0.5
param set FORM_ROLL_OFS 0.0
param set FORM_PITCH_OFS 0.0
param set FORM_YAW_OFS 0.0
param set FORM_ROLL_KP 2.0
param set FORM_ROLL_KD 0.0
param set FORM_PITCH_KP 2.0
param set FORM_PITCH_KD 0.0
param set FORM_YAW_KP 2.0
param set FORM_YAW_KD 0.0
param set FORM_ROLL_RMAX 1.22
param set FORM_PTCH_RMAX 1.05
param set FORM_YAW_RMAX 0.87
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
param set FORM_ROLL_FF 0.5
param set FORM_PITCH_FF 0.5
param set FORM_YAW_FF 0.5
param set FORM_ROLL_OFS 0.0
param set FORM_PITCH_OFS 0.0
param set FORM_YAW_OFS 0.0
param set FORM_ROLL_KP 2.0
param set FORM_ROLL_KD 0.0
param set FORM_PITCH_KP 2.0
param set FORM_PITCH_KD 0.0
param set FORM_YAW_KP 2.0
param set FORM_YAW_KD 0.0
param set FORM_ROLL_RMAX 1.22
param set FORM_PTCH_RMAX 1.05
param set FORM_YAW_RMAX 0.87
param set COM_OF_LOSS_T 1.0
param save
reboot
```

从机进入 Offboard 后，接收器才会继续发布 `vehicle_rates_setpoint`，由 CAN 控制链正式接管。
主机不走从机的 Offboard 控制链，而是在 `FixedwingRateControl.cpp` 内直接对自身推力输出叠加偏航同步加速。

### 6.5 传输验证方法

可以用 PX4 shell 里的 `listener` 配合 `uavcan status` 做链路验证。
需要注意：`listener` 观察的是 PX4 内部 uORB 主题，不是直接抓原始 CAN 帧；因此它更适合验证“消息已经被正确接收并转换为控制量”。

主机侧建议先确认输入源正常：

```bash
listener manual_control_setpoint -r 5 -n 20
```

拨动遥控器时，应能看到 `throttle/yaw/roll/pitch` 持续变化，范围通常在 `[-1, 1]`。

从机侧建议依次检查：

```bash
uavcan status
listener offboard_control_mode -r 5 -n 20
```

判断标准：
- `uavcan status` 能看到主机节点在线，且总线没有明显错误累积。
- 主机正常发送时，`offboard_control_mode` 应持续更新，并看到 `body_rate = true`。
- 如果从机已经切入 Offboard，再执行：

```bash
listener vehicle_rates_setpoint -r 5 -n 20
```

此时应能看到 `roll/pitch/yaw/thrust_body[0]` 持续更新，并随主机姿态、主机 `p/q/r` 与从机 `FORM_POSITION` 发生对应变化。

如果 `listener vehicle_rates_setpoint` 没有更新，优先检查：
- 主机是否已开启 `UAVCAN_PUB_FORM=1`
- 从机是否已开启 `UAVCAN_SUB_FORM=1` 和 `FORM_FOLLOWER_EN=1`
- 左右从机 `FORM_POSITION` 是否分别设为 `1/2`
- 主从机 `UAVCAN_NODE_ID` 是否冲突
- 从机当前是否已经切入 Offboard
