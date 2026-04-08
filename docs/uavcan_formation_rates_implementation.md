# UAVCAN 编队姿态控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队姿态控制功能，用于三机刚性连接编队飞行系统。中央主飞控读取遥控器输入，同时广播主机实际姿态角与机体系角速度；左右从飞控按各自 `FORM_POSITION` 在本地完成控制解算，然后在 Offboard attitude 模式下执行。

当前实现面向 `make cuav_7-nano_default`，相关功能已经通过板级配置编进固件。

核心机制：

```text
从机先保持自身原有飞行模式（如自稳/自主）
  -> 持续接收主机 CAN 指令并发布 offboard_control_mode 心跳
  -> 只有当从机被人工切入 Offboard 后
  -> 才开始发布 vehicle_attitude_setpoint，由 CAN 链路接管控制权
```

## 2. 当前实际使用的文件

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 自定义消息 | `src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20040.ControlInput.uavcan` | 定义编队控制广播消息 `throttle/yaw/roll/pitch/att_roll/att_pitch/att_yaw/p/q/r/flags` |
| 主机发送器 | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | 订阅 `manual_control_setpoint`、`vehicle_attitude`、`vehicle_angular_velocity`，广播 `ControlInput` |
| UAVCAN 接收器 | `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}` | 接收 `ControlInput`，在从机本地完成控制解算并发布 `offboard_control_mode` 和 `vehicle_attitude_setpoint` |
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
油门也是 `[-1, 1]` 实际代码使用会进行`(throttle + 1.f) * 0.5f`处理转换到 `[0, 1]`

### 3.3 传输方式

```text
主机广播一条 ControlInput
  -> 左右从机都接收
  -> 每个从机按本机 FORM_POSITION 判断自己是左机还是右机
  -> 用同一份代码、本地按符号完成左右差异化解算
```

## 4. 当前控制映射关系

控制方式: 当前改为“遥控器姿态主控 + 相对姿态辅助修正”结构，由从机发布 `vehicle_attitude_setpoint` 注入固定翼姿态环，再由 `fw_att_control` 继续下发 `vehicle_rates_setpoint` 给速率环。

因此：
- 从机 `roll / pitch` 姿态设定值由两部分共同决定，但当前以遥控器姿态主控部分为主。
- `FORM_R2P_GAIN` 当前承担 `roll -> pitch` 的耦合作用，在遥控器主控支路中把主机滚转输入映射为左右从机反向俯仰。
- `FORM_YAW_K` 当前同时承担两部分作用：从机侧偏航时外侧机附加油门，以及主机侧偏航时同步加速。
- `yaw` 当前先采用简单跟随：从机发布 `leader_yaw + FORM_YAW_OFS` 到姿态设定值中，后续再进一步细化专门的偏航控制律。
- 从机控制链只有在自身飞行模式已经切入 Offboard 时才真正接管。

### 4.1 从机本地解算

- 左机 `side_sign = +1`
- 右机 `side_sign = -1`

定义：
- 主机姿态角：`leader_roll / leader_pitch / leader_yaw`
- 主机角速度：`leader_p / leader_q`
- 从机姿态角：`self_roll / self_pitch`
- 从机角速度：`self_roll_rate / self_pitch_rate`

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
```

从机姿态设定值先分别计算两部分：

```text
relative_roll_corr  = FORM_ROLL_FF * (leader_roll + roll_rel_des)
                    + FORM_ROLL_KP * e_roll
                    + FORM_ROLL_KD * (leader_p - self_roll_rate) * 0.2

relative_pitch_corr = FORM_PITCH_FF * (leader_pitch + pitch_rel_des)
                    + FORM_PITCH_KP * e_pitch
                    + FORM_PITCH_KD * (leader_q - self_pitch_rate) * 0.2

stick_roll_target = roll * FORM_ROLL_AMAX

stick_pitch_target = side_sign * roll * FORM_R2P_GAIN * FORM_PTCH_AMAX
                   + pitch * FORM_PITCH_SYNC * FORM_PTCH_AMAX

stick_roll_att  = self_roll
                + FORM_ROLL_COMP * (stick_roll_target - self_roll)

stick_pitch_att = self_pitch
                + FORM_PITCH_COMP * (stick_pitch_target - self_pitch)

roll_sp  = constrain(1.0 * stick_roll_att  + 0.2 * relative_roll_corr,
                     -FORM_ROLL_AMAX, FORM_ROLL_AMAX)
pitch_sp = constrain(1.0 * stick_pitch_att + 0.1 * relative_pitch_corr,
                     -FORM_PTCH_AMAX, FORM_PTCH_AMAX)
yaw_sp   = wrap_pi(leader_yaw + FORM_YAW_OFS)

q_d = quat_from_euler(roll_sp, pitch_sp, yaw_sp)

base_thrust   = (throttle + 1) / 2
outer_boost   = left ? max(yaw, 0) : max(-yaw, 0)
thrust_x      = base_thrust + outer_boost * FORM_YAW_K
```

说明：
- 当前 `roll/pitch` 辅助修正权重是代码内固定常量，未做成参数。
- 当前实现中 `roll` 辅助权重为 `0.2`，`pitch` 辅助权重为 `0.1`。
- `FORM_ROLL_KD / FORM_PITCH_KD` 作用在相对角速度误差上，并通过固定比例 `0.2` 转成姿态修正量；默认值当前均为 `0.0`。
- `FORM_YAW_FF / FORM_YAW_KP / FORM_YAW_KD` 在当前姿态注入版本中暂未参与实际偏航修正，保留给后续专门的 yaw 控制逻辑。
- 当前 `FORM_ROLL_RMAX / FORM_PTCH_RMAX / FORM_YAW_RMAX` 保留在参数表中，但在姿态注入版本里不参与实际限幅。

主机侧还会在固定翼速率控制输出阶段增加一项偏航同步加速：

```text
master_thrust = thrust_body[0] + 0.5 * abs(manual.yaw) * FORM_YAW_K
```

其中这项逻辑只在 `FORM_FOLLOWER_EN == 0` 且 `UAVCAN_PUB_FORM == 1` 时生效，也就是主机身份并实际开启编队广播时生效；从机身份下不在 `FixedwingRateControl.cpp` 里执行这一步。

### 4.2 物理意义

- 主机向右滚时，左右从机会根据主机真实滚转姿态与从机自身滚转姿态的相对误差生成辅助滚转修正；同时 `FORM_R2P_GAIN` 会在遥控器主控支路里让左机抬头、右机低头，形成整体右滚。
- 主机向左滚时，左右从机相反动作，形成整体左滚。
- 主机 `leader_roll / leader_pitch` 通过 `FORM_ROLL_FF / FORM_PITCH_FF` 显式进入辅助修正项，提高从机跟随速度。
- 主机与从机的姿态差、角速度差分别通过 `KP / KD` 项形成反馈与阻尼，其中 `KD` 通过固定比例换算成姿态修正量。
- 遥控器主控支路当前直接生成 `roll/pitch` 姿态目标，因此当前从机控制不是纯相对姿态控制，而是“遥控器主控姿态 + 相对姿态辅助修正”的叠加结构。
- 当前辅助修正支路中，`pitch` 轴权重最小，为 `0.1`；`roll` 辅助权重为 `0.2`，因此编队修正对俯仰的介入更弱。
- 当前 `yaw` 先采用简单跟随，即把 `leader_yaw + FORM_YAW_OFS` 写入姿态设定值；固定翼姿态控制器仍主要显式使用 `roll/pitch` 设定值，后续再补专门的 yaw 控制设计。
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
- 查找姿态幅值参数 `FORM_ROLL_AMAX / FORM_PTCH_AMAX`
- 查找前馈/反馈阻尼参数 `FORM_ROLL_FF / KP / KD`、`FORM_PITCH_FF / KP / KD`
- 注册 `ControlInput` 订阅回调

接收回调时：
1. 仅在 `parameter_update` 到达时刷新参数
2. 检查 `FORM_FOLLOWER_EN`，未开启则直接返回
3. 检查 `flags` 里的 `FLAG_VALID`
4. 检查 `FORM_POSITION` 是否为左机或右机
5. 读取从机自身 `vehicle_attitude`、`vehicle_angular_velocity` 和 `vehicle_status`
6. 计算本机 `self_roll/self_pitch/self_yaw` 与 `self_roll_rate/self_pitch_rate`
7. 计算 `side_sign` 与偏航外侧机油门增量
8. 发布 `offboard_control_mode`，其中 `attitude=true`
9. 如果当前不是 Offboard，则直接退出这次回调，不发布 `vehicle_attitude_setpoint`
10. 如果当前已经是 Offboard，则继续按“遥控器主控姿态 + 相对姿态辅助修正”逻辑生成 `vehicle_attitude_setpoint`
11. 由 `FixedwingAttitudeControl` 将姿态设定值转为 `vehicle_rates_setpoint`
12. 再由 `FixedwingRateControl` 完成执行

### 5.3 下游控制链

```text
FormationRatesBridge
  -> offboard_control_mode(attitude=true)
  -> vehicle_attitude_setpoint
  -> Commander 进入/维持 Offboard attitude 控制链
  -> FixedwingAttitudeControl 将姿态设定值转换为 vehicle_rates_setpoint
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
| `FORM_ROLL_AMAX` | 遥控器主控支路的最大滚转姿态角 |
| `FORM_PTCH_AMAX` | 遥控器主控支路的最大俯仰姿态角 |
| `FORM_ROLL_FF` | 滚转 leader 姿态前馈增益 |
| `FORM_ROLL_KP` | 滚转相对姿态反馈比例增益 |
| `FORM_ROLL_KD` | 滚转相对速率差阻尼增益 |
| `FORM_PITCH_FF` | 俯仰 leader 姿态前馈增益 |
| `FORM_PITCH_KP` | 俯仰相对姿态反馈比例增益 |
| `FORM_PITCH_KD` | 俯仰相对速率差阻尼增益 |
| `FORM_ROLL_COMP` | 遥控器主控滚转姿态跟随增益 |
| `FORM_PITCH_SYNC` | 遥控器主控俯仰同步增益 |
| `FORM_PITCH_COMP` | 遥控器主控俯仰姿态跟随增益 |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 预留叠加参数

当前代码里以下参数保留在参数表中，但当前姿态注入版本不直接参与实际姿态设定值生成，保留给后续扩展或旧版速率逻辑参考：

| 参数 | 当前状态 |
| ---- | ---- |
| `FORM_YAW_FF` | 当前 yaw 简单跟随版本中暂未参与实际偏航修正 |
| `FORM_YAW_KP` | 当前 yaw 简单跟随版本中暂未参与实际偏航修正 |
| `FORM_YAW_KD` | 当前 yaw 简单跟随版本中暂未参与实际偏航修正 |
| `FORM_YAW_SYNC` | 当前 yaw 简单跟随版本中暂未参与实际偏航修正 |
| `FORM_YAW_COMP` | 当前 yaw 简单跟随版本中暂未参与实际偏航修正 |
| `FORM_ROLL_RMAX` | 姿态注入版本中暂未参与实际限幅 |
| `FORM_PTCH_RMAX` | 姿态注入版本中暂未参与实际限幅 |
| `FORM_YAW_RMAX` | 姿态注入版本中暂未参与实际限幅 |

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
param set FORM_ROLL_AMAX 0.52
param set FORM_PTCH_AMAX 0.35
param set FORM_ROLL_FF 0.5
param set FORM_PITCH_FF 0.5
param set FORM_ROLL_OFS 0.0
param set FORM_PITCH_OFS 0.0
param set FORM_YAW_OFS 0.0
param set FORM_ROLL_KP 2.0
param set FORM_ROLL_KD 0.0
param set FORM_PITCH_KP 2.0
param set FORM_PITCH_KD 0.0
param set FORM_ROLL_COMP 1.0
param set FORM_PITCH_SYNC 0.1
param set FORM_PITCH_COMP 1.0
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
param set FORM_ROLL_AMAX 0.52
param set FORM_PTCH_AMAX 0.35
param set FORM_ROLL_FF 0.5
param set FORM_PITCH_FF 0.5
param set FORM_ROLL_OFS 0.0
param set FORM_PITCH_OFS 0.0
param set FORM_YAW_OFS 0.0
param set FORM_ROLL_KP 2.0
param set FORM_ROLL_KD 0.0
param set FORM_PITCH_KP 2.0
param set FORM_PITCH_KD 0.0
param set FORM_ROLL_COMP 1.0
param set FORM_PITCH_SYNC 0.1
param set FORM_PITCH_COMP 1.0
param set COM_OF_LOSS_T 1.0
param save
reboot
```

从机进入 Offboard 后，接收器才会继续发布 `vehicle_attitude_setpoint`，由 CAN 控制链正式接管；随后 `FixedwingAttitudeControl` 会继续生成 `vehicle_rates_setpoint` 给速率环。
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
- 主机正常发送时，`offboard_control_mode` 应持续更新，并看到 `attitude = true`。
- 如果从机已经切入 Offboard，再执行：

```bash
listener vehicle_attitude_setpoint -r 5 -n 20
```

此时应能看到 `q_d` 与 `thrust_body[0]` 持续更新，并随主机姿态、主机 `p/q` 与从机 `FORM_POSITION` 发生对应变化。

如果还想继续验证姿态环到速率环的下游转换，可以再执行：

```bash
listener vehicle_rates_setpoint -r 5 -n 20
```

此时应能看到 `fw_att_control` 根据 `vehicle_attitude_setpoint` 继续生成的 `roll/pitch/yaw/thrust_body[0]` 更新。

如果 `listener vehicle_attitude_setpoint` 没有更新，优先检查：
- 主机是否已开启 `UAVCAN_PUB_FORM=1`
- 从机是否已开启 `UAVCAN_SUB_FORM=1` 和 `FORM_FOLLOWER_EN=1`
- 左右从机 `FORM_POSITION` 是否分别设为 `1/2`
- 主从机 `UAVCAN_NODE_ID` 是否冲突
- 从机当前是否已经切入 Offboard
