# UAVCAN 编队姿态控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队姿态控制功能，用于三机刚性连接编队飞行系统。当前架构为**"主机发送期望控制姿态 + 从机跟随与铰链修正 + 执行级补偿（速率设定层 + 分配器）"**：

- 中央主飞控通过 DroneCAN 广播**主机期望控制姿态**：主机姿态设定（`vehicle_attitude_setpoint.q_d` 反解出的滚转/俯仰角）、期望滚转速率（`vehicle_rates_setpoint.roll`）、推力设定（`att_sp.thrust_body[0]`）与偏航速率指令（统一取 `rates.yaw`，即固定翼速率环的偏航角速率设定）；
- 左右从飞控按各自 `FORM_POSITION`，把主机期望控制姿态**合成为自身的姿态设定**（跟随主机滚转/俯仰 + 铰链运动学前馈补偿 + 航向保持 + 推力透传），在 Offboard attitude 模式下执行；
- 从机在下发执行前做两项补偿：**滚转速率→俯仰速率前馈**（翼尖铰链结构耦合，在 `fw_att_control` 的速率设定层施加，不触碰力矩通道）、**偏航力矩→转弯外侧机增推**（在 `ControlAllocator` 分配前施加）。详见 4.3 节。

由于指令源是主机的**期望控制姿态（设定值）**而非实测值，主机在手动与自主（AUTO/MISSION 等）模式下都能带动编队飞行；从机与主机控制器接收同一目标，两机各自闭环收敛，天然同步。

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
| 自定义消息 | `src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20040.ControlInput.uavcan` | 定义编队控制广播消息 `thrust/pitch/yaw/roll_target/roll_rate_target/flags` |
| 主机发送器 | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | 订阅 `vehicle_attitude_setpoint`、`vehicle_rates_setpoint`、`vehicle_status`，把主机期望控制姿态封装后广播 |
| UAVCAN 接收器 | `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}` | 接收 `ControlInput`，在从机本地合成姿态设定并发布 `offboard_control_mode` 和 `vehicle_attitude_setpoint` |
| 姿态控制层定制 | `src/modules/fw_att_control/FixedwingAttitudeControl.{hpp,cpp}` | 从机滚转速率→俯仰速率前馈（`CA_R2P_K`）：在 `vehicle_rates_setpoint` 发布前叠加；ACRO 不经过本模块 |
| 速率控制层定制 | `src/modules/fw_rate_control/FixedwingRateControl.{hpp,cpp}` | 主机偏航同步增推（`FORM_YAW_K_M`）；消费从机 `yaw_sp_move_rate` 时的速率限幅 |
| 分配器定制 | `src/modules/control_allocator/ControlAllocator.{hpp,cpp}` | 从机执行级补偿：偏航力矩→转弯外侧机增推 |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c`、`src/modules/control_allocator/module.yaml` | 定义编队参数（`FORM_*`）与滚转→俯仰混控增益（`CA_R2P_K`，由 `fw_att_control` 使用） |
| 启动接入 | `src/drivers/uavcan/uavcan_main.cpp`、`src/drivers/uavcan/sensors/sensor_bridge.cpp` | 根据参数和板级开关自动初始化发送器/接收器 |
| 速率内环定制（可选） | `src/modules/fw_rate_control/LADRC1.{hpp,cpp}`、`FixedwingRateControl.cpp`、`fw_rate_control_params.c` | `FW_ADRC_R_EN` / `FW_ADRC_P_EN` 分轴开启时，对应轴速率环由 PID 切换为一阶 LADRC（详见 5.4 与 6.3 节） |
| 板级配置 | `boards/cuav/7-nano/default.px4board` | 开启 `CONFIG_UAVCAN_FORMATION_RATES_SENDER` 与 `CONFIG_UAVCAN_SENSOR_FORMATION_RATES` |

## 3. 传输格式

### 3.1 UAVCAN 消息载体

当前实现使用自定义 DroneCAN 消息 `dronecan::formation::ControlInput`，携带主机期望控制姿态，字段如下：

| 字段 | 类型 | 含义 | 取值范围 |
| ---- | ---- | ---- | ---- |
| `thrust` | `float16` | 主机推力设定（`att_sp.thrust_body[0]`） | `[0, 1]` |
| `pitch` | `float16` | 主机俯仰姿态设定（`att_sp.q_d` 反解 `euler.theta()`） | `rad` |
| `yaw` | `float16` | 主机偏航速率指令（`rates_sp.yaw`，各模式统一） | `rad/s` |
| `roll_target` | `float16` | 主机滚转姿态设定（`att_sp.q_d` 反解 `euler.phi()`） | `rad` |
| `roll_rate_target` | `float16` | 主机期望滚转速率（`rates_sp.roll`，铰链前馈参考量） | `rad/s` |
| `flags` | `uint8` | 状态位（`FLAG_VALID`） | 位掩码 |

当前发送频率为 300 Hz：

```cpp
// src/drivers/uavcan/formation_rates_sender.hpp
static constexpr unsigned MAX_RATE_HZ = 300;
```

### 3.2 取值范围与数据语义

主机发送端读取的是**主机控制器正在跟踪的设定值**，而不是遥控器原始输入或实测姿态：

```cpp
// src/drivers/uavcan/formation_rates_sender.cpp
vehicle_attitude_setpoint_s att_sp{};
_vehicle_attitude_setpoint_sub.copy(&att_sp);

vehicle_rates_setpoint_s rates_sp{};
_vehicle_rates_setpoint_sub.copy(&rates_sp);

matrix::Quatf q_sp(att_sp.q_d);
matrix::Eulerf euler_sp(q_sp);
```

- `roll_target / pitch` 来自 `att_sp.q_d`（姿态控制器目标四元数）的反解：手动模式下约等于摇杆映射结果（含主机侧姿态整形），自主模式下是导航外环输出；
- `roll_rate_target` 装载 `rates_sp.roll`（期望滚转速率），作为从机铰链修正的前馈参考；
- `thrust` 装载 `att_sp.thrust_body[0]`（主机推力设定），发送前 `constrain` 到 `[0, 1]`；
- `yaw` 统一取 `rates_sp.yaw`（**不再按 `nav_state` 切换数据源**），校验有限值且 `rates_sp` 时间戳距今不超过 500 ms，否则发 0：
  - STAB / ALTCTL / POSCTL：协调转弯项 ＋ 偏航杆量（`FW_MAN_YR_MAX` 折算，限幅 ±`FW_Y_RMAX`）；
  - ACRO：杆量偏航速率（`FW_ACRO_Z_MAX`）；
  - MISSION / LOITER / RTL / TAKEOFF / LAND / DESCEND / OFFBOARD：协调转弯速率 `tan(φ)·cos(θ)·g/V`。

背景说明：`rates.yaw` 是固定翼速率环真正的偏航角速率设定，姿态类模式由 `fw_att_control` 发布、ACRO 由 `fw_rate_control` 发布，是唯一可靠的偏航指令源。此前实现按 `nav_state` 在自主模式改用 `att_sp.yaw_sp_move_rate`，但固定翼链路（`fw_att_control` / `fw_lateral_longitudinal_control`）**从不写该字段**——它零初始化、`PX4_ISFINITE()` 会误通过，导致主机在 MISSION 下实际发出 `yaw = 0`，从机退化为仅用自身协调转弯。现统一改用 `rates_sp.yaw`，MISSION 等自主模式可正常驱动从机偏航。

发送门控：固定翼或过渡模式，且 `att_sp` 时间戳距今不超过 500 ms。固定翼 MANUAL 下无 `vehicle_attitude_setpoint` 生产者（`fw_att_control` 姿态分支要求 `attitude` 使能，`fw_lateral_longitudinal_control` 要求 position/velocity/altitude 使能），500 ms 后自动停发——该模式不支持编队链。


## 4. 当前控制映射关系

控制方式：**主机发送期望控制姿态、从机跟随与铰链修正、执行级补偿**。从机发布 `vehicle_attitude_setpoint` 注入固定翼姿态环，由 `fw_att_control` 继续下发 `vehicle_rates_setpoint` 给速率环；下发时叠加滚转→俯仰速率前馈，再由 `ControlAllocator` 在分配前叠加外侧增推。

因此：
- 从机 `roll/pitch` 姿态设定值直接跟随主机的姿态设定（`roll_target / pitch`），从机不再做摇杆重映射；
- `pitch` 通道额外叠加一项铰链运动学前馈：`side_sign * FORM_HINGE_K * roll_rate_target`，用于补偿编队滚转时的翼尖铰链几何耦合；
- 从机姿态设定的航向分量保持**自身当前航向**（不接收航向位置指令）；偏航运动由 `yaw_sp_move_rate = msg.yaw` 直接驱动（`FORM_FOLLOWER_EN=1` 时 `fw_att_control` 用它替换自身协调转弯项，见 5.3 节），无有效指令时回退滚转协调转弯；
- 从机推力直接透传主机推力设定；滚转速率→俯仰速率前馈在 `fw_att_control` 内完成（速率设定发布前，见 4.3.1），转弯外侧增推在 `ControlAllocator` 分配前完成（见 4.3.2）；
- 从机控制链只有在自身飞行模式已经切入 Offboard 时才真正接管。

### 4.1 从机本地解算

- 左机 `side_sign = +1`
- 右机 `side_sign = -1`

定义：
- 主机姿态设定：`roll_target / pitch`（`msg.roll_target / msg.pitch`）
- 主机期望滚转速率：`roll_rate_target`（`msg.roll_rate_target`）
- 从机自身航向：`self_yaw`

**从机姿态设定值**计算：

```text
roll_sp  = constrain(msg.roll_target, -FORM_ROLL_AMAX, +FORM_ROLL_AMAX)

pitch_sp = constrain(msg.pitch + side_sign * FORM_HINGE_K * msg.roll_rate_target,
                     -FORM_PTCH_AMAX, +FORM_PTCH_AMAX)

yaw_sp   = wrap_pi(self_yaw)            # 保持自身航向

q_d = quat_from_euler(roll_sp, pitch_sp, yaw_sp)

yaw_sp_move_rate = msg.yaw              # 偏航速率指令（前馈通道）

thrust_body[0]   = constrain(msg.thrust, 0, 1)   # 主机推力直接透传
```

说明：
- 从机无需任何遥控器输入：主机在手动或自主模式下，从机都按同一份期望控制姿态飞行；
- `FORM_HINGE_K` 的量纲是时间（s）：`[s] × [rad/s] = [rad]`，物理含义是"主机滚转速率→从机俯仰补偿"的等效几何耦合时间尺度；
- 铰链修正与主机滚转**意图**同相位（前馈），不需要等从机实测出现误差后才补偿；
- 左右从机修正方向相反（`side_sign` 镜像），对应两侧铰链的运动学镜像关系；
- 从机航向分量不做位置指令（保持自身航向），体现链翼构型"铰链约束偏航、从机随编队整体转向"的特征；固定翼从机的转弯实际由滚转协调转弯完成；
- 推力不做从机侧增推，直接透传主机推力设定（增推逻辑见 4.3 节）。

### 4.2 物理意义

- 主机滚转设定直接作为左、右从机的滚转设定（同一目标），三机滚转动作同源、无跟踪滞后链条；
- 主机以速率 `p` 滚转（或计划滚转）时，翼尖铰链约束迫使两侧从机产生几何上必需的点头/抬头运动；`FORM_HINGE_K * p` 作为前馈量叠加到从机俯仰设定，让从机主动完成该运动，而不是被动承受铰链结构载荷；
- 左右从机铰链修正方向相反（`side_sign` 镜像），对应链翼构型"铰链释放相对滚转、约束相对偏航"的约束特征；
- 从机航向保持自身，通过 `yaw_sp_move_rate` 接收偏航速率意图并由 `fw_att_control` 直接跟踪（`FORM_FOLLOWER_EN=1`：替换自身协调转弯项；无有效指令或本机手动接管时回退协调转弯，详见 5.3 节），偏航与主机经铰链保持一致；
- 从机推力透传主机推力设定：主机在自主模式（TECS 推力输出）或手动模式（油门映射）下，从机都能正确跟随；
- 主机侧仍保留一项偏航同步加速（`FixedwingRateControl` 内，仅主机且 `UAVCAN_PUB_FORM=1` 时生效，按模式取信号源）：

```text
杆量分支 (manual=true, rates=true): master_thrust = thrust_body[0] + 0.5 * abs(manual.yaw)   * FORM_YAW_K_M
速率分支 (manual=false)           : master_thrust = thrust_body[0] + 0.5 * abs(rates_sp.yaw) * FORM_YAW_K_M
```

- 杆量分支实际覆盖 **STAB / ALTCTL / POSCTL / ACRO**；固定翼 **MANUAL 不生效**——其 `flag_control_rates_enabled = false`，整段增推代码位于该判断内部，不会执行；
- 速率分支覆盖 **MISSION / LOITER / RTL / TAKEOFF / LAND / DESCEND / OFFBOARD**；
- 非手动模式下 `manual_control_setpoint` 不再刷新，不能用杆量（否则取到切换前的残留值）；
- ⚠ **量纲不一致**：杆量分支的 `yaw` 是无量纲杆量（0~1），速率分支的 `yaw` 是 rad/s，同一个 `FORM_YAW_K_M` 在两条分支里含义不同（满杆 ≈ 0.5·K_M，而 0.4 rad/s 的协调转弯 ≈ 0.2·K_M）；
- ⚠ **自主模式收益有限**：MISSION 下 `thrust_body[0]` 是 TECS 的空速闭环输出，叠加的开环增推会被 TECS 反调抵消（稳态空速不变、积分器被压低），因此该分支只影响过渡瞬态，不建议作为自主模式的"转弯增推"手段。

### 4.3 执行级补偿（从机）

两项补偿的层级不同：一在**速率设定域**、一在**控制分配前**。

#### 4.3.1 滚转速率→俯仰速率前馈（`CA_R2P_K`）—— 位于 `fw_att_control`

在 `FixedwingAttitudeControl::Run()` 中，`body_rates_setpoint` 合成完毕、`vehicle_rates_setpoint` 发布之前（放在 tailsitter 变换之前，工作在固定翼角速率域）：

```cpp
// src/modules/fw_att_control/FixedwingAttitudeControl.cpp
if ((_param_form_follower_en != 0) && (fabsf(_side_sign) > FLT_EPSILON)) {
    const float pitch_rate_ff = _side_sign * _param_ca_r2p_k.get() * body_rates_setpoint(0);
    const float pitch_rate_max_pos = radians(_param_fw_p_rmax_pos.get());
    const float pitch_rate_max_neg = radians(_param_fw_p_rmax_neg.get());

    body_rates_setpoint(1) = math::constrain(body_rates_setpoint(1) + pitch_rate_ff,
                                             -pitch_rate_max_neg, pitch_rate_max_pos);
}
```

即 $\omega_{y,sp} \leftarrow \mathrm{clamp}\left(\omega_{y,sp} + \mathrm{side\_sign}\cdot K_{r2p}\cdot \omega_{x,sp}\right)$。

要点：

- **作用在速率设定域**：只改俯仰速率设定，滚转通道完全不受影响；俯仰 LADRC 把它当作"正常指令"跟踪，其 ESO 输入 `_u_prev` **只含自己的输出**，`z2` 只反映真实扰动 → 两者真正解耦（不再依赖 `b0` 准确才能不误判混控）；
- **不占用力矩权限**：不进入 `τy`、不与 LADRC 抢 ±1 限幅，从根上消除早期"混控吃满俯仰通道 → 单侧升降舵失灵"的失效模式；
- **无稳态拖尾**：前馈源是 `body_rates_setpoint(0)`（纯速率指令，不含积分/配平），打杆回正后自然归零，不需要高通清洗；
- **限幅**：叠加后按 `FW_P_RMAX_POS` / `FW_P_RMAX_NEG`（默认 ±60°/s）限幅；
- **门控**：`FORM_FOLLOWER_EN=1` 且 `FORM_POSITION=1/2`（`side_sign≠0`）才执行，主机（`FORM_FOLLOWER_EN=0`）不受影响；
- **ACRO 不覆盖**：ACRO 下 `rates_sp` 由 `fw_rate_control::vehicle_manual_poll()` 直接发布，不经过本模块，因此不做该补偿；
- **与 `FORM_HINGE_K` 的关系**：`formation_rates.cpp` 在**姿态设定层**另有一条 `pitch_sp += side_sign·FORM_HINGE_K·msg.roll_rate_target`（跟随主机期望滚转速率）。两条同时开会双重补偿，联调时先把其中一条置 0。

⚠ **参数含义已变**：`CA_R2P_K` 从"力矩比"变为**速率比**（Δ俯仰速率 / 滚转速率，无量纲）。相同的数值不再是相同的补偿量，需重新标定，建议从 `0.2~0.3` 起。

> **历史沿革**（为什么改到这里）：
> ① 最初在 `ControlAllocator` 力矩域（`c[0](1) += c[0](0)·K·side_sign`）——占用俯仰权限，且 LADRC 的 ESO 看不到该分量，把它当扰动反相抵消，混控白做；
> ② 后移到 `fw_rate_control` 力矩域（`setAppliedControl()` 之前）——ESO 可见了，但仍与 ADRC 共用同一个 `τy` 通道，且 `τx` 里的稳态成分（滚转积分、副翼配平）会把升降舵拖出长时间偏置，需额外加 0.4 s 高通清洗；
> ③ 现方案改到 `fw_att_control` 速率设定域——上述问题一并消失，代码也更短。

#### 4.3.2 转弯外侧增推（`FORM_YAW_K`）—— 仍在 `ControlAllocator`

```cpp
// src/modules/control_allocator/ControlAllocator.cpp
if (_is_follower) {
    // 转弯外侧增推：按偏航力矩设定单侧截断叠加推力（只增不减）
    c[0](3) = math::constrain(c[0](3) + math::max(_side_sign * c[0](2), 0.f) * _yaw_throttle_gain, 0.f, 1.f);
}
```

- `_yaw_throttle_gain` 来自 `FORM_YAW_K`（从机外侧增推系数）：转弯时按从机的偏航力矩设定（`τz`）给**外侧**从机加推（左转时右机增推、右转时左机增推——`side_sign` 与 `τz` 乘积单侧截断自动选出外侧机），内侧机不动作；主机侧改用 `FORM_YAW_K_M`，不再共用该参数；
- 该补偿需要知道**分配前**的 `τz`，因此仍留在分配器；它由 `_is_follower` 门控（`FORM_FOLLOWER_EN≠0`）；
- 主机与第二分配实例：主机（`FORM_FOLLOWER_EN=0`）下两项均不生效；多矩阵场景的第二实例（`c[1]`，读 `vehicle_torque_setpoint1`）不含偏航增推，也不含 R2P（R2P 已在更上游的 `fw_att_control` 完成，且不在力矩域），当前三机均为单一固定翼有效性矩阵，不受影响。

## 5. 代码流程

### 5.1 主机端：formation_rates_sender

初始化时：
- 启动 300 Hz 定时器

周期执行时：
1. 读取 `vehicle_attitude_setpoint`（主机姿态设定），超过 500 ms 未更新则停止发送
2. 读取 `vehicle_status`，仅在固定翼模式或过渡模式下继续工作
3. 读取 `vehicle_rates_setpoint`（含期望滚转速率 `rates_sp.roll`）
4. 反解 `att_sp.q_d` 得到滚转/俯仰设定角 `euler_sp.phi() / euler_sp.theta()`
5. 取 `rates_sp.yaw` 作为偏航指令源（校验有限值与 500 ms 新鲜度，否则发 0）
6. 将 `thrust/pitch/yaw/roll_target/roll_rate_target` 打包到 `ControlInput`
7. 设置 `flags = FLAG_VALID` 并广播单条 `ControlInput`

### 5.2 从机端：formation_rates

初始化时：
- 查找参数 `FORM_FOLLOWER_EN / FORM_POSITION / FORM_HINGE_K / FORM_ROLL_AMAX / FORM_PTCH_AMAX`
- 注册 `ControlInput` 订阅回调

接收回调时：
1. 仅在 `parameter_update` 到达时刷新参数
2. 检查 `FORM_FOLLOWER_EN`，未开启则直接返回
3. 检查 `flags` 里的 `FLAG_VALID`
4. 检查 `FORM_POSITION` 是否为左机或右机
5. 读取从机自身 `vehicle_attitude`（取 `self_yaw`）和 `vehicle_status`
6. 发布 `offboard_control_mode`，其中 `attitude=true`
7. 如果当前不是 Offboard，则直接退出这次回调，不发布 `vehicle_attitude_setpoint`
8. 如果当前已经是 Offboard，则按"跟随 + 铰链修正"逻辑生成 `vehicle_attitude_setpoint`：
   - `roll_sp = clip(msg.roll_target)`
   - `pitch_sp = clip(msg.pitch + side_sign * FORM_HINGE_K * msg.roll_rate_target)`
   - `yaw_sp = self_yaw`，`yaw_sp_move_rate = msg.yaw`
   - `thrust_body[0] = clip(msg.thrust)`
9. 由 `FixedwingAttitudeControl` 将姿态设定值转为 `vehicle_rates_setpoint`（从机：发布前叠加滚转→俯仰速率前馈）
10. 由 `FixedwingRateControl` 生成力矩/推力设定，再由 `ControlAllocator` 叠加从机定制后解算执行

### 5.3 下游控制链

```text
FormationRatesBridge
  -> offboard_control_mode(attitude=true)
  -> vehicle_attitude_setpoint (q_d / yaw_sp_move_rate / thrust_body)
  -> Commander 进入/维持 Offboard attitude 控制链
  -> FixedwingAttitudeControl：从 q_d 取 roll/pitch 生成速率设定；
     从机（FORM_FOLLOWER_EN=1）消费 yaw_sp_move_rate → 替换自身协调转弯项（见下方说明）
     从机（FORM_FOLLOWER_EN=1）：rates.pitch += side_sign·CA_R2P_K·rates.roll（发布前叠加）
  -> FixedwingRateControl 生成 vehicle_torque_setpoint / vehicle_thrust_setpoint
  -> ControlAllocator（从机）：叠加外侧增推后解算到舵面/电机
```

注意：`yaw_sp_move_rate` 的消费端已实现（2026-10-09）：`fw_att_control` 姿态分支中，`FORM_FOLLOWER_EN=1` 且收到新鲜指令（`att_sp` 时间戳 <100 ms、非手动接管、`|yaw_sp_move_rate| > 0.01 rad/s`）时，用主机偏航速率指令**替换**自身协调转弯项并发布（限幅 ±`FW_Y_RMAX`）——铰链约束相对偏航（两机偏航速率应一致），且主机指令本身已含其协调转弯分量，故采用替换而非叠加；条件不满足时回退原协调转弯逻辑（行为不变）。从机偏航力矩因此与偏航指令同向，`ControlAllocator` 外侧增推门（`max(side_sign·τz,0)`）在编队机动中可按设计触发；从机姿态设定的航向角分量与 `q_d` 的 yaw 分量仍不被消费（从机保持自身航向）。

速率内环可选启用 LADRC（`FW_ADRC_R_EN` / `FW_ADRC_P_EN` 分轴独立开启，见 5.4 节）；偏航与其余链路不受影响。

Offboard 丢失保护由 PX4 原生参数 `COM_OF_LOSS_T` 控制，当前实现没有额外的编队私有超时参数。

### 5.4 速率内环 LADRC（可选）

`FW_ADRC_R_EN=1`（滚转）或 `FW_ADRC_P_EN=1`（俯仰）时，`FixedwingRateControl` 的**对应轴速率内环由 PID 切换为一阶线性自抗扰控制（LADRC1）**，两轴可独立开启，偏航轴保持 PID，原 PID 代码完整保留（置 0 即回退）。速率设定值随姿态外环/空速时变，因此控制器含 TD 指令平滑与导数前馈；对链翼构型，翼尖铰链耦合载荷计入"总扰动"，由 ESO 估计并实时抵消，无需显式建模。

> ⚠ 与从机 R2P 的关系：`CA_R2P_K` 现在作用在**速率设定层**（`fw_att_control`），只是把俯仰速率设定平移一个量，LADRC 把它当作正常指令跟踪；其 ESO 输入 `_u_prev` 里**不含**任何混控分量，`z2` 只反映真实扰动——因此 `b0` 不准也不会把混控误归成扰动。更早的实现把它放在力矩域（分配器 / 速率控制层），都会与 ESO 发生交互（分配器里会被当作扰动抵消，速率控制层里需依赖 `b0` 正确才能不误判），这也是最终上移到速率设定层的原因（见 4.3.1）。

关联仿真：`docs/reference/LADRC1.m`。

```text
被控对象 (每个轴, 相对阶 1):   ṗ = f + b0·u
    f 为总扰动 (气动非线性、轴间耦合、铰链载荷、阵风等); u 为控制量 (域与 PID 输出相同)

① TD (跟踪微分器, 常开):      ṙ* = λ·(r − r*), 输出 r* 与 ṙ*
② ESO (2 阶扩张状态观测器):   e  = z1 − y
                              ż1 = z2 − β1·e + b0·u_prev
                              ż2 = −β2·e        (z1 → 角速率估计, z2 → 总扰动估计 f)
③ SEF + ④ 扰动补偿:           u = (ωc·(r* − z1) + clamp(ṙ*/b0, ±0.5) − z2) / b0

带宽参数化 (Gao, ACC 2003):  β1 = 2ωo, β2 = ωo², kp = ωc（ωo 固定取 4×ωc）;  闭环等价于极点 −ωc 的一阶惯性环节。

微分前馈 `ṙ*/b0` 单独限幅到 ±0.5 控制量（`LADRC1` 类内 `kFfMax` 常量）：b0 偏小或指令翻极时，`ṙ*/b0` 会产生巨大尖峰（b0=2、140°/s 翻极时约 49），限幅后削到 0.5。取 0.5 而不是 1.0 的原因：前馈与比例项 `ωc·e/b0` 叠加，若两者都能到 1.0，环路会在瞬态长期饱和并激发结构模态；而正常机动（30° 滚转、`FW_R_TC=0.4 s`）的 `ṙ* ≈ 3 rad/s²`，在 b0=10 时仅对应 0.3 控制量，不会触发限幅。若 b0 过小（如 2），同样机动的前馈会直接顶到限幅变成开关量。
```

信号流（下游与 5.3 节一致，仅替换滚转/俯仰通道）：

```text
vehicle_rates_setpoint (rad/s) ─┐
                                ▼
        LADRC1 (roll / pitch) → 替换 angular_acceleration_setpoint(0/1)
                                ▼
        ×空速缩放² → +trim → 限幅 → ControlAllocator → 舵面
        偏航轴: PID 原样

限幅后写回: u_prev = (力矩指令 − trim) / airspeed_scaling²   (ESO 使用实际施加值 → 反饱和)
注: 从机 R2P 已上移到速率设定层, 上述链路中不再含任何混控分量
```

关键工程点：

- **影子运行**：某轴开关为 0 时该轴 LADRC 仍每拍计算（不接管输出），保证飞行中随时切换无跳变；开关跳变时自动清零对应轴 PID 积分（接管期间该轴 PID 输出被丢弃，积分不清零会在切回瞬间造成力矩突跳）；
- **反饱和**：ESO 模型输入使用限幅后的"实际施加值"（trim 项由 z2 自适应吸收）；
- **离散化保护**：TD 与 ESO 均为显式欧拉，内部积分步长按观测器带宽限制为 `dt ≤ kMaxDtRatio/ωo`（`kMaxDtRatio = 0.25`，ωc=10 时约 6.25 ms）→ 陀螺掉帧或 20 ms backup 调度时不会越过稳定边界（正常 ≥250 Hz 采样不受影响）；
- **扰动估计限幅**：`z2` 限制在 ±`kZ2MaxRatio·b0`（`kZ2MaxRatio = 0.5`，即扰动最多等价于 50% 舵量）→ 模型失配、测量噪声时扰动估计不会无限累积，也不会把舵面顶到满行程；
- **b0 下限**：`setGains()` 将 b0 限制到 ≥ `kMinB0 = 0.5`，避免 b0 误设为 0 时 `ωc/b0` 爆炸；
- **复位**：`rates_sp.reset_integral`、落地、非固定翼状态、控制量非有限时，ESO/TD 热启动复位（避免初始瞬态）；测量/指令出现非有限值时内部兜底（异常拍输出清零，下一拍以当前测量/指令重新热启动），NaN 不扩散、也不会锁死该轴；
- **日志**：某轴被接管时，`rate_ctrl_status.rollspeed_integ / pitchspeed_integ` 中对应字段记录该轴的 z2 扰动估计（未接管轴与偏航仍为 PID 积分）；
- tailsitter 场景自动回退 PID；偏航轴、姿态外环、前馈/trim/空速缩放逻辑均未改动；
- 状态：2026-10-09 编译验证通过（cuav_7-nano；含非有限值兜底与开关跳变清零积分、主机增推按模式取源、从机偏航指令消费端），速率环默认参数按极点对标（见 6.3 节）；未实飞。

## 6. 实际生效参数

### 6.1 主机参数（中央机，Node ID=1）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 本机节点 ID，主机固定为 `1` |
| `UAVCAN_PUB_FORM` | 发送器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 主机应设为 `0`，用于区分主机/从机身份 |
| `FORM_YAW_K_M` | 主机偏航同步加速系数（`FixedwingRateControl` 内，仅主机生效；手动类按杆量、自主类按 `rates_sp.yaw`，见 4.2 节）。默认 `0.3`，建议首飞先 `0.2~0.3`；`FORM_YAW_K` 现在只用于从机分配器，主机不再读取 |

### 6.2 从机参数（左/右机）

基础通信参数：
| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 左机为 `2`，右机为 `3` |
| `UAVCAN_SUB_FORM` | 接收器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 从机内部处理开关，设为 `1` |
| `FORM_POSITION` | 左机设 `1`，右机设 `2` |

编队控制参数（`Formation Control` 组）：
| 参数 | 默认 | 说明 |
| ---- | ---- | ---- |
| `FORM_HINGE_K` | 0.01 | 铰链修正前馈增益（单位：s）：`pitch += side_sign * FORM_HINGE_K * 主机期望滚转速率`（地面测试值） |
| `FORM_ROLL_AMAX` | 1.05 | 从机滚转姿态设定限幅（rad，≈60°） |
| `FORM_PTCH_AMAX` | 1.05 | 从机俯仰姿态设定限幅（rad，≈60°，含铰链修正） |
| `FORM_YAW_K` | 3.0 | 从机分配器外侧增推系数：`thrust += max(side_sign * τz, 0) * FORM_YAW_K`。试飞建议先从 `1.0` 起（不要用 3.0），确认无“越失稳越增推”后再逐步加；主机侧改用 `FORM_YAW_K_M` |

分配器参数（`Control Allocation` 组，现由 `fw_att_control` 使用，仅从机生效）：
| 参数 | 默认 | 说明 |
| ---- | ---- | ---- |
| `CA_R2P_K` | 0.5 | 滚转速率→俯仰速率前馈增益：`rates.pitch += rates.roll * CA_R2P_K * side_sign`（左右符号自动镜像，在 `fw_att_control` 发布 `vehicle_rates_setpoint` 之前施加，叠加后按 `FW_P_RMAX_*` 限幅）。**量纲已从力矩比变为速率比**，数值需重新标定，建议从 `0.2~0.3` 起 |

其他：
| 参数 | 说明 |
| ---- | ---- |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 速率内环参数（`FW ADRC` 组，可选）

开关 `FW_ADRC_R_EN` / `FW_ADRC_P_EN` 分别启用对应轴（结构与代码见 5.4 节）：

| 参数 | 默认值 | 单位 | 说明 |
| ---- | ---- | ---- | ---- |
| `FW_ADRC_R_EN` | 0 | — | 0 = PID（默认）；1 = LADRC 接管滚转轴 |
| `FW_ADRC_P_EN` | 0 | — | 0 = PID（默认）；1 = LADRC 接管俯仰轴 |
| `FW_ADRC_R_B0` | 25.0 | rad/s² | 滚转模型增益 b0 |
| `FW_ADRC_R_WC` | 4.0 | rad/s | 滚转控制器带宽 ωc（闭环极点 −ωc；起步值对标原生 PID 速率环极点） |
| `FW_ADRC_P_B0` | 20.0 | rad/s² | 俯仰模型增益 b0 |
| `FW_ADRC_P_WC` | 3.0 | rad/s | 俯仰控制器带宽 ωc（起步值对标原生 PID 速率环极点） |

TD 带宽固定 40 rad/s（`LADRC1` 类内 `kTdLambda` 常量）；观测器带宽固定为 4×ωc（`LADRC1` 类内 `kWoRatio` 常量，Gao 建议 3~5×）：两者均不参数化（带宽参数化的本意即把可调量压缩为 b0 + 一个带宽），如需调整修改常量后重新编译。

注：原单一开关 `FW_ADRC_EN` 已拆分为 `FW_ADRC_R_EN` / `FW_ADRC_P_EN` 两轴独立开关；`b0`/`WC` 参数亦改为轴前缀命名：`FW_ADRC_R_B0` / `FW_ADRC_R_WC` / `FW_ADRC_P_B0` / `FW_ADRC_P_WC`（与轴开关命名对齐）。以上旧名在固件中均不再存在，旧保存值失效，升级后需重新设置。

**注意参数耦合**：`WC`（ωc）是唯一的“总强度”旋钮，一个参数同时决定
- 比例增益：kp = ωc；
- 观测器（扰动估计）速度：ωo = 4×ωc；
- “隐式积分”强度（*z2* 通道，承担原 PID *积分角色*）：β2 = ωo² = 16×ωc²，与 ωc 平方相关。

b0 定义：单位归一化力矩指令（±1）在配平空速下产生的角加速度：

```text
b0_R = q̄·S·b·Clδa/Ixx × δa_max ≈ 25 rad/s²   (滚转: 力臂=翼展 b, 惯量=Ixx, 效能=副翼)
b0_P = q̄·S·c·Cmδe/Iyy × δe_max ≈ 20 rad/s²   (俯仰: 力臂=弦长 c, 惯量=Iyy, 效能=升降舵)
```

滚转/俯仰是两套物理通道（力臂、惯量、舵面效能均不同），所以 b0 分轴给定、数值不同；偏航未换（仍 PID），没有 b0。默认值按 VLM 气动数据 + "力矩指令 ±1 ≈ 全舵面行程（约 ±25°）"估算，实机建议辨识后替换。

ωc 起步值按**极点对标**选取：与原生 PID 速率环的实际闭环极点（约 3~5 rad/s，由气动阻尼与反馈共同决定）相当，便于直接对比替换效果。**当前试飞基线（6.5 节）为 `P_B0=10.0`、`P_WC=10.0`；这是本机试飞配置，不是固件默认值（默认 `P_B0=20`、`P_WC=3`）。** 等效比例增益 `ωc/b0` 对应 1.0，与原生 PID 的 `FW_PR_P = 0.08` 相比约 **12.5 倍**——满舵出现在速率误差 1.0 rad/s（≈57°/s），而原生 PID 要到 12.5 rad/s（≈716°/s）才满舵。12.5 倍本身不失控，但需注意两点：① `ωc = 10 rad/s`（1.6 Hz）与 0.78 Hz 结构模态只差约 2 倍，低于 3 倍经验准则；② 若实际 `b0` 接近 VLM 估算的 20，则 `ωc/b0` 只有 0.5（约 6.25 倍），基线偏保守。建议先用阶跃/扫频辨识 `b0`（单位力矩指令下的角加速度 `b0 = Δṙ/Δu`），再按 `ωc/b0` 与 `FW_PR_P` 的比值决定 `WC`；分档建议 `ωc = 5 → 7 → 10`，每档单独验证后再升。若需*单独*增强积分而不动 P，则调 `kWoRatio`。

**调参**顺序（重要性排序）：

1. b0：b0 *偏小* → 算出的舵量偏大 → 等效增益高 → 容易*振荡*；b0 *偏大* → 响应*迟钝*。b0 不准还会折算进等效积分强度（Ki ∝ β2/b0）：b0 偏大时积分被“稀释”，稳态偏差收敛与抗扰变差。
2. ωc（控制器带宽）：*振荡就降、跟踪慢就升*；需明显快于姿态外环带宽。P、观测速度、隐式积分（∝ωc²）随它一起变——若响应已够快但稳态偏差收敛慢/抗扰不足，说明缺的是积分而非比例，见第 3 条。
3. 观测器带宽比（一般不动）：ωo 固定为 4×ωc。仅当需要*单独加强/减弱扰动抑制（积分）、又不想改 P 与响应速度时*，修改 `kWoRatio`（保持 Gao 建议区间 3~5）后重新编译：调大 → 扰动估计更快、隐式积分更强，但对速率测量噪声更敏感。

启用建议：先保持两轴开关为 `0`（PID）验证基础行为，再逐轴置 `1`、小幅滚转/俯仰打杆观察（出现振荡降 `WC` 或核对 `B0`），通过后再进行常规科目与编队联调；闭环验证见 6.6 节。

### 6.4 推荐配置示例

主机：
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_FORM 1
param set FORM_FOLLOWER_EN 0
param set FORM_YAW_K_M 0.3
param set FORM_YAW_K 0

param set FW_ADRC_P_EN 1
param set FW_ADRC_P_B0 10.0
param set FW_ADRC_P_WC 10.0
param set FW_ADRC_R_EN 0
param set FW_ADRC_R_B0 25.0
param set FW_ADRC_R_WC 4.0
param save
reboot
```

左/右机：
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 2 / 3
param set UAVCAN_SUB_FORM 1
param set FORM_FOLLOWER_EN 1
param set FORM_POSITION 1 / 2

param set FORM_HINGE_K 0.01
param set FORM_ROLL_AMAX 1.05
param set FORM_PTCH_AMAX 1.05
param set FORM_YAW_K 1.0
param set CA_R2P_K 0.2
param set COM_OF_LOSS_T 5.0

param set FW_ADRC_P_EN 1
param set FW_ADRC_P_B0 10.0
param set FW_ADRC_P_WC 10.0
param set FW_ADRC_R_EN 0
param set FW_ADRC_R_B0 25.0
param set FW_ADRC_R_WC 4.0
param save
reboot
```


说明：
- 三者作用不同：`FORM_HINGE_K` 为铰链前馈（姿态设定层，跟随**主机**期望滚转速率）、`FORM_YAW_K` 为从机转弯外侧增推、`CA_R2P_K` 为滚转速率→俯仰速率前馈（速率设定层，跟随**本机**滚转速率设定）。前两者都从不同角度补偿铰链耦合，**同时打开会双重补偿**，建议先只开 `CA_R2P_K`（`FORM_HINGE_K=0`）；
- 主机/从机增推已拆分：主机用 `FORM_YAW_K_M`（建议 `0.3`），从机用 `FORM_YAW_K`（首飞建议 `1.0`）；主机侧 `FORM_YAW_K` 不再被读取，建议显式设 `0` 避免误解；
- `CA_R2P_K` 作用在速率设定域：不改力矩，**不可能**饱和俯仰通道，也不参与分配器限幅；判断补偿是否过大看飞机滚转时是否出现明显的俯仰点头/抬头。参数含义已从“力矩比”改为“速率比”，旧值不可直接沿用；
- `FW_ADRC_P_B0` 基线为 `10.0`（配合 `P_WC=10.0` 时 `ωc/b0 = 1.0`，约 12.5× 原生 `FW_PR_P = 0.08`）。首次试飞建议先分档 `b0 = 20 → 15 → 10`（等效增益 0.5 → 0.67 → 1.0），每次只改这一个量，用阶跃响应和 `rate_ctrl_status.pitchspeed_integ`（ADRC 开启时该字段记录的是 z2 扰动估计，单位 rad/s²，不是速率积分）判断 b0 是否匹配；不要与 `FORM_YAW_K`/`CA_R2P_K` 同时改。
- 从机进入 Offboard 后，接收器才会继续发布 `vehicle_attitude_setpoint`，由 CAN 控制链正式接管；随后 `FixedwingAttitudeControl` 会继续生成 `vehicle_rates_setpoint` 给速率环。
- 主机不走从机的 Offboard 控制链，而是在 `FixedwingRateControl.cpp` 内直接对自身推力输出叠加偏航同步加速。
- 速率内环 LADRC：**当前试飞基线为仅俯仰轴开启**——主机与左右从机均 `FW_ADRC_P_EN=1`、`FW_ADRC_R_EN=0`；俯仰使用 `P_B0=10.0`、`P_WC=10.0`（`ωc/b0 = 1.0`，约 12.5× 原生 `FW_PR_P=0.08`），左右从机保持对称。滚转轴仍为 PID（`FW_R_RMAX` 默认 70°/s）。这套参数是本机试飞基线，不是固件默认值（默认见 6.3 节：`P_B0=20`、`P_WC=3`）；等效比例增益取决于本机实际 `b0`，建议先用阶跃/扫频辨识 `b0`，再调 `WC`；ωo 固定 4×ωc，TD 与 ESO 内部积分步长受 `kMaxDtRatio/ωo` 限制（ωc=10 时约 6.25 ms）。
