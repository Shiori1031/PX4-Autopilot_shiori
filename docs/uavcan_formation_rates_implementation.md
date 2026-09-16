# UAVCAN 编队姿态控制功能记录

## 1. 功能概述

为 PX4 新增了一套基于 UAVCAN 的编队姿态控制功能，用于三机刚性连接编队飞行系统。当前架构为**"主机发送期望控制姿态 + 从机跟随与铰链修正 + 分配器执行级补偿"**：

- 中央主飞控通过 DroneCAN 广播**主机期望控制姿态**：主机姿态设定（`vehicle_attitude_setpoint.q_d` 反解出的滚转/俯仰角）、期望滚转速率（`vehicle_rates_setpoint.roll`）、推力设定（`att_sp.thrust_body[0]`）与偏航速率指令（手动模式取 `rates.yaw`，自主模式取 `yaw_sp_move_rate`）；
- 左右从飞控按各自 `FORM_POSITION`，把主机期望控制姿态**合成为自身的姿态设定**（跟随主机滚转/俯仰 + 铰链运动学前馈补偿 + 航向保持 + 推力透传），在 Offboard attitude 模式下执行；
- 从机的 `ControlAllocator` 在力矩分配前做两项执行级补偿：滚转力矩→俯仰通道混控（翼尖铰链结构耦合）、偏航力矩→转弯外侧机增推。

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
| 分配器定制 | `src/modules/control_allocator/ControlAllocator.{hpp,cpp}` | 从机执行级补偿：滚转→俯仰力矩混控、偏航→外侧机增推 |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c`、`src/modules/control_allocator/module.yaml` | 定义编队参数（`FORM_*`）与分配器混控增益（`CA_R2P_K`） |
| 启动接入 | `src/drivers/uavcan/uavcan_main.cpp`、`src/drivers/uavcan/sensors/sensor_bridge.cpp` | 根据参数和板级开关自动初始化发送器/接收器 |
| 速率内环定制（可选） | `src/modules/fw_rate_control/LADRC1.{hpp,cpp}`、`FixedwingRateControl.cpp`、`fw_rate_control_params.c` | `FW_ADRC_EN=1` 时滚转/俯仰速率环由 PID 切换为一阶 LADRC（详见 5.4 与 6.3 节） |
| 板级配置 | `boards/cuav/7-nano/default.px4board` | 开启 `CONFIG_UAVCAN_FORMATION_RATES_SENDER` 与 `CONFIG_UAVCAN_SENSOR_FORMATION_RATES` |

## 3. 传输格式

### 3.1 UAVCAN 消息载体

当前实现使用自定义 DroneCAN 消息 `dronecan::formation::ControlInput`，携带主机期望控制姿态，字段如下：

| 字段 | 类型 | 含义 | 取值范围 |
| ---- | ---- | ---- | ---- |
| `thrust` | `float16` | 主机推力设定（`att_sp.thrust_body[0]`） | `[0, 1]` |
| `pitch` | `float16` | 主机俯仰姿态设定（`att_sp.q_d` 反解 `euler.theta()`） | `rad` |
| `yaw` | `float16` | 主机偏航速率指令（手动 `rates.yaw` / 自主 `yaw_sp_move_rate`） | `rad/s` |
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
- `yaw` 按主机飞行模式切换数据源：
  - 手动姿态操纵模式（MANUAL / STAB / ACRO / ALTCTL / POSCTL）：取 `rates.yaw`（操纵杆偏航已折算在内）；
  - 自主/半自主模式（AUTO / MISSION / OFFBOARD 等）：取 `att_sp.yaw_sp_move_rate`（导航解算的偏航速率）。

背景说明：手动模式下 `fw_att_control` 发布的 `att_sp.yaw_sp_move_rate` 保持零初始化值（`0.0f` 是有限值，`PX4_ISFINITE()` 检查会误通过），因此必须显式判断 `nav_state` 才能取到真正的偏航指令源。

发送门控：固定翼或过渡模式，且 `att_sp` 时间戳距今不超过 500 ms。


## 4. 当前控制映射关系

控制方式：**主机发送期望控制姿态、从机跟随与铰链修正、分配器执行级补偿**。从机发布 `vehicle_attitude_setpoint` 注入固定翼姿态环，由 `fw_att_control` 继续下发 `vehicle_rates_setpoint` 给速率环；从机的 `ControlAllocator` 在力矩/推力分配前再叠加编队定制。

因此：
- 从机 `roll/pitch` 姿态设定值直接跟随主机的姿态设定（`roll_target / pitch`），从机不再做摇杆重映射；
- `pitch` 通道额外叠加一项铰链运动学前馈：`side_sign * FORM_HINGE_K * roll_rate_target`，用于补偿编队滚转时的翼尖铰链几何耦合；
- 从机姿态设定的航向分量保持**自身当前航向**（不接收航向位置指令），偏航运动由 `yaw_sp_move_rate = msg.yaw` 与滚转协调转弯共同决定；
- 从机推力直接透传主机推力设定；转弯外侧增推与滚转→俯仰力矩混控均移到 `ControlAllocator` 中完成；
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
- 从机航向保持自身，仅通过 `yaw_sp_move_rate` 接收偏航速率意图；固定翼从机的实际转弯由 `fw_att_control` 的协调转弯控制器按滚转生成（详见 5.3 节），与主机滚转同源；
- 从机推力透传主机推力设定：主机在自主模式（TECS 推力输出）或手动模式（油门映射）下，从机都能正确跟随；
- 主机侧仍保留一项偏航同步加速（`FixedwingRateControl` 内，仅主机且 `UAVCAN_PUB_FORM=1` 时生效）：

```text
master_thrust = thrust_body[0] + 0.5 * abs(manual.yaw) * FORM_YAW_K
```

### 4.3 分配器执行级补偿（从机）

从机 `ControlAllocator` 在控制设定向量 `c = [τx, τy, τz, Tx, Ty, Tz]` 组装完成后、`allocate()` 执行前做两项补偿：

```cpp
// src/modules/control_allocator/ControlAllocator.cpp
if (_is_follower) {
    // 1) 铰链耦合：把部分滚转力矩引入俯仰通道（左右从机符号相反）
    c[0](1) += c[0](0) * _roll_to_pitch_mix * _side_sign;
    // 2) 转弯外侧增推：按偏航力矩设定单侧截断叠加推力（只增不减）
    c[0](3) = math::constrain(c[0](3) + math::max(_side_sign * c[0](2), 0.f) * _yaw_throttle_gain, 0.f, 1.f);
}
```

- `_roll_to_pitch_mix` 来自 `CA_R2P_K`（滚转→俯仰混控增益）：从机滚转时升降舵联动一个俯仰修正，用于补偿铰链传递的结构耦合载荷；
- `_yaw_throttle_gain` 来自 `FORM_YAW_K`（外侧增推系数）：转弯时按从机的偏航力矩设定（`τz`）给**外侧**从机加推（左转时右机增推、右转时左机增推——`side_sign` 与 `τz` 乘积单侧截断自动选出外侧机），内侧机不动作；
- 力矩混控的左右符号由 `side_sign` 自动镜像，无需给左右从机分别配置相反系数；
- 主机（`FORM_FOLLOWER_EN=0`）下这两项不生效；第二分配实例（多矩阵场景）未包含该定制，当前三机均为单一固定翼有效性矩阵，不受影响。

## 5. 代码流程

### 5.1 主机端：formation_rates_sender

初始化时：
- 启动 300 Hz 定时器

周期执行时：
1. 读取 `vehicle_attitude_setpoint`（主机姿态设定），超过 500 ms 未更新则停止发送
2. 读取 `vehicle_status`，仅在固定翼模式或过渡模式下继续工作
3. 读取 `vehicle_rates_setpoint`（含期望滚转速率 `rates_sp.roll`）
4. 反解 `att_sp.q_d` 得到滚转/俯仰设定角 `euler_sp.phi() / euler_sp.theta()`
5. 按 `nav_state` 选择偏航指令源（手动模式取 `rates.yaw`，自主模式取 `att_sp.yaw_sp_move_rate`；非有限值时回退）
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
9. 由 `FixedwingAttitudeControl` 将姿态设定值转为 `vehicle_rates_setpoint`
10. 由 `FixedwingRateControl` 生成力矩/推力设定，再由 `ControlAllocator` 叠加从机定制后解算执行

### 5.3 下游控制链

```text
FormationRatesBridge
  -> offboard_control_mode(attitude=true)
  -> vehicle_attitude_setpoint (q_d / yaw_sp_move_rate / thrust_body)
  -> Commander 进入/维持 Offboard attitude 控制链
  -> FixedwingAttitudeControl：从 q_d 取 roll/pitch 生成速率设定；
     偏航由协调转弯控制器按滚转生成（不消费 q_d 的 yaw 分量与 yaw_sp_move_rate）
  -> FixedwingRateControl 生成 vehicle_torque_setpoint / vehicle_thrust_setpoint
  -> ControlAllocator（从机）：叠加滚转→俯仰混控与外侧增推后解算到舵面/电机
```

注意：当前 `fw_att_control` 在 attitude 模式下**只消费 `q_d` 的 roll/pitch 分量**，从机偏航速率实际由协调转弯控制器按滚转设定生成（`r_sp ≈ tan(φ)·cos(θ)·g/V`），不读取 `yaw_sp_move_rate`。因此从机实际转弯由滚转主导、与主机同源；`yaw_sp_move_rate` 写入姿态设定属于预留通道，待后续为固定翼偏航前馈打补丁后才能生效。

速率内环可选启用 LADRC（`FW_ADRC_EN=1`，仅替换滚转/俯仰，见 5.4 节）；偏航与其余链路不受影响。

Offboard 丢失保护由 PX4 原生参数 `COM_OF_LOSS_T` 控制，当前实现没有额外的编队私有超时参数。

### 5.4 速率内环 LADRC（可选）

`FW_ADRC_EN=1` 时，`FixedwingRateControl` 的**滚转/俯仰速率内环由 PID 切换为一阶线性自抗扰控制（LADRC1）**，偏航轴保持 PID，原 PID 代码完整保留（置 0 即回退）。速率设定值随姿态外环/空速时变，因此控制器含 TD 指令平滑与导数前馈；对链翼构型，翼尖铰链耦合载荷计入"总扰动"，由 ESO 估计并实时抵消，无需显式建模。关联仿真：`docs/reference/LADRC1.m`。

```text
被控对象 (每个轴, 相对阶 1):   ṗ = f + b0·u
    f 为总扰动 (气动非线性、轴间耦合、铰链载荷、阵风等); u 为控制量 (域与 PID 输出相同)

① TD (跟踪微分器, 常开):      ṙ* = λ·(r − r*), 输出 r* 与 ṙ*
② ESO (2 阶扩张状态观测器):   e  = z1 − y
                              ż1 = z2 − β1·e + b0·u_prev
                              ż2 = −β2·e        (z1 → 角速率估计, z2 → 总扰动估计 f)
③ SEF + ④ 扰动补偿:           u = (ωc·(r* − z1) + ṙ* − z2) / b0

带宽参数化 (Gao, ACC 2003):  β1 = 2ωo, β2 = ωo², kp = ωc（ωo 固定取 4×ωc）;  闭环等价于极点 −ωc 的一阶惯性环节。
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
```

关键工程点：

- **影子运行**：`FW_ADRC_EN=0` 时 LADRC 仍每拍计算（不接管输出），保证飞行中随时切换无跳变；
- **反饱和**：ESO 模型输入使用限幅后的"实际施加值"（trim 项由 z2 自适应吸收）；
- **复位**：`rates_sp.reset_integral`、落地、非固定翼状态、控制量非有限时，ESO/TD 热启动复位（避免初始瞬态）；
- **日志**：ADRC 模式下 `rate_ctrl_status.rollspeed_integ / pitchspeed_integ` 记录 z2 扰动估计（偏航字段仍为 PID 积分）；
- tailsitter 场景自动回退 PID；偏航轴、姿态外环、前馈/trim/空速缩放逻辑均未改动；
- 状态：2026-09-12 编译验证通过（cuav_7-nano），未实飞。

## 6. 实际生效参数

### 6.1 主机参数（中央机，Node ID=1）

| 参数 | 说明 |
| ---- | ---- |
| `UAVCAN_ENABLE` | UAVCAN 使能，通常设为 `3` |
| `UAVCAN_NODE_ID` | 本机节点 ID，主机固定为 `1` |
| `UAVCAN_PUB_FORM` | 发送器使能，设为 `1` |
| `FORM_FOLLOWER_EN` | 主机应设为 `0`，用于区分主机/从机身份 |
| `FORM_YAW_K` | 主机偏航同步加速系数（`FixedwingRateControl` 内，仅主机生效） |

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
| `FORM_HINGE_K` | 1.0 | 铰链修正前馈增益（单位：s）：`pitch += side_sign * FORM_HINGE_K * 主机期望滚转速率` |
| `FORM_ROLL_AMAX` | 0.52 | 从机滚转姿态设定限幅（rad） |
| `FORM_PTCH_AMAX` | 0.35 | 从机俯仰姿态设定限幅（rad，含铰链修正） |
| `FORM_YAW_K` | 0.3 | 分配器外侧增推系数：`thrust += max(side_sign * τz, 0) * FORM_YAW_K` |

分配器参数（`Control Allocation` 组，仅从机生效）：
| 参数 | 默认 | 说明 |
| ---- | ---- | ---- |
| `CA_R2P_K` | 0.0 | 滚转→俯仰力矩混控增益：`τy += τx * CA_R2P_K * side_sign`（左右符号自动镜像） |

其他：
| 参数 | 说明 |
| ---- | ---- |
| `COM_OF_LOSS_T` | Offboard 丢失超时，使用 PX4 原生机制 |

### 6.3 速率内环参数（`FW ADRC` 组，可选）

`FW_ADRC_EN=1` 时启用（结构与代码见 5.4 节）：

| 参数 | 默认值 | 单位 | 说明 |
| ---- | ---- | ---- | ---- |
| `FW_ADRC_EN` | 0 | — | 0 = PID（默认）；1 = LADRC 接管滚转/俯仰 |

| `FW_ADRC_B0_R` | 25.0 | rad/s² | 滚转模型增益 b0 |
| `FW_ADRC_WC_R` | 10.0 | rad/s | 滚转控制器带宽 ωc（闭环极点 −ωc） |

| `FW_ADRC_B0_P` | 20.0 | rad/s² | 俯仰模型增益 b0 |
| `FW_ADRC_WC_P` | 8.0 | rad/s | 俯仰控制器带宽 ωc |

TD 带宽固定 40 rad/s（`LADRC1` 类内 `kTdLambda` 常量）；观测器带宽固定为 4×ωc（`LADRC1` 类内 `kWoRatio` 常量，Gao 建议 3~5×）：两者均不参数化（带宽参数化的本意即把可调量压缩为 b0 + 一个带宽），如需调整修改常量后重新编译。

**注意参数耦合**：`WC`（ωc）是唯一的“总强度”旋钮，一个参数同时决定
- 比例增益：kp = ωc；
- 观测器（扰动估计）速度：ωo = 4×ωc；
- “隐式积分”强度（*z2* 通道，承担原 PID *积分角色*）：β2 = ωo² = 16×ωc²，与 ωc 平方相关。

b0 定义：单位归一化力矩指令（±1）在配平空速下产生的角加速度：

```text
b0_R = q̄·S·b·Clδa/Ixx × δa_max ≈ 25 rad/s²   (滚转: 力臂=翼展 b, 惯量=Ixx, 效能=副翼)
b0_P = q̄·S·c·Cmδe/Iyy × δe_max ≈ 20 rad/s²   (俯仰: 力臂=弦长 c, 惯量=Iyy, 效能=升降舵)
```

滚转/俯仰是两套物理通道（力臂、惯量、舵面效能均不同），所以 b0 分轴给定、数值不同；偏航未换（仍 PID），没有 b0。默认值按 VLM 气动数据 + "力矩指令 ±1 ≈ 全舵面行程（约 ±25°）"估算，实机请核对舵面行程后微调。

**调参**顺序（重要性排序）：

1. b0：b0 *偏小* → 算出的舵量偏大 → 等效增益高 → 容易*振荡*；b0 *偏大* → 响应*迟钝*。b0 不准还会折算进等效积分强度（Ki ∝ β2/b0）：b0 偏大时积分被“稀释”，稳态偏差收敛与抗扰变差。
2. ωc（控制器带宽）：*振荡就降、跟踪慢就升*；需明显快于姿态外环带宽。P、观测速度、隐式积分（∝ωc²）随它一起变——若响应已够快但稳态偏差收敛慢/抗扰不足，说明缺的是积分而非比例，见第 3 条。
3. 观测器带宽比（一般不动）：ωo 固定为 4×ωc。仅当需要*单独加强/减弱扰动抑制（积分）、又不想改 P 与响应速度时*，修改 `kWoRatio`（保持 Gao 建议区间 3~5）后重新编译：调大 → 扰动估计更快、隐式积分更强，但对速率测量噪声更敏感。

启用建议：先以默认 `FW_ADRC_EN=0` 验证基础行为，再地面站设为 1、小幅滚转/俯仰打杆观察（出现振荡降 `WC` 或核对 `B0`），通过后再进行常规科目与编队联调；闭环验证见 6.6 节。

### 6.4 参数变更说明（相对旧版）

本版改造后，以下旧参数已**从固件中删除**（QGC 中不再出现，历史保存值失效）：

`FORM_R2P_GAIN`、`FORM_RLEV_THR`、`FORM_RLEV_K`、`FORM_ROLL_FF / KP / KD`、`FORM_PITCH_FF / KP / KD`、`FORM_YAW_FF / KP / KD`、`FORM_ROLL_RMAX / PTCH_RMAX / YAW_RMAX`、`FORM_PITCH_SYNC`、`FORM_YAW_SYNC`。

对应的旧功能已由新架构替代：

- 遥控器映射 → 主机发送姿态设定（从机不再需要摇杆输入）；
- 相对姿态辅助（FF/KP/KD）→ 从机直接跟随主机设定 + 铰链前馈；
- 滚转回正保护 → 由姿态设定限幅（`FORM_ROLL_AMAX / FORM_PTCH_AMAX`）与主机自身控制保障；
- 俯仰同步/滚转耦合 → 铰链修正 `FORM_HINGE_K` + 分配器 `CA_R2P_K`。

⚠️ 升级固件后请检查 `CA_SV_CSx_TRQ_R`（升降舵滚转力矩系数）：若之前在 QGC 中设置过非零值，需清零，避免与 `CA_R2P_K` 双重叠加。

### 6.5 推荐配置示例

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

param set FORM_HINGE_K 1.0
param set FORM_ROLL_AMAX 0.52
param set FORM_PTCH_AMAX 0.35
param set FORM_YAW_K 0.3
param set CA_R2P_K 0.0
param set COM_OF_LOSS_T 1.0

param set FW_ADRC_EN 1
param set FW_ADRC_B0_R 25.0
param set FW_ADRC_WC_R 10.0
param set FW_ADRC_B0_P 20.0
param set FW_ADRC_WC_P 8.0
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

param set FORM_HINGE_K 1.0
param set FORM_ROLL_AMAX 0.52
param set FORM_PTCH_AMAX 0.35
param set FORM_YAW_K 0.3
param set CA_R2P_K 0.0
param set COM_OF_LOSS_T 1.0

param set FW_ADRC_EN 1
param set FW_ADRC_B0_R 25.0
param set FW_ADRC_WC_R 10.0
param set FW_ADRC_B0_P 20.0
param set FW_ADRC_WC_P 8.0

param save
reboot
```

说明：
- `CA_R2P_K` 建议先从 `0.0` 开始试飞（先验证基础跟随），确认正常后再逐步加入滚转→俯仰耦合；
- `FORM_HINGE_K` 初始 `1.0`（等效 1 s 几何时间尺度），按试飞中铰链过渡过程的从机俯仰跟随效果调整；
- 从机进入 Offboard 后，接收器才会继续发布 `vehicle_attitude_setpoint`，由 CAN 控制链正式接管；随后 `FixedwingAttitudeControl` 会继续生成 `vehicle_rates_setpoint` 给速率环。
- 主机不走从机的 Offboard 控制链，而是在 `FixedwingRateControl.cpp` 内直接对自身推力输出叠加偏航同步加速。
- 速率内环 LADRC：从机示例已按 `FW_ADRC_EN=1` 配置（左右从机一起开、保持对称），主机暂为 `0`（保留 PID 作为基准，从机验证通过后可再置 1）；`B0 / WC` 为按仿真机型估算的初值（ωo 固定 4×ωc，见 6.3 节），实机请按各自舵面行程核对。

### 6.6 传输验证方法

可以用 PX4 shell 里的 `listener` 配合 `uavcan status` 做链路验证。
需要注意：`listener` 观察的是 PX4 内部 uORB 主题，不是直接抓原始 CAN 帧；因此它更适合验证"消息已经被正确接收并转换为控制量"。

`主机侧`建议先确认期望控制姿态源正常：

```bash
listener vehicle_attitude_setpoint -r 5 -n 20
listener vehicle_rates_setpoint -r 5 -n 20
```

手动模式下拨动遥控器、或在自主模式下执行任务时，应能看到 `att_sp.q_d`、`thrust_body[0]` 与 `rates_sp.roll` 持续更新。

`从机侧`建议依次检查：

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

此时应能看到 `q_d` 与 `thrust_body[0]` 持续更新，并满足：
- 从机 `q_d` 的滚转分量跟随主机姿态设定（`roll_target`）；
- 从机 `q_d` 的俯仰分量 = 主机俯仰设定 + 铰链修正（左右从机符号相反，随主机滚转速率变化）；
- `thrust_body[0]` 与主机推力设定一致。

如果还想继续验证姿态环到速率环的下游转换，可以再执行：

```bash
listener vehicle_rates_setpoint -r 5 -n 20
```

此时应能看到 `fw_att_control` 根据 `vehicle_attitude_setpoint` 继续生成的 `roll/pitch/yaw/thrust_body[0]` 更新（偏航通道由协调转弯按滚转生成）。

如已启用速率内环 LADRC（`FW_ADRC_EN=1`），可进一步检查速率跟踪：

```bash
listener vehicle_angular_velocity -r 5 -n 20
listener vehicle_rates_setpoint -r 5 -n 20
```

观察角速率跟随速率设定的相位与幅值；`rate_ctrl_status` 的 `rollspeed_integ / pitchspeed_integ` 此时为 LADRC 扰动估计 z2（偏航字段仍为 PID 积分），可用于判断阻尼与耦合载荷的量级。

如果 `listener vehicle_attitude_setpoint` 没有更新，优先检查：
- 主机是否已开启 `UAVCAN_PUB_FORM=1`
- 从机是否已开启 `UAVCAN_SUB_FORM=1` 和 `FORM_FOLLOWER_EN=1`
- 左右从机 `FORM_POSITION` 是否分别设为 `1/2`
- 主从机 `UAVCAN_NODE_ID` 是否冲突
- 从机当前是否已经切入 Offboard
- 主从机固件是否为同一版本（消息格式在本次改造后已变更，新旧固件不兼容）
