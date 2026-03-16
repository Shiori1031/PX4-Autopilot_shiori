# UAVCAN 编队速率控制功能记录

## 1. 功能概述

为 PX4 新增了**编队速率控制功能**，用于三机刚性连接编队飞行系统。中央主飞控通过 UAVCAN 总线向左右从飞控发送角速率设定值，从飞控通过 Offboard 模式执行，实现"整机当副翼"的协同控制。

**核心机制：** 主机滚转输入 → 从机俯仰速率指令 → 整机升力差 → 编队滚转力矩

核心构成如下：

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 消息定义 | `msg/FormationRatesSetpoint.msg` | 编队速率设定值 uORB 消息（向后兼容，可选使用） |
| **主机发送器** | `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}` | **集成解算+发送**：订阅遥控器输入，内部解算左右从机速率指令，通过 UAVCAN 广播 |
| UAVCAN 接收器 | `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}` | 接收 UAVCAN 速率指令，**直接发布控制指令**，自动生效 |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c` | 解算参数（FORM_R2P_GAIN、FORM_ROLL_COMP 等）统一在 UAVCAN 参数文件中 |
| 自动启动 | UAVCAN 驱动自动初始化 | 无需手动启动任何模块！ |


## 2. 传输的数据

### 2.1 UAVCAN 消息格式
复用标准的 `uavcan::equipment::actuator::ArrayCommand` 消息，使用特殊的 actuator_id 编码：

| Actuator ID | 数据类型 | 含义 | 命令类型 |
| ----------- | -------- | ---- | -------- |
| 100 | float | Roll rate（滚转角速度，rad/s） | COMMAND_TYPE_SPEED |
| 101 | float | **Pitch rate（俯仰角速度，rad/s）** | COMMAND_TYPE_SPEED |
| 102 | float | Yaw rate（偏航角速度，rad/s） | COMMAND_TYPE_SPEED |
| 103 | float | Thrust_X（前向推力，0-1） | COMMAND_TYPE_UNITLESS |
| 104 | float | Thrust_Y（横向推力，0-1） | COMMAND_TYPE_UNITLESS |
| 105 | float | Thrust_Z（垂向推力，0-1） | COMMAND_TYPE_UNITLESS |
| 110 | uint8 | Formation position（0=中央/1=左/2=右） | COMMAND_TYPE_UNITLESS |

**注意：**
- 使用 actuator_id 100-110 范围，避免与标准舵机通道（0-15）冲突
- Pitch rate（actuator_id 101）是从机的主要控制通道
- 传输频率：200Hz

### 2.2 uORB 消息内容
`FormationRatesSetpoint` 包含：
- `timestamp`: 指令生成时间戳（微秒）
- `timestamp_sample`: 控制数据采样时间戳（微秒）
- `source_node_id`: 源节点 ID（主飞控，通常为 1）
- `target_node_id`: 目标节点 ID（2=左机/3=右机）
- `roll`: 滚转角速度（rad/s）
- `pitch`: **俯仰角速度（rad/s）** ← 从机主要控制通道
- `yaw`: 偏航角速度（rad/s）
- `thrust_body[3]`: 机体坐标系推力 [前/右/下]，归一化（0-1）
- `reset_integral`: 复位积分器标志
- `formation_position`: 编队位置（0=中央/1=左/2=右）

### 2.3 控制映射关系

**机械结构前提：**
三机通过刚性连杆连接，连接处允许相对俯仰（铰链式），但不允许相对滚转。因此：
- 从机 roll 通道接收主机滚转指令，并在接收端通过相对姿态补偿形成闭环
- 从机**通过整体俯仰充当编队的"副翼"**——升降舵偏转改变本机升力，左右升力差产生编队滚转力矩

**核心算法：**
```
从机 pitch_rate = ±主机 roll × FORM_R2P_GAIN + 主机 pitch × FORM_PITCH_SYNC
从机 thrust    = 主机油门 ± max/min(主机 yaw, 0) × FORM_THR_DIFF //把主机分为左右轴，由左右定加推力，不减小
从机 yaw_rate  = 主机 yaw ± 主机 roll × FORM_YAW_K
发送 roll_cmd_tx = 主机 roll
从机 roll_rate_sp = FORM_ROLL_COMP × (roll_cmd_tx - 从机自身roll)
```

#### 2.3.1 编队滚转（核心控制）

主机飞手打横滚杆 → 左右从机俯仰方向相反 → 升力差 → 编队整体滚转

| 主机输入          | 左机 pitch             | 右机 pitch              | 物理效果 |
| --------         | ----------             | ----------             | -------- |
| Roll 右（+0.5） | +1.0 rad/s（抬头加升力） | -1.0 rad/s（低头减升力） | 编队右滚 |
| Roll 左（-0.5） | -1.0 rad/s（低头减升力） | +1.0 rad/s（抬头加升力） | 编队左滚 |
| Roll 中（0）    | 0（中立）                | 0（中立）              | 直飞不偏 |

关键参数：`FORM_R2P_GAIN`（默认 2.0），值越大升降舵偏转越大，滚转响应越快。

#### 2.3.2 编队俯仰（三机同步）

主机飞手拉推俯仰杆 → 三机一起抬头/低头。因为连接允许相对俯仰，从机不会自动跟随主机俯仰，需要通过 UAVCAN 同步。

| 主机输入 | 左机 pitch | 右机 pitch | 主机自身 |
| -------- | ---------- | ---------- | -------- |
| Pitch上拉 | += `pitch × FORM_PITCH_SYNC` | += `pitch × FORM_PITCH_SYNC` | 自行控制 |
| Pitch下推 | -= `pitch × FORM_PITCH_SYNC` | -= `pitch × FORM_PITCH_SYNC` | 自行控制 |

关键参数：`FORM_PITCH_SYNC`（默认 0.1），值越大从机跟随俯仰越紧密。

注意：俯仰同步与滚转映射叠加在同一个 pitch 通道，最终值 = 滚转映射 + 俯仰同步。

#### 2.3.3 编队转弯（偏航差速 + 协调偏航）

转弯时外侧机线速度更大，需更多推力；同时添加偏航耦合防止侧滑。差速与偏航输入相关，未朝对应方向偏航时不减推力。

| 主机输入 | 左机推力 | 右机推力 | 说明 |
| -------- | -------- | -------- | ---- |
| Yaw 右（+0.5） | 基础 + 0.025 | 基础 | 左机增加推力 |
| Yaw 左（-0.5） | 基础 | 基础 + 0.025 | 右机增加推力 |
| Yaw 中（0） | 基础 | 基础 | 直飞，三机推力同步 |

偏航耦合同理，`FORM_YAW_K` 控制偏航随横滚的比例，防止转弯侧滑。

#### 2.3.4 推力同步（直飞）

主机油门量（`manual.throttle`）直接作为从机的基础推力。三机电机同步运转，无横滚输入时推力完全一致。

```
从机 thrust = (主机 throttle + 1.0) × 0.5  （遥控器 -1~1 映射到 0~1）
```

#### 2.3.5 综合映射公式

**左机完整指令：**
```
roll_cmd_tx = manual.roll
roll_rate_sp = FORM_ROLL_COMP × (roll_cmd_tx - self_roll)
pitch_rate = manual.roll × FORM_R2P_GAIN + manual.pitch × FORM_PITCH_SYNC
yaw_rate   = manual.yaw  + (-manual.roll × FORM_YAW_K)
thrust     = (manual.throttle + 1) / 2 + max(manual.yaw, 0) × FORM_THR_DIFF
```

**右机完整指令：**
```
roll_cmd_tx = manual.roll
roll_rate_sp = FORM_ROLL_COMP × (roll_cmd_tx - self_roll)
pitch_rate = -manual.roll × FORM_R2P_GAIN + manual.pitch × FORM_PITCH_SYNC
yaw_rate   = manual.yaw  + (+manual.roll × FORM_YAW_K)
thrust     = (manual.throttle + 1) / 2 - min(manual.yaw, 0) × FORM_THR_DIFF
```

注意 pitch_rate 中滚转映射**左右相反**（一个抬头一个低头），而俯仰同步**左右相同**（三机一起抬头）。

## 3. 主要代码文件

- **`msg/FormationRatesSetpoint.msg`**：uORB 消息定义，包含角速率、推力、编队位置标识（向后兼容）
- **`src/drivers/uavcan/formation_rates_sender.{hpp,cpp}`**：**主机集成发送器**，订阅遥控器输入，内部解算左右从机速率指令，通过 UAVCAN 广播
- **`src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}`**：**从机 UAVCAN 接收器**，解析 ArrayCommand，直接发布 vehicle_rates_setpoint + offboard_control_mode
- **`src/drivers/uavcan/uavcan_params.c`**：所有编队参数（FORM_R2P_GAIN、FORM_ROLL_COMP、FORM_YAW_K、FORM_FOLLOWER_EN、FORM_POSITION 等）

## 4. 参数配置

### 4.1 必需参数

**主飞控（中央，Node ID=1）：**

| 参数 | 类型 | 默认值 | 说明 |
| ---- | ---- | ------ | ---- |
| `UAVCAN_ENABLE` | INT32 | 0 | UAVCAN 使能，设为 2 或 3 |
| `UAVCAN_NODE_ID` | INT32 | 1 | 本机节点 ID，**必须为 1** |
| `UAVCAN_PUB_FORM` | INT32 | 0 | **编队控制发送器使能，设为 1**（启用发送器模块） |
| `FORM_MASTER_EN` | INT32 | 0 | 主机控制器使能（预留参数，当前版本未使用） |
| `FORM_R2P_GAIN` | FLOAT | 2.0 | **Roll→Pitch 映射增益**（关键参数，推荐 1.5-3.0） |
| `FORM_YAW_K` | FLOAT | 0.3 | 偏航耦合系数（协调转弯） |
| `FORM_THR_DIFF` | FLOAT | 0.05 | 油门差异（外侧/内侧补偿） |
| `FORM_PITCH_SYNC` | FLOAT | 0.1 | 俯仰同步系数（队形保持） |


**从飞控（左机/右机）：**

| 参数 | 类型 | 默认值 | 说明 |
| ---- | ---- | ------ | ---- |
| `UAVCAN_ENABLE` | INT32 | 0 | UAVCAN 使能，设为 2 或 3 |
| `UAVCAN_NODE_ID` | INT32 | - | **左机=2，右机=3** |
| `UAVCAN_SUB_FORM` | INT32 | 0 | **编队控制订阅使能，设为 1**（启用接收器模块） |
| `FORM_FOLLOWER_EN` | INT32 | 0 | **编队跟随器内部使能，设为 1**（接收器处理消息开关） |
| `FORM_ROLL_COMP` | FLOAT | 2.0 | roll 相对误差补偿增益（用于 `roll = gain × (cmd_roll - self_roll)`） |
| `FORM_POSITION` | INT32 | 0 | **编队位置**（1=左/2=右），**必须正确设置** |
| `FORM_TIMEOUT` | FLOAT | 0.5 | 指令超时时间（秒） |
| `COM_OF_LOSS_T` | FLOAT | 1.0 | Offboard 超时时间（秒） |

### 4.2 配置示例

**中央主飞控（节点 ID=1）：**
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_FORM 1          # 启用编队控制发送器
param set FORM_R2P_GAIN 2.0          # 滚转到俯仰映射增益
param set FORM_YAW_K 0.3             # 偏航耦合系数
param set FORM_THR_DIFF 0.05         # 油门差异
param set FORM_PITCH_SYNC 0.1        # 俯仰同步系数
param save
reboot

# UAVCAN 驱动自动初始化发送器，无需手动启动
```

**左侧从飞控（节点 ID=2）：**
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 2
param set UAVCAN_SUB_FORM 1          # 启用编队控制接收器（外层开关）
param set FORM_FOLLOWER_EN 1         # 启用跟随器消息处理（内层开关）
param set FORM_ROLL_COMP 2.0         # 从机滚转补偿增益
param set FORM_POSITION 1            # 左机
param set FORM_TIMEOUT 0.5           # 指令超时时间
param save
reboot

# 无需手动启动任何模块！
# 当收到编队指令时，自动切换到 Offboard 模式：
commander mode offboard              # 通过 MAVLink Shell 或 QGC
```

**右侧从飞控（节点 ID=3）：**
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 3
param set UAVCAN_SUB_FORM 1          # 启用编队控制接收器（外层开关）
param set FORM_FOLLOWER_EN 1         # 启用跟随器消息处理（内层开关）
param set FORM_ROLL_COMP 2.0         # 从机滚转补偿增益
param set FORM_POSITION 2            # 右机
param set FORM_TIMEOUT 0.5           # 指令超时时间
param save
reboot

# 无需手动启动任何模块！
# 切换到 Offboard 模式即可：
commander mode offboard
```

### 4.3 无 GPS 配置（从机测试用）

**如果从机没有 GPS 模块，需要额外配置以下参数来绕过位置估计检查：**

```bash
# 从机端额外配置（仅用于测试/开发环境）
param set COM_ARM_WO_GPS 1           # 允许无 GPS 解锁
param set CBRK_VELPOSERR 501090426   # 禁用速度/位置错误检查（断路器）
param set COM_POS_FS_EPH 100         # 放宽位置估计健康阈值（米）
param set COM_POS_FS_GAIN 0          # 禁用位置丢失故障保护
param set EKF2_HGT_REF 0             # 高度参考源：0=气压计
param set EKF2_GPS_CHECK 0           # 禁用 GPS 质量检查

# 对于固定翼，可能还需要：
param set FW_ARSP_MODE 0             # 禁用空速传感器要求（如果没有）

param save
reboot
```

**⚠️ 重要安全提示：**
- 这些设置会**降低安全性**，仅适用于地面测试或受控环境
- 编队速率控制（body_rate 模式）只需要姿态估计，不需要位置信息
- 确保 IMU（陀螺仪/加速度计）和气压计工作正常
- 生产环境建议使用 GPS 或其他定位系统（光流、UWB 等）
- `CBRK_VELPOSERR=501090426` 是紧急断路器，绕过了关键安全检查

**验证配置是否生效：**
```bash
# 检查解锁前检查状态
commander status

# 如果仍显示 "not ready"，查看具体原因
commander check

# 查看 EKF 状态
ekf2 status
```


## 5. 验证方法

### 5.1 检查 UAVCAN 总线状态
```bash
uavcan status
```
预期输出：
```
Online nodes (Node ID, Health, Mode):
    1 OK         OPERATIONAL    # 主机
    2 OK         OPERATIONAL    # 左机
    3 OK         OPERATIONAL    # 右机
```

### 5.2 主机端检查编队指令生成
```bash
# 发送器集成在 UAVCAN 驱动中，通过 uavcan status 查看
uavcan status                        # 检查发送器状态
```

### 5.3 从机端监听接收的速率指令
```bash
# 从机端无需启动任何模块，UAVCAN 接收器自动工作
listener vehicle_rates_setpoint      # 查看直接输出的控制指令
listener offboard_control_mode       # 确认 body_rate = true
```
**预期输出示例（左机，主机向左滚转）：**
```
TOPIC: vehicle_rates_setpoint
    timestamp: 123456789
    roll: 0.05
    pitch: 1.0                     # 左机抬头：+1.0 rad/s
    yaw: -0.15
    thrust_body[0]: 0.55           # 外侧机增加推力
```

### 5.4 检查 Offboard 模式状态
```bash
# 从机端
listener offboard_control_mode     # body_rate 应为 true
listener vehicle_rates_setpoint    # 传递给 fw_rate_control 的速率
listener vehicle_status            # 确认 nav_state = 14（Offboard）
```

### 5.5 地面滚转测试
1. 主机移动遥控器横滚摇杆至左侧
2. 观察从机升降舵响应：
   - 左机升降舵下偏（低头）
   - 右机升降舵上偏（抬头）
3. 确认响应方向正确

## 6. 代码流程概述

### 6.1 主机端：formation_rates_sender（发送器）

**文件：** `src/drivers/uavcan/formation_rates_sender.{hpp,cpp}`

**初始化流程：**
UAVCAN 驱动启动时自动创建 `FormationRatesSender` 实例，构造函数中查找参数句柄（FORM_R2P_GAIN 等），`init()` 加载参数并启动 200Hz 定时器。

**周期执行流程（`periodic_update`，200Hz）：**
1. 读取 `manual_control_setpoint`（遥控器输入），数据超过 500ms 未更新则跳过
2. 检查 `vehicle_status`，仅在固定翼模式下工作
3. **计算左机指令**：
   - `roll_cmd_tx = manual.roll`（发送给从机的滚转命令）
   - `pitch_rate = manual.roll × FORM_R2P_GAIN`（核心映射：主机横滚→从机俯仰）
   - `yaw_rate = manual.yaw + (-manual.roll × FORM_YAW_K)`（协调转弯偏航）
   - `thrust = (throttle+1)/2 + max(manual.yaw, 0) × FORM_THR_DIFF`（右偏航时左机加推）
   - `formation_position = 1`（标识为左机）
4. 将以上数据编码为 `ArrayCommand`（actuator_id 100-110），UAVCAN 广播
5. **计算右机指令**：与左机类似但 pitch 和 yaw 耦合方向**取反**，推力按 `-min(manual.yaw, 0)` 处理，`formation_position = 2`
6. 广播右机消息

**关键点：** 左右机 pitch 方向相反，形成升力差产生编队滚转力矩。

### 6.2 从机端：formation_rates（接收器）

**文件：** `src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}`

**初始化流程：**
UAVCAN 驱动启动时检查 `UAVCAN_SUB_FORM` 参数，如果为 1 则创建 `FormationRatesBridge` 实例，注册 `ArrayCommand` 消息的订阅回调。

**接收回调流程（`formation_rates_sub_cb`，每收到消息触发）：**
1. 检查 `FORM_FOLLOWER_EN` 参数（内层开关），未启用则直接返回
2. 读取 `vehicle_attitude`，获取从机自身滚转角 `self_roll`
3. 遍历 `ArrayCommand` 中所有 `Command`，按 `actuator_id` 解码：
   - 100→滚转速率、101→俯仰速率、102→偏航速率
   - 103→前向推力、104→横向推力、105→垂向推力
   - 110→编队位置标识
4. 对比 `formation_position` 与本机 `FORM_POSITION` 参数，不匹配则忽略
5. **发布 `offboard_control_mode`**：设置 `body_rate=true`，维持 Offboard 模式心跳
6. **发布 `vehicle_rates_setpoint`**：
   - `rates_sp.roll = FORM_ROLL_COMP × (接收roll_cmd_tx - self_roll)`（相对滚转闭环补偿后的最终执行值）
   - `pitch/yaw/thrust` 使用接收值

**关键点：** 从机通过 FORM_POSITION 参数过滤消息，只响应属于自己的指令；并对 roll 通道引入相对姿态闭环补偿。Offboard 心跳由接收器自动维持，无需额外模块。

### 6.3 注意事项

- **FORM_POSITION 必须正确**：左机=1，右机=2，错误会导致控制反向或无响应
- **Offboard 模式切换**：从机需切换到 Offboard 模式才会响应编队指令
- **超时保护**：编队指令 500ms 超时（FORM_TIMEOUT），Offboard 模式 1000ms 超时（COM_OF_LOSS_T）
- **增益调试**：
  - `FORM_R2P_GAIN`：1.5-3.0（过小响应弱，过大可能振荡）
   - `FORM_ROLL_COMP`：1.5-3.0（从机 roll 补偿增益，过小跟随慢，过大易振荡）
  - `FORM_YAW_K`：0.2-0.5（观察转弯时侧滑情况调整）
  - `FORM_THR_DIFF`：0.03-0.08（观察转弯时速度差调整）
- **地面测试充分**：移动所有遥控器摇杆，确认从机升降舵响应方向正确

---
