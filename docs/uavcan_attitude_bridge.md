# UAVCAN 姿态与位置互通功能记录

## 1. 功能概述

为 PX4 UAVCAN 驱动新增了 **姿态与位置互通能力**。每台飞控既能把本机姿态和位置广播到 CAN 总线，也能订阅其它节点的姿态和位置并在系统内部以 uORB 话题形式提供给后续模块。

核心构成如下：

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 姿态与位置发布 | `src/drivers/uavcan/attitude.{hpp,cpp}` | 订阅本机 `vehicle_attitude`、角速度、线加速度和 `vehicle_local_position`，封装为 `uavcan::equipment::ahrs::Solution` 定期广播（位置数据通过 `orientation_covariance` 字段传输） |
| 姿态与位置接收 | `src/drivers/uavcan/sensors/ahrs_solution.{hpp,cpp}` | 订阅 CAN 上的 `Solution` 消息并发布到新的 uORB 话题 `uavcan_attitude`（默认支持 8 个不同节点） |
| 数据通道 | `msg/UavcanAttitude.msg` | 定义保存远端姿态/角速度/加速度/位置/速度及来源节点 ID（四元数存储为 PX4 习惯顺序 w,x,y,z） |
| 参数与配置 | `src/drivers/uavcan/uavcan_params.c`、`src/drivers/uavcan/Kconfig`、`src/drivers/uavcan/CMakeLists.txt` | 增加开关参数 `UAVCAN_PUB_ATT`、`UAVCAN_SUB_ATT` 及对应的 Kconfig、编译项 |

该实现无须额外启动新模块，均集成在现有 `uavcan` 驱动中。

## 2. 参数说明

### 2.1 核心参数

| 参数 | 类型 | 默认值 | 取值范围 | 作用 |
| ---- | ---- | ---- | ---- | ---- |
| `UAVCAN_PUB_ATT` | INT32 | 0 | 0 或 1 | **姿态发布开关**。设为 1 时开启本机姿态广播（`uavcan.equipment.ahrs.Solution` 消息），广播频率 100 Hz |
| `UAVCAN_SUB_ATT` | INT32 | 0 | 0 或 1 | **姿态订阅开关**。设为 1 时开启姿态订阅并发布到 `uavcan_attitude` uORB 话题 |
| `UAVCAN_ENABLE` | INT32 | 0 | 0, 1, 2, 3 | **UAVCAN 驱动使能**。0=禁用，1=使能，2=自动枚举（推荐），3=客户端模式。需 ≥1 才能使姿态互通功能生效 |
| `UAVCAN_NODE_ID` | INT32 | 1 | 1-125 | **本机节点 ID**。必须在 CAN 总线上唯一，用于标识姿态消息来源。推荐规划：主飞控=1，备飞控=2，其它传感器节点≥10 |

### 2.2 UAVCAN 基础配置参数

| 参数 | 类型 | 默认值 | 取值范围 | 说明 |
| ---- | ---- | ---- | ---- | ---- |
| `UAVCAN_BITRATE` | INT32 | 1000000 | 20000-1000000 | **CAN 总线波特率**（bps）。常用值：1000000（1 Mbps，推荐）、500000（500 Kbps）、250000（250 Kbps）。两端必须一致 |
| `UAVCAN_ESC_IDLT` | INT32 | 0 | 0-2000 | ESC 怠速 PWM 值（与姿态互通无关） |
| `UAVCAN_RNG_MIN` | FLOAT | 0.3 | 0.0-200.0 | 激光测距仪最小距离（米，与姿态互通无关） |
| `UAVCAN_RNG_MAX` | FLOAT | 200.0 | 0.0-1000.0 | 激光测距仪最大距离（米，与姿态互通无关） |


### 2.3 参数配置方式

**方式一：QGroundControl 图形界面**
1. 连接飞控到 QGC
2. 打开 **车辆设置 → 参数**
3. 搜索 `UAVCAN` 筛选相关参数
4. 修改 `UAVCAN_ENABLE`为3、`UAVCAN_PUB_ATT`为1、`UAVCAN_SUB_ATT`为1、`UAVCAN_NODE_ID`设置不同
5. 点击 **应用并重启**

**方式二：MAVLink 控制台（NSH）**
```bash
# 查看当前参数值
param show UAVCAN_ENABLE
param show UAVCAN_NODE_ID
param show UAVCAN_PUB_ATT
param show UAVCAN_SUB_ATT

# 设置参数
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_ATT 1
param set UAVCAN_SUB_ATT 1
param set UAVCAN_BITRATE 1000000

# 保存参数并重启
param save
reboot
```

### 2.4 配置示例

**场景 1：飞控互通（双向收发）**
```
飞控1（节点 ID=1）：
  UAVCAN_ENABLE    = 3
  UAVCAN_NODE_ID   = 1
  UAVCAN_PUB_ATT   = 1  ← 发布姿态
  UAVCAN_SUB_ATT   = 1  ← 订阅姿态
  UAVCAN_BITRATE   = 1000000

飞控2（节点 ID=2）：
  UAVCAN_ENABLE    = 3
  UAVCAN_NODE_ID   = 2
  UAVCAN_PUB_ATT   = 1  ← 发布姿态
  UAVCAN_SUB_ATT   = 1  ← 订阅姿态
  UAVCAN_BITRATE   = 1000000

飞控3（节点 ID=3）：
  UAVCAN_ENABLE    = 3
  UAVCAN_NODE_ID   = 3
  UAVCAN_PUB_ATT   = 1  ← 发布姿态
  UAVCAN_SUB_ATT   = 1  ← 订阅姿态
  UAVCAN_BITRATE   = 1000000
```

**场景 2：主机发布，备机仅接收（单向传输）**
```
主飞控（节点 ID=1）：
  UAVCAN_ENABLE    = 2
  UAVCAN_NODE_ID   = 1
  UAVCAN_PUB_ATT   = 1  ← 仅发布
  UAVCAN_SUB_ATT   = 0  ← 不订阅
  UAVCAN_BITRATE   = 1000000

备飞控（节点 ID=2）：
  UAVCAN_ENABLE    = 2
  UAVCAN_NODE_ID   = 2
  UAVCAN_PUB_ATT   = 0  ← 不发布
  UAVCAN_SUB_ATT   = 1  ← 仅订阅
  UAVCAN_BITRATE   = 1000000
```
```
注：`ahrs_solution.cpp` 默认支持同时接收最多 8 个不同节点的姿态数据。

### 2.5 参数验证检查表

配置完成后，按以下步骤验证参数是否正确生效：

1. **检查参数持久化**
   ```bash
   param show UAVCAN_*
   ```
   确认所有 `UAVCAN_*` 参数与预期一致。

2. **检查 UAVCAN 驱动运行状态与总线健康**
   ```bash
   uavcan status
   ```
   应显示总线波特率、节点 ID、收发包计数等信息。
   观察 `TX errors`、`RX errors` 应为 0 或极低值，`TX overflow count` 应为 0。


3. **检查姿态和位置接收**（在接收端执行）
   ```bash
   listener uavcan_attitude
   ```
   应显示来自远端节点的姿态和位置数据，`source_node_id` 字段标识来源。
   
   **预期输出示例：**
   ```
   TOPIC: uavcan_attitude
   timestamp: 1234567890
   timestamp_sample: 1234567800
   source_node_id: 2
   q[0]: 1.000  q[1]: 0.001  q[2]: 0.002  q[3]: 0.003
   angular_velocity: [0.01, 0.02, 0.03]
   linear_acceleration: [0.1, 0.2, 9.8]
   xy_valid: true
   z_valid: true
   v_xy_valid: true
   v_z_valid: true
   x: 10.5  y: 5.2  z: -2.3
   vx: 1.2  vy: 0.5  vz: -0.1
   heading: 0.785
   eph: 0.5  epv: 0.3
   ```


## 3. 主要代码

- `msg/UavcanAttitude.msg`：新增 uORB 消息，记录远端姿态、角速度、线加速度、位置、速度、航向及精度，带 `source_node_id` 字段便于区分来源。
- `src/drivers/uavcan/attitude.cpp`：建立定时器周期性读取 uORB 姿态和位置并广播；位置数据通过 `orientation_covariance` 字段（9 个元素）传输，格式为 `[x, y, z, vx, vy, vz, heading, eph, epv]`。
- `src/drivers/uavcan/sensors/ahrs_solution.cpp`：订阅 UAVCAN `Solution` 消息，自动将四元数从 (x, y, z, w) 转回 PX4 的 (w, x, y, z) 顺序，并从 `orientation_covariance` 字段解析位置数据（默认最多缓存 8 个来源节点）。
- `src/drivers/uavcan/uavcan_main.cpp`：在初始化流程中读取 `UAVCAN_PUB_ATT`，条件启动发布器。
- `src/drivers/uavcan/sensors/sensor_bridge.cpp`：在工厂方法中根据 `UAVCAN_SUB_ATT` 创建姿态订阅桥。
- `src/drivers/uavcan/uavcan_params.c`，`src/drivers/uavcan/Kconfig`：增加对应参数、Kconfig 选项。
- `src/drivers/uavcan/CMakeLists.txt`：登记新增源码。

### 3.1 位置数据传输实现说明

由于 UAVCAN 标准的 `uavcan::equipment::ahrs::Solution` 消息类型本身不包含位置字段，本实现采用了在 `orientation_covariance` 数组中嵌入位置数据的方案：

**编码格式（发布端）：**
```
orientation_covariance[0] = x       // NED 北向位置 (m)
orientation_covariance[1] = y       // NED 东向位置 (m)
orientation_covariance[2] = z       // NED 下向位置 (m)
orientation_covariance[3] = vx      // NED 北向速度 (m/s)
orientation_covariance[4] = vy      // NED 东向速度 (m/s)
orientation_covariance[5] = vz      // NED 下向速度 (m/s)
orientation_covariance[6] = heading // 航向角 (rad)
orientation_covariance[7] = eph     // 水平位置误差标准差 (m)
orientation_covariance[8] = epv     // 垂直位置误差标准差 (m)
```

**注意事项：**
- 该方案是一种工程权衡，利用了协方差字段的扩展能力
- 所有参与互通的飞控必须使用相同版本的代码以确保数据格式一致
- 位置数据采用 NED（北-东-地）坐标系，原点为各飞控本地位置估计的参考点
