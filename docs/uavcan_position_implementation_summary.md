# UAVCAN 姿态与全球位置互通功能记录

## 1. 功能概述

为 PX4 UAVCAN 驱动新增了 **姿态与全球位置互通能力**。每台飞控既能把本机姿态和 GPS 位置广播到 CAN 总线，也能订阅其它节点的姿态和位置并在系统内部以 uORB 话题形式提供给后续模块。

核心构成如下：

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 姿态与位置发布 | `src/drivers/uavcan/attitude.{hpp,cpp}` | 订阅本机 `vehicle_attitude`、角速度、线加速度和 `vehicle_global_position`，封装为 `uavcan::equipment::ahrs::Solution` 定期广播（GPS 位置数据通过 `orientation_covariance` 字段传输） |
| 姿态与位置接收 | `src/drivers/uavcan/sensors/ahrs_solution.{hpp,cpp}` | 订阅 CAN 上的 `Solution` 消息并发布到新的 uORB 话题 `uavcan_attitude`（默认支持 8 个不同节点） |
| 数据通道 | `msg/UavcanAttitude.msg` | 定义保存远端姿态/角速度/加速度/GPS位置/精度及来源节点 ID（四元数存储为 PX4 习惯顺序 w,x,y,z） |
| 参数与配置 | `src/drivers/uavcan/uavcan_params.c` 等 | 复用开关参数 `UAVCAN_PUB_ATT`、`UAVCAN_SUB_ATT` 控制发布和订阅 |

该实现无须额外启动新模块，均集成在现有 `uavcan` 驱动中。

## 2. 传输的数据

### 2.1 标准 UAVCAN 字段
- **四元数姿态**：4个float，从 `vehicle_attitude` 获取，PX4格式 [w,x,y,z] 转换为 UAVCAN格式 [x,y,z,w]
- **角速度**：3个float (rad/s)，从 `vehicle_angular_velocity` 获取
- **线性加速度**：3个float (m/s²)，从 `vehicle_acceleration` 获取
- **时间戳**：uint64 (微秒)，数据采样时刻
- **节点ID**：uint8，发送飞控的 CAN 节点标识

### 2.2 GPS 全球位置数据（嵌入在 orientation_covariance 字段）
由于 UAVCAN 标准的 `Solution` 消息不包含位置字段，本实现在 `orientation_covariance` 数组（9个元素）中嵌入 GPS 数据：

```
orientation_covariance[0] = lat     // 纬度（度，-90~+90，WGS84）
orientation_covariance[1] = lon     // 经度（度，-180~+180，WGS84）
orientation_covariance[2] = alt     // 海拔高度 AMSL（米）
orientation_covariance[3] = 0.0f    // 预留（未来可用于速度）
orientation_covariance[4] = 0.0f    // 预留
orientation_covariance[5] = 0.0f    // 预留
orientation_covariance[6] = eph     // 水平位置精度（米）
orientation_covariance[7] = epv     // 垂直位置精度（米）
orientation_covariance[8] = 0.0f    // 预留
```

**注意**：
- 所有飞控必须使用相同版本代码以确保数据格式一致
- 使用 WGS84 全球坐标系，无需统一本地参考点
- 单精度浮点传输，精度约 1.1 米（满足大多数编队飞行需求）

## 3. 主要代码文件

- **`msg/UavcanAttitude.msg`**：定义 uORB 消息，记录远端姿态、GPS 位置、精度及来源节点 ID
- **`src/drivers/uavcan/attitude.cpp`**：周期性读取本机姿态和 GPS 位置（`vehicle_global_position`）并通过 UAVCAN 广播，100Hz 频率
- **`src/drivers/uavcan/sensors/ahrs_solution.cpp`**：订阅 UAVCAN `Solution` 消息，解析姿态和 GPS 数据，发布到 `uavcan_attitude` 话题（默认支持 8 个来源节点）
- **`src/drivers/uavcan/uavcan_main.cpp`**：根据 `UAVCAN_PUB_ATT` 参数启动发布器
- **`src/drivers/uavcan/sensors/sensor_bridge.cpp`**：根据 `UAVCAN_SUB_ATT` 参数创建订阅桥

## 4. 参数配置

### 4.1 必需参数

| 参数 | 类型 | 默认值 | 说明 |
| ---- | ---- | ------ | ---- |
| `UAVCAN_ENABLE` | INT32 | 0 | UAVCAN 使能，设为 2 或 3 |
| `UAVCAN_NODE_ID` | INT32 | 1 | 本机节点 ID（1-125），**必须唯一** |
| `UAVCAN_PUB_ATT` | INT32 | 0 | 姿态发布开关，设为 1 开启 |
| `UAVCAN_SUB_ATT` | INT32 | 0 | 姿态订阅开关，设为 1 开启 |
| `UAVCAN_BITRATE` | INT32 | 1000000 | CAN 总线波特率（bps） |

### 4.2 配置示例

**双向互通配置（飞控之间相互收发）：**
```bash
# 飞控 1（节点 ID=1）
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_ATT 1
param set UAVCAN_SUB_ATT 1
param save
reboot

# 飞控 2（节点 ID=2）
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 2
param set UAVCAN_PUB_ATT 1
param set UAVCAN_SUB_ATT 1
param save
reboot
```

## 5. 验证方法

### 5.1 检查 UAVCAN 总线状态
```bash
uavcan status
```
预期输出：
```
Online nodes (Node ID, Health, Mode):
    1 OK         OPERATIONAL
    2 OK         OPERATIONAL
```

### 5.2 监听接收到的姿态和位置数据
```bash
listener uavcan_attitude
```
**预期输出示例：**
```
TOPIC: uavcan_attitude
    timestamp: 123456789
    timestamp_sample: 123456000
    source_node_id: 2
    
    q: [0.68554, 0.11731, 0.12902, -0.70654] (Roll: -1.3 deg, Pitch: 20.0 deg,)
    angular_velocity: [0.0023, -0.0012, 0.0045]
    linear_acceleration: [-0.123, 0.456, -9.81]
    
    lat: 31.234567          # 纬度（度）
    lon: 121.456789         # 经度（度）
    alt: 125.3              # 海拔（米）
    
    xy_valid: true          # GPS 水平位置有效
    z_valid: true           # GPS 高度有效
    
    eph: 0.8                # 水平精度（米）
    epv: 1.2                # 垂直精度（米）
```

### 5.3 检查参数配置
```bash
param show UAVCAN_*
```

## 6. 在应用中使用接收到的数据

### 6.1 订阅远端飞控数据

在你的应用模块中，直接订阅 `uavcan_attitude` 话题即可获取远端飞控的姿态和位置数据：

```cpp
#include <uORB/topics/uavcan_attitude.h>
#include <uORB/Subscription.hpp>

class MyModule
{
private:
    // 订阅远端飞控数据（可以订阅多个实例）
    uORB::Subscription _remote_attitude_sub{ORB_ID(uavcan_attitude), 0};  // 实例 0
    // uORB::Subscription _remote_attitude_sub2{ORB_ID(uavcan_attitude), 1};  // 实例 1（如需）
    
public:
    void update()
    {
        uavcan_attitude_s remote_data;
        
        // 读取远端飞控数据
        if (_remote_attitude_sub.update(&remote_data)) {
            // 检查数据来源
            uint8_t remote_node_id = remote_data.source_node_id;
            
            // 读取姿态数据
            float qw = remote_data.q[0];  // 四元数 w
            float qx = remote_data.q[1];  // 四元数 x
            float qy = remote_data.q[2];  // 四元数 y
            float qz = remote_data.q[3];  // 四元数 z
            
            // 读取角速度（rad/s）
            float roll_rate = remote_data.angular_velocity[0];
            float pitch_rate = remote_data.angular_velocity[1];
            float yaw_rate = remote_data.angular_velocity[2];
            
            // 读取加速度（m/s²）
            float ax = remote_data.linear_acceleration[0];
            float ay = remote_data.linear_acceleration[1];
            float az = remote_data.linear_acceleration[2];
            
            // 读取 GPS 位置数据（需先检查有效性）
            if (remote_data.xy_valid) {
                double latitude = remote_data.lat;   // 纬度（度）
                double longitude = remote_data.lon;  // 经度（度）
                float eph = remote_data.eph;         // 水平精度（米）
                
                // 使用经纬度数据...
            }
            
            if (remote_data.z_valid) {
                float altitude = remote_data.alt;    // 海拔高度（米）
                float epv = remote_data.epv;         // 垂直精度（米）
                
                // 使用高度数据...
            }
            
            // 检查数据时效性
            uint64_t data_age_us = hrt_absolute_time() - remote_data.timestamp;
            if (data_age_us < 100000) {  // 数据在 100ms 内有效
                // 数据新鲜，可以使用
            }
        }
    }
};
```

### 6.2 多飞控数据处理示例

如果有多个远端飞控，可以根据 `source_node_id` 区分：

```cpp
void process_multiple_remotes()
{
    uavcan_attitude_s remote_data;
    
    // 遍历所有可能的实例（最多 8 个）
    for (int i = 0; i < 8; i++) {
        uORB::Subscription remote_sub{ORB_ID(uavcan_attitude), i};
        
        if (remote_sub.copy(&remote_data)) {
            // 根据节点 ID 处理不同的飞控
            switch (remote_data.source_node_id) {
                case 1:
                    // 处理节点 1（主机）的数据
                    handle_leader_data(remote_data);
                    break;
                    
                case 2:
                    // 处理节点 2（僚机1）的数据
                    handle_wingman1_data(remote_data);
                    break;
                    
                case 3:
                    // 处理节点 3（僚机2）的数据
                    handle_wingman2_data(remote_data);
                    break;
                    
                default:
                    // 处理其他节点
                    PX4_INFO("Received data from node %d", remote_data.source_node_id);
                    break;
            }
        }
    }
}
```

### 6.3 编队飞行应用示例

计算与长机的相对位置（简化示例）：

```cpp
#include <lib/geo/geo.h>  // 地理坐标转换函数

void calculate_formation_offset()
{
    // 订阅长机数据（假设长机是节点 ID 1）
    uavcan_attitude_s leader_data;
    if (_leader_sub.copy(&leader_data) && leader_data.xy_valid) {
        
        // 获取本机 GPS 位置
        vehicle_global_position_s own_pos;
        if (_own_gps_sub.copy(&own_pos)) {
            
            // 计算与长机的相对距离和方位
            double lat1 = leader_data.lat;
            double lon1 = leader_data.lon;
            double lat2 = own_pos.lat;
            double lon2 = own_pos.lon;
            
            // 使用 PX4 地理库计算距离
            float distance = get_distance_to_next_waypoint(lat1, lon1, lat2, lon2);
            float bearing = get_bearing_to_next_waypoint(lat1, lon1, lat2, lon2);
            
            // 计算相对位置（NED 坐标系）
            float offset_north = distance * cosf(bearing);
            float offset_east = distance * sinf(bearing);
            float offset_down = own_pos.alt - leader_data.alt;
            
            PX4_INFO("Formation offset: N=%.1f E=%.1f D=%.1f", 
                     (double)offset_north, (double)offset_east, (double)offset_down);
            
            // 根据偏移量调整飞行控制...
        }
    }
}
```

### 6.4 注意事项

- **数据有效性检查**：始终检查 `xy_valid`、`z_valid` 标志
- **时效性检查**：通过 `timestamp` 判断数据是否过时
- **精度评估**：使用 `eph`、`epv` 评估位置精度是否满足需求
- **节点识别**：通过 `source_node_id` 识别数据来源
- **多实例订阅**：`uavcan_attitude` 支持最多 8 个实例，对应 8 个不同的远端节点

---

