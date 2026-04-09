# Formation Motor PWM Module

这个小模块用于让一架四旋翼通过 DroneCAN/UAVCAN v0 向另一架四旋翼发送:

- 4 个电机的真实转速 `RPM`
- 1 个指定 PWM 输出口的实际输出值 `us`

## 数据从哪里读取

- 电机真实转速读取自 `esc_status` 话题中的 `esc_status_s::esc[i].esc_rpm`
- 为了保证四个电机顺序正确，代码优先使用 `esc_status_s::esc[i].actuator_function` 去匹配 `Motor1..Motor4`
- 如果底层 ESC 驱动没有填 `actuator_function`，代码会退回到 `esc[0..3]`
- 额外 PWM 输出读取自 `actuator_outputs` 话题中的 `actuator_outputs_s::output[channel - 1]`
- `actuator_outputs.output[]` 在 PWM 输出板上对应的是实际输出脉宽值，单位是微秒

## 发送端

- 文件: `src/drivers/uavcan/formation_motor_pwm_sender.[cpp|hpp]`
- 自定义消息: `dronecan::formation::MotorPwmStatus`
- 发送开关参数: `UAVCAN_PUB_MRPM=1`
- PWM 通道选择参数: `UAVCAN_MPWM_CH`

## 接收端

- 文件: `src/drivers/uavcan/sensors/formation_motor_pwm.[cpp|hpp]`
- 接收开关参数: `UAVCAN_SUB_MRPM=1`
- 接收到的数据发布到本地 uORB 话题: `formation_motor_pwm`

本地话题字段:

- `motor_rpm[4]`: 四个电机 RPM
- `pwm_us`: 额外 PWM 输出值
- `pwm_channel`: 发送端选中的 PWM 通道号，1-based
- `motor_valid_mask`: bit0..bit3 分别对应电机 1..4 是否有效
- `flags`: 总体有效位、PWM 有效位、ESC 遥测有效位
- `source_node_id`: 发送节点 Node ID

## 集成开关

编译期 Kconfig:

- `UAVCAN_FORMATION_MOTOR_PWM_SENDER`
- `UAVCAN_SENSOR_FORMATION_MOTOR_PWM`

运行时参数:

- `UAVCAN_PUB_MRPM`
- `UAVCAN_SUB_MRPM`
- `UAVCAN_MPWM_CH`

## 最小交付文件

- `msg/FormationMotorPwm.msg`
- `src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20041.MotorPwmStatus.uavcan`
- `src/drivers/uavcan/formation_motor_pwm_sender.cpp`
- `src/drivers/uavcan/formation_motor_pwm_sender.hpp`
- `src/drivers/uavcan/sensors/formation_motor_pwm.cpp`
- `src/drivers/uavcan/sensors/formation_motor_pwm.hpp`
- 以及几个接入点修改文件:
  - `msg/CMakeLists.txt`
  - `src/drivers/uavcan/CMakeLists.txt`
  - `src/drivers/uavcan/Kconfig`
  - `src/drivers/uavcan/uavcan_main.cpp`
  - `src/drivers/uavcan/uavcan_main.hpp`
  - `src/drivers/uavcan/sensors/sensor_bridge.cpp`
  - `src/drivers/uavcan/uavcan_params.c`

## 修改位置索引

下面按“文件 + 行号 + 修改内容”列出这次接入所改的位置，方便移植给别人时逐项对照。

### 1. 新增 uORB 话题

`msg/FormationMotorPwm.msg`

- 新增整个文件
- 定义本地 uORB 结构体 `formation_motor_pwm_s`
- 用于在接收端把 CAN 数据转换为 PX4 内部可订阅的话题

`msg/CMakeLists.txt`

- 第 93 行添加 `FormationMotorPwm.msg`

### 2. 新增 DroneCAN 自定义消息

`src/drivers/uavcan/libdronecan/dsdl/dronecan/formation/20041.MotorPwmStatus.uavcan`

- 新增整个文件
- 定义 CAN 总线上实际发送的消息:
  - `motor_rpm[4]`
  - `pwm_us`
  - `pwm_channel`
  - `motor_valid_mask`
  - `flags`

### 3. 新增发送端源码

`src/drivers/uavcan/formation_motor_pwm_sender.hpp`

- 新增整个文件
- 定义发送类 `FormationMotorPwmSender`

`src/drivers/uavcan/formation_motor_pwm_sender.cpp`

- 新增整个文件
- 实现发送逻辑:
  - 从 `esc_status` 读取四个电机 RPM
  - 从 `actuator_outputs` 读取一个 PWM 输出通道
  - 打包成 `MotorPwmStatus`
  - 通过 UAVCAN 广播

### 4. 新增接收端源码

`src/drivers/uavcan/sensors/formation_motor_pwm.hpp`

- 新增整个文件
- 定义接收桥 `FormationMotorPwmBridge`

`src/drivers/uavcan/sensors/formation_motor_pwm.cpp`

- 新增整个文件
- 实现接收逻辑:
  - 接收 `MotorPwmStatus`
  - 解析到本地 `formation_motor_pwm` uORB 话题

### 5. 接入 UAVCAN 模块编译

`src/drivers/uavcan/CMakeLists.txt`

- 第 151 行添加 `formation_motor_pwm_sender.cpp`
- 第 152 行添加 `formation_motor_pwm_sender.hpp`
- 第 177 行添加 `sensors/formation_motor_pwm.cpp`
- 第 178 行添加 `sensors/formation_motor_pwm.hpp`

### 6. 接入 Kconfig 开关

`src/drivers/uavcan/Kconfig`

- 第 25 行添加 `config UAVCAN_FORMATION_MOTOR_PWM_SENDER`
- 第 65 行添加 `config UAVCAN_SENSOR_FORMATION_MOTOR_PWM`

### 7. 接入主节点初始化

`src/drivers/uavcan/uavcan_main.hpp`

- 第 75 行添加 `#include "formation_motor_pwm_sender.hpp"`
- 第 284 行添加成员 `FormationMotorPwmSender _formation_motor_pwm_sender;`

`src/drivers/uavcan/uavcan_main.cpp`

- 第 93 行在构造函数初始化列表中添加 `_formation_motor_pwm_sender(_node),`
- 第 574 行开始添加 `UAVCAN_PUB_MRPM` 参数判断与发送端初始化逻辑

### 8. 接入接收桥工厂

`src/drivers/uavcan/sensors/sensor_bridge.cpp`

- 第 85 行添加 `#include "formation_motor_pwm.hpp"`
- 第 244 行开始添加 `UAVCAN_SUB_MRPM` 参数判断
- 满足条件时创建 `FormationMotorPwmBridge`

### 9. 新增运行时参数

`src/drivers/uavcan/uavcan_params.c`

- 第 280 行添加 `PARAM_DEFINE_INT32(UAVCAN_PUB_MRPM, 0);`
- 第 510 行添加 `PARAM_DEFINE_INT32(UAVCAN_SUB_MRPM, 0);`
- 第 558 行添加 `PARAM_DEFINE_INT32(UAVCAN_MPWM_CH, 5);`

### 10. 文档

`docs/uavcan_formation_motor_pwm_implementation.md`

- 新增整个文件
- 记录该模块的数据来源、发送端、接收端、参数和修改位置
