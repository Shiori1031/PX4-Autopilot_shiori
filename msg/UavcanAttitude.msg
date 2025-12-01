###############################################################################
# @file UavcanAttitude.msg
#
# 从远程飞控接收的 UAVCAN 姿态和位置信息
#
# @group UAVCAN
#
# TOPICS uavcan_attitude
###############################################################################

uint64 timestamp             # 接收时间戳（微秒）
uint64 timestamp_sample      # 原始采样时间戳（微秒）
uint8 source_node_id         # 源节点 CAN ID（1-125）

# 姿态数据
# PX4 四元数顺序 (w, x, y, z)
float32[4] q                 # 四元数姿态
float32[3] angular_velocity  # 角速度（rad/s）
float32[3] linear_acceleration  # 线性加速度（m/s²）

# 位置数据（通过 orientation_covariance 字段传输）
bool xy_valid                # 纬度和经度是否有效
bool z_valid                 # 高度是否有效
bool v_xy_valid              # 水平速度是否有效
bool v_z_valid               # 垂直速度是否有效

# 全球位置（GPS 坐标）
float64 lat                  # 纬度（度，-90~+90）
float64 lon                  # 经度（度，-180~+180）
float32 alt                  # 海拔高度 AMSL（米）

# NED 坐标系速度（m/s）
float32 vx                   # 北向速度
float32 vy                   # 东向速度
float32 vz                   # 向下速度

# 航向角（弧度，-π 到 +π）
float32 heading              # 航向角

# 位置和速度精度（标准差，单位：米）
float32 eph                  # 水平位置误差
float32 epv                  # 垂直位置误差
