# PN 制导单周期诊断数据；全部坐标字段显式标注坐标系，供 rosbag 离线复盘。
std_msgs/Header header

# 输入位置（guidance_control_node 内部统一 NWU，米）
geometry_msgs/Point uav_position_nwu
geometry_msgs/Point target_position_nwu

# InterceptGuidance 输出、pitch 门控后、One Euro 后的速度（均为 NED，米/秒）
geometry_msgs/Vector3 strategy_velocity_ned
geometry_msgs/Vector3 gated_velocity_ned
geometry_msgs/Vector3 filtered_velocity_ned

# LOS 几何与 PN 内部滤波结果
float64 distance
float64 horizontal_distance
float64 los_yaw
float64 los_pitch
float64 los_yaw_rate
float64 los_pitch_rate
float64 desired_yaw

# guidance_control_node 对下降速度施加的平滑门控权重 [0, 1]
float64 pitch_gate_weight
