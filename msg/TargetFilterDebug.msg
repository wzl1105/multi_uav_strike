# ============================================================================
# 跟踪目标位置/速度 滤波前后对比 (debug 用,2026-07-16)
#   同一时刻 publish:
#     - *_raw      = 滤波前的原始值
#     - *_filtered = One Euro Filter 后的值
#     - *_delta    = raw - filtered (≈ 噪声幅度,可直接看还在不在跳)
#   用法:
#     rostopic echo -n 5 /mission/target_filter_debug   # 位置
#     rostopic echo -n 5 /guidance/target_filter_debug  # 速度
# ============================================================================
std_msgs/Header header

# ---- 位置滤波对比 (NWU,米) ----
geometry_msgs/Point  pos_raw
geometry_msgs/Point  pos_filtered
geometry_msgs/Point  pos_delta

# ---- 速度指令滤波前后对比 (NED,m/s) ----
# 注:vel_raw / vel_filtered 都是坐标转换到 PX4 ENU 之前的 NED 速度；
#    yaw_rate 不再复用 vel_*.z，保证垂直速度可用于打击复盘。
geometry_msgs/Vector3 vel_raw
geometry_msgs/Vector3 vel_filtered
geometry_msgs/Vector3 vel_delta

# ---- 元信息 ----
uint64  cluster_id    # 当前锁定的 cluster_id(变 → target_estimator 重分配了)
bool    is_locked     # 当前是否锁定(is_locked=false 时所有 filtered 值无意义)
string  source        # "position" / "velocity",用来区分两个 node 发的同类型 msg