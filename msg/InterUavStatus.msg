# 多机心跳(对应协议补充规范 §6)
# comm_node 按距离动态调频:基础帧率 / 5Hz (≤40m) / 20Hz (≤20m)
string  sn
uint8   work_mode
int32   skill_type
string  skill_id
uint64  timestamp_us

# 当前 mission 执行阶段(MissionState.phase 枚举值,供 uav_avoidance 仲裁优先级用)
#   0=GROUND_IDLE 1=TAKING_OFF 2=HOVERING 3=WAYPOINT_FOLLOW 4=GUIDANCE_TRACK
#   5=GUIDANCE_STRIKE 6=RETURNING 7=COMPLETE 8=FAILED 9=HOLDING
# uav_avoidance 判定规则:phase=STRIKE(5)>TRACK(4)>waypoint-follow(3)按work_mode>其它让路
uint8   phase

# GPS(MSL 高度,米)
float64 lat
float64 lon
float64 alt

# 偏航角(弧度,NED 顺时针,0=N)
float32 yaw

# NED 速度(m/s):来自 /mavros/local_position/velocity (源 ENU,comm_node/starling_bridge
# 内部转 NED 后填入,转换式见 comm_node.cpp::velocityCallback 注释)。
#   - 与 yaw 同用 NED 约定,心跳"姿态相关"字段自洽
#   - uav_avoidance 用它判定邻居是否静止(防御圈收缩)
#   - uav_avoidance 也用它算真实 closing speed(替代原位置差估算)
#   - 静态/初始化阶段=0,下游应据此判断信源是否就绪
#   - z 方向:正值 = 向下(NED),UAV 起飞/悬停时 ≈ 0
float32 vx
float32 vy
float32 vz
