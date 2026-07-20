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
