# 任务状态上报(对应协议 0x1002 CLUSTER_MISSION_STATE)
# 定频上报,不是状态切换才发(每 1Hz 一次)
string  skill_flow_id
string  skill_id
int32   skill_type
# state 编码:
#   0 = 未开始,1 = 执行中,2 = 暂停,3 = 完成,4 = 失败
int32   state
# 当前任务执行阶段 (mission_manager 的 MissionPhase 枚举值),GS 用此判断 UAV 是 waypoint-following / guidance-striking / holding
#   0 = GROUND_IDLE, 1 = TAKING_OFF, 2 = HOVERING, 3 = WAYPOINT_FOLLOW, 4 = GUIDANCE_TRACK,
#   5 = GUIDANCE_STRIKE, 6 = RETURNING, 7 = COMPLETE, 8 = FAILED, 9 = HOLDING,
#   10 = OVERRIDDEN, 11 = TARGET_LOST_HOVER
uint8   phase
