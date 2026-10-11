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

# Attack(105) 协同打击专用:
#   本机到达 attack_point 的微秒时间戳(0 = 还没到)
#   comm_node 从这里读并填进 InterUavStatus 心跳,partner 据此计算 strike_sync_time
uint64 attack_arrival_time_us

# === 2026-08-25: GS 是否已下达 attack_cmd action=0 (打) ===
#   0 = 未下达或已 ignore(1)/stash(2)/清锁,末段图像导引(chase 几何)不下沉;
#   1 = 已下达,SEARCH_TRACK 下 UAV 持续 stand-off 跟着但不下沉,直到
#       phase 推进或 GS 二次指令(action=1/2)清掉。
#   用户口径:"我要末端图像导引,但是接收到 attackCmd 之后才要"
#   → 只有 GS 显式发攻击指令,TRACK/STRIKE 末段 chase 才下沉,避免
#     TRACK 阶段 UAV 自己在 horiz≤22m 时往下冲。
uint8 attack_cmd_received

# === 2026-10-11: 拉烟 session latch (治本打击后飞手接管还发目标) ===
#   0 = 本次 Attack 还没 commit (拉烟),或新 TaskFlow 已 reset;
#   1 = 本次 Attack 已 strike commit, 拉烟触发, "任务完成" 语义;
#       target_estimator 收到后停发 cluster_states / target_estimated_gps /
#       cluster_event / raw_detection_marker, 直到新 skill 进来 (新 TaskFlow) 才
#       reset。飞手松手不回 OFFBOARD 不重置 (用户原话)。
uint8 strike_committed
