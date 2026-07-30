# ============================================================================
# 单条 skill 定义 — 任务流中的原子单元
# 与 WorkMode 独立(WorkMode 由 GS 单独下发)
# skill_type 枚举(参考 TZS v4.0 §9.2.6):
#   100 = Takeoff-弹射 (catapult takeoff,走弹射起飞流程,等 PX4 抛飞模式 + 外部 trigger)
#   101 = Gather(集结)
#   102 = Search(搜索)
#   103 = Return(返航,任务流末尾触发 PX4 AUTO.LAND)
#   104 = Relay(中继,本版本不实现)
#   105 = Attack(多目标打击)
#   106 = Takeoff-地面 (ground takeoff,走原 OFFBOARD+ARM 流程)
#
# TODO(skill_type_mapping): 协议升级中,旧版本 100=Takeoff(弹射/地面)由 takeoff_subtype
#   区分(0=地面,1=弹射);新协议改为 100=弹射 / 106=地面分开,不再使用 takeoff_subtype。
#   当前 mission_manager_node.cpp 已兼容新旧两套 (同时认 100 和 106 为 takeoff),
#   gs_simulator_node.py 应改为: 弹射任务下发 100, 地面任务下发 106。
#   此字段后续版本会弃用,具体时序等 GS 端协议冻结后清理。
# ============================================================================

string   skill_id
uint8    skill_type

# 仅 skill_type=100 时有效
# 0 = 地面起飞,1 = 弹射起飞
# 协议补充规范 §1.1 要求弹射起飞后置必须有 skill(由 GS 端强校验)
uint8    takeoff_subtype

# 仅 skill_type=100 时有效
# 起飞目标高度(米，相对 PX4 home altitude,即目标 alt = ref_alt + takeoff_altitude)
# 0 表示使用 mission_manager 私有 param ~takeoff_altitude 兜底
# skill_type != 100 时此字段忽略
float32  takeoff_altitude

# 仅 skill_type=100 或 106 (地面/弹射起飞)时有效
# 起飞爬升速度(米/秒)。mission_manager 内部夹在 [1, 5]:
#   下限 1 — 避免弹射/弱风场景下爬升过慢、过渡时间过长
#   上限 5 — PX4 MPC_VEL_UP_MAX(~5 m/s)安全区,避免速度饱和+overshoot
# 0 或负值 → 使用 mission_manager 私有 param ~takeoff_climb_rate 兜底
# skill_type != 100/106 时此字段忽略
float32  takeoff_speed

# ============================================================================
# 任务优先级 — 数值越大优先级越高
#
# 设计要点(2026-07):
#   粗粒度(跨 UAV)优先级由 WorkMode 决定,不是本字段:
#     - SEARCH_STRIKE      = 最高
#     - SEARCH_TRACK       = 次高
#     - SEARCH_ONLY        = 低
#     - DENIED_ENV_FLIGHT  = 低
#   详见 WorkMode.msg
#
# 本字段 (Skill.priority) 用途:
#   1. 细粒度:同 UAV 内多条 skill 并存时仲裁(目前 task_flow 单 UAV 顺序执行,
#             本字段在单机内暂不生效,主要为未来扩展预留)
#   2. 备援:GS 端在 work_mode 之外,基于此字段决定任务的细节优先级
#            (例如"先打距离近的目标"还是"先打高威胁目标")
#   3. 跨 UAV 备援:当两条 task_flow 来自不同 GS 节点时,本字段可作为额外仲裁依据
#
# 建议取值范围(参考 TZS 协议 §9.2.6,本字段与 work_mode 独立):
#   0  - 9    : 系统保留 (紧急迫降/失联保留通道)
#   10 - 49   : 低优先级
#   50 - 99   : 中优先级
#   100- 199  : 高优先级
#   200- 255  : 最高优先级(预留,医疗救护/Abort 场景)
#
# 默认值建议(GS 下发 task_flow 时,如果 work_mode 已能表达优先级,本字段可填 0):
#   100 Takeoff : priority = 60  (过渡阶段,不可中断)
#   101 Gather  : priority = 20
#   102 Search  : priority = 0   (依赖 work_mode 决定)
#   103 Return  : priority = 90  (过渡阶段,不可中断,含 AUTO.LAND 落地)
#   105 Attack  : priority = 140
#
# 多机协同注意(基于 work_mode,与 Skill.priority 字段无关):
#   - 同一区域多 UAV 路径冲突时,work_mode 优先级高的 UAV 保持原路径
#   - 跨 phase 时(例如本机 PHASE_TAKING_OFF, 邻机 work_mode=SEARCH_STRIKE):
#     邻机应保持原路径,本机起飞避让
# ============================================================================
uint8    priority

# 进场巡航速度(米/秒)
float32  cruise_speed

# 任务区执行速度(米/秒)
# skill_type=105 时,此值也是 WP1→WP2 的打击速度
float32  task_speed

# 多机协同模式
# 0 = 默认,1 = 顺序惯序,2 = 编队
uint8    formation

# 搜索技能的云台俯仰角(度)
# 负值 = 向下,0 = 水平,正值 = 向上
float32  gimbal_pitch

# 进场航点(沿此段飞行至任务区,不触发任务执行)
nav_msgs/Path arrive_path

# 任务区航点(完成首个点 = 准入门;末点 = 准出门)
nav_msgs/Path skill_area_path

# 兜底参数(预留 JSON 字符串,具体字段后续追加)
string   params_json

# ============================================================================
# 目标类型位掩码(2026-07-30 新增,GS 端 skill_params.target_type 透传)
# 仅 skill_type=105 (Attack) 生效;其他 skill 忽略
# 位定义:
#   bit 0 (0x01) = person(人)
#   bit 1 (0x02) = car(车)
#   bit 2+       = 协议未定义,按 0 处理(任意 label 匹配)
# 取值示例:
#   0          = 任意 label(保持旧行为,GS 漏发 / 老 Skill 也走这里)
#   1          = 仅 person
#   2          = 仅 car
#   3          = person + car(人车都打,选距 WP2 最近)
#   4+         = 仅未定义 bit 置位,回退到 0(任意 label)
#
# 路由语义:WP2-gate 内的 cluster 中,label 匹配此掩码的候选,挑距 WP2 最近
#   的那个作 primary 触发 strike;其他全部走 type=3 被动上报
# ============================================================================
uint32   target_type
