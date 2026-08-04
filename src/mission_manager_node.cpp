/**
 * mission_manager_node.cpp
 * 任务管理器：核心状态机，协调已有模块执行三种工作模式
 *
 * 职责：
 * - 响应地面站工作模式指令
 * - 管理搜索、跟踪、打击流程
 * - 协调 gimbal_simulator + target_estimator + guidance_control
 * - 实现机间避障（搜索阶段全程）
 * - 触发毫米波雷达急停（低速时）
 *
 * 工作模式：
 * - MODE_SEARCH_ONLY: 全图搜索，识别目标只回传，不跟踪不打击
 * - MODE_SEARCH_TRACK: 搜索即跟踪，识别后锁定+螺旋定位，不打击
 * - MODE_SEARCH_STRIKE: 搜索即打击，定位完成→制导打击
 *
 * 重要说明：
 * - 工作模式由地面站预设下发，不是自动切换
 * - 无人机上电预加载航线，筒射完毕后开始执行
 *
 * 订阅：
 * - /mission/mode                    - 工作模式（来自 comm_node）
 * - /mission/waypoint_cmd           - 航点命令（来自 comm_node）
 * - /target_estimator/cluster_states - 10Hz 所有 alive cluster 快照(由 target_estimator_node 发布)
 * - /target_los_angle               - 目标LOS角度（来自 gimbal_simulator）
 * - /target_estimated_pose           - 目标估计位置（来自 target_estimator）
 * - /target_estimated_twist         - 目标估计速度
 * - /inter_uav/target_info           - 其他无人机发现的目标
 * - /mavros/local_position/pose     - 本机位置
 * - /detection/obstacle              - 毫米波雷达障碍检测
 *
 * 发布：
 * - /waypoint_executor/control      - 航点执行器控制命令
 * - /guidance/enable                - 制导使能
 * - /guidance/target_pose           - 制导目标
 * - /avoidance/vector               - 避障向量
 * - /emergency/stop                 - 急停命令
 * - /mission/status                 - 任务状态
 */

 using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/HomePosition.h>
#include <tf/transform_datatypes.h> // 用于RPY转四元数
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <thread>
#include <chrono>
#include <sstream>
#include <unordered_map>
#include <boost/make_shared.hpp>

// === Phase 1+ 类型化消息 ===
#include "multi_uav_strike/TaskFlow.h"
#include "multi_uav_strike/Skill.h"
#include "multi_uav_strike/WorkMode.h"
#include "multi_uav_strike/AttackCmd.h"
#include "multi_uav_strike/MissionState.h"
#include "multi_uav_strike/DetectTarget.h"
#include "multi_uav_strike/DetectTargets.h"
#include "multi_uav_strike/ClusterState.h"      // 所有 alive cluster 的 10Hz 快照(由 target_estimator_node 发布)
                                                // 替代原 cluster_event(后者只 publish 一次,会被 IN_TASK 时序窗口漏掉)
#include "multi_uav_strike/TrackingState.h"
#include "multi_uav_strike/WaypointStatus.h"
#include "multi_uav_strike/InterUavStatus.h"     // 多机心跳订阅(多机 gather 同步用,2026-07-24 替代原 UavGatherStatus)
#include "multi_uav_strike/TargetFilterDebug.h"  // 滤波前后对比 debug msg (2026-07-16)
#include "multi_uav_strike/one_euro_filter.h"  // 自适应低通,平滑 target_estimated_pose (2026-07-16)

// OneEuroFilter1D 在 multi_uav_strike 命名空间下,本文件的 MissionManager 不在
// 该命名空间 → 加 using 让类成员声明 (target_smooth_x_) 等直接写短名。
using multi_uav_strike::OneEuroFilter1D;

#include <std_msgs/UInt64.h>   // mission/primary_target (cluster_id, 主目标) → target_estimator

// 工作模式枚举
// WorkMode 只表达「在任务区域内做什么」(对应 TZS MAV_CMD_SET_WORKMODE 语义)。
// 起飞/落地/返航/集结等阶段不归 WorkMode 管,由 MissionPhase 表达。
// 与 WorkMode.msg 不同步:
//   IDLE              = 0  (保留,UAV 空闲)
//   SEARCH_ONLY       = 1
//   SEARCH_TRACK      = 2
//   SEARCH_STRIKE     = 3
//   DENIED_ENV_FLIGHT = 4  (预留,未实现)
enum class WorkMode {
    IDLE,             // 等待:只获取 UAV 信息,不发任何控制指令(PX4 SITL 不发 setpoint)
    SEARCH_ONLY,      // 全图搜索
    SEARCH_TRACK,     // 搜索即跟踪
    SEARCH_STRIKE,    // 搜索即打击
    DENIED_ENV_FLIGHT // 拒止环境飞行(预留,GPS 拒止/强对抗场景,未实现)
};

// 任务状态
enum class TaskStatus {
    IDLE,
    EXECUTING_WAYPOINT,  // 执行航点
    TARGET_LOCKED,       // 目标锁定
    SPIRAL_APPROACH,     // 螺旋接近
    GUIDANCE_APPROACH,   // 制导接近
    STRIKE_EXECUTED,     // 打击完成
    EMERGENCY_STOP       // 急停
};

// PX4 SITL 起飞状态
enum class TakeoffState {
    TAKEOFF_IDLE,           // 空闲状态，等待开始起飞
    TAKEOFF_WAITING_FCU,   // 等待 FCU 连接
    // ===== 弹射起飞特有状态(skill_type=100) =====
    TAKEOFF_CATAPULT_ARMED,     // 弹射模式就绪,等外部 trigger
    TAKEOFF_CATAPULT_TRIGGERED, // 收到 trigger,等 PX4 进 POSCTL
    TAKEOFF_CATAPULT_POSCTL,    // PX4 POSCTL 稳定,准备 OFFBOARD 接管
    // ===== 标准起飞流程(地面 + 弹射后段共用) =====
    TAKEOFF_SETTING_OFFBOARD, // 正在切换 OFFBOARD 模式
    TAKEOFF_TAKEOFF_EXEC,  // 执行起飞爬升
    TAKEOFF_HOVERING,      // 悬停等待
    TAKEOFF_COMPLETE,      // 起飞完成
    TAKEOFF_FAILED         // 起飞失败
};

// 内部任务执行阶段 (独立于 WorkMode, 由 mission_manager 自己推进)
// WorkMode 是 GS 的意图 (SEARCH_*), MissionPhase 是实际在执行什么
// 引入原因: 地面站把"起飞+航点"打包成一个任务, 期间 WorkMode 由 SET_WORKMODE 控制,
enum class MissionPhase {
    PHASE_GROUND_IDLE,       // 在地面无任务
    PHASE_TAKING_OFF,        // 正在执行 OFFBOARD+ARM+爬升
    PHASE_HOVERING,          // 起飞完成, 悬停等待自动交接
    PHASE_WAYPOINT_FOLLOW,   // 正在按航点飞
    PHASE_GUIDANCE_TRACK,    // 搜索跟踪 (guidance)
    PHASE_GUIDANCE_STRIKE,   // 搜索打击 (guidance)
    PHASE_RETURNING,         // 返航
    PHASE_COMPLETE,          // 任务完成
    PHASE_FAILED,            // 失败
    PHASE_HOLDING,           // 原地悬停:等待集结伙伴或新 task_flow
    PHASE_OVERRIDDEN,        // 飞手接管(PX4 mode != OFFBOARD);机载感知,等 GS 发起恢复
    PHASE_TARGET_LOST_HOVER  // 目标丢失悬停(Guidance 阶段 target 消失后保持位置)
};

// === Phase 2: Skill 生命周期 ===
// 单条 Skill 在机载内部的执行阶段,独立于 WorkMode(MissionPhase)
enum class SkillState {
    PENDING,         // 已入队,等待上一 skill 完成
    TRANSIT,         // fly arrive_path,等门控:executor 报 ARRIVE_DONE
    ENTRY_PENDING,   // 已到 arrive 末点,等门控:接近 skill_area_path[0]
    IN_TASK,         // 准入门触发,执行 skill 本体(搜索/集结等待/打击)
    EXIT_PENDING,    // executor 报 SKILL_AREA_DONE,等一帧发最终遥测
    COMPLETE,        // skill 已完成
    FAILED           // 超时或不容许状态转换
};

// === Phase 2: 当前 Skill 的运行时容器 ===
struct SkillRuntime {
    multi_uav_strike::Skill  msg;            // 输入 Skill 完整拷贝
    SkillState               state;          // 当前生命周期阶段
    ros::Time                state_enter_time;
    // 准入门计时(ENTRY_PENDING 等待超过此时间 → FAILED)
    ros::Time                entry_gate_enter_time;
    // 任务区耗时上限(IN_TASK 等待超过此时间 → FAILED,可配 0=无限)
    ros::Time                in_task_enter_time;
    // 集结超时计时(仅 skill_type=101 适用)
    ros::Time                gather_enter_time;
    // 本机到达集结点并进入 HOLDING 的时刻;等待至少一个心跳超时窗后才允许通过屏障
    ros::Time                gather_local_arrived_time;
    // 集结期望伙伴名单(仅 skill_type=101 适用)
    //   - 任务流注入:从 Skill.params_json["devices_sn"] 解出(GS 真实协议)
    //   - 老路径 fallback:launch 私有 param ~expected_sns(空时)
    //   - 空 → 单机路径(single_uav_arrived)
    std::vector<std::string> gather_partners_;
    // === 2026-07-24: 伙伴 HOLDING 一次性标记 ===
    //   分布式集结屏障必须等"每个伙伴都到过 HOLDING",而不是"每个伙伴都正在 HOLDING"。
    //   后者会让先到的 2 架走了(phase 切到下一阶段),第 3 架永远等不到。
    //   关键路径:interUavStatusCallback 收到伙伴 phase=HOLDING → 写入此 map(只写一次)。
    //   checkAllUavsGathered() 改为遍历此 map。
    std::map<std::string, ros::Time> gather_partner_holding_seen_;
    // 暂停/失败原因,日志用
    std::string              last_event;

    // === Attack(105) 多机协同专用字段 ===
    //   不污染 MissionPhase enum,只在 mission_manager 内部跟踪
    enum class AttackPhase { IDLE, APPROACHING, SEARCHING, ABANDONED };
    AttackPhase              attack_phase_ = AttackPhase::IDLE;
    ros::Time                attack_search_enter_time_;   // SEARCHING 计时起点
    // 协同伙伴名单(从 Skill.params_json["devices_sn"] 解出,已剔除自己)
    std::vector<std::string> attack_partners_;
    // 本机在 partners_ 全员(GS 下发原始顺序,含本机)中的位置;-1=未列入(单机模式)
    int                      attack_seq_index_ = -1;
    // formation=1 sequential:上一架 partner 的 sn(本机非第一架时有效)
    std::string              attack_seq_prev_sn_;
    // formation=1 sequential barrier 等待起始(伙伴 phase 进 GUIDANCE_STRIKE 触发)
    ros::Time                attack_seq_wait_start_;
    // formation=1 barrier 满足后,本机 strike 触发允许的时刻(攻击前缓冲 1s)
    ros::Time                attack_seq_strike_granted_at_;
    // formation=2 全局同步开火时刻(初始预估 + partner 到达后重算)
    ros::Time                attack_strike_sync_time_;
    // 本机到达 attack_point 的时刻(上报用,comm_node 写进心跳)
    ros::Time                attack_arrival_time_;
    // Attack(105) 到达 attack_point 后等待 strike trigger 的开始时刻(2026-07-26)
    //   - attack_arrival_time_ 是给 partner 算 strike_sync_time 的语义锚点,
    //     仍然以 UAV *踩到* attack_point 的那一刻为准
    //   - attack_hover_enter_time_ 是给 mission_manager 自己算兜底超时用,
    //     两值在 IN_TASK case 中同一帧被设置,数值相同但语义不同
    //   - 超时时长 attack_hover_timeout_sec_(MM 成员,可配,默认 30s)
    ros::Time                attack_hover_enter_time_;
    // formation=2:本机 attack_point(由 planCoordinatedAttack 实时规划填入)
    double                   attack_point_lat_ = 0.0;
    double                   attack_point_lon_ = 0.0;
    double                   attack_point_alt_ = 0.0;

    // === CODEX DIRECT-LOOK START ===
    // Attack(105) 收到后先原地朝 WP2 观察。预检期间尚未把 Skill 下发给
    // waypoint_executor，避免飞机先向 WP1 窜动；超时后再原样下发旧路径。
    bool                     direct_look_attempted_ = false;
    bool                     direct_look_active_ = false;
    // 三帧确认后为 true：Attack Skill 已装载进 executor，但 guidance 正在接管。
    // 用它允许 WP1 前继续刷新锁定 cluster，并在真丢失回退时恢复当前 Attack 路径。
    bool                     direct_strike_committed_ = false;
    ros::Time                direct_look_start_time_;
    uint64_t                 direct_look_cluster_id_ = 0;
    uint64_t                 direct_look_last_seen_us_ = 0;
    int                      direct_look_consecutive_hits_ = 0;
    // === CODEX DIRECT-LOOK END ===
};

// === Phase 5: 目标锁定状态(供 attack_cmd 决��前置) ===
enum class TrackLockState {
    NOT_LOCKED,        // 未识别或识别后已退出
    LOCKED_WAIT_CONFIRM, // SEARCH_TRACK 模式:识别到目标,等地面站 attack_cmd
    LOCKED_AUTO          // SEARCH_STRIKE 模式:识别即上报,可直接触发 strike
};

struct TrackedTarget {
    bool                  is_valid;
    multi_uav_strike::DetectTarget::ConstPtr latest;
    TrackLockState        lock_state;
    ros::Time             locked_at;
    // 锁定瞬间 YOLO 的原始 label (string,如 "person"/"car"),
    // 用于 attack_cmd action=1/2 加黑名单时携带 label 信息
    // (DetectTarget.label 是 uint32 占位 =1,丢失了原始 label)
    std::string           yolo_label;
};

// === 目标级"已忽略"黑名单(label + TTL) ===
// 用于解决 SEARCH_TRACK 模式下的目标聚类缺失问题:
//   1. UAV 检测并锁定目标 A (label="person")
//   2. GS 发 attack_cmd action=1/2 (忽略/暂存)
//   3. 当前实现: tracked_target_ 清空 → 下一帧 YOLO 再命中同一目标 → 又重新锁
//   4. 加本黑名单后: attack_cmd 触发时记录 label+到期时间;
//      后续 checkYoloDrivenStrike 进门控时检查新命中 label 是否在黑名单 + 未过期
//      → 视为同一类目标(由 GS ignore 过),跳过锁定。
//
// 设计权衡(2026-07 改为 label-only):
//   - 原始方案带空间半径(20m)需要靠 UAV 当前位置做 proxy,
//     但 DetectTarget.obj_lat/lon 实际是 UAV pose 占位(不是真实目标位置),
//     UAV 一边扫描一边移动,3s 就走出 20m,空间匹配失效。
//   - 改 label-only:20s 内同 label 一律忽略,扫描/悬停都稳定;空间维度
//     留给未来云台+UAV pose 反推真实目标位置(C 方案)落地后再补。
//   - 不持久化(节点重启即清空),只防同一任务流内重复
//   - 与未来 reported_targets_ 上报去重列表解耦,语义独立
//   - 用户的语义("同位置忽略人后再看到车可追")由 label 区分已满足
struct IgnoredTarget {
    std::string  label;          // YoloDetection.label 原值,如 "person"/"car"
    ros::Time    ignore_until;   // 过期时间 (now + ~ignored_retention_sec_)
    ros::Time    ignore_set_time;// 用于日志/debug
    // === 位置维度(2026-07-16 加入匹配):只屏蔽被 GS action=1/2 指定的
    //   那个物理目标,同 label 的其他物理目标仍可被锁定 ===
    double       ned_x           = 0.0;   // 物理目标 NWU 坐标 X(米)
    double       ned_y           = 0.0;   // 物理目标 NWU 坐标 Y(米)
    double       ned_alt         = 0.0;
    double       radius          = 10.0;  // 屏蔽半径(米),按 label 类别:人=10m, 车=20m
};

// OneEuroFilter1D 已抽到 include/multi_uav_strike/one_euro_filter.h,
//   mission_manager_node.cpp 和 guidance_control_node.cpp 共用。

class MissionManager {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber cluster_states_sub_;  // target_estimator_node 10Hz 发布的所有 alive cluster 快照
    ros::Subscriber gimbal_los_sub_;
    ros::Subscriber target_est_pose_sub_;
    ros::Subscriber target_est_twist_sub_;
    ros::Subscriber target_est_gps_sub_;   // target_estimated_gps → 目标真实 GPS
    ros::Subscriber inter_uav_target_sub_;
    ros::Subscriber self_pose_sub_;
    ros::Subscriber self_gps_sub_;       // mavros/global_position/global → 真实 GPS
    ros::Subscriber obstacle_sub_;
    ros::Subscriber mavros_state_sub_;    // PX4 SITL: 飞控状态
    // === Phase 2: 类型化订阅 ===
    ros::Subscriber task_flow_sub_;        // 任务流入口(multi_uav_strike/TaskFlow)
    ros::Subscriber work_mode_sub_;        // 工作模式 typed(multi_uav_strike/WorkMode)
    ros::Subscriber attack_cmd_sub_;       // MAV_CMD_ATTACK typed(multi_uav_strike/AttackCmd)
    // cluster_states_sub_ 在上方已声明
    ros::Subscriber waypoint_status_sub_;  // 航段状态(Phase 4 启用)
    ros::Subscriber inter_uav_status_sub_; // 多机心跳(/inter_uav/uav_status) — 多机 gather 同步用

    // ============== Service Client ==============
    ros::ServiceClient set_mode_client_; // PX4 SITL: 模式切换
    ros::ServiceClient arming_client_;  // PX4 SITL: 解锁
    ros::ServiceClient command_long_client_; // PX4 SITL: COMMAND_LONG 备用通道（绕过 SET_MODE 静默拒绝问题）

    // ============== PX4 起飞 ==============
    ros::Publisher takeoff_setpoint_pub_; // PX4 SITL: 位置 setpoint 发布器

    // ==============

    // ============== 发布 ==============
    ros::Publisher waypoint_control_pub_;
    ros::Publisher guidance_enable_pub_;
    ros::Publisher guidance_mode_pub_;       // 模式："strike" 或 "track"
    ros::Publisher guidance_target_pub_;
    ros::Publisher target_filter_debug_pub_;   // 2026-07-16: 位置滤波前后对比 debug topic
    ros::Publisher guidance_speed_pub_;  // 拦截速度(下发到 guidance_control)
    ros::Publisher emergency_stop_pub_;
    ros::Publisher status_pub_;
    ros::Publisher uav_pose_nwu_pub_;     // NWU姿态发布(RViz用)
    // === Phase 2: 类型化发布 ===
    ros::Publisher mission_state_pub_;       // 1Hz 定频上报 multi_uav_strike/MissionState
    ros::Publisher work_mode_state_pub_;     // 1Hz 定频上报当前 work_mode (typed),bridge 订阅后 KCP 回传
    ros::Publisher detect_target_pub_;       // 单条 target:multi_uav_strike/DetectTarget
    ros::Publisher detect_targets_pub_;      // 批量:multi_uav_strike/DetectTargets
    ros::Publisher tracking_state_pub_;      // multi_uav_strike/TrackingState
    ros::Publisher waypoint_skill_pub_;      // 向 executor 下发 Skill(分段航点)
    // 注:集结多机状态(原 gather_status_pub_)已删除(2026-07-24) —
    //   改为复用 InterUavStatus.phase(机间避障同一条总线),UavGatherStatus 不再参与数据通路。
    ros::Publisher primary_target_pub_;      // 主目标 cluster_id → target_estimator (UInt64)
                                             // 0 = mission 未指定主目标(estimator 回退最高置信度)

    // === Search(102) 云台俯仰角下发(2026-07-29) ===
    //   102 IN_TASK 期间:publish sr.msg.gimbal_pitch(度)
    //   其余时间(其它 skill_type / 非 IN_TASK / 新 task_flow):publish gimbal_pitch_default_
    //   topic: /<ns>/mission/gimbal_command (Float32),由 gimbal_control_node 订阅闭环
    ros::Publisher gimbal_command_pub_;
    ros::Timer     gimbal_command_timer_;
    double         gimbal_pitch_default_     = 0.0;   // 非搜索期间下发的"归位"俯仰角
    double         gimbal_command_rate_hz_   = 5.0;   // 下发频率(Hz),5Hz 够云台伺服响应
    double         current_gimbal_pitch_     = 0.0;   // 当前正下发到 gimbal_command 的值

    // ============== 定时器 ==============
    ros::Timer mission_timer_;
    ros::Timer mission_state_timer_;   // Phase 2: 1Hz MissionState 定频上报
    ros::Timer skill_advance_timer_;   // Phase 3: Skill 状态机推进(独立频率)
    ros::Timer tracking_state_timer_;  // 跟踪锁定期间定频发 tracking_state(state=1) 给 GCS
    ros::Timer viz_tf_timer_;          // 周期重广播 static TF map->viz_frame_(防止 rviz 漏接 latch)

    // tracking_state 上报频率(Hz,协议 §6 / 补充规范 §5)。
    //   跟踪中: 5Hz — GCS 实时闪烁
    //   未跟踪: 1Hz — 降频保活,让 GCS 知道 UAV 在线但仍能省带宽
    static constexpr double TRACKING_STATE_RATE_HZ      = 5.0;
    static constexpr double TRACKING_STATE_IDLE_RATE_HZ = 1.0;

    // ============== 状态 ==============
    WorkMode current_work_mode_;
    TaskStatus current_task_status_;
    bool is_mission_started_;

    // 目标状态
    struct TargetState {
        bool is_detected;
        bool is_locked;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped twist;
        sensor_msgs::NavSatFix gps;        // 目标 GPS(来自 target_estimated_gps)
        bool has_gps = false;              // 是否收到过有效 GPS
        double lock_time;  // 锁定持续时间
        bool is_shared;    // 是否已共享给队友
    };
    TargetState current_target_;

    // 本机状态
    geometry_msgs::PoseStamped current_pose_;
    bool is_pose_received_;

    // 真实 GPS(来自 mavros/global_position/global,WGS84 + AMSL 米)
    // 用于 DetectTarget.dev_lat/lon/alt — 替代之前用 current_pose NED 占位
    sensor_msgs::NavSatFix current_gps_;
    bool is_gps_received_ = false;

    // DetectTarget.label 协议类别 ID:GCS 按此渲染(1=人,2=车,协议 §11.2.1)。
    // 检测端产生的是字符串 label(如 "general_target"/"person"/"car"),经 labelToClassId 映射;
    // 映射不到时退化为 unknown_label_id_(可配,默认 1)。
    int unknown_label_id_ = 1;

    // GPS 参考点(机间避障时将邻居 GPS 转本地 NED)。
    // 不再 yaml 硬编码 — 由 homePositionCallback() 从 PX4 /mavros/home_position/home 自动加载
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;
    bool ref_initialized_ = false;
    ros::Subscriber home_position_sub_;

    // 共享世界参考点(只用于 RViz 显示的世界坐标偏移,与控制链路无关)
    // 多机 SITL 时各机 home 不同。改法:各机 RViz 内容仍发布在本机 local 帧
    //   "<ns>/map"(原点=本机 home),再由 mission_manager 广播 static TF
    //   map -> <ns>/map,平移量 = 本机 home 相对 world_ref 的 NWU 偏移。
    //   这样 plane / 航线 / marker 全都统一被 TF 偏移,不再各算各的。
    // 单一 UAV(world_ref_auto=true):world_ref=home → 偏移 0 → identity TF → 永在原点。
    double world_ref_lat_;
    double world_ref_lon_;
    double world_ref_alt_;
    // 单机自动取本机 home 作为 world_ref(默认 true):
    //   - 单机场景:world_ref = home → 偏移 = 0 → static TF 为 identity → UAV 在原点
    //   - 多机场景:launch 把 world_ref_auto 设为 false + 注入共享 world_ref(通常 = uav0 home)
    //              → uav0 identity 在原点,uav1/uav2 按 GPS 偏移分开
    bool world_ref_auto_ = true;
    // RViz 帧名 = "<ns>/map"(去掉命名空间前导 '/');无命名空间时回落 "map"
    std::string viz_frame_;
    // static TF 广播器:home 锁定后发一次 map -> viz_frame_
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // 避障参数 — 旧 avoidance_safe_distance / current_avoidance_vector 已废弃
    //   (原本发给 /avoidance/vector 无人订阅,机间避让由 uav_avoidance_node + waypoint_executor 接管)

    // 毫米波避障参数
    double low_speed_threshold_;  // 12 m/s
    bool is_obstacle_detected_;

    // 下发给 guidance_control_node 的拦截速度(InterceptGuidance 用)
    double guidance_speed_;
    // 兜底最低下发速度(2026-07-26):Skill 消息里 task_speed=0(GS 漏发/老版本默认值)
    //   时 guidance_speed_ 会被 0 覆盖,guidance_control_node 拿到零速会直接忽略
    //   ("Ignoring non-positive uav_speed"),plane 不动。强制 ≥ 此值,保证 guidance
    //   至少以最低速度朝目标推进。可通过 ~min_guidance_speed 调。
    double min_guidance_speed_;
    // strike 触发时的 cluster 新鲜度门槛(2026-07-26):locked_cluster_time_ 超过此值
    //   即视为 stale,triggerStrike() 入口放弃 strike 并清锁,让 waypoint 继续
    //   飞向 attack_point(目标位置),新 cluster 进得来时重新当 primary 锁上。
    //   解决 "到了 WP1 没目标,但 stale cluster(13.8s)反复触发 strike → guidance
    //   ping-pong(0.4s 内 5 个 cycle,heading 抖、velocity 跳)" 的根因。
    double max_strike_cluster_age_sec_;
    // 上报 cluster 新鲜度门槛(2026-07-29):只要 cluster.last_seen_age <= 此值,就允许任何一次
    //   派发里走 publish DetectTarget 流程(无论 type=1/3);超过即仅 skip 上报,不 erase 去重,
    //   这样绿球还能保留;超过 max_strike_cluster_age_sec_(0.5s) 时再 erase strike_dispatched_
    //   允许重新派发 strike。两个阈值并存,避免上报被 0.5s 误杀(B1)。
    double max_report_cluster_age_sec_;
    // GS 上报位置栅格去重半径(2026-07-29):C2 — 同一物理目标因投影抖动生成多个 cluster_id 时,
    //   走 cluster_id dedup 会漏掉。我们额外维护 reported_to_gs_ (cluster_id → ned_x,ned_y),
    //   新待上报 cluster 与已上报位置距离 < 此值 且 label 相同 → 视为同一目标,跳过上报。
    //   默认 40m — 介于单个目标聚类抖动(<25m)与目标间最小间距(58m)之间,既不漏也不串。
    double report_dedup_radius_m_;
    double obstacle_distance_;

    // === Attack(105) WP2-gate(2026-07-30)===
    //   cluster 必须落在 WP2 周围此半径内才允许成为打击目标,避免远距离 cluster 抢先锁定
    //   导致 UAV 飞离 WP2 攻击方向。
    //   推荐 20~40;过大容易混进临近目标,过小会错过 YOLO 反投影抖动。
    double wp2_strike_radius_m_;
    //   整个 Attack 阶段:从未有 cluster 通过 WP2-gate 时的长悬停上限(秒)。
    //   用于"确实没找到目标就悬停"的语义,远大于现有 attack_hover_timeout_sec_(30s)。
    //   推荐 120~600;过短会"还没看清就放弃",过长会卡死仿真。
    double attack_no_target_hold_sec_;
    //   是否曾有 cluster 通过 WP2-gate。驱动上面两个超时的选择:
    //     - false: 一直悬停 attack_no_target_hold_sec_(长)
    //     - true:  兜底 attack_hover_timeout_sec_(短,30s)
    //   === 2026-07-31 严格化 ===
    //   置位条件: WP2 距离门内 + (SEARCH_TRACK 任意) / (SEARCH_STRIKE labelMatchesTargetType) + 非 stale
    //   旧版在 WP2 距离门过就置位, 会让 wrong-label / stale cluster 错误地把
    //   300s 长超时降级到 30s 短超时。
    bool attack_wp2_target_seen_;
    //   taskFlowCallback 中清零
    //   105 IN_TASK 到达 WP1 的时刻(上升沿触发一次);用于延时后云台开始朝 WP2 看。
    //   wp1_reached_ 用于避免反复触发。
    bool   attack_wp1_reached_;
    ros::Time attack_wp1_reached_time_;
    //   strike 是否已触发;触发后云台从"朝 WP2 看"切换到"朝 primary cluster 看"。
    bool   attack_strike_phase_active_;

    // === Attack(105) 云台俯仰角控制(2026-07-30)===
    //   105 IN_TASK 到达 WP1 + 延时已过后,云台开始朝 WP2 看之前的延迟秒数。
    //   避免在 WP1 短暂悬停时云台就开始抖动。
    double attack_wp2_look_delay_sec_;
    //   105 阶段尚未到达 WP1 或延时内时的云台默认俯仰角(度,正值=下视,GS 语义)。
    double attack_gimbal_look_default_pitch_deg_;

    // === CODEX DIRECT-LOOK START ===
    // Attack(105) 直视预检参数。云台只控制俯仰，水平朝向由位置 setpoint 的 yaw 完成。
    bool   attack_direct_look_enabled_;
    double attack_direct_look_timeout_sec_;
    double attack_direct_look_settle_sec_;
    int    attack_direct_look_confirm_hits_;
    // === CODEX DIRECT-LOOK END ===

    // 螺旋接近参数
    double spiral_approach_radius_;
    double spiral_approach_angle_;
    bool is_spiral_active_;

    // 制导接近参数
    bool is_guidance_active_;
    double strike_distance_threshold_;  // 打击成功距离阈值

    // 参数
    double mission_loop_rate_;
    double target_lock_confidence_;  // 目标锁定所需置信度
    double spiral_approach_duration_; // 螺旋接近持续时间（秒）

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;

    // PX4 SITL 起飞状态
    TakeoffState takeoff_state_;
    bool is_px4_connected_;       // FCU 是否连接
    bool is_offboard_mode_;        // 是否已切换到 OFFBOARD
    double takeoff_altitude_;      // 起飞目标高度 (NED: z 向下为正)
    double takeoff_check_interval_;
    double takeoff_stable_time_;   // 高度达标后稳定等待（秒）
    double takeoff_hover_time_;    // 悬停等待（秒）
    double takeoff_climb_rate_;    // 起飞爬升斜坡速率 (m/s,默认 3.0,避免阶跃 setpoint 引起速度饱和)
    double takeoff_start_alt_;     // TAKEOFF_EXEC 起始时刻的高度(米,ENU 上为正),用作斜坡起点
    // ===== 弹射起飞流程状态(skill_type=100) =====
    // TODO(skill_type_mapping): 协议升级后 skill_type=100 仅表示弹射,
    //   skill_type=106 表示地面起飞;此处所有判定同步更新
    bool is_catapult_takeoff_ = false;          // 当前 task_flow 是否为弹射起飞
    bool is_px4_catapult_mode_ = false;         // PX4 是否已被 comm_node 切到抛飞模式
    bool catapult_trigger_received_ = false;    // 是否收到 /mission/catapult_trigger
    ros::Time catapult_posctl_enter_time_;      // PX4 进入 POSCTL 的时刻(用于稳定等待)
    double catapult_posctl_stable_sec_ = 1.0;   // POSCTL 持续多久才算稳定
    ros::Subscriber catapult_trigger_sub_;
    ros::Subscriber px4_catapult_mode_sub_;
    // LOCAL_POSITION_NED 发送频率（Hz）。某些板子 PX4 重启后默认频率太低，
    // 必须上电后用 MAV_CMD_SET_MESSAGE_INTERVAL (511) 设置一次，否则位置数据延迟大、
    // OFFBOARD 控制发散。FCU 断连后 flag 会清掉，PX4 重启重连时自动重发。
    double local_position_rate_hz_;
    bool is_local_position_rate_set_;
    int set_message_rate_retry_count_;     // 当前 FCU 连接期间的累计失败次数
    int max_set_message_rate_retries_;     // 超过此值放弃 (放行到 SETTING_OFFBOARD)
    ros::Time set_message_rate_start_time_;// 第一次尝试的时间, 用于 max_wait 超时
    double set_message_rate_max_wait_sec_; // 累计尝试时间上限 (秒)
    ros::Time takeoff_start_time_;
    ros::Time setpoint_start_time_;   // setpoint publisher 线程真正开始发包的时刻（用于 OFFBOARD 请求前的稳定等待）
    int setpoint_publish_count_;      // setpoint publisher 已发包计数（调试用：>0 证明 mavros 在转）
    mavros_msgs::State current_mavros_state_;
    geometry_msgs::PoseStamped takeoff_setpoint_; // 起飞位置 setpoint
    std::atomic<bool> setpoint_running_{false};
    std::thread setpoint_thread_;
    int takeoff_retry_count_;        // 起飞重试次数
    int max_takeoff_retries_;        // 最大重试次数
    double takeoff_retry_delay_;     // 重试延迟（秒）
    ros::Time takeoff_failed_time_;  // 进入 FAILED 状态的时间
    bool takeoff_failed_logged_;     // 是否已打印 FAILED 日志（避免重复刷屏）

    // ============== PX4 返航降落 (Return skill 完成后) ==============
    // 当任务流的最后一个 skill 是 Return(skill_type=103)且其 COMPLETE 时,
    // mission_manager 切到 PHASE_RETURNING 并通过 setMode("AUTO.LAND") 让 PX4
    // 自带降落+着陆后自动 disarm。落地后保持 RETURNING,UAV 不再起 setpoint publisher。
    // 适用:
    //   - 任务流结束于返航 → 自动落地收尾
    //   - 任务流中段包含 Return → 仅落地,不结束任务(交给后续 skill)
    //     (但当前业务上 Return 一般作为最后一条 skill)
    // 多机协同:其他 UAV 不应把 PHASE_RETURNING 当成"可抢占",应继续执行各自任务。
    bool        is_landing_in_progress_;     // 是否正在执行 AUTO.LAND 流程
    ros::Time   landing_start_time_;         // 触发 AUTO.LAND 的时刻(用于安全超时)
    bool        landing_complete_logged_;    // 是否已打印"落地完成"日志(避免刷屏)
    bool        landing_already_triggered_;  // 本 task_flow 是否已为最后一个 Return skill 触发过降落
                                            // (防止 disarm → 状态复位后,10Hz 状态机又重入 COMPLETE 分支重复触发)
    double      landing_safety_timeout_sec_; // 着陆后仍未 disarm 的兜底超时(秒)

    // === 飞手接管感知与受控恢复(2026-07-23) ===
    // PX4 FC mode 离开 OFFBOARD 时视为飞手接管。接管瞬间写一次快照,
    //   切回 OFFBOARD 时按快照 resume(详见 mavrosStateCallback / executeRecoveryFromOverride)。
    bool        is_fcu_overridden_       = false;
    // suppress_recovery_:taskFlowCallback 主动 setMode("OFFBOARD") 时拉起,
    //   阻止 mavrosStateCallback 在 setMode 的 spinOnce 期间按"snapshot 恢复旧 phase"
    //   那条路径跑 — 新 TaskFlow 接下来要自己从零设 phase,不能被旧 snapshot 污染。
    bool        suppress_recovery_       = false;
    ros::Time   override_enter_time_;
    struct OverrideSnapshot {
        MissionPhase phase_when_overrided;
        WorkMode     work_mode_when_overrided;
        std::string  flow_id_when_overrided;
        std::string  skill_id_when_overrided;
        ros::Time    snapshot_time;
    } override_snapshot_;

    // === 目标丢失状态机(2026-07-23) ===
    // Guidance 阶段(SEARCH_TRACK/STRIKE)目标消失后,先 hover 在原位一段时间,
    //   超过更长时间则回到 WAYPOINT_FOLLOW 继续扫。
    ros::Time   target_lost_hover_enter_time_;
    bool        target_lost_hover_active_   = false;
    double      target_lost_hold_sec_       = 2.0;   // GUIDANCE_* → TARGET_LOST_HOVER 阈值
    double      target_lost_resume_sec_     = 10.0;  // TARGET_LOST_HOVER → WAYPOINT_FOLLOW 阈值

    // ============== MissionPhase 状态机 ==============
    MissionPhase current_phase_;         // 当前任务执行阶段
    bool is_waypoints_received_;         // 地面站是否已下发航点
    size_t current_waypoint_count_;      // 当前收到的航点数量
    // is_takeoff_handoff_done_ 已删除:phase 接管协调职责后,该 flag 不再需要
    // (原逻辑靠 work_mode=SEARCH_ONLY 标志判断"已交接",会污染 work_mode 语义,
    //  现在由 current_phase_==PHASE_WAYPOINT_FOLLOW 表达交接完成)
    MissionPhase phase_before_takeoff_;   // 进入 PHASE_TAKING_OFF 前缓存的 phase,performTakeoffHandoff 失败时恢复用
    WorkMode work_mode_before_takeoff_;  // 起飞 skill 到来时缓存的用户 work_mode(TAKEOFF skill 完成后恢复,默认 IDLE)
    WorkMode post_takeoff_work_mode_;     // 起飞完成后 work_mode 兜底值(~post_takeoff_work_mode,默认 SEARCH_ONLY, 兼容老 GS 不发 SET_WORKMODE 的场景)

    // === Phase 2+: Skill 流运行时数据 ===
    std::vector<SkillRuntime> skill_queue_;  // 多条 Skill 顺序排队
    size_t current_skill_index_;             // 当前推进中的 skill 下标(0-based)
    bool has_task_flow_;                     // 是否有有效任务流(true 时不接收入参覆盖)
    std::string current_flow_id_;            // 当前任务流的 flow_id(上报 MissionState 用)
    int self_device_id_;                    // 本机 device_id(任务流的 device_id,默认自身)
    // UAV 硬件序列号(命名约定 "<group>-<plane>",如 "1-1"),由 launch/env 注入。
    // 实机上由 starling_bridge 同样参数 / env 持有,本节点保留以备跨节点一致性诊断;
    // 当前 mission_manager 任务流过滤仍用 device_id(数值)。
    std::string self_uav_device_sn_;
    // 各 skill_type 的超时设置(秒,0=无限)
    double entry_gate_timeout_;   // ENTRY_PENDING 默认 30s
    double in_task_timeout_;      // IN_TASK 默认 0(无限)
    double gather_timeout_;       // 101 集结合计超时默认 60s
    // 期望集结 UAV SN 列表(参数 --expected-sns "uav0,uav2"),101 用
    std::vector<std::string> expected_sns_;
    // === 多机集结同步(Phase 7 接通)===
    // 伙伴心跳缓存(来自 comm_node 发的 /inter_uav/uav_status)
    //  - key = 伙伴 SN
    //  - value = (last InterUavStatus, 最近刷新时间)
    // 用途:IN_TASK (skill_type=101) 时遍历 expected/gather_partners_ 名单,
    //       查心跳里 skill_type==101 的伙伴数,达到 gather_threshold_ 视为齐了
    struct PartnerHeartbeat {
        multi_uav_strike::InterUavStatus::ConstPtr latest;
        ros::Time last_update;
    };
    std::map<std::string, PartnerHeartbeat> partner_heartbeats_;
    double partner_heartbeat_timeout_;  // 伙伴心跳超时(秒,默认 3.0)
    double gather_threshold_;           // 门限 1.0=全齐,0.7=70% 即可
    // task_flow 最新缓存(防止回调顺序与启动冲突)
    multi_uav_strike::TaskFlow::ConstPtr latest_task_flow_;
    // 来自 typed WorkMode 的 topic,优先级高于 String(过渡期双订阅)
    uint8_t typed_work_mode_;
    ros::Time typed_work_mode_set_time_;
    bool has_typed_work_mode_;
    // 跟踪目标(typed)
    TrackedTarget tracked_target_;

    // === Cluster 事件订阅状态(从 cluster_states 10Hz 快照中提炼) ===
    // dedup: 同一 cluster_id 在同 task_flow 内只触发一次"上报/锁定"
    // (cluster_states 是周期发布,若不用 dedup 就会 10Hz 重复上报同一目标)
    //
    // 2026-07-29: 拆成两个独立容器。
    //   - reported_to_gs_: 上报给 GS 的去重集合(只在新 task_flow 切换、attack_cmd ignore/stash
    //     时整体清空,**不在 stale 时 erase**);键 = cluster_id,值 = 上报时记录的 (label, ned_x, ned_y)
    //     用于 C2 位置栅格去重(检查新 cluster 是否与已上报位置 < report_dedup_radius_m_ 且 label 一致)。
    //   - strike_dispatched_: 打击派发记录,仅用于 dedup 重复 lock transfer/strike 触发;
    //     stale 跳过时 erase(允许新检测重新派发)。原本混在一起导致 stale erase 同时把
    //     上报去重也废了,航向扫掠每次 pass 都重发同一条 → C1 多报。
    struct ReportedTarget {
        std::string label;
        double ned_x;
        double ned_y;
    };
    std::unordered_map<uint64_t, ReportedTarget> reported_to_gs_;
    std::unordered_map<uint64_t, int>            strike_dispatched_;
    ros::Time last_reported_cleanup_;   // 1Hz 节流清理开关(暂无过期清理,保留作 hook)

    // 当前已锁定的 cluster 快照位置(NWU 米,降级用于 triggerStrike 桥接到 current_target_)
    // 不存整个 ClusterTarget 是为了避免 ConstPtr 生命周期问题 + 解耦 target_estimator 内部
    double      locked_cluster_ned_x_  = 0.0;
    double      locked_cluster_ned_y_  = 0.0;
    double      locked_cluster_ned_alt_ = 0.0;
    uint64_t    locked_cluster_id_     = 0;     // 0 = 未锁定
    ros::Time   locked_cluster_time_;
    // target_estimator 给的"目标最后一次被 YOLO 实际命中"的微秒时间戳(2026-07-26)。
    //   区别于 locked_cluster_time_(后者只是 cluster_states 回调到达时刻,只要 cluster 还
    //   在 alive 列表里就会被 cluster_states 10Hz 持续刷新,不能反映目标真实可见性)。
    //   触发 strike 的真正 stale 判据是 last_seen_us:UAV 飞过目标位置后 YOLO 看不见目标,
    //   cluster_states 仍把 cluster 当 alive(last_seen_us 是 stale 时刻),沿用其 pose 会
    //   让 guidance 把 UAV 拉回旧位置 → "倒退"飞行 + target_lost 100ms ping-pong。
    uint64_t    locked_cluster_last_seen_us_ = 0; // 0 = 未知(从未锁定)

    // === Attack(105) strike gating ===
    //   clusterStatesCallback 在 SEARCH_STRIKE 分支检测到门控未过(sequential 等前一架 /
    //   coordinated 等同步时间)时:publish type=3 + 锁定 cluster,但不调 triggerStrike(),
    //   把这个"挂起"的 strike 记在此处。advanceAttackPhaseMachine 每 tick 重新评估门控,
    //   通过则补一次 triggerStrike()。
    bool        attack_strike_pending_           = false;
    uint64_t    attack_strike_pending_cluster_id_ = 0;
    ros::Time   attack_strike_pending_time_;     // 挂起起始(用于超时放弃)

    // Attack(105) hover 阶段 setpoint publisher latched 标记(2026-07-26):
    //   跑到 attack_point 起 setpoint publisher 锁 XY+Z 时设 true,直到 Attack skill
    //   推进到 EXIT_PENDING 才清 false。runPx4TakeoffSequence 每 tick 会按 phase 检查
    //   是否该 stop setpoint("避免覆盖其他模块"),但 Attack hover 是 mission_manager
    //   主动 latch 的,phase 又故意保留为 WAYPOINT_FOLLOW 不让 GS 看到 phase 跳变,
    //   所以如果不豁免,runPx4TakeoffSequence 一看到 phase!=TAKING_OFF/HOVERING/HOLDING
    //   立刻把刚起的 publisher 停了 → PX4 OFFBOARD 失流 → AUTO.LAND failsafe。
    //   用这个标志在 runPx4TakeoffSequence 守卫里做豁免。
    bool        attack_hover_setpoint_latched_   = false;
    // === 2026-08-03: TARGET_LOST_HOVER 阶段 latch setpoint publisher ===
    //   目标 lost 进入 hover 等待时,如果直接 disableGuidance + stop waypoint_executor
    //   但不启 setpoint publisher,PX4 OFFBOARD 500ms~1s 内会 COM_OFFBOARD_LOSS_TIMEOUT
    //   → AUTO.LAND failsafe。复用 attack_hover 同样的 latch 模式:startSetpointPublisher
    //   锁当前位置 + 50Hz 喂 OFFBOARD 心跳,resume 时再停。
    bool        target_lost_hover_setpoint_latched_ = false;

    // === Attack(105) 多机协同参数 ===
    double      attack_safety_margin_m_         = 3.0;   // 协同:每架距目标保留距离
    double      attack_close_range_threshold_m_ = 5.0;   // 协同:UAV 已近时 attack_point = 当前位置
    double      attack_sync_strike_buffer_sec_  = 1.0;   // 协同:同步开火缓冲
    double      attack_strike_pending_timeout_sec_ = 60.0; // 门控挂起 strike 超时
    double      attack_hover_timeout_sec_          = 30.0; // Attack(105) 到 attack_point 等 strike trigger 兜底超时(2026-07-26)
                                                                //   与 attack_strike_pending_timeout_sec_ 互为补充:
                                                                //     - 后者:已看到目标但 gating WAITING → 60s 放弃(由 advanceAttackPhaseMachine 兜底)
                                                                //     - 此处:到 attack_point 始终没看到目标 → 30s 放弃(由 advanceSkillStateMachine 兜底)
                                                                //   收到任何 cluster 后前者会接管。本超时只看 attack_hover_enter_time_,不看 attack_strike_pending_。

    // === 已忽略目标黑名单 ===
    std::vector<IgnoredTarget> ignored_targets_;
    double ignored_retention_sec_;     // 黑名单保留时长 (秒)
    double ignored_match_radius_m_;    // 已弃用:label-only 方案不再做位置匹配;
                                       //   保留 param 仅为兼容 yaml 配置,代码内不读取
    // === 2026-07-16: 按 label 类别区分屏蔽半径 ===
    //   person 类(人/行人):     10m (目标小,间隔近)
    //   vehicle 类(车/卡车/巴士):20m (目标大,可能遮挡视野)
    //   其它/未知:              default
    double ignore_radius_person_m_  = 10.0;
    double ignore_radius_car_m_     = 20.0;
    double ignore_radius_default_m_ = 10.0;
    ros::Time last_ignored_cleanup_;   // 上次清理过期项的时间(避免每帧遍历)

    // === 2026-07-16: One Euro Filter 平滑跟踪目标位置(NWU x/y/z 各一路) ===
    //   target_estimator 输出的位置带聚类抖动,直接喂给 guidance_control_node 会让
    //   UAV 在悬停跟踪时来回晃。One Euro Filter:静止/慢速大幅平滑(去抖),
    //   快速运动几乎不滞后(跟上),救援目标多为慢速,默认参数足够。
    OneEuroFilter1D target_smooth_x_;
    OneEuroFilter1D target_smooth_y_;
    OneEuroFilter1D target_smooth_z_;
    ros::Time target_smooth_last_time_;   // 上次滤波调用时间,用于计算 dt
    // 跟踪目标切换(新 lock / action=1/2 释放 / cluster_id 转移)时必须 reset,
    //   否则旧目标的 x_prev/dx_prev 会污染新目标的初值,造成切换瞬间位置偏飞。
    uint64_t target_smooth_locked_cluster_id_ = 0;  // 上次锁定时的 cluster_id
    // WaypointStatus(Phase 4)
    multi_uav_strike::WaypointStatus::ConstPtr latest_wp_status_;
    ros::Time latest_wp_status_time_;
    // 救生索:当 typed WorkMode 与 String 模式冲突时优先 typed
    bool prefer_typed_mode_ = true;  // 保留字段但固定 true(只走 typed WorkMode)
    // MissionState 1Hz 已发布标志(避免重复打印)
    ros::Time last_mission_state_pub_time_;

public:
    MissionManager() : nh_private_("~"),
        current_work_mode_(WorkMode::IDLE),
        current_task_status_(TaskStatus::IDLE),
        is_mission_started_(false),
        is_pose_received_(false),
        is_obstacle_detected_(false),
        obstacle_distance_(100.0),
        low_speed_threshold_(12.0),
        is_spiral_active_(false),
        is_guidance_active_(false),
        guidance_speed_(10.0),    // 默认 10 m/s,可在 mission_manager.yaml 的 ~max_speed 覆盖
        min_guidance_speed_(2.0), // 兜底最低下发速度(2026-07-26),见字段注释
        max_strike_cluster_age_sec_(0.5),   // strike cluster 新鲜度门槛(2026-07-26),见字段注释
        max_report_cluster_age_sec_(5.0),   // 上报 cluster 新鲜度门槛(2026-07-29),见字段注释
        report_dedup_radius_m_(25.0),       // GS 上报位置栅格去重半径(2026-07-29,2026-07-31 40→25),见字段注释
        strike_distance_threshold_(2.0),
        mission_loop_rate_(50.0),
        target_lock_confidence_(0.7),
        spiral_approach_duration_(10.0),
        use_sim_(false),
        self_device_id_(0),
        is_px4_connected_(false),
        is_offboard_mode_(false),
        takeoff_state_(TakeoffState::TAKEOFF_IDLE),
        takeoff_altitude_(50.0),
        takeoff_check_interval_(0.5),
        takeoff_stable_time_(1.0),     // 高度达标后稳定 1 秒（原 2 秒）
        takeoff_hover_time_(0.5),      // 悬停 0.5 秒（原 2 秒）
        takeoff_climb_rate_(3.0),      // 起飞爬升斜坡速率 (m/s),详见 TAKEOFF_TAKEOFF_EXEC 注释
        takeoff_start_alt_(0.0),       // TAKEOFF_EXEC 入口处填实际高度,这里 0 仅占位
        takeoff_retry_count_(0),
        max_takeoff_retries_(3),
        takeoff_retry_delay_(5.0),
        takeoff_failed_logged_(false),
        is_landing_in_progress_(false),
        landing_start_time_(ros::Time()),
        landing_complete_logged_(false),
        landing_already_triggered_(false),
        landing_safety_timeout_sec_(120.0),  // AUTO.LAND 兜底超时 120s
        is_fcu_overridden_(false),
        suppress_recovery_(false),
        override_enter_time_(ros::Time()),
        target_lost_hover_enter_time_(ros::Time()),
        target_lost_hover_active_(false),
        current_phase_(MissionPhase::PHASE_GROUND_IDLE),
        is_waypoints_received_(false),
        current_waypoint_count_(0),
        phase_before_takeoff_(MissionPhase::PHASE_GROUND_IDLE),
        work_mode_before_takeoff_(WorkMode::IDLE),
        post_takeoff_work_mode_(WorkMode::SEARCH_ONLY),
        local_position_rate_hz_(20.0),       // 默认 20Hz (50ms), 板子一般够用
        is_local_position_rate_set_(false),
        set_message_rate_retry_count_(0),
        max_set_message_rate_retries_(3),
        set_message_rate_start_time_(ros::Time()),  // isZero() 表示还没开始尝试
        set_message_rate_max_wait_sec_(5.0),
        ignored_retention_sec_(3600.0),     // 黑名单默认保留 1 小时(覆盖整个 task_flow,
                                              //   新 TaskFlow 到达时 taskFlowCallback 会清空)
        ignored_match_radius_m_(20.0),      // 空间匹配默认 20m
        last_ignored_cleanup_(ros::Time()),
        last_reported_cleanup_(ros::Time()) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();
        initTargetState();

        ROS_INFO("[MissionManager] Initialized. device_id=%d uav_device_sn=%s, low speed threshold: %.1f m/s, use_sim: %s",
                 self_device_id_, (self_uav_device_sn_.empty() ? "(unset)" : self_uav_device_sn_.c_str()),
                 low_speed_threshold_, use_sim_ ? "true" : "false");
    }

    void initParams() {
        nh_private_.param<int>("device_id", self_device_id_, 0);
        // uav_device_sn:与 comm_node / uav_avoidance 同名参数;launch 必须注入设备 SN
        nh_private_.param<std::string>("uav_device_sn", self_uav_device_sn_, "");
        nh_private_.param<double>("low_speed_threshold", low_speed_threshold_, 12.0);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<double>("mission_loop_rate", mission_loop_rate_, 50.0);
        nh_private_.param<double>("target_lock_confidence", target_lock_confidence_, 0.7);
        nh_private_.param<double>("spiral_approach_duration", spiral_approach_duration_, 10.0);
        nh_private_.param<double>("spiral_approach_radius", spiral_approach_radius_, 20.0);

        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, false);
        nh_private_.param<double>("takeoff_altitude", takeoff_altitude_, 50.0);
        nh_private_.param<int>("max_takeoff_retries", max_takeoff_retries_, 3);
        nh_private_.param<double>("takeoff_retry_delay", takeoff_retry_delay_, 5.0);
        nh_private_.param<double>("takeoff_stable_time", takeoff_stable_time_, 1.0);  // 高度达标后稳定时间
        nh_private_.param<double>("takeoff_hover_time", takeoff_hover_time_, 0.5);    // 悬停等待时间
        nh_private_.param<double>("takeoff_climb_rate", takeoff_climb_rate_, 3.0);    // 起飞 setpoint 斜坡速率 (m/s),取代原阶跃跳变
        nh_private_.param<double>("local_position_rate_hz", local_position_rate_hz_, 20.0);  // LOCAL_POSITION_NED 发送频率
        nh_private_.param<int>("max_set_message_rate_retries", max_set_message_rate_retries_, 3);
        nh_private_.param<double>("set_message_rate_max_wait_sec", set_message_rate_max_wait_sec_, 5.0);
        // GPS 参考点: 由 PX4 home 自动填充,param() 仅作 fallback
        nh_private_.param<double>("ref_lat", ref_lat_, 36.096);
        nh_private_.param<double>("ref_lon", ref_lon_, 114.392);
        nh_private_.param<double>("ref_alt", ref_alt_, 100.0);
        // 共享世界参考点(只用于 RViz quad/pose_nwu 的世界坐标转换,不参与控制)
        nh_private_.param<double>("world_ref_lat", world_ref_lat_, 36.058);
        nh_private_.param<double>("world_ref_lon", world_ref_lon_, 114.549);
        nh_private_.param<double>("world_ref_alt", world_ref_alt_, 75.0);
        // 单机自动模式:默认 true → home 到位后把 world_ref 覆盖成本机 home,
        //   使 x/y offset=0,单机永远在原点;多机 launch 设 false 并注入共享 world_ref。
        nh_private_.param<bool>("world_ref_auto", world_ref_auto_, true);

        // RViz 帧名:去掉命名空间前导 '/' 后拼 "/map"(如 /uav0 → uav0/map);无命名空间 → map
        {
            std::string ns = nh_.getNamespace();
            if (!ns.empty() && ns[0] == '/') ns = ns.substr(1);
            viz_frame_ = ns.empty() ? "map" : (ns + "/map");
        }

        // 起飞后 work_mode 兜底值(默认 SEARCH_ONLY,兼容老 GS 不发 SET_WORKMODE 的场景)
        // 用户已通过 SET_WORKMODE 设过 SEARCH_TRACK/STRIKE 时,起飞后保留用户的设置,不会落到这里
        {
            std::string post_takeoff_mode_str;
            nh_private_.param<std::string>("post_takeoff_work_mode", post_takeoff_mode_str, "SEARCH_ONLY");
            if      (post_takeoff_mode_str == "IDLE")              post_takeoff_work_mode_ = WorkMode::IDLE;
            else if (post_takeoff_mode_str == "SEARCH_ONLY")       post_takeoff_work_mode_ = WorkMode::SEARCH_ONLY;
            else if (post_takeoff_mode_str == "SEARCH_TRACK")      post_takeoff_work_mode_ = WorkMode::SEARCH_TRACK;
            else if (post_takeoff_mode_str == "SEARCH_STRIKE")     post_takeoff_work_mode_ = WorkMode::SEARCH_STRIKE;
            else if (post_takeoff_mode_str == "DENIED_ENV_FLIGHT") post_takeoff_work_mode_ = WorkMode::DENIED_ENV_FLIGHT;
            else {
                ROS_WARN_THROTTLE(5.0, "[MissionManager] Unknown ~post_takeoff_work_mode='%s', fallback to SEARCH_ONLY",
                                  post_takeoff_mode_str.c_str());
                post_takeoff_work_mode_ = WorkMode::SEARCH_ONLY;
            }
        }

        // 根据 use_sim 设置 topic
        if (use_sim_) {
            pose_topic_ = "quad/pose";
        } else {
            pose_topic_ = "mavros/local_position/pose";
        }

        // === Phase 2: Skill 流相关参数 ===
        nh_private_.param<double>("entry_gate_timeout", entry_gate_timeout_, 30.0); // ENTRY_PENDING 上限 30s
        nh_private_.param<double>("in_task_timeout",    in_task_timeout_, 0.0);    // IN_TASK 默认不限(0)
        nh_private_.param<double>("gather_timeout",     gather_timeout_, 60.0);    // 101 集结合计超时 60s

        // === 已忽略目标黑名单参数 (Phase 5.5: 目标聚类轻量替代) ===
        nh_private_.param<double>("ignored_retention_sec",  ignored_retention_sec_,  3600.0);  // 默认 1 小时,覆盖整个 task_flow
        nh_private_.param<double>("ignored_match_radius_m", ignored_match_radius_m_, 20.0);  // 默认 20m (已弃用,保留兼容)
        // === 2026-07-16: 按 label 类别区分屏蔽半径 ===
        nh_private_.param<double>("ignore_radius_person_m",  ignore_radius_person_m_,  10.0);
        nh_private_.param<double>("ignore_radius_car_m",     ignore_radius_car_m_,     20.0);
        nh_private_.param<double>("ignore_radius_default_m", ignore_radius_default_m_, 10.0);
        nh_private_.param<int>("unknown_label_id", unknown_label_id_, 1);  // label 映射不到时的兜底类别 ID

        // === 2026-07-16: One Euro Filter 参数 ===
        //   min_cutoff: 静止时截止频率;越小越平滑(去抖越狠),但低速运动时滞后也越大
        //              1.0Hz ≈ 时间常数 0.16s,适合目标几乎不动的救援场景
        //   beta:       速度系数;越大越能跟上快速运动
        //              0.007 适配慢速行人(0~2m/s);若是高速车辆改为 0.05~0.1
        //   d_cutoff:   导数通道截止频率,通常固定 1.0Hz 即可
        // 调参经验:聚类抖动看着还晃 → 降 min_cutoff;UAV 跟不上目标 → 升 beta
        double te_min_cutoff, te_beta, te_d_cutoff;
        nh_private_.param<double>("target_smooth_min_cutoff_hz", te_min_cutoff, 1.0);
        nh_private_.param<double>("target_smooth_beta",          te_beta,       0.007);
        nh_private_.param<double>("target_smooth_d_cutoff_hz",   te_d_cutoff,   1.0);
        target_smooth_x_ = OneEuroFilter1D(te_min_cutoff, te_beta, te_d_cutoff);
        target_smooth_y_ = OneEuroFilter1D(te_min_cutoff, te_beta, te_d_cutoff);
        target_smooth_z_ = OneEuroFilter1D(te_min_cutoff, te_beta, te_d_cutoff);

        // 期望集结 UAV SN 列表(逗号分隔字符串,如 "uav0,uav2")
        std::string sns_str;
        nh_private_.param<std::string>("expected_sns", sns_str, "");
        expected_sns_.clear();
        if (!sns_str.empty()) {
            std::stringstream ss(sns_str);
            std::string item;
            while (std::getline(ss, item, ',')) {
                if (!item.empty()) expected_sns_.push_back(item);
            }
        }
        if (!expected_sns_.empty()) {
            ROS_INFO("[MissionManager] Expected gather SNs: %zu", expected_sns_.size());
        }

        // === 多机集结同步参数(Phase 7 接通)===
        // 伙伴心跳超时:超过此时间未刷新视为失联,失联 SN 从 arrived 集合中剔除
        nh_private_.param<double>("partner_heartbeat_timeout", partner_heartbeat_timeout_, 3.0);
        // 集结门限:到达伙伴数 / 期望伙伴数 ≥ 此值 → 视为"齐了",推进下一条 skill
        // 1.0 = 必须全部到齐;0.7 = 70% 到齐即可(降级兼容掉队 UAV)
        nh_private_.param<double>("gather_threshold", gather_threshold_, 1.0);
        ROS_INFO("[MissionManager] Gather sync: heartbeat_timeout=%.1fs, threshold=%.2f",
                 partner_heartbeat_timeout_, gather_threshold_);

        // === 目标丢失状态机参数(2026-07-23) ===
        // Guidance 阶段目标消失后,先进入 hover 一段时间(航向保持),
        //   再超时则回 WAYPOINT_FOLLOW 继续扫。
        nh_private_.param<double>("target_lost_hold_sec",   target_lost_hold_sec_,   2.0);
        nh_private_.param<double>("target_lost_resume_sec", target_lost_resume_sec_, 10.0);

        // === Attack(105) 多机协同参数(2026-07-26) ===
        // formation=2 协同:每架 UAV attack_point 距目标保留的安全距离(避免贴脸)
        nh_private_.param<double>("attack_safety_margin_m",      attack_safety_margin_m_,      3.0);
        // formation=2 协同:UAV 距目标 < 此值时 attack_point = 当前位置(已在目标附近)
        nh_private_.param<double>("attack_close_range_threshold_m", attack_close_range_threshold_m_, 5.0);
        // formation=2 协同:strike_sync_time 相对最大飞行时间的额外缓冲秒数
        nh_private_.param<double>("attack_sync_strike_buffer_sec",  attack_sync_strike_buffer_sec_,  1.0);
        // formation=1/2 门控挂起 strike 的超时,超时则放弃当前 skill
        nh_private_.param<double>("attack_strike_pending_timeout_sec",
                                                          attack_strike_pending_timeout_sec_, 60.0);
        // Attack(105):到 attack_point 后等 strike trigger 的兜底超时,见 attack_hover_timeout_sec_ 注释
        nh_private_.param<double>("attack_hover_timeout_sec",
                                                          attack_hover_timeout_sec_, 30.0);
        // 下发给 guidance 的兜底最低速度(2026-07-26),见 min_guidance_speed_ 字段注释
        nh_private_.param<double>("min_guidance_speed", min_guidance_speed_, 2.0);
        // strike cluster 新鲜度门槛(2026-07-26),见 max_strike_cluster_age_sec_ 字段注释
        nh_private_.param<double>("max_strike_cluster_age_sec", max_strike_cluster_age_sec_, 0.5);
        // 上报 cluster 新鲜度门槛(2026-07-29),见 max_report_cluster_age_sec_ 字段注释
        nh_private_.param<double>("max_report_cluster_age_sec", max_report_cluster_age_sec_, 5.0);
        // GS 上报位置栅格去重半径(2026-07-29),见 report_dedup_radius_m_ 字段注释
        nh_private_.param<double>("report_dedup_radius_m", report_dedup_radius_m_, 25.0);

        // === 云台俯仰角下发参数(2026-07-29) ===
        // 搜索任务结束后 gimbal_command 下发的归位俯仰角(度)。典型搜索场景用 -15~-45,
        //   默认 0.0 = 水平(云台归位、不影响起飞/集结/返航段的视线)。
        nh_private_.param<double>("gimbal_pitch_default",   gimbal_pitch_default_,   0.0);
        // gimbal_command topic 下发频率(Hz)。详见 mission_manager.yaml 注释。
        nh_private_.param<double>("gimbal_command_rate_hz", gimbal_command_rate_hz_, 5.0);

        // === Attack(105) WP2-gate + 云台联动参数(2026-07-30) ===
        nh_private_.param<double>("wp2_strike_radius_m",            wp2_strike_radius_m_,            30.0);
        nh_private_.param<double>("attack_no_target_hold_sec",      attack_no_target_hold_sec_,      300.0);
        nh_private_.param<double>("attack_wp2_look_delay_sec",      attack_wp2_look_delay_sec_,      2.0);
        nh_private_.param<double>("attack_gimbal_look_default_pitch_deg",
                                                                attack_gimbal_look_default_pitch_deg_, 0.0);
        // === CODEX DIRECT-LOOK START ===
        nh_private_.param<bool>("attack_direct_look_enabled",
                                attack_direct_look_enabled_, true);
        nh_private_.param<double>("attack_direct_look_timeout_sec",
                                  attack_direct_look_timeout_sec_, 4.0);
        nh_private_.param<double>("attack_direct_look_settle_sec",
                                  attack_direct_look_settle_sec_, 1.5);
        nh_private_.param<int>("attack_direct_look_confirm_hits",
                               attack_direct_look_confirm_hits_, 3);
        attack_direct_look_confirm_hits_ = std::max(1, attack_direct_look_confirm_hits_);
        // === CODEX DIRECT-LOOK END ===
    }

    void initSubscribers() {
        // YOLO 检测结果:由 typed YoloDetection 统一接管(multi_uav_strike/YoloDetection)
// 删除原 std_msgs/String legacy 订阅 yolo_result_sub_,避免与 detection_simulator_node
// 发布的 YoloDetection 类型冲突

        // 云台 LOS 角度
        gimbal_los_sub_ = nh_.subscribe(
            "target_los_angle", 10,
            &MissionManager::gimbalLosCallback, this);

        // 目标估计位置
        target_est_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &MissionManager::targetEstPoseCallback, this);

        // 目标估计速度
        target_est_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &MissionManager::targetEstTwistCallback, this);

        // 目标估计 GPS(来自 target_estimator_node 的 target_estimated_gps)
        target_est_gps_sub_ = nh_.subscribe(
            "target_estimated_gps", 10,
            &MissionManager::targetEstGpsCallback, this);

        // 邻居无人机位置由 uav_avoidance_node 通过 /avoidance/cmd 提供(已带优先级 + cmd 类型),
        //   本节点不再单独订阅 other_uav_poses(原 repulsion 向量发布到 /avoidance/vector 但无订阅,已删除)。

        // 其他无人机发现的目标
        inter_uav_target_sub_ = nh_.subscribe(
            "inter_uav/target_info", 10,
            &MissionManager::interUavTargetCallback, this);

        // 本机位置（根据 use_sim 选择 topic）
        self_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &MissionManager::selfPoseCallback, this);

        // 本机真实 GPS(mavros/global_position/global → NavSatFix)
        // 真机必须等 GPS lock 后才有有效值;仿真下 mavros SITL 一般也输出
        self_gps_sub_ = nh_.subscribe(
            "mavros/global_position/global", 10,
            &MissionManager::selfGpsCallback, this);

        // 毫米波雷达障碍检测
        obstacle_sub_ = nh_.subscribe(
            "detection/obstacle", 10,
            &MissionManager::obstacleCallback, this);

        // PX4 SITL: 飞控状态订阅
        if (!use_sim_) {
            mavros_state_sub_ = nh_.subscribe(
                "mavros/state", 10,
                &MissionManager::mavrosStateCallback, this);

            // PX4 SITL: 模式切换 service client
            set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
                "mavros/set_mode");

            // PX4 SITL: 解锁 service client
            arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
                "mavros/cmd/arming");

            // PX4 SITL: COMMAND_LONG 备用通道（某些 commander 状态下 SET_MODE 会被静默拒收）
            command_long_client_ = nh_.serviceClient<mavros_msgs::CommandLong>(
                "mavros/cmd/command");

            ROS_INFO("[MissionManager] PX4 SITL mode enabled, subscribing to mavros/state");
        }

        // === Phase 2: 类型化订阅 ===
        // TaskFlow 入口(本机 device_id 匹配的 flow 才会被采纳)
        task_flow_sub_ = nh_.subscribe("mission/task_flow", 10,
                                       &MissionManager::taskFlowCallback, this);

        // 类型化 WorkMode(GS 升级到 typed 时启用,与 String 模式并存双订阅)
        work_mode_sub_ = nh_.subscribe("mission/work_mode", 10,
                                       &MissionManager::typedWorkModeCallback, this);

        // MAV_CMD_ATTACK typed(地面站确认指令)
        attack_cmd_sub_ = nh_.subscribe("mission/attack_cmd", 10,
                                        &MissionManager::attackCmdCallback, this);

        // cluster_states(target_estimator_node 10Hz 发布的所有 alive cluster 快照)
        // 用 snapshot 而非一次性 cluster_event 是为了解决时序窗口 bug:
        //   原 cluster_event 在"新 cluster 首次进入 FOV"时 publish 一次,如果那一刻 UAV 还没进
        //   IN_TASK (可能在 TRANSIT/ENTRY_PENDING),gating 直接 return,UAV 进入 IN_TASK 后
        //   target_estimator 不会再发 cluster_event,目标永远锁不上。
        //   改用 10Hz snapshot 后,任何在 IN_TASK 期间还在 alive 的 cluster 都会被重新扫到并触发。
        cluster_states_sub_ = nh_.subscribe("cluster_states", 10,
                                            &MissionManager::clusterStatesCallback, this);

        // waypoint_executor 段状态(Phase 4 启用)
        waypoint_status_sub_ = nh_.subscribe("waypoint_executor/status", 10,
                                              &MissionManager::waypointStatusCallback, this);

        // 多机心跳(由 comm_node 发布到 /inter_uav/uav_status) — 多机 gather 同步
        // 50Hz 上限避免高频回调拖累,实际 comm_node 自适应 1/5/20Hz
        inter_uav_status_sub_ = nh_.subscribe(
            "/inter_uav/uav_status", 20,
            &MissionManager::interUavStatusCallback, this);

        // === PX4 home 自动加载(ref_lat/lon/alt 的唯一权威来源)===
        home_position_sub_ = nh_.subscribe("mavros/home_position/home", 10,
                                            &MissionManager::homePositionCallback, this);

        // ===== 弹射起飞流程(协议 TODO: skill_type=100 = 弹射, 106 = 地面) =====
        // 弹射指令触发(由外部弹射控制节点发,本节点不实现发端)
        catapult_trigger_sub_ = nh_.subscribe("mission/catapult_trigger", 10,
                                              &MissionManager::catapultTriggerCallback, this);
        // PX4 抛飞模式就绪(由 comm_node 切完 PX4 模式后发 true)
        px4_catapult_mode_sub_ = nh_.subscribe("px4/catapult_mode_ready", 10,
                                               &MissionManager::px4CatapultModeCallback, this);
    }

    void initPublishers() {
        waypoint_control_pub_ = nh_.advertise<std_msgs::String>(
            "waypoint_executor/control", 10);

        guidance_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "guidance/enable", 10);

        guidance_mode_pub_ = nh_.advertise<std_msgs::String>(
            "guidance/mode", 10);

        guidance_speed_pub_ = nh_.advertise<std_msgs::Float32>(
            "guidance/guidance_speed", 10);

        guidance_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "guidance/target_pose", 10);

        target_filter_debug_pub_ = nh_.advertise<multi_uav_strike::TargetFilterDebug>(
            "mission/target_filter_debug", 10);

        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>(
            "emergency/stop", 10);

        status_pub_ = nh_.advertise<std_msgs::String>(
            "mission/status", 10);

        uav_pose_nwu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "quad/pose_nwu", 10);  // NWU姿态发布(RViz用)

        // 已移除 /mission/mode (String) 发布:auto-handoff 通过 typed Skill
        // (pushCurrentSkillToExecutor → waypoint_executor/skill) 通知 executor 起算

        // PX4 SITL: 起飞位置 setpoint 发布器
        if (!use_sim_) {
            takeoff_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                "mavros/setpoint_position/local", 10);
        }

        // === Phase 2: 类型化发布 ===
        // 1Hz MissionState(GS 关心的上报)
        mission_state_pub_ = nh_.advertise<multi_uav_strike::MissionState>(
            "mission/mission_state", 10);

        // 当前 work_mode 状态(供 starling_bridge 订阅,KCP BIZ_WORK_MODE_REPORT 0x1004 上报)
        work_mode_state_pub_ = nh_.advertise<multi_uav_strike::WorkMode>(
            "mission/current_work_mode", 10);

        // 目标上报(单条)
        detect_target_pub_ = nh_.advertise<multi_uav_strike::DetectTarget>(
            "mission/detect_target", 10);

        // 目标上报(批量预留)
        detect_targets_pub_ = nh_.advertise<multi_uav_strike::DetectTargets>(
            "mission/detect_targets", 10);

        // 跟踪状态(协议 43000)
        tracking_state_pub_ = nh_.advertise<multi_uav_strike::TrackingState>(
            "mission/tracking_state", 10);

        // 向 waypoint_executor 下发 Skill(分段航点)
        waypoint_skill_pub_ = nh_.advertise<multi_uav_strike::Skill>(
            "waypoint_executor/skill", 10);

        // 集结多机状态(Phase 7)— 已删除(2026-07-24)
        //   改用 InterUavStatus 复用机间避障的总线,不再单独 publish /inter_uav/gather_status。

        // 主目标 cluster_id → target_estimator。
        // 多目标改造: estimator 不再自己挑最高置信度,而是 mission_manager
        // 显式指定"正在交战的那个 cluster_id",guidance 由此锚定,避免
        // 最高置信度在目标间跳变。
        primary_target_pub_ = nh_.advertise<std_msgs::UInt64>(
            "mission/primary_target", 10);

        // === 云台俯仰角下发(2026-07-29) ===
        //   102 Search IN_TASK → publish sr.msg.gimbal_pitch(度)
        //   其余 skill_type 或非 IN_TASK → publish gimbal_pitch_default_
        //   topic: /<ns>/mission/gimbal_command (Float32)
        gimbal_command_pub_ = nh_.advertise<std_msgs::Float32>(
            "mission/gimbal_command", 10);
    }

    void initTimers() {
        mission_timer_ = nh_.createTimer(
            ros::Duration(1.0 / mission_loop_rate_),
            &MissionManager::missionTimerCallback, this);

        // === Phase 2: MissionState 1Hz 定频上报 ===
        mission_state_timer_ = nh_.createTimer(
            ros::Duration(1.0),
            &MissionManager::missionStateTimerCallback, this);

        // === Phase 3: Skill 状态机推进(10Hz,稍慢也无妨) ===
        skill_advance_timer_ = nh_.createTimer(
            ros::Duration(0.1),
            &MissionManager::skillAdvanceTimerCallback, this);

        // === TrackingState 周期上报 — 协议 §6 ===
        // GCS 端要连续接收才会高亮 UI。timer 启动时按"未跟踪"1Hz 起步,
        // 跟踪中时 callback 内 setPeriod(5Hz) 切换。
        tracking_state_timer_ = nh_.createTimer(
            ros::Duration(1.0 / TRACKING_STATE_IDLE_RATE_HZ),
            &MissionManager::trackingStateTimerCallback, this);

        // 静态 TF 周期重广播:home 锁定后启动,每 30s 重发一次,防止 rviz 启动晚于广播时机
        // 漏接 latched TF、或 TF 缓存被回收后丢失 viz_frame_。
        viz_tf_timer_ = nh_.createTimer(
            ros::Duration(30.0),
            &MissionManager::vizTfTimerCallback, this);
        viz_tf_timer_.stop();  // 等 home 锁定后再 start()

        // === 云台俯仰角下发(2026-07-29) ===
        // 频率从 yaml ~gimbal_command_rate_hz 读取(默认 5Hz);callback 内只读
        //   current_gimbal_pitch_ 并 publish,状态值由 advanceSkillStateMachine
        //   在 102 Search 进入/退出 IN_TASK 时改写,不在 callback 里改,避免 timer 抢占。
        gimbal_command_timer_ = nh_.createTimer(
            ros::Duration(1.0 / gimbal_command_rate_hz_),
            &MissionManager::gimbalCommandTimerCallback, this);
    }

    void initTargetState() {
        current_target_.is_detected = false;
        current_target_.is_locked = false;  // 修复:初始未锁定,等 YOLO 命中再置 true
        current_target_.lock_time = 0.0;
        current_target_.is_shared = false;
    }

    // ============== 回调函数 ==============
    // 注: 已移除 /mission/mode (String) 和 /mission/waypoint_cmd (Path) 两条 legacy 路径。
    //     模式与航点的统一入口是 typed /mission/task_flow (TaskFlow.msg) +
    //     可选 typed /mission/work_mode (WorkMode.msg)。
    //     起飞/搜索/打击/返航的阶段切换全部由 taskFlowCallback() 解析完成。

    void gimbalLosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 云台 LOS 角度，用于判断跟踪精度
        // msg->z 是跟踪置信度
    }

    void targetEstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!current_target_.is_locked) {
            // 未锁定:仍然 reset filter 避免下次锁定时初值残留
            target_smooth_x_.reset();
            target_smooth_y_.reset();
            target_smooth_z_.reset();
            target_smooth_last_time_ = ros::Time();
            return;
        }

        // === 2026-07-16: One Euro Filter 平滑 position,消除聚类抖动 ===
        // 注意:这里**不要**在 cluster_id 变化时 reset!
        //   target_estimator 因 LOS 投影偏移会频繁给同一物理目标重分配 cluster_id
        //   (见 clusterStatesCallback 中 SEARCH_TRACK transfer 注释,1768 行附近),
        //   如果 cluster_id 变就 reset filter,filter 每帧重来 → 平滑白做。
        //   物理目标真切换 → 由 triggerStrike() / attackCmdCallback() 的 is_locked=false
        //   翻转处理(在那里 reset),这里只看 position。
        (void)locked_cluster_id_;
        (void)target_smooth_locked_cluster_id_;

        ros::Time now = ros::Time::now();
        double te = target_smooth_last_time_.isZero()
                        ? 0.1
                        : (now - target_smooth_last_time_).toSec();
        target_smooth_last_time_ = now;

        // 复制整条 msg(保留 stamp / frame_id / orientation 等),
        // 然后只对 position.x/y/z 覆盖为滤波后值。
        current_target_.pose               = *msg;
        current_target_.pose.pose.position.x = target_smooth_x_.filter(msg->pose.position.x, te);
        current_target_.pose.pose.position.y = target_smooth_y_.filter(msg->pose.position.y, te);
        current_target_.pose.pose.position.z = target_smooth_z_.filter(msg->pose.position.z, te);

        // === 2026-07-16: Debug — 把 raw 和 filtered 一起发到 debug topic ===
        //   用来 rqt_plot / rostopic echo 对比,看 One Euro 是否真的在工作
        //   以及 residual 噪声(=|delta|)还有多大。publish 频率自然 = 10Hz
        //   (target_estimated_pose 的频率);若嫌刷屏可以 throttle。
        multi_uav_strike::TargetFilterDebug dbg;
        dbg.header.stamp = now;
        dbg.header.frame_id = msg->header.frame_id;
        dbg.pos_raw.x       = msg->pose.position.x;
        dbg.pos_raw.y       = msg->pose.position.y;
        dbg.pos_raw.z       = msg->pose.position.z;
        dbg.pos_filtered.x  = current_target_.pose.pose.position.x;
        dbg.pos_filtered.y  = current_target_.pose.pose.position.y;
        dbg.pos_filtered.z  = current_target_.pose.pose.position.z;
        dbg.pos_delta.x     = dbg.pos_raw.x - dbg.pos_filtered.x;
        dbg.pos_delta.y     = dbg.pos_raw.y - dbg.pos_filtered.y;
        dbg.pos_delta.z     = dbg.pos_raw.z - dbg.pos_filtered.z;
        // 速度维度本节点不直接产生(由 guidance_control_node 计算),填 0
        dbg.vel_raw = dbg.vel_filtered = dbg.vel_delta = geometry_msgs::Vector3();
        dbg.cluster_id = locked_cluster_id_;
        dbg.is_locked  = true;
        dbg.source     = "position";
        target_filter_debug_pub_.publish(dbg);
    }

    void targetEstTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (current_target_.is_locked) {
            current_target_.twist = *msg;
        }
    }

    /**
     * target_estimated_gps 回调 — 缓存目标真实 GPS(由 target_estimator_node 用 UAV GPS + 本地偏移算出)
     * 仅当 status.status >= STATUS_FIX 时认为有效。
     * 注:暂不要求 is_locked,因为 buildDetectTarget 在 lock 之前也可能用
     *(typedYoloCallback / checkYoloDrivenStrike 都会立即构造 DetectTarget)
     */
    void targetEstGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            current_target_.gps = *msg;
            current_target_.has_gps = true;
        }
    }

    /**
     * PX4 home 回调 — 锁一次
     * PX4 的 LOCAL_POSITION_NED 参考系由 EKF2 在启动时锁定,
     * 后续 home 更新(MAV_CMD_DO_SET_HOME / disarm)不会重置 EKF2 origin,
     * 也不让 local_position 跳变。如果跟着 home 更新去重算 NED,
     * 反而让 setpoint 与 UAV 当前 local_position 不在同一 frame → 偏飞。
     */
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (ref_initialized_) return;  // 锁一次
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            return;  // PX4 home 未稳定前发 0/0,忽略
        }
        ROS_WARN("[MissionManager] >>>> PX4 home locked: (%.7f, %.7f, %.2f)",
                 msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        ref_lat_ = msg->geo.latitude;
        ref_lon_ = msg->geo.longitude;
        ref_alt_ = msg->geo.altitude;
        ref_initialized_ = true;

        // 单机自动模式:把 world_ref 覆盖成本机 home → 偏移 = 0 → identity TF → 永在原点。
        // 多机 launch 关掉 world_ref_auto 并注入共享 world_ref(通常 = uav0 home),
        // 各机据此算出自己相对 world_ref 的偏移。
        if (world_ref_auto_) {
            world_ref_lat_ = ref_lat_;
            world_ref_lon_ = ref_lon_;
            world_ref_alt_ = ref_alt_;
            ROS_WARN("[MissionManager] world_ref_auto: world_ref <- home (%.7f, %.7f, %.2f)",
                     world_ref_lat_, world_ref_lon_, world_ref_alt_);
        }

        // 广播 static TF: map -> viz_frame_
        // 本机所有 RViz 内容(plane / 航线 / marker)都发布在 viz_frame_(local, 原点=本机 home),
        // 由这条 TF 统一平移到世界 map 帧。平移 = 本机 home 相对 world_ref 的 NWU 偏移。
        //   north_m = (home_lat - world_ref_lat) * 111000
        //   east_m  = (home_lon - world_ref_lon) * 111000 * cos(world_ref_lat)
        //   NWU: x = north_m, y = -east_m, z = 0(各机 home 高一致 + 避 geoid 陷阱)
        publishVizFrameTf();
        // 启动周期重广播 timer,防止 rviz 启动晚于 home 锁定时漏接 latched TF。
        if (!viz_tf_timer_.hasStarted()) {
            viz_tf_timer_.start();
        }
    }

    /**
     * 广播 static TF: map -> viz_frame_
     * 本机所有 RViz 内容(plane / 航线 / marker)都发布在 viz_frame_(local, 原点=本机 home),
     * 由这条 TF 统一平移到世界 map 帧。平移 = 本机 home 相对 world_ref 的 NWU 偏移。
     *   north_m = (home_lat - world_ref_lat) * 111000
     *   east_m  = (home_lon - world_ref_lon) * 111000 * cos(world_ref_lat)
     *   NWU: x = north_m, y = -east_m, z = 0(各机 home 高一致 + 避 geoid 陷阱)
     * 周期性重广播是为了兜底 rviz 启动晚于 home 锁定、或 TF 缓存被回收时
     * 仍能找到 viz_frame_(否则 target_estimator / gimbal 等会报 Frame does not exist)。
     */
    void publishVizFrameTf() {
        if (!ref_initialized_) return;  // home 还没锁定,无参考点可发
        const double cos_lat = std::cos(world_ref_lat_ * M_PI / 180.0);
        const double north_m = (ref_lat_ - world_ref_lat_) * 111000.0;
        const double east_m  = (ref_lon_ - world_ref_lon_) * 111000.0 * cos_lat;

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "map";       // 世界帧(RViz Fixed Frame)
        tf_msg.child_frame_id  = viz_frame_;  // 本机 local 帧
        tf_msg.transform.translation.x =  north_m;
        tf_msg.transform.translation.y = -east_m;
        tf_msg.transform.translation.z =  0.0;
        tf_msg.transform.rotation.w = 1.0;    // 只平移,不旋转
        static_tf_broadcaster_.sendTransform(tf_msg);
        ROS_INFO_THROTTLE(60.0,
            "[MissionManager] static TF map -> %s : (%.2f, %.2f, 0)",
            viz_frame_.c_str(), north_m, -east_m);
    }

    /**
     * 周期重广播 static TF,见 publishVizFrameTf() 注释。
     */
    void vizTfTimerCallback(const ros::TimerEvent&) {
        publishVizFrameTf();
    }

    /**
     * NED (本地 NED/Earth 偏移,米) -> GPS (WGS84 lat/lon, AMSL alt 米)
     *   旧 gpsToNed 已删除 — 唯一调用者 otherUavPosesCallback 移除后无 caller,
     *   nedToGps 仍保留给 cluster 上行使用。
     * 注:ref_initialized_ 必须为 true(PX4 home 已锁定)才能保证精度
     */
    void nedToGps(double ned_x, double ned_y, double ned_z,
                  double& lat, double& lon, double& alt) const {
        const double EARTH_R = 6378137.0;
        const double RAD_PER_DEG = M_PI / 180.0;
        const double DEG_PER_RAD = 180.0 / M_PI;
        lat = ref_lat_ + ned_x / EARTH_R * DEG_PER_RAD;
        double cos_lat = std::cos(ref_lat_ * RAD_PER_DEG);
        if (cos_lat < 1e-6) cos_lat = 1e-6;  // 防止 cos(90°)=0 除零
        lon = ref_lon_ - ned_y / (EARTH_R * cos_lat) * DEG_PER_RAD;  // NWU +y=西 → 经度东向 = -y
        alt = ref_alt_ - ned_z;  // NED z 下向 → AMSL 上向
    }

    void interUavTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 收到其他无人机发现的目标
        // 可以选择是否协同跟踪
        ROS_INFO_THROTTLE(10.0, "[MissionManager] Received target info from other UAV: %s",
                 msg->header.frame_id.c_str());
    }

    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_mavros_state_ = *msg;

        // 检测 FCU 断连 (上次的 connected=true, 这次变 false) — 清掉 LOCAL_POSITION_NED
        // 频率标志, 下次重连时重新发 SET_MESSAGE_INTERVAL (PX4 重启会重置消息间隔)
        bool was_connected = is_px4_connected_;
        is_px4_connected_ = msg->connected;
        if (was_connected && !is_px4_connected_) {
            ROS_WARN("[MissionManager] >>>>> PX4 FCU DISCONNECTED — will re-set LOCAL_POSITION_NED "
                     "rate on reconnect");
            is_local_position_rate_set_ = false;
            set_message_rate_retry_count_ = 0;
            set_message_rate_start_time_ = ros::Time();  // isZero(), 下次重连时重新记录起始
        }

        is_offboard_mode_ = (msg->mode == "OFFBOARD");

        // 检测 OFFBOARD 模式切换（throttle 避免 mavros 1Hz 状态推送时刷屏）
        if (is_offboard_mode_ && !isTakeoffComplete()) {
            ROS_WARN_THROTTLE(5.0, "[MissionManager] OFFBOARD mode detected, waiting for takeoff complete...");
        }

        // === 2026-07-23: 飞手接管检测 + AUTO.LAND 完成收尾 ===
        // 接管判定:PX4 mode != OFFBOARD,且当前 phase 处于飞行阶段 (排除 GROUND_IDLE / FAILED / COMPLETE)。
        // 快照规则:只在 OFFBOARD → 其他模式的瞬变上写,后续模式跳变保留首次快照(用户口径 B)。
        bool prev_override = is_fcu_overridden_;
        bool aircraft_live_phase =
            current_phase_ != MissionPhase::PHASE_GROUND_IDLE &&
            current_phase_ != MissionPhase::PHASE_FAILED &&
            current_phase_ != MissionPhase::PHASE_COMPLETE &&
            // TAKING_OFF 不算"飞行中被接管":taskFlowCallback 在 triggerOffboardAndArm 之前
            //   就先把 phase 切到 TAKING_OFF(为让 runPx4TakeoffSequence 门控通过),此时 PX4 仍
            //   在 AUTO.LOITER/AUTO.TAKEOFF 等"起飞前"模式。如果不豁免,mavrosStateCallback 下一拍
            //   会立刻把 phase 打成 OVERRIDDEN,runPx4TakeoffSequence 在 2964 行门控处早 return,
            //   起飞卡死在 IDLE — 这就是用户在 SITL 启动后"一直是 AUTO.LOITER"的根因。
            // HOLDING/RETURNING 同理不算被接管:RETURNING 期间 PX4 本就在 AUTO.LAND(非 OFFBOARD),
            // HOLDING 落地收尾窗口也可能短暂并存 AUTO.LAND。
            current_phase_ != MissionPhase::PHASE_TAKING_OFF &&
            current_phase_ != MissionPhase::PHASE_HOLDING &&
            current_phase_ != MissionPhase::PHASE_RETURNING;
        is_fcu_overridden_ = (msg->mode != "OFFBOARD") && aircraft_live_phase;

        if (is_fcu_overridden_ && !prev_override) {
            // 接管起点:写一次快照 + 切 phase 通知心跳
            override_enter_time_ = ros::Time::now();
            override_snapshot_.phase_when_overrided     = current_phase_;
            override_snapshot_.work_mode_when_overrided = current_work_mode_;
            override_snapshot_.flow_id_when_overrided    = current_flow_id_;
            override_snapshot_.skill_id_when_overrided   =
                (current_skill_index_ < skill_queue_.size())
                    ? skill_queue_[current_skill_index_].msg.skill_id
                    : std::string();
            override_snapshot_.snapshot_time             = ros::Time::now();
            current_phase_ = MissionPhase::PHASE_OVERRIDDEN;
            ROS_ERROR("[MissionManager] >>>> PILOT OVERRIDE: px4_mode=%s prev_phase=%s prev_work_mode=%s flow=%s skill=%s",
                      msg->mode.c_str(),
                      missionPhaseToString(override_snapshot_.phase_when_overrided).c_str(),
                      workModeToString(override_snapshot_.work_mode_when_overrided).c_str(),
                      override_snapshot_.flow_id_when_overrided.c_str(),
                      override_snapshot_.skill_id_when_overrided.c_str());
        }
        if (!is_fcu_overridden_ && prev_override) {
            // 切回 OFFBOARD:走恢复流程
            // suppress_recovery_:taskFlowCallback 主动 setMode("OFFBOARD") 时拉起,
            //   期间不能让 snapshot 恢复(否则会污染新 flow 的 phase 初始化)。
            if (suppress_recovery_) {
                ROS_WARN("[MissionManager] Override transition detected but suppressed "
                         "(taskFlowCallback is taking back OFFBOARD proactively)");
            } else {
                executeRecoveryFromOverride();
            }
        }

        // AUTO.LAND 完成检测:armed=false 且正在落地 → 视为已经完成,等 GCS 发新 TaskFlow。
        // 原 triggerPx4Landing 只 set is_landing_in_progress_=true,checkLandingComplete 只 log 不清,
        // 此处集中重置。任何 disarm 状态都视为地面 idle，避免 phase=OVERRIDDEN
        // 与 is_fcu_overridden_=true 同时残留到下一次 TaskFlow，导致 taskFlowCallback 走
        // setMode(OFFBOARD) 死路。
        //
        // === 2026-07-26 fix: 必须 armed=true→false 跳变才能触发 ===
        //   原版 6ff6e90 把 PHASE_TAKING_OFF 加进 was_in_flight 后,首条 mavrosStateCallback
        //   (msg->armed=false,phase=TAKING_OFF,刚由 taskFlowCallback 设上去)就会误触发:
        //     - takeoff 序列刚把 phase 切到 TAKING_OFF → PX4 还没真 arm → msg->armed 本来就是 0
        //     - disarm-on-landing 看到 "armed=false + TAKING_OFF" 立刻 reset phase=GROUND_IDLE
        //     - runPx4TakeoffSequence 在 2964 行门控早 return,起飞卡死;用户看到的现象:
        //         "triggerOffboardAndArm SUCCESS 之后立刻 UAV disarmed on landing → AUTO.LAND"
        //   只有 armed 从 1→0 的真 disarm 才需要 reset (PX4 真的飞不起来),初始 0 不是 disarm 事件。
        const bool was_armed = current_mavros_state_.armed;  // 在 line 1248 赋值之前的快照
        if (!msg->armed && was_armed) {
            const bool was_in_flight =
                is_landing_in_progress_ ||
                current_phase_ == MissionPhase::PHASE_OVERRIDDEN ||
                current_phase_ == MissionPhase::PHASE_RETURNING ||
                current_phase_ == MissionPhase::PHASE_TAKING_OFF;
            if (was_in_flight) {
                ROS_WARN("[MissionManager] UAV disarmed on landing (mode=%s prev_phase=%s) — "
                         "resetting to PHASE_GROUND_IDLE for fresh takeoff",
                         msg->mode.c_str(), missionPhaseToString().c_str());
                is_landing_in_progress_  = false;
                is_fcu_overridden_       = false;
                current_phase_           = MissionPhase::PHASE_GROUND_IDLE;
                // 注意:不动 has_task_flow_,等 GCS 显式发新 flow
            }
        }
    }

    // === 2026-07-23: 从飞手接管中受控恢复 ===
    // 触发时机:mavrosStateCallback 检测到 is_fcu_overridden_ 由 true→false,即 PX4 mode 重新变回
    // OFFBOARD(由 GS 发 MAV_CMD_DO_SET_MODE(176, OFFBOARD) 直通飞控完成)。因此进入本函数时飞机
    // *已经* 在 OFFBOARD,无需再调 setMode()——那样会在订阅回调里跑 5s 阻塞 spinOnce,造成重入。
    // 恢复策略:按接管瞬间的快照还原 phase;若快照是 GUIDANCE_*,看锁定目标是否还新鲜决定继续制导
    // 还是转 TARGET_LOST_HOVER。
    void executeRecoveryFromOverride() {
        ROS_WARN("[MissionManager] <<<< RECOVERY from override: restoring phase=%s work_mode=%s flow=%s skill=%s",
                 missionPhaseToString(override_snapshot_.phase_when_overrided).c_str(),
                 workModeToString(override_snapshot_.work_mode_when_overrided).c_str(),
                 override_snapshot_.flow_id_when_overrided.c_str(),
                 override_snapshot_.skill_id_when_overrided.c_str());

        // 1. 还原接管瞬间的执行阶段
        current_phase_ = override_snapshot_.phase_when_overrided;

        // 2. 若接管前在制导阶段,判断锁定目标是否还在视野(用锁定时间新鲜度近似)
        if (current_phase_ == MissionPhase::PHASE_GUIDANCE_TRACK ||
            current_phase_ == MissionPhase::PHASE_GUIDANCE_STRIKE) {
            ros::Time now = ros::Time::now();
            double age = (now - locked_cluster_time_).toSec();
            if (locked_cluster_id_ != 0 && age < target_lost_hold_sec_) {
                // 目标仍新鲜:is_guidance_active_ 在接管期间未被动过,guidance_control_node 立即接管
                ROS_WARN("[MissionManager] Guidance resume (target fresh, cluster=%lu age=%.1fs)",
                         (unsigned long)locked_cluster_id_, age);
            } else {
                // 目标已丢失:转悬停状态,关制导。setpoint 交给 waypoint_executor 的 stop(0 vel),
                // 航向由 yaw_speed_gate_ 在 v=0 时天然冻结(保持当前值,用户口径)。
                ROS_WARN("[MissionManager] Guidance target lost on recovery (cluster=%lu age=%.1fs) → TARGET_LOST_HOVER",
                         (unsigned long)locked_cluster_id_, age);
                current_phase_                = MissionPhase::PHASE_TARGET_LOST_HOVER;
                target_lost_hover_active_     = true;
                target_lost_hover_enter_time_ = now;
                disableGuidance();
            }
        } else if (current_phase_ == MissionPhase::PHASE_WAYPOINT_FOLLOW ||
                   current_phase_ == MissionPhase::PHASE_HOVERING) {
            // waypoint_executor 的段状态(segment_phase_/arrive_idx_/seg_start_x_/y_)在接管期间被
            // 天然保留,setpoint 流也未断,切回 OFFBOARD 后立即被飞控接走,无需额外指令。
            ROS_WARN("[MissionManager] Waypoint/hover resume from override");
        } else {
            ROS_WARN("[MissionManager] Recovery restored phase=%s (no guidance/waypoint action)",
                     missionPhaseToString(current_phase_).c_str());
        }

        publishMissionState();  // 让 GS 立即看到恢复后的 phase
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // 仿真输入是 NED 坐标系
            current_pose_ = *msg;
        } else {
            // Mavros 输入是 ENU/FLU 坐标系，需要转换为 NED/FRD
            // 位置: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            current_pose_.pose.position.x = msg->pose.position.y;
            current_pose_.pose.position.y = msg->pose.position.x;
            current_pose_.pose.position.z = -msg->pose.position.z;
            // 四元数 ENU/FLU -> NED/FRD:
            //   q_ned = q_T * q_enu * q_S
            //   q_T = (1/√2, 1/√2, 0, 0)  世界系 ENU→NED(绕(1,1,0)轴 180°)
            //   q_S = (1, 0, 0, 0)         机体系 FLU→FRD(绕 x 轴 180°)
            // 展开后:
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_pose_.pose.orientation.w = kSqrtHalf * (qe.w + qe.z);
            current_pose_.pose.orientation.x = kSqrtHalf * (qe.x + qe.y);
            current_pose_.pose.orientation.y = kSqrtHalf * (qe.x - qe.y);
            current_pose_.pose.orientation.z = kSqrtHalf * (qe.w - qe.z);
            current_pose_.header = msg->header;
        }
        is_pose_received_ = true;

        // 发布NWU姿态用于RViz显示
        // 内容全在本机 local 帧 viz_frame_(原点=本机 home),多机分离交给 static TF map->viz_frame_。
        //   位置:local NED -> NWU(x=north, y=-east, z=-down),不再自己算 GPS 偏移。
        //   单机:TF identity → 原点;多机:TF 平移 → 各机分开。
        // 控制链路完全不受影响(此 topic 仅 RViz 订阅)。
        geometry_msgs::PoseStamped uav_pose_nwu;
        uav_pose_nwu.pose.position.x = current_pose_.pose.position.x;   // north
        uav_pose_nwu.pose.position.y = -current_pose_.pose.position.y;  // -east
        uav_pose_nwu.pose.position.z = -current_pose_.pose.position.z;  // -down (up)
        // 四元数 NED/FRD -> NWU/FLU:对 yaw,只取 y,z 取反即可保持航向角取负
        // (NED yaw 83° CW from N → NWU yaw -83° CCW from N)
        uav_pose_nwu.pose.orientation.w =  current_pose_.pose.orientation.w;
        uav_pose_nwu.pose.orientation.x =  current_pose_.pose.orientation.x;
        uav_pose_nwu.pose.orientation.y = -current_pose_.pose.orientation.y;
        uav_pose_nwu.pose.orientation.z = -current_pose_.pose.orientation.z;
        uav_pose_nwu.header.stamp = ros::Time::now();
        uav_pose_nwu.header.frame_id = viz_frame_;  // 本机 local 帧,由 static TF 偏移
        uav_pose_nwu_pub_.publish(uav_pose_nwu);
    }

    /**
     * mavros/global_position/global 回调 — 真实 GPS 缓存
     * 仅当 status.status >= STATUS_FIX 时认为有效;否则保留上次值但 is_gps_received_ 不置 true,
     * 避免 buildDetectTarget 用无效 GPS 当目标 GPS 上行。
     */
    void selfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            current_gps_ = *msg;
            is_gps_received_ = true;
        }
    }

    void obstacleCallback(const std_msgs::String::ConstPtr& msg) {
        // 毫米波雷达障碍检测
        // 格式："distance,angle" 或自定义
        // 简化处理
        if (msg->data.empty()) {
            is_obstacle_detected_ = false;
            return;
        }

        // 解析障碍物距离
        try {
            obstacle_distance_ = std::stod(msg->data);
            is_obstacle_detected_ = (obstacle_distance_ < 30.0);  // 30米内认为有障碍
        } catch (...) {
            is_obstacle_detected_ = false;
        }
    }

    /**
     * 弹射起飞 trigger 回调
     * 由外部弹射控制节点触发,本节点不实现发端
     * 仅当 is_catapult_takeoff_=true(当前 task_flow 首发是 skill_type=100)时认账,
     *   否则丢弃(防御性:防止地面起飞误触发)
     */
    void catapultTriggerCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data) return;
        if (!is_catapult_takeoff_) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] catapult_trigger received but current task_flow "
                                   "is NOT catapult takeoff (skill_type=%s) — ignored",
                              (skill_queue_.empty() ? "none" :
                               std::to_string(skill_queue_[0].msg.skill_type).c_str()));
            return;
        }
        catapult_trigger_received_ = true;
        ROS_WARN("[MissionManager] >>>> Catapult TRIGGER received (armed in PX4 catapult mode=%s)",
                 is_px4_catapult_mode_ ? "YES" : "NO (waiting for comm_node)");
    }

    /**
     * PX4 抛飞模式就绪回调(由 comm_node 切完 PX4 模式后发 true)
     * - true:  comm_node 已把 PX4 切到抛飞模式(MAV_CMD_DO_GO_ARMED/类似指令)
     * - false: 收到 reset,准备 reset 流程
     * 配合 is_catapult_takeoff_ 使用:只有当前 task_flow 是弹射起飞时才接收
     */
    void px4CatapultModeCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_px4_catapult_mode_ = msg->data;
        ROS_WARN("[MissionManager] PX4 catapult_mode_ready = %s",
                 is_px4_catapult_mode_ ? "TRUE (PX4 in catapult mode)" : "FALSE");
    }

    /**
     * Whether UAV is currently in flight (used to reject takeoff task_flow while airborne)
     * Ground states (return false): GROUND_IDLE / FAILED / HOLDING
     * In-flight states (return true): TAKING_OFF / HOVERING / WAYPOINT_FOLLOW /
     *   GUIDANCE_TRACK / GUIDANCE_STRIKE / RETURNING / COMPLETE
     * Note: COMPLETE counts as in-flight since UAV is usually still airborne / returning.
     */
    bool isInFlight() const {
        switch (current_phase_) {
            case MissionPhase::PHASE_GROUND_IDLE:
            case MissionPhase::PHASE_FAILED:
            case MissionPhase::PHASE_HOLDING:
                return false;
            case MissionPhase::PHASE_TAKING_OFF:
            case MissionPhase::PHASE_HOVERING:
            case MissionPhase::PHASE_WAYPOINT_FOLLOW:
            case MissionPhase::PHASE_GUIDANCE_TRACK:
            case MissionPhase::PHASE_GUIDANCE_STRIKE:
            case MissionPhase::PHASE_RETURNING:
            case MissionPhase::PHASE_COMPLETE:
            case MissionPhase::PHASE_OVERRIDDEN:          // 飞手接管:物理上仍在空中
            case MissionPhase::PHASE_TARGET_LOST_HOVER:   // 目标丢失悬停:仍在空中
                return true;
            default:
                return false;
        }
    }

    // ============== Phase 2: 类型化回调 ==============

    /**
     * TaskFlow 入口
     * - device_id 匹配 self_device_id_ 的 flow 才会被采纳(其它 UAV 的 flow 透传给 comm_node 转发)
     * - 收到后:清空 skill_queue_,把 flow.skills 拷贝进去,current_skill_index_=0
     *   然后立刻把第一个 skill 推到 waypoint_executor + 标记 has_task_flow_=true
     */
    void taskFlowCallback(const multi_uav_strike::TaskFlow::ConstPtr& msg) {
        // device_id 过滤(0=广播,其它=本机 device_id)
        if (msg->device_id != 0 && msg->device_id != self_device_id_) {
            return;
        }

        // === 飞手接管场景下的"新任务"路径(2026-07-23,用户口径 E)===
        // 与"继续任务"路径(等 GS 发 50000 切回 OFFBOARD, mavrosStateCallback 触发 snapshot recovery)
        //   并列的另一条 UX:GS 直接重发新 TaskFlow, 由程序**主动**拿回 OFFBOARD 控制权。
        //
        // 必须按顺序:
        //   1) suppress_recovery_=true  → 防止 setMode 的 spinOnce 期间 mavrosStateCallback
        //      看到"OFFBOARD 且 prev_override=true"后跑 snapshot recovery, 把新 flow 的 phase
        //      初始化污染成旧 phase(尤其 GUIDANCE_*/TARGET_LOST_HOVER 那条支路)。
        //   2) setMode("OFFBOARD") 阻塞等 PX4 确认(最多 5s)。失败(飞手在 5s 内又抢了)
        //      → 直接 return,新 flow 不入队(GS 应改为先发 50000)。
        //   3) suppress_recovery_=false,mavrosStateCallback 此时已把 is_fcu_overridden_ 刷成 false,
        //      未来回调 prev_override=false,不会再触发 recovery。
        //
        // === 2026-07-28 扩展:接管场景覆盖 PHASE_HOLDING ===
        //   原 is_fcu_overridden_ 在 mavrosStateCallback L1322-1335 把 PHASE_HOLDING 排除在
        //   aircraft_live_phase 外(避免 setpoint publisher 切换瞬间的 PX4 mode 抖动误判),
        //   代价:HOLDING 期间飞手真接管(PX4 mode 漂到 POSITION/AUTO.RTL/AUTO.LAND 等)
        //   不会被识别,新 TaskFlow 不会主动 setMode,飞手持续控制。
        //   业务上 HOLDING 本就是"等新 task_flow"状态,GS 一旦下发新 flow 即可视为接管信号,
        //   无需等到 phase 翻 OVERRIDDEN。这里把 HOLDING + mode!=OFFBOARD + armed 也并入同一
        //   setMode 路径;false positive 几乎不存在(HOLDING 期间 PX4 短暂漂移 + GS 同时下发
        //   新 task 这种三连击,即使触发 setMode 也是"GS 想要控制权"的合理表现)。
        const bool need_takeback_offboard =
            is_fcu_overridden_ ||
            (current_phase_ == MissionPhase::PHASE_HOLDING &&
             current_mavros_state_.mode != "OFFBOARD" &&
             current_mavros_state_.armed);
        if (need_takeback_offboard) {
            if (!current_mavros_state_.armed) {
                // 飞机已 disarm 落地(典型场景:AUTO.LAND 完成 / 飞手手动 disarm)。
                // PX4 在 AUTO.LAND/AUTO.LOITER 等状态下不允许直接切 OFFBOARD。
                // 不走 setMode(OFFBOARD) 死路,改为清掉接管标志 + phase 重置,
                // 让后面 takeoff 段走标准的 triggerOffboardAndArm() 序列。
                ROS_WARN("[MissionManager] >>>> Disarmed on landing during OVERRIDE (mode=%s) — "
                         "clearing OVERRIDE flag, treating as fresh takeoff (flow_id=%s)",
                         current_mavros_state_.mode.c_str(), msg->flow_id.c_str());
                is_fcu_overridden_ = false;
                current_phase_     = MissionPhase::PHASE_GROUND_IDLE;
            } else {
                ROS_WARN("[MissionManager] >>>> New TaskFlow during OVERRIDE — proactively requesting OFFBOARD "
                         "(flow_id=%s, skills=%lu)", msg->flow_id.c_str(), msg->skills.size());
                suppress_recovery_ = true;
                bool ok = setMode("OFFBOARD");
                suppress_recovery_ = false;
                if (!ok) {
                    ROS_ERROR("[MissionManager] >>>> OFFBOARD switch failed (pilot may have re-overridden) — "
                              "dropping new TaskFlow flow_id=%s. GS should retry or send SET_MODE(50000) first.",
                              msg->flow_id.c_str());
                    return;
                }
                ROS_WARN("[MissionManager] >>>> OFFBOARD confirmed, proceeding with new TaskFlow flow_id=%s",
                         msg->flow_id.c_str());
            }
        }

        // === 飞行安全门:若 UAV 已在飞行中,且新 task_flow 首发是 takeoff skill ===
        // 用户口径 D(2026-07-23):已空中时不再一律 REJECT,而是"剥掉首个 takeoff skill 再入队"——
        //   典型场景是飞手接管后 GS 直接重发完整 flow([Takeoff, Search, ...]),飞机已在空中,
        //   起飞段应跳过,直接从后续 skill 续飞。
        //   仅当整个 flow 只有一个孤立 takeoff([Takeoff])时仍 REJECT,避免起飞后空跑无后续任务。
        // 例外:phase=GROUND_IDLE/HOLDING 时(地面或悬停待命)takeoff 正常执行,不剥。
        //
        // "已空中"判定:**必须** isInFlight() && current_mavros_state_.armed 同时为真 —
        //   之前只看 isInFlight() 有 bug:飞手手动 disarm 落地后 phase 还停在 PHASE_OVERRIDDEN
        //   (我们刚把 OVERRIDDEN 算成 in-flight),此时 GS 发 [Takeoff] 会被错误剥掉,而飞机其实
        //   在地上 disarmed,必须执行 takeoff。加上 armed 检查后:disarm 状态下无论 phase 是什么,
        //   takeoff 都正常执行(地面起飞流程:arm + 爬升)。
        bool uav_airborne = isInFlight() && current_mavros_state_.armed;
        bool strip_leading_takeoff = false;
        if (!msg->skills.empty() &&
            (msg->skills[0].skill_type == 100 || msg->skills[0].skill_type == 106) &&
            uav_airborne) {
            if (msg->skills.size() <= 1) {
                ROS_ERROR("[MissionManager] >>>> REJECT single-takeoff task_flow: UAV already in flight "
                          "(phase=%s, armed=%d, takeoff skill_type=%u, no follow-on skills). Ignored for safety.",
                          missionPhaseToString().c_str(),
                          current_mavros_state_.armed ? 1 : 0,
                          static_cast<unsigned>(msg->skills[0].skill_type));
                return;
            }
            strip_leading_takeoff = true;
            ROS_WARN("[MissionManager] >>>> Skipping leading takeoff (already airborne, phase=%s armed=%d) — "
                     "starting from skill '%s' (type=%u)",
                     missionPhaseToString().c_str(),
                     current_mavros_state_.armed ? 1 : 0,
                     msg->skills[1].skill_id.c_str(),
                     static_cast<unsigned>(msg->skills[1].skill_type));
        }

        // === 返航中收到新 task_flow 的兜底 ===
        // 场景:Return skill 触发了 AUTO.LAND,但中途(未 disarm 前)GS 决定不降了,下发新任务
        // 处理:清掉 is_landing_in_progress_,让 advanceSkillStateMachine 推新 skill
        //   不需要切 PX4 mode(下面会触发起飞流程,OFFBOARD+ARM 序列会接管)
        // 同时清掉 landing_already_triggered_:让新 task_flow 里如果有 Return,能再次触发。
        if (is_landing_in_progress_) {
            ROS_WARN("[MissionManager] New TaskFlow arrived during AUTO.LAND — aborting landing, "
                     "will reset to PHASE_GROUND_IDLE for fresh takeoff");
            is_landing_in_progress_      = false;
            landing_complete_logged_     = false;
            landing_already_triggered_   = false;
            current_phase_               = MissionPhase::PHASE_GROUND_IDLE;
        }

        // 防御：如果当前 setpoint publisher 在跑(说明前序 flow 结束后进入 PHASE_HOLDING 或正在 takeoff),
        // 新 task_flow 到达时先停掉,避免与 waypoint_executor 双发 setpoint 冲突。
        if (setpoint_running_) {
            stopSetpointPublisher();
            ROS_WARN("[MissionManager] Stopped setpoint publisher before processing new task_flow");
        }

        // 注:guidance 残留清理 / current_target_ 清锁 放到 preserve_track 判定之后,
        //     Attack(105) 协同场景需要保留并切 strike 模式,见下方分支。

        ROS_WARN("[MissionManager] >>>> Received TaskFlow id=%s skills=%lu device_id=%u",
                 msg->flow_id.c_str(), msg->skills.size(), msg->device_id);

        latest_task_flow_ = msg;
        current_flow_id_  = msg->flow_id;
        has_task_flow_    = true;

        // === 新 TaskFlow 到达:清空 cluster dedup 状态 + 锁定目标,允许新一轮上报/锁定 ===
        //   - reported_to_gs_ / strike_dispatched_ 清空:之前 task_flow 上报/派发过的
        //     cluster_id 在新 flow 中可重新上报/派发(实际 cluster 已 TTL 或飞离,新 flow
        //     重新看到也大概率是不同目标,但语义上保持独立)
        //   - ignored_targets_ 清空:新 task_flow 给个干净黑名单起点,避免上一个 flow 的
        //     "忽略/暂存" 状态泄漏到新 flow;同一个黑名单项(物理目标)在 flow 之间可能已经
        //     不再相关(cluster_estimator 重启或换搜索区)
        //   - locked_cluster_id_ = 0:cluster_states 缓存失效
        //   - tracked_target_ 的清空见下方 preserve_track 分支(Attack(105)首发时不清)
        reported_to_gs_.clear();
        strike_dispatched_.clear();
        ignored_targets_.clear();
        locked_cluster_id_     = 0;
        locked_cluster_ned_x_  = 0.0;
        locked_cluster_ned_y_  = 0.0;
        locked_cluster_ned_alt_ = 0.0;
        locked_cluster_time_   = ros::Time();
        // 多目标: 新 flow 到达 → 清主目标,estimator 回退最高置信度
        publishPrimaryTarget(0);

        // 重置航点接收标志(避免前序 flow 的 is_waypoints_received_ 残留误导 performTakeoffHandoff)
        is_waypoints_received_ = false;

        // === 新 TaskFlow 到达:云台俯仰角复位到默认值(2026-07-29) ===
        //   上一条 flow 的 102 Search 可能在 IN_TASK 设了 non-default gimbal_pitch,
        //   新 flow 到达后这条 skill 流会被替换,如果不显式复位,timer 会继续用旧值发布,
        //   直到下一条 102 IN_TASK 才覆盖(若新 flow 没有 102 则永远不复位)。
        //   兜底语义:任何新 flow 到达 = "上一段搜索结束" → gimbal 归位。
        if (current_gimbal_pitch_ != gimbal_pitch_default_) {
            ROS_WARN("[MissionManager] New TaskFlow: gimbal_pitch %.2f -> default %.2f deg",
                     current_gimbal_pitch_, gimbal_pitch_default_);
            current_gimbal_pitch_ = gimbal_pitch_default_;
        }

        // === 新 TaskFlow:重置 Attack(105) WP2-gate + 云台状态(2026-07-30) ===
        attack_wp2_target_seen_      = false;
        attack_wp1_reached_          = false;
        attack_wp1_reached_time_     = ros::Time();
        attack_strike_phase_active_  = false;

        // === 2026-08-05: 新 TaskFlow 到达 → 立即 stop waypoint_executor ===
        //   背景:之前在 SEARCH_TRACK (skill 102, phase=4) 时 waypoint_executor 还在发
        //   setpoint_velocity,如果此时收到 attack-targets (skill 105),skill_queue 被清空后
        //   waypoint_executor 还在过渡态 (旧 SKILL_AREA → PENDING) 会继续以旧路径发速度。
        //   guidance_control 同时也发(STRIKE 模式接管后),两者交替穿插 → PX4 收到混乱指令。
        //   修复:taskFlowCallback 一进来,无论新 flow 是什么 skill,先 stop waypoint_executor
        //   让它立刻 is_executing_=false,然后再清空 skill_queue 推新 skill,避免 1s 重叠窗口。
        //   waypoint_executor.controlCallback 收到 "stop" 后调 stop() → publishZeroVelocity,
        //   再下次 50Hz tick 起不再发 setpoint,只保留 status/current_target 上报。
        {
            std_msgs::String stop_cmd;
            stop_cmd.data = "stop";
            waypoint_control_pub_.publish(stop_cmd);
        }

        // 把 skill 流拷进运行时队列
        skill_queue_.clear();
        skill_queue_.reserve(msg->skills.size());
        // strip_leading_takeoff(已空中收到 [Takeoff, ...]):跳过 index 0 的 takeoff skill
        size_t skill_begin_idx = strip_leading_takeoff ? 1u : 0u;
        for (size_t si = skill_begin_idx; si < msg->skills.size(); ++si) {
            const auto& s = msg->skills[si];
            SkillRuntime sr;
            sr.msg = s;
            sr.state = SkillState::PENDING;
            sr.state_enter_time = ros::Time::now();
            sr.gather_enter_time = ros::Time();
            sr.gather_local_arrived_time = ros::Time();
            sr.last_event = "queued";
            // === Attack(105) 多机协同:从 params_json 解 devices_sn 作协同伙伴名单 ===
            //   - formation 字段(skill.formation):0=默认/1=惯序/2=协同
            //   - 协议文档(skill_type=105 时 formation 语义) 见 Skill.msg + 协议补充规范
            //   - partners 列表 fallback 到 launch 私有 param ~expected_sns(空时退化为单机)
            if (s.skill_type == 105) {
                sr.attack_phase_ = SkillRuntime::AttackPhase::IDLE;
                auto full_partners = parseDevicesSnFromParamsJson(s.params_json);
                if (full_partners.empty()) {
                    full_partners = expected_sns_;  // fallback
                }
                // 找自己在 full 中的位置(formation=1 sequential 用,必须在 erase 前算)
                // self_device_id_ 是 int,partner 是 string:用 std::to_string 转换
                const std::string my_sn = std::to_string(self_device_id_);
                auto it = std::find(full_partners.begin(),
                                    full_partners.end(), my_sn);
                sr.attack_seq_index_ = (it != full_partners.end())
                    ? static_cast<int>(it - full_partners.begin()) : -1;
                // formation=1 sequential:缓存上一架 partner 的 sn
                if (sr.attack_seq_index_ > 0 &&
                    sr.attack_seq_index_ - 1 < static_cast<int>(full_partners.size())) {
                    sr.attack_seq_prev_sn_ =
                        full_partners[sr.attack_seq_index_ - 1];
                }
                // 剔除自己(避免把自己当协同伙伴)
                sr.attack_partners_ = full_partners;
                sr.attack_partners_.erase(
                    std::remove_if(sr.attack_partners_.begin(), sr.attack_partners_.end(),
                        [&](const std::string& sn){ return sn == my_sn; }),
                    sr.attack_partners_.end());
                ROS_INFO("[MissionManager] Attack skill '%s' mode=%u partners=%zu my_idx=%d prev=%s",
                         s.skill_id.c_str(),
                         static_cast<unsigned>(s.formation),
                         sr.attack_partners_.size(),
                         sr.attack_seq_index_,
                         sr.attack_seq_prev_sn_.c_str());
            }

            // === 集结合(skill_type=101):从 params_json 解 devices_sn 作多机同步名单 ===
            //   1) 任务流带 "devices_sn":[...] → 用它(GS 真实协议下发)
            //   2) 没带 → fallback 到 launch 私有 param ~expected_sns(老路径)
            //   3) 都没 → gather_partners_ 留空,IN_TASK 走 single_uav_arrived
            if (s.skill_type == 101) {
                sr.gather_partners_ = parseDevicesSnFromParamsJson(s.params_json);
                if (sr.gather_partners_.empty()) {
                    sr.gather_partners_ = expected_sns_;  // fallback
                }
                if (!sr.gather_partners_.empty()) {
                    ROS_INFO("[MissionManager] Gather skill '%s' partners (%zu): %s",
                             s.skill_id.c_str(), sr.gather_partners_.size(),
                             [&]() {
                                 std::string s_joined;
                                 for (size_t i = 0; i < sr.gather_partners_.size(); ++i) {
                                     if (i) s_joined += ",";
                                     s_joined += sr.gather_partners_[i];
                                 }
                                 return s_joined;
                             }().c_str());
                }
            }
            skill_queue_.push_back(sr);
        }
        current_skill_index_ = 0;

        // === Skill flow 适配:TaskFlow 含非空 arrive_path/skill_area_path 的 skill 时,
        //     视为"航点已下发",让 runPx4TakeoffSequence 在 HOVERING 时能触发 performTakeoffHandoff()
        //     否则 takeoff 永远卡在 HOVERING 等 legacy waypointCallback
        bool has_path_skills = false;
        for (const auto& s : msg->skills) {
            if (!s.arrive_path.poses.empty() || !s.skill_area_path.poses.empty()) {
                has_path_skills = true;
                break;
            }
        }
        if (has_path_skills) {
            is_waypoints_received_ = true;
            current_waypoint_count_ = 1;  // 占位,让 performTakeoffHandoff 不报 0
        }

        // 第一个 skill 是 Takeoff(type=100 或 106)时:推进 phase=PHASE_TAKING_OFF
        // 注意:不再操作 work_mode(只表达"在任务区内做什么",起飞是 phase 不是 work_mode)
        // 协议 TODO(skill_type_mapping): 100=弹射起飞,106=地面起飞
        if (!skill_queue_.empty() && (skill_queue_[0].msg.skill_type == 100 || skill_queue_[0].msg.skill_type == 106)) {
            // === 弹射起飞判定 (skill_type=100) ===
            // 设置 is_catapult_takeoff_,并在每个新 task_flow 开始时清掉旧的 trigger 状态
            // 这样:新一发 task_flow 是弹射则重新等 trigger;不是则彻底忽略外部 trigger。
            bool new_is_catapult = (skill_queue_[0].msg.skill_type == 100);
            if (new_is_catapult != is_catapult_takeoff_) {
                ROS_WARN("[MissionManager] Takeoff type changed: %s -> %s "
                         "(resetting catapult trigger state)",
                         is_catapult_takeoff_ ? "CATAPULT(100)" : "GROUND(106)",
                         new_is_catapult ? "CATAPULT(100)" : "GROUND(106)");
            }
            is_catapult_takeoff_       = new_is_catapult;
            catapult_trigger_received_ = false;   // 每个新 task_flow 重新等 trigger
            // is_px4_catapult_mode_ 保持:comm_node 切的模式有持续性,不应每次 flow 重置
            //   (万一 comm_node 没发 false 关闭信号,这里避免误清)
            catapult_posctl_enter_time_ = ros::Time();  // isZero() 表示还没进入 POSCTL

            if (current_phase_ != MissionPhase::PHASE_TAKING_OFF) {
                // 缓存起飞前的 phase 和 work_mode,performTakeoffHandoff 用
                phase_before_takeoff_ = current_phase_;
                work_mode_before_takeoff_ = current_work_mode_;
                // === 2026-07-28 fix: 守卫从 COMPLETE/FAILED 二选一扩到所有非 IDLE 状态 ===
                //   背景:前序 takeoff 在 HOVERING(等航点超时)、SETTING_OFFBOARD(setMode 死循环)等中间
                //   状态卡住时,后续 taskFlowCallback 会进 L1820 takeoff 分支但守卫不命中,takeoff_state_
                //   卡在中间状态 → runPx4TakeoffSequence switch 直接跳进 HOVERING 等"已完成"分支,
                //   永远不走 triggerOffboardAndArm,PX4 卡在 AUTO.RTL/UAV 一直 disarm。
                //   典型观察:mission_manager 报 "HOVERING waiting for waypoints" 但 PX4 还在 AUTO.RTL,
                //   mission_state.phase=TAKING_OFF 但 takeoff_state_ 已是 HOVERING,被外层判定为"已起飞"。
                if (takeoff_state_ != TakeoffState::TAKEOFF_IDLE) {
                    ROS_WARN("[MissionManager] Re-takeoff detected: resetting takeoff_state_ "
                             "(and retry/failed flags) from current to IDLE");
                    takeoff_state_        = TakeoffState::TAKEOFF_IDLE;
                    // 同步清掉上一次起飞尝试的可重入标志 ——
                    // 修了 HOVERING 之后,"FLOW → HOVERING 卡死 → disarm → FLOW" 这条路能走通了,
                    // 这些字段必须一起清,否则 retry_count 残留会让第二次起飞首次失败就撞 max_retries。
                    takeoff_retry_count_   = 0;
                    takeoff_failed_logged_ = false;
                    takeoff_failed_time_   = ros::Time();
                }
                current_phase_ = MissionPhase::PHASE_TAKING_OFF;
                ROS_WARN("[MissionManager] TaskFlow Takeoff skill to phase=TAKING_OFF "
                         "(cached prev_phase=%s, prev_work_mode=%s)",
                         missionPhaseToString(phase_before_takeoff_).c_str(),
                         workModeToString(work_mode_before_takeoff_).c_str());
            }
            // === 协议层接管:takeoff_altitude 由 Skill.msg 提供(GS/KCP 端下发)
            //     非 0 表示业务侧给了具体高度,覆盖 mission_manager 私有 param 兜底
            //     skill_type != 100 时该字段忽略(对照 Skill.msg 注释)
            const auto& tk = skill_queue_[0].msg;
            if (tk.takeoff_altitude > 0.0f) {
                takeoff_altitude_ = static_cast<double>(tk.takeoff_altitude);
                ROS_WARN("[MissionManager] Takeoff altitude from Skill msg: %.2f m (relative to home)",
                         takeoff_altitude_);
            } else {
                ROS_WARN("[MissionManager] Takeoff altitude missing in Skill msg, "
                         "fallback to private param ~takeoff_altitude = %.2f m",
                         takeoff_altitude_);
            }
        }

        // 立刻把第一个 skill 推到 waypoint_executor (takeoff 守卫在 pushCurrentSkillToExecutor 内部)
        if (!skill_queue_.empty()) {
            pushCurrentSkillToExecutor();
        }

        // === 协同打击特例:TaskFlow 首发 skill 是 Attack(105)时,
        //     必须保留当前 tracked_target_ —— 不管是 broadcast(GS 协同全队)
        //     还是精确点名本机(device_id == self_device_id_),
        //     都意味着 GS 期望本机用**当前已锁目标**执行这次打击,
        //     清空会让跟踪逻辑重启,目标丢失。
        //     其余情况(非 Attack 首发 / takeoff 等)按原逻辑清空旧目标。
        bool preserve_track = false;
        if (!skill_queue_.empty() && skill_queue_[0].msg.skill_type == 105) {
            preserve_track = true;
        }

        if (!preserve_track) {
            tracked_target_.is_valid = false;
            setLockState(TrackLockState::NOT_LOCKED);
        } else {
            ROS_WARN("[MissionManager] TaskFlow first skill is Attack(105) "
                     "(device_id=%u). Preserving current track lock.",
                     msg->device_id);
            // current_target_ 不在这里同步 —— Attack skill 推到 waypoint_executor 后
            // 自身会带目标坐标下来覆盖,避免此处重复写造成姿态不一致
        }

        // === 新 TaskFlow 接管:guidance 状态分两条路径处理 ===
        // 路径 A (非 Attack 首发): disableGuidance + 清 current_target_ 旧锁
        //   背景:startGuidanceApproach() publish enable=true 后,老 taskFlow 走了但没显式关,
        //         guidance_control_node 仍按旧 current_target_ 生成 setpoint,PX4 OFFBOARD 下
        //         飞机会按旧目标飞直到新 taskFlow 自己再触发 startGuidanceApproach()(几 s gap)。
        //   处理:收到新 taskFlow 后第一时间关 guidance,保证新任务从干净状态开始。
        // 路径 B (Attack 首发协同): 保留 guidance,显式切 mode = "strike"
        //   背景:典型场景 —— 另一架机在跟踪目标,GS 看准时机广播 Attack(105) 协同打击。
        //         期望语义:tracked_target_ 保留,guidance 继续推,但 mode 从 "track" → "strike"
        //         让 guidance_control_node 切到攻击路径(弹道前置等)。
        //   处理:不 disable,直接 publish enable=true + mode="strike" 覆盖之前的 track 模式。
        //         这里强制覆盖,不依赖 GS 是否同时下发 WorkMode=SEARCH_STRIKE(协议上 work_mode
        //         与 task_flow 解耦,GS 可能不发 work_mode,但语义上必须切到 strike 路径)。
        if (!preserve_track) {
            if (is_guidance_active_) {
                disableGuidance();
                ROS_WARN("[MissionManager] Disabled residual guidance from previous task_flow");
            }
            // 清 current_target_ 旧锁:
            //   - guidance_control_node 缓存的旧值不应污染新 taskFlow
            //   - triggerStrike()/checkYoloDrivenStrike() 入口(current_target_.is_locked)
            //     需回到 NOT_LOCKED,否则新 taskFlow 一开始可能被误判"已有目标",跳过 SEARCH 阶段
            current_target_.is_locked   = false;
            current_target_.is_detected = false;
            current_target_.is_shared   = false;
            // current_target_.pose 不强制清零:留给新一轮 YOLO/cluster 覆盖
        } else {
            // Attack 协同:显式把 guidance 切到 strike 模式
            // 注意:不调 disableGuidance(),避免破坏正在跟踪的目标
            //       不动 tracked_target_ / current_target_,目标锁原样保留
            if (is_guidance_active_) {
                std_msgs::Bool enable;
                enable.data = true;
                guidance_enable_pub_.publish(enable);

                std_msgs::String mode_msg;
                mode_msg.data = "strike";  // 强制覆盖 track,即使 current_work_mode_ 还是 SEARCH_TRACK
                guidance_mode_pub_.publish(mode_msg);

                ROS_WARN("[MissionManager] Attack(105) cooperative: guidance mode -> 'strike' "
                         "(preserved lock on current target)");
            } else {
                // guidance 之前没启(罕见),Attack 协同要求必须有 active guidance
                // 这种情况下 GS 期望"用当前已锁目标"但 guidance 没在跑,留个 warn
                ROS_WARN("[MissionManager] Attack(105) cooperative but guidance not active — "
                         "no track to convert to strike. GS may need to re-send Track first.");
            }
        }

        // 兜底清空:此处 setLockState 早 return (因新 flow 总是 NOT_LOCKED→NOT_LOCKED,
        // 或前面 preserve_track=true 时 lock_state 保持; 总之上面已处理,这里用 setLockState
        // 兜底以保持所有清空路径统一)。
        setLockState(TrackLockState::NOT_LOCKED);
    }

    /**
     * 把当前 skill_queue_[current_skill_index_] 的 Skill 推到 waypoint_executor
     * 并把 state 从 PENDING → TRANSIT
     * 空 queue 时不做事
     */
    void pushCurrentSkillToExecutor() {
        if (current_skill_index_ >= skill_queue_.size()) {
            return;
        }
        auto& sr = skill_queue_[current_skill_index_];
        // === 守卫:takeoff skill 不推给 executor ===
        // 起飞走 phase 通道 (runPx4TakeoffSequence() + performTakeoffHandoff()),
        //   不归 executor 管。如果把 arrive_path 推给 executor,它会立即开始飞,
        //   而 takeoff 还没爬升到目标高度,两套控制打架(handoff 强制 COMPLETE 也晚了一步)。
        //   即使 arrive_path 为空也不推 — 保持"takeoff 不下发"的语义一致,让
        //   后续 search/gather 真正要飞的航点从下一条 skill 开始下发。
        if (sr.msg.skill_type == 100 || sr.msg.skill_type == 106) {
            return;
        }
        // === Attack(105) formation=2 协同:实时规划 attack_point + strike_sync_time ===
        //   仅 attack_partners_ 非空时执行(空 → 退化为 formation=0 单机);
        //   若 partner 心跳未收齐则稍后重试(本 tick 不推 executor)。
        if (sr.msg.skill_type == 105 && sr.msg.formation == 2) {
            if (!planCoordinatedAttack(sr)) {
                ROS_INFO_THROTTLE(2.0,
                    "[MissionManager] Co-op plan: waiting for partner positions...");
                return;  // 等下次 tick
            }
        }

        // === CODEX DIRECT-LOOK START ===
        // Attack(105) 首次下发前先原地直视 WP2。这里故意不 publish waypoint_skill：
        // 预检成功由 guidance 直接接管；预检超时后 direct_look_attempted_=true，
        // 下一次进入本函数会继续执行下方原有 publish，完整保留 WP1→WP2 逻辑。
        // 已有有效跟踪/制导时保留原 cooperative Attack 接管语义，不能起 position hold
        // 与正在发布的 guidance 速度抢控制；直视预检只服务于“尚未锁定目标”的 Attack。
        if (sr.msg.skill_type == 105 && attack_direct_look_enabled_ &&
            !is_guidance_active_ && !tracked_target_.is_valid &&
            !sr.direct_look_attempted_) {
            sr.direct_look_attempted_ = true;
            if (!is_pose_received_ || !is_gps_received_ ||
                sr.msg.skill_area_path.poses.empty()) {
                ROS_WARN("[MissionManager] Attack direct-look skipped: pose=%s gps=%s WP2=%s; "
                         "falling back to original WP1->WP2 path",
                         is_pose_received_ ? "ready" : "missing",
                         is_gps_received_ ? "ready" : "missing",
                         sr.msg.skill_area_path.poses.empty() ? "missing" : "ready");
            } else {
                startSetpointPublisher();
                // startSetpointPublisher 默认 z=0.5；预检必须覆盖为当前高度，做到真正原地保持。
                takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // NED y -> ENU east
                takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // NED x -> ENU north
                takeoff_setpoint_.pose.position.z = -current_pose_.pose.position.z; // NED down -> ENU up

                // 云台没有 yaw 能力：用机头 yaw 指向 WP2。GPS 差得到 NED bearing，
                // 再转换成 MAVROS position setpoint 使用的 ENU yaw。
                const auto& wp2 = sr.msg.skill_area_path.poses.back();
                const double north = gpsDeltaN(current_gps_.latitude, wp2.pose.position.x);
                const double east  = gpsDeltaE(current_gps_.longitude, wp2.pose.position.y,
                                               current_gps_.latitude);
                const double yaw_ned = std::atan2(east, north);
                const double yaw_enu = M_PI_2 - yaw_ned;
                const double half = 0.5 * yaw_enu;
                takeoff_setpoint_.pose.orientation.x = 0.0;
                takeoff_setpoint_.pose.orientation.y = 0.0;
                takeoff_setpoint_.pose.orientation.z = std::sin(half);
                takeoff_setpoint_.pose.orientation.w = std::cos(half);

                sr.direct_look_active_ = true;
                sr.direct_look_start_time_ = ros::Time::now();
                sr.direct_look_cluster_id_ = 0;
                sr.direct_look_last_seen_us_ = 0;
                sr.direct_look_consecutive_hits_ = 0;
                sr.state = SkillState::TRANSIT;
                sr.state_enter_time = sr.direct_look_start_time_;
                sr.last_event = "TRANSIT: Attack direct-look precheck";
                current_work_mode_ = WorkMode::SEARCH_STRIKE;
                attack_hover_setpoint_latched_ = true;
                ROS_WARN("[MissionManager] Attack(105) DIRECT_LOOK started: hold current position, "
                         "nose+gimbal -> WP2, settle=%.1fs timeout=%.1fs confirm=%d frames",
                         attack_direct_look_settle_sec_, attack_direct_look_timeout_sec_,
                         attack_direct_look_confirm_hits_);
                return;
            }
        }
        // === CODEX DIRECT-LOOK END ===
        // 推到 executor。若上一条 gather 正在 HOLDING,先发布新 skill 让 executor
        // 建好速度 setpoint 源,再停 mission_manager 的 position-hold,避免 OFFBOARD 真空。
        const bool handoff_from_holding =
            current_phase_ == MissionPhase::PHASE_HOLDING;
        waypoint_skill_pub_.publish(sr.msg);
        if (handoff_from_holding) {
            current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
            if (setpoint_running_) {
                stopSetpointPublisher();
            }
            ROS_WARN("[MissionManager] HOLDING to WAYPOINT_FOLLOW: next skill handed to executor");
        }

        // 状态推进 PENDING → TRANSIT
        if (sr.state == SkillState::PENDING) {
            skill_queue_[current_skill_index_].state = SkillState::TRANSIT;
            skill_queue_[current_skill_index_].state_enter_time = ros::Time::now();
            skill_queue_[current_skill_index_].last_event = "TRANSIT: pushed to executor";
            ROS_WARN("[MissionManager] Skill[%zu] id=%s type=%u to TRANSIT",
                     current_skill_index_, sr.msg.skill_id.c_str(), sr.msg.skill_type);
        }
    }

    /**
     * Attack(105) formation=2 实时空间规划:
     *   - 收集各 UAV(含自己)当前位置
     *   - 给本机分配 attack_point:沿自己朝目标方位**就近**选,
     *     距目标 d_uav - safety_margin(不越过目标)
     *   - 计算初始 strike_sync_time = now + max(飞行时间) + buffer
     *   - 改写 skill_area_path.WP2 = 本机 attack_point
     *   - 在 params_json 加 strike_sync_time_us(初始值,partner 到达后 recomputeCoordinatedStrikeSync 重算)
     *
     *  返回 true 表示规划成功(skill_area_path + params_json 已就地改写);
     *  返回 false 表示 partner 心跳未收齐,稍后重试。
     */
    bool planCoordinatedAttack(SkillRuntime& sr) {
        if (sr.attack_partners_.empty()) {
            // 没人协同 → 退化为 formation=0 单机,waypoint_executor 收到原样 skill
            return true;
        }
        struct UavPos { std::string sn; double lat, lon, alt; };
        std::vector<UavPos> positions;
        // 本机(self_device_id_ 是 int,转 string)
        positions.push_back({std::to_string(self_device_id_),
                             current_gps_.latitude, current_gps_.longitude, current_gps_.altitude});
        // partners(从 heartbeats_)
        for (const auto& sn : sr.attack_partners_) {
            auto hb = findHeartbeat(sn);
            if (!hb || hb->lat == 0.0 || hb->lon == 0.0) {
                return false;  // 心跳未收齐,稍后重试
            }
            positions.push_back({sn, hb->lat, hb->lon, hb->alt});
        }
        // 目标位置 = GS WP2(skill_area_path 末点)
        if (sr.msg.skill_area_path.poses.empty()) {
            ROS_ERROR("[MissionManager] Co-op: Attack skill_area_path empty");
            return true;  // 退化,让上层处理
        }
        const auto& wp2 = sr.msg.skill_area_path.poses.back();
        const double tgt_lat = wp2.pose.position.x;
        const double tgt_lon = wp2.pose.position.y;
        const double tgt_alt = wp2.pose.position.z;  // MSL,可能=0(地面)
        // 每架 UAV 在自己 bearing 上就近选 attack_point
        double max_flight_time = 0.0;
        const double speed = std::max(static_cast<double>(sr.msg.task_speed), 1.0);
        for (const auto& p : positions) {
            // UAV 朝目标方位(简化为局部切平面)
            double dx_n = gpsDeltaN(p.lat, tgt_lat);  // 北向分量
            double dx_e = gpsDeltaE(p.lon, tgt_lon, p.lat);  // 东向分量
            double d = std::hypot(dx_n, dx_e);
            double bearing = std::atan2(dx_e, dx_n);  // UAV 朝目标方位
            // 计算 attack_point 距目标距离(就近、不越过)
            double ap_dist = std::max(d - attack_safety_margin_m_, 0.0);
            // 若 UAV 已近(d < close_range_threshold,默认 5m),attack_point = 当前位置
            if (d < attack_close_range_threshold_m_) {
                ap_dist = 0.0;
            }
            // 只为本机算精确 attack_point,partner 各自算
            if (p.sn == std::to_string(self_device_id_)) {
                if (ap_dist == 0.0) {
                    sr.attack_point_lat_ = p.lat;
                    sr.attack_point_lon_ = p.lon;
                } else {
                    // 沿 bearing 反方向走 ap_dist(远离目标 = bearing + pi)
                    //   1° lat ≈ EARTH_R * π/180 米 → meters-to-deg-lat = meters / (EARTH_R*π/180)
                    constexpr double DEG_LAT_PER_M = 180.0 / (6378137.0 * M_PI);
                    constexpr double COS_TGT_LAT = 1.0;  // 简化: tgt_lat 较小时 cos≈1
                    (void)COS_TGT_LAT;
                    sr.attack_point_lat_ = tgt_lat - ap_dist * std::cos(bearing) * DEG_LAT_PER_M;
                    sr.attack_point_lon_ = tgt_lon -
                        ap_dist * std::sin(bearing) * DEG_LAT_PER_M /
                        std::cos(tgt_lat * M_PI / 180.0);
                }
                sr.attack_point_alt_ = p.alt;  // 保持本机高度,避免不必要的爬升
                // (后续 waypoint_executor 会按 attack_floor_altitude_m 再做 WP2 地板 clamp)
            }
            // 飞行时间预估
            double flight_time = d / speed;
            if (flight_time > max_flight_time) max_flight_time = flight_time;
        }
        // 初始 strike_sync_time 预估(partner 到达后会通过 recomputeCoordinatedStrikeSync 重算)
        sr.attack_strike_sync_time_ = ros::Time::now() +
            ros::Duration(max_flight_time + attack_sync_strike_buffer_sec_);
        // 改写 skill_area_path.WP2 = 本机 attack_point
        sr.msg.skill_area_path.poses.back().pose.position.x = sr.attack_point_lat_;
        sr.msg.skill_area_path.poses.back().pose.position.y = sr.attack_point_lon_;
        sr.msg.skill_area_path.poses.back().pose.position.z = sr.attack_point_alt_;
        // 改写 params_json,加 strike_sync_time_us
        char buf[256];
        std::snprintf(buf, sizeof(buf), ",\"strike_sync_time_us\":%ld",
                      static_cast<long>(sr.attack_strike_sync_time_.toNSec() / 1000));
        sr.msg.params_json += buf;
        ROS_WARN("[MissionManager] Co-op plan: target=(%.6f,%.6f) "
                 "my_attack=(%.6f,%.6f) strike_sync=%.3f max_flight=%.2fs",
                 tgt_lat, tgt_lon,
                 sr.attack_point_lat_, sr.attack_point_lon_,
                 sr.attack_strike_sync_time_.toSec(), max_flight_time);
        return true;
    }

    /** 简化 GPS 球面近似:lat 差 → 北向米 */
    static double gpsDeltaN(double lat_from, double lat_to) {
        static constexpr double EARTH_R = 6378137.0;
        return (lat_to - lat_from) * M_PI / 180.0 * EARTH_R;
    }
    /** 简化 GPS 球面近似:lon 差 → 东向米 */
    static double gpsDeltaE(double lon_from, double lon_to, double lat_ref) {
        static constexpr double EARTH_R = 6378137.0;
        return (lon_to - lon_from) * M_PI / 180.0 * EARTH_R *
               std::cos(lat_ref * M_PI / 180.0);
    }

    /**
     * 给定 UAV 与目标 GPS(度/正高),返回云台俯仰角(度,正值=下视,GS 语义)。
     * 用于 gimbal_command 下发:mission_manager 解算 → gimbal_simulator 纯跟随。
     * horiz < 1m 时返回 90°(正下方极限),不会除零。
     */
    static double computePitchDegToLLA(double uav_lat, double uav_lon, double uav_alt,
                                       double tgt_lat, double tgt_lon, double tgt_alt) {
        const double horiz = std::hypot(gpsDeltaN(uav_lat, tgt_lat),
                                        gpsDeltaE(uav_lon, tgt_lon, uav_lat));
        const double drop  = uav_alt - tgt_alt;
        if (horiz < 1.0) return 90.0;
        return std::atan2(drop, horiz) * 180.0 / M_PI;
    }

    /** 朝 WP2(skill_area_path.poses.back())的俯仰角,前提:UAV GPS 已收到 + skill_area 非空 */
    double computePitchDegToWp2(const SkillRuntime& sr) const {
        if (sr.msg.skill_area_path.poses.empty() || !is_gps_received_) {
            return attack_gimbal_look_default_pitch_deg_;
        }
        const auto& wp2 = sr.msg.skill_area_path.poses.back();
        return computePitchDegToLLA(
            current_gps_.latitude, current_gps_.longitude, current_gps_.altitude,
            wp2.pose.position.x,    wp2.pose.position.y,    wp2.pose.position.z);
    }

    /**
     * 朝 primary cluster 的俯仰角:用 home GPS 做 NED 原点,把
     *   current_target_.pose(NWU,相对 home) 反推成 lat/lon/alt,再算 pitch。
     * 前提:home 已锁定 + target_estimated_pose 收到至少一帧。
     * home 与 target_estimator 共享同一 /mavros/home_position/home,所以 NED 原点一致。
     *
     * 字段约定(与 target_estimator_node.cpp::publishTargetGps 对齐):
     *   p.x = NWU.x = 北向 (相对 home)
     *   p.y = NWU.y = 西向 (相对 home, 正 = 西)
     *   p.z = NWU.z = 上向 (相对 home altitude)
     *   此前注释写"NED (x=东向, y=北向, z=下向)"是错的,p.x/p.y/p.z 全用反。
     */
    double computePitchDegToPrimaryCluster() const {
        if (!ref_initialized_ || !is_pose_received_) {
            return attack_gimbal_look_default_pitch_deg_;
        }
        const auto& p = current_target_.pose.pose.position;
        // NWU → LLA(正高):北→lat, 西→-East→lon(经度东向), 上→alt(加上 home)
        const double cluster_alt = ref_alt_ + p.z;                      // p.z = Up
        const double cluster_lat = ref_lat_ + p.x / 111000.0;           // p.x = North
        const double cluster_lon = ref_lon_ -
            p.y / (111000.0 * std::cos(ref_lat_ * M_PI / 180.0));      // p.y = West,lon -= west_m
        return computePitchDegToLLA(
            current_gps_.latitude, current_gps_.longitude, current_gps_.altitude,
            cluster_lat, cluster_lon, cluster_alt);
    }

    /**
     * cluster 到 WP2 的水平距离(米):用 PX4 home 作统一 NED 原点,
     *   把 WP2 lat/lon 转 NED,再与 cluster (ned_x, ned_y) 求 Euclidean 距离。
     * 仅用于 Attack(105) WP2-gate。
     * home 未锁定 → +inf(永远不通过 gate,直到 home 稳定)。
     *
     * 字段约定(与 target_estimator_node.cpp::publishTargetGps 对齐):
     *   cluster.ned_x     = NWU.x = 北向 (相对 home, 正 = 北)
     *   cluster.ned_y     = NWU.y = 西向 (相对 home, 正 = 西,即 -East)
     *   cluster.ned_alt   = NWU.z = 上向 (相对 home altitude)
     *   此前注释写成"ned_x=东向"是错的,导致 d_north/d_east 字段用反,
     *   cluster 真实位置在 WP2 附近时反而算出 200m+,WP2-gate (30m) 永远拒。
     */
    double clusterDistToWp2Meters(const multi_uav_strike::ClusterTarget& cluster,
                                 const SkillRuntime& sr) const {
        if (!ref_initialized_ || sr.msg.skill_area_path.poses.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        const auto& wp2 = sr.msg.skill_area_path.poses.back();
        const double wp2_north_m = gpsDeltaN(ref_lat_, wp2.pose.position.x);
        const double wp2_east_m  = gpsDeltaE(ref_lon_, wp2.pose.position.y, ref_lat_);
        const double d_north = cluster.ned_x - wp2_north_m;     // c.ned_x = North
        const double d_east  = (-cluster.ned_y) - wp2_east_m;   // c.ned_y = West,East = -West
        return std::hypot(d_north, d_east);
    }
    /** Heartbeat 缓存查找(由 sn 找最近一条 InterUavStatus)
     *  注:缓存本身是 partner_heartbeats_(std::map<sn, PartnerHeartbeat>),
     *    PartnerHeartbeat.latest 是 InterUavStatus::ConstPtr。
     */
    multi_uav_strike::InterUavStatus::ConstPtr findHeartbeat(
        const std::string& sn) const {
        auto it = partner_heartbeats_.find(sn);
        if (it == partner_heartbeats_.end()) return nullptr;
        return it->second.latest;
    }

    /**
     * Attack(105) strike gating — 用于 clusterStatesCallback / 后续重试
     *   READY   = 可以 triggerStrike
     *   WAITING = 还不行(sequential 等前一架 / coordinated 等同步时间)
     *
     * formation=0:无门控,直接 READY(单机 Attack 与老 SEARCH_STRIKE 路径一致)
     * formation=1 sequential:
     *   - attack_seq_index_ == 0 → 第一架,直接 READY
     *   - 否则等 attack_seq_prev_sn_ 的最近心跳 phase 进 GUIDANCE_STRIKE(=5)
     *   - 心跳未到 / phase 未到 → WAITING
     * formation=2 coordinated:
     *   - attack_strike_sync_time_ 未规划 → WAITING
     *   - 当前时间 < sync 时间 → WAITING
     *   - 否则 READY
     *
     * 注:此函数**不再**额外检查 UAV 是否到达 attack_point。Strike 允许在 skill_area_path
     *   内任何时刻 fire(由 clusterStatesCallback 顶部的 skill_idx>=1 门控保证"至少到过 WP1"):
     *   - WP1 刚踩到(idx=0→1):executor 立即转向 WP2,任何 cluster 命中都会 fire
     *   - WP1 → WP2 飞行段:cluster 命中 fire
     *   - WP2 踩到(此前的版本会把这里 block 住):仍 fire,旧 hover 兜底只在 strike 一直未
     *     fire(全程没看到目标)时由 attack_hover_timeout_sec_ 兜底退出,见 IN_TASK case。
     *   之前的 attack_hover_enter_time_ 检查已于 2026-07-26 用户口径变更后移除。
     */
    enum class AttackStrikeGating { READY, WAITING };
    AttackStrikeGating checkAttackStrikeGating(const SkillRuntime& sr) {
        if (sr.msg.skill_type != 105) {
            return AttackStrikeGating::READY;  // 非 Attack 不参与门控
        }
        const uint8_t mode = sr.msg.formation;
        if (mode == 0) {
            return AttackStrikeGating::READY;  // 默认 / 单机
        }
        if (mode == 1) {
            // sequential:等前一架 partner 进 GUIDANCE_STRIKE
            if (sr.attack_seq_index_ <= 0) {
                return AttackStrikeGating::READY;  // 第一架或未列入
            }
            if (sr.attack_seq_prev_sn_.empty()) {
                return AttackStrikeGating::READY;  // 无 prev 名单,放行(异常 fallback)
            }
            auto prev_hb = findHeartbeat(sr.attack_seq_prev_sn_);
            if (!prev_hb) {
                return AttackStrikeGating::WAITING;  // 心跳未到
            }
            // PHASE_GUIDANCE_STRIKE = 5(见 InterUavStatus.msg 注释)
            if (prev_hb->phase == 5) {
                return AttackStrikeGating::READY;
            }
            return AttackStrikeGating::WAITING;
        }
        if (mode == 2) {
            // coordinated:等到 strike_sync_time_
            if (sr.attack_strike_sync_time_.isZero()) {
                return AttackStrikeGating::WAITING;
            }
            if (ros::Time::now() >= sr.attack_strike_sync_time_) {
                return AttackStrikeGating::READY;
            }
            return AttackStrikeGating::WAITING;
        }
        // 未知 formation 模式:放行(防御性)
        return AttackStrikeGating::READY;
    }

    /**
     * label 是否匹配 GS 下发的 target_type 位掩码(2026-07-30 新增,2026-07-30 车类合一)
     *
     * 语义:
     *   target_type == 0                  → 任意 label 匹配(保持旧行为)
     *   bit 0 (0x01) 置位                → "person" 匹配
     *   bit 1 (0x02) 置位                → 整个"车类" 匹配(car/truck/bus/van/vehicle)
     *                                       与 buildDetectTarget labelToClassId 对齐(车类都报 2)
     *   0x03 (人+车)                    → 两者都匹配
     *   0x04+ 仅有未定义 bit 置位        → 视为 0(任意 label,防御性)
     *
     * 例:GS 下发 1 → 只挑 person;下发 2 → 只挑车类(含 truck);下发 0/3/4+ → 不限
     */
    bool labelMatchesTargetType(const std::string& label, uint32_t target_type) const {
        const uint32_t known = target_type & 0x03u;
        if (known == 0u) {
            return true;  // 0 或 仅未定义位 → 任意
        }
        if ((known & 0x01u) && label == "person") return true;
        if ((known & 0x02u) && isVehicleLabel(label)) return true;
        return false;
    }

    /**
     * 是否属于"车类"(2026-07-30 与 labelToClassId 保持一致)
     * 集中放这里是为了 target_type 路由与 buildDetectTarget 上报语义对齐:
     *   labelToClassId 把 car/truck/bus/van/vehicle 全报 class_id=2(车),
     *   所以 target_type bit1(0x02)也要覆盖这五个,否则机载选了 truck 但
     *   GS 端看到上报是"车"会出现"打哪个"的歧义
     */
    static bool isVehicleLabel(const std::string& label) {
        return label == "car"     || label == "vehicle" || label == "truck" ||
               label == "bus"     || label == "van";
    }

    /**
     * 类型化 WorkMode 回调
     * - 唯一的工作模式入口(已移除 /mission/mode String 兼容路径)
     * - WorkMode 只表达"在任务区域内做什么",阶段语义由 MissionPhase 承载
     */
    void typedWorkModeCallback(const multi_uav_strike::WorkMode::ConstPtr& msg) {
        typed_work_mode_ = msg->mode;
        typed_work_mode_set_time_ = ros::Time::now();
        has_typed_work_mode_ = true;

        // 映射 typed mode → 内部 WorkMode
        // (WorkMode.msg 已精简,本 switch 只处理当前活跃的 5 个值)
        WorkMode new_mode = current_work_mode_;
        switch (msg->mode) {
            case multi_uav_strike::WorkMode::IDLE:
                new_mode = WorkMode::IDLE;
                break;
            case multi_uav_strike::WorkMode::SEARCH_ONLY:
                new_mode = WorkMode::SEARCH_ONLY;
                break;
            case multi_uav_strike::WorkMode::SEARCH_TRACK:
                new_mode = WorkMode::SEARCH_TRACK;
                break;
            case multi_uav_strike::WorkMode::SEARCH_STRIKE:
                new_mode = WorkMode::SEARCH_STRIKE;
                break;
            case multi_uav_strike::WorkMode::DENIED_ENV_FLIGHT:
                new_mode = WorkMode::DENIED_ENV_FLIGHT;
                break;
            default:
                ROS_WARN_THROTTLE(5.0, "[MissionManager] Unknown typed work_mode=%u", msg->mode);
                return;
        }
        if (new_mode != current_work_mode_) {
            WorkMode old_mode = current_work_mode_;
            current_work_mode_ = new_mode;
            ROS_WARN("[MissionManager] Typed WorkMode: %s (overriding %s)",
                     workModeToString(new_mode).c_str(),
                     workModeToString(old_mode).c_str());
        }
    }

    /**
     * MAV_CMD_ATTACK typed 回调
     * 仅在 SEARCH_TRACK 模式下有意义(目标已锁定等地面站确认)
     *
     * action:
     *   0 = 打: 触发 strike + 上报 DetectTarget(type=1)
     *   1 = 忽略: 丢弃目标,继续 search
     *   2 = 暂存: 上报 DetectTarget(type=2),继续 search
     */
    void attackCmdCallback(const multi_uav_strike::AttackCmd::ConstPtr& msg) {
        // 必须在 LOCKED_WAIT_CONFIRM 状态(SEARCH_TRACK 已锁定目标等确认)
        if (!tracked_target_.is_valid ||
            tracked_target_.lock_state != TrackLockState::LOCKED_WAIT_CONFIRM) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] attack_cmd received but no target locked "
                                   "(action=%u), ignoring", msg->action);
            return;
        }

        multi_uav_strike::DetectTarget dt;
        dt.timestamp_us = ros::Time::now().toNSec() / 1000;
        if (tracked_target_.latest) {
            dt.label        = tracked_target_.latest->label;
            dt.confidence   = tracked_target_.latest->confidence;
            // 设备 GPS — 同 buildDetectTarget,优先 real GPS,未 lock 时退化为 0
            if (is_gps_received_) {
                dt.dev_lat = current_gps_.latitude;
                dt.dev_lon = current_gps_.longitude;
                dt.dev_alt = current_gps_.altitude;
            } else {
                dt.dev_lat = 0.0;
                dt.dev_lon = 0.0;
                dt.dev_alt = 0.0;
            }
            // 设备偏航(弧度,NWU)— 从 current_pose 四元数解算
            {
                const auto& q = current_pose_.pose.orientation;
                const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                dt.dev_yaw = std::atan2(siny_cosp, cosy_cosp);
            }
            dt.obj_lat      = tracked_target_.latest->obj_lat;
            dt.obj_lon      = tracked_target_.latest->obj_lon;
            dt.obj_alt      = tracked_target_.latest->obj_alt;
            dt.target_type  = 3;  // 默认普通搜索目标
            dt.img_data     = tracked_target_.latest->img_data;  // 透传之前的 JPEG
        }
        dt.img_format = 1;  // JPEG

        switch (msg->action) {
            case 0: {
                // 打击
                dt.target_type = 1;
                ROS_WARN("[MissionManager] AttackCmd action=0 (打) — triggering strike");
                triggerStrike();
                detect_target_pub_.publish(dt);
                {
                    multi_uav_strike::DetectTargets batch;
                    batch.targets.push_back(dt);
                    detect_targets_pub_.publish(batch);
                }
                break;
            }
            case 1: {
                // 忽略 — 不上报目标,但加黑名单(20s/20m/同 label 内不再自动锁)
                //   否则下一帧 YOLO 再命中同目标,checkYoloDrivenStrike 门控 4
                //   (lock_state==NOT_LOCKED) 满足 → 重新锁 → ignore 指令形同虚设
                //
                // 同时必须停掉已 active 的 guidance:
                //   - startGuidanceApproach() 设了 is_guidance_active_=true
                //   - guidance_control_node 持续向目标位置飞,UAV 不会停
                //   - 后续 YOLO 命中 → 门控 3 (is_guidance_active_) 早返回,门控 7 (黑名单) 永远跑不到
                //   - 必须 disableGuidance() + 发 "resume" 给 waypoint_executor 才能真正"忽略"
                ROS_WARN("[MissionManager] AttackCmd action=1 (忽略) — adding to blacklist, resume search");
                if (tracked_target_.latest) {
                    addIgnoredTarget(locked_cluster_ned_x_, locked_cluster_ned_y_, locked_cluster_ned_alt_,
                                     tracked_target_.yolo_label);
                }
                if (is_guidance_active_) {
                    disableGuidance();
                    ROS_WARN("[MissionManager]   - guidance DISABLED (was active before ignore)");
                }
                // 清掉 reported_to_gs_ / strike_dispatched_,让下一个新 cluster(包括同 label
                //   不同位置的目标 2) 可以重新进入 dispatch 流程,而不是被 dedup 跳过。
                reported_to_gs_.clear();
                strike_dispatched_.clear();
                // 清掉 TargetState,避免 handleSearchTrack/Strike 误以为"仍锁定"再次触发 guidance
                current_target_.is_locked   = false;
                current_target_.is_detected = false;
                current_target_.is_shared   = false;
                // 释放锁定:reset One Euro Filter,旧目标的 x_prev/dx_prev 不再影响下次锁定
                target_smooth_x_.reset();
                target_smooth_y_.reset();
                target_smooth_z_.reset();
                target_smooth_last_time_        = ros::Time();
                target_smooth_locked_cluster_id_ = 0;
                // 重启 waypoint_executor(它现在还是 stop 状态,要 resume 才能继续扫描)
                {
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                }
                // phase 回 WAYPOINT_FOLLOW(原本被 startGuidanceApproach 改成 GUIDANCE_TRACK,
                //   MissionState 上报里 GS 看到的应是航点跟踪阶段,而非 guidance)
                current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                tracked_target_.is_valid = false;
                setLockState(TrackLockState::NOT_LOCKED);
                // 多目标: 忽略主目标后,清掉 primary;estimator 在下一帧重新锚定下一个
                publishPrimaryTarget(0);
                return;
            }
            case 2: {
                // 暂存 — 上报 DetectTarget(type=2) 同时加黑名单(本次跳过,接下来不应再自动锁)
                //   收尾动作与 action=1 完全一致(action=2 仅多 publish 一个 DetectTarget)
                dt.target_type = 2;
                ROS_WARN("[MissionManager] AttackCmd action=2 (暂存) — adding to blacklist");
                detect_target_pub_.publish(dt);
                {
                    multi_uav_strike::DetectTargets batch;
                    batch.targets.push_back(dt);
                    detect_targets_pub_.publish(batch);
                }
                if (tracked_target_.latest) {
                    addIgnoredTarget(locked_cluster_ned_x_, locked_cluster_ned_y_, locked_cluster_ned_alt_,
                                     tracked_target_.yolo_label);
                }
                if (is_guidance_active_) {
                    disableGuidance();
                    ROS_WARN("[MissionManager]   - guidance DISABLED (was active before stash)");
                }
                // 清掉 reported_to_gs_ / strike_dispatched_,让目标 2(同 label 不同位置)
                //   可以重新进入 dispatch。
                reported_to_gs_.clear();
                strike_dispatched_.clear();
                current_target_.is_locked   = false;
                current_target_.is_detected = false;
                current_target_.is_shared   = false;
                // 释放锁定:reset One Euro Filter,旧目标的 x_prev/dx_prev 不再影响下次锁定
                target_smooth_x_.reset();
                target_smooth_y_.reset();
                target_smooth_z_.reset();
                target_smooth_last_time_        = ros::Time();
                target_smooth_locked_cluster_id_ = 0;
                {
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                }
                current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                tracked_target_.is_valid = false;
                setLockState(TrackLockState::NOT_LOCKED);
                // 多目标: 暂存后清 primary,estimator 在下一帧重新锚定下一个
                publishPrimaryTarget(0);
                return;
            }
            default:
                ROS_WARN("[MissionManager] Unknown attack_cmd action=%u", msg->action);
                return;
        }

        // 无论打还是暂存,清掉锁定态(下一次识别需要重新等确认)
        // 注:action=1/2 已在前面的 case 里 return,这里只到 action=0(打)路径
        // === 2026-08-05 修:action=0 (打) 路径不再清锁定态 ===
        //   旧 bug:打完立刻清 tracked_target_.is_valid + publishPrimaryTarget(0),
        //   下一帧 clusterStatesCallback 进 SEARCH_STRIKE 分支看到 is_valid=false →
        //   走 dispatch 路径按"距 WP2 最近 + 最高置信度"重选 primary,覆盖 triggerStrike 锁定的目标。
        //   实测:用户跟踪 cluster 1 (person, conf 0.88) 发 attack_cmd 打 person,
        //   结果被 cluster 2 (car, conf 0.94, WP2-gate 内) 覆盖,打了车。
        //   修法:action=0 (打) 时保持 is_valid=true + publishPrimaryTarget(locked_cluster_id_),
        //   让 clusterStatesCallback 跳 dispatch 路径,沿用 triggerStrike 锁定的目标直到打击结束。
        //   解锁时机:strike_distance_threshold_ 达到 → direct_strike_committed_=true 流程内;
        //   或者 phase 切到 GUIDANCE_TRACK/GUIDANCE_STRIKE 之外的下一阶段(complete/failed)。
        if (msg->action != 0) {
            tracked_target_.is_valid = false;
            setLockState(TrackLockState::NOT_LOCKED);
            // 多目标: 打完/暂存后清主目标,estimator 在下一帧 cluster_states 重新锚定
            //   下一个未上报的 cluster(若有)
            publishPrimaryTarget(0);
        } else {
            // action=0 (打):保持锁定,只更新 primary_target 指向 locked_cluster_id_
            //   (target_estimator 仍可能拿到老 primary 值,这里强行覆盖)
            if (locked_cluster_id_ != 0) {
                publishPrimaryTarget(locked_cluster_id_);
            }
        }
    }

    /**
     * 触发 strike(用现有 strikes_distance_threshold 流程)
     * 把现有 TargetState 桥接到 guidance_control_node
     *
     * 改进:current_target_.pose.position 用 cluster.ned_x/y/z(NWU 米)直接填,
     *   不再用 obj_lat/lon/alt(后者是 WGS84,填进 NED 字段会偏飞)。
     *   targetEstPoseCallback(target_estimated_pose 来自 target_estimator 10Hz 实时位置)
     *   在 is_locked=true 后会持续更新 current_target_.pose,几帧后即可收敛到精确位置。
     */
    void triggerStrike() {
        // === Stale cluster 守卫(2026-07-26)===
        //   场景:UAV 到达 attack_point 时若没有真实目标在视野里,target_estimator
        //   可能仍持有很久前锁上的 stale cluster(last_seen_us 在过去几秒甚至更久)。
        //   该 cluster 会反复触发 strike → guidance → target_lost → 重新触发 strike
        //   → guidance 的 ping-pong,飞机背着倒退飞。
        //   真正能反映"目标是否还在视野里"的判据是 locked_cluster_last_seen_us_
        //   (YOLO 实际命中时刻,微秒);locked_cluster_time_ 只是 cluster_states 回调
        //   时刻,只要 cluster 还活着就会被 10Hz 持续刷新,age 永远是 0.05~0.1s,
        //   用它判 stale 等于不判。
        //   守门:last_seen_us_age > 阈值时放弃 strike 并清锁,让 waypoint_executor
        //   继续沿 skill_area_path 飞向 attack_point,新 cluster 进得来时重新触发 strike。
        if (locked_cluster_id_ == 0) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] triggerStrike aborted: no locked cluster");
            return;
        }
        if (locked_cluster_last_seen_us_ != 0) {
            const double last_seen_age_sec =
                (ros::Time::now().toNSec() / 1000ULL - locked_cluster_last_seen_us_) / 1e6;
            if (last_seen_age_sec > max_strike_cluster_age_sec_) {
                ROS_WARN_THROTTLE(2.0, "[MissionManager] triggerStrike aborted: stale "
                                  "locked_cluster_id_=%lu last_seen_age=%.2fs > max=%.2fs "
                                  "— clearing lock, let waypoint continue to attack_point",
                                  static_cast<unsigned long>(locked_cluster_id_),
                                  last_seen_age_sec, max_strike_cluster_age_sec_);
                locked_cluster_id_            = 0;
                locked_cluster_time_          = ros::Time();
                locked_cluster_last_seen_us_  = 0;
                tracked_target_.is_valid      = false;
                setLockState(TrackLockState::NOT_LOCKED);
                return;
            }
        }
        // 新锁定:reset One Euro Filter,确保首次位置不会被旧目标的 x_prev 污染
        target_smooth_x_.reset();
        target_smooth_y_.reset();
        target_smooth_z_.reset();
        target_smooth_last_time_        = ros::Time();
        target_smooth_locked_cluster_id_ = 0;  // targetEstPoseCallback 会在下次回调时刷成新 cluster_id
        if (tracked_target_.latest) {
            current_target_.is_detected = true;
            current_target_.is_locked   = true;
            // 优先用 locked_cluster_* (NWU,米) — clusterStatesCallback 在锁定瞬间存的快照,
            //   降级到 0 让 target_estimated_pose 在下一拍覆盖
            current_target_.pose.pose.position.x = locked_cluster_ned_x_;
            current_target_.pose.pose.position.y = locked_cluster_ned_y_;
            current_target_.pose.pose.position.z = locked_cluster_ned_alt_;
        }
        // 强制进入 SEARCH_STRIKE 分支以触发现有 strike 流程
        current_work_mode_ = WorkMode::SEARCH_STRIKE;
        // === Attack(105) 云台切到"朝 primary cluster 看"(2026-07-30)===
        //   一旦 strike 真触发,云台语义由"朝 WP2 看"切到"朝聚类位置看"。
        //   gimbalCommandTimerCallback 读此标志决定俯仰角计算源。
        attack_strike_phase_active_ = true;
        startGuidanceApproach();
    }

    /**
     * cluster_states 回调 — target_estimator_node 10Hz 发布所有 alive cluster 的快照
     * 替代原 clusterEventCallback(后者在新 cluster 首次出现时 publish 一次,
     * 若该时机早于 IN_TASK,目标永远锁不上;改为周期 snapshot 后,
     * UAV 进入 IN_TASK 时任何还活着的 cluster 都会被扫到并触发锁定)。
     *
     * 行为取决于 current_work_mode_:
     *   IDLE:                                                    忽略
     *   SEARCH_ONLY  + 当前 skill IN_TASK + 同 cluster 未上报过: 上报 DetectTarget(type=3 普通搜索目标)一次
     *   SEARCH_TRACK + 当前 skill IN_TASK + 同 cluster 未上报过: 锁目标 + 发布 TrackingState(state=1,等地面站 attack_cmd)
     *   SEARCH_STRIKE + 当前 skill IN_TASK + 同 cluster 未上报过: 上报 DetectTarget(type=1 攻击目标) + 触发 strike
     *
     * 多目标改造 (2026-07):
     *   - 同一 cluster_states 回调里可能含多个 cluster。SEARCH_TRACK/STRIKE 模式下,
     *     只有**首个**未被报告过、且未被黑名单的 cluster 会被锁定为主目标
     *     (或转移自同 label);其余 cluster 仅上报 DetectTarget(type=3) 给 GS,不动 tracked_target_。
     *   - 主目标被选定/转移时,发布 mission/primary_target (UInt64 cluster_id),
     *     target_estimator 据此锚定 guidance 的 target_estimated_pose,避免最高置信度跳变。
     *   - tracked_target_ / locked_cluster_* / current_target_ 仍保持**单目标**语义,代表主目标。
     *   - 打击仍是单机单目标:打完一个 → 状态复位 → 下一帧主目标切下一个未打击 cluster。
     *
     * 关键 gating(同原 clusterEventCallback):
     *   1. work_mode != IDLE
     *   2. current_skill_index_ 必须指向一个有效 skill
     *   3. skill.state 必须 == IN_TASK(UAV 已进入 skill_area_path,正在执行)
     *   4. skill.skill_type ∈ {102 SEARCH, 105 ATTACK}
     *   5. cluster.label 不在 ignored_targets_ 黑名单内(label-only 匹配)
     *   6. cluster.cluster_id 未在 reported_to_gs_ / strike_dispatched_ 内(dedup by id)
     *   7. cluster 与 reported_to_gs_ 已有位置 < report_dedup_radius_m_ 且 label 同 → 跳过(dedup by pos)
     */
    void clusterStatesCallback(const multi_uav_strike::ClusterState::ConstPtr& msg) {
        if (current_work_mode_ == WorkMode::IDLE) {
            return;
        }

        // === Gating 1: 必须有有效 skill ===
        if (current_skill_index_ >= skill_queue_.size()) {
            return;
        }
        auto& sr = skill_queue_[current_skill_index_];
        const bool direct_look = (sr.msg.skill_type == 105 && sr.direct_look_active_);
        const bool direct_strike_tracking =
            (sr.msg.skill_type == 105 && sr.direct_strike_committed_ &&
             (is_guidance_active_ || current_phase_ == MissionPhase::PHASE_GUIDANCE_STRIKE));
        const bool direct_pre_wp = direct_look || direct_strike_tracking;
        if (sr.state != SkillState::IN_TASK && !direct_pre_wp) {
            return;
        }
        if (sr.msg.skill_type != 102 && sr.msg.skill_type != 105) {
            return;
        }

        // === Gating 1.5: UAV 必须已到达 skill_area_path 至少第一条 ===
        //   102 SEARCH 跟踪起点:必须压线飞到第一条 skill_area 才开始锁/上报目标
        //   (此前 bug:UAV 还在飞向第一条途中,phase=SKILL_AREA 一进 IN_TASK 立即锁目标)
        //   waypoint_executor idx 语义: idx = "当前要飞的航点编号" (0=第一条还没到,
        //                              1=第一条到了开始飞第二条) — 见 executeSegmentFlight:730-732
        //   所以 skill_idx >= 1 = "至少到过第一条"
        //   skill_total=0(空 skill_area)跳过此门控,按 SEARCH 无搜索区场景处理
        // === CODEX DIRECT-LOOK START ===
        // 直视预检发生在 Skill 尚未下发、尚未到达 WP1 的窗口，因此只对这个明确子阶段
        // 绕过 waypoint status / skill_idx 门控；正常 Search 和回退后的 Attack 完全沿用旧门控。
        if (direct_look &&
            (ros::Time::now() - sr.direct_look_start_time_).toSec() <
                attack_direct_look_settle_sec_) {
            return;  // 先给机头与云台留出稳定时间，不把转动途中的掠过检测计入三帧
        }
        if (!direct_pre_wp &&
            (!latest_wp_status_ || latest_wp_status_->skill_id != sr.msg.skill_id)) {
            // 2026-07-29 B3:之前这里静默 return,排查时无法判断为什么 cluster_states
            //   全部被丢。加 WARN_THROTTLE 一眼看到 skill_id 不匹配窗口期。
            ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster callback gated: wp_status "
                              "skill_id mismatch (wp=%s vs expected=%s), "
                              "skip lock/report (typical during skill transition)",
                              (latest_wp_status_ ? latest_wp_status_->skill_id.c_str() : "<none>"),
                              sr.msg.skill_id.c_str());
            return;
        }
        if (!direct_pre_wp && latest_wp_status_->skill_total > 0 &&
            latest_wp_status_->skill_idx < 1) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster callback gated: UAV not yet at "
                              "first skill_area (skill_idx=%u / skill_total=%u), "
                              "skip lock/report until arrival",
                              latest_wp_status_->skill_idx, latest_wp_status_->skill_total);
            return;
        }
        // === CODEX DIRECT-LOOK END ===

        // === Gating 2: 1Hz 节流 dedup 清理(避免每帧遍历) ===
        cleanupReportedClusters();

        // === Attack(105) SEARCH_STRIKE target_type 路由(2026-07-30) ===
        //   旧逻辑:第一个过所有门的 cluster 即 primary(多目标乱枪)
        //   新逻辑:全部过 WP2-gate + label 匹配 target_type 位的 cluster 收集,
        //     loop 外挑距 WP2 最近的那个调 processSearchStrikeStrike
        //   仅在 Attack(105) + SEARCH_STRIKE 模式下启用,其他 mode 走原路径
        std::vector<const multi_uav_strike::ClusterTarget*> attack_candidates;
        const multi_uav_strike::ClusterTarget* attack_best = nullptr;
        double attack_best_d_wp2 = std::numeric_limits<double>::infinity();

        // === 2026-08-03: 提前收 alive cluster_ids,case 体用它判断"旧目标是否还在 FOV" ===
        //   背景:SEARCH_TRACK 阶段"已锁任何 label 拒绝"守卫需要区分:
        //     a) 旧目标还在 FOV(同一 cluster_id 或同 label 多个 cluster_id 在 msg 里)
        //        → case 体应 break,转移由 transfer 路径按距离阈值决定
        //     b) 旧目标已经离开 FOV(locked_cluster_id_ 不在 msg 里)
        //        → case 体应允许锁新目标,否则永远卡在旧锁 + TARGET_LOST_HOVER
        //   实现:在 for loop 开始前一次性扫描,后续 O(1) 查表。
        std::set<uint64_t> alive_cluster_ids;
        for (const auto& cluster : msg->targets) {
            alive_cluster_ids.insert(cluster.cluster_id);
        }

        // === 遍历所有 alive cluster,对每个独立应用 gating + 派发 ===
        for (const auto& cluster : msg->targets) {
            // 黑名单(GS attack_cmd action=1/2 推入的 label + 位置名单)
            // 必须传 NED:同 label 不同物理目标不应被一起屏蔽(2026-07-16 改造)
            if (isTargetIgnored(cluster.label, cluster.ned_x, cluster.ned_y)) {
                ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster[%lu] label=%s IGNORED (in blacklist, "
                                  "near ignored target within radius)",
                                  cluster.cluster_id, cluster.label.c_str());
                continue;
            }

            // === Stale gate by cluster.last_seen_us (2026-07-26, 2026-07-29 拆分阈值) ===
            //   真正能反映"目标是否还在视野里"的判据是 cluster.last_seen_us
            //   (target_estimator 在 YOLO 命中时刷新,见 target_estimator_node.cpp:285)。
            //   cluster_states 10Hz 持续把 cluster 当 alive,但 UAV 飞过目标位置后
            //   YOLO 不再命中 → last_seen_us 停在过去。
            //   之前用 locked_cluster_time_(cluster_states 回调时刻)做 stale 判据无效,
            //   因为只要 cluster 还活着就会被 10Hz 持续刷新,age 永远是 0.05~0.1s,
            //   触发不了任何 threshold,导致:
            //     1. clusterStatesCallback 仍把 stale cluster 派发到 SEARCH_STRIKE,
            //        triggerStrike() 启动 guidance 飞向 stale pose,UAV "倒退"
            //     2. handleSearchStrike 每 tick 看 is_guidance_active_=false +
            //        current_target_.is_locked=true → 反复 startGuidanceApproach,
            //        触发 100ms 一次的 GUIDANCE_STRIKE ↔ TARGET_LOST_HOVER ping-pong
            //   === 2026-07-29 改造:两个阈值分立 ===
            //     - max_strike_cluster_age_sec_ (0.5s):UAV 安全门,**无差别应用** —
            //       stale 会让 UAV 飞向旧位置,必须 skip strike 触发,并 erase
            //       strike_dispatched_(允许新检测重新派发)
            //     - max_report_cluster_age_sec_ (5.0s):GS 上报门,**只对"已上报过"的 cluster
            //       生效** — 重复上报必须新鲜,避免 GS 反复收到旧坐标;首次上报豁免,
            //       即使 stale 也比漏报强(典型时序:短暂遮挡/边缘掠过 → 目标晋升后
            //       last_seen_us 已超 5s,但 gating 一旦开放(wp_status 到位 / 锁转移 /
            //       skill 切换)就该立刻上报,不该被永久屏蔽)
            //   原版 bug B1:0.5s 阈值同时挡上报 → 绿球持久亮但永远报不出去
            //   二次 bug(同日):5s 阈值无差别拦首次上报 → 同样导致"绿了但从未上报过的
            //     目标永远报不出"(5 目标只报 2 的根因之一)
            bool is_strike_stale = false;
            if (cluster.last_seen_us != 0) {
                const ros::Time now_ros = ros::Time::now();
                const double    cluster_age_sec =
                    (now_ros.toNSec() / 1000ULL - cluster.last_seen_us) / 1e6;
                if (cluster_age_sec > max_strike_cluster_age_sec_) {
                    is_strike_stale = true;
                    // erase strike_dispatched_ 仅此一项,reported_to_gs_ 保留
                    strike_dispatched_.erase(cluster.cluster_id);
                }
                // === stale gate 仅对"已上报过"的 cluster 生效 ===
                //   C1 dedup 检查(下方)在 stale 之后,但这里**提前**用同样的查询
                //   避免把 stale 判断推迟到 dedup 通过后,导致 stale 计算结果被覆盖
                const bool already_reported =
                    reported_to_gs_.count(cluster.cluster_id) > 0 ||
                    strike_dispatched_.count(cluster.cluster_id) > 0;
                if (cluster_age_sec > max_report_cluster_age_sec_ && already_reported) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster[%lu] label=%s STALE-republish "
                                      "(last_seen_age=%.2fs > max=%.2fs, already_reported=true, "
                                      "pose=%.1f,%.1f,%.1f) → skip both report and strike",
                                      cluster.cluster_id, cluster.label.c_str(),
                                      cluster_age_sec, max_report_cluster_age_sec_,
                                      cluster.ned_x, cluster.ned_y, cluster.ned_alt);
                    continue;
                }
                if (cluster_age_sec > max_report_cluster_age_sec_) {
                    // Stale 但从未上报过:首次上报豁免,GS 看到比漏报强
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster[%lu] label=%s STALE-first-report "
                                      "(last_seen_age=%.2fs > max=%.2fs, already_reported=false, "
                                      "pose=%.1f,%.1f,%.1f) → allow first publish",
                                      cluster.cluster_id, cluster.label.c_str(),
                                      cluster_age_sec, max_report_cluster_age_sec_,
                                      cluster.ned_x, cluster.ned_y, cluster.ned_alt);
                }
                if (is_strike_stale) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster[%lu] label=%s STALE-strike "
                                      "(last_seen_age=%.2fs > strike_max=%.2fs, "
                                      "≤ report_max=%.2fs) → skip strike, keep report",
                                      cluster.cluster_id, cluster.label.c_str(),
                                      cluster_age_sec, max_strike_cluster_age_sec_,
                                      max_report_cluster_age_sec_);
                }
            }

            // === SEARCH_TRACK 模式特例:lock 跟 label 走,不跟 cluster_id ===
            // 场景:FOV 边缘目标反复进出 / UAV 高速机动 → target_estimator 因 LOS 投影位置
            //   偏移反复开新 cluster_id;但目标没换。如果按 cluster_id 死锁,tracked_target_
            //   会指向已"stale"的旧 cluster,UAV 飞向错位置,attack_cmd 拿到的是旧坐标。
            // 解法:同 label 已锁 → 把 lock 转移到当前 cluster,持续刷新位置。
            //   这等价于"没有聚类时每帧 YOLO 命中都重新锁"的旧行为,只是入口变成 cluster。
            // === 2026-08-03 修:stale cluster 拒绝 transfer ===
            //   旧 bug:transfer 路径无 stale 检查,把 locked_cluster_last_seen_us_ 写成
            //   stale 时间戳 → 下个 tick 立即 TARGET_LOST_HOVER → waypoint_executor
            //   收到 stop,guidance 关了没 resume → 飞机滞空 → failsafe。
            //   (首次锁 stale 在下方 case 头部已拦,这里是同 label 转移路径的对应拦)
            // === 2026-08-03 修:距离去抖 — transfer 距离阈值 ===
            //   旧 bug:FOV 里同时有多个同 label 目标(A, B 距离 5m),YOLO 在两者间
            //   反复切换 → cluster_id 也反复变化 → transfer 路径把 locked_cluster_*
            //   来回覆盖到 A/B 位置 → 制导指令跳来跳去(虽然 One Euro Filter 在
            //   targetEstPoseCallback 层有平滑,但跳变太密时残差仍大)。
            //   修法:new cluster 与当前 locked_cluster_* 距离 < report_dedup_radius_m_ * 0.5
            //   (约 12.5m) → 不 transfer,保持原锁,让 target_estimator primary_target
            //   继续指旧 cluster 收位置,One Euro 平滑自然吸掉小幅抖动。
            //   例外:locked_cluster_id_==0(从未锁)或新 cluster 距离远超阈值(旧目标
            //   真走了)→ 仍允许 transfer,避免被卡死。
            bool transfer_distance_ok = true;
            if (locked_cluster_id_ != 0) {
                const double dx = cluster.ned_x - locked_cluster_ned_x_;
                const double dy = cluster.ned_y - locked_cluster_ned_y_;
                const double transfer_dist = std::hypot(dx, dy);
                const double transfer_threshold = report_dedup_radius_m_ * 0.5;
                if (transfer_dist < transfer_threshold) {
                    // 距离太近,认为"还是同一个目标,只是 cluster_id 在抖",不 transfer
                    transfer_distance_ok = false;
                }
            }
            if (current_work_mode_ == WorkMode::SEARCH_TRACK &&
                tracked_target_.is_valid &&
                tracked_target_.yolo_label == cluster.label &&
                !is_strike_stale &&
                !is_guidance_active_ &&  // 2026-08-03:guidance 已在飞时不允许 transfer,
                                          //   否则 triggerStrike 后下一帧 transfer 改锁
                                          //   → targetEstPoseCallback 收新位置 → 飞走
                transfer_distance_ok) {
                // 转移到新 cluster(可能 cluster_id 与上帧不同,只要 label 一致就 transfer)
                multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                tracked_target_.latest     = boost::make_shared<multi_uav_strike::DetectTarget>(dt);
                // 仍是 LOCKED_WAIT_CONFIRM → setLockState 早 return,不发重复 state=2
                setLockState(TrackLockState::LOCKED_WAIT_CONFIRM);
                tracked_target_.locked_at  = ros::Time::now();
                // 更新 locked_cluster_* 用于 triggerStrike
                locked_cluster_ned_x_         = cluster.ned_x;
                locked_cluster_ned_y_         = cluster.ned_y;
                locked_cluster_ned_alt_       = cluster.ned_alt;
                locked_cluster_id_            = cluster.cluster_id;
                locked_cluster_time_          = ros::Time::now();
                locked_cluster_last_seen_us_  = cluster.last_seen_us;
                // 当前 cluster_id 加入 strike_dispatched_(C1 拆分容器),避免同一 cluster 反复 transfer 触发 startGuidanceApproach
                strike_dispatched_[cluster.cluster_id] = -1;
                // === 多目标: 通知 estimator 主目标 cluster_id 转移 ===
                publishPrimaryTarget(cluster.cluster_id);
                continue;  // 本次循环内此 cluster 处理完,不去 dispatch
            }

            // dedup — 同一 cluster_id 同 task_flow 只处理一次
            // 但: 如果这是当前 locked cluster_id,每次 callback 都要刷新
            //   locked_cluster_time_ 和 locked_cluster_last_seen_us_,
            //   否则 advanceSkillStateMachine 算 (now - locked_cluster_time_)
            //   永远 = "首次派发以来秒数",2.0s 后必然误判 TARGET_LOST_HOVER,
            //   而此时 cluster 实际 YOLO 命中可能一直很新。 (2026-07-26)
            // === 2026-07-29:合并 reported_to_gs_ + strike_dispatched_ ===
            //   同一 cluster_id 已在任何一个 dex 中均视为已处理过。
            if (reported_to_gs_.count(cluster.cluster_id) ||
                strike_dispatched_.count(cluster.cluster_id)) {
                if (cluster.cluster_id == locked_cluster_id_ &&
                    cluster.last_seen_us != 0) {
                    locked_cluster_time_         = ros::Time::now();
                    locked_cluster_last_seen_us_ = cluster.last_seen_us;
                }
                continue;
            }

            // === C2 位置栅格去重(2026-07-29) ===
            //   同一物理目标因投影抖动产生多个 cluster_id 时,cluster_id dedup 漏掉。
            //   额外检查:与 reported_to_gs_ 已有位置距离 < report_dedup_radius_m_ 且 label
            //   相同 → 视为同一目标,跳过(不 publish、不 dispatch、不写 dedup)。
            //   这条放在 cluster_id dedup 之后:同一 cluster_id 永远不会双发。
            //   放在 dispatch 之前:还没准备 publish 时就拦截,避免 buildDetectTarget 浪费 +
            //   ROS_WARN 噪音。
            {
                bool dedup_hit = false;
                const double r2 = report_dedup_radius_m_ * report_dedup_radius_m_;
                for (const auto& kv : reported_to_gs_) {
                    if (kv.second.label != cluster.label) continue;
                    const double dx = cluster.ned_x - kv.second.ned_x;
                    const double dy = cluster.ned_y - kv.second.ned_y;
                    if (dx*dx + dy*dy < r2) {
                        dedup_hit = true;
                        break;
                    }
                }
                if (dedup_hit) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Cluster[%lu] label=%s dedup "
                                      "by POSITION (within %.1fm of reported cluster) → skip",
                                      cluster.cluster_id, cluster.label.c_str(),
                                      report_dedup_radius_m_);
                    continue;
                }
            }

            // === 多目标: SEARCH_TRACK/STRIKE 模式若已锁定(主目标已就位),
            //     其他新出现的 cluster 仅被动上报 type=3 给 GS,不再覆盖 tracked_target_。
            //     这样保证一帧里 N 个目标时:UAV 只打一个,GS 能看到 N 个。
            if ((current_work_mode_ == WorkMode::SEARCH_TRACK ||
                 current_work_mode_ == WorkMode::SEARCH_STRIKE) &&
                tracked_target_.is_valid) {
                multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                detect_target_pub_.publish(dt);
                {
                    multi_uav_strike::DetectTargets batch;
                    batch.targets.push_back(dt);
                    detect_targets_pub_.publish(batch);
                }
                // 只记 reported_to_gs_,不进 strike_dispatched_(没触发 strike)
                reported_to_gs_[cluster.cluster_id] = {cluster.label, cluster.ned_x, cluster.ned_y};
                ROS_WARN("[MissionManager] Cluster[%lu] label=%s %s → non-primary, published type=3",
                         cluster.cluster_id, cluster.label.c_str(),
                         (current_work_mode_ == WorkMode::SEARCH_TRACK ? "SEARCH_TRACK" : "SEARCH_STRIKE"));
                continue;
            }

            // 缓存为 locked_cluster_*,供 attackCmdCallback / triggerStrike 使用
            locked_cluster_ned_x_         = cluster.ned_x;
            locked_cluster_ned_y_         = cluster.ned_y;
            locked_cluster_ned_alt_       = cluster.ned_alt;
            locked_cluster_id_            = cluster.cluster_id;
            locked_cluster_time_          = ros::Time::now();
            locked_cluster_last_seen_us_  = cluster.last_seen_us;

            // === Attack(105) WP2-gate(2026-07-30)===
            //   cluster 距 WP2 > wp2_strike_radius_m_ 时,仅被动上报 type=3,绝不动 locked/strike。
            //   避免 UAV 沿 WP1→WP2 飞行段先看到远处目标,被抢先锁走,飞到错位置。
            //   标记 attack_wp2_target_seen_ 表示本次 Attack 见过 WP2 范围内的目标,
            //   驱动"有/无目标"两套 hover 超时(见 advanceSkillStateMachine)。
            if (sr.msg.skill_type == 105) {
                const double d_wp2 = clusterDistToWp2Meters(cluster, sr);
                if (d_wp2 > wp2_strike_radius_m_) {
                    multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                    detect_target_pub_.publish(dt);
                    {
                        multi_uav_strike::DetectTargets batch;
                        batch.targets.push_back(dt);
                        detect_targets_pub_.publish(batch);
                    }
                    reported_to_gs_[cluster.cluster_id] = {cluster.label, cluster.ned_x, cluster.ned_y};
                    ROS_WARN_THROTTLE(2.0,
                        "[MissionManager] Cluster[%lu] label=%s dist_to_wp2=%.1fm > radius=%.1fm "
                        "→ outside WP2 zone, no strike (skill=105)",
                        cluster.cluster_id, cluster.label.c_str(),
                        d_wp2, wp2_strike_radius_m_);
                    continue;
                }
                // === 2026-07-31: 严格化 seen 判定 ===
                //   旧版: WP2 距离门过就 attack_wp2_target_seen_ = true
                //         → wrong-label / stale cluster 也把 300s 长超时降级到 30s 短超时
                //   新版: 仅当 target_type 匹配 + 非 stale 时才置位
                //   注意: attack_wp2_target_seen_ 现在延后到 target_type 路由 + 候选收集
                //         之后才置位(分别见下方 SEARCH_STRIKE / SEARCH_TRACK 两个分支末尾)

                // === Attack(105) target_type 路由(2026-07-30) ===
                //   WP2-gate 内的 cluster 若 label 不匹配 target_type 位掩码 → 标 type=3 走非主路径
                //   仅 SEARCH_STRIKE 模式下做路由;SEARCH_TRACK 仍走原"先到先锁"逻辑
                if (current_work_mode_ == WorkMode::SEARCH_STRIKE &&
                    !labelMatchesTargetType(cluster.label, sr.msg.target_type)) {
                    multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                    detect_target_pub_.publish(dt);
                    {
                        multi_uav_strike::DetectTargets batch;
                        batch.targets.push_back(dt);
                        detect_targets_pub_.publish(batch);
                    }
                    reported_to_gs_[cluster.cluster_id] = {cluster.label, cluster.ned_x, cluster.ned_y};
                    ROS_WARN_THROTTLE(2.0,
                        "[MissionManager] Cluster[%lu] label=%s target_type=0x%x no-match → publish type=3",
                        cluster.cluster_id, cluster.label.c_str(), sr.msg.target_type);
                    continue;
                }

                // === 收集候选(2026-07-30 deferred-pick) ===
                //   Attack(105) + SEARCH_STRIKE 下不再"先到先得",先收集所有 WP2-gate + label
                //   匹配 + 新鲜的 cluster,loop 外挑距 WP2 最近调 strike
                //   stale cluster 不入候选(strike 会拒绝,收进来会污染 best)
                if (current_work_mode_ == WorkMode::SEARCH_STRIKE) {
                    if (is_strike_stale) {
                        // 降级 type=3 上报(语义:看到过但已 stale),不进 strike_dispatched_
                        //   让下一帧 cluster 重新进 candidate 列表,完成"stale 后等新检测"语义
                        //   (Bug B:旧版在 SEARCH_STRIKE case 写 reported_to_gs_ 把重试堵死)
                        multi_uav_strike::DetectTarget dt_s = buildDetectTarget(cluster, /*target_type=*/3);
                        detect_target_pub_.publish(dt_s);
                        {
                            multi_uav_strike::DetectTargets batch;
                            batch.targets.push_back(dt_s);
                            detect_targets_pub_.publish(batch);
                        }
                        ROS_WARN_THROTTLE(2.0,
                            "[MissionManager] Cluster[%lu] label=%s SEARCH_STRIKE stale → "
                            "publish type=3 (no lock, no strike, re-try next frame)",
                            cluster.cluster_id, cluster.label.c_str());
                        continue;
                    }
                    attack_candidates.push_back(&cluster);
                    if (d_wp2 < attack_best_d_wp2) {
                        attack_best_d_wp2 = d_wp2;
                        attack_best       = &cluster;
                    }
                    // === 2026-07-31: 见到"可打目标"才置位 seen ===
                    //   到此步已通过: WP2 距离门 + target_type 路由 + !is_strike_stale
                    attack_wp2_target_seen_ = true;
                    continue;  // 不进 work_mode switch,由 loop 外的 post-process 处理
                }

                // === 2026-07-31: SEARCH_TRACK 模式 (非 SEARCH_STRIKE 分支) 也仅在 fresh 时置位 ===
                if (!is_strike_stale) {
                    attack_wp2_target_seen_ = true;
                }
            }

            // === 按 work_mode 派发 ===
            switch (current_work_mode_) {
            case WorkMode::SEARCH_ONLY: {
                // 普通搜索 — 仅上报 1 次 type=3
                multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                detect_target_pub_.publish(dt);
                {
                    multi_uav_strike::DetectTargets batch;
                    batch.targets.push_back(dt);
                    detect_targets_pub_.publish(batch);
                }
                reported_to_gs_[cluster.cluster_id] = {cluster.label, cluster.ned_x, cluster.ned_y};
                ROS_WARN("[MissionManager] Cluster[%lu] label=%s SEARCH_ONLY → published type=3",
                         cluster.cluster_id, cluster.label.c_str());
                break;
            }
            case WorkMode::SEARCH_TRACK: {
                // === 2026-08-03 修:stale 短路 — 不要锁 stale cluster,否则 80ms 内必然
                //   TARGET_LOST_HOVER → waypoint_executor 收到 stop + guidance 关
                //   但没人发 resume → UAV 滞空 → failsafe (典型复现:cluster 在 FOV 边缘
                //   短暂命中后失锁,stale 6.92s 仍被首次上报豁免(>=5s 才 skip report),
                //   SEARCH_TRACK case 旧版无 stale 守门 → 直接锁 + 启动 guidance)。
                //   与下方 SEARCH_STRIKE 头部 is_strike_stale 拦截对齐。 ===
                if (is_strike_stale) {
                    // 复用上方 STALE-strike log 的 cluster_age 信息,这里仅点出"锁拒绝"语义
                    ROS_WARN_THROTTLE(2.0,
                        "[MissionManager] Cluster[%lu] label=%s SEARCH_TRACK stale "
                        "(last_seen_us set but >strike_max=%.2fs) → skip lock, keep scanning "
                        "(will retry when YOLO produces a fresh hit)",
                        cluster.cluster_id, cluster.label.c_str(),
                        max_strike_cluster_age_sec_);
                    break;
                }
                // === 2026-08-03 修:换 label 拒绝 — 锁定 A 后不能被 B 静默覆盖 ===
                //   旧 bug:case 体内无"已锁 label"守卫,UAV 朝向位置有多个目标时,
                //   任意一个 fresh 的、不同 label 的 cluster 进 dispatch 路径都会直接
                //   setLockState + startGuidanceApproach,覆盖原 tracked_target_。
                //   GS 看到的 tracking_state 还在报旧 label,用户点打时 triggerStrike
                //   用最新 locked_cluster_*(已经被 B 覆盖),UAV 飞向 B,GS 端以为打的是 A。
                //   修法:已锁 label_X,新 cluster label_Y != X → 跳过,等用户显式 ignore
                //   (AttackCmd action=1) 或新 task_flow 触发取消跟踪后再锁下一个。
                //   例外:tracked_target_.is_valid=false(打完/ignore/失锁恢复后)允许首次锁。
                // === 2026-08-03 修:已锁任何 label 拒绝 — 让 transfer 路径处理同 label ===
                //   进一步收紧:即使同 label,case 体也不该再锁一次(会被 FOV 里同 label
                //   多目标 A/B 反复覆盖)。transfer 路径(L2892)在 case 之前已经按
                //   "距离 > report_dedup_radius_m_*0.5"判断过:
                //     - 距离够:旧目标真走了,transfer 路径已 continue,这里到不了
                //     - 距离不够:目标没变,只是 cluster_id 在抖,保持原锁
                //   所以 case 体只要检测 is_valid=true 就一律 break,任何覆盖都
                //   由 transfer 路径决定,避免双锁路径打架。
                if (tracked_target_.is_valid) {
                    // === 2026-08-03: 区分"旧目标还在 FOV" vs "已离开" ===
                    //   旧目标 cluster_id 仍在 alive_cluster_ids 中 → 同 label 的 transfer
                    //     路径已经处理(距离阈值),case 体不重复锁
                    //   旧目标 cluster_id 不在 alive_cluster_ids 中 → 目标已离开 estimator
                    //     聚类,transfer 路径看不到旧 cluster,无法 transfer;此时如果还
                    //     break,会永远卡在旧锁 + TARGET_LOST_HOVER
                    //   → 允许 case 体锁新目标(类比"打完/ignore 后重新锁")
                    // === 2026-08-03 修:guidance 阶段禁止 case 体重锁 ===
                    //   旧 bug:triggerStrike 启动 guidance 后 phase=GUIDANCE_STRIKE,
                    //   locked_cluster_* 仍指向 triggerStrike 当时的 cluster,但下一次
                    //   clusterStatesCallback 看到旧 cluster 已离开 FOV,走"已离开"分支
                    //   重新锁新 cluster → locked_cluster_* 被覆盖 → targetEstPoseCallback
                    //   收到新位置 → guidance 飞向新目标(用户感觉"自己飞走了")。
                    //   修法:guidance 已经在飞(is_guidance_active_=true)时一律拒绝 case
                    //   体改锁,让 guidance 按 triggerStrike 锁定的目标飞完;旧目标真丢了
                    //   走 target_lost_hold_sec_ 路径自然处理 TARGET_LOST_HOVER。
                    if (is_guidance_active_) {
                        if (tracked_target_.yolo_label != cluster.label) {
                            ROS_WARN_THROTTLE(2.0,
                                "[MissionManager] Cluster[%lu] label=%s SEARCH_TRACK: guidance "
                                "already active, ignoring different label='%s' (lock on '%s')",
                                cluster.cluster_id, cluster.label.c_str(),
                                cluster.label.c_str(),
                                tracked_target_.yolo_label.c_str());
                        }
                        break;
                    }
                    const bool old_still_alive = (locked_cluster_id_ != 0) &&
                        alive_cluster_ids.count(locked_cluster_id_) > 0;
                    if (old_still_alive) {
                        if (tracked_target_.yolo_label != cluster.label) {
                            ROS_WARN_THROTTLE(2.0,
                                "[MissionManager] Cluster[%lu] label=%s SEARCH_TRACK: already locked "
                                "on label='%s' (cluster_id=%lu still alive) → skip different label "
                                "(use AttackCmd action=1 ignore or new task_flow to switch)",
                                cluster.cluster_id, cluster.label.c_str(),
                                tracked_target_.yolo_label.c_str(),
                                static_cast<unsigned long>(locked_cluster_id_));
                        }
                        // 同 label:transfer 路径已按距离阈值处理,到这里说明被 transfer 拒绝
                        // (距离太近,目标没变,只是 cluster_id 在抖)。一律 break — 不让
                        // case 体再次锁,避免双锁路径打架。
                        break;
                    }
                    // 旧目标已离开 + guidance 未在跑 → 落到下面正常锁新目标。
                    // 发一次性 log 让 GS 能看到换锁时刻。
                    ROS_WARN_THROTTLE(2.0,
                        "[MissionManager] Cluster[%lu] label=%s SEARCH_TRACK: previous lock "
                        "on label='%s' (cluster_id=%lu) is no longer alive → re-lock to new target",
                        cluster.cluster_id, cluster.label.c_str(),
                        tracked_target_.yolo_label.c_str(),
                        static_cast<unsigned long>(locked_cluster_id_));
                }
                // 搜索跟踪 — 锁目标 + 发 tracking_state,**不上报** DetectTarget
                // 把 cluster 数据构造为 DetectTarget 缓存到 tracked_target_.latest,
                // 供后续 attackCmdCallback (action=0/1/2) 构造上行 dt 使用
                multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/3);
                auto dt_ptr = boost::make_shared<multi_uav_strike::DetectTarget>(dt);
                tracked_target_.is_valid   = true;
                tracked_target_.latest     = dt_ptr;
                tracked_target_.yolo_label = cluster.label;  // 字段名沿用,语义改为 cluster label
                // setLockState 内部在 NOT_LOCKED→LOCKED_* 转移时自动发一帧 state=2
                // (识别未跟踪,等 GCS ack);之后由 tracking_state_timer 持续发 state=1。
                setLockState(TrackLockState::LOCKED_WAIT_CONFIRM);
                tracked_target_.locked_at  = ros::Time::now();

                // SEARCH_TRACK 首次锁:仅记 strike_dispatched_,不进 reported_to_gs_
                //   (GS 端会从 tracking_state 看到锁定状态)
                strike_dispatched_[cluster.cluster_id] = -1;  // -1 = 仅锁未上报
                // task_speed=0(GS 漏发 / 老 Skill 模板默认值)时会被 guidance_control
                //   报"non-positive uav_speed"忽略,plane 不动,改成 std::max 兜底,见
                //   min_guidance_speed_ 字段注释(2026-07-26)
                //   Skill.msg task_speed 是 float32,需 cast 到 double 才能 std::max
                guidance_speed_ = std::max(static_cast<double>(sr.msg.task_speed), min_guidance_speed_);
                startGuidanceApproach();
                // === 多目标: 通知 estimator 主目标 cluster_id ===
                publishPrimaryTarget(cluster.cluster_id);
                ROS_WARN("[MissionManager] Cluster[%lu] label=%s SEARCH_TRACK → locked (primary), "
                         "tracking_state published, waiting for attack_cmd",
                         cluster.cluster_id, cluster.label.c_str());
                break;
            }
            case WorkMode::SEARCH_STRIKE: {
                // === 2026-07-30 Bug A 修:stale 时降级 type=3,不再无条件抢 primary 锁 ===
                //   旧逻辑:case 顶部先把 tracked_target_.is_valid=true + setLockState,
                //   3056 行才判 is_strike_stale,导致 stale cluster 把锁占住,
                //   后续 cluster 全部降级 non-primary → 真正目标永远打不了
                //   新逻辑:stale 在 case 头部短路,只 publish type=3 给 GS,不动锁
                if (is_strike_stale) {
                    multi_uav_strike::DetectTarget dt_d = buildDetectTarget(cluster, /*target_type=*/3);
                    detect_target_pub_.publish(dt_d);
                    {
                        multi_uav_strike::DetectTargets batch;
                        batch.targets.push_back(dt_d);
                        detect_targets_pub_.publish(batch);
                    }
                    // === Bug B 修:不写 reported_to_gs_,让下一帧 cluster 重新进 strike 路径 ===
                    //   旧版"等下一帧/下一次新检测触发 strike"注释名存实亡:
                    //     reported_to_gs_[id] 写了 → 下一帧 dedup 直接 continue → 永远不补打
                    ROS_WARN_THROTTLE(2.0,
                        "[MissionManager] Cluster[%lu] label=%s SEARCH_STRIKE stale "
                        "→ publish type=3 (no lock, no strike, re-try next frame)",
                        cluster.cluster_id, cluster.label.c_str());
                    break;
                }
                // 新鲜命中:正常 strike 流程(Attack(105) deferred-pick 也复用此方法)
                processSearchStrikeStrike(cluster, sr);
                break;
            }
            default:
                // 其他模式(DENIED_ENV_FLIGHT 等)不处理
                break;
            }  // end switch (current_work_mode_)
        }  // end for each cluster

        // === Attack(105) target_type 路由:loop 完挑最佳(2026-07-30) ===
        //   attack_best 已在 for 循环中按"距 WP2 最近"挑出,这里只 dispatch
        //   注意:仅当 Attack(105) + SEARCH_STRIKE 才走到这里;其他 skill/mode
        //     走原 SEARCH_STRIKE case(对 Search(102) 同样兼容)
        if (sr.msg.skill_type == 105 &&
            current_work_mode_ == WorkMode::SEARCH_STRIKE &&
            attack_best != nullptr) {
            // === CODEX DIRECT-LOOK START ===
            // 直视阶段必须是同一 cluster 连续三帧都通过：新鲜度、WP2 距离门和
            // target_type 路由均已在上方候选收集处验证。cluster_id 变化立即重新计数。
            if (direct_look) {
                if (sr.direct_look_cluster_id_ == attack_best->cluster_id &&
                    attack_best->last_seen_us > sr.direct_look_last_seen_us_) {
                    ++sr.direct_look_consecutive_hits_;
                } else if (sr.direct_look_cluster_id_ != attack_best->cluster_id) {
                    sr.direct_look_cluster_id_ = attack_best->cluster_id;
                    sr.direct_look_consecutive_hits_ = 1;
                } else {
                    // cluster_states 重复快照不算新检测帧；等待 last_seen_us 真正刷新。
                    return;
                }
                sr.direct_look_last_seen_us_ = attack_best->last_seen_us;
                ROS_WARN("[MissionManager] Attack DIRECT_LOOK Cluster[%lu] confirmed %d/%d frames",
                         static_cast<unsigned long>(attack_best->cluster_id),
                         sr.direct_look_consecutive_hits_, attack_direct_look_confirm_hits_);
                if (sr.direct_look_consecutive_hits_ < attack_direct_look_confirm_hits_) {
                    return;
                }
                sr.direct_look_active_ = false;
                sr.direct_strike_committed_ = true;
                ROS_WARN("[MissionManager] Attack DIRECT_LOOK confirmed Cluster[%lu] for %d "
                         "consecutive frames -> direct strike (skip WP1)",
                         static_cast<unsigned long>(attack_best->cluster_id),
                         attack_direct_look_confirm_hits_);

                // 先把当前 Attack 的 WP1->WP2 队列装入 executor，再由下面
                // triggerStrike()->startGuidanceApproach() 立即 stop。这样目标若之后
                // 真实丢失，resume 的是当前 Attack，而不是上一条 Search(102) 队列。
                sr.state = SkillState::PENDING;
                sr.state_enter_time = ros::Time::now();
                sr.last_event = "DIRECT_LOOK confirmed: preload Attack path before guidance";
                pushCurrentSkillToExecutor();
            }
            // === CODEX DIRECT-LOOK END ===
            ROS_WARN("[MissionManager] Attack(105) target_type=0x%x picked best: "
                     "Cluster[%lu] label=%s d_wp2=%.1fm (candidates=%zu)",
                     sr.msg.target_type,
                     attack_best->cluster_id, attack_best->label.c_str(),
                     attack_best_d_wp2, attack_candidates.size());
            // 直接复用 case 体内的 processSearchStrikeStrike(锁+gate+publish+strike)
            //   锁前已过滤 stale(collect 段),所以这里 is_strike_stale=false
            processSearchStrikeStrike(*attack_best, sr);
        } else if (direct_look) {
            // 当前快照没有任何合格候选，不满足“连续帧”，下一次从 1 重新累计。
            sr.direct_look_cluster_id_ = 0;
            sr.direct_look_last_seen_us_ = 0;
            sr.direct_look_consecutive_hits_ = 0;
        }
    }

    /**
     * SEARCH_STRIKE 派发主路径(2026-07-30 抽出)
     *  - 写 tracked_target_ + locked_cluster_* 锁
     *  - Attack(105) formation=1/2 时按 checkAttackStrikeGating 门控
     *  - 门控通过:publish type=1 + triggerStrike
     *  - 门控 WAITING:publish type=3 替代 + 记 attack_strike_pending_ 留待后续重试
     *
     * 由 clusterStatesCallback 的 SEARCH_STRIKE case(直接命中 cluster)与
     *   循环外 Attack(105) target_type 路由(从候选挑最优)两处复用
     * 调用前必须保证 cluster 已通过 stale gate(否则会写锁到陈旧位置)
     */
    void processSearchStrikeStrike(const multi_uav_strike::ClusterTarget& cluster,
                                    const SkillRuntime& sr) {
        // 锁
        multi_uav_strike::DetectTarget dt = buildDetectTarget(cluster, /*target_type=*/1);
        auto dt_ptr = boost::make_shared<multi_uav_strike::DetectTarget>(dt);
        tracked_target_.is_valid   = true;
        tracked_target_.latest     = dt_ptr;
        tracked_target_.yolo_label = cluster.label;
        setLockState(TrackLockState::LOCKED_AUTO);
        tracked_target_.locked_at  = ros::Time::now();

        // === 2026-07-31: 同步 locked_cluster_* 给 triggerStrike 用 ===
        //   triggerStrike() 入口按 locked_cluster_last_seen_us_ 做 stale 守门。
        //   若不刷成 best 的值, 用的是上一次 SEARCH_TRACK 锁的 cluster 时间
        //   (e.g. 16.71s 前 stale), triggerStrike 会 abort, 之后 processSearchStrikeStrike
        //   仍 publishPrimaryTarget + 发 type=1, GS 误以为 strike 成功, 但 UAV 没真飞。
        //   同步后 triggerStrike 看到 best 的 last_seen_us, deferred-pick 已经过滤了
        //   !is_strike_stale, 所以一定 fresh, 不再 abort。
        //   原则: 尽量打对的 (best 是 deferred-pick 选出来的, label_ok + fresh + 距 WP2 最近)
        locked_cluster_ned_x_         = cluster.ned_x;
        locked_cluster_ned_y_         = cluster.ned_y;
        locked_cluster_ned_alt_       = cluster.ned_alt;
        locked_cluster_id_            = cluster.cluster_id;
        locked_cluster_time_          = ros::Time::now();
        locked_cluster_last_seen_us_  = cluster.last_seen_us;

        // === Attack(105) formation=1/2:strike gating ===
        //   sequential:等前一架 partner 进 GUIDANCE_STRIKE
        //   coordinated:等到全局 strike_sync_time_
        //   默认 / formation=0:无门控,走原路径
        // 不通过时:publish type=3 替代 type=1,记 attack_strike_pending_ 留待后续重试
        auto gating = checkAttackStrikeGating(sr);
        if (gating == AttackStrikeGating::WAITING) {
            multi_uav_strike::DetectTarget dt_w = buildDetectTarget(cluster, /*target_type=*/3);
            detect_target_pub_.publish(dt_w);
            {
                multi_uav_strike::DetectTargets batch;
                batch.targets.push_back(dt_w);
                detect_targets_pub_.publish(batch);
            }
            reported_to_gs_[cluster.cluster_id] = {cluster.label, cluster.ned_x, cluster.ned_y};
            attack_strike_pending_             = true;
            attack_strike_pending_cluster_id_  = cluster.cluster_id;
            attack_strike_pending_time_        = ros::Time::now();
            publishPrimaryTarget(cluster.cluster_id);
            ROS_WARN_THROTTLE(2.0,
                "[MissionManager] Cluster[%lu] label=%s Attack(105) strike GATED "
                "(mode=%u, prev=%s, sync=%.2f) — waiting",
                cluster.cluster_id, cluster.label.c_str(),
                static_cast<unsigned>(sr.msg.formation),
                sr.attack_seq_prev_sn_.c_str(),
                sr.attack_strike_sync_time_.toSec());
            return;
        }

        // 门控通过或非 Attack(105) → 正常 strike
        detect_target_pub_.publish(dt);
        {
            multi_uav_strike::DetectTargets batch;
            batch.targets.push_back(dt);
            detect_targets_pub_.publish(batch);
        }
        reported_to_gs_[cluster.cluster_id]    = {cluster.label, cluster.ned_x, cluster.ned_y};
        strike_dispatched_[cluster.cluster_id] = 1;
        // task_speed=0(GS 漏发 / 老 Skill 模板默认值)时会被 guidance_control
        //   报"non-positive uav_speed"忽略,plane 不动,改成 std::max 兜底,见
        //   min_guidance_speed_ 字段注释(2026-07-26)
        //   Skill.msg task_speed 是 float32,需 cast 到 double 才能 std::max
        guidance_speed_ = std::max(static_cast<double>(sr.msg.task_speed), min_guidance_speed_);
        // 门控通过的 strike:清掉挂起标记,避免后续 advanceAttackPhaseMachine 重复触发
        attack_strike_pending_             = false;
        attack_strike_pending_cluster_id_  = 0;
        triggerStrike();
        // === 多目标: 通知 estimator 主目标 cluster_id ===
        publishPrimaryTarget(cluster.cluster_id);
        ROS_WARN("[MissionManager] Cluster[%lu] label=%s SEARCH_STRIKE → "
                 "strike + published type=1 (primary)",
                 cluster.cluster_id, cluster.label.c_str());
    }

    /**
     * 通知 target_estimator 当前主目标 cluster_id。
     * 0 = 未指定,estimator 回退最高置信度。
     */
    void publishPrimaryTarget(uint64_t cluster_id) {
        std_msgs::UInt64 msg;
        msg.data = cluster_id;
        primary_target_pub_.publish(msg);
    }

    /**
     * 1Hz 节流清理 reported_to_gs_ / strike_dispatched_ 过期项
     * TTL = ignored_retention_sec_(默认 20s)— 与 ignored_targets_ 对齐
     * 实际语义:同一 cluster_id 在 20s 内只上报一次;20s 后允许重新上报
     * (正常情况下 cluster_ttl 在 target_estimator 是 60s,这里 20s 已经足够防止快速重复)
     */
    void cleanupReportedClusters() {
        ros::Time now = ros::Time::now();
        if ((now - last_reported_cleanup_).toSec() < 1.0) return;
        last_reported_cleanup_ = now;
        // 注:reported_to_gs_ / strike_dispatched_ 暂不主动清(键只是 cluster_id,占用很小);
        //     当新 TaskFlow 到达时 taskFlowCallback 会整体清空。stale gate 单独
        //     erase strike_dispatched_(但不动 reported_to_gs_)—见 clusterStatesCallback 注释。
        // 此处函数保留为空 hook,以便未来需要 TTL 时启用。
    }

    /**
     * 从 ClusterTarget 构造 DetectTarget
     * 调用方:
     *   - clusterStatesCallback:按 work_mode 派发时(SEARCH_ONLY 上报 / SEARCH_TRACK 缓存 / SEARCH_STRIKE 上报+strike)
     *   - 不再被 attackCmdCallback 调用(后者直接复用 tracked_target_.latest)
     *
     * 目标 GPS 来源优先级:
     *   1. current_target_.gps (来自 target_estimated_gps,target_estimator 实时发布 best cluster 的 GPS)
     *   2. cluster.ned_x/y/z 通过 nedToGps 转 WGS84 (需要 ref_initialized_=true)
     *   3. 设备 GPS 退化(obj_lat = dev_lat, obj_alt = 0)— 临时占位
     *
     * img_data:ClusterTarget 不携带图像,故 img_data 留空
     *   (原 typedYoloCallback 会从 YoloDetection.img_data 透传 JPEG,
     *    cluster 链路下图像通过 detection/yolo_result 单独订阅,目前未接入)
     */
    /**
     * 检测端字符串 label → 协议类别 ID(DetectTarget.label,见协议 §11.2.1)。
     * GCS 按此 ID 渲染目标(1=人,2=车)。大小写不敏感,含常见同义词。
     * 映射不到的 label(如仿真占位 "general_target")退化为 unknown_label_id_。
     * 类别扩充时在此表追加即可。
     */
    uint32_t labelToClassId(const std::string& label) const {
        std::string s;
        s.reserve(label.size());
        for (char c : label) s.push_back(static_cast<char>(std::tolower((unsigned char)c)));

        // 1 = 人
        if (s == "person" || s == "people" || s == "human" || s == "pedestrian" || s == "人")
            return 1u;
        // 2 = 车
        if (s == "car" || s == "vehicle" || s == "truck" || s == "bus" || s == "van" ||
            s == "车" || s == "汽车")
            return 2u;

        return static_cast<uint32_t>(unknown_label_id_);
    }

    multi_uav_strike::DetectTarget buildDetectTarget(
        const multi_uav_strike::ClusterTarget& cluster,
        uint8_t target_type = 3) {
        multi_uav_strike::DetectTarget dt;
        dt.timestamp_us = cluster.last_seen_us;
        dt.label        = labelToClassId(cluster.label);  // 协议类别 ID(1=人/2=车),GCS 据此渲染
        dt.confidence   = cluster.confidence;
        dt.target_type  = target_type;

        // 设备 GPS — 优先 mavros/global_position/global (WGS84 lat/lon, AMSL alt)
        if (is_gps_received_) {
            dt.dev_lat = current_gps_.latitude;
            dt.dev_lon = current_gps_.longitude;
            dt.dev_alt = current_gps_.altitude;
        } else {
            static bool warned_no_gps = false;
            if (!warned_no_gps) {
                ROS_WARN_THROTTLE(10.0, "[MissionManager] DetectTarget.dev_lat/lon/alt = 0 "
                                        "(no GPS lock yet; will not republish this warn)");
                warned_no_gps = true;
            }
            dt.dev_lat = 0.0;
            dt.dev_lon = 0.0;
            dt.dev_alt = 0.0;
        }

        // 从 current_pose (NWU) 四元数解算偏航角,协议 §9.3 要求
        {
            const auto& q = current_pose_.pose.orientation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            dt.dev_yaw = std::atan2(siny_cosp, cosy_cosp);
        }

        // 目标 GPS — 优先 cluster 自身的 NED 转 GPS(2026-07-29:不再优先
        //   current_target_.gps,该字段是 target_estimated_gps 的缓存,
        //   而 target_estimated_gps 在 target_estimator_node.cpp:498-518 只发
        //   PRIMARY 一个 cluster 的 GPS。clusterStatesCallback 在 10Hz 快照里
        //   遍历所有 promoted cluster,若优先用 current_target_.gps,
        //   非 primary 全部拿到 PRIMARY 的旧坐标 → GS 看到 N 个 cluster 同一坐标)
        //   current_target_.gps 仅在 ref_initialized_=false(罕见,home 未锁)时兜底
        if (ref_initialized_) {
            nedToGps(cluster.ned_x, cluster.ned_y, cluster.ned_alt,
                     dt.obj_lat, dt.obj_lon, dt.obj_alt);
        } else if (current_target_.has_gps) {
            dt.obj_lat = current_target_.gps.latitude;
            dt.obj_lon = current_target_.gps.longitude;
            dt.obj_alt = current_target_.gps.altitude;
        } else {
            dt.obj_lat = dt.dev_lat;
            dt.obj_lon = dt.dev_lon;
            dt.obj_alt = 0.0;
        }

        // 图像:ClusterTarget 已透传最新一帧 YoloDetection.img_data,
        //   这里直接拷贝;img_format 与源对齐(1 = JPEG)
        dt.img_format = cluster.img_format;
        dt.img_data   = cluster.img_data;
        return dt;
    }

    /**
     * (已删除) 原 checkYoloDrivenStrike — 由 clusterStatesCallback 在 10Hz 快照扫描中
     *   直接完成 SEARCH_STRIKE/SEARCH_TRACK 的 strike/lock 决策。
     *   见 clusterStatesCallback 的门控说明。
     */

    /**
     * 判断给定 label + 位置的 cluster 是否在 ignored_targets_ 黑名单内
     * 匹配规则(label + 位置, 2026-07-16 改造):
     *   1. label 必须完全相等(string equality)
     *   2. ignore_until > now(未过期)
     *   3. (cluster.ned_x, cluster.ned_y) 与黑名单项 (ned_x, ned_y) 的水平距离
     *      ≤ 该黑名单项的 radius (按类别:人=10m, 车=20m)
     * 目的:只屏蔽 GS action=1/2 指定的"那个物理目标",同 label 的其它物理目标
     *   仍可被锁定/上报。修复前 label-only 方案会把同一 label 的所有目标屏蔽
     *   (例如 action=2 暂存目标 1 后,目标 2 也被忽略,即使两者距离 15m+)。
     * 副作用:每次调用顺手清理过期条目(throttle 1Hz,避免每帧都遍历)
     */
    bool isTargetIgnored(const std::string& label, double ned_x, double ned_y) {
        ros::Time now = ros::Time::now();
        // 节流清理过期项:1Hz 一次足够(ignore_until 精度秒级)
        if ((now - last_ignored_cleanup_).toSec() > 1.0) {
            auto it = ignored_targets_.begin();
            while (it != ignored_targets_.end()) {
                if (now >= it->ignore_until) {
                    ROS_INFO("[MissionManager] Ignored blacklist expired: label=%s "
                             "(alive %.1fs)",
                             it->label.c_str(),
                             (now - it->ignore_set_time).toSec());
                    it = ignored_targets_.erase(it);
                } else {
                    ++it;
                }
            }
            last_ignored_cleanup_ = now;
        }
        for (const auto& it : ignored_targets_) {
            if (it.label != label) continue;
            if (now >= it.ignore_until) continue;
            // 水平距离平方 ≤ 半径平方(避免 sqrt)
            double dx = ned_x - it.ned_x;
            double dy = ned_y - it.ned_y;
            double r2 = it.radius * it.radius;
            if (dx * dx + dy * dy <= r2) {
                return true;  // label 相等 + 未过期 + 位置接近 → 忽略
            }
        }
        return false;
    }

    /**
     * 推一条目标到 ignored_targets_ 黑名单
     * 由 attackCmdCallback 在 action=1/2 时调用
     * @param ned_x/ned_y/ned_alt  锁定瞬间物理目标的 NWU 坐标(米),用于后续位置匹配
     * @param yolo_label 锁定瞬间 YOLO 原始 string label
     *
     * 屏蔽半径按 label 类别自动取值:
     *   - person 类 → ignore_radius_person_m_  (默认 10m)
     *   - vehicle 类 → ignore_radius_car_m_     (默认 20m)
     *   - 其它        → ignore_radius_default_m_ (默认 10m)
     * 避免同一 label 的不同物理目标(例如两个 person 距离 15m+)被一起屏蔽。
     */
    void addIgnoredTarget(double ned_x, double ned_y, double ned_alt,
                          const std::string& yolo_label) {
        IgnoredTarget entry;
        entry.ned_x           = ned_x;
        entry.ned_y           = ned_y;
        entry.ned_alt         = ned_alt;
        entry.label           = yolo_label;
        entry.radius          = getIgnoreRadiusForLabel(yolo_label);
        entry.ignore_set_time = ros::Time::now();
        entry.ignore_until    = entry.ignore_set_time + ros::Duration(ignored_retention_sec_);
        ignored_targets_.push_back(entry);
        ROS_WARN("[MissionManager] Added to IGNORED blacklist (label=%s pos=(%.2f,%.2f,%.2f) "
                 "radius=%.1fm ttl=%.1fs, current_size=%zu)",
                 entry.label.c_str(), entry.ned_x, entry.ned_y, entry.ned_alt,
                 entry.radius, ignored_retention_sec_, ignored_targets_.size());
    }

    /**
     * 按 YOLO label 返回屏蔽半径(米)
     * 大小写不敏感,匹配规则:
     *   - "person" / "human" / "pedestrian"  → ignore_radius_person_m_ (10m)
     *   - "car" / "vehicle" / "truck" / "bus" / "van" → ignore_radius_car_m_ (20m)
     *   - 其它        → ignore_radius_default_m_ (10m)
     */
    double getIgnoreRadiusForLabel(const std::string& label) const {
        std::string lower = label;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        if (lower.find("person")    != std::string::npos ||
            lower.find("human")     != std::string::npos ||
            lower.find("pedestrian")!= std::string::npos) {
            return ignore_radius_person_m_;
        }
        if (lower.find("car")     != std::string::npos ||
            lower.find("vehicle") != std::string::npos ||
            lower.find("truck")   != std::string::npos ||
            lower.find("bus")     != std::string::npos ||
            lower.find("van")     != std::string::npos) {
            return ignore_radius_car_m_;
        }
        return ignore_radius_default_m_;
    }

    /**
     * WaypointStatus typed 回调(Phase 4 启用)
     * 仅做缓存,advanceSkillStateMachine() 用 latest_wp_status_ 推进 SkillState
     */
    void waypointStatusCallback(const multi_uav_strike::WaypointStatus::ConstPtr& msg) {
        latest_wp_status_ = msg;
        latest_wp_status_time_ = ros::Time::now();
    }

    /**
     * 多机心跳回调 — 多机 gather 同步用
     *   - 仅缓存本机(由 nh_.getNamespace() 决定)以外的 SN
     *   - 缓存带时间戳,skill_advance_timer 周期清理超时
     *   - === 2026-07-24: 当前 skill 是 gather(101)且伙伴 phase=HOLDING,标记一次性 seen ===
     *     解决"先到 2 架走了,phase 切到下一阶段,最后一架永远等不到所有人 HOLDING"的问题。
     * 注:回调里不做"是否到齐"判断,统一在 skillAdvanceTimerCallback IN_TASK 分支
     *   调 checkAllUavsGathered() 做实时快照,避免竞争
     */
    void interUavStatusCallback(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        std::string my_ns = nh_.getNamespace();
        if (!my_ns.empty() && my_ns[0] == '/') my_ns = my_ns.substr(1);
        if (msg->sn == my_ns) {
            return;  // 忽略自己的
        }
        partner_heartbeats_[msg->sn].latest      = msg;
        partner_heartbeats_[msg->sn].last_update = ros::Time::now();

        // === 2026-07-24: gather 屏障 — 标记"已见过此伙伴 HOLDING" ===
        // 当前任务运行时(skill_queue_[current_skill_index_] 有效)+ skill_type==101
        // + 伙伴当前 phase==HOLDING → 写入 gather_partner_holding_seen_,只写一次
        // (map::insert 不覆盖)。partner_heartbeat_timeout_ 在 checkAllUavsGathered()
        //   里用于 fresh 检测,这里不做额外时间窗判定(本机已在 HOLDING 等待
        //   partner_heartbeat_timeout_ 时间窗,伙伴心跳进入窗口前不会调这里)。
        if (current_skill_index_ >= skill_queue_.size()) {
            return;
        }
        SkillRuntime& sr_ref = skill_queue_[current_skill_index_];
        if (sr_ref.msg.skill_type != 101) {
            return;  // 当前不是 gather skill
        }
        if (sr_ref.gather_enter_time.isZero()) {
            return;  // 还没真正进入 gather(可能还在 ENTRY_PENDING)
        }
        // 伙伴当前 phase 是否 HOLDING
        if (msg->phase != static_cast<uint8_t>(MissionPhase::PHASE_HOLDING)) {
            return;
        }
        // 写入(只写一次,后续不再更新 — 一旦见过就永久算"到过")
        auto ins = sr_ref.gather_partner_holding_seen_.insert(
            {msg->sn, ros::Time::now()});
        if (ins.second) {
            ROS_WARN("[MissionManager] Skill[101] Partner %s acknowledged HOLDING (seen=%zu/%zu)",
                     msg->sn.c_str(),
                     sr_ref.gather_partner_holding_seen_.size(),
                     sr_ref.gather_partners_.size());
        }
    }

    /**
     * 从 Skill.params_json 解析 "devices_sn":["uav0","uav1",...]
     *   - 简单子串提取,无 nlohmann/json 依赖
     *   - 容错:解析失败或字段缺失 → 返回空 vector(→ 单机路径)
     *   - 取出的字符串会 strip 首尾空白
     */
    std::vector<std::string> parseDevicesSnFromParamsJson(const std::string& json_str) {
        std::vector<std::string> result;
        const std::string key = "\"devices_sn\"";
        size_t key_pos = json_str.find(key);
        if (key_pos == std::string::npos) return result;
        size_t arr_start = json_str.find('[', key_pos);
        if (arr_start == std::string::npos) return result;
        size_t arr_end = json_str.find(']', arr_start);
        if (arr_end == std::string::npos) return result;
        std::string arr = json_str.substr(arr_start + 1, arr_end - arr_start - 1);
        size_t pos = 0;
        while (pos < arr.size()) {
            while (pos < arr.size() &&
                   (arr[pos] == ' ' || arr[pos] == '\t' || arr[pos] == '\n' ||
                    arr[pos] == '\r' || arr[pos] == ',')) {
                ++pos;
            }
            if (pos >= arr.size() || arr[pos] != '"') break;
            ++pos;
            size_t end_quote = arr.find('"', pos);
            if (end_quote == std::string::npos) break;
            std::string item = arr.substr(pos, end_quote - pos);
            // strip 前后空白
            size_t a = 0, b = item.size();
            while (a < b && (item[a] == ' ' || item[a] == '\t')) ++a;
            while (b > a && (item[b-1] == ' ' || item[b-1] == '\t')) --b;
            if (b > a) result.push_back(item.substr(a, b - a));
            pos = end_quote + 1;
        }
        return result;
    }

    // ============== 定时器回调 ==============

    void missionTimerCallback(const ros::TimerEvent&) {
        if (!is_pose_received_) {
            return;
        }

        // PX4 SITL: 运行起飞状态机
        if (!use_sim_) {
            runPx4TakeoffSequence();
            if (!isTakeoffComplete()) {
                // 起飞未完成，跳过任务执行
                return;
            }
            // === 返航落地监控 (必须在 isTakeoffComplete 早 return 之后调用,避免落地检查被卡) ===
            // 正常情况下 AUTO.LAND 流程中 takeoff_state_ 仍是 COMPLETE,这里能跑到
            checkLandingComplete();
        }

        // 检查急停条件（毫米波雷达）
        checkEmergencyStop();

        // 根据工作模式执行对应行为
        // 注意: takeoff 流程由 runPx4TakeoffSequence() 推进,mission_loop 不再处理
        switch (current_work_mode_) {
            case WorkMode::IDLE:
                handleIdle();
                break;
            case WorkMode::SEARCH_ONLY:
                handleSearchOnly();
                break;
            case WorkMode::SEARCH_TRACK:
                handleSearchTrack();
                break;
            case WorkMode::SEARCH_STRIKE:
                handleSearchStrike();
                break;
            case WorkMode::DENIED_ENV_FLIGHT:
                handleDeniedEnvFlight();
                break;
        }

        // 发布状态
        publishStatus();
    }

    // PX4 SITL: 执行起飞状态机
    void runPx4TakeoffSequence() {
        // 关键门控：只有 PHASE_TAKING_OFF 或 PHASE_HOVERING 才推进起飞流程（避免 GROUND_IDLE 时 FCU 一连上就自动起飞）。
        // 注:HOLDING 也豁免 — enterHoldState() 需要 setpoint publisher 持续发 OFFBOARD setpoint 来稳悬停
        //    (taskFlowCallback 会在新 flow 到达时显式 stop, 这里早 return 不会漏停)
        if (current_phase_ != MissionPhase::PHASE_TAKING_OFF &&
            current_phase_ != MissionPhase::PHASE_HOVERING &&
            current_phase_ != MissionPhase::PHASE_HOLDING) {
            // 切出 TAKING_OFF 时：停止 setpoint 发布器（避免持续发送位置指令覆盖其他模块）
            //   例外:Attack(105) hover 阶段 phase 故意保留为 WAYPOINT_FOLLOW(不让 GS 看到
            //     phase 跳变),但 mission_manager 主动 latch 了 setpoint publisher 锁 XY+Z
            //     在 attack_point,一旦停掉 PX4 OFFBOARD 失流 → COM_OFFBOARD_LOSS_TIMEOUT →
            //     AUTO.LAND failsafe(2026-07-26 修复:用 attack_hover_setpoint_latched_ 标记
            //     豁免,Attack EXIT_PENDING 路径会清标志并停 publisher)。
            if (setpoint_running_ && !attack_hover_setpoint_latched_) {
                stopSetpointPublisher();
            }
            // 注意:不再重置 takeoff_state_。phase=WAYPOINT_FOLLOW/HOLDING 时 takeoff_state_=COMPLETE
            // 是 missionTimer 主循环能继续跑航点跟踪/悬停的前提, 不能清。
            return;
        }

        switch (takeoff_state_) {
            case TakeoffState::TAKEOFF_IDLE:
                // 已在 TAKEOFF 模式：等待 FCU 连接后开始
                if (!use_sim_ && is_px4_connected_) {
                    // 弹射起飞(skill_type=100)与地面起飞(skill_type=106)分流:
                    //   弹射:不能直接切 OFFBOARD,先让 PX4 进抛飞模式等外部 trigger
                    //   地面:走原流程,FCU 连上就 SET_MODE → COMMAND_LONG → ARM
                    if (is_catapult_takeoff_) {
                        takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_ARMED;
                        ROS_WARN("[MissionManager] PX4 Connected, CATAPULT takeoff flow — "
                                 "waiting for catapult_mode_ready=%s + catapult_trigger",
                                 is_px4_catapult_mode_ ? "true" : "false (need comm_node)");
                    } else {
                        takeoff_state_ = TakeoffState::TAKEOFF_WAITING_FCU;
                        ROS_WARN("[MissionManager] PX4 Connected, waiting for initialization...");
                    }
                }
                break;

            case TakeoffState::TAKEOFF_CATAPULT_ARMED:
                // 弹射起飞第一步:等 PX4 抛飞模式就绪 (comm_node 通过 px4/catapult_mode_ready 通知)
                //                  + 外部 trigger (mission/catapult_trigger)
                // 必须两者都到,因为:
                //   - 只到 PX4 模式没 trigger:不会发射, UAV 静等 → 浪费
                //   - 只到 trigger 没 PX4 模式:外部发弹射但 PX4 没准备好,危险
                if (!is_px4_catapult_mode_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT_ARMED: waiting for PX4 catapult "
                                          "mode (comm_node sets it via px4/catapult_mode_ready)");
                    break;
                }
                if (!catapult_trigger_received_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT_ARMED: PX4 catapult mode ready, "
                                          "waiting for catapult_trigger from external node");
                    break;
                }
                // 两个都齐了 — 记录起始时间,准备监视 POSCTL 进入
                catapult_posctl_enter_time_ = ros::Time();
                takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_TRIGGERED;
                ROS_WARN("[MissionManager] >>>> CATAPULT armed+triggered, watching PX4 enter POSCTL "
                         "(commander will switch to POSCTL after launch detects sustained climb)...");
                break;

            case TakeoffState::TAKEOFF_CATAPULT_TRIGGERED: {
                // 弹射起飞第二步:等 PX4 自动切到 POSCTL(发射后 PX4 自主控制并进入位置模式)
                // 通过 mavros/state 反馈,current_mavros_state_.mode == "POSCTL" 视为进入
                // 兜底超时:60s 内没进 POSCTL 视为发射异常,转 FAILED
                if (current_mavros_state_.mode == "POSCTL") {
                    catapult_posctl_enter_time_ = ros::Time::now();
                    takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_POSCTL;
                    ROS_WARN("[MissionManager] >>>> CATAPULT: PX4 entered POSCTL (mode=%s, armed=%d)",
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0);
                } else {
                    double since_trigger = catapult_posctl_enter_time_.isZero()
                        ? (ros::Time::now() - (ros::Time::now() - ros::Duration(catapult_posctl_stable_sec_))).toSec()
                        : 0.0;
                    // 注意:catapult_posctl_enter_time_ 此时 isZero (TRIGGERED 进入前清零),
                    // 用 takeoff_start_time_ (Phase=TAKING_OFF 时刻) 估算经过时间更稳
                    static ros::Time catapult_trigger_armed_time;  // 静态变量,记录 armed→triggered 的过渡
                    if (catapult_posctl_enter_time_.isZero() && catapult_trigger_armed_time.isZero()) {
                        catapult_trigger_armed_time = ros::Time::now();
                    }
                    double elapsed_in_triggered = (ros::Time::now() - catapult_trigger_armed_time).toSec();
                    if (elapsed_in_triggered > 60.0) {
                        ROS_ERROR("[MissionManager] CATAPULT: PX4 did not enter POSCTL within 60s, "
                                  "FAILED (current mode=%s)",
                                  current_mavros_state_.mode.c_str());
                        catapult_trigger_armed_time = ros::Time();  // 重置
                        takeoff_state_ = TakeoffState::TAKEOFF_FAILED;
                        current_phase_ = MissionPhase::PHASE_FAILED;
                    } else {
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT: waiting for PX4 POSCTL "
                                              "(current mode=%s, %.1fs/60s)",
                                          current_mavros_state_.mode.c_str(), elapsed_in_triggered);
                    }
                    (void)since_trigger;  // 抑制未使用变量警告
                }
                break;
            }

            case TakeoffState::TAKEOFF_CATAPULT_POSCTL: {
                // 弹射起飞第三步:PX4 POSCTL 稳定 catapult_posctl_stable_sec_(1s) 后
                //                启动 setpoint publisher + 切 OFFBOARD 接管
                // 此处复用 TAKEOFF_WAITING_FCU 的 LOCAL_POSITION_NED rate 设置逻辑
                if (is_local_position_rate_set_ == false) {
                    if (set_message_rate_start_time_.isZero()) {
                        set_message_rate_start_time_ = ros::Time::now();
                    }
                    if (setMessageRate(32, local_position_rate_hz_)) {
                        is_local_position_rate_set_ = true;
                        set_message_rate_start_time_ = ros::Time();
                        set_message_rate_retry_count_ = 0;
                    } else {
                        set_message_rate_retry_count_++;
                        double elapsed = (ros::Time::now() - set_message_rate_start_time_).toSec();
                        bool retries_done = (set_message_rate_retry_count_ >= max_set_message_rate_retries_);
                        bool timeout = (elapsed > set_message_rate_max_wait_sec_);
                        if (retries_done || timeout) {
                            ROS_ERROR("[MissionManager] CATAPULT: LOCAL_POSITION_NED rate set GIVE UP — "
                                      "proceeding anyway");
                            is_local_position_rate_set_ = true;
                            set_message_rate_start_time_ = ros::Time();
                            set_message_rate_retry_count_ = 0;
                        } else {
                            break;  // 下个 tick 再试
                        }
                    }
                }
                // POSCTL 稳定等待(系统不抖后再接管)
                double posctl_elapsed = (ros::Time::now() - catapult_posctl_enter_time_).toSec();
                if (posctl_elapsed < catapult_posctl_stable_sec_) {
                    ROS_WARN_THROTTLE(0.3, "[MissionManager] CATAPULT: POSCTL stable wait %.2fs/%.2fs",
                                      posctl_elapsed, catapult_posctl_stable_sec_);
                    break;
                }
                // 稳定时间到 — 进 SETTING_OFFBOARD 接管
                ROS_WARN("[MissionManager] >>>> CATAPULT: POSCTL stable, switching to OFFBOARD takeover");
                takeoff_state_ = TakeoffState::TAKEOFF_SETTING_OFFBOARD;
                startSetpointPublisher();
                break;
            }

            case TakeoffState::TAKEOFF_WAITING_FCU:
                if (!is_px4_connected_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Waiting for PX4 FCU connection...");
                    break;
                }
                // FCU 已连, 但还没调过 LOCAL_POSITION_NED 频率 — 调一下
                if (!is_local_position_rate_set_) {
                    // 第一次进这分支时记录起始时间 (用 isZero 判定, 避免 set_now 重复覆盖)
                    if (set_message_rate_start_time_.isZero()) {
                        set_message_rate_start_time_ = ros::Time::now();
                    }
                    if (setMessageRate(32, local_position_rate_hz_)) {
                        // 成功 — 标记完成, 进入下一步
                        is_local_position_rate_set_ = true;
                        ROS_WARN("[MissionManager] >>>>> PX4 LOCAL_POSITION_NED rate set to %.1f Hz "
                                 "(attempt %d, elapsed %.2fs)",
                                 local_position_rate_hz_,
                                 set_message_rate_retry_count_ + 1,
                                 (ros::Time::now() - set_message_rate_start_time_).toSec());
                        set_message_rate_start_time_ = ros::Time();  // 清零, 下次 FCU 重连时复用
                        set_message_rate_retry_count_ = 0;
                        // fall through 进 SETTING_OFFBOARD
                    } else {
                        set_message_rate_retry_count_++;
                        double elapsed = (ros::Time::now() - set_message_rate_start_time_).toSec();
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] >>>>> LOCAL_POSITION_NED rate set FAILED "
                                              "(attempt %d/%d, elapsed %.1fs/%.1fs)",
                                              set_message_rate_retry_count_, max_set_message_rate_retries_,
                                              elapsed, set_message_rate_max_wait_sec_);
                        // 判定放弃条件: 重试次数耗尽 OR 总等待时间超上限
                        bool retries_done = (set_message_rate_retry_count_ >= max_set_message_rate_retries_);
                        bool timeout = (elapsed > set_message_rate_max_wait_sec_);
                        if (retries_done || timeout) {
                            ROS_ERROR("[MissionManager] >>>>> LOCAL_POSITION_NED rate set GIVE UP "
                                      "(%s after %d attempts / %.1fs). Proceeding with PX4 default rate — "
                                      "UAV may oscillate in OFFBOARD. Check PX4 MAVLink config or reboot FCU.",
                                      retries_done ? "retries exhausted" : "timeout",
                                      set_message_rate_retry_count_, elapsed);
                            // 强制标记完成, 放行进 OFFBOARD; flag 重置交给 FCU 断连时处理
                            is_local_position_rate_set_ = true;
                            set_message_rate_start_time_ = ros::Time();
                            set_message_rate_retry_count_ = 0;
                            // fall through 进 SETTING_OFFBOARD
                        } else {
                            // 还没放弃 — 下个 tick 再试
                            break;
                        }
                    }
                }
                // 通过到这里说明已经成功 (或放弃), 进 SETTING_OFFBOARD
                takeoff_state_ = TakeoffState::TAKEOFF_SETTING_OFFBOARD;
                startSetpointPublisher();
                break;

            case TakeoffState::TAKEOFF_SETTING_OFFBOARD: {
                // ===== 关键: setpoint publisher 起来后稳定流 ≥ 150ms 再触发 OFFBOARD+ARM 序列 =====
                // PX4 要求最近 COM_OFFBOARD_LOSS_TIMEOUT (默认 0.5s) 内有连续 setpoint
                // 才接受 OFFBOARD 切换。150ms 已经足够安全。
                double sp_elapsed = (ros::Time::now() - setpoint_start_time_).toSec();
                if (sp_elapsed < 0.15) {
                    ROS_WARN_THROTTLE(0.5, "[MissionManager] Pre-warming setpoint stream: %.0f ms / 150 ms (count=%d)",
                                      sp_elapsed * 1000.0, setpoint_publish_count_);
                    break;
                }
                // 一次性快速序列: SET_MODE → COMMAND_LONG → ARM，不等 mode 确认。
                // 模仿 pymavlink 的 50ms 内连发 SET_MODE+ARM 的行为 — 这是它在同一架
                // FCU 上能跑通的关键时机 (PX4 commander 100~500ms 内会 revert OFFBOARD)。
                if (triggerOffboardAndArm()) {
                    takeoff_state_ = TakeoffState::TAKEOFF_TAKEOFF_EXEC;
                    takeoff_start_time_ = ros::Time::now();

                    // ===== 决定本次起飞爬升速率 =====
                    // 优先取当前 takeoff skill 的 Skill.takeoff_speed(GS 在 task_flow
                    // 里下发),对 skill_type=100(弹射)/106(地面)均生效。夹到 [1, 5] m/s:
                    //   - 下限 1:避免风大/弹射后初始姿态不稳时爬升过慢、过渡时间过长
                    //   - 上限 5:PX4 MPC_VEL_UP_MAX(~5 m/s)安全区,避免速度饱和+overshoot
                    // skill 字段 <= 0 或当前 skill 非起飞 → 退化为私有 param ~takeoff_climb_rate_
                    constexpr double MIN_CLIMB = 1.0;
                    constexpr double MAX_CLIMB = 5.0;
                    double effective_climb = takeoff_climb_rate_;  // 默认兜底
                    if (current_skill_index_ < skill_queue_.size()) {
                        const auto& tk = skill_queue_[current_skill_index_].msg;
                        if (tk.skill_type == 100 || tk.skill_type == 106) {
                            if (tk.takeoff_speed > 0.0f) {
                                double s = static_cast<double>(tk.takeoff_speed);
                                if (s < MIN_CLIMB) s = MIN_CLIMB;
                                if (s > MAX_CLIMB) s = MAX_CLIMB;
                                effective_climb = s;
                                ROS_INFO("[MissionManager] Takeoff climb rate from skill.takeoff_speed=%.2f "
                                         "(clamped to [%.1f, %.1f] → %.2f m/s)",
                                         tk.takeoff_speed, MIN_CLIMB, MAX_CLIMB, effective_climb);
                            }
                        }
                    }
                    takeoff_climb_rate_ = effective_climb;  // 后续 TAKEOFF_TAKEOFF_EXEC 每 tick 用此值斜坡推

                    // ===== 不再阶跃跳到目标高度 =====
                    // 原因: 0.5m → 30m 的阶跃会让 PX4 速度饱和(MPC_VEL_UP_MAX),真机上引起
                    //       40m 级别 overshoot,然后快速下落,衔接 waypoint 时有明显下沉。
                    // 改为: 记录当前实际高度作为斜坡起点,每个 tick 按 takeoff_climb_rate_
                    //       线性推高,直到达到 takeoff_altitude_。PX4 在斜坡模式下稳态跟踪,
                    //       几乎不超调。
                    takeoff_start_alt_ = -current_pose_.pose.position.z;
                    takeoff_setpoint_.pose.position.z = takeoff_start_alt_;
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM SUCCESS, takeoff climb started "
                             "(sp=%.0fms, target alt=%.1f m, climb_rate=%.1f m/s, start_alt=%.2f m)",
                             sp_elapsed * 1000.0, takeoff_altitude_,
                             takeoff_climb_rate_, takeoff_start_alt_);
                } else {
                    // 不进 FAILED 状态，下一 tick 会再走一遍 SETTING_OFFBOARD 重试整组动作
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM sequence FAILED (PX4 mode=%s armed=%d), "
                             "will retry next cycle",
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0);
                }
                break;
            }

            case TakeoffState::TAKEOFF_TAKEOFF_EXEC: {
                // ===== setpoint z 斜坡推进 =====
                // 每个 tick 按 takeoff_climb_rate_ 把目标高度往 takeoff_altitude_ 推,
                // 不再阶跃跳变。斜坡速度 3 m/s 默认,落在 PX4 MPC_VEL_UP_MAX(5 m/s)安全区内,
                // 真机不饱和、不超调。
                double elapsed_climb = (ros::Time::now() - takeoff_start_time_).toSec();
                double ramped_z = takeoff_start_alt_ + takeoff_climb_rate_ * elapsed_climb;
                if (ramped_z > takeoff_altitude_) {
                    ramped_z = takeoff_altitude_;
                }
                takeoff_setpoint_.pose.position.z = ramped_z;

                // 检查高度 (NED: z 向下为正)
                double current_alt = -current_pose_.pose.position.z;
                if (current_alt >= takeoff_altitude_ - 1.0f) {  // 高度容差 1m(保留:避免小机型无法精确达到目标卡死)
                    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                    if (elapsed > takeoff_stable_time_) {  // 高度稳定时间（ROS 参数）
                        takeoff_state_ = TakeoffState::TAKEOFF_HOVERING;
                        takeoff_start_time_ = ros::Time::now();
                        current_phase_ = MissionPhase::PHASE_HOVERING;  // phase 推进:TAKING_OFF → HOVERING
                        ROS_WARN("[MissionManager] Takeoff altitude reached, hovering (phase=HOVERING, ramped_z=%.2f)...",
                                 ramped_z);
                    }
                } else {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Takeoff climbing: cur=%.1f / tgt=%.1f m (ramp=%.2f)",
                                     current_alt, takeoff_altitude_, ramped_z);
                }

                // 超时检测
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > 60.0) {
                    ROS_ERROR("[MissionManager] Takeoff timeout!");
                    takeoff_state_ = TakeoffState::TAKEOFF_FAILED;
                    current_phase_ = MissionPhase::PHASE_FAILED;  // phase 推进:→ FAILED
                }
                break;
            }

            case TakeoffState::TAKEOFF_HOVERING: {
                // 悬停一段时间后进入任务
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > takeoff_hover_time_) {  // 悬停等待时间（ROS 参数）
                    // === Auto-handoff 判定 ===
                    // 真实 GS 场景下: 任务 = TAKEOFF + 航点列表, 期间 phase 推进 TAKING_OFF→HOVERING→WAYPOINT_FOLLOW
                    // 不能等用户手动切 work_mode(违反 SET_WORKMODE 协议语义),必须在这里自动交接给 waypoint_executor。
                    if (is_waypoints_received_) {
                        performTakeoffHandoff();
                        // performTakeoffHandoff 会:
                        //   1. 恢复 work_mode_before_takeoff_ (或回落到 ~post_takeoff_work_mode)
                        //   2. 立刻停 setpoint_thread (gap ~20ms < 500ms PX4 timeout)
                        //   3. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
                        //   4. current_phase_=PHASE_WAYPOINT_FOLLOW
                        // 下次 tick 时 runPx4TakeoffSequence 因为 phase!=TAKING_OFF 而 early-return,
                        // 不再进入 HOVERING 分支, 等价于原来的 is_takeoff_handoff_done_ 防重入效果。
                    } else {
                        // 还没收到航点, 保持 HOVERING, 让 setpoint_thread 持续发悬停点
                        // 保险: 避免在没航点的情况下交接, 导致 waypoint_executor 看到空队列报错
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] HOVERING waiting for waypoints "
                                              "(is_waypoints_received_=%d, waypoint_count=%zu)",
                                              is_waypoints_received_ ? 1 : 0, current_waypoint_count_);
                        // 重置 elapsed, 再多等一拍, 直到航点到达
                        takeoff_start_time_ = ros::Time::now();
                    }
                }
                break;
            }

            case TakeoffState::TAKEOFF_COMPLETE:
                // 起飞完成，等待任务模式
                break;

            case TakeoffState::TAKEOFF_FAILED: {
                if (!takeoff_failed_logged_) {
                    ROS_ERROR("[MissionManager] Takeoff failed! Retry %d / %d in %.1f s",
                              takeoff_retry_count_, max_takeoff_retries_, takeoff_retry_delay_);
                    takeoff_failed_time_ = ros::Time::now();
                    takeoff_failed_logged_ = true;
                }

                // 等待重试延迟后复位
                double failed_elapsed = (ros::Time::now() - takeoff_failed_time_).toSec();
                if (failed_elapsed < takeoff_retry_delay_) {
                    break;
                }

                if (takeoff_retry_count_ < max_takeoff_retries_) {
                    // 重试：复位到 IDLE 重新跑整个起飞流程
                    takeoff_retry_count_++;
                    takeoff_failed_logged_ = false;
                    takeoff_state_ = TakeoffState::TAKEOFF_IDLE;
                    ROS_WARN("[MissionManager] Retrying takeoff sequence (attempt %d / %d)...",
                             takeoff_retry_count_, max_takeoff_retries_);
                } else {
                    ROS_ERROR_THROTTLE(5.0, "[MissionManager] Takeoff failed after %d retries. Giving up.",
                                       max_takeoff_retries_);
                }
                break;
            }
        }
    }

    // === Auto-handoff: TAKEOFF_HOVERING 完成后, 自动切到 PHASE_WAYPOINT_FOLLOW ===
    // 触发条件: 高度达标 + hover 计时到 + 航点已下发 + phase 还在 TAKING_OFF/HOVERING
    // 流程:
    //   1. 恢复用户起飞前的工作模式(work_mode_before_takeoff_),
    //      若起飞前是 IDLE 则落到 ~post_takeoff_work_mode(默认 SEARCH_ONLY)
    //   2. waypoint_executor 已通过 typed Skill 收到新路径,无需再发 String mode
    //   3. 立即停 setpoint_thread (gap ~20ms, 远小于 PX4 COM_OFFBOARD_LOSS_TIMEOUT=500ms)
    //   4. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE (让 missionTimerCallback 继续跑)
    //   5. current_phase_=PHASE_WAYPOINT_FOLLOW (下次 tick runPx4TakeoffSequence early-return)
    void performTakeoffHandoff() {
        // 1. 恢复用户起飞前的工作模式,而不是硬切 SEARCH_ONLY
        //    场景:用户在地面站提前发过 SET_WORKMODE=SEARCH_TRACK,起飞 skill 不应覆盖用户意图
        //    兼容:如果起飞前是 IDLE(典型场景),回落到 ~post_takeoff_work_mode(默认 SEARCH_ONLY)
        WorkMode restore_mode = work_mode_before_takeoff_;
        if (restore_mode == WorkMode::IDLE) {
            restore_mode = post_takeoff_work_mode_;
        }
        // === 2026-07-26 fix: 不覆盖新 TaskFlow 首发 Attack(105) 已经 promote 的 SEARCH_STRIKE ===
        //   时序:advanceSkillStateMachine 先跑 (ENTRY_PENDING→IN_TASK → Attack(105) 路径在 4197 行
        //   把 current_work_mode_ 升到 SEARCH_STRIKE),然后 performTakeoffHandoff 才在同 tick 跑。
        //   旧逻辑会无条件 restore 到 work_mode_before_takeoff_,把 SEARCH_STRIKE 又打回 SEARCH_ONLY,
        //   之后 clusterStatesCallback 走 SEARCH_ONLY 分支,YOLO 命中只 type=3 上报、从不 strike。
        //   判定:新 TaskFlow 首发是 Attack(105) 且已被 promote → 跳过 restore(已经是正确语义)。
        bool attack_already_promoted = false;
        if (current_skill_index_ < skill_queue_.size() &&
            skill_queue_[current_skill_index_].msg.skill_type == 105 &&
            current_work_mode_ == WorkMode::SEARCH_STRIKE) {
            attack_already_promoted = true;
        }
        bool mode_restored = false;
        if (!attack_already_promoted && restore_mode != current_work_mode_) {
            current_work_mode_ = restore_mode;
            mode_restored = true;
        }
        work_mode_before_takeoff_ = WorkMode::IDLE;  // 一次性缓存,清空防误用
        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF: TAKEOFF -> WAYPOINT_FOLLOW "
                 "(waypoints=%zu, work_mode=%s%s%s) =====",
                 current_waypoint_count_,
                 workModeToString(current_work_mode_).c_str(),
                 mode_restored ? " [restored]" : "",
                 attack_already_promoted ? " [attack-promote preserved]" : "");
        // phase_before_takeoff_ 不在此处清,因为它将在 GROUND_IDLE → TAKING_OFF → HOVERING → WAYPOINT_FOLLOW 链路结束后
        // 由新 task_flow 的 entry 路径再次覆盖。这里清掉反而可能在重新进入 PHASE_TAKING_OFF 的瞬间丢失兜底。
        disableGuidance();  // 起飞后默认关闭制导,等 SEARCH_TRACK/STRIKE 再 enable

        // 3. waypoint_executor 已通过 typed Skill(由 pushCurrentSkillToExecutor
        //    把下一条 Skill 推到 /waypoint_executor/skill) 收到新路径,无需再发
        //    旧的 String mode 通知。

        // 4. 立刻停 setpoint_thread (此时 waypoint_executor 即将接管)
        if (setpoint_running_) {
            stopSetpointPublisher();
            ROS_WARN("[MissionManager]   - Takeoff setpoint publisher stopped");
        }
        
        // 5. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
        //    让 isTakeoffComplete()=true, missionTimerCallback 继续跑 mission 主循环
        takeoff_state_ = TakeoffState::TAKEOFF_COMPLETE;

        // 6. 显式把起飞 skill 标记为 COMPLETE,而不是等 advanceSkillStateMachine
        //    自己走完 TRANSIT→EXIT_PENDING→COMPLETE(那要 100ms 才推进,中间还有 50ms 计时)
        //    起飞 skill 没有 arrive_path/skill_area_path,EXIT_PENDING 阶段无意义,直接 COMPLETE 更清晰
        //    下次 skillAdvanceTimerCallback tick 就会走 COMPLETE 分支推下一个 skill
        if (current_skill_index_ < skill_queue_.size() &&
            (skill_queue_[current_skill_index_].msg.skill_type == 100 ||
             skill_queue_[current_skill_index_].msg.skill_type == 106)) {
            auto& sr = skill_queue_[current_skill_index_];
            const std::string prev_state_str = skillStateStr(sr.state);
            sr.state = SkillState::COMPLETE;
            sr.state_enter_time = ros::Time::now();
            sr.last_event = "handoff forced " + prev_state_str + " to COMPLETE";
            ROS_WARN("[MissionManager]   - Takeoff skill[%zu] id=%s  COMPLETE (forced by handoff, prev=%s)",
                     current_skill_index_, sr.msg.skill_id.c_str(), prev_state_str.c_str());
        } else {
            ROS_WARN_THROTTLE(5.0, "[MissionManager]   - performTakeoffHandoff: current_skill_[%zu] is not takeoff "
                                  "(skill_type=%u), skip force-COMPLETE",
                              current_skill_index_,
                              (current_skill_index_ < skill_queue_.size())
                                  ? skill_queue_[current_skill_index_].msg.skill_type : 0);
        }

        // 7. 推进 MissionPhase
        current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;

        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF COMPLETE, UAV now waypoint-following =====");
    }

    // === HOLDING: 所有 skill 完成后原地悬停等新 task_flow ===
    // 复用 startSetpointPublisher() 的 position-hold 机制(刚修好的航向 + 50Hz + OFFBOARD 持续 setpoint)
    // 与 takeoff 的区别仅在于 z 目标:起飞是 takeoff_altitude_,holding 是当前高度
    // PX4 OFFBOARD 持续 setpoint 才能稳悬停,这是最简单可控的 hold 方案
    void enterHoldState() {
        if (is_pose_received_) {
        ROS_WARN_THROTTLE(5.0, "[MissionManager] HOLDING current_pose (NED): x=%.2f y=%.2f z=%.2f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
        }

        // 幂等:advanceSkillStateMachine() 在 10Hz 重复 tick,只要 last_skill 仍是 COMPLETE 就会再次进入这里。
        // 已经在 HOLDING 且 setpoint publisher 在跑时直接 return,避免反复 start/stop。
        // 关键守卫:setpoint_running_ 必须为 true 才 early-return —— 只看 phase==HOLDING 不够,
        // 因为 taskFlowCallback 收到新 task_flow 时会 stopSetpointPublisher()(L1401) 但**不重置 phase**
        // (新 flow 首发不是 takeoff 时),导致 phase=HOLDING 但 publisher 已停,此时必须重启+衔接。
        // 否则会出现两 setpoint 源同时停的真空间隙,PX4 COM_OFFBOARD_LOSS_TIMEOUT → AUTO.LAND failsafe。
        if (current_phase_ == MissionPhase::PHASE_HOLDING && setpoint_running_) {
            // === CODEX HOLD-ALTITUDE-LATCH START ===
            // 已经在 HOLDING 且 publisher 活着时必须保持首次进入 HOLDING 锁存的 z。
            // 旧逻辑每个 10Hz tick 都用当前高度覆盖 setpoint：飞机瞬时下沉一点，
            // 目标高度也跟着降低，形成持续向下的“棘轮”。这里直接返回，不再改 z。
            // 新 TaskFlow 会先 stopSetpointPublisher()；下一次真正进入 HOLDING 时
            // setpoint_running_=false，仍会走下方首次锁存分支记录新的当前高度。
            // === CODEX HOLD-ALTITUDE-LATCH END ===
            return;
        }
        current_phase_ = MissionPhase::PHASE_HOLDING;
        // has_task_flow_ 保持 true(队列不清,用于 MissionState 上报 last_skill_type 供 GS 观测)
        // skill_queue_ 也不清
        // current_skill_index_ 保持指向最后一个完成的 skill

        if (!is_pose_received_) {
            ROS_WARN("[MissionManager] HOLDING: no pose yet (will wait inside startSetpointPublisher up to 10s)");
        }

        // === 无缝接管 waypoint_executor → setpoint publisher ===
        // 背景(实测日志 1784519200.309~201.011):
        //   - waypoint_executor 在 L200.309 自己报 SKILL_AREA COMPLETE 后停止发 setpoint_velocity
        //   - mission_manager 在 L200.582 state→COMPLETE,L200.682 才进 enterHoldState
        //   - 距离 ~373ms,加上 startSetpointPublisher 线程启动 + PX4 端识别新 setpoint_position,
        //     总真空 > PX4 COM_OFFBOARD_LOSS_TIMEOUT(500ms)→ AUTO.LAND failsafe
        // 修复策略:让两个 setpoint 源短暂并存 ~50ms,堵住真空窗口:
        //   1. 立刻给 waypoint_executor 发 "stop"(它在下一个 10Hz tick 内停发 setpoint_velocity)
        //   2. 立刻 startSetpointPublisher() 启 setpoint_position 50Hz
        //   3. 重叠期间 PX4 OFFBOARD 任一收到 setpoint 就保持(不冲突,topic 分别是
        //      /mavros/setpoint_velocity/cmd_vel 和 /mavros/setpoint_position/local)
        //   4. 50ms 后 waypoint_executor 完全停止,只剩 setpoint publisher 接管
        // 关键:不能反过来(先停 waypoint_executor 再启 publisher)—— 那就在两者之间产生真空
        {
            std_msgs::String stop_cmd;
            stop_cmd.data = "stop";
            waypoint_control_pub_.publish(stop_cmd);
            ROS_WARN("[MissionManager]   - HOLDING: stop command sent to waypoint_executor "
                     "(will stop ~50ms later, overlapping with setpoint publisher)");
        }

        if (!setpoint_running_) {
            // 启动 setpoint publisher (它会 wait 最多 10s 等首帧 pose,然后 latch xy + 计算航向)
            startSetpointPublisher();
            // 覆盖 z 目标为当前高度(NED z 向下为正 → ENU z up 正数)
            if (is_pose_received_) {
                double current_alt_enu = -current_pose_.pose.position.z;
                takeoff_setpoint_.pose.position.z = current_alt_enu;
                ROS_WARN("[MissionManager] HOLDING setpoint z=%.2fm (current altitude, xy+heading latched from pose)",
                         current_alt_enu);
            }
        } else {
            // 已经运行中(理论上 performTakeoffHandoff 已经停了 setpoint,这里只是兜底)
            // 直接更新 z 为当前高度
            if (is_pose_received_) {
                takeoff_setpoint_.pose.position.z = -current_pose_.pose.position.z;
            }
        }

        ROS_WARN("[MissionManager] ===== HOLDING: pose+heading locked, awaiting next task_flow =====");
    }

    // === RETURN + PX4 AUTO.LAND 落地 ===
    // 由 advanceSkillStateMachine COMPLETE 分支(最后一个 skill = 103)调用
    // 流程:
    //   1. 停 waypoint_executor (避免与 PX4 内部控制冲突)
    //   2. 停 setpoint publisher (AUTO.LAND 接管,不需要 OFFBOARD setpoint)
    //   3. 禁用制导 (如有)
    //   4. setMode("AUTO.LAND") — PX4 自带降落 + 着陆后自动 disarm
    //   5. 标志 is_landing_in_progress_=true,phase=PHASE_RETURNING
    //   6. 后续由 checkLandingComplete() 在 missionTimer 中监控 current_mavros_state_.armed
    //      → false 判定落地完成
    // 注意:
    //   - PX4 AUTO.LAND 需要 UAV 已解锁(armed=true),否则会拒绝
    //   - PX4 着陆后自动 disarm(配置 COM_DISARM_LAND),无需我们再发 disarm 命令
    //   - 兜底:落地安全超时 landing_safety_timeout_sec_(默认 120s) 后若仍未 disarm,
    //     强制调 armVehicle(false) — 防止 PX4 因地形/传感器异常卡在 AUTO.LAND
    //   - 任务流中段出现 Return 也走同一入口(advanceSkillStateMachine 不区分位置)
    //   - 中途有用户重新下发 task_flow 时,taskFlowCallback 顶部会清 is_landing_in_progress_
    void triggerPx4Landing() {
        if (is_landing_in_progress_ || landing_already_triggered_) {
            return;  // 幂等:已在降落流程,或本 task_flow 已触发过返航,避免 10Hz 状态机重复触发
        }
        is_landing_in_progress_      = true;
        landing_already_triggered_   = true;
        landing_start_time_          = ros::Time::now();
        landing_complete_logged_     = false;
        current_phase_               = MissionPhase::PHASE_RETURNING;

        ROS_WARN("[MissionManager] ===== Return-skill  PX4 AUTO.LAND triggered =====");

        // 1. 停 waypoint_executor (AUTO.LAND 接管水平位置 + 下降率,executor 不应再发速度)
        {
            std_msgs::String cmd;
            cmd.data = "stop";
            waypoint_control_pub_.publish(cmd);
        }

        // 2. 停 setpoint publisher (PX4 AUTO.LAND 不接受 OFFBOARD setpoint,留着会冲突)
        if (setpoint_running_) {
            stopSetpointPublisher();
        }

        // 3. 禁用制导 (如有遗留)
        if (is_guidance_active_) {
            disableGuidance();
        }

        // 4. 切 PX4 到 AUTO.LAND (set_mode_client 仅在 !use_sim_ 初始化)
        if (!use_sim_ && set_mode_client_.exists()) {
            mavros_msgs::SetMode sm;
            sm.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(sm) && sm.response.mode_sent) {
                ROS_WARN("[MissionManager] AUTO.LAND: PX4 mode accepted, waiting for touchdown");
            } else {
                ROS_ERROR("[MissionManager] AUTO.LAND set FAILED (mode_sent=%d, current_mode=%s)",
                          sm.response.mode_sent ? 1 : 0,
                          current_mavros_state_.mode.c_str());
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "[MissionManager] AUTO.LAND: use_sim_=%d, skipping actual mode call (sim mode); "
                     "waiting for disarm", use_sim_ ? 1 : 0);
        }
    }

    // 由 missionTimerCallback 每 tick 调用 — 监控 PX4 落地完成
    // 完成判定:current_mavros_state_.armed 从 true → false (PX4 AUTO.LAND 着陆后自动 disarm)
    // 兜底:启动后 landing_safety_timeout_sec_(默认 120s) 仍未 disarm → 强制 disarm
    void checkLandingComplete() {
        if (!is_landing_in_progress_) {
            return;
        }

        if (!current_mavros_state_.armed) {
            // 落地 + disarm 完成
            if (!landing_complete_logged_) {
                double elapsed = (ros::Time::now() - landing_start_time_).toSec();
                ROS_WARN("[MissionManager] ===== LANDING COMPLETE: PX4 disarmed "
                         "(elapsed=%.1fs, mode=%s) =====",
                         elapsed, current_mavros_state_.mode.c_str());
                ROS_WARN("[MissionManager] ===== UAV on ground. Awaiting next task_flow or shutdown. =====");
                landing_complete_logged_ = true;
                // 权威复位:不依赖 mavrosStateCallback 的 armed=false 分支时序(那段只在
                // is_landing_in_progress_ 仍为 true 的那一拍触发,易被并发回调抢先清掉而漏跑)。
                // 这里直接落地=GROUND_IDLE,清接管标志,让下一条带 takeoff 的 task_flow 能正常起飞
                // (否则 phase 残留在 RETURNING/OVERRIDDEN → isInFlight()==true → taskFlowCallback 剥/拒 takeoff)。
                is_landing_in_progress_    = false;
                is_fcu_overridden_         = false;
                current_phase_             = MissionPhase::PHASE_GROUND_IDLE;
                // === 2026-07-28 fix: 同时清 landing_already_triggered_,允许后续 task_flow 的
                //   Return(103) 再次触发 AUTO.LAND。原版只清 is_landing_in_progress_,
                //   导致 taskFlowCallback L1678 的清理块(if (is_landing_in_progress_))在第二次
                //   起落时条件已为 false 而被跳过,landing_already_triggered_ 残留为 true,
                //   advanceSkillStateMachine L4607 的双标志守卫永远 false,triggerPx4Landing()
                //   不会被调用,Return skill 静默吞掉 → PX4 OFFBOARD 失流 → failsafe 进 position。
                landing_already_triggered_ = false;
                // 不再切回 PHASE_HOLDING (UAV 已在地面,setpoint publisher 不需要再起)
                // GS 通过 MissionState.phase=GROUND_IDLE + state.armed=false 即可判定任务结束
            }
            return;
        }

        // 还在 ARMED 状态 — 检查是否需要兜底
        double elapsed = (ros::Time::now() - landing_start_time_).toSec();
        if (elapsed > landing_safety_timeout_sec_) {
            // AUTO.LAND 卡住(可能因地形/传感器问题未触发 disarm),强制 disarm 兜底
            // 强制前:先尝试切 POSCTL (脱离 AUTO.LAND 状态机),再 disarm,某些固件要求模式非 LAND 才能 disarm
            if (!use_sim_ && set_mode_client_.exists()) {
                mavros_msgs::SetMode sm;
                sm.request.custom_mode = "POSCTL";
                if (set_mode_client_.call(sm)) {
                    ROS_WARN("[MissionManager]   - switched to POSCTL for force-disarm");
                }
                ros::Duration(0.2).sleep();
            }
            ROS_ERROR("[MissionManager] Landing timeout (%.1fs > %.1fs), forcing disarm",
                      elapsed, landing_safety_timeout_sec_);
            if (!use_sim_ && arming_client_.exists()) {
                armVehicle(false);  // 调 mavros/cmd/arming value=false
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "[MissionManager] AUTO.LAND in progress (elapsed=%.1fs, mode=%s, armed=%d)",
                              elapsed,
                              current_mavros_state_.mode.c_str(),
                              current_mavros_state_.armed ? 1 : 0);
        }
    }

    // PX4 SITL: 启动 setpoint 发布线程
    void startSetpointPublisher() {
        if (setpoint_running_) {
            return;
        }
        // 等待首次位置数据（确保使用当前 UAV 位置作为起飞起点）
        // 3588 + 真机场景下 mavros 启动到首帧 local_position 可能 >4s，所以放宽到 10s
        int wait_count = 0;
        while (!is_pose_received_ && ros::ok() && wait_count < 500) {  // 最多等 10 秒
            ros::Duration(0.02).sleep();
            wait_count++;
        }
        if (!is_pose_received_) {
            ROS_WARN("[MissionManager] >>>> No pose received before setpoint publisher start "
                     "(waited %.1fs), using default (0,0). Will jump to actual position when "
                     "first pose arrives — may cause brief takeoff overshoot.",
                     wait_count * 0.02);
            // mavros/setpoint_position/local 期望 ENU，0,0 直接发即可
            takeoff_setpoint_.pose.position.x = 0;
            takeoff_setpoint_.pose.position.y = 0;
        } else {
            // current_pose_ 是 NED（来自 selfPoseCallback 的 ENU→NED 转换）
            // mavros/setpoint_position/local 期望 ENU：x_east = y_ned, y_north = x_ned
            takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // y_ned → x_enu (east)
            takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // x_ned → y_enu (north)
            ROS_INFO("[MissionManager] Takeoff setpoint start: x=%.2f, y=%.2f (ENU, from NED pos=%.2f,%.2f, pose received after %.1fs)",
                     takeoff_setpoint_.pose.position.x, takeoff_setpoint_.pose.position.y,
                     current_pose_.pose.position.x, current_pose_.pose.position.y,
                     wait_count * 0.02);
        }

        // 初始低高度（ENU z up=正，直接用正值即可）
        takeoff_setpoint_.pose.position.z = 0.5;

        // 计算起飞航向：用当前机头朝向发布 setpoint，避免 PX4 接管瞬间 yaw 回零而转圈
        // current_pose_ 是 NED 约定，mavros/setpoint_position/local 默认期望 ENU
        if (is_pose_received_) {
            const auto& q = current_pose_.pose.orientation;
            // NED 下的 yaw：0=北，顺时针为正
            double yaw_ned = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            cerr<<"[MissionManager] Takeoff setpoint heading: current NED yaw = "<< yaw_ned * 180.0 / M_PI << endl;
            // NED yaw → ENU yaw：ENU 0=东，逆时针为正
            // 关系：机头指 NED-北 等价于 ENU-北（ENU yaw=π/2）；机头指 NED-东 等价于 ENU-东（ENU yaw=0）
            double yaw_enu = M_PI_2 - yaw_ned;
            while (yaw_enu > M_PI)  yaw_enu -= 2.0 * M_PI;
            while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
            // 绕 ENU Z 轴的旋转四元数 (0, 0, sin(yaw/2), cos(yaw/2))
            double half = 0.5 * yaw_enu;
            takeoff_setpoint_.pose.orientation.x = 0.0;
            takeoff_setpoint_.pose.orientation.y = 0.0;
            takeoff_setpoint_.pose.orientation.z = std::sin(half);
            takeoff_setpoint_.pose.orientation.w = std::cos(half);
            ROS_INFO("[MissionManager] Takeoff setpoint heading: NED yaw=%.2f -> ENU yaw=%.2f",
                     yaw_ned * 180.0 / M_PI, yaw_enu * 180.0 / M_PI);
        } else {
            // 没拿到姿态就保持默认 (yaw=0, ENU 朝东)，至少不会去强行改一个未知朝向
            takeoff_setpoint_.pose.orientation.x = 0.0;
            takeoff_setpoint_.pose.orientation.y = 0.0;
            takeoff_setpoint_.pose.orientation.z = 0.0;
            takeoff_setpoint_.pose.orientation.w = 1.0;
        }

        setpoint_publish_count_ = 0;
        setpoint_start_time_ = ros::Time::now();   // 关键：thread 即将开始的时刻
        setpoint_running_ = true;
        setpoint_thread_ = std::thread([this]() {
            ROS_INFO("[MissionManager] Setpoint publisher thread started");
            ros::Rate rate(50);  // 50Hz
            ros::Time last_log = ros::Time::now();
            // 🛠️ 新增：创建一个基准时间，模拟板子时间是和飞控同步对齐的
            ros::Time fake_now = ros::Time::now();

            while (ros::ok() && setpoint_running_) {
                // 🛠️ 修正：不要用 Time(0)，也不要用本地错误的 Time::now()
                // 让我们手动构建一个稳定的单调递增时间戳流发送给 Mavros
                fake_now += ros::Duration(0.02); // 50Hz 每次递增 20ms
                takeoff_setpoint_.header.stamp = fake_now;
                // 航向已在 startSetpointPublisher() 里按当前机头朝向算好，
                // 这里不要每帧覆盖成 w=1，否则 PX4 接管瞬间仍会 yaw 回零转圈
                takeoff_setpoint_pub_.publish(takeoff_setpoint_);
                setpoint_publish_count_++;
                //ros::spinOnce();
                rate.sleep();

                // 每 5s 打印一次发包状态 + PX4 当前 mode（多 UAV 对比观察用）
                if ((ros::Time::now() - last_log).toSec() > 5.0) {
                    ROS_WARN("[MissionManager] setpoint stream: %d pkts, PX4 mode=%s armed=%d connected=%d system_status=%d",
                             setpoint_publish_count_,
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0,
                             current_mavros_state_.connected ? 1 : 0,
                             current_mavros_state_.system_status);
                    last_log = ros::Time::now();
                }
            }
            ROS_INFO("[MissionManager] Setpoint publisher thread stopped (total pkts=%d)", setpoint_publish_count_);
        });
    }

    void stopSetpointPublisher() {
        setpoint_running_ = false;
        // === 2026-08-03: 清掉所有 latch 标志,避免 setpoint 停了但标志还残留 ===
        //   之前 stop 只清 setpoint_running_,attack_hover_setpoint_latched_ 和
        //   target_lost_hover_setpoint_latched_ 留着 — 下次 taskFlowCallback 或
        //   enterHoldState 等流程 stop+restart 时,startSetpointPublisher() 看到
        //   setpoint_running_=false 会启新线程,但 latch 标志污染会让上游误判
        //   "已经被我启过 / 我拥有这个 setpoint 流"。
        attack_hover_setpoint_latched_        = false;
        target_lost_hover_setpoint_latched_   = false;
        if (setpoint_thread_.joinable()) {
            setpoint_thread_.join();
        }
    }

    bool setMode(const std::string& mode) {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        ROS_WARN("[MissionManager] >>>>> setMode(%s) called, PX4 current_mode=%s connected=%d armed=%d setpoint_pkts=%d",
                 mode.c_str(),
                 current_mavros_state_.mode.c_str(),
                 current_mavros_state_.connected ? 1 : 0,
                 current_mavros_state_.armed ? 1 : 0,
                 setpoint_publish_count_);

        // ===== Path 1: 标准 SET_MODE 服务（走 SET_MODE MAVLink 消息）=====
        bool set_mode_sent = false;
        if (set_mode_client_.call(set_mode) && set_mode.response.mode_sent) {
            set_mode_sent = true;
        } else {
            ROS_WARN("[MissionManager] >>>>> SET_MODE service failed, will try COMMAND_LONG fallback");
        }

        // ===== 等 5 秒看是否切成功（覆盖 SET_MODE 和 COMMAND_LONG 都试一遍）=====
        ros::Rate rate(10);
        auto start = ros::Time::now();
        int poll_count = 0;
        bool tried_command_long = false;
        while (ros::ok() && (ros::Time::now() - start).toSec() < 5.0) {
            ros::spinOnce();
            poll_count++;
            if (current_mavros_state_.mode == mode) {
                ROS_INFO("[MissionManager] >>>>> Mode confirmed: %s (took %.2fs, %d polls)",
                         mode.c_str(), (ros::Time::now() - start).toSec(), poll_count);
                return true;
            }
            // 1.5s 还没切成功,且没试过 COMMAND_LONG,就走第二条路径
            if (!tried_command_long && (ros::Time::now() - start).toSec() > 1.5) {
                tried_command_long = true;
                ROS_WARN("[MissionManager] >>>>> 1.5s elapsed, PX4 still in '%s'. Trying COMMAND_LONG fallback...",
                         current_mavros_state_.mode.c_str());
                sendSetModeCommandLong(mode);
            }
            // 每 2s 报一次当前 PX4 mode（10Hz tick 中每 20 帧一次，避免被噪声淹没）
            if (poll_count % 20 == 0) {
                ROS_WARN("[MissionManager] ... waiting for mode=%s, PX4 currently in '%s' (%.1fs elapsed)",
                         mode.c_str(), current_mavros_state_.mode.c_str(),
                         (ros::Time::now() - start).toSec());
            }
            rate.sleep();
        }

        ROS_WARN("[MissionManager] >>>>> Mode change TIMEOUT after 5s, PX4 mode='%s' (expected '%s'), setpoint_pkts=%d",
                 current_mavros_state_.mode.c_str(), mode.c_str(), setpoint_publish_count_);
        return false;
    }

    // ===== Path 2: COMMAND_LONG 备用通道 =====
    // 某些 PX4 commander 状态下 SET_MODE 会被静默忽略（result=ACCEPTED 但 mode 不变），
    // COMMAND_LONG 走 handle_command 这条路径，PX4 会打明确的拒绝原因，
    // 在我们的场景里它实际上更可靠。
    void sendSetModeCommandLong(const std::string& mode) {
        if (!command_long_client_.exists()) {
            ROS_WARN("[MissionManager] >>>>> COMMAND_LONG service not available");
            return;
        }

        // PX4 custom main mode 编号（v1.10+ 稳定）
        static const std::map<std::string, uint32_t> px4_modes = {
            {"MANUAL", 1}, {"ALTCTL", 2}, {"POSCTL", 3}, {"AUTO", 4},
            {"ACRO", 5}, {"OFFBOARD", 6}, {"STABILIZED", 7}, {"RATTITUDE", 8},
        };

        auto it = px4_modes.find(mode);
        if (it == px4_modes.end()) {
            ROS_WARN("[MissionManager] >>>>> Unknown mode '%s' for COMMAND_LONG", mode.c_str());
            return;
        }

        mavros_msgs::CommandLong cmd;
        cmd.request.command = 176;                       // MAV_CMD_DO_SET_MODE
        cmd.request.param1 = 0x8C;                       // base_mode: SAFETY_ARMED | CUSTOM_MODE_ENABLED | GUIDED | STABILIZE
        cmd.request.param2 = static_cast<float>(it->second); // PX4 custom main mode
        cmd.request.param3 = 0.0;                        // custom_sub_mode
        cmd.request.param4 = 0.0;
        cmd.request.param5 = 0.0;
        cmd.request.param6 = 0.0;
        cmd.request.param7 = 0.0;
        cmd.request.broadcast = false;
        cmd.request.confirmation = 0;

        ROS_WARN("[MissionManager] >>>>> COMMAND_LONG DO_SET_MODE: base=0x%X custom=%d (%s)",
                 (int)cmd.request.param1, (int)cmd.request.param2, mode.c_str());

        if (!command_long_client_.call(cmd)) {
            ROS_WARN("[MissionManager] >>>>> COMMAND_LONG service call FAILED (RPC error)");
            return;
        }

        ROS_WARN("[MissionManager] >>>>> COMMAND_LONG response: success=%d result=%d (result: 0=ACCEPTED 1=TEMP_REJ 2=DENIED 3=UNSUPPORTED)",
                 cmd.response.success, cmd.response.result);
    }

    /**
     * 设置 PX4 MAVLink 消息发送频率（MAV_CMD_SET_MESSAGE_INTERVAL=511）
     * 某些板子上电后默认 LOCAL_POSITION_NED 频率太低 (1Hz 或 5Hz),
     * 必须上电后调一次才能让 OFFBOARD 控制流畅。
     *
     * @param msg_id     MAVLink 消息 ID (e.g. 32 = LOCAL_POSITION_NED)
     * @param rate_hz    目标频率 (Hz)
     * @return true 表示调用成功 (success=true 且 result=0=ACCEPTED)
     */
    bool setMessageRate(uint32_t msg_id, double rate_hz) {
        if (!command_long_client_.exists()) {
            ROS_WARN("[MissionManager] >>>>> command_long service not available, "
                     "cannot set msg %u rate", msg_id);
            return false;
        }
        if (rate_hz <= 0.0) {
            ROS_WARN("[MissionManager] >>>>> Invalid rate %.1f Hz for msg %u, skip", rate_hz, msg_id);
            return false;
        }

        mavros_msgs::CommandLong cmd;
        cmd.request.command = 511;                                // MAV_CMD_SET_MESSAGE_INTERVAL
        cmd.request.param1 = static_cast<float>(msg_id);          // 消息 ID
        cmd.request.param2 = static_cast<float>(1000000.0 / rate_hz);  // interval (us)
        cmd.request.param3 = 0.0;
        cmd.request.param4 = 0.0;
        cmd.request.param5 = 0.0;
        cmd.request.param6 = 0.0;
        cmd.request.param7 = 0.0;
        cmd.request.broadcast = false;
        cmd.request.confirmation = 0;

        ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL msg_id=%u rate=%.1fHz (interval=%.0fus)",
                 msg_id, rate_hz, cmd.request.param2);

        if (!command_long_client_.call(cmd)) {
            ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL service call FAILED (RPC error)");
            return false;
        }

        // MAV_CMD result: 0=ACCEPTED, 1=TEMP_REJ, 2=DENIED, 3=UNSUPPORTED, 4=FAILED, 5=IN_PROGRESS
        bool ok = (cmd.response.success && cmd.response.result == 0);
        ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL result: success=%d result=%d (%s)",
                 cmd.response.success, cmd.response.result, ok ? "ACCEPTED" : "REJECTED");
        return ok;
    }

    bool armVehicle(bool arm) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;

        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("[MissionManager] Vehicle %s", arm ? "ARMED" : "DISARMED");
            return true;
        }

        ROS_WARN_THROTTLE(2.0, "[MissionManager] Failed to %s vehicle", arm ? "arm" : "disarm");
        return false;
    }

    /**
     * ============================================================
     * OFFBOARD + ARM 快速序列（核心修复）
     * ============================================================
     *
     * 为什么要用这个序列而不是 setMode() 等 5 秒？
     *
     * 1. PX4 commander 内部逻辑:
     *    当收到 SET_MODE(OFFBOARD) 后，commander 会切到 MAIN_STATE_OFFBOARD，
     *    但紧接着每 100~500ms 会重新检查 is_offboard_available()：
     *      if (main_state == OFFBOARD && !is_offboard_available())
     *          main_state = previous_main_state;   // 立刻 revert
     *    这个 is_offboard_available() 检查的是 offboard 模块是否最近收到过 setpoint，
     *    正常情况下 mavros 在持续发 setpoint，应该返回 true。但如果有任何原因导致
     *    offboard 模块的 update 滞后于 commander 检查（CPU 抖动、MAVLink 延迟、单串口
     *    互斥等），PX4 会瞬间 revert 回 AUTO.LOITER。
     *
     * 2. pymavlink 为什么能切成功:
     *    它用的就是 race-through — SET_MODE → 50ms → ARM。在 revert 窗口内 ARM
     *    到达，commander 进入 ARM transition 状态，OFFBOARD 模式在 transition 期间
     *    被"卡住"，PX4 不得不保留 OFFBOARD 直到 ARM 完成。
     *
     * 3. 我们之前的代码 (setMode 等 5 秒) 错就错在:
     *    等到 PX4 已经 revert 回 LOITER 才去 ARM，结果 ARM 是在 LOITER 模式下请求的，
     *    UAV 上了 LOITER 的锁，但 mode 还是 LOITER — setpoint 被忽略，UAV 不动。
     *
     * 4. 这个函数的策略:
     *    SET_MODE → 50ms → COMMAND_LONG → 50ms → ARM。
     *    完整序列 ≤ 200ms，远小于 commander 的 revert 窗口。
     *    ARM 之后再做最多 3 次重试（每次 50ms），命中 PX4 还在 transition 的窗口。
     *
     * 返回：true 表示 ARM 成功（最关键的指标，UAV 能否起飞就靠它），
     *       false 表示 ARM 失败；上层 state machine 会再次重试整组动作。
     */
    bool triggerOffboardAndArm() {
        ros::Time t0 = ros::Time::now();

        // === 2026-07-30: 逐项判断当前状态,只在需要时发 ===
        //   场景:PX4 已经在 OFFBOARD+armed (飞手手动接管后又下发新 task_flow 进入接管场景,
        //   或前序 triggerOffboardAndArm 已成功)时,旧的"无脑 4 步"会刷屏:
        //     - DO_SET_MODE result=1 TEMP_REJ
        //     - FCU: Arming denied! Already armed
        //   飞机本身正常,executor 在飞,但日志噪音大。
        //   改成:mode==OFFBOARD 跳过 SET_MODE+DO_SET_MODE,armed==true 跳过 ARM+重试。
        //   返回值语义不变 (true = 当前已 OFFBOARD+armed,可起飞)。
        //   注意:**不**用 early-return,因为这样会丢掉"手动 ARM 但 mode 还在 LOITER"
        //   这种半状态的接管能力 — 应该只切 mode、不再发 ARM。
        const bool need_set_mode = (current_mavros_state_.mode != "OFFBOARD");
        const bool need_arm      = !current_mavros_state_.armed;

        // -------- Step 1+2: SET_MODE service → 50ms → COMMAND_LONG DO_SET_MODE --------
        // 命中 PX4 commander handle_command 路径 — 某些固件版本对这条路径更稳定
        bool set_mode_sent = false;
        if (need_set_mode) {
            mavros_msgs::SetMode set_mode;
            set_mode.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(set_mode)) {
                set_mode_sent = set_mode.response.mode_sent;
            } else {
                ROS_WARN("[MissionManager] >>>> SET_MODE service: RPC call FAILED");
            }
            ros::Duration(0.05).sleep();
            sendSetModeCommandLong("OFFBOARD");
            ros::Duration(0.05).sleep();
        } else {
            ROS_INFO_THROTTLE(5.0,
                "[MissionManager] triggerOffboardAndArm: mode already OFFBOARD, skip SET_MODE/DO_SET_MODE");
        }

        // -------- Step 3+4: ARM + 最多 3 次重试(每步都查 armed,已在 armed 直接视为成功) --------
        bool armed = !need_arm;   // 不需要 ARM → 视为已成功,跳过整个 ARM 段
        if (need_arm) {
            // 关键时机 — ARM 必须在 commander 还没 revert 之前到达
            armed = armVehicle(true);
            for (int i = 0; i < 3 && !armed && ros::ok(); ++i) {
                ros::Duration(0.05).sleep();
                armed = armVehicle(true);
            }
        } else {
            ROS_INFO_THROTTLE(5.0,
                "[MissionManager] triggerOffboardAndArm: already armed, skip ARM");
        }

        ros::Duration(0.05).sleep();
        ROS_WARN("[MissionManager] >>>> [%4.0fms] triggerOffboardAndArm DONE: armed=%d, "
                 "PX4 mode=%s armed=%d sys_status=%d (need_set_mode=%d need_arm=%d)",
                 (ros::Time::now() - t0).toSec() * 1000.0, armed ? 1 : 0,
                 current_mavros_state_.mode.c_str(),
                 current_mavros_state_.armed ? 1 : 0,
                 current_mavros_state_.system_status,
                 need_set_mode ? 1 : 0, need_arm ? 1 : 0);
        return armed;
    }

    // PX4 SITL: 获取起飞是否完成
    bool isTakeoffComplete() const {
        return takeoff_state_ == TakeoffState::TAKEOFF_COMPLETE;
    }

    // 旧 avoidanceTimerCallback + computeAndPublishAvoidanceVector 已删除 —
    //   原 repulsion 向量发到 /avoidance/vector 但无订阅,机间避让完全由
    //   uav_avoidance_node + waypoint_executor 接管,这里不再重复计算。

    // ============== Phase 2: MissionState 1Hz 定频上报 ==============

    /**
     * 1Hz 周期发送 MissionState — 不论 Skill 状态如何都发(用户明确要求"定频")
     */
    void missionStateTimerCallback(const ros::TimerEvent&) {
        publishMissionState();
        publishCurrentWorkMode();
    }

    void publishMissionState() {
        multi_uav_strike::MissionState ms;
        ms.skill_flow_id = current_flow_id_;
        ms.skill_type    = -1;  // -1 表示无 active skill
        ms.skill_id      = "";
        ms.state         = 0;   // 默认未开始
        ms.phase         = static_cast<uint8_t>(current_phase_);  // 当前任务阶段(GS 用此判断 holding/waypoint/guidance)
        ms.attack_arrival_time_us = 0;  // 默认未到

        if (current_skill_index_ < skill_queue_.size()) {
            const auto& sr = skill_queue_[current_skill_index_];
            ms.skill_id   = sr.msg.skill_id;
            ms.skill_type = static_cast<int32_t>(sr.msg.skill_type);
            ms.state      = skillStateToInt(sr.state);
            // Attack(105) 专用:本机到达 attack_point 的微秒时间戳
            //   仅 Attack(105) + 已记录到 attack_arrival_time_ 时填;其它场景固定 0
            if (sr.msg.skill_type == 105 && !sr.attack_arrival_time_.isZero()) {
                ms.attack_arrival_time_us =
                    sr.attack_arrival_time_.toNSec() / 1000;
            }
        }

        mission_state_pub_.publish(ms);
        last_mission_state_pub_time_ = ros::Time::now();
    }

    /**
     * 1Hz 上报当前 work_mode(typed) — 给 starling_bridge 订阅,然后通过 KCP BIZ_WORK_MODE_REPORT(0x1004)
     * 回传给地面站。注意:这里发的就是内部 current_work_mode_ 的 typed 映射值,与 GS 通过 SET_WORKMODE
     * 下发的语义一致(starling_bridge 已把 GCS 0/1/2/3 映射为 SEARCH_STRIKE/TRACK/ONLY/DENIED_ENV_FLIGHT)。
     */
    void publishCurrentWorkMode() {
        multi_uav_strike::WorkMode wm;
        // 当前 WorkMode 枚举值与 typed WorkMode.msg 常量数值对齐(SEARCH_STRIKE=5 等),
        // 直接 cast 即可,无需 switch。
        wm.mode = static_cast<uint8_t>(current_work_mode_);
        work_mode_state_pub_.publish(wm);
    }

    static int32_t skillStateToInt(SkillState s) {
        switch (s) {
            case SkillState::PENDING:        return 0;
            case SkillState::TRANSIT:        return 1;
            case SkillState::ENTRY_PENDING:  return 1;  // 执行中
            case SkillState::IN_TASK:        return 1;
            case SkillState::EXIT_PENDING:   return 1;
            case SkillState::COMPLETE:       return 3;
            case SkillState::FAILED:         return 4;
            default:                         return 0;
        }
    }

    static std::string skillStateStr(SkillState s) {
        switch (s) {
            case SkillState::PENDING:       return "PENDING";
            case SkillState::TRANSIT:       return "TRANSIT";
            case SkillState::ENTRY_PENDING: return "ENTRY_PENDING";
            case SkillState::IN_TASK:       return "IN_TASK";
            case SkillState::EXIT_PENDING:  return "EXIT_PENDING";
            case SkillState::COMPLETE:      return "COMPLETE";
            case SkillState::FAILED:        return "FAILED";
            default:                        return "UNKNOWN";
        }
    }

    // ============== Phase 3: Skill 状态机推进 ==============

    /**
     * 10Hz tick,推进当前 skill 的 SkillState 转换
     *
     * 状态转换(根据 waypoint_executor 的 WaypointStatus 反馈 + 距离判定):
     *   PENDING → TRANSIT:                executor 收到 Skill 后自动推
     *   TRANSIT → ENTRY_PENDING:           phase == ARRIVE 且 arrive_idx == arrive_total
     *   ENTRY_PENDING → IN_TASK:          phase == SKILL_AREA 且距 path[0] ≤ arrival_threshold
     *   IN_TASK → EXIT_PENDING:           phase == SKILL_AREA 且 skill_idx == skill_total
     *   EXIT_PENDING → COMPLETE:          等一帧发最终遥测后
     *
     * 失败:
     *   ENTRY_PENDING 超时 entry_gate_timeout_(30s) → FAILED
     *   IN_TASK 超时 in_task_timeout_(0=无限) → FAILED
     */
    void skillAdvanceTimerCallback(const ros::TimerEvent&) {
        if (skill_queue_.empty()) return;
        // 每 5s 心跳打印当前 skill 状态机状态（慢速参考；状态变化时已有单独日志）
        const SkillRuntime& sr = skill_queue_[current_skill_index_];
        ROS_INFO_THROTTLE(5.0, "[MissionManager] Skill[%zu] id=%s type=%u state=%s, phase=%s",
                          current_skill_index_, sr.msg.skill_id.c_str(),
                          static_cast<unsigned>(sr.msg.skill_type),
                          skillStateStr(sr.state).c_str(),
                          missionPhaseToString().c_str());
        if (current_skill_index_ >= skill_queue_.size()) return;
        // === CODEX DIRECT-LOOK START ===
        // 先处理预检超时；超时函数会原样下发 Skill 并切回标准 TRANSIT。
        advanceAttackDirectLook();
        // === CODEX DIRECT-LOOK END ===

        // === CODEX DIRECT-LOOK START ===
        // 直接打击到达目标阈值后，接回原 Skill 的标准收尾：
        // EXIT_PENDING -> COMPLETE -> HOLDING，不能继续压到地面后再恢复 WP1。
        completeDirectStrikeIfReached();
        // === CODEX DIRECT-LOOK END ===

        // === CODEX DIRECT-LOOK START ===
        // 历史背景:这段每 10Hz 重发 stop 是为了"跨 topic 不保证首次 publish 到达顺序"
        // 兜底。后果:waypoint_executor.controlCallback 收到 stop → stop() → publishZeroVelocity()
        //   → 往 /mavros/setpoint_velocity/cmd_vel_unstamped 发一帧全 0。
        //   guidance_control_node 也在 50Hz 发速度,被这一帧 0 插进去 → PX4 OFFBOARD 看到
        //   速度跳变,横滚/偏航都被推一下(实测 2026-08-04-16-11 bag:STRIKE 期间每 100ms
        //   一帧全 0,正好每 5 帧 guidance 数据中插一帧 0)。
        // 修法:删掉这段重发 stop。"stop" 已在 triggerStrike → startGuidanceApproach 时
        //   通过 mission_manager → waypoint_executor 发过一次(line 4423 附近),单次足够。
        //   后续阶段要恢复 waypoint(EXIT_PENDING → COMPLETE → HOLDING)走 resume 路径,
        //   不需要每 tick 再发 stop。
        // === CODEX DIRECT-LOOK END ===
        advanceSkillStateMachine();
        // Attack(105) 专用 tick:门控重试 + 同步时间重算 + 挂起超时放弃
        advanceAttackPhaseMachine();
    }

    // === CODEX DIRECT-LOOK START ===
    /**
     * 直视打击命中判定。
     * current_pose_ 是 NED (North,East,Down)，locked_cluster_* 实际是 NWU
     * (North,West,Up)，统一到 NWU 后计算三维距离。达到既有
     * strike_distance_threshold_（common.yaml 默认 2m）即按正常 Skill 完成收尾。
     */
    void completeDirectStrikeIfReached() {
        if (current_skill_index_ >= skill_queue_.size()) return;
        SkillRuntime& sr = skill_queue_[current_skill_index_];
        if (!sr.direct_strike_committed_ || !is_guidance_active_ ||
            locked_cluster_id_ == 0 || !is_pose_received_) return;

        const double uav_north = current_pose_.pose.position.x;
        const double uav_west  = -current_pose_.pose.position.y;
        const double uav_up    = -current_pose_.pose.position.z;
        const double dx = uav_north - locked_cluster_ned_x_;
        const double dy = uav_west  - locked_cluster_ned_y_;
        const double dz = uav_up    - locked_cluster_ned_alt_;
        const double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (distance > strike_distance_threshold_) return;

        ROS_WARN("[MissionManager] Attack DIRECT_STRIKE reached target: distance=%.2fm <= %.2fm; "
                 "finishing Attack via EXIT_PENDING -> COMPLETE -> HOLDING",
                 distance, strike_distance_threshold_);

        // 先关闭 velocity guidance，再由 enterHoldState 启当前位置 setpoint，保持
        // OFFBOARD 连续且不再让已预装的 WP1->WP2 队列恢复执行。
        disableGuidance();
        std_msgs::String stop_cmd;
        stop_cmd.data = "stop";
        waypoint_control_pub_.publish(stop_cmd);

        sr.direct_strike_committed_ = false;
        sr.state = SkillState::EXIT_PENDING;
        sr.state_enter_time = ros::Time::now();
        sr.last_event = "DIRECT_STRIKE reached target -> EXIT_PENDING";
        current_task_status_ = TaskStatus::STRIKE_EXECUTED;
        attack_strike_phase_active_ = false;
        target_lost_hover_active_ = false;
        target_lost_hover_setpoint_latched_ = false;
        current_target_.is_locked = false;
        current_target_.is_detected = false;
        tracked_target_.is_valid = false;
        locked_cluster_id_ = 0;
        locked_cluster_last_seen_us_ = 0;
        setLockState(TrackLockState::NOT_LOCKED);
        publishPrimaryTarget(0);
        enterHoldState();
        publishMissionState();
    }
    // === CODEX DIRECT-LOOK END ===

    // === CODEX DIRECT-LOOK START ===
    /** Attack(105) 原地直视预检超时：没有连续三帧有效目标就无损回退 WP1→WP2。 */
    void advanceAttackDirectLook() {
        if (current_skill_index_ >= skill_queue_.size()) return;
        SkillRuntime& sr = skill_queue_[current_skill_index_];
        if (!sr.direct_look_active_) return;
        if ((ros::Time::now() - sr.direct_look_start_time_).toSec() <
            attack_direct_look_timeout_sec_) return;

        sr.direct_look_active_ = false;
        sr.direct_look_cluster_id_ = 0;
        sr.direct_look_last_seen_us_ = 0;
        sr.direct_look_consecutive_hits_ = 0;
        sr.state = SkillState::PENDING;
        sr.state_enter_time = ros::Time::now();
        sr.last_event = "Attack direct-look timeout: fallback to WP1->WP2";
        ROS_WARN("[MissionManager] Attack DIRECT_LOOK timeout after %.1fs: no 3-frame target, "
                 "falling back to original WP1->WP2 path",
                 attack_direct_look_timeout_sec_);

        // 先让 executor 建立速度 setpoint 源，再停止 position hold，避免 OFFBOARD 指令真空。
        pushCurrentSkillToExecutor();
        attack_hover_setpoint_latched_ = false;
        if (setpoint_running_) stopSetpointPublisher();
    }
    // === CODEX DIRECT-LOOK END ===

    void advanceSkillStateMachine() {
        if (current_skill_index_ >= skill_queue_.size()) return;
        SkillRuntime& sr = skill_queue_[current_skill_index_];
        const ros::Time now = ros::Time::now();

        // === 目标丢失状态机(2026-07-23,用户口径 C;2026-07-26 改用 last_seen_us)===
        // 判据:locked_cluster_last_seen_us_(YOLO 实际命中微秒戳)的新鲜度。
        //   此前用 locked_cluster_time_(cluster_states 回调到达时刻)无效 — dedup 路径下
        //   locked_cluster_time_ 只在首次派发时被设,后续 10Hz callback 直接 continue,
        //   不刷新时间戳,导致 (now - locked_cluster_time_) 单调递增 → 首次派发后 2.0s
        //   (target_lost_hold_sec_)必然误判 TARGET_LOST_HOVER,即使 YOLO 命中一直新鲜。
        //   修法:dedup 分支同步刷新 locked_cluster_time_/last_seen_us_ (见 clusterStatesCallback),
        //        此处改用 (now - last_seen_us) 反映"YOLO 多久没看到目标"。
        //   GUIDANCE_TRACK/STRIKE 中 age > T_hold(2s) → 进入 TARGET_LOST_HOVER。
        //   TARGET_LOST_HOVER 中 age > T_resume(10s) → 回 WAYPOINT_FOLLOW,resume 段扫描。
        if ((current_phase_ == MissionPhase::PHASE_GUIDANCE_TRACK ||
             current_phase_ == MissionPhase::PHASE_GUIDANCE_STRIKE) &&
            locked_cluster_id_ != 0 && locked_cluster_last_seen_us_ != 0) {
            double age = (now.toNSec() / 1000ULL - locked_cluster_last_seen_us_) / 1e6;
            if (age > target_lost_hold_sec_) {
                ROS_WARN("[MissionManager] Target lost in guidance (cluster=%lu age=%.1fs > hold=%.1fs) "
                         "→ TARGET_LOST_HOVER (will re-try up to resume=%.1fs, then resume segment scan)",
                         (unsigned long)locked_cluster_id_, age, target_lost_hold_sec_,
                         target_lost_resume_sec_);
                current_phase_                = MissionPhase::PHASE_TARGET_LOST_HOVER;
                target_lost_hover_active_     = true;
                target_lost_hover_enter_time_ = now;
                disableGuidance();
                std_msgs::String stop_cmd;
                stop_cmd.data = "stop";
                waypoint_control_pub_.publish(stop_cmd);

                // === 2026-08-03: 启 setpoint publisher 锁当前位置 ===
                //   旧的"disableGuidance + stop"会让 PX4 OFFBOARD 500ms~1s 内失流
                //   → AUTO.LAND failsafe,UAV 永远等不到 target_lost_resume_sec_ 的
                //   resume 分支。复用 attack_hover 同样的 latch 模式:起 50Hz 位置
                //   setpoint,锁当前 pose 维持 OFFBOARD 心跳。
                //   若 setpoint_running_ 已经是 true(例如 attack_hover 启的),直接
                //   复用,只置 latch 标志以便 resume 时知道是自己启的。
                if (!setpoint_running_ && is_pose_received_) {
                    startSetpointPublisher();
                    takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // y_ned → x_enu (east)
                    takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // x_ned → y_enu (north)
                    takeoff_setpoint_.pose.position.z = -current_pose_.pose.position.z; // 当前高度 (NED z 取反)
                    target_lost_hover_setpoint_latched_ = true;
                    ROS_WARN("[MissionManager] TARGET_LOST_HOVER setpoint publisher latched at current pose "
                             "(pos=%.2f,%.2f, alt=%.2f) — will hover %.1fs waiting for re-acquire",
                             takeoff_setpoint_.pose.position.x,
                             takeoff_setpoint_.pose.position.y,
                             takeoff_setpoint_.pose.position.z,
                             target_lost_resume_sec_);
                } else if (setpoint_running_) {
                    // 已被 attack_hover 等其他 latch 占用,只标自己的位
                    target_lost_hover_setpoint_latched_ = true;
                    ROS_WARN("[MissionManager] TARGET_LOST_HOVER: setpoint publisher already running (attack_hover?), "
                             "reuse it for OFFBOARD heartbeat during hover");
                }
                publishMissionState();
            }
        } else if (current_phase_ == MissionPhase::PHASE_TARGET_LOST_HOVER &&
                   target_lost_hover_active_) {
            // 恢复超时: 用 last_seen_us 优先;若从未锁定(target_lost 期间 0),fallback 进入时间
            double age = (locked_cluster_last_seen_us_ != 0)
                             ? (now.toNSec() / 1000ULL - locked_cluster_last_seen_us_) / 1e6
                             : (now - target_lost_hover_enter_time_).toSec();
            if (age > target_lost_resume_sec_) {
                ROS_WARN("[MissionManager] Target lost resume timeout (age=%.1fs > resume=%.1fs) "
                         "→ WAYPOINT_FOLLOW, resuming segment scan",
                         age, target_lost_resume_sec_);
                target_lost_hover_active_ = false;
                locked_cluster_id_        = 0;
                locked_cluster_ned_x_     = 0.0;
                locked_cluster_ned_y_     = 0.0;
                locked_cluster_ned_alt_   = 0.0;
                locked_cluster_time_      = ros::Time();
                tracked_target_.is_valid  = false;
                // 直视打击真实丢失并完成等待后，切回标准 Attack 航点门控。
                // executor 此时已缓存当前 WP1->WP2，resume 后从 WP1 正常开始。
                sr.direct_strike_committed_ = false;
                publishPrimaryTarget(0);  // estimator 回退最高置信度
                current_phase_            = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                // === 2026-08-03: 停 setpoint publisher,让 waypoint_executor 接管 ===
                //   顺序:先发 resume 给 waypoint_executor(让它开始发速度 setpoint),
                //   再停本节点自己的 setpoint publisher — 两路 setpoint 不能并存,
                //   但 waypoint_executor resume 后会从 stop 状态切回正常发速度,
                //   mission_manager 这边 setpoint publisher 立即停也不会有真空
                //   (OFFBOARD 收到下一帧 waypoint_executor 速度 setpoint 接管)。
                std_msgs::String resume_cmd;
                resume_cmd.data = "resume";
                waypoint_control_pub_.publish(resume_cmd);
                if (target_lost_hover_setpoint_latched_) {
                    target_lost_hover_setpoint_latched_ = false;
                    if (setpoint_running_) {
                        stopSetpointPublisher();
                    }
                }
                publishMissionState();
            }
        }

        switch (sr.state) {
            case SkillState::PENDING:
                // 等 pushCurrentSkillToExecutor() 推过去
                pushCurrentSkillToExecutor();
                break;

            case SkillState::TRANSIT: {
                // 特殊门控:skill_type=100 (Takeoff) — 起飞没有 arrive_path/skill_area_path,
                //   用 isTakeoffComplete() 当门控;一旦起飞完成直接跳 EXIT_PENDING(中间状态无意义)
                if (sr.msg.skill_type == 100 || sr.msg.skill_type == 106) {
                    if (!isTakeoffComplete()) break;
                    sr.state = SkillState::EXIT_PENDING;
                    sr.state_enter_time = now;
                    sr.last_event = "TRANSIT to EXIT_PENDING (takeoff complete)";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to EXIT_PENDING (takeoff)",
                             current_skill_index_, sr.msg.skill_id.c_str());
                    break;
                }
                // 普通门控:executor 报告 phase=ARRIVE 且 arrive_idx 走完 (arrive_idx >= arrive_total)
                // 修复:原来是 arrive_idx + 1 >= arrive_total,会在 idx = total - 1 (即飞机刚开始
                //   飞向最后一个 arrive 航点)时就触发,导致 COMPLETE 太早、enterHoldState 在中途捕获 pose,
                //   飞机继续飞到末点后又飞回捕获点。改为 idx >= total,等最后一个 arrive 航点 *到达* 才推进。
                if (!latest_wp_status_) break;
                if (latest_wp_status_->arrive_idx >= latest_wp_status_->arrive_total &&
                    latest_wp_status_->skill_id == sr.msg.skill_id) {
                    // === 修复:skill_area_path 为空时,executor 直接 ARRIVE→COMPLETE,
                    //   不会发 SKILL_AREA。如果照原逻辑进 ENTRY_PENDING 等 phase==SKILL_AREA,
                    //   永远等不到,30s 后 entry_gate_timeout 才 FAILED,期间 UAV 悬停不动。
                    //   语义上"已在入口"(没有 skill_area 需要进),直接进 IN_TASK。
                    if (sr.msg.skill_area_path.poses.empty()) {
                        sr.state = SkillState::IN_TASK;
                        sr.state_enter_time = now;
                        sr.in_task_enter_time = now;
                        if (sr.msg.skill_type == 101) {
                            // 101 集结合:记录进入时刻,等待其他 UAV 或 timeout
                            sr.gather_enter_time = now;
                        }
                        // === 102 Search IN_TASK 入口(2026-07-29):下发 gimbal_pitch ===
                        //   gimbal_command_timer_callback 不读 skill_state,只读 current_gimbal_pitch_,
                        //   所以状态值在 transition 一次性写一次即可,timer 5Hz 会自动续发。
                        if (sr.msg.skill_type == 102) {
                            current_gimbal_pitch_ = sr.msg.gimbal_pitch;
                            ROS_WARN("[MissionManager] Search(102) IN_TASK: gimbal_pitch -> %.2f deg "
                                     "(will publish on /<ns>/mission/gimbal_command @%.1fHz)",
                                     current_gimbal_pitch_, gimbal_command_rate_hz_);
                        }
                        sr.last_event = "TRANSIT to IN_TASK (skill_area empty, skip ENTRY_PENDING)";
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s to IN_TASK "
                                 "(type=%u, skill_area empty, skipping ENTRY_PENDING)",
                                 current_skill_index_, sr.msg.skill_id.c_str(),
                                 static_cast<unsigned>(sr.msg.skill_type));
                    } else {
                        sr.state = SkillState::ENTRY_PENDING;
                        sr.state_enter_time = now;
                        sr.entry_gate_enter_time = now;
                        sr.last_event = "TRANSIT to ENTRY_PENDING (arrive last point reached)";
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s to ENTRY_PENDING",
                                 current_skill_index_, sr.msg.skill_id.c_str());
                    }
                }
                break;
            }

            case SkillState::ENTRY_PENDING: {
                // 门控:executor 报告 phase=SKILL_AREA(或 phase=COMPLETE — 同点 catch 不到 SKILL_AREA 的补救)
                //   - 正常情况:ARRIVE → SKILL_AREA → COMPLETE,mission_manager 看到 SKILL_AREA 时开门
                //   - 同点场景:ARRIVE/SKILL_AREA 同一坐标,executor 20ms 内 SKILL_AREA 跳过,
                //     mission_manager 的 10Hz tick 可能错过 SKILL_AREA → 改判 phase=COMPLETE 也开门
                //   - skill_area_path 空:line 2247 早就 TRANSIT → IN_TASK 跳过 ENTRY_PENDING,走不到这里
                bool in_skill_area = (latest_wp_status_ &&
                                      (latest_wp_status_->phase == multi_uav_strike::WaypointStatus::PHASE_SKILL_AREA ||
                                       latest_wp_status_->phase == multi_uav_strike::WaypointStatus::PHASE_COMPLETE) &&
                                      latest_wp_status_->skill_id == sr.msg.skill_id);
                if (in_skill_area) {
                    sr.state = SkillState::IN_TASK;
                    sr.state_enter_time = now;
                    sr.in_task_enter_time = now;
                    if (sr.msg.skill_type == 101) {
                        // 101 集结合:记录进入时刻,等待其他 UAV 或 timeout
                        sr.gather_enter_time = now;
                    }
                    // 修复:进 skill_area 时保持 PHASE_WAYPOINT_FOLLOW,不再提前切到 GUIDANCE_*
                    // 之前这里按 work_mode 提前切 phase,导致航点跟踪阶段 phase=GUIDANCE_*
                    // 但 guidance 还没 enable(YOLO 还没命中),状态错位;且 handleSearch*
                    // 一旦 is_locked 误置 true 就会立即调 startGuidanceApproach()。
                    // phase → GUIDANCE_* 的转换由 startGuidanceApproach() 在 YOLO 命中
                    // 实际启动 guidance 时切(见 startGuidanceApproach 头部)。

                    // === Attack(105) 语义:work_mode 必须为 SEARCH_STRIKE 才能触发 strike ===
                    //   taskFlowCallback 遵守"work_mode 只由 SET_WORKMODE 控制"的协议约定不主动切;
                    //   但 Attack(105) 进 IN_TASK 后 clusterStatesCallback 派发必须走 SEARCH_STRIKE 分支,
                    //   否则 YOLO 命中只会上报 type=3、不触发 strike、UAV 沿 waypoint 飞完不制导。
                    //   这点与既有的 Attack 协同分支(line 1858-1868 强制 guidance mode="strike")同源:
                    //   协议上 work_mode 是意图通道,Attack 任务语义上等同于"在任务区内做的事"=SEARCH_STRIKE。
                    if (sr.msg.skill_type == 105 && current_work_mode_ != WorkMode::SEARCH_STRIKE) {
                        WorkMode old_mode = current_work_mode_;
                        current_work_mode_ = WorkMode::SEARCH_STRIKE;
                        ROS_WARN("[MissionManager] Attack(105) skill IN_TASK: promoting work_mode %s -> SEARCH_STRIKE",
                                 workModeToString(old_mode).c_str());
                    }
                    // === 102 Search IN_TASK 入口(2026-07-29):下发 gimbal_pitch ===
                    //   同 TRANSIT→IN_TASK(skill_area 空)分支,这里覆盖"非空 skill_area"主路径。
                    //   放在 work_mode promote 之后,顺序无关,只为保证 state 切完再 publish。
                    if (sr.msg.skill_type == 102) {
                        current_gimbal_pitch_ = sr.msg.gimbal_pitch;
                        ROS_WARN("[MissionManager] Search(102) IN_TASK: gimbal_pitch -> %.2f deg "
                                 "(will publish on /<ns>/mission/gimbal_command @%.1fHz)",
                                 current_gimbal_pitch_, gimbal_command_rate_hz_);
                    }

                    sr.last_event = "ENTRY_PENDING to IN_TASK";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to IN_TASK (type=%u, phase=%s)",
                             current_skill_index_, sr.msg.skill_id.c_str(),
                             static_cast<unsigned>(sr.msg.skill_type),
                             missionPhaseToString().c_str());
                } else if (entry_gate_timeout_ > 0.0 &&
                           (now - sr.entry_gate_enter_time).toSec() > entry_gate_timeout_) {
                    sr.state = SkillState::FAILED;
                    sr.state_enter_time = now;
                    sr.last_event = "ENTRY_PENDING to FAILED (timeout)";
                    ROS_ERROR("[MissionManager] Skill[%zu] id=%s entry gate TIMEOUT (%.1fs)",
                              current_skill_index_, sr.msg.skill_id.c_str(),
                              (now - sr.entry_gate_enter_time).toSec());
                }
                break;
            }

            case SkillState::IN_TASK: {
                // === cluster_states 10Hz 快照驱动的 strike/lock 决策 ===
                // 替代原 checkYoloDrivenStrike (后者在 timer 里轮询 latest_yolo_ 做 100Hz→N Hz 的去重判断,
                // 现由 target_estimator 做聚类后,周期 publish cluster_states,
                // clusterStatesCallback 已经在这里完成 SEARCH_STRIKE → triggerStrike()
                //   以及 SEARCH_TRACK → 锁目标 + tracking_state 上报。
                // IN_TASK 分支不再需要主动轮询 strike/lock 决策,只等 attack_cmd 处理 SEARCH_TRACK 的确认。
                // (代码保留注释,便于将来在此处加技能超时/计数等通用逻辑。)

                // === Attack(105) sticky 防御性 promote:每 tick 强制 work_mode=SEARCH_STRIKE ===
                //   背景:ENTRY_PENDING→IN_TASK 一次性 promote(4209 行)只在 transition 时跑一次。
                //     - GS 通过 typedWorkModeCallback 持续发 SET_WORKMODE=SEARCH_ONLY 会被覆盖回去,
                //       1Hz topic /uav0/mission/current_work_mode 又会显示 mode:3
                //     - 用户 SITL 实测 2026-07-26:Topic 一直是 3,clusterStatesCallback 派发走 SEARCH_ONLY 分支,
                //       YOLO 命中只 type=3,从不 strike(就反馈"一直 search_only,没切 search_strike")
                //   fix:在 IN_TASK case 头部,每 10Hz tick 强制 current_work_mode_=SEARCH_STRIKE,
                //     让 1Hz publish 总能赶上最新值,即便 GS 同 tick 后发 SEARCH_ONLY 也会被下一拍
                //     10Hz skillAdvanceTimer 重新升回 SEARCH_STRIKE(topic 100ms 内刷新)。
                //   协议注释:这条强制 promote 与 typedWorkModeCallback 在 GS 显式 SET_WORKMODE=IDLE
                //     时仍然冲突,但 IDLE 表示"退出任务",Attack(105) IN_TASK 期间 GS 不应发 IDLE。
                if (sr.msg.skill_type == 105 && current_work_mode_ != WorkMode::SEARCH_STRIKE) {
                    WorkMode old_mode = current_work_mode_;
                    current_work_mode_ = WorkMode::SEARCH_STRIKE;
                    ROS_WARN_THROTTLE(5.0, "[MissionManager] Attack(105) IN_TASK sticky promote: "
                                          "work_mode %s -> SEARCH_STRIKE",
                                          workModeToString(old_mode).c_str());
                }

                // === Attack(105) WP1 到达上升沿检测(2026-07-30)===
                //   skill_idx >= 1 表示 UAV 已到达 skill_area_path 第一条(WP1)。
                //   触发后 attack_wp2_look_delay_sec_ 秒,云台开始朝 WP2 看
                //   (逻辑在 gimbalCommandTimerCallback)。
                //   上升沿:首次进入时打点,后续每 tick 不再打。
                if (sr.msg.skill_type == 105 && !attack_wp1_reached_ &&
                    latest_wp_status_ && latest_wp_status_->skill_id == sr.msg.skill_id &&
                    latest_wp_status_->skill_total > 0 && latest_wp_status_->skill_idx >= 1) {
                    attack_wp1_reached_      = true;
                    attack_wp1_reached_time_ = now;
                    ROS_WARN("[MissionManager] Attack(105) WP1 reached (skill_idx=%u), "
                             "gimbal will look at WP2 after %.1fs",
                             latest_wp_status_->skill_idx, attack_wp2_look_delay_sec_);
                }

                // skill_type=101 集结合:本机先到点进入 HOLDING,再等伙伴 HOLDING 或 timeout
                if (sr.msg.skill_type == 101) {
                    // 本机已飞到 skill_area_path 最后一个航点。
                    // skill_total>0 防护:GATHER 应至少有 1 条 skill_area,空路径靠 timeout 兜底。
                    const bool local_arrived = latest_wp_status_ &&
                        latest_wp_status_->skill_id == sr.msg.skill_id &&
                        latest_wp_status_->skill_total > 0 &&
                        latest_wp_status_->skill_idx >= latest_wp_status_->skill_total;
                    const bool has_remote_partners = std::any_of(
                        sr.gather_partners_.begin(), sr.gather_partners_.end(),
                        [&](const std::string& sn) { return sn != self_uav_device_sn_; });

                    // 多机屏障第一阶段:本机到点后保持原位,并通过 InterUavStatus.phase=HOLDING
                    // 广播“已到达”。不能因本机到点就立即 COMPLETE,否则伙伴来不及观察 HOLDING。
                    if (has_remote_partners && local_arrived &&
                        sr.gather_local_arrived_time.isZero()) {
                        sr.gather_local_arrived_time = now;
                        enterHoldState();
                        sr.last_event = "IN_TASK: local UAV arrived, waiting for gather partners";
                        ROS_WARN("[MissionManager] Skill[101] Local UAV arrived; HOLDING for %zu partners",
                                 sr.gather_partners_.size());
                    }

                    // MissionState 和 InterUavStatus 都是周期链路。至少保持一个伙伴心跳超时窗,
                    // 确保本机 HOLDING 已经被伙伴看到,再允许通过分布式屏障。
                    const bool local_holding_announced = local_arrived &&
                        !sr.gather_local_arrived_time.isZero() &&
                        (now - sr.gather_local_arrived_time).toSec() >= partner_heartbeat_timeout_;
                    const bool partners_arrived = has_remote_partners &&
                        local_holding_announced &&
                        checkAllUavsGathered(sr.gather_partners_);
                    const bool timed_out = gather_timeout_ > 0.0 &&
                        (now - sr.gather_enter_time).toSec() > gather_timeout_;
                    const bool gather_complete = has_remote_partners ?
                        partners_arrived : local_arrived;

                    if (gather_complete || timed_out) {
                        sr.state = SkillState::EXIT_PENDING;
                        sr.state_enter_time = now;
                        sr.last_event = timed_out ?
                            "IN_TASK to EXIT_PENDING (gather timeout)" :
                            has_remote_partners ?
                                "IN_TASK to EXIT_PENDING (all UAVs holding)" :
                                "IN_TASK to EXIT_PENDING (single UAV at gather point)";
                        ROS_WARN("[MissionManager] Skill[101] Gather %s (partners=%zu, threshold=%.2f)",
                                 timed_out ? "TIMEOUT" :
                                 has_remote_partners ? "COMPLETE (all UAVs HOLDING)" :
                                                       "ARRIVED (single UAV)",
                                 sr.gather_partners_.size(), gather_threshold_);
                    }
                    break;
                }

                // 其它 skill_type:门控 = skill_idx 走完 (idx >= total),不限定 phase
                // 修复:原来是 skill_idx + 1 >= skill_total,会在 idx = total - 1 (即飞机刚开始
                //   飞向最后一个 skill_area 航点)时就触发,导致 enterHoldState 在中途捕获 pose,
                //   飞机继续飞到末点后又飞回捕获点。改为 idx >= total,等最后一个 skill_area 航点
                //   *到达* 才推进。此时 phase 已切到 SKILL_AREA(或 COMPLETE,无 skill_area),但 idx >= total
                //   仍成立。
                if (!latest_wp_status_) break;
                if (latest_wp_status_->skill_idx >= latest_wp_status_->skill_total &&
                    latest_wp_status_->skill_id == sr.msg.skill_id) {
                    // === Attack(105) 专用:到达 attack_point 不立即 EXIT_PENDING(2026-07-26 修复) ===
                    //   旧行为:UAV 踩到 attack_point 立刻 EXIT_PENDING → COMPLETE。
                    //     此时目标转头刚进视野,target_estimator→YOLO→clusterStatesCallback→
                    //     triggerStrike() 还没拿到第一帧就被收尾,YOLO 后续命中只发 type=3
                    //     (cluster_states_callback pending 路径)然后 skill 整体结束,UAV 转向下一
                    //     skill 或回 hold,完全错过 strike 时机。
                    //   新行为:在 attack_point 锁住一帧,沿用已有 trigger 链路:
                    //     - 记录 attack_arrival_time_(partner 据此算 strike_sync_time 不变)
                    //     - 记录 attack_hover_enter_time_(mission_manager 自己兜底超时用)
                    //     - 切 attack_phase_ = SEARCHING(沿用 search 阶段的目标检测/聚类)
                    //     - 起 setpoint publisher 在 attack_point 锁 XY+Z(详见下面 inline 说明)
                    //     - **不**转换 SkillState,保持在 IN_TASK
                    //   strike 真触发由 clusterStatesCallback(SEARCH_STRIKE 分支)+ advanceAttackPhaseMachine
                    //     完成,本分支不直接调 triggerStrike()。
                    //   退出路径见本 if 块之后的两段:(a) attack_hover_timeout 兜底;(b) 已有 attack_strike_pending_timeout 兜底。
                    if (sr.msg.skill_type == 105 &&
                        sr.attack_arrival_time_.isZero()) {
                        sr.attack_arrival_time_     = now;
                        sr.attack_hover_enter_time_ = now;
                        sr.attack_phase_            = SkillRuntime::AttackPhase::SEARCHING;

                        // 起 setpoint publisher 锁位置在 attack_point,避免 OFFBOARD 失流 →
                        //   PX4 COM_OFFBOARD_LOSS_TIMEOUT → AUTO.LAND failsafe。
                        //   复用 startSetpointPublisher()/takeoff_setpoint_ 既有机制,
                        //   此时 UAV 刚踩到 attack_point,current_pose_ 即是该处,直接 latch。
                        //   注意:**不**改 current_phase_,与 enterHoldState() 在此处的区别。
                        if (!setpoint_running_ && is_pose_received_) {
                            startSetpointPublisher();
                            takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // y_ned → x_enu (east)
                            takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // x_ned → y_enu (north)
                            takeoff_setpoint_.pose.position.z = -current_pose_.pose.position.z; // 当前高度
                            attack_hover_setpoint_latched_ = true;
                            ROS_WARN("[MissionManager] Attack(105) setpoint publisher latched at attack_point "
                                     "(pos=%.2f,%.2f, alt=%.2f)",
                                     takeoff_setpoint_.pose.position.x,
                                     takeoff_setpoint_.pose.position.y,
                                     takeoff_setpoint_.pose.position.z);
                        }

                        ROS_WARN("[MissionManager] Attack(105) arrived at attack_point ts=%.3f → "
                                 "HOVERING for strike trigger (hover_timeout=%.1fs, pending=%s)",
                                 sr.attack_arrival_time_.toSec(), attack_hover_timeout_sec_,
                                 attack_strike_pending_ ? "true" : "false");
                    }
                    // Attack(105) 不进 EXIT_PENDING(已在上面分支完成 hover 起步);其余 skill_type 沿用旧行为
                    if (sr.msg.skill_type != 105) {
                        // === 102 Search IN_TASK 出口(2026-07-29):gimbal 归位 ===
                        //   skill_area_path 走完,自然收尾:把 current_gimbal_pitch_ 复位到
                        //   gimbal_pitch_default_(默认 0,水平),gimbal_command_timer 继续 5Hz
                        //   把"水平"姿态推下去,gimbal_control_node 收到后会回正视线。
                        if (sr.msg.skill_type == 102) {
                            ROS_WARN("[MissionManager] Search(102) EXIT_PENDING: gimbal_pitch %.2f -> default %.2f deg",
                                     current_gimbal_pitch_, gimbal_pitch_default_);
                            current_gimbal_pitch_ = gimbal_pitch_default_;
                        }
                        sr.state = SkillState::EXIT_PENDING;
                        sr.state_enter_time = now;
                        sr.last_event = "IN_TASK to EXIT_PENDING (skill area done)";
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s to EXIT_PENDING",
                                 current_skill_index_, sr.msg.skill_id.c_str());
                    }
                }

                // === Attack(105) 等待 strike trigger 兜底超时(2026-07-26) ===
                //   覆盖"到 attack_point 始终没看到目标"的场景,与 attack_strike_pending_timeout_sec_
                //   在 advanceAttackPhaseMachine 中的 60s 兜底互为补充:
                //     - 本超时:即使完全没设过 attack_strike_pending_(没收到任何 cluster)也跑;
                //       设置后无敌触发,就放弃。
                //     - 已有兜底:只在 attack_strike_pending_ 已 true 但 gating 一直 WAITING 时跑。
                //   优先级:此处 EXIT_PENDING(攻击放弃)在 in_task_timeout → FAILED 之前;若 in_task_timeout_
                //   同时爆,FAILED 覆盖(更严重,符合"内层 timeout 被外层 timeout 顶上"的语义)。
                if (sr.msg.skill_type == 105 &&
                    !sr.attack_hover_enter_time_.isZero()) {
                    // === 2026-07-30: 双超时语义 ===
                    //   - 见过 WP2 范围内的目标(attack_wp2_target_seen_=true):
                    //       30s 兜底,防 stale 后一直挂。
                    //   - 从未见过 WP2 范围内的目标:"确实没目标就悬停"语义,
                    //       长超时 attack_no_target_hold_sec_(默认 300s),给仿真更多时间看清。
                    const double hover_timeout = attack_wp2_target_seen_
                        ? attack_hover_timeout_sec_
                        : attack_no_target_hold_sec_;
                    if (hover_timeout > 0.0 &&
                        (now - sr.attack_hover_enter_time_).toSec() > hover_timeout) {
                        sr.attack_phase_      = SkillRuntime::AttackPhase::ABANDONED;
                        sr.state               = SkillState::EXIT_PENDING;
                        sr.state_enter_time    = now;
                        sr.last_event          = attack_wp2_target_seen_
                            ? "IN_TASK to EXIT_PENDING (attack hover timeout)"
                            : "IN_TASK to EXIT_PENDING (attack no-target hold timeout)";
                        // 结束 hover:清 latched 标志并停 setpoint publisher。
                        //   此时 phase=WAYPOINT_FOLLOW,runPx4TakeoffSequence 下个 tick
                        //   看到标志为 false 才会 stop(否则会因 phase 不对不停,后续 skill
                        //   接管时残留 publisher 流会冲突)。
                        attack_hover_setpoint_latched_ = false;
                        if (setpoint_running_) {
                            stopSetpointPublisher();
                        }
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s Attack(105) HOVER TIMEOUT "
                                 "(%.1fs > %.1fs, wp2_seen=%s), ABANDONED (pending=%s)",
                                 current_skill_index_, sr.msg.skill_id.c_str(),
                                 (now - sr.attack_hover_enter_time_).toSec(),
                                 hover_timeout,
                                 attack_wp2_target_seen_ ? "true" : "false",
                                 attack_strike_pending_ ? "true" : "false");
                    }
                }

                // in_task_timeout 检查
                if (in_task_timeout_ > 0.0 &&
                    (now - sr.in_task_enter_time).toSec() > in_task_timeout_) {
                    // === 102 Search IN_TASK 超时(2026-07-29):gimbal 同样归位 ===
                    //   与上面 IN_TASK→EXIT_PENDING(skill area done)同语义,
                    //   只是这里是 timeout 强切 FAILED,云台也要回正避免下一条 skill 进来时
                    //   gimbal 还卡在原搜索俯仰角。
                    if (sr.msg.skill_type == 102) {
                        ROS_WARN("[MissionManager] Search(102) IN_TASK TIMEOUT: gimbal_pitch %.2f -> default %.2f deg",
                                 current_gimbal_pitch_, gimbal_pitch_default_);
                        current_gimbal_pitch_ = gimbal_pitch_default_;
                    }
                    sr.state = SkillState::FAILED;
                    sr.state_enter_time = now;
                    sr.last_event = "IN_TASK to FAILED (timeout)";
                    ROS_ERROR("[MissionManager] Skill[%zu] id=%s in_task TIMEOUT (%.1fs)",
                              current_skill_index_, sr.msg.skill_id.c_str(),
                              (now - sr.in_task_enter_time).toSec());
                }
                break;
            }

            case SkillState::EXIT_PENDING: {
                // 收尾一帧后 → COMPLETE
                if ((now - sr.state_enter_time).toSec() > 0.1) {
                    sr.state = SkillState::COMPLETE;
                    sr.state_enter_time = now;
                    sr.last_event = "EXIT_PENDING to COMPLETE";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to COMPLETE",
                             current_skill_index_, sr.msg.skill_id.c_str());
                }
                break;
            }

            case SkillState::COMPLETE: {
                // 推下一个 skill
                if (current_skill_index_ + 1 < skill_queue_.size()) {
                    current_skill_index_++;
                    // 注意:不再根据 skill_type 自动切 work_mode(违反协议语义:work_mode 只由 SET_WORKMODE 控制)
                    // 推进后调用 pushCurrentSkillToExecutor() 推到 waypoint_executor 即可
                    pushCurrentSkillToExecutor();
                    ROS_WARN("[MissionManager] >>>> Advancing to Skill[%zu]/%zu (type=%u, work_mode 保持 %s)",
                             current_skill_index_, skill_queue_.size(),
                             skill_queue_[current_skill_index_].msg.skill_type,
                             workModeToString().c_str());
                } else {
                    // 全部完成 — 检查最后一个 skill 是否是 Return (skill_type=103)
                    //   是 → 触发 PX4 AUTO.LAND 降落 (PX4 着陆后自动 disarm)
                    //   否 → 进入 PHASE_HOLDING, 原地+当前航向悬停等新 task_flow
                    const auto& last_skill = skill_queue_.back().msg;
                    if (last_skill.skill_type == 103) {
                        // Return skill:触发一次 AUTO.LAND;已在降落流程或已触发过则幂等空操作,
                        // 绝不能掉进 enterHoldState()——那会在 disarm 后重启贴地 setpoint 流,
                        // 并把 phase 写成 HOLDING 被后续误判为飞手接管(见 mavrosStateCallback)。
                        // landing_already_triggered_ 守卫:disarm → is_landing_in_progress_=false
                        // 后,10Hz 状态机会反复重入此分支,防止重复触发+重复打印。
                        if (!is_landing_in_progress_ && !landing_already_triggered_) {
                            ROS_WARN("[MissionManager] >>>> Last skill is Return (103) "
                                     " triggering PX4 AUTO.LAND (safety_timeout=%.1fs)",
                                     landing_safety_timeout_sec_);
                            triggerPx4Landing();
                        }
                        // else: 降落进行中或已触发过,等 checkLandingComplete / mavrosStateCallback 收尾
                    } else {
                        ROS_WARN_THROTTLE(5.0, "[MissionManager] >>>> All skills COMPLETE (%zu total) to PHASE_HOLDING",
                                 skill_queue_.size());
                        enterHoldState();
                    }
                }
                break;
            }

            case SkillState::FAILED: {
                // 失败 — 停在当前 skill,等用户手动干预(IDLE 模式 → 重新下发)
                ROS_ERROR_THROTTLE(5.0, "[MissionManager] Skill[%zu] id=%s FAILED, last_event=%s",
                                   current_skill_index_, sr.msg.skill_id.c_str(),
                                   sr.last_event.c_str());
                break;
            }
        }
    }

    /**
     * Attack(105) 专用 tick — 与 advanceSkillStateMachine 并行调用:
     *   1) formation=2 时定期根据 partner 最新位置重算 strike_sync_time
     *   2) clusterStatesCallback 因门控挂起的 strike 在 READY 时补 trigger
     *   3) 已 strike / 已 abandon 后:清挂起标记,防重复
     *
     * 不在这里改 phase(改 phase 由 triggerStrike → startGuidanceApproach 负责)。
     */
    void advanceAttackPhaseMachine() {
        if (current_skill_index_ >= skill_queue_.size()) return;
        SkillRuntime& sr = skill_queue_[current_skill_index_];
        if (sr.msg.skill_type != 105) {
            attack_strike_pending_ = false;  // 非 Attack 不挂起
            return;
        }
        // 1) formation=2:周期重算 strike_sync_time
        if (sr.msg.formation == 2 && !sr.attack_partners_.empty()) {
            recomputeCoordinatedStrikeSync(sr);
        }
        // 2) 门控通过的 strike:补 triggerStrike
        if (attack_strike_pending_) {
            auto gating = checkAttackStrikeGating(sr);
            if (gating == AttackStrikeGating::READY) {
                ROS_WARN("[MissionManager] Attack(105) gating cleared → "
                         "firing pending strike (cluster=%lu)",
                         (unsigned long)attack_strike_pending_cluster_id_);
                attack_strike_pending_            = false;
                attack_strike_pending_cluster_id_ = 0;
                // 已 locked_cluster_* 在 clusterStatesCallback 挂起时已缓存,直接 trigger
                triggerStrike();
            } else {
                // 3) 挂起超时(默认 60s):放弃当前 strike,转 ABANDONED
                if (!attack_strike_pending_time_.isZero() &&
                    attack_strike_pending_time_ != ros::Time() &&
                    (ros::Time::now() - attack_strike_pending_time_).toSec() >
                        attack_strike_pending_timeout_sec_) {
                    ROS_WARN("[MissionManager] Attack(105) strike pending TIMEOUT "
                             "(%.1fs > %.1fs), abandoning",
                             (ros::Time::now() - attack_strike_pending_time_).toSec(),
                             attack_strike_pending_timeout_sec_);
                    attack_strike_pending_            = false;
                    attack_strike_pending_cluster_id_ = 0;
                    sr.attack_phase_                  = SkillRuntime::AttackPhase::ABANDONED;
                    // 推进当前 skill
                    if (sr.state == SkillState::IN_TASK) {
                        sr.state = SkillState::EXIT_PENDING;
                        sr.state_enter_time = ros::Time::now();
                        sr.last_event = "IN_TASK to EXIT_PENDING (attack abandon)";
                        // 结束 hover:同上,清标志停 publisher
                        attack_hover_setpoint_latched_ = false;
                        if (setpoint_running_) {
                            stopSetpointPublisher();
                        }
                    }
                }
            }
        }
    }

    /**
     * Attack(105) formation=2 coordinated:根据 partner 最新位置重算 strike_sync_time
     *   - 每架 UAV 按当前 bearing → 目标距离 / task_speed 算飞行时间
     *   - strike_sync_time = now + max(飞行时间) + buffer
     *   - partner 位置用最近心跳;心跳未到的 partner 跳过(保留旧预估)
     * 仅在 formation=2 + IN_TASK + 已规划过一次时调用(planCoordinatedAttack 做过初始化)。
     */
    void recomputeCoordinatedStrikeSync(SkillRuntime& sr) {
        if (sr.attack_strike_sync_time_.isZero()) return;
        if (sr.msg.skill_area_path.poses.empty()) return;
        const auto& wp2 = sr.msg.skill_area_path.poses.back();
        const double tgt_lat = wp2.pose.position.x;
        const double tgt_lon = wp2.pose.position.y;
        const double speed   = std::max(static_cast<double>(sr.msg.task_speed), 1.0);
        double max_flight_time = 0.0;
        // 本机
        {
            double dx_n = gpsDeltaN(current_gps_.latitude, tgt_lat);
            double dx_e = gpsDeltaE(current_gps_.longitude, tgt_lon, current_gps_.latitude);
            double d = std::hypot(dx_n, dx_e);
            double t = d / speed;
            if (t > max_flight_time) max_flight_time = t;
        }
        // partners
        for (const auto& sn : sr.attack_partners_) {
            auto hb = findHeartbeat(sn);
            if (!hb || hb->lat == 0.0 || hb->lon == 0.0) continue;
            double dx_n = gpsDeltaN(hb->lat, tgt_lat);
            double dx_e = gpsDeltaE(hb->lon, tgt_lon, hb->lat);
            double d = std::hypot(dx_n, dx_e);
            double t = d / speed;
            if (t > max_flight_time) max_flight_time = t;
        }
        ros::Time new_sync = ros::Time::now() +
            ros::Duration(max_flight_time + attack_sync_strike_buffer_sec_);
        // 仅在显著变化时更新(避免每 tick 微小波动都写)
        if ((new_sync - sr.attack_strike_sync_time_).toSec() > 0.05 ||
            (sr.attack_strike_sync_time_ - new_sync).toSec() > 0.05) {
            sr.attack_strike_sync_time_ = new_sync;
        }
    }

    /**
     * 101 集结 — 用 InterUavStatus 实现多 UAV 到达屏障
     * 参数:gather_partners — 本条 gather skill 的期望设备 SN 列表
     *   空或仅含本机 → 单机路径,返回 false(由 local_arrived 决定完成)
     *   含其它设备 → 遍历 partner_heartbeats_:
     *     - 伙伴 SN 有未超时的心跳(默认 3s)
     *     - 伙伴在当前 gather skill 的 gather_partner_holding_seen_ 出现过
     *       (即至少见过一次 phase=HOLDING — 不要求当前仍在 HOLDING)
     *   到达伙伴数 / 期望伙伴数(去本机) >= gather_threshold_ 时返回 true。
     *
     * 本机是否到达由调用方单独检查;不同 UAV 的 skill_id 可以不同,因此这里只按 SN 和 seen 标记判断。
     *
     * === 2026-07-24 关键变更 ===
     * 原实现判 partner_heartbeats_[sn].latest->phase == HOLDING → 只能等"伙伴正在 HOLDING"。
     *   后果:2 架先到 → 走 → phase 切到下一阶段(WAYPOINT_FOLLOW/GUIDANCE_*)→ 第 3 架永远等不到。
     * 改为查 gather_partner_holding_seen_(interUavStatusCallback 里只要见过 phase=HOLDING
     *   就一次性写入,map::insert 不覆盖)→ 伙伴走过 HOLDING 后即使离开也算"到过"。
     * 函数签名从 const 改为非 const,因为要访问 skill_queue_[current_skill_index_] 的 seen map。
     */
    bool checkAllUavsGathered(const std::vector<std::string>& gather_partners) {
        if (gather_partners.empty()) {
            // 单机/未配置多机 → 让 single_uav_arrived 处理
            return false;
        }
        ros::Time now = ros::Time::now();
        size_t arrived_count = 0;
        size_t valid_partners = 0;
        // === 2026-07-24: 用 device_sn 而非 namespace 过滤"自己" ===
        // 之前用 nh_.getNamespace()("uav0")去和 gather_partners_("1-1"...)比对 → 永远不命中 → 自己总被算
        // 修复:partner 名单语义是设备 SN(GS 在 Skill.params_json["devices_sn"] 注入的也是 SN),
        //   InterUavStatus.msg 里填的也是 device_sn_,两边单位一致 → 用 self_uav_device_sn_ 过滤。
        // 注:self_uav_device_sn_ 由 launch ~uav_device_sn 注入,空 → 退化为"不过滤"(原 buggy 行为),
        //   真机多机测试 launch 都用 TZS_DEVICE_SN 设了 1-1/1-2/1-3,这里没问题。
        // 注 2:launch 默认="$(optenv TZS_DEVICE_SN 1-1)",改 launch 后本字段有值,过滤生效。
        const std::string& my_sn = self_uav_device_sn_;

        // 拿到当前 gather skill 的 seen map(避免空指针)。
        // skill_queue_ 可能在 task_flow 切换时被 clear,所以先判 index 合法。
        // skill_queue_[i].msg.skill_type==101 双重确认(若当前是 takeoff 等其它 skill,
        //   不应该用 gather 的 seen map — 但 IN_TASK 分支只在 skill_type==101 时才进,
        //   所以这里也只在该条件下取 seen map)。
        const std::map<std::string, ros::Time>* seen_map_ptr = nullptr;
        if (current_skill_index_ < skill_queue_.size()) {
            const auto& sr_ref = skill_queue_[current_skill_index_];
            if (sr_ref.msg.skill_type == 101) {
                seen_map_ptr = &sr_ref.gather_partner_holding_seen_;
            }
        }

        for (const auto& sn : gather_partners) {
            if (sn == my_sn) continue;  // 跳过自己(SN 匹配)
            ++valid_partners;
            // 心跳是否新鲜(防止伙伴断网后 stale 残留被错算"到过")
            auto it = partner_heartbeats_.find(sn);
            if (it == partner_heartbeats_.end()) continue;
            const auto& ph = it->second;
            if (!ph.latest) continue;
            if ((now - ph.last_update).toSec() > partner_heartbeat_timeout_) continue;
            // === 2026-07-24: 改为一次性 seen 标记 ===
            // 只要此伙伴在 seen map 里出现过(任何时刻)→ 算到达。
            // 离开 HOLDING 后 phase 变了,但 map 不会被清,屏障可正确通过。
            if (seen_map_ptr != nullptr &&
                seen_map_ptr->find(sn) != seen_map_ptr->end()) {
                ++arrived_count;
            }
        }
        if (valid_partners == 0) {
            return false;  // 名单里全是本机 → 等同单机路径
        }
        double ratio = static_cast<double>(arrived_count) / static_cast<double>(valid_partners);
        return ratio >= gather_threshold_;
    }

    // ============== 任务处理 ==============

    void handleIdle() {
        current_task_status_ = TaskStatus::IDLE;
    }

    void handleSearchOnly() {
        // 全图搜索模式
        current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;

        // 目标检测后只回传，不做其他处理
        if (current_target_.is_detected && !current_target_.is_shared) {
            reportTargetToGs();
            current_target_.is_shared = true;
        }
    }

    void handleSearchTrack() {
        // TARGET_LOST_HOVER 必须由 advanceSkillStateMachine 的 10s 恢复计时管理。
        // 禁止本 handler 在进入 hover 的同一 tick 因 is_guidance_active_=false
        // 立即清锁并 resume，从而绕过悬停等待。
        if (target_lost_hover_active_) return;
        // 搜索即跟踪模式
        // 如果已启动制导，不应再设置 waypoint 模式
        if (is_guidance_active_) {
            return;
        }

        if (!current_target_.is_locked) {
            // 目标未锁定，执行航点
            current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
        } else {
            // === Stale-locked 守卫(2026-07-26)===
            //   场景:target_lost_hold 触发 disableGuidance(),但 current_target_.is_locked
            //     没被清(clusterStatesCallback 锁定时设的,disableGuidance 不改它)。
            //     下一个 tick handleSearchTrack 看到 is_guidance_active_=false +
            //     is_locked=true → 反复 startGuidanceApproach(),触发 100ms 一次的
            //     GUIDANCE_TRACK ↔ TARGET_LOST_HOVER ping-pong,且 guidance 仍按
            //     stale pose 给 setpoint → UAV 飞回旧位置("倒退")。
            //   守门:is_locked 但 last_seen_us 已老 → 主动 disable + 清锁,
            //     让 waypoint 接管,与 clusterStatesCallback 派发前的 stale gate 互补。
            //   阈值 2026-07-26:用 target_lost_hold_sec_(默认 2.0s)代替 max_strike_cluster_age_sec_(0.5s),
            //     让 stale-locked guard 跟 advanceSkillStateMachine 共用同一判据和阈值,
            //     避免 0.5s vs 2.0s 两个 guard 在同一目标的 last_seen 上互相干扰。
            if (locked_cluster_last_seen_us_ != 0) {
                const double last_seen_age_sec =
                    (ros::Time::now().toNSec() / 1000ULL - locked_cluster_last_seen_us_) / 1e6;
                if (last_seen_age_sec > target_lost_hold_sec_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] handleSearchTrack: locked target "
                                      "stale (last_seen_age=%.2fs > hold=%.2fs, cluster=%lu) "
                                      "— disable guidance, clear lock, resume waypoint",
                                      last_seen_age_sec, target_lost_hold_sec_,
                                      static_cast<unsigned long>(locked_cluster_id_));
                    disableGuidance();
                    current_target_.is_locked   = false;
                    current_target_.is_detected = false;
                    current_target_.is_shared   = false;
                    locked_cluster_id_            = 0;
                    locked_cluster_time_          = ros::Time();
                    locked_cluster_last_seen_us_  = 0;
                    tracked_target_.is_valid      = false;
                    setLockState(TrackLockState::NOT_LOCKED);
                    // 重启 waypoint_executor(若它还在 stop 状态)
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                    current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                    current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
                    publishPrimaryTarget(0);
                    return;
                }
            }
            // 目标已锁定，直接启动制导接近
            startGuidanceApproach();
        }
    }

    void handleSearchStrike() {
        // 同 handleSearchTrack：目标丢失悬停期间不能立即恢复航点。
        if (target_lost_hover_active_) return;
        // 搜索即打击模式
        // 如果已启动制导，不应再设置 waypoint 模式
        if (is_guidance_active_) {
            return;
        }

        if (!current_target_.is_locked) {
            // 目标未锁定，执行航点
            current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
        } else {
            // === Stale-locked 守卫(2026-07-26),与 handleSearchTrack 同因同解 ===
            //   阈值统一用 target_lost_hold_sec_(默认 2.0s)代替 max_strike_cluster_age_sec_(0.5s)。
            if (locked_cluster_last_seen_us_ != 0) {
                const double last_seen_age_sec =
                    (ros::Time::now().toNSec() / 1000ULL - locked_cluster_last_seen_us_) / 1e6;
                if (last_seen_age_sec > target_lost_hold_sec_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] handleSearchStrike: locked target "
                                      "stale (last_seen_age=%.2fs > hold=%.2fs, cluster=%lu) "
                                      "— disable guidance, clear lock, resume waypoint",
                                      last_seen_age_sec, target_lost_hold_sec_,
                                      static_cast<unsigned long>(locked_cluster_id_));
                    disableGuidance();
                    current_target_.is_locked   = false;
                    current_target_.is_detected = false;
                    current_target_.is_shared   = false;
                    locked_cluster_id_            = 0;
                    locked_cluster_time_          = ros::Time();
                    locked_cluster_last_seen_us_  = 0;
                    tracked_target_.is_valid      = false;
                    setLockState(TrackLockState::NOT_LOCKED);
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                    current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                    current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
                    publishPrimaryTarget(0);
                    return;
                }
            }
            // 目标已锁定，直接启动制导接近
            startGuidanceApproach();
        }
    }

    void handleDeniedEnvFlight() {
        // 拒止环境飞行(预留模式,未实现)
        // 未来扩展:
        //   - GPS 拒止场景下用视觉/VIO 替代 GPS 定位
        //   - 强对抗环境下主动规避雷达/激光锁定
        //   - 与电子战模块配合
        // 当前实现:仅维持 IDLE-like 行为(不执行 search/track/strike),等待后续 PR
        current_task_status_ = TaskStatus::IDLE;
        ROS_WARN_THROTTLE(10.0, "[MissionManager] DENIED_ENV_FLIGHT mode reserved (not implemented yet)");
    }

    void startSpiralApproach() {
        if (!current_target_.is_locked) {
            return;
        }

        is_spiral_active_ = true;
        current_task_status_ = TaskStatus::SPIRAL_APPROACH;

        // 停止航点执行
        std_msgs::String cmd;
        cmd.data = "stop";
        waypoint_control_pub_.publish(cmd);

        // 向 waypoint_executor 发送螺旋接近命令
        cmd.data = "spiral:" +
                   std::to_string(current_target_.pose.pose.position.x) + "," +
                   std::to_string(current_target_.pose.pose.position.y) + "," +
                   std::to_string(current_target_.pose.pose.position.z) + "," +
                   std::to_string(spiral_approach_radius_);
        waypoint_control_pub_.publish(cmd);

        ROS_INFO("[MissionManager] Starting spiral approach to target");
    }

    bool is_spiral_complete_ = false;

    void startGuidanceApproach() {
        is_guidance_active_ = true;
        current_task_status_ = TaskStatus::GUIDANCE_APPROACH;

        // 修复:实际启动 guidance 时再切 phase → GUIDANCE_*
        // (之前 advanceSkillStateMachine 进 IN_TASK 时就切了,航点跟踪阶段 phase 提前变,
        //  且 is_guidance_active 与 phase 错位)。现在 phase 切换与 guidance 启动同步:
        //  YOLO 命中 → triggerStrike → startGuidanceApproach → 切 phase + enable + stop
        // 三件事同时发生,GS 看到 phase=GUIDANCE_* 必定意味着 guidance 真的在跑。
        if (current_work_mode_ == WorkMode::SEARCH_STRIKE) {
            current_phase_ = MissionPhase::PHASE_GUIDANCE_STRIKE;
        } else if (current_work_mode_ == WorkMode::SEARCH_TRACK) {
            current_phase_ = MissionPhase::PHASE_GUIDANCE_TRACK;
        }
        ROS_WARN("[MissionManager] >>>>> phase -> %s on guidance start",
                 missionPhaseToString().c_str());

        // 停止航点执行，避免和 guidance 冲突
        ROS_WARN("[MissionManager] >>>>> stop command about to be published to waypoint_executor");
        std_msgs::String cmd;
        cmd.data = "stop";
        waypoint_control_pub_.publish(cmd);
        ROS_WARN("[MissionManager] >>>>> stop command published");

        // === 2026-07-30: 关 attack_hover 留下的 setpoint publisher,避免和 guidance 冲突 ===
        //   根因:UAV 到 attack_point (20m AGL) 时 mission_manager 会启动位置 setpoint publisher
        //     锁在 attack_point,防止 OFFBOARD 失流。strike 触发后 startGuidanceApproach() 只停了
        //     waypoint_executor,没停这个 publisher — PX4 OFFBOARD 收到持续的位置 setpoint
        //     会盖过 guidance 的 velocity 命令,UAV 仍悬停在 attack_point 上方,guidance 看似无效。
        //   修复:guidance 真启动时必须停掉 attack_hover setpoint,让 velocity 接管。
        //   注意:不调 disableGuidance()(它本来就只发 enable=false,不碰 setpoint)。
        if (attack_hover_setpoint_latched_ || setpoint_running_) {
            attack_hover_setpoint_latched_ = false;
            if (setpoint_running_) {
                stopSetpointPublisher();
            }
            ROS_WARN("[MissionManager] Stopped attack_hover setpoint publisher for guidance handoff");
        }

        // 确定模式：SEARCH_STRIKE -> "strike", SEARCH_TRACK -> "track"
        std::string guidance_mode = (current_work_mode_ == WorkMode::SEARCH_STRIKE) ? "strike" : "track";

        // 使能制导
        std_msgs::Bool enable;
        enable.data = true;
        guidance_enable_pub_.publish(enable);

        // 发送模式
        std_msgs::String mode_msg;
        mode_msg.data = guidance_mode;
        guidance_mode_pub_.publish(mode_msg);

        // 发送目标给制导,目前没有用
        guidance_target_pub_.publish(current_target_.pose);

        // 同步下发拦截速度(InterceptGuidance 用)
        std_msgs::Float32 speed_msg;
        speed_msg.data = static_cast<float>(guidance_speed_);
        guidance_speed_pub_.publish(speed_msg);

        ROS_WARN_THROTTLE(5.0, "[MissionManager] Starting guidance approach to target (mode: %s, guidance_speed_=%.2f)",
                          guidance_mode.c_str(), guidance_speed_);
    }

    void disableGuidance() {
        is_guidance_active_ = false;

        // 禁用制导
        std_msgs::Bool enable;
        enable.data = false;
        guidance_enable_pub_.publish(enable);

        ROS_INFO("[MissionManager] Guidance disabled for SEARCH_ONLY mode");
    }

    // void enableGuidance() {
    //     is_guidance_active_ = true;

    //     // 使能制导（guidance_control_node 在 enabled 但无目标时会持续发零速度，
    //     // 维持 PX4 OFFBOARD 心跳，避免 UAV 因 setpoint 断流触发 failsafe 降落）
    //     std_msgs::Bool enable;
    //     enable.data = true;
    //     guidance_enable_pub_.publish(enable);

    //     // 同步下发拦截速度(InterceptGuidance 用)
    //     std_msgs::Float32 speed_msg;
    //     speed_msg.data = static_cast<float>(guidance_speed_);
    //     guidance_speed_pub_.publish(speed_msg);

    //     ROS_INFO("[MissionManager] Guidance enabled (strike_speed=%.2f m/s)", guidance_speed_);
    // }

    void checkEmergencyStop() {
        // 低速时检查毫米波雷达
        // TODO: 获取当前速度判断是否低于阈值

        if (is_obstacle_detected_ && obstacle_distance_ < 10.0) {
            ROS_WARN("[MissionManager] OBSTACLE DETECTED! Emergency stop!");

            // 发布急停命令
            std_msgs::Bool stop;
            stop.data = true;
            emergency_stop_pub_.publish(stop);

            current_task_status_ = TaskStatus::EMERGENCY_STOP;
            is_mission_started_ = false;
        }
    }

    void reportTargetToGs() {
        // 通过 comm_node 回传目标信息（由 comm_node 处理）
        // 这里只是标记
        ROS_INFO("[MissionManager] Reporting target to GS");
    }

    void stopMission() {
        ROS_WARN("[MissionManager] Mission stopped");

        // 停止所有子模块
        is_mission_started_ = false;
        is_spiral_active_ = false;
        is_guidance_active_ = false;

        // 禁用制导
        std_msgs::Bool enable;
        enable.data = false;
        guidance_enable_pub_.publish(enable);

        // 重置目标状态
        initTargetState();

        current_task_status_ = TaskStatus::IDLE;
    }

    void publishStatus() {
        std_msgs::String status;

        std::ostringstream oss;
        oss << "mode:" << workModeToString() << ";"
            << "status:" << taskStatusToString() << ";"
            << "target_locked:" << (current_target_.is_locked ? "true" : "false") << ";"
            << "spiral_active:" << (is_spiral_active_ ? "true" : "false") << ";"
            << "guidance_active:" << (is_guidance_active_ ? "true" : "false");

        status.data = oss.str();
        status_pub_.publish(status);
    }

    // 默认打印 current_work_mode_；传参可打印任意 WorkMode(用于缓存/兜底值等场景)
    std::string workModeToString(WorkMode mode = static_cast<WorkMode>(-1)) {
        WorkMode m = (static_cast<int>(mode) < 0) ? current_work_mode_ : mode;
        switch (m) {
            case WorkMode::IDLE: return "IDLE";
            case WorkMode::SEARCH_ONLY: return "SEARCH_ONLY";
            case WorkMode::SEARCH_TRACK: return "SEARCH_TRACK";
            case WorkMode::SEARCH_STRIKE: return "SEARCH_STRIKE";
            case WorkMode::DENIED_ENV_FLIGHT: return "DENIED_ENV_FLIGHT";
            default: return "UNKNOWN";
        }
    }

    // 默认打印 current_phase_；传参可打印任意 MissionPhase
    std::string missionPhaseToString(MissionPhase phase = static_cast<MissionPhase>(-1)) {
        MissionPhase p = (static_cast<int>(phase) < 0) ? current_phase_ : phase;
        switch (p) {
            case MissionPhase::PHASE_GROUND_IDLE:      return "GROUND_IDLE";
            case MissionPhase::PHASE_TAKING_OFF:       return "TAKING_OFF";
            case MissionPhase::PHASE_HOVERING:         return "HOVERING";
            case MissionPhase::PHASE_WAYPOINT_FOLLOW:  return "WAYPOINT_FOLLOW";
            case MissionPhase::PHASE_GUIDANCE_TRACK:   return "GUIDANCE_TRACK";
            case MissionPhase::PHASE_GUIDANCE_STRIKE:  return "GUIDANCE_STRIKE";
            case MissionPhase::PHASE_RETURNING:        return "RETURNING";
            case MissionPhase::PHASE_COMPLETE:         return "COMPLETE";
            case MissionPhase::PHASE_FAILED:           return "FAILED";
            case MissionPhase::PHASE_HOLDING:          return "HOLDING";
            case MissionPhase::PHASE_OVERRIDDEN:          return "OVERRIDDEN";
            case MissionPhase::PHASE_TARGET_LOST_HOVER:   return "TARGET_LOST_HOVER";
            default: return "UNKNOWN";
        }
    }

    // ===================================================================
    // TrackingState 上报 — 协议 §6 / 补充规范 §5
    // 协议只定义两个状态:0=未跟踪, 1=跟踪中。
    // GCS 端要"连续接收"才会高亮 UI、闪烁提示,故机载端持续发:
    //   跟踪中  → 5Hz  state=1
    //   未跟踪  → 1Hz  state=0(保活 + 让 GCS 知道 UAV 在线)
    // ===================================================================

    /**
     * 构造并 publish 一帧 TrackingState。
     * state 严格按协议 §6 编码:0=未跟踪, 1=跟踪中。
     * 跟踪中时填当前 locked 目标的 GPS;未跟踪时位置字段填 0(协议 §6 未定义).
     */
    void publishTrackingState(uint8_t state) {
        multi_uav_strike::TrackingState ts;
        ts.state       = state;
        ts.flag        = 0;
        ts.target_dist = 0.0f;
        ts.target_lat  = 0.0;
        ts.target_lon  = 0.0;
        ts.target_alt  = 0.0;
        // 跟踪中:用 latest 的目标位置填 GPS 字段
        if (state == 1 && tracked_target_.is_valid && tracked_target_.latest) {
            ts.target_dist = 50.0f;  // 占位(协议 §9.4)
            ts.target_lat  = tracked_target_.latest->obj_lat;
            ts.target_lon  = tracked_target_.latest->obj_lon;
            ts.target_alt  = tracked_target_.latest->obj_alt;
        }
        tracking_state_pub_.publish(ts);
    }

    /**
     * 集中设置 lock_state。
     * 不在这里 publish 任何 TrackingState — tracking_state 的发送完全由 timer 驱动
     * (锁定→5Hz state=1,未锁→1Hz state=0),保证 GCS 持续有流。
     * 把所有 6+ 处 lock_state 赋值统一收敛到这里,便于以后扩展。
     */
    void setLockState(TrackLockState new_state) {
        if (tracked_target_.lock_state == new_state) return;  // 状态没变,无副作用
        tracked_target_.lock_state = new_state;
    }

    /**
     * 周期 timer:GCS 端要"连续接收"才显示"跟踪中"。
     *   仅在 SEARCH_TRACK 模式下发送(协议 §6:用于搜索跟踪)。
     *     锁定 (LOCKED_WAIT_CONFIRM / LOCKED_AUTO) → 5Hz 发 state=1
     *     未锁定                          → 1Hz 发 state=0(降频保活)
     *   其他模式(IDLE / GATHER / RETURN / etc.) → 不发
     * 频率切换:每 tick 都 setPeriod(1-5Hz 时可忽略开销;ROS Noetic ros::Timer 无
     *   period() getter,无法在外部判断"是否已切换",直接调用最简单)。
     */
    void trackingStateTimerCallback(const ros::TimerEvent&) {
        // 只在 SEARCH_TRACK 模式下上报,其余模式一律不发
        if (current_work_mode_ != WorkMode::SEARCH_TRACK) {
            return;
        }
        const bool is_locked = (tracked_target_.lock_state == TrackLockState::LOCKED_WAIT_CONFIRM ||
                                tracked_target_.lock_state == TrackLockState::LOCKED_AUTO);
        const ros::Duration want_period(is_locked ? (1.0 / TRACKING_STATE_RATE_HZ)        // 5Hz
                                                  : (1.0 / TRACKING_STATE_IDLE_RATE_HZ));  // 1Hz
        tracking_state_timer_.setPeriod(want_period);
        publishTrackingState(is_locked ? 1 : 0);
    }

    /**
     * 云台俯仰角下发(2026-07-29, 2026-07-30 扩 Attack(105))
     *   - 频率:~gimbal_command_rate_hz (默认 5Hz)
     *   - 内容:current_gimbal_pitch_(度,GS 语义:正值=下视)
     *     * 102 Search IN_TASK → sr.msg.gimbal_pitch(在 advanceSkillStateMachine IN_TASK
     *       入口改写,本 callback 只读)
     *     * 105 IN_TASK → 按子阶段(见下)
     *     * 其它 skill_type / 非 IN_TASK → gimbal_pitch_default_(默认 0,水平)
     *   - 兜底:有订阅者才 publish,避免空订阅时刷日志
     *
     *   Attack(105) 子阶段(2026-07-30):
     *     - strike 已触发:朝 primary cluster(用 computePitchDegToPrimaryCluster)
     *     - WP1 已达 + 延时已过:朝 WP2(用 computePitchDegToWp2)
     *     - 其他(刚进 IN_TASK / 延时内):attack_gimbal_look_default_pitch_deg_(默认 0 水平)
     */
    void gimbalCommandTimerCallback(const ros::TimerEvent&) {
        if (gimbal_command_pub_.getNumSubscribers() == 0) return;

        // === Attack(105) 阶段云台俯仰角计算(2026-07-30)===
        //   仅在 105 IN_TASK 期间覆盖 current_gimbal_pitch_。其余 skill_type 保持
        //   advanceSkillStateMachine / 102 IN_TASK 入口已写入的值不变。
        if (current_skill_index_ < skill_queue_.size()) {
            const SkillRuntime& sr = skill_queue_[current_skill_index_];
            if (sr.msg.skill_type == 105 && sr.direct_look_active_) {
                // === CODEX DIRECT-LOOK START ===
                // 云台本身无偏航能力：水平角由上面的机头 yaw setpoint 承担，
                // 此处只复用现有几何解算，让云台俯仰直接朝 WP2。
                current_gimbal_pitch_ = computePitchDegToWp2(sr);
                // === CODEX DIRECT-LOOK END ===
            } else if (sr.state == SkillState::IN_TASK && sr.msg.skill_type == 105) {
                const ros::Time now = ros::Time::now();
                if (attack_strike_phase_active_) {
                    current_gimbal_pitch_ = computePitchDegToPrimaryCluster();
                } else if (attack_wp1_reached_ &&
                           !attack_wp1_reached_time_.isZero() &&
                           (now - attack_wp1_reached_time_).toSec() >= attack_wp2_look_delay_sec_) {
                    current_gimbal_pitch_ = computePitchDegToWp2(sr);
                } else {
                    current_gimbal_pitch_ = attack_gimbal_look_default_pitch_deg_;
                }
            }
        }

        std_msgs::Float32 msg;
        // 地面站发送45度，云台需要-45完成下偏
        msg.data = -static_cast<float>(current_gimbal_pitch_);
        gimbal_command_pub_.publish(msg);
    }

    std::string taskStatusToString() {
        switch (current_task_status_) {
            case TaskStatus::IDLE: return "IDLE";
            case TaskStatus::EXECUTING_WAYPOINT: return "EXECUTING_WAYPOINT";
            case TaskStatus::TARGET_LOCKED: return "TARGET_LOCKED";
            case TaskStatus::SPIRAL_APPROACH: return "SPIRAL_APPROACH";
            case TaskStatus::GUIDANCE_APPROACH: return "GUIDANCE_APPROACH";
            case TaskStatus::STRIKE_EXECUTED: return "STRIKE_EXECUTED";
            case TaskStatus::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManager manager;
    ros::spin();
    return 0;
}
