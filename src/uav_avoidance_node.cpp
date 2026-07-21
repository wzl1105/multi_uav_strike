/**
 * uav_avoidance_node.cpp
 *
 * 多机机间避障决策节点(纯 ROS 内部,不动 starling_bridge)
 *
 * 职责:
 *   - 接收邻居无人机心跳(/inter_uav/uav_status — 全局 sn + 位置 + state)
 *   - 接收自身当前航段终点(/<ns>/waypoint_executor/current_target)
 *   - 基于 task_priority + uav_id 优先级判定"谁让谁"
 *   - 发布避障指令(/<ns>/avoidance/cmd,multi_uav_strike/AvoidanceCmd)
 *
 * 数据源说明:
 *   心跳 InterUavStatus 已经包含所有需要的信息(sn/lat/lon/alt/work_mode/phase),
 *   因此不再订阅 PoseArray 之类的位置冗余源,也不需要 lat/lon → sn 反查。
 *   心跳频率由 comm_node 按距离自适应(基础/5Hz/20Hz),1Hz 下位置 1s 旧
 *   对 5m/s 速度误差 ≈ 5m,相对 20-40m 阈值可接受。
 *
 * 策略:
 *   - 横向(lateral)严禁越线 — 压航线核心约束
 *   - 高度差 < vertical_safe 时让下位优先调高度
 *   - 调不开 → 减速 / 停车
 *   - 仍要撞 → 回退(reverse_allowed=true)
 *   - 优先依据:skill.priority 大的优先,同优则 uav_id 小的优先
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include "multi_uav_strike/AvoidanceCmd.h"
#include "multi_uav_strike/InterUavStatus.h"
#include "multi_uav_strike/WorkMode.h"
#include "multi_uav_strike/MissionState.h"

class UavAvoidance {
public:
    UavAvoidance() : nh_(), nh_private_("~") {
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<std::string>("uav_pose_topic", uav_pose_topic_, "");
        if (uav_pose_topic_.empty()) {
            uav_pose_topic_ = use_sim_ ? "quad/pose" : "mavros/local_position/pose";
        }
        nh_private_.param<std::string>("uav_id", uav_id_, "uav0");
        // uav_device_sn:本机 device_sn;空 → 回落到 uav_id("uav0")
        // 与 comm_node 的 ~uav_device_sn 一致,这样心跳的 sn 和这里的 sn 是同一字符串
        nh_private_.param<std::string>("uav_device_sn", uav_device_sn_, uav_id_);
        nh_private_.param<double>("collision_threshold", collision_threshold_, 8.0);
        nh_private_.param<double>("proximity_threshold", proximity_threshold_, 30.0);
        nh_private_.param<double>("vertical_safe", vertical_safe_, 5.0);
        // 垂直避让方向 buffer(m):dalt = other_alt - self_alt(正 = 对方在我上方)。
        //   dalt <  vertical_buffer_(我与对方等高 / 更高 / 仅略低)→ 向上爬升避让
        //   dalt >= vertical_buffer_(我明显在对方下方)         → 向下俯冲避让
        nh_private_.param<double>("vertical_buffer", vertical_buffer_, 2.0);
        // 垂直避让速率(m/s,正值):强(冲突区)/ 弱(接近区),方向由 buffer 规则决定
        nh_private_.param<double>("vertical_avoid_strong", vertical_avoid_strong_, 2.5);
        nh_private_.param<double>("vertical_avoid_mild", vertical_avoid_mild_, 1.5);
        // max_avoid_distance:超过此距离直接判定"无需避障",原硬编码 50m
        nh_private_.param<double>("max_avoid_distance", max_avoid_distance_, 100.0);
        nh_private_.param<double>("loop_freq", loop_freq_, 20.0);

        // 从 uav_id 提取数字(旧 tiebreaker 路径,保留向后兼容)
        self_uav_num_ = extractUavNumber(uav_id_);
        // device_sn 用于 tiebreaker(字典序,"1-1" 优先于 "1-2")

        ROS_INFO("[UavAvoidance] self_id=%s device_sn=%s num=%d thresholds(coll=%.1f/prox=%.1f/vert_safe=%.1f/max_dist=%.1f) [gps=%s]",
                 uav_id_.c_str(), uav_device_sn_.c_str(), self_uav_num_,
                 collision_threshold_, proximity_threshold_, vertical_safe_, max_avoid_distance_,
                 have_raw_gps_ ? "raw(HOME-indep)" : "global(EKF2,HOME-dep)");

        // ========== 订阅 ==========
        self_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10,
                                       &UavAvoidance::selfPoseCb, this);

        std::string current_target_t = "waypoint_executor/current_target";
        current_target_sub_ = nh_.subscribe(current_target_t, 10,
                                            &UavAvoidance::currentTargetCb, this);

        // 心跳是 comm_node 在全局 topic "/inter_uav/uav_status" 上发(绝对路径),
        // 这里是相对路径会落在 /<ns>/inter_uav/uav_status 上,收不到。
        // 心跳里包含 sn + lat/lon/alt + work_mode + phase + timestamp_us,
        // 唯一信源 — 不再订阅 PoseArray 之类的冗余位置源。
        std::string heartbeat_t = "/inter_uav/uav_status";
        heartbeat_sub_ = nh_.subscribe(heartbeat_t, 50,
                                       &UavAvoidance::heartbeatCb, this);

        // 订阅 mavros GPS — 双源策略
        //   1) /mavros/global_position/global(EKF2 估计,几乎一定存在)— 作为 primary
        //   2) /mavros/global_position/raw(传感器原始读数,HOME 无关)— 作为 override
        // 背景:多机 SITL 各自 PX4 HOME 不同时,NED+HOME 投影会让算出的 GPS 偏移,
        //       同一对邻居会算出 1m vs 43m 的距离。raw GPS 是真值,但不是所有 mavros
        //       配置都开这个 topic(某些版本/PX4 固件不发)。所以双订阅:
        //       - /global 一到就先填上,保证避障不空转
        //       - /raw 一到就覆盖,拿到 HOME 无关的真值
        gps_global_sub_ = nh_.subscribe("mavros/global_position/global", 10,
                                        &UavAvoidance::gpsGlobalCallback, this);
        gps_raw_sub_ = nh_.subscribe("mavros/global_position/raw", 10,
                                     &UavAvoidance::gpsRawCallback, this);
        have_self_gps_ = false;
        have_raw_gps_ = false;

        // 订阅本机当前 work_mode + mission_state,用于本地 (work_mode, phase) 联合优先级判定
        self_work_mode_sub_ = nh_.subscribe("mission/current_work_mode", 10,
                                            &UavAvoidance::selfWorkModeCb, this);
        self_mission_state_sub_ = nh_.subscribe("mission/mission_state", 10,
                                                &UavAvoidance::selfMissionStateCb, this);

        // ========== 发布 ==========
        std::string out_t = "avoidance/cmd";
        avoidance_pub_ = nh_.advertise<multi_uav_strike::AvoidanceCmd>(out_t, 10);

        // ========== 定时器 ==========
        timer_ = nh_.createTimer(ros::Duration(1.0 / loop_freq_),
                                 &UavAvoidance::tickCb, this);

        have_self_pose_ = false;
        have_current_target_ = false;
    }

private:
    ros::NodeHandle nh_, nh_private_;

    ros::Subscriber self_pose_sub_;
    ros::Subscriber current_target_sub_;
    ros::Subscriber heartbeat_sub_;
    ros::Subscriber gps_global_sub_;     // /mavros/global_position/global — EKF2 估计(必发)
    ros::Subscriber gps_raw_sub_;        // /mavros/global_position/raw    — 传感器原始(HOME 无关,可能不发)
    ros::Subscriber self_work_mode_sub_;
    ros::Subscriber self_mission_state_sub_;
    ros::Publisher  avoidance_pub_;
    ros::Timer      timer_;

    bool use_sim_;
    std::string uav_pose_topic_;
    std::string uav_id_;
    // 自身 device_sn(命名约定 "1-1" / "1-2"...),默认回落到 uav_id
    // 用作心跳里 sn 的查找键,以及优先级相同时的 tiebreaker(字典序)
    std::string uav_device_sn_;
    double collision_threshold_;
    double proximity_threshold_;
    double vertical_safe_;
    double vertical_buffer_;        // 垂直避让方向 buffer(见构造函数注释)
    double vertical_avoid_strong_;  // 冲突区垂直避让速率 (m/s)
    double vertical_avoid_mild_;    // 接近区垂直避让速率 (m/s)
    double max_avoid_distance_;   // > 此距离直接判定"无需避障"
    double loop_freq_;
    int self_uav_num_;

    // 自位置(NED,日志/调试用)
    bool have_self_pose_;
    geometry_msgs::PoseStamped self_pose_;

    // 自位置 GPS(直接来自 mavros/global_position/raw,不依赖 PX4 HOME)
    // 多机 SITL 即使各自 PX4 HOME 不同,raw GPS 仍是世界坐标系下的真实 GPS
    // → 机间避障距离计算必须用这个,不能用 NED+HOME 投影(会算错,见 [AVOID] SELF log)
    bool have_self_gps_;
    sensor_msgs::NavSatFix self_gps_;
    // 是否收到过 /raw(用于日志统计;raw 真值会覆盖 global)
    bool have_raw_gps_;

    // 当前航段终点
    bool have_current_target_;
    geometry_msgs::PoseStamped current_target_;

    // 邻居表(心跳 cache,已过滤 self + 过期)— tickCb 直接迭代算距离与判定 state,
    //   sn / lat / lon / alt / work_mode / phase / timestamp_us 全部就齐
    std::vector<multi_uav_strike::InterUavStatus::ConstPtr> heartbeats_;
    ros::Time last_heartbeat_purge_;

    // 本机当前 work_mode + phase(供 (work_mode,phase) 联合优先级判定用)
    uint8_t self_work_mode_ = 0;   // 默认 IDLE,后续会收到 mission/current_work_mode
    uint8_t self_phase_      = 0;  // 默认 GROUND_IDLE,后续会收到 mission/mission_state
    bool    have_self_state_ = false;

    int extractUavNumber(const std::string& id) {
        // "uav0" → 0, "uav12" → 12
        size_t pos = id.find_last_not_of("0123456789");
        if (pos == std::string::npos) return 0;
        std::string num = id.substr(pos + 1);
        return num.empty() ? 0 : std::stoi(num);
    }

    void selfPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        self_pose_ = *msg;
        if (!have_self_pose_) {
            ROS_WARN("[UavAvoidance] first self_pose received (NED x=%.2f y=%.2f z=%.2f)",
                     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
        have_self_pose_ = true;
    }

    void gpsGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // mavros/global_position/global = EKF2 估计。HOME 依赖(EKF2 origin = HOME),
        //   但 mavros 默认必发。先填这个保证避障不空转。
        // 仅在尚未收到 /raw 时填充,避免 /raw 真值被 /global 覆盖回去
        if (have_raw_gps_) {
            return;
        }
        self_gps_ = *msg;
        if (!have_self_gps_) {
            ROS_WARN("[UavAvoidance] first GPS received (/global, EKF2 HOME-dep): lat=%.7f lon=%.7f alt=%.2f",
                     msg->latitude, msg->longitude, msg->altitude);
        }
        have_self_gps_ = true;
    }

    void gpsRawCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // mavros/global_position/raw = PX4 GPS 传感器原始读数,不依赖 PX4 HOME。
        // 多机 SITL 即使各自 PX4 HOME 不同,raw GPS 仍是世界坐标系下的真实 GPS
        // → 机间避障距离计算必须用这个,不能用 NED+HOME 投影
        //   (之前每机 HOME 差十几米,NED→GPS 投影错位 → 同一对邻居算出 1m vs 43m)
        self_gps_ = *msg;
        if (!have_raw_gps_) {
            ROS_WARN("[UavAvoidance] first raw GPS received (HOME-indep, overrides /global): lat=%.7f lon=%.7f alt=%.2f",
                     msg->latitude, msg->longitude, msg->altitude);
            have_raw_gps_ = true;
        }
        // 首次拿到任意 GPS 才标记 have_self_gps_(可能比 gpsGlobalCallback 更早到)
        if (!have_self_gps_) {
            have_self_gps_ = true;
        }
    }

    void currentTargetCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target_ = *msg;
        have_current_target_ = true;
    }

    void selfWorkModeCb(const multi_uav_strike::WorkMode::ConstPtr& msg) {
        self_work_mode_ = msg->mode;
        have_self_state_ = true;
    }

    void selfMissionStateCb(const multi_uav_strike::MissionState::ConstPtr& msg) {
        self_phase_ = msg->phase;
        have_self_state_ = true;
    }

    /**
     * 将 mission_manager 本地 WorkMode 枚举值映射到 WorkMode.msg 值。
     *
     * mission_manager_node.cpp 内部定义了一份 `enum class WorkMode`
     *   IDLE=0, SEARCH_ONLY=1, SEARCH_TRACK=2, SEARCH_STRIKE=3, DENIED_ENV_FLIGHT=4
     * 与 WorkMode.msg
     *   IDLE=0, TAKEOFF=1, GATHER=2, SEARCH_ONLY=3, SEARCH_TRACK=4, SEARCH_STRIKE=5, ..., DENIED_ENV_FLIGHT=8
     * 数值不一致。mission_manager 在 publishCurrentWorkMode() 里直接
     * `static_cast<uint8_t>(current_work_mode_)` 把本地枚举投成 uint8 发出,
     * 因此 InterUavStatus.work_mode 拿到的是 LOCAL 值,不是 msg 值。
     * 这里做一次翻译,让 computeAvoidanceRank 能按 msg 语义分支。
     */
    static uint8_t translateLocalWorkModeToMsg(uint8_t local_wm) {
        switch (local_wm) {
            case 1:  return 3;  // local SEARCH_ONLY       → msg SEARCH_ONLY
            case 2:  return 4;  // local SEARCH_TRACK      → msg SEARCH_TRACK
            case 3:  return 5;  // local SEARCH_STRIKE     → msg SEARCH_STRIKE
            case 4:  return 8;  // local DENIED_ENV_FLIGHT → msg DENIED_ENV_FLIGHT
            case 0:  // IDLE — msg 同样为 0,直接返回
            default: // 未知:透传(可能是 msg 直接发的,或新加的值)
                return local_wm;
        }
    }

    /**
     * 联合 (work_mode, phase) 优先级数值,数值越小优先级越高。
     *
     * 设计(参考 WorkMode.msg 注释 + 用户规则):
     *   group 1 (100): phase=5 GUIDANCE_STRIKE — 最高,正在打击,其他 UAV 一律让出
     *   group 2 (200): phase=4 GUIDANCE_TRACK  — 正在跟踪,其他 UAV 让出射界
     *   group 3 (300+): phase=3 WAYPOINT_FOLLOW (搜索/航点跟踪),按 work_mode 细分:
     *                   300 = SEARCH_STRIKE, 301 = SEARCH_TRACK,
     *                   302 = SEARCH_ONLY,   303 = DENIED_ENV_FLIGHT,
     *                   304 = 其它未知 work_mode 但仍在搜索
     *   group 4 (900): 其它阶段 (TAKING_OFF/HOVERING/RETURNING/GROUND_IDLE/
     *                   COMPLETE/FAILED/HOLDING/IDLE) — 让出给活跃 UAV
     *
     * 入参 work_mode 期望是 WorkMode.msg 值空间(translateLocalWorkModeToMsg 翻译后)。
     * 同 rank 时由 caller 用 device_sn 字典序 tiebreak(sn 小的赢)。
     */
    static int computeAvoidanceRank(uint8_t work_mode_msg, uint8_t phase) {
        if (phase == 5 /* GUIDANCE_STRIKE */) return 100;
        if (phase == 4 /* GUIDANCE_TRACK  */) return 200;
        if (phase == 3 /* WAYPOINT_FOLLOW */) {
            switch (work_mode_msg) {
                case 5 /* SEARCH_STRIKE */:      return 300;
                case 4 /* SEARCH_TRACK */:       return 301;
                case 3 /* SEARCH_ONLY */:        return 302;
                case 8 /* DENIED_ENV_FLIGHT */:  return 303;
                default:                          return 304;
            }
        }
        return 900;
    }

    /**
     * 是否处于"无法主动让路"的状态 — 即 UAV 在物理位置上接近静止。
     * 这些 UAV 不能响应 avoidance cmd(无 active 控制环路),所以在冲突时
     * 必须由对方主动避让。
     *
     * phase enum (MissionPhase):
     *   0 GROUND_IDLE 1 TAKING_OFF 2 HOVERING 3 WAYPOINT_FOLLOW
     *   4 GUIDANCE_TRACK 5 GUIDANCE_STRIKE 6 RETURNING 7 COMPLETE
     *   8 FAILED 9 HOLDING
     *
     * 视作 stationary:GROUND_IDLE / HOLDING / COMPLETE / FAILED
     * 不视作(可主动避让):TAKING_OFF / HOVERING / WAYPOINT_FOLLOW /
     *   GUIDANCE_TRACK / GUIDANCE_STRIKE / RETURNING
     *   (RETURNING 可能在飞回 home,自动避让 cmd 会被 waypoint_executor 接收)
     */
    static bool isStationaryPhase(uint8_t phase) {
        switch (phase) {
            case 0:  // GROUND_IDLE
            case 7:  // COMPLETE
            case 8:  // FAILED
            case 9:  // HOLDING
                return true;
            default:
                return false;
        }
    }

    static std::string phaseStr(uint8_t phase) {
        switch (phase) {
            case 0: return "GROUND_IDLE";
            case 1: return "TAKING_OFF";
            case 2: return "HOVERING";
            case 3: return "WAYPOINT_FOLLOW";
            case 4: return "GUIDANCE_TRACK";
            case 5: return "GUIDANCE_STRIKE";
            case 6: return "RETURNING";
            case 7: return "COMPLETE";
            case 8: return "FAILED";
            case 9: return "HOLDING";
            default: return "UNKNOWN";
        }
    }

    void heartbeatCb(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        // 自环过滤:心跳 topic 是全局 /inter_uav/uav_status,本机 comm_node 也会发,
        // 这里必须先过滤 self。否则 heartbeats_ 里会含本机心跳,tickCb 距离计算时
        // 会算到 0m,把自己当成"最近邻居",触发 tie against self,完全没法避让。
        if (msg->sn == uav_device_sn_) {
            return;
        }
        // 心跳是 tickCb 唯一的邻居信源,周期性清理过期(避免断连 UAV 残留)
        ros::Time now = ros::Time::now();
        if ((now - last_heartbeat_purge_).toSec() > 1.0) {
            heartbeats_.erase(
                std::remove_if(heartbeats_.begin(), heartbeats_.end(),
                               [&](const multi_uav_strike::InterUavStatus::ConstPtr& h) {
                                   // h->timestamp_us 是微秒,转 ros::Time
                                   double age_sec = now.toSec() - static_cast<double>(h->timestamp_us) * 1e-6;
                                   return age_sec > 5.0;
                               }),
                heartbeats_.end());
            last_heartbeat_purge_ = now;
        }
        heartbeats_.push_back(msg);
    }

    void tickCb(const ros::TimerEvent&) {
        multi_uav_strike::AvoidanceCmd cmd;
        cmd.lateral_blocked = true;
        cmd.lateral_max = 0.0;
        cmd.vertical_bias = 0.0;
        cmd.speed_scale = 1.0;
        cmd.reverse_allowed = false;

        // 每条日志都带 uav_id 前缀,方便多机并发打印时 grep 区分
        const std::string TAG = "[AVOID " + uav_id_ + "]";

        // ========== 缺前置条件 → 不避让 ==========
        if (!have_self_gps_ || !have_self_state_ || heartbeats_.empty()) {
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 直接用 mavros raw GPS(HOME 无关,多机一致) ==========
        // 之前用 NED + 每机 PX4 home 投影,多机 SITL 各自 HOME 不同时,同一对邻居
        // 会算出 1m vs 43m 的距离 → 现改用 raw GPS,所有 UAV 都在同一坐标系。
        const double self_lat = self_gps_.latitude;
        const double self_lon = self_gps_.longitude;
        const double self_alt = self_gps_.altitude;
        const double cos_self_lat = std::max(0.1, std::cos(self_lat * M_PI / 180.0));

        // ========== SELF 完整信息:我是谁,我在哪,我现在在干啥 ==========
        const uint8_t self_wm_msg = translateLocalWorkModeToMsg(self_work_mode_);
        const int self_rank = computeAvoidanceRank(self_wm_msg, self_phase_);

        // ========== 列出收到的所有心跳邻居 + 各自距离 ==========
        // 心跳是唯一信源:sn / lat / lon / alt / work_mode / phase 全部自带,
        // 不再需要 PoseArray + lat/lon → sn 反查这种多源拼凑结构。
        double min_dist = 1e9;
        size_t min_idx = 0;
        bool found = false;
        for (size_t i = 0; i < heartbeats_.size(); ++i) {
            const auto& h = heartbeats_[i];
            const double n_lat = h->lat;
            const double n_lon = h->lon;
            const double n_alt = h->alt;
            const double dlat = (n_lat - self_lat);
            const double dlon = (n_lon - self_lon);
            const double dalt = (n_alt - self_alt);
            const double dx = dlat * 111000.0;
            const double dy = dlon * 111000.0 * cos_self_lat;
            const double dist = std::sqrt(dx * dx + dy * dy + dalt * dalt);
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
                found = true;
            }
        }

        // ========== 距离超过阈值 → 不需避障 ==========
        if (!found || min_dist > max_avoid_distance_) {
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 直接从最近心跳取 sn / state / alt ==========
        const auto& nearest = heartbeats_[min_idx];
        const std::string& other_sn        = nearest->sn;
        const uint8_t      other_work_mode = nearest->work_mode;
        const uint8_t      other_phase     = nearest->phase;
        const double       other_alt       = nearest->alt;

        // ========== 优先级判定 (work_mode + phase 联合) ==========
        const uint8_t other_wm_msg = translateLocalWorkModeToMsg(other_work_mode);
        const int other_rank = computeAvoidanceRank(other_wm_msg, other_phase);

        // 静止 UAV 检测:HOLDING/GROUND_IDLE 等状态无法主动让路(没有 active 控制器
        // 消费 avoidance cmd)。冲突时必须由"动"的那一方避让,否则双方都不动会撞。
        const bool self_is_stationary  = isStationaryPhase(self_phase_);
        const bool other_is_stationary = isStationaryPhase(other_phase);

        std::string priority_reason;
        bool i_am_lower_priority = false;

        if (other_is_stationary && !self_is_stationary) {
            // 对方静止(典型场景:完成任务后 HOLDING),自己必须主动避让
            // (低 rank=900 也不重要,关键是不能撞静止的 UAV)
            i_am_lower_priority = true;
            priority_reason = "other is stationary (phase=" + phaseStr(other_phase) +
                              " sn=" + other_sn + "), self must avoid regardless of rank";
        } else if (self_is_stationary && !other_is_stationary) {
            // 自己静止 → 对方应该避让,但因为我们没法主动让路,这条路走不通。
            // 仍标记 self has priority,把 avoidance cmd 让对方去响应(对方 waypoint_executor 会执行)
            i_am_lower_priority = false;
            priority_reason = "self is stationary (phase=" + phaseStr(self_phase_) + "), self has priority (other should avoid)";
        } else if (self_rank > other_rank) {
            i_am_lower_priority = true;
            priority_reason = "self_rank(" + std::to_string(self_rank) + ") > other_rank(" + std::to_string(other_rank) + ") → self yields";
        } else if (self_rank == other_rank) {
            if (uav_device_sn_ > other_sn) {
                i_am_lower_priority = true;
                priority_reason = "tie: same rank(" + std::to_string(self_rank) + "), self_sn(" + uav_device_sn_ + ") > other_sn(" + other_sn + ") → self yields";
            } else {
                priority_reason = "tie: same rank(" + std::to_string(self_rank) + "), self_sn(" + uav_device_sn_ + ") <= other_sn(" + other_sn + ") → self has priority";
            }
        } else {
            priority_reason = "self_rank(" + std::to_string(self_rank) + ") < other_rank(" + std::to_string(other_rank) + ") → self has priority";
        }

        if (!i_am_lower_priority) {
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 我需要避让 ==========
        const double dalt_with_other = other_alt - self_alt;

        ROS_WARN_THROTTLE(1.0, "%s AVOIDING: reason=[%s], target=sn=%s dist=%.2fm dalt=%+.2fm (thresholds coll=%.1f prox=%.1f vert_safe=%.1f)",
                          TAG.c_str(), priority_reason.c_str(), other_sn.c_str(),
                          min_dist, dalt_with_other,
                          collision_threshold_, proximity_threshold_, vertical_safe_);

        // ========== 垂直避让方向:buffer 规则 ==========
        // dalt_with_other = other_alt - self_alt(正 = 对方在我上方,即我在下面)
        //   dalt <  vertical_buffer_(我等高/更高/仅略低于对方)→ 向上爬升(climb),dir=+1
        //   dalt >= vertical_buffer_(我明显在对方下方)        → 向下俯冲(descend),dir=-1
        // vertical_bias 语义:向上爬升速率(正=向上,负=向下),executor 里 vz -= bias。
        const bool climb_up = (dalt_with_other < vertical_buffer_);
        const double vdir = climb_up ? +1.0 : -1.0;
        const char* vdir_str = climb_up ? "CLIMB-UP" : "DESCEND";

        if (min_dist < collision_threshold_) {
            if (std::fabs(dalt_with_other) < vertical_safe_) {
                // 高度还没拉开 → 按方向强避让
                cmd.vertical_bias = vdir * vertical_avoid_strong_;
                if (min_dist < collision_threshold_ * 0.6) {
                    cmd.speed_scale = 0.0;
                    cmd.reverse_allowed = true;
                    ROS_WARN_THROTTLE(1.0, "%s   → COLLISION-CRITICAL (dist<%.1f): STOP+REVERSE horizontal, vert=%s bias=%+.2f, reverse_allowed=%d",
                                      TAG.c_str(), collision_threshold_ * 0.6,
                                      vdir_str, cmd.vertical_bias, cmd.reverse_allowed);
                } else {
                    cmd.speed_scale = 0.2;
                    ROS_WARN_THROTTLE(1.0, "%s   → COLLISION (dist<%.1f): slow speed_scale=%.2f, vert=%s bias=%+.2f",
                                      TAG.c_str(), collision_threshold_,
                                      cmd.speed_scale, vdir_str, cmd.vertical_bias);
                }
            } else {
                // 高度已拉开 > vert_safe:垂直不再动作(executor 会锁存保持高度),
                // 仅减速从上/下方通过
                cmd.vertical_bias = 0.0;
                cmd.speed_scale = 0.4;
                ROS_WARN_THROTTLE(1.0, "%s   → COLLISION-DIST but alt-separated (|dalt|=%.1f>vert_safe=%.1f): hold alt, speed_scale=%.2f",
                                  TAG.c_str(), std::fabs(dalt_with_other), vertical_safe_,
                                  cmd.speed_scale);
            }
        } else if (min_dist < proximity_threshold_) {
            // 接近区(prox<dist<coll):仅减速,不做垂直动作。
            // 垂直避让留到冲突区(<coll)触发,避免过早/过大的高度反应(40m 就抬高太夸张)。
            // 渐进式减速:在 proximity 边界处 scale=1.0(刚进入时不减速,避免远处就开始"刹车"看不出来),
            // 线性 ramp 到 collision 边界处 scale=0.3(让靠近碰撞时有明显减速效果)。
            double t = (proximity_threshold_ - min_dist) /
                       (proximity_threshold_ - collision_threshold_);
            t = std::max(0.0, std::min(1.0, t));
            cmd.speed_scale = 1.0 - 0.7 * t;  // 1.0@prox边界 → 0.3@coll边界
            ROS_WARN_THROTTLE(1.0, "%s   → PROXIMITY (%.1f<dist<%.1f): ramped speed_scale=%.2f (t=%.2f, no vertical, |dalt|=%.1f)",
                              TAG.c_str(), collision_threshold_, proximity_threshold_,
                              cmd.speed_scale, t, std::fabs(dalt_with_other));
        }

        // 横向严禁 — 已经在 cmd.lateral_blocked=true / lateral_max=0.0
        avoidance_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_avoidance_node");
    UavAvoidance node;
    ros::spin();
    return 0;
}
