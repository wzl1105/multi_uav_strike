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
#include <geometry_msgs/TwistStamped.h>
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
        nh_private_.param<double>("collision_t_react", collision_t_react_, 1.5);
        nh_private_.param<double>("proximity_t_react", proximity_t_react_, 2.0);
        nh_private_.param<double>("max_avoid_t_react", max_avoid_t_react_, 2.5);
        nh_private_.param<double>("vertical_safe", vertical_safe_, 5.0);
        // 垂直避让方向 buffer(m):dalt = other_alt - self_alt(正 = 对方在我上方)。
        //   dalt <  vertical_buffer_(我与对方等高 / 更高 / 仅略低)→ 向上爬升避让
        //   dalt >= vertical_buffer_(我明显在对方下方)         → 向下俯冲避让
        nh_private_.param<double>("vertical_buffer", vertical_buffer_, 2.0);
        // 垂直避让速率(m/s,正值):强(冲突区)/ 弱(接近区),方向由 buffer 规则决定
        nh_private_.param<double>("vertical_avoid_strong", vertical_avoid_strong_, 5.0);
        nh_private_.param<double>("vertical_avoid_mild", vertical_avoid_mild_, 1.5);
        // max_avoid_distance:超过此距离直接判定"无需避障",原硬编码 50m
        nh_private_.param<double>("max_avoid_distance", max_avoid_distance_, 100.0);
        // === 2026-07-24: 动态高度避让参数 ===
        // 邻居水平速度(m/s)低于此阈值即视为"静止/HOLDING",vertical_bias 衰减为
        // vertical_bias_soft_;默认 0 完全不推高。HOLDING 飞机典型抖动 < 0.2m/s,
        // 0.5m/s 留余量。
        nh_private_.param<double>("stationary_speed_threshold", stationary_speed_threshold_, 0.5);
        nh_private_.param<double>("vertical_bias_soft",            vertical_bias_soft_,            0.0);
        // === 2026-07-24: 阈值随闭合速度双向缩放 ===
        // 替换原 max(base, t_react*cs) 单向放大逻辑。允许 cs<0(分离)时缩阈值,
        // 也允许 cs>0 时按 gain 放大;最终 eff_thresh = base * cs_scale(cs)。
        // cs_scale = clamp(1.0 + gain*cs, min_scale, max_scale)
        //   gain=0.3:每 m/s 闭合速度把阈值乘以 +0.3(慢接近:+30%、相向 20m/s:+600%→截断 max)
        //   min_scale=0.5:分离时阈值不低于基线 50%(避免阈值塌缩到接近 0)
        //   max_scale=2.0:接近时阈值不超过基线 200%(留余量,避免一接近就触发 vertical 死循环)
        nh_private_.param<double>("avoid_threshold_gain",     avoid_threshold_gain_,     0.3);
        nh_private_.param<double>("avoid_threshold_min_scale", avoid_threshold_min_scale_, 0.5);
        nh_private_.param<double>("avoid_threshold_max_scale", avoid_threshold_max_scale_, 2.0);
        // velocity_local 径向闭合速度滤波 + 无本机速度时的距离差 fallback。
        nh_private_.param<double>("closing_speed_alpha", closing_speed_alpha_, 0.2);
        nh_private_.param<double>("closing_speed_fallback_min_dt", closing_speed_fallback_min_dt_, 0.5);
        closing_speed_alpha_ = std::max(0.0, std::min(1.0, closing_speed_alpha_));
        closing_speed_fallback_min_dt_ = std::max(0.05, closing_speed_fallback_min_dt_);
        // critical 区只有确认接近才倒车；确认分离则低速继续向前离开交汇点。
        nh_private_.param<double>("reverse_min_closing_speed", reverse_min_closing_speed_, 0.5);
        nh_private_.param<double>("separating_speed_scale", separating_speed_scale_, 0.4);
        reverse_min_closing_speed_ = std::max(0.0, reverse_min_closing_speed_);
        separating_speed_scale_ = std::max(0.0, std::min(1.0, separating_speed_scale_));
        // === 2026-07-24: 垂直迟滞(抗 |dalt| 在 eff_vert 附近 toggle 造成的剧烈点头)===
        nh_private_.param<double>("vertical_hysteresis",      vertical_hysteresis_,      1.0);
        // 水平避让区迟滞比例:进入阈值缩小、退出阈值放大,避免在 prox/coll 边界反复切换。
        nh_private_.param<double>("horizontal_hysteresis",    horizontal_hysteresis_,    0.05);
        horizontal_hysteresis_ = std::max(0.0, std::min(0.45, horizontal_hysteresis_));
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
        self_velocity_sub_ = nh_.subscribe(
            "mavros/local_position/velocity_local", 10,
            &UavAvoidance::selfVelocityCb, this);

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
    ros::Subscriber self_velocity_sub_;
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
    double collision_t_react_;       // 碰撞阈值对应的反应时间(s)
    double proximity_t_react_;       // 接近阈值对应的反应时间(s)
    double max_avoid_t_react_;       // 最大避障距离对应的反应时间(s)
    double vertical_safe_;
    double vertical_buffer_;        // 垂直避让方向 buffer(见构造函数注释)
    double vertical_avoid_strong_;  // 冲突区垂直避让速率 (m/s)
    double vertical_avoid_mild_;    // 接近区垂直避让速率 (m/s)
    double max_avoid_distance_;   // > 此距离直接判定"无需避障"
    double loop_freq_;
    // === 2026-07-24: 邻居速度驱动的动态高度避让 ===
    // 邻居水平速度 < stationary_speed_threshold_ 时视为静止(典型:对方已完成
    //   gather 进入 HOLDING,或本身任务结束),把自己的 vertical_avoid_strong 衰减
    //   到 vertical_bias_soft_(默认 0)。背景:集结点相近 + |dalt|<vertical_safe 时,
    //   现行逻辑会把接近者推高 5m/s,3D 距离判不到航点 → gather 卡死。
    //   静止邻居对我无相对速度,横向严禁仍生效,纵向我"慢慢的接近",不会再把自
    //   己水平位置飞过去撞上对方 — 但前提是 GS 让集结点水平+垂直间隔 > vertical_safe。
    // stationary_speed_threshold_ 单位 m/s,EMA 简单平滑(α=0.3)以抗噪声。
    double stationary_speed_threshold_;
    double vertical_bias_soft_;     // 邻居静止时的 vertical_bias 衰减目标(m/s)
    // === 2026-07-24: 阈值随闭合速度双向缩放参数 ===
    // 替代旧 max(base, t_react*cs) 单向放大;现在 eff_thresh = base * cs_scale,
    //   cs_scale 由 avoid_threshold_gain_、min_scale_、max_scale_ 钳制。
    double avoid_threshold_gain_;
    double avoid_threshold_min_scale_;
    double avoid_threshold_max_scale_;
    double closing_speed_alpha_;
    double closing_speed_fallback_min_dt_;
    double reverse_min_closing_speed_;
    double separating_speed_scale_;
    // === 2026-07-24: 垂直迟滞(hysteresis)===
    // 解决 GPS 噪声(~0.5m)+ eff_vert 硬阈值导致的 vbias 反复 toggle、vz 在 -5↔0
    // 之间疯狂跳、PX4 机身剧烈点头的问题。
    //   - 正在爬升时:要求 |dalt| > eff_vert + vertical_hysteresis_ 才释放(防 GPS 抖动
    //     把已爬到位的高度判回"未拉开"再次 push)
    //   - 未在爬升时:要求 |dalt| < eff_vert - vertical_hysteresis_ 才开始 push(防抖)
    //   死区 = 2 * vertical_hysteresis_,默认 1.0m 即总死区 2m,大于 GPS 噪声
    double vertical_hysteresis_;
    // 状态机:本 tick 是否在爬升
    bool   was_pushing_vertically_ = false;
    // === 2026-07-24: 水平迟滞(抗 speed_scale 反复 toggle 引起的俯仰点头)===
    // 同样根因:GPS 距离噪声 + cs_scale 随 cs 抖动 → eff_prox/eff_coll 在阈值附近反复
    //   穿越 → UAV 在"避让区/非避让区"反复 toggle → speed_scale 每 0.5~1s 在 1.0 ↔ 0.7
    //   之间跳 → PX4 反复加减速 → 机身俯仰点头(低头抬头)。
    // 进入阈值 = base * (1 - margin),离开阈值 = base * (1 + margin),死区 = 2 * margin。
    // 默认 margin=0.05(5%),死区 10% — 远大于 GPS 噪声造成的 eff_prox/eff_coll 摆幅。
    double horizontal_hysteresis_;
    // 状态机:本 tick 是否在 proximity / collision 避让区
    bool   was_in_prox_zone_  = false;
    bool   was_in_coll_zone_  = false;
    // critical 倒车锁存:只允许在确认接近时进入,离开 critical/换邻居时释放。
    bool   reverse_active_    = false;
    double reverse_release_distance_ = 0.0;
    int self_uav_num_;

    // 最近邻距离变化估计闭合速度,用于高速接近时提前扩大避障区。
    // 只在收到同一最近邻的新心跳样本时更新,不能在 20Hz tick 中重复对同一份
    // 1~20Hz GPS 样本求导,否则会在新心跳到达的单个 tick 产生闭合速度尖峰。
    double last_min_dist_ = -1.0;
    ros::Time last_min_dist_time_;
    double closing_speed_smoothed_ = 0.0;
    std::string last_nearest_sn_;
    uint64_t last_nearest_timestamp_us_ = 0;

    // 自位置(NED,日志/调试用)
    bool have_self_pose_;
    geometry_msgs::PoseStamped self_pose_;

    // 本机 NED 速度,来自 mavros/local_position/velocity_local(ROS 消息本身为 ENU)。
    bool have_self_velocity_ = false;
    double self_vx_ned_ = 0.0;
    double self_vy_ned_ = 0.0;
    double self_vz_ned_ = 0.0;

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

    void selfVelocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // MAVROS velocity_local 是 ENU: x=East,y=North,z=Up；统一转为 NED。
        self_vx_ned_ = msg->twist.linear.y;
        self_vy_ned_ = msg->twist.linear.x;
        self_vz_ned_ = -msg->twist.linear.z;
        if (!have_self_velocity_) {
            ROS_WARN("[UavAvoidance] first velocity_local received (NED): vx=%.2f vy=%.2f vz=%.2f m/s",
                     self_vx_ned_, self_vy_ned_, self_vz_ned_);
        }
        have_self_velocity_ = true;
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
     *   8 FAILED 9 HOLDING 10 OVERRIDDEN 11 TARGET_LOST_HOVER
     *
     * 视作 stationary:GROUND_IDLE / HOLDING / COMPLETE / FAILED / OVERRIDDEN / TARGET_LOST_HOVER
     * 不视作(可主动避让):TAKING_OFF / HOVERING / WAYPOINT_FOLLOW /
     *   GUIDANCE_TRACK / GUIDANCE_STRIKE / RETURNING
     *   (RETURNING 可能在飞回 home,自动避让 cmd 会被 waypoint_executor 接收)
     *   OVERRIDDEN:飞手接管,飞机不会响应程控避让指令 → 邻居主动让路
     *   TARGET_LOST_HOVER:目标丢失悬停,意图回扫描,视作不可移动
     */
    static bool isStationaryPhase(uint8_t phase) {
        switch (phase) {
            case 0:   // GROUND_IDLE
            case 7:   // COMPLETE
            case 8:   // FAILED
            case 9:   // HOLDING
            case 10:  // OVERRIDDEN
            case 11:  // TARGET_LOST_HOVER
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
            case 10: return "OVERRIDDEN";
            case 11: return "TARGET_LOST_HOVER";
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
        // 同一 SN 只保留最新心跳。旧实现每帧 push_back,最近距离会在约 5 秒窗口内
        // 粘住历史最小值,并在清理时阶跃,直接污染闭合速度和动态阈值。
        for (auto& h : heartbeats_) {
            if (h->sn == msg->sn) {
                if (msg->timestamp_us > h->timestamp_us) {
                    h = msg;
                }
                return;
            }
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
            last_min_dist_ = -1.0;
            last_min_dist_time_ = ros::Time::now();
            closing_speed_smoothed_ = 0.0;
            last_nearest_sn_.clear();
            last_nearest_timestamp_us_ = 0;
            was_in_prox_zone_ = false;
            was_in_coll_zone_ = false;
            was_pushing_vertically_ = false;
            reverse_active_ = false;
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
        double nearest_dx = 0.0;
        double nearest_dy = 0.0;
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
                nearest_dx = dx;
                nearest_dy = dy;
                min_idx = i;
                found = true;
            }
        }

        // ========== 按闭合速度扩大反应距离 ==========
        // 首选双方 velocity_local 计算水平径向闭合速度；本机速度尚未就绪时才用
        // 距离差 fallback，并强制最小采样窗口，避免 20Hz GPS 噪声被微分放大。
        if (!found) {
            last_min_dist_ = -1.0;
            last_min_dist_time_ = ros::Time::now();
            closing_speed_smoothed_ = 0.0;
            last_nearest_sn_.clear();
            last_nearest_timestamp_us_ = 0;
            was_in_prox_zone_ = false;
            was_in_coll_zone_ = false;
            was_pushing_vertically_ = false;
            reverse_active_ = false;
            avoidance_pub_.publish(cmd);
            return;
        }

        const auto& nearest_sample = heartbeats_[min_idx];
        const ros::Time now = ros::Time::now();
        const bool neighbor_changed = (nearest_sample->sn != last_nearest_sn_);
        const bool new_distance_sample =
            neighbor_changed ||
            (nearest_sample->timestamp_us != last_nearest_timestamp_us_);
        const double horizontal_dist = std::sqrt(
            nearest_dx * nearest_dx + nearest_dy * nearest_dy);
        const bool use_relative_velocity =
            have_self_velocity_ && horizontal_dist > 0.1;
        double closing_speed_raw = closing_speed_smoothed_;
        const char* closing_speed_source = use_relative_velocity
            ? "relative_velocity" : "distance_fallback";

        if (neighbor_changed) {
            last_min_dist_ = -1.0;
            closing_speed_smoothed_ = 0.0;
            reverse_active_ = false;
        }
        if (new_distance_sample) {
            bool have_new_closing_speed = false;
            if (use_relative_velocity) {
                const double relative_vx =
                    static_cast<double>(nearest_sample->vx) - self_vx_ned_;
                const double relative_vy =
                    static_cast<double>(nearest_sample->vy) - self_vy_ned_;
                // range_rate = dot(r, v_rel)/|r|；closing_speed 取其相反数。
                closing_speed_raw = -(
                    nearest_dx * relative_vx + nearest_dy * relative_vy) /
                    horizontal_dist;
                have_new_closing_speed = true;
                last_min_dist_ = min_dist;
                last_min_dist_time_ = now;
            } else if (last_min_dist_ < 0.0) {
                // fallback 首帧只建基线，不产生速度。
                last_min_dist_ = min_dist;
                last_min_dist_time_ = now;
            } else {
                const double dt = (now - last_min_dist_time_).toSec();
                if (dt >= closing_speed_fallback_min_dt_ && dt < 10.0) {
                    closing_speed_raw = (last_min_dist_ - min_dist) / dt;
                    have_new_closing_speed = true;
                    last_min_dist_ = min_dist;
                    last_min_dist_time_ = now;
                } else if (dt >= 10.0) {
                    last_min_dist_ = min_dist;
                    last_min_dist_time_ = now;
                }
            }

            if (have_new_closing_speed) {
                closing_speed_raw =
                    std::max(-50.0, std::min(200.0, closing_speed_raw));
                closing_speed_smoothed_ =
                    closing_speed_alpha_ * closing_speed_raw +
                    (1.0 - closing_speed_alpha_) * closing_speed_smoothed_;
            }
            // 即便 fallback 尚未积累到最小窗口，也标记该心跳已处理；距离基线保留。
            last_nearest_sn_ = nearest_sample->sn;
            last_nearest_timestamp_us_ = nearest_sample->timestamp_us;
        }

        // cs 保留符号:正=接近，负=分离。
        const double cs = closing_speed_smoothed_;
        const char* closing_state = cs > reverse_min_closing_speed_
            ? "approaching"
            : (cs < -reverse_min_closing_speed_ ? "separating" : "uncertain");

        // === 2026-07-24: 阈值随闭合速度双向缩放 ===
        // cs_scale = clamp(1.0 + gain*cs, min_scale, max_scale)
        //   cs>0(接近)→ cs_scale > 1 → 阈值放大 → 早触发,留更多缓冲
        //   cs<0(分离)→ cs_scale < 1 → 阈值缩小 → 允许更近距离(已知要分开)
        //   cs=0 → cs_scale = 1 → 用基线阈值
        // 例(gain=0.3, min=0.5, max=2.0):
        //   cs = -2 m/s(快分离) → cs_scale = clamp(0.4, 0.5, 2.0) = 0.5 → 阈值×0.5
        //   cs = +1 m/s(慢接近)  → cs_scale = clamp(1.3, 0.5, 2.0) = 1.3 → 阈值×1.3
        //   cs = +3 m/s(快速接近)→ cs_scale = clamp(1.9, 0.5, 2.0) = 1.9 → 阈值×1.9
        double cs_scale = 1.0 + avoid_threshold_gain_ * cs;
        if (cs_scale < avoid_threshold_min_scale_) cs_scale = avoid_threshold_min_scale_;
        if (cs_scale > avoid_threshold_max_scale_) cs_scale = avoid_threshold_max_scale_;

        const double eff_max_avoid = max_avoid_distance_ * cs_scale;
        const double eff_prox      = proximity_threshold_ * cs_scale;
        const double eff_coll      = collision_threshold_ * cs_scale;
        // 关键:vertical_safe 也跟着缩 — 解决"dalt≈vertical_safe 卡在边界"死区
        //   cs>0 时需要更大垂直间隔,cs<0 时允许更小
        const double eff_vert      = vertical_safe_       * cs_scale;

        // ========== 距离超过阈值 → 不需避障 ==========
        if (min_dist > eff_max_avoid) {
            was_in_prox_zone_ = false;
            was_in_coll_zone_ = false;
            was_pushing_vertically_ = false;
            reverse_active_ = false;
            avoidance_pub_.publish(cmd);
            return;
        }

        // === 水平避让区迟滞 ===
        // 未进入时使用较小的 enter 阈值,已进入时使用较大的 exit 阈值。这样距离或
        // 动态阈值有小幅抖动时不会在 NORMAL/PROX/COLLISION 间来回切换。
        const double prox_enter = eff_prox * (1.0 - horizontal_hysteresis_);
        const double prox_exit  = eff_prox * (1.0 + horizontal_hysteresis_);
        const double coll_enter = eff_coll * (1.0 - horizontal_hysteresis_);
        const double coll_exit  = eff_coll * (1.0 + horizontal_hysteresis_);
        bool in_coll_zone = reverse_active_
            ? (min_dist < reverse_release_distance_)
            : (min_dist < (was_in_coll_zone_ ? coll_exit : coll_enter));
        bool in_prox_zone = min_dist < (was_in_prox_zone_ ? prox_exit : prox_enter);
        if (in_coll_zone) in_prox_zone = true;

        // ========== 直接从最近心跳取 sn / state / alt / velocity ==========
        const auto& nearest = heartbeats_[min_idx];
        const std::string& other_sn        = nearest->sn;
        const uint8_t      other_work_mode = nearest->work_mode;
        const uint8_t      other_phase     = nearest->phase;
        const double       other_alt       = nearest->alt;
        const float        other_vx        = nearest->vx;
        const float        other_vy        = nearest->vy;
        const float        other_vz        = nearest->vz;

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
            was_in_prox_zone_ = false;
            was_in_coll_zone_ = false;
            was_pushing_vertically_ = false;
            reverse_active_ = false;
            avoidance_pub_.publish(cmd);
            return;
        }

        was_in_prox_zone_ = in_prox_zone;
        was_in_coll_zone_ = in_coll_zone;

        // ========== 邻居"是否静止"判定(2026-07-24) ==========
        // 水平速度(m/s)的平方和,小于阈值 → 静止(HOLDING / 任务结束 / 等)。
        // 注:InterUavStatus.vx/vy 是 NED 系(comm_node / starling_bridge 内部已把
        //   mavros ENU 源转成 NED),但 sqrt(vx²+vy²) 是水平速度模长,对旋转不变
        //   (ENU→NED 或其它正交变换都保持模长),所以这里不需再做系变换。
        // EMA 滤波抗单次心跳抖动(下游速度 0.5s 内多帧平均)。
        const double neighbor_speed_sq =
            static_cast<double>(other_vx) * other_vx +
            static_cast<double>(other_vy) * other_vy;
        const double stationary_th_sq =
            stationary_speed_threshold_ * stationary_speed_threshold_;
        const bool neighbor_is_stationary = (neighbor_speed_sq < stationary_th_sq);
        // vertical_bias_soft_factor:1.0 = 完全软(垂直避让衰减为 vertical_bias_soft_);
        //                                  0.0 = 现行强避让(vertical_avoid_strong_)
        // 用 stationary_th_sq 当斜坡长度,从 0 → 1 线性插值,接近阈值时已部分衰减
        // (平滑过渡,避免 hard 切换造成姿态扰动)。
        double vertical_bias_soft_factor = 0.0;
        if (stationary_th_sq > 0.0 && neighbor_speed_sq < stationary_th_sq) {
            vertical_bias_soft_factor =
                1.0 - std::sqrt(neighbor_speed_sq / stationary_th_sq);
        }

        // ========== 我需要避让 ==========
        const double dalt_with_other = other_alt - self_alt;

        ROS_WARN_THROTTLE(1.0, "%s AVOIDING: reason=[%s], target=sn=%s dist=%.2fm dalt=%+.2fm (thresholds coll=%.1f prox=%.1f max=%.1f cs_raw=%+.1f cs=%+.1fm/s source=%s state=%s cs_scale=%.2f eff_vert=%.1f)",
                          TAG.c_str(), priority_reason.c_str(), other_sn.c_str(),
                          min_dist, dalt_with_other,
                          eff_coll, eff_prox, eff_max_avoid,
                          closing_speed_raw, cs, closing_speed_source,
                          closing_state, cs_scale, eff_vert);

        // ========== 垂直避让方向:buffer 规则 ==========
        // dalt_with_other = other_alt - self_alt(正 = 对方在我上方,即我在下面)
        //   dalt <  vertical_buffer_(我等高/更高/仅略低于对方)→ 向上爬升(climb),dir=+1
        //   dalt >= vertical_buffer_(我明显在对方下方)        → 向下俯冲(descend),dir=-1
        // vertical_bias 语义:向上爬升速率(正=向上,负=向下),executor 里 vz -= bias。
        const bool climb_up = (dalt_with_other < vertical_buffer_);
        const double vdir = climb_up ? +1.0 : -1.0;
        const char* vdir_str = climb_up ? "CLIMB-UP" : "DESCEND";

        // === 2026-07-24: 垂直迟滞 — 把 push/release 阈值分开,带死区 ===
        // 上一 tick 在爬 → 释放阈值 = eff_vert + hysteresis(必须爬到位才放手)
        // 上一 tick 未爬 → push  阈值 = eff_vert - hysteresis(必须显著低才重新推)
        // 死区 2*hysteresis(默认 2m)远大于 GPS 噪声(~0.5m),彻底切断 |dalt| 在阈值附近
        //   因噪声反复 toggle vbias 导致的 vz 在 -5↔0 之间疯狂跳、PX4 机身剧烈点头。
        // 不在 collision 区(min_dist>=eff_coll)时重置状态机,允许重新进入。
        if (!in_coll_zone) {
            if (reverse_active_) {
                ROS_WARN("%s   <<< REVERSE EXIT: collision zone cleared, dist=%.2fm release_dist=%.2fm",
                         TAG.c_str(), min_dist, reverse_release_distance_);
            }
            was_pushing_vertically_ = false;
            reverse_active_ = false;
        }

        if (in_coll_zone) {
            const double fabs_dalt = std::fabs(dalt_with_other);
            const double push_threshold = was_pushing_vertically_
                ? (eff_vert + vertical_hysteresis_)   // 已在爬:必须显著超过才放
                : (eff_vert - vertical_hysteresis_);  // 未爬:必须显著低于才推
            const bool should_push_vertical = (fabs_dalt < push_threshold);

            if (should_push_vertical) {
                // 高度还没拉开 → 按方向强避让,根据邻居是否静止做软化
                // 邻居静止 → 把 vertical_bias 衰减到 vertical_bias_soft_(默认 0),
                //   让"水平慢慢接近",不再把自己推高到飞不到航点。
                // 邻居动 → 现行 strong(5 m/s),正常强避让。
                const double bias_mag =
                    vertical_bias_soft_factor * vertical_bias_soft_ +
                    (1.0 - vertical_bias_soft_factor) * vertical_avoid_strong_;
                cmd.vertical_bias = vdir * bias_mag;
                was_pushing_vertically_ = true;
                const double critical_enter_distance = eff_coll * 0.6;
                const bool collision_critical = reverse_active_
                    ? (min_dist < reverse_release_distance_)
                    : (min_dist < critical_enter_distance);
                if (collision_critical) {
                    // 只有确认仍在接近时才允许首次进入倒车。倒车启动后锁存到离开
                    // critical 区，避免倒车自身造成 cs<0 后每帧开关。
                    cmd.lateral_blocked = false;
                    if (!reverse_active_ && cs > reverse_min_closing_speed_) {
                        reverse_active_ = true;
                        // 固定本次倒车的释放距离，避免倒车造成 cs<0、eff_coll 缩小后
                        // critical 边界瞬间收缩，引发前进/倒车循环。
                        reverse_release_distance_ = eff_coll * 0.7;
                        ROS_WARN("%s   >>> REVERSE ENTER: cs=%+.2fm/s > %.2f, target=%s release_dist=%.1fm",
                                 TAG.c_str(), cs, reverse_min_closing_speed_,
                                 other_sn.c_str(), reverse_release_distance_);
                    }

                    if (reverse_active_) {
                        cmd.speed_scale = 0.0;
                        cmd.reverse_allowed = true;
                    } else if (cs < -reverse_min_closing_speed_) {
                        // 已经过交汇点并确认分离:保持正向低速离开，绝不倒回交汇点。
                        cmd.speed_scale = separating_speed_scale_;
                        cmd.reverse_allowed = false;
                    } else {
                        // 闭合速度接近零、方向未确认:停车等待垂直避让，不盲目倒车。
                        cmd.speed_scale = 0.0;
                        cmd.reverse_allowed = false;
                    }
                    ROS_WARN_THROTTLE(1.0, "%s   → COLLISION-CRITICAL (dist<%.1f): state=%s reverse_active=%d speed_scale=%.2f vert=%s bias=%+.2f (soft_factor=%.2f)",
                                      TAG.c_str(),
                                      reverse_active_ ? reverse_release_distance_
                                                      : critical_enter_distance,
                                      closing_state, reverse_active_ ? 1 : 0,
                                      cmd.speed_scale, vdir_str, cmd.vertical_bias,
                                      vertical_bias_soft_factor);
                } else {
                    reverse_active_ = false;
                    cmd.speed_scale = 0.2;
                    ROS_WARN_THROTTLE(1.0, "%s   → COLLISION (dist<%.1f, |dalt|=%.1f%s eff_vert=%.1f): slow speed_scale=%.2f, vert=%s bias=%+.2f (soft_factor=%.2f, other_vh=%.2f)",
                                      TAG.c_str(), eff_coll, fabs_dalt,
                                      should_push_vertical ? "<" : ">=",
                                      eff_vert,
                                      cmd.speed_scale, vdir_str, cmd.vertical_bias,
                                      vertical_bias_soft_factor,
                                      std::sqrt(neighbor_speed_sq));
                }
            } else {
                // 高度已拉开(带迟滞余量):垂直不再动作(executor 会锁存保持高度),
                // 仅减速从上/下方通过
                cmd.vertical_bias = 0.0;
                cmd.speed_scale = 0.4;
                was_pushing_vertically_ = false;
                reverse_active_ = false;
                const char* hyst_side = was_pushing_vertically_ ? "+hyst" : "-hyst";
                (void)hyst_side;  // 占位,日志只用于表达"用了哪一侧迟滞"
                ROS_WARN_THROTTLE(1.0, "%s   → COLLISION-DIST but alt-separated (|dalt|=%.1f, eff_vert_thresh=%.1f): hold alt, speed_scale=%.2f",
                                  TAG.c_str(), fabs_dalt,
                                  push_threshold,
                                  cmd.speed_scale);
            }
        } else if (in_prox_zone) {
            // 接近区(prox<dist<coll):仅减速,不做垂直动作。
            // 垂直避让留到冲突区(<coll)触发,避免过早/过大的高度反应(40m 就抬高太夸张)。
            // 渐进式减速:在 proximity 边界处 scale=1.0(刚进入时不减速,避免远处就开始"刹车"看不出来),
            // 线性 ramp 到 collision 边界处 scale=0.3(让靠近碰撞时有明显减速效果)。
            double t = (eff_prox - min_dist) /
                       (eff_prox - eff_coll);
            t = std::max(0.0, std::min(1.0, t));
            cmd.speed_scale = 1.0 - 0.7 * t;  // 1.0@prox边界 → 0.3@coll边界
            ROS_WARN_THROTTLE(1.0, "%s   → PROXIMITY (%.1f<dist<%.1f): ramped speed_scale=%.2f (t=%.2f, no vertical, |dalt|=%.1f)",
                              TAG.c_str(), eff_coll, eff_prox,
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
