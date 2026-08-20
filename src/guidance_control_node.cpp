#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <cmath>
//===================制导修改====================
#include <algorithm>
//===================制导修改====================
#include <Eigen/Eigen>

#include "multi_uav_strike/guidance_strategies.h"
#include "multi_uav_strike/GuidancePnDebug.h"   // PN 坐标/LOS/速度链路 rosbag 诊断
#include "multi_uav_strike/TargetFilterDebug.h"  // 滤波前后对比 debug msg (2026-07-16)
#include "multi_uav_strike/one_euro_filter.h"  // 速度指令平滑(2026-07-16),消除聚类抖动→PD→UAV 晃
#include "multi_uav_strike/InterUavStatus.h"  // /inter_uav/uav_status 心跳(替代 other_uav_poses)
#include "multi_uav_strike/MissionState.h"
//===================制导修改====================
#include "multi_uav_strike/PrimaryImageTarget.h"
//===================制导修改====================

class GuidanceControlNode {
private:
    // ROS interfaces
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber target_pose_sub_;
    ros::Subscriber target_twist_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber los_angle_sub_;
    ros::Subscriber real_target_sub_;  // 真实目标位置用于评估
    ros::Subscriber enable_sub_;       // 使能控制（来自 mission_manager）
    ros::Subscriber wp2_fallback_target_sub_;  // 2026-08-20: WP2 fallback target (NWU PoseStamped)
    ros::Subscriber mode_sub_;        // 模式控制："strike" 或 "track"
    ros::Subscriber uav_speed_sub_;   // 拦截速度(来自 mission_manager, 下发到 InterceptGuidance)
    ros::Subscriber heartbeat_sub_;        // 邻居心跳 /inter_uav/uav_status(替代原 other_uav_poses)
    ros::Subscriber home_position_sub_;    // PX4 home,锁一次用于 GPS→NED 转换
    ros::Subscriber mavros_state_sub_;     // 真机解锁门控
    ros::Subscriber mission_state_sub_;    // 真机起飞高度完成门控
    //===================制导修改====================
    ros::Subscriber primary_image_target_sub_;
    //===================制导修改====================

    // Publishers
    ros::Publisher vel_cmd_pub_;          // setpoint_velocity/cmd_vel_unstamped
    ros::Publisher attitude_cmd_pub_;     // setpoint_attitude/attitude
    ros::Publisher attitude_rates_pub_;   // setpoint_attitude/cmd_vel (角速率)
    ros::Publisher thrust_cmd_pub_;       // thrust
    ros::Publisher flight_mode_pub_;      // flight_mode
    ros::Publisher strike_eval_pub_;      // 打击评估结果
    ros::Publisher thrust_dir_pub_;       // 推力方向可视化
    ros::Publisher intercept_point_pub_; // 拦截点可视化
    ros::Publisher debug_pub_;           // 调试信息
    ros::Publisher target_filter_debug_pub_;  // 2026-07-16: 速度滤波前后对比 debug topic
    ros::Publisher pn_debug_pub_;             // PN 输入/LOS/三阶段速度，供 rosbag 离线复盘
    ros::Publisher velocity_active_pub_;      // 实际发出首帧速度后确认无缝接管
    std::string viz_frame_;              // RViz 帧名 "<ns>/map"(多机分离由 static TF 偏移)

    // Timer for guidance computation
    ros::Timer guidance_timer_;

    // Current state
    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::PoseStamped current_target_pose_;
    //=======高度下降===========//
    // 与 MissionManager 命中判定同源的实时原始目标坐标。current_target_pose_
    // 仍保留给原有平滑水平制导；下降时序不能使用滞后的滤波坐标提前释放高度。
    geometry_msgs::PoseStamped raw_target_pose_;
    bool raw_target_pose_valid_ = false;
    //=======高度下降===========//
    geometry_msgs::TwistStamped current_target_twist_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::Point real_target_pos_;  // 真实目标位置

    bool is_uav_pose_received_ = false;
    bool is_target_pose_received_ = false;
    bool is_target_twist_received_ = false;
    bool is_los_received_ = false;
    bool is_real_target_received_ = false;
    bool mavros_state_received_ = false;
    bool mission_state_received_ = false;
    bool mavros_connected_ = false;
    bool mavros_armed_ = false;
    uint8_t current_mission_phase_ = 0;

    // 使能控制：SEARCH_ONLY 模式下 guidance 不工作
    bool is_enabled_;
    std::string current_mode_;  // "strike" 或 "track"

    // Guidance strategy
    std::unique_ptr<multi_uav_strike::GuidanceStrategy> guidance_strategy_;
    multi_uav_strike::GuidanceStrategyType current_strategy_type_;
    std::string current_strategy_name_;

    // Control mode
    int flight_mode_velocity_;
    int flight_mode_attitude_;

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;
    std::string vel_cmd_topic_;
    std::string attitude_cmd_topic_;
    std::string attitude_rates_topic_;
    std::string thrust_cmd_topic_;

    // GPS 参考点（GPS -> NED 转换用，由 PX4 /mavros/home_position/home 首次锁定）
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;
    bool ref_initialized_ = false;  // 是否已收到 PX4 home(锁一次,后续 PX4 home 变化不更新)

    // 跟踪约束参数
    double min_altitude_;            // 最小高度限制
    double max_tracking_distance_;   // 最大跟踪距离
    double desired_tracking_angle_;   // 期望跟踪角度（目标在机头前下方）

    // TRACK模式专用参数
    double track_altitude_;          // 固定跟踪高度（m）
    double track_horiz_dist_;        // 水平安全距离（m，stand-off距离，仅作为低空 fallback）
    double track_P_;                 // P控制增益
    double track_I_;                 // I控制增益
    double track_max_speed_;         // 最大速度限制（m/s）
    double track_k_approach_;        // 距离→速度的斜率（m/s per m，距离每超出 stand-off 1m 速度增加多少）
    double track_integral_x_;        // PI积分项 x
    double track_integral_y_;        // PI积分项 y

    // ===== 新增跟踪几何参数（基于视线角俯仰角跟踪） =====
    // 思路：固定跟踪高度下，根据期望的 LOS 俯仰角自动算出 stand-off 水平距离：
    //       tan(pitch) = alt_diff / horiz_dist  →  horiz_dist = alt_diff / tan(pitch)
    // 这样目标始终出现在云台视野的上半部分（45° 云台默认指向，30° 目标 = 视野上方 15°），
    // 既保证相机持续看到目标，又留出俯冲空间给后续 strike。
    double desired_los_pitch_deg_;   // 期望 LOS 俯仰角（目标在水平线下方角度），默认 30°
    double min_horiz_dist_;          // stand-off 水平距离下限（避免目标几乎在脚下）
    double max_horiz_dist_;          // stand-off 水平距离上限（避免离得太远看不到）
    double yaw_los_threshold_deg_;   // 偏航 LOS 角阈值（target 偏离航向超过此值时优先转向不前进），默认 45°

    // 偏航角速度控制参数（PX4 SITL 用）
    double los_kp_yaw_;              // 偏航 P 控制增益
    double los_max_rate_;            // 偏航角速度限幅（rad/s）

    // === 2026-07-16: 速度指令平滑(One Euro Filter) ===
    // 背景:即使 mission_manager 已经平滑了 target_estimated_pose,这里 P 控制
    //   v = K_p * (tgt - uav) 还是会把任何位置残留抖动乘以增益变成速度抖动,
    //   PX4 高刚度控制下就让 UAV 上下左右晃。本节点再过一次 One Euro Filter
    //   压平速度指令,从根本上消除"速度量调"造成的视觉抖动。
    multi_uav_strike::OneEuroFilter1D vel_filter_x_;
    multi_uav_strike::OneEuroFilter1D vel_filter_y_;
    multi_uav_strike::OneEuroFilter1D vel_filter_z_;
    multi_uav_strike::OneEuroFilter1D vel_filter_yaw_;
    //===================制导修改====================
    // main PN增强使用独立目标滤波状态；不得覆盖IMAGE/GEO使用的原始目标坐标。
    multi_uav_strike::OneEuroFilter1D pn_target_filter_x_;
    multi_uav_strike::OneEuroFilter1D pn_target_filter_y_;
    multi_uav_strike::OneEuroFilter1D pn_target_filter_z_;
    geometry_msgs::PoseStamped pn_filtered_target_pose_;
    bool pn_filtered_target_valid_ = false;
    ros::Time pn_last_target_update_time_;
    // === 2026-08-20: Attack(105) WP1→WP2 强制 guidance 时,接收 mission_manager 发的 WP2 fallback 目标 ===
    //   绕过 triggerStrike() 的 cluster lock gate,直接用 WP2 自身作为 PN 目标。
    //   一旦 cluster 被 target_estimator 接管 (target_estimated_pose 持续更新),
    //   mission_manager 自动停 fallback (cluster lock 路径),这里 wp2_fallback_valid_ 也会 stale timeout。
    geometry_msgs::PoseStamped wp2_fallback_pose_;
    bool wp2_fallback_valid_ = false;
    ros::Time wp2_fallback_last_seen_;
    double wp2_fallback_stale_timeout_sec_ = 0.5;  // > 100ms publish 周期 ⇒ 视为 mission_manager 停了
    double pn_cluster_stale_timeout_sec_ = 0.5;
    double pn_cluster_stale_speed_scale_ = 0.33;
    bool pn_cluster_stale_latch_ = false;
    geometry_msgs::Vector3 last_known_uav_velocity_ned_;
    geometry_msgs::Point prev_uav_position_nwu_;
    ros::Time prev_uav_position_time_;
    bool has_prev_uav_position_ = false;
    //===================制导修改====================
    ros::Time                    vel_filter_last_time_;
    // 目标位置突变超过此值时，在 targetPoseCallback 内、下一次 PN 计算之前重置滤波器。
    static constexpr double kTargetSwitchJumpM = 5.0;
    // 上一帧目标位置(NWU),用于 targetPoseCallback 内做"目标切换"突变检测
    double last_target_pos_x_  = 0.0;
    double last_target_pos_y_  = 0.0;
    double last_target_pos_z_  = 0.0;
    bool   last_target_pos_valid_ = false;
    // === 2026-08-04: STRIKE 反馈环解耦 (Layer 6) — cluster stale 软兜底 ===
    //   target_estimated_pose 没新消息 → cluster 已 stale,UAV 不该再激进 chase
    //   last_known 位置(否则单帧噪声会被 PN 当成新信号反馈到 body)
    //   threshold 0.5s:target_estimator 10Hz,正常每 0.1s 一帧;0.5s 才有新意
    ros::Time last_target_update_time_;
    static constexpr double kClusterStaleThresholdSec = 0.5;
    bool cluster_stale_latch_ = false;  // 进入过 stale(避免反复 warn)

    // === 2026-08-05: STRIKE 反馈环解耦 (Handover 修复) ===
    //   waypoint_executor 在 STRIKE 接管前最后发的速度(5.14, -0.81, -1.61) 与 PN 接管后
    //   第一帧 (2.83, 0.19, 0) 差距很大。如果 OneEuro filter 从 x_prev=0 开始,
    //   会先塌陷到 0.33 m/s 再爬回 5.27,过程中 yaw rate 也会饱和摆动。
    //   修法:guidance enable 时用 UAV 当前实际速度初始化 filter,x_prev 跳过 0 状态。
    // geometry_msgs::Vector3 last_known_uav_velocity_ned_;  // pose-delta 计算
    // geometry_msgs::Point   prev_uav_pos_ned_;
    // ros::Time               prev_uav_pos_time_;
    bool                    has_prev_uav_pos_   = false;
    bool                    was_enabled_prev_   = false;

    // STRIKE模式参数
    double strike_min_altitude_;      // 开始下降的最小高度阈值

    // 打击评估相关
    double strike_distance_threshold_;  // 打击成功距离阈值(m)
    bool first_strike_evaluated_;      // 是否已评估第一次打击
    double min_strike_distance_;       // 第一次打击最小距离
    double strike_altitude_;            // 打击时无人机高度
    double strike_time_;                // 打击时间戳
    bool is_approaching_;               // 是否正在接近目标
    double prev_distance_;              // 上一时刻距离
    // === 2026-07-26: strike_eval 总开关 (默认关闭) ===
    //   现状 /target_position 是 target_motion_simulator 发的 ground truth,primary_index 配置错误
    //   (yaml 里 primary_index: 10 超出范围,fallback 到 0 → x=60,y=-100 不是用户真目标),
    //   evaluateStrike 算 min_dist 永远偏大,STRIKE EVAL 一直 FAILURE 但不是真没打中。
    //   等 primary_index 修对 / 或者干脆删除此评估路径后再开。
    //   真正任务完成判定走 attack_cmd 协议 (GS 收到 type=1 + fire action=0)。
    bool enable_strike_evaluation_ = false;

    // 机间避障相关 — 邻居表用 /inter_uav/uav_status 心跳替代原 PoseArray,
    //   sn + lat/lon/alt 自带,自环过滤后直接迭代算距离(NED 转换在 applyInterUavAvoidance 内按需进行)
    std::string uav_device_sn_;   // 自环过滤键(launch 传入 device_sn,默认回落 uav_name)
    std::vector<multi_uav_strike::InterUavStatus::ConstPtr> heartbeats_;
    ros::Time last_heartbeat_purge_;
    double avoidance_safe_distance_;

    //===================制导修改====================
    enum class StrikeVisionState { GEO_FALLBACK, IMAGE_GUIDANCE };
    std::string strike_guidance_logic_ = "image_geo_fallback";
    multi_uav_strike::PrimaryImageTarget current_image_target_;
    bool image_target_received_ = false;
    uint32_t last_image_frame_seq_ = 0;
    uint64_t last_image_stamp_us_ = 0;
    int image_confirm_count_ = 0;
    int image_confirm_frames_ = 1;
    double image_visible_timeout_sec_ = 0.4;
    double image_reacquire_blend_sec_ = 0.7;
    double image_max_speed_ = 5.0;
    //===================制导修改====================
    // 原值保留：double image_max_vertical_speed_ = 1.0;
    double image_max_vertical_speed_ = 2.0;
    //=======高度下降===========//
    double strike_descent_rate_ = 2.0;
    double strike_vertical_lag_ = 2.0;
    double strike_commit_radius_ = 3.0;
    double strike_min_commit_altitude_ = 2.0;
    // === 2026-08-19: STRIKE 闭环下降 ===
    //   跟踪 UAV 实际下降率 (PX4 限速 / 电机饱和时实际比 commanded 小),
    //   feasibleHorizontalSpeedForDescent 据此反推水平速度,确保水平/垂直同时到达。
    double actual_descent_rate_ema_ = 0.0;
    double last_altitude_nwu_ = 0.0;
    ros::Time last_altitude_time_;
    bool altitude_filter_initialized_ = false;
    //=============闭环下降=============//
    //=======高度下降===========//
    bool strike_slant_descent_enabled_ = true;
    double strike_slant_descent_max_angle_deg_ = 25.0;
    double strike_slant_min_horizontal_distance_m_ = 1.0;
    //===================制导修改====================
    double image_center_deadband_ = 0.03;
    double image_ray_alpha_ = 0.25;
    double image_yaw_kp_ = 0.6;
    double image_yaw_rate_max_ = 0.5;
    //===================制导修改====================
    // 2026-08-05 bag 修复：近距减速、垂直视线退化保护、启动新鲜度和指令斜率限制。
    double strike_slowdown_radius_m_ = 10.0;
    double strike_horizontal_stop_radius_m_ = 1.0;
    //========速度修改========
    // false: IMAGE/GEO 水平速度直接使用 Attack task_speed，不按目标距离减速或归零。
    bool strike_terminal_speed_schedule_enabled_ = false;
    //========速度修改========
    double strike_terminal_world_direction_radius_m_ = 8.0;
    double strike_yaw_freeze_radius_m_ = 3.0;
    double strike_target_pose_timeout_sec_ = 0.5;
    double strike_max_horizontal_accel_ = 2.0;
    //===================制导修改====================
    // 原值保留：double strike_max_vertical_accel_ = 1.0;
    double strike_max_vertical_accel_ = 1.5;
    //===================制导修改====================
    double strike_max_yaw_accel_ = 1.0;
    double attack_max_speed_ = 40.0;
    // === 2026-08-20: 图像居中终端冲刺 (image-terminal chase) ===
    //   进入条件: image_ready && bbox 居中 && g.horizontal ≤ chase_min_horiz_m_
    //   chase 中 vx/vy/vz 完全沿 filtered_image_ray_nwu_,speed = track_max_speed_,
    //   绕过 terminalHorizontalSpeed / feasibleHorizontalSpeedForDescent /
    //   computeSlantDescentSpeed / enforcePreCommitAltitudeGuard 全部前馈钳位.
    bool   strike_terminal_image_chase_enabled_      = true;
    double strike_terminal_image_chase_min_horiz_m_  = 16.0;
    double strike_terminal_image_chase_latch_sec_    = 0.5;
    // 2026-08-20: chase 触发用更宽的死区(>image_center_deadband_),
    //   17-43-21 bag 在 horiz=1.37m 时 bbox 漂出 0.06 带导致 chase 提前退出,
    //   放宽到 0.10 允许更大波动, 让 chase 覆盖到命中圈边缘。ray 在 ~1m 内仍可信。
    double strike_terminal_image_chase_center_band_ = 0.10;
    bool   terminal_image_chase_active_              = false;
    ros::Time terminal_image_chase_enter_time_;
    // chase enter/exit 边沿日志 (2026-08-20),last=true/false 翻转时打 WARN
    bool   last_chase_log_active_                    = false;
    // 本次 Attack 一旦得到有效世界坐标便锁存；图像或 cluster 后续消失时
    // GEO 继续使用该坐标，直到 guidance 被明确关闭。
    bool strike_target_latched_valid_ = false;
    ros::Time last_target_pose_receive_time_;
    ros::Time strike_guidance_enable_time_;
    ros::Time last_strike_cmd_time_;
    //===================制导修改====================
    Eigen::Vector3d filtered_image_ray_nwu_{0.0, 0.0, -1.0};
    bool image_ray_filter_valid_ = false;
    StrikeVisionState strike_vision_state_ = StrikeVisionState::GEO_FALLBACK;
    ros::Time strike_vision_switch_time_;
    geometry_msgs::Twist strike_blend_from_ned_;
    geometry_msgs::Twist last_strike_cmd_ned_;
    bool last_strike_cmd_valid_ = false;
    //===================制导修改====================

public:
    GuidanceControlNode() : nh_private_("~"),
        first_strike_evaluated_(false),
        min_strike_distance_(std::numeric_limits<double>::max()),
        strike_altitude_(0.0),
        strike_time_(0.0),
        is_approaching_(true),
        prev_distance_(std::numeric_limits<double>::max()),
        is_enabled_(false),
        current_mode_("strike"),
        track_integral_x_(0.0),
        track_integral_y_(0.0),
        strike_min_altitude_(20.0),
        track_k_approach_(0.5),
        desired_los_pitch_deg_(30.0),
        min_horiz_dist_(5.0),
        max_horiz_dist_(100.0),
        yaw_los_threshold_deg_(45.0),
        use_sim_(true) {
        initParams();
        initSubscribers();
        initPublishers();
        initGuidanceStrategy();

        // Create timer for guidance loop (50 Hz)
        guidance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / 50.0),
            &GuidanceControlNode::guidanceTimerCallback,
            this);

        ROS_INFO("Guidance Control Node initialized with strategy: %s (disabled by default)",
                 current_strategy_name_.c_str());
    }

    void initParams() {
        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, true);

        // device_sn(心跳 sn)用于自环过滤 — 命名约定 "1-1" / "1-2",默认回落到 uav_name
        nh_private_.param<std::string>("uav_device_sn", uav_device_sn_, "");

        // GPS 参考点不再从 yaml 硬编码,而由 homePositionCallback() 从 PX4 /mavros/home_position/home 首次锁定
        // (yaml 里仍可配 ref_lat/lon/alt 作为节点启动到 home 到达之间的临时 fallback,但实际 GPS→NED 在 ref_initialized_ 后才生效)

        // 根据 use_sim 设置 topic 名称
        if (use_sim_) {
            pose_topic_ = "quad/pose";
            vel_cmd_topic_ = "quad/setpoint_velocity/cmd_vel_unstamped";
            attitude_cmd_topic_ = "quad/setpoint_attitude/attitude";
            attitude_rates_topic_ = "quad/setpoint_attitude/cmd_vel";
            thrust_cmd_topic_ = "quad/thrust";
        } else {
            pose_topic_ = "mavros/local_position/pose";
            vel_cmd_topic_ = "mavros/setpoint_velocity/cmd_vel_unstamped";
            attitude_cmd_topic_ = "mavros/setpoint_attitude/attitude";
            attitude_rates_topic_ = "mavros/setpoint_attitude/cmd_vel";
            thrust_cmd_topic_ = "mavros/setpoint_attitude/thrust";
        }

        // Strategy selection parameter
        std::string strategy_str;
        nh_private_.param<std::string>("guidance_strategy", strategy_str, "intercept");

        if (strategy_str == "intercept") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        } else if (strategy_str == "minsnap") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::MINSNAP;
            current_strategy_name_ = "minsnap";
        } else if (strategy_str == "los") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::LOS;
            current_strategy_name_ = "los";
        } else {
            ROS_WARN("[Guidance] Unknown strategy '%s', defaulting to 'intercept'", strategy_str.c_str());
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        }

        nh_private_.param<int>("flight_mode_velocity", flight_mode_velocity_, 0);
        nh_private_.param<int>("flight_mode_attitude", flight_mode_attitude_, 1);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<bool>("enable_strike_evaluation",  enable_strike_evaluation_,  false);  // strike_eval 总开关(默认关闭,见字段注释)

        // TRACK模式专用参数
        nh_private_.param<double>("track_altitude", track_altitude_, 30.0);       // 固定跟踪高度
        nh_private_.param<double>("track_horiz_dist", track_horiz_dist_, 8.0);    // 水平安全距离（仅低空 fallback）
        nh_private_.param<double>("track_P", track_P_, 1.0);                     // P控制增益
        nh_private_.param<double>("track_I", track_I_, 0.1);                     // I控制增益
        nh_private_.param<double>("uav_speed", track_max_speed_, 5.0);     // 最大速度限制
        // 距离→速度的连续斜率（消除内/外边界的硬切换：每超出 stand-off 1m 速度增加多少 m/s）
        nh_private_.param<double>("track_k_approach", track_k_approach_, 0.5);

        // ===== 新增跟踪几何参数（视线角跟踪） =====
        // 摄像头默认 45° 向下，30° 目标 = 视野上方 15°，方便切换 strike 时下俯
        nh_private_.param<double>("desired_los_pitch_deg", desired_los_pitch_deg_, 30.0);
        nh_private_.param<double>("min_horiz_dist", min_horiz_dist_, 5.0);
        nh_private_.param<double>("max_horiz_dist", max_horiz_dist_, 100.0);
        // 目标偏离航向 > 45° 时只转向不前进（避免大角度侧向/后退冲撞障碍）
        nh_private_.param<double>("yaw_los_threshold_deg", yaw_los_threshold_deg_, 45.0);

        // 偏航角速度控制（PX4 SITL）
        nh_private_.param<double>("los_kp_yaw", los_kp_yaw_, 1.5);
        nh_private_.param<double>("los_max_rate", los_max_rate_, 1.2);

        // === 2026-07-16: 速度指令 One Euro Filter 参数 ===
        //   调参经验:还看着 UAV 抖 → 把 vel_min_cutoff_hz 降(0.5~1.0);
        //            UAV 反应迟钝跟不上目标 → 升 vel_beta(0.02~0.05)
        //   速度通道比位置通道可以略激进(响应要快),所以默认 beta 比位置稍大。
        // === 2026-08-01: 收紧默认值, 消除 ~2 Hz 机体俯仰振荡 ===
        //   现象: rosbag 分析发现 cmd_vel_unstamped 方向 ±2°@1 Hz 摆动,
        //         触发 PX4 俯仰/滚转交替, 但 yaw 不抖.
        //   旧默认值 1.5Hz + beta=0.015 对 1-2 Hz 摆动几乎是直通的
        //   (beta 让截止频率随 dx/dt 升高,反而放行稳态摆动).
        //   新默认值 0.5Hz + beta=0.001 把 1 Hz+ 成分削掉 ~12 dB,
        //   PN 输出的 0.3 m/s std 摆动会被压到 ~0.05 m/s.
        //===================制导修改====================
        // 按STRIKE选择隔离输出滤波默认值：PN吸收main增强值，IMAGE保持local_save实测值。
        std::string strike_logic_for_filter_defaults;
        nh_private_.param<std::string>("strike_guidance_logic",
                                       strike_logic_for_filter_defaults,
                                       "image_geo_fallback");
        const bool use_main_pn_filter_defaults =
            strike_logic_for_filter_defaults == "pn" ||
            strike_logic_for_filter_defaults == "legacy_pn";
        const double default_vel_min_cutoff = use_main_pn_filter_defaults ? 0.25 : 0.5;
        const double default_vel_beta = use_main_pn_filter_defaults ? 0.02 : 0.001;
        const double default_vel_d_cutoff = use_main_pn_filter_defaults ? 0.5 : 1.0;
        //===================制导修改====================
        double vel_min_cutoff, vel_beta, vel_d_cutoff;
        nh_private_.param<double>("vel_smooth_min_cutoff_hz", vel_min_cutoff,
                                  default_vel_min_cutoff);
        nh_private_.param<double>("vel_smooth_beta", vel_beta, default_vel_beta);
        nh_private_.param<double>("vel_smooth_d_cutoff_hz", vel_d_cutoff,
                                  default_vel_d_cutoff);
        vel_filter_x_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_y_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_z_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_yaw_ = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        //===================制导修改====================
        double pn_target_min_cutoff, pn_target_beta, pn_target_d_cutoff;
        nh_private_.param<double>("pn_target_filter_min_cutoff_hz",
                                  pn_target_min_cutoff, 1.0);
        nh_private_.param<double>("pn_target_filter_beta", pn_target_beta, 0.005);
        nh_private_.param<double>("pn_target_filter_d_cutoff_hz",
                                  pn_target_d_cutoff, 1.0);
        pn_target_filter_x_ = multi_uav_strike::OneEuroFilter1D(
            pn_target_min_cutoff, pn_target_beta, pn_target_d_cutoff);
        pn_target_filter_y_ = multi_uav_strike::OneEuroFilter1D(
            pn_target_min_cutoff, pn_target_beta, pn_target_d_cutoff);
        pn_target_filter_z_ = multi_uav_strike::OneEuroFilter1D(
            pn_target_min_cutoff, pn_target_beta, pn_target_d_cutoff);
        nh_private_.param<double>("pn_cluster_stale_timeout_sec",
                                  pn_cluster_stale_timeout_sec_, 0.5);
        nh_private_.param<double>("pn_cluster_stale_speed_scale",
                                  pn_cluster_stale_speed_scale_, 0.33);
        pn_cluster_stale_timeout_sec_ = std::max(0.1, pn_cluster_stale_timeout_sec_);
        pn_cluster_stale_speed_scale_ = clampValue(
            pn_cluster_stale_speed_scale_, 0.0, 1.0);
        //===================制导修改====================
        vel_filter_last_time_ = ros::Time();
        ROS_INFO("[Guidance] Vel cmd OneEuro: min_cutoff=%.2fHz beta=%.3f d_cutoff=%.2fHz",
                 vel_min_cutoff, vel_beta, vel_d_cutoff);

        // STRIKE模式专用参数
        nh_private_.param<double>("strike_min_altitude", strike_min_altitude_, 20.0);  // 开始下降的最小高度阈值

        //===================制导修改====================
        nh_private_.param<std::string>("strike_guidance_logic", strike_guidance_logic_,
                                       "image_geo_fallback");
        //===================制导修改====================
        if (strike_guidance_logic_ == "legacy_pn") {
            ROS_WARN("[Guidance] strike_guidance_logic=legacy_pn is deprecated; using pn");
            strike_guidance_logic_ = "pn";
        } else if (strike_guidance_logic_ != "image_geo_fallback" &&
                   strike_guidance_logic_ != "pn") {
            ROS_ERROR("[Guidance] Invalid strike_guidance_logic='%s'; fallback to image_geo_fallback",
                      strike_guidance_logic_.c_str());
            strike_guidance_logic_ = "image_geo_fallback";
        }
        ROS_WARN("[Guidance] STRIKE guidance selected: %s", strike_guidance_logic_.c_str());
        //===================制导修改====================
        nh_private_.param<int>("image_confirm_frames", image_confirm_frames_, 1);
        nh_private_.param<double>("image_visible_timeout_sec", image_visible_timeout_sec_, 0.4);
        nh_private_.param<double>("image_reacquire_blend_sec", image_reacquire_blend_sec_, 0.7);
        nh_private_.param<double>("image_max_speed", image_max_speed_, 5.0);
        //===================制导修改====================
        // 原参数默认值保留：
        // nh_private_.param<double>("image_max_vertical_speed", image_max_vertical_speed_, 1.0);
        nh_private_.param<double>("image_max_vertical_speed", image_max_vertical_speed_, 2.0);
        //=======高度下降===========//
        nh_private_.param<double>("strike_descent_rate", strike_descent_rate_, 2.0);
        nh_private_.param<double>("strike_vertical_lag", strike_vertical_lag_, 2.0);
        nh_private_.param<double>("strike_commit_radius", strike_commit_radius_, 3.0);
        nh_private_.param<double>("strike_min_commit_altitude",
                                  strike_min_commit_altitude_, 2.0);
        //=======高度下降===========//
        nh_private_.param<bool>("strike_slant_descent_enabled",
                                strike_slant_descent_enabled_, true);
        nh_private_.param<double>("strike_slant_descent_max_angle_deg",
                                  strike_slant_descent_max_angle_deg_, 25.0);
        nh_private_.param<double>("strike_slant_min_horizontal_distance_m",
                                  strike_slant_min_horizontal_distance_m_, 1.0);
        //===================制导修改====================
        nh_private_.param<double>("image_center_deadband", image_center_deadband_, 0.03);
        nh_private_.param<double>("image_ray_alpha", image_ray_alpha_, 0.25);
        nh_private_.param<double>("image_yaw_kp", image_yaw_kp_, 0.6);
        nh_private_.param<double>("image_yaw_rate_max", image_yaw_rate_max_, 0.5);
        //===================制导修改====================
        nh_private_.param<double>("strike_slowdown_radius_m", strike_slowdown_radius_m_, 10.0);
        nh_private_.param<double>("strike_horizontal_stop_radius_m",
                                  strike_horizontal_stop_radius_m_, 1.0);
        //========速度修改========
        nh_private_.param<bool>("strike_terminal_speed_schedule_enabled",
                                strike_terminal_speed_schedule_enabled_, false);
        //========速度修改========
        nh_private_.param<double>("strike_terminal_world_direction_radius_m",
                                  strike_terminal_world_direction_radius_m_, 8.0);
        nh_private_.param<double>("strike_yaw_freeze_radius_m", strike_yaw_freeze_radius_m_, 3.0);
        nh_private_.param<double>("strike_target_pose_timeout_sec",
                                  strike_target_pose_timeout_sec_, 0.5);
        nh_private_.param<double>("strike_max_horizontal_accel",
                                  strike_max_horizontal_accel_, 2.0);
        //===================制导修改====================
        // 原参数默认值保留：
        // nh_private_.param<double>("strike_max_vertical_accel",
        //                           strike_max_vertical_accel_, 1.0);
        nh_private_.param<double>("strike_max_vertical_accel",
                                  strike_max_vertical_accel_, 1.5);
        //===================制导修改====================
        nh_private_.param<double>("strike_max_yaw_accel", strike_max_yaw_accel_, 1.0);
        nh_private_.param<double>("attack_max_speed", attack_max_speed_, 40.0);
        // === 2026-08-20: 图像居中终端冲刺 ===
        nh_private_.param<bool>("strike_terminal_image_chase_enabled",
                                strike_terminal_image_chase_enabled_, true);
        nh_private_.param<double>("strike_terminal_image_chase_min_horiz_m",
                                  strike_terminal_image_chase_min_horiz_m_, 10.0);
        nh_private_.param<double>("strike_terminal_image_chase_latch_sec",
                                  strike_terminal_image_chase_latch_sec_, 0.5);
        nh_private_.param<double>("strike_terminal_image_chase_center_band",
                                  strike_terminal_image_chase_center_band_, 0.06);
        strike_terminal_image_chase_min_horiz_m_ =
            std::max(strike_terminal_image_chase_min_horiz_m_, 0.0);
        strike_terminal_image_chase_latch_sec_ =
            std::max(strike_terminal_image_chase_latch_sec_, 0.0);
        strike_terminal_image_chase_center_band_ =
            std::max(strike_terminal_image_chase_center_band_, image_center_deadband_);
        attack_max_speed_ = std::max(0.1, std::min(40.0, attack_max_speed_));
        strike_slowdown_radius_m_ = std::max(
            strike_slowdown_radius_m_, strike_horizontal_stop_radius_m_ + 0.1);
        strike_terminal_world_direction_radius_m_ = std::max(
            strike_terminal_world_direction_radius_m_, strike_horizontal_stop_radius_m_);
        //===================制导修改====================
        image_max_vertical_speed_ = std::max(0.1, image_max_vertical_speed_);
        //=======高度下降===========//
        strike_descent_rate_ = std::max(0.1, strike_descent_rate_);
        strike_vertical_lag_ = std::max(0.1, strike_vertical_lag_);
        strike_commit_radius_ = std::max(0.1, strike_commit_radius_);
        strike_min_commit_altitude_ = std::max(0.1, strike_min_commit_altitude_);
        //=======高度下降===========//
        strike_max_vertical_accel_ = std::max(0.1, strike_max_vertical_accel_);
        strike_slant_min_horizontal_distance_m_ = std::max(
            0.1, strike_slant_min_horizontal_distance_m_);
        strike_slant_descent_max_angle_deg_ = clampValue(
            strike_slant_descent_max_angle_deg_, 1.0, 60.0);
        //===================制导修改====================
        //===================制导修改====================
        //===================制导修改====================

        ROS_INFO("[Guidance] Mode: %s, pose_topic: %s", use_sim_ ? "SIMULATION" : "PX4 SITL", pose_topic_.c_str());
    }

    void initSubscribers() {
        target_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &GuidanceControlNode::targetPoseCallback, this);

        target_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &GuidanceControlNode::targetTwistCallback, this);

        uav_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &GuidanceControlNode::uavPoseCallback, this);

        los_angle_sub_ = nh_.subscribe(
            "target_los_angle", 10,
            &GuidanceControlNode::losAngleCallback, this);

        // 订阅真实目标位置用于评估（用于判断是否真正击中目标）
        real_target_sub_ = nh_.subscribe(
            "/target_position", 10,
            &GuidanceControlNode::realTargetCallback, this);

        // 使能控制（SEARCH_ONLY 模式下不工作）
        enable_sub_ = nh_.subscribe(
            "guidance/enable", 10,
            &GuidanceControlNode::enableCallback, this);

        // 模式控制（"strike" 或 "track"）
        mode_sub_ = nh_.subscribe(
            "guidance/mode", 10,
            &GuidanceControlNode::modeCallback, this);

        // 拦截速度(来自 mission_manager,转发给 InterceptGuidance)
        uav_speed_sub_ = nh_.subscribe(
            "guidance/guidance_speed", 10,
            &GuidanceControlNode::uavSpeedCallback, this);

        // 邻居心跳 /inter_uav/uav_status — 替代原 other_uav_poses(PoseArray),
        //   心跳自带 sn + lat/lon/alt,避免重复数据源和 lat/lon→sn 反查的 hack。
        heartbeat_sub_ = nh_.subscribe(
            "/inter_uav/uav_status", 50,
            &GuidanceControlNode::heartbeatCallback, this);

        // PX4 home 订阅（仅用于 GPS→NED 转换的参考原点）
        home_position_sub_ = nh_.subscribe(
            "mavros/home_position/home", 10,
            &GuidanceControlNode::homePositionCallback, this);

        if (!use_sim_) {
            mavros_state_sub_ = nh_.subscribe(
                "mavros/state", 10,
                &GuidanceControlNode::mavrosStateCallback, this);
            mission_state_sub_ = nh_.subscribe(
                "mission/mission_state", 10,
                &GuidanceControlNode::missionStateCallback, this);
        }

        //===================制导修改====================
        primary_image_target_sub_ = nh_.subscribe(
            "guidance/primary_image_target", 10,
            &GuidanceControlNode::primaryImageTargetCallback, this);
        //===================制导修改====================

        // === 2026-08-20: WP2 fallback target subscription ===
        //   Attack(105) WP1→WP2 强制 guidance 路径:cluster 未 lock 时,
        //   mission_manager 用 WP2 自身作为 PN 目标 publish 到这个 topic。
        wp2_fallback_target_sub_ = nh_.subscribe(
            "wp2_fallback/target_pose", 10,
            &GuidanceControlNode::wp2FallbackTargetCallback, this);
    }

    void initPublishers() {
        // RViz 帧名:去掉命名空间前导 '/' 后拼 "/map"(如 /uav0 → uav0/map);无命名空间 → map
        {
            std::string vns = nh_.getNamespace();
            if (!vns.empty() && vns[0] == '/') vns = vns.substr(1);
            viz_frame_ = vns.empty() ? "map" : (vns + "/map");
        }

        vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            vel_cmd_topic_, 10);
        velocity_active_pub_ = nh_.advertise<std_msgs::Bool>(
            "guidance/velocity_active", 10);

        attitude_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            attitude_cmd_topic_, 10);

        attitude_rates_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            attitude_rates_topic_, 10);

        thrust_cmd_pub_ = nh_.advertise<mavros_msgs::Thrust>(
            thrust_cmd_topic_, 10);

        // flight_mode_pub_ 在仿真模式使用 quad/flight_mode，PX4 模式不使用（通过 mavros/set_mode 切换）
        if (use_sim_) {
            flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
                "quad/flight_mode", 10);
        }

        strike_eval_pub_ = nh_.advertise<std_msgs::Bool>(
            "strike_evaluation", 10);  // 发布打击评估结果

        thrust_dir_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "thrust_direction", 10);  // 发布推力方向可视化

        intercept_point_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "intercept_point", 10);  // 拦截点可视化

        target_filter_debug_pub_ = nh_.advertise<multi_uav_strike::TargetFilterDebug>(
            "guidance/target_filter_debug", 10);  // 2026-07-16: 速度滤波前后对比

        pn_debug_pub_ = nh_.advertise<multi_uav_strike::GuidancePnDebug>(
            "guidance/pn_debug", 10);
    }

    void initGuidanceStrategy() {
        guidance_strategy_ = multi_uav_strike::createGuidanceStrategy(
            current_strategy_type_, nh_private_);
    }

    // Callback methods
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        const double nx = msg->pose.position.x;
        const double ny = msg->pose.position.y;
        const double nz = msg->pose.position.z;

        //=======高度下降===========//
        raw_target_pose_ = *msg;
        raw_target_pose_valid_ = true;
        //=======高度下降===========//

        // === 2026-08-04: STRIKE 反馈环解耦 (Layer 5) — PN 输入 OneEuro LPF ===
        //   target_estimator EMA α=0.1 后残余噪声 ~0.05m,10Hz 发布在 50Hz PN 调用
        //   帧边界产生 0.04 rad/s los_rate 阶跃 → v_lateral 单帧 ~6m/s 跳变(被 nav_ratio 限幅)。
        //   对 nx/ny/nz 加 1Hz OneEuro,稳态 std 砍 ~70%,首跳 ≤0.1s 滞后可接受。
        const double fx = pn_target_filter_x_.filter(nx, msg->header.stamp.toSec());
        const double fy = pn_target_filter_y_.filter(ny, msg->header.stamp.toSec());
        const double fz = pn_target_filter_z_.filter(nz, msg->header.stamp.toSec());

        // Jump detection below uses raw values (nx/ny/nz) — keeps it independent of LPF lag.

        // === 2026-07-31: 目标位置突变(>5m)→ 同步 reset 滤波器 ===
        //   在 targetPoseCallback 内做检测的好处: resetLosFilter() 与
        //   上一次 PN computeCommand 之间没有其它 computeCommand → 首帧
        //   computeCommand 拿到的就是 fresh 状态, 不会把旧目标的 LOS
        //   角速度套到新目标上.
        //   One Euro 通道也在这里 reset, 与 LOS 滤波器一起丢旧状态,
        //   避免旧目标的速度平滑状态在新目标第一帧"残影"一拍再跟上.
        if (last_target_pos_valid_) {
            const double dx = nx - last_target_pos_x_;
            const double dy = ny - last_target_pos_y_;
            const double dz = nz - last_target_pos_z_;
            const double jump = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (jump > kTargetSwitchJumpM) {
                vel_filter_x_.reset();
                vel_filter_y_.reset();
                vel_filter_z_.reset();
                vel_filter_yaw_.reset();
                //===================制导修改====================
                pn_target_filter_x_.reset();
                pn_target_filter_y_.reset();
                pn_target_filter_z_.reset();
                pn_filtered_target_valid_ = false;
                pn_cluster_stale_latch_ = false;
                //===================制导修改====================
                if (current_strategy_type_ ==
                    multi_uav_strike::GuidanceStrategyType::INTERCEPT &&
                    guidance_strategy_) {
                    auto* intercept_strategy =
                        static_cast<multi_uav_strike::InterceptGuidance*>(
                            guidance_strategy_.get());
                    intercept_strategy->resetLosFilter();
                }
                ROS_WARN_THROTTLE(1.0,
                    "[Guidance] Target switched (jump=%.1fm), "
                    "reset velocity OneEuro + PN LOS filters",
                    jump);
            }
        }
        last_target_pos_x_ = nx;
        last_target_pos_y_ = ny;
        last_target_pos_z_ = nz;
        last_target_pos_valid_ = true;

        current_target_pose_ = *msg;
        current_target_pose_.pose.position.x = fx;
        current_target_pose_.pose.position.y = fy;
        current_target_pose_.pose.position.z = fz;
        is_target_pose_received_ = true;
        //===================制导修改====================
        last_target_pose_receive_time_ = ros::Time::now();
        if (is_enabled_ && current_mode_ == "strike") {
            strike_target_latched_valid_ = true;
        }

        // main分支PN Layer 5：10Hz目标阶跃先经独立1Hz OneEuro，再交给50Hz PN。
        // IMAGE/GEO继续使用上面的current_target_pose_原值，二者互不污染。
        pn_filtered_target_pose_ = *msg;
        const double filter_time = msg->header.stamp.isZero()
            ? ros::Time::now().toSec() : msg->header.stamp.toSec();
        pn_filtered_target_pose_.pose.position.x = pn_target_filter_x_.filter(nx, filter_time);
        pn_filtered_target_pose_.pose.position.y = pn_target_filter_y_.filter(ny, filter_time);
        pn_filtered_target_pose_.pose.position.z = pn_target_filter_z_.filter(nz, filter_time);
        pn_filtered_target_valid_ = true;
        pn_last_target_update_time_ = ros::Time::now();
        //===================制导修改====================
    }

    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_target_twist_ = *msg;
        is_target_twist_received_ = true;
    }

    //===================制导修改====================
    void primaryImageTargetCallback(
        const multi_uav_strike::PrimaryImageTarget::ConstPtr& msg) {
        current_image_target_ = *msg;
        image_target_received_ = true;

        // statesTimerCb 会重复发布同一帧，只有 frame_seq/stamp 真正变化才累计确认帧。
        const bool is_new_frame = msg->frame_seq != last_image_frame_seq_ ||
                                  msg->stamp_us != last_image_stamp_us_;
        if (is_new_frame) {
            last_image_frame_seq_ = msg->frame_seq;
            last_image_stamp_us_ = msg->stamp_us;
            if (msg->visible) {
                image_confirm_count_ = std::min(image_confirm_count_ + 1,
                                                image_confirm_frames_);
                Eigen::Vector3d ray(msg->world_ray_nwu.x,
                                    msg->world_ray_nwu.y,
                                    msg->world_ray_nwu.z);
                if (ray.norm() > 1e-6) {
                    ray.normalize();
                    if (!image_ray_filter_valid_) {
                        filtered_image_ray_nwu_ = ray;
                        image_ray_filter_valid_ = true;
                    } else {
                        filtered_image_ray_nwu_ =
                            image_ray_alpha_ * ray +
                            (1.0 - image_ray_alpha_) * filtered_image_ray_nwu_;
                        filtered_image_ray_nwu_.normalize();
                    }
                }
            }
        }
    }
    //===================制导修改====================

    // === 2026-08-20: WP2 fallback target callback (NWU PoseStamped from mission_manager) ===
    //   Attack(105) WP1→WP2 always-on guidance 路径:cluster 未 lock 时,
    //   mission_manager 用 WP2 自身作为 PN 目标 publish,这里同步喂给:
    //     1. PN 路径:pn_filtered_target_pose_ (经 OneEuro LPF) + pn_last_target_update_time_ = now
    //     2. image_geo 路径:current_target_pose_ + last_target_pose_receive_time_
    //   FIFO subscriber ordering 保证此 callback 在 enableCallback 之后触发。
    //   一旦 cluster 被 target_estimator 接管 (target_estimated_pose 持续更新),
    //   guidanceTimerCallback 会通过 stale timeout 把 wp2_fallback_valid_ 清掉。
    void wp2FallbackTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!std::isfinite(msg->pose.position.x) ||
            !std::isfinite(msg->pose.position.y) ||
            !std::isfinite(msg->pose.position.z)) {
            ROS_WARN_THROTTLE(2.0, "[Guidance] WP2 fallback: non-finite pose, ignore");
            return;
        }

        wp2_fallback_pose_         = *msg;
        wp2_fallback_valid_        = true;
        wp2_fallback_last_seen_    = ros::Time::now();

        const double t = msg->header.stamp.isZero()
            ? ros::Time::now().toSec()
            : msg->header.stamp.toSec();

        // === PN 路径:经 OneEuro LPF (避免 cold filter 首帧 0, A1/A5) ===
        pn_filtered_target_pose_ = *msg;
        pn_filtered_target_pose_.pose.position.x =
            pn_target_filter_x_.filter(msg->pose.position.x, t);
        pn_filtered_target_pose_.pose.position.y =
            pn_target_filter_y_.filter(msg->pose.position.y, t);
        pn_filtered_target_pose_.pose.position.z =
            pn_target_filter_z_.filter(msg->pose.position.z, t);
        pn_filtered_target_valid_    = true;
        pn_last_target_update_time_  = ros::Time::now();  // FIFO 在 enableCallback 之后

        // === image_geo 路径:同步 current_target_pose_ 让 strike_target_latched_valid_ 也走通 ===
        current_target_pose_          = pn_filtered_target_pose_;
        is_target_pose_received_      = true;
        last_target_pose_receive_time_ = ros::Time::now();
        if (is_enabled_ && current_mode_ == "strike") {
            strike_target_latched_valid_ = true;
        }

        ROS_INFO_THROTTLE(2.0, "[Guidance] WP2 fallback target (NWU)=(%.2f, %.2f, %.2f) "
                              "fed into PN + image_geo paths",
                          pn_filtered_target_pose_.pose.position.x,
                          pn_filtered_target_pose_.pose.position.y,
                          pn_filtered_target_pose_.pose.position.z);
    }
    //===================制导修改====================

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // ===== NED → NWU 坐标转换 =====
            // NED: X=North, Y=East, Z=Down
            // NWU: X=North, Y=West, Z=Up
            // 位置：X不变, Y取反, Z取反
            current_uav_pose_.pose.position.x = msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;

            // 四元数：q_nwu = q_frame * q_ned, q_frame = (0,1,0,0) 表示绕X轴180°
            // 展开后: w' = -x, x' = w, y' = -z, z' = y
            {
                const auto& q = msg->pose.orientation;
                current_uav_pose_.pose.orientation.w = -q.x;
                current_uav_pose_.pose.orientation.x =  q.w;
                current_uav_pose_.pose.orientation.y = -q.z;
                current_uav_pose_.pose.orientation.z =  q.y;
            }
        } else {
            // ===== ENU → NWU 坐标转换 =====
            // Mavros 输入是 ENU: X=East, Y=North, Z=Up
            // NWU: X=North, Y=West, Z=Up
            // 帧旋转 R_frame: 绕Z轴 -90° (ENU X→NWU -Y, ENU Y→NWU X, ENU Z→NWU Z)
            // 位置：x = y_enu, y = -x_enu, z = z_enu
            current_uav_pose_.pose.position.x = msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z = msg->pose.position.z;

            // 四元数：q_nwu = q_frame * q_enu, q_frame = (√2/2, 0, 0, -√2/2)
            // 展开后: w'=(w+z)/√2, x'=(x+y)/√2, y'=(y-x)/√2, z'=(z-w)/√2
            {
                const auto& q = msg->pose.orientation;
                const double s = 0.70710678118654752440;  // sqrt(2)/2
                current_uav_pose_.pose.orientation.w = s * (q.w + q.z);
                current_uav_pose_.pose.orientation.x = s * (q.x + q.y);
                current_uav_pose_.pose.orientation.y = s * (q.y - q.x);
                current_uav_pose_.pose.orientation.z = s * (q.z - q.w);
            }
        }

        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;
        is_uav_pose_received_ = true;

        //===================制导修改====================
        // main分支handover思想：由内部NWU位置差分估计实际NED速度，PN接管时作为滤波初值。
        if (has_prev_uav_position_) {
            const double dt = (msg->header.stamp - prev_uav_position_time_).toSec();
            if (dt > 0.001 && dt < 0.5) {
                const double dn = current_uav_pose_.pose.position.x - prev_uav_position_nwu_.x;
                const double dw = current_uav_pose_.pose.position.y - prev_uav_position_nwu_.y;
                const double du = current_uav_pose_.pose.position.z - prev_uav_position_nwu_.z;
                last_known_uav_velocity_ned_.x = dn / dt;
                last_known_uav_velocity_ned_.y = -dw / dt;
                last_known_uav_velocity_ned_.z = -du / dt;
            }
        }
        prev_uav_position_nwu_ = current_uav_pose_.pose.position;
        prev_uav_position_time_ = msg->header.stamp;
        has_prev_uav_position_ = true;
        //===================制导修改====================
    }

    void losAngleCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_angle_ = *msg;
        is_los_received_ = true;
    }

    void realTargetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        real_target_pos_ = *msg;
        is_real_target_received_ = true;
    }

    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        //===================制导修改====================
        const bool rising_edge = (!is_enabled_ && msg->data);
        if (rising_edge) {
            strike_guidance_enable_time_ = ros::Time::now();
            image_confirm_count_ = 0;
            image_ray_filter_valid_ = false;
            strike_vision_state_ = StrikeVisionState::GEO_FALLBACK;
            strike_vision_switch_time_ = ros::Time();
            // === 2026-08-20: 重置 chase latch, 新一轮 strike 不继承上一次末尾状态 ===
            terminal_image_chase_active_ = false;
            terminal_image_chase_enter_time_ = ros::Time();
            const double target_age = last_target_pose_receive_time_.isZero()
                ? std::numeric_limits<double>::infinity()
                : (ros::Time::now() - last_target_pose_receive_time_).toSec();
            // mission_manager 按 mode→target→speed→enable 发布，允许刚在 enable 前
            // 到达的本次目标成为 GEO 锁存目标，同时拒绝上一任务遗留的旧坐标。
            strike_target_latched_valid_ = is_target_pose_received_ &&
                target_age >= 0.0 && target_age <= strike_target_pose_timeout_sec_;
            //========速度修改========
            // 原逻辑从零速度开始做 slew limit，会造成打击接管后速度先掉到零再爬升。
            // last_strike_cmd_valid_ = false;
            last_strike_cmd_ned_ = geometry_msgs::Twist();
            if (has_prev_uav_position_) {
                last_strike_cmd_ned_.linear.x = last_known_uav_velocity_ned_.x;
                last_strike_cmd_ned_.linear.y = last_known_uav_velocity_ned_.y;
                last_strike_cmd_ned_.linear.z = last_known_uav_velocity_ned_.z;
                last_strike_cmd_valid_ = true;
                strike_blend_from_ned_ = last_strike_cmd_ned_;
                ROS_WARN("[Guidance] IMAGE/GEO handover seed v_NED=(%.2f,%.2f,%.2f)",
                         last_strike_cmd_ned_.linear.x,
                         last_strike_cmd_ned_.linear.y,
                         last_strike_cmd_ned_.linear.z);
            } else {
                last_strike_cmd_valid_ = false;
            }
            //========速度修改========
            last_strike_cmd_time_ = ros::Time();
            //===================制导修改====================
            pn_target_filter_x_.reset();
            pn_target_filter_y_.reset();
            pn_target_filter_z_.reset();
            pn_filtered_target_valid_ = false;
            pn_last_target_update_time_ = ros::Time();
            pn_cluster_stale_latch_ = false;
            if (strike_guidance_logic_ == "pn") {
                if (current_strategy_type_ ==
                        multi_uav_strike::GuidanceStrategyType::INTERCEPT &&
                    guidance_strategy_) {
                    auto* pn = static_cast<multi_uav_strike::InterceptGuidance*>(
                        guidance_strategy_.get());
                    pn->resetLosFilter();
                }
                if (has_prev_uav_position_) {
                    vel_filter_x_.x_prev = last_known_uav_velocity_ned_.x;
                    vel_filter_y_.x_prev = last_known_uav_velocity_ned_.y;
                    vel_filter_z_.x_prev = last_known_uav_velocity_ned_.z;
                    vel_filter_yaw_.x_prev = 0.0;
                    vel_filter_x_.initialized = true;
                    vel_filter_y_.initialized = true;
                    vel_filter_z_.initialized = true;
                    vel_filter_yaw_.initialized = true;
                    vel_filter_x_.dx_prev = vel_filter_y_.dx_prev =
                        vel_filter_z_.dx_prev = vel_filter_yaw_.dx_prev = 0.0;
                    ROS_WARN("[Guidance] PN handover seed v_NED=(%.2f,%.2f,%.2f)",
                             last_known_uav_velocity_ned_.x,
                             last_known_uav_velocity_ned_.y,
                             last_known_uav_velocity_ned_.z);
                }
            }
            //===================制导修改====================
        }
        //===================制导修改====================
        is_enabled_ = msg->data;
        if (is_enabled_) {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Guidance ENABLED (mode: %s)", current_mode_.c_str());
            // === 2026-08-05: Handover 修复 — 用 UAV 当前速度初始化 OneEuro filter ===
            //   防止 waypoint→guidance 交接时 OneEuro 从 x_prev=0 起步,
            //   输出塌陷再爬升(~0.94s 抖动窗口)
            // 直接给 filter 内部状态赋值,跳过第一帧"从 0 收敛"的过渡
            if (has_prev_uav_pos_) {
                vel_filter_x_.x_prev      = last_known_uav_velocity_ned_.x;
                vel_filter_y_.x_prev      = last_known_uav_velocity_ned_.y;
                vel_filter_z_.x_prev      = last_known_uav_velocity_ned_.z;
                // yaw rate filter:waypoint 阶段 wz=-0.6 是个控制量,但 P-controller
                //   在 STRIKE 里只用 yaw_err 算新 wz,旧的 yaw rate 不重要,留 0 即可
                vel_filter_yaw_.x_prev   = 0.0;
                vel_filter_x_.initialized = true;
                vel_filter_y_.initialized = true;
                vel_filter_z_.initialized = true;
                vel_filter_yaw_.initialized = true;
                vel_filter_x_.dx_prev = vel_filter_y_.dx_prev = vel_filter_z_.dx_prev = vel_filter_yaw_.dx_prev = 0.0;
                ROS_WARN_THROTTLE(2.0,
                    "[Guidance] Handover seed: v_NED=(%.2f,%.2f,%.2f) m/s",
                    last_known_uav_velocity_ned_.x,
                    last_known_uav_velocity_ned_.y,
                    last_known_uav_velocity_ned_.z);
            }
        } else {
            //===================制导修改====================
            // 每次退出制导都清掉图像确认、滤波和切换历史，防止下一次 Attack
            // 继承上一目标的视线或速度混合状态。
            image_confirm_count_ = 0;
            image_ray_filter_valid_ = false;
            strike_vision_state_ = StrikeVisionState::GEO_FALLBACK;
            strike_vision_switch_time_ = ros::Time();
            last_strike_cmd_valid_ = false;
            last_strike_cmd_time_ = ros::Time();
            strike_target_latched_valid_ = false;
            //===================制导修改====================
            ROS_WARN_THROTTLE(5.0, "[Guidance] Guidance DISABLED (SEARCH_ONLY mode)");
        }
        was_enabled_prev_ = is_enabled_;
    }

    void modeCallback(const std_msgs::String::ConstPtr& msg) {
        //===================制导修改====================
        const bool entering_strike = (current_mode_ != "strike" && msg->data == "strike");
        if (entering_strike) {
            image_confirm_count_ = 0;
            image_ray_filter_valid_ = false;
            strike_vision_state_ = StrikeVisionState::GEO_FALLBACK;
            strike_vision_switch_time_ = ros::Time();
            // === 2026-08-20: 新一轮 strike 必须重置 chase latch ===
            //   否则上一次 strike 末尾残留的 terminal_image_chase_active_ 会让这次
            //   进入 strike 时 image 还没收到就误触发 chase, 跳过必要的前置检查。
            terminal_image_chase_active_ = false;
            terminal_image_chase_enter_time_ = ros::Time();
            // === 2026-08-19: 进入 strike 重置闭环下降跟踪器 ===
            //   否则上一个 strike 的实际下降率 EMA 会污染新一次 strike 的闭环水平计算。
            resetActualDescentRate();
            //========速度修改========
            // mode 消息可能晚于 enable 到达，不能把 enableCallback 已建立的交接速度清零。
            // last_strike_cmd_valid_ = false;
            if (!last_strike_cmd_valid_ && has_prev_uav_position_) {
                last_strike_cmd_ned_ = geometry_msgs::Twist();
                last_strike_cmd_ned_.linear.x = last_known_uav_velocity_ned_.x;
                last_strike_cmd_ned_.linear.y = last_known_uav_velocity_ned_.y;
                last_strike_cmd_ned_.linear.z = last_known_uav_velocity_ned_.z;
                last_strike_cmd_valid_ = true;
                strike_blend_from_ned_ = last_strike_cmd_ned_;
            }
            //========速度修改========
            last_strike_cmd_time_ = ros::Time();
            strike_guidance_enable_time_ = ros::Time::now();
        }
        //===================制导修改====================
        current_mode_ = msg->data;
        ROS_WARN_THROTTLE(5.0, "[Guidance] Mode changed to: %s", current_mode_.c_str());
    }

    void uavSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
        const double speed = static_cast<double>(msg->data);
        if (speed <= 0.0) {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Ignoring non-positive uav_speed: %.2f", speed);
            return;
        }
        const double limited_speed = std::min(speed, attack_max_speed_);
        if (speed > attack_max_speed_) {
            ROS_WARN("[Guidance] requested attack speed %.2f exceeds %.2fm/s; clamped",
                     speed, attack_max_speed_);
        }
        // 更新本节点的速度上限(用于 applyInterUavAvoidance 和 vel cmd 限速)
        track_max_speed_ = limited_speed;
        // 下发到当前激活的策略(目前只有 Intercept 用到)
        if (guidance_strategy_) {
            guidance_strategy_->setUavSpeed(limited_speed);
        }
        ROS_INFO("[Guidance] uav_speed updated to %.2f m/s", limited_speed);
    }

    // 邻居心跳回调（机间避障用）— 替代原 otherUavPosesCallback。
    //   心跳自带 sn + lat/lon/alt,不需要反查,直接缓存 + 自环过滤 + 过期清理。
    //   applyInterUavAvoidance 内部按需把 lat/lon/alt 转 NED。
    void heartbeatCallback(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        // 自环过滤:本机 comm_node 也会发到 /inter_uav/uav_status,不剔除会把
        //   自己算成 0m 最近邻居,applyInterUavAvoidance 拒动。
        if (!uav_device_sn_.empty() && msg->sn == uav_device_sn_) {
            return;
        }
        // 周期清理过期心跳(避免断连 UAV 残留)
        ros::Time now = ros::Time::now();
        if ((now - last_heartbeat_purge_).toSec() > 1.0) {
            heartbeats_.erase(
                std::remove_if(heartbeats_.begin(), heartbeats_.end(),
                               [&](const multi_uav_strike::InterUavStatus::ConstPtr& h) {
                                   double age_sec = now.toSec() -
                                                    static_cast<double>(h->timestamp_us) * 1e-6;
                                   return age_sec > 5.0;
                               }),
                heartbeats_.end());
            last_heartbeat_purge_ = now;
        }
        heartbeats_.push_back(msg);
    }

    /**
     * PX4 home 回调 — 锁一次
     * PX4 的 LOCAL_POSITION_NED 参考系由 EKF2 在启动时锁定,
     * 后续 home 更新(MAV_CMD_DO_SET_HOME / disarm)不会重置 EKF2 origin,
     * 也不让 local_position 跳变。如果跟着 home 更新去重算 NED,
     * 反而让 setpoint 与 UAV 当前 local_position 不在同一 frame → 偏飞。
     */
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        mavros_state_received_ = true;
        mavros_connected_ = msg->connected;
        mavros_armed_ = msg->armed;
    }

    void missionStateCallback(const multi_uav_strike::MissionState::ConstPtr& msg) {
        mission_state_received_ = true;
        current_mission_phase_ = msg->phase;
    }

    //========不要fail safe============//
    // 只有真正写入 MAVROS velocity topic 后才确认接管。MissionManager 收到该确认
    // 才让 waypoint/position 发布者退出，保证交接过程中至少有一个 setpoint 源存活。
    void publishGuidanceVelocity(const geometry_msgs::Twist& cmd) {
        vel_cmd_pub_.publish(cmd);
        std_msgs::Bool active;
        active.data = true;
        velocity_active_pub_.publish(active);
    }
    //========不要fail safe============//

    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (ref_initialized_) return;  // 锁一次
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            return;  // PX4 home 未稳定前发 0/0,忽略
        }
        ROS_WARN("[Guidance] >>>> PX4 home locked: (%.7f, %.7f, %.2f)",
                 msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        ref_lat_ = msg->geo.latitude;
        ref_lon_ = msg->geo.longitude;
        ref_alt_ = msg->geo.altitude;
        ref_initialized_ = true;
    }

    /**
     * GPS (WGS84) -> NED 坐标转换
     */
    void gpsToNed(double lat, double lon, double alt,
                  double& ned_x, double& ned_y, double& ned_z) {
        const double EARTH_R = 6378137.0;
        double d_lat = lat - ref_lat_;
        ned_x = d_lat * M_PI / 180.0 * EARTH_R;  // 北向
        double d_lon = lon - ref_lon_;
        ned_y = d_lon * M_PI / 180.0 * EARTH_R * cos(ref_lat_ * M_PI / 180.0);  // 东向
        ned_z = -(alt - ref_alt_);  // 下向
    }

    // 机间避障（人工势场）— 数据源:心跳 cache。
    //   直接迭代 heartbeats_ 把每条心跳的 lat/lon/alt 转 NED 算距离。
    //   ref_initialized_=false 时(PX4 home 未到)gpsToNed 出来的 NED 无意义,直接跳过。
    void applyInterUavAvoidance(double& vx, double& vy, double& vz) {
        if (heartbeats_.empty() || !ref_initialized_) {
            return;
        }

        double repulsion_gain = 10.0;
        double min_safe_distance = 25.0;

        double fx = 0.0, fy = 0.0, fz = 0.0;

        for (const auto& h : heartbeats_) {
            double ned_x, ned_y, ned_z;
            gpsToNed(h->lat, h->lon, h->alt, ned_x, ned_y, ned_z);
            double dx = current_uav_pose_.pose.position.x - ned_x;
            double dy = current_uav_pose_.pose.position.y - ned_y;
            double dz = current_uav_pose_.pose.position.z - ned_z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < min_safe_distance) {
                if (dist < 2)
                    dist = 2;  // 避免除以零
                double force_magnitude = repulsion_gain / (dist * dist);
                fx += (dx / dist) * force_magnitude;
                fy += (dy / dist) * force_magnitude;
                fz += (dz / dist) * force_magnitude;
            }
        }

        vx += fx;
        vy += fy;

        // 限速
        double speed = sqrt(vx*vx + vy*vy + vz*vz);
        if (speed > track_max_speed_ * 1.5) {
            double scale = (track_max_speed_ * 1.5) / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }
    }

    // 计算打击评估
    void evaluateStrike() {
        if (!enable_strike_evaluation_) {
            return;
        }
        if (first_strike_evaluated_ || !is_real_target_received_) {
            return;
        }

        // 计算无人机到真实目标的距离
        double dx = current_uav_pose_.pose.position.x - real_target_pos_.x;
        double dy = current_uav_pose_.pose.position.y - real_target_pos_.y;
        double dz = current_uav_pose_.pose.position.z - real_target_pos_.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        // 检测是否到达最近点（从接近转为远离）
        if (is_approaching_) {
            if (distance < prev_distance_) {
                // 仍在接近
                prev_distance_ = distance;
            } else {
                // 开始远离，到达最近点
                is_approaching_ = false;
                min_strike_distance_ = prev_distance_;
                strike_altitude_ = current_uav_pose_.pose.position.z;
                strike_time_ = ros::Time::now().toSec();

                // 发布评估结果
                std_msgs::Bool eval_msg;
                eval_msg.data = (min_strike_distance_ < strike_distance_threshold_);
                strike_eval_pub_.publish(eval_msg);

                ROS_WARN("[STRIKE EVAL] %s: min_dist=%.2f m (threshold=%.2f), alt=%.2f m, t=%.2f s",
                         (min_strike_distance_ < strike_distance_threshold_) ? "SUCCESS" : "FAILURE",
                         min_strike_distance_, strike_distance_threshold_,
                         strike_altitude_, strike_time_);

                first_strike_evaluated_ = true;
            }
        }
    }

    //===================制导修改====================
    static double clampValue(double value, double low, double high) {
        return std::max(low, std::min(high, value));
    }

    double currentYawNed() const {
        const auto& q = current_uav_pose_.pose.orientation;
        const double yaw_nwu = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return -yaw_nwu;
    }

    struct StrikeTargetGeometry {
        double north = 0.0;
        double east = 0.0;
        double down = 0.0;
        double horizontal = 0.0;
        double distance = 0.0;
    };

    StrikeTargetGeometry strikeTargetGeometry(
        const geometry_msgs::PoseStamped& target_pose) const {
        StrikeTargetGeometry g;
        // current pose/target pose 均为节点内部 NWU；这里统一转换到 NED。
        g.north = target_pose.pose.position.x - current_uav_pose_.pose.position.x;
        g.east = -(target_pose.pose.position.y - current_uav_pose_.pose.position.y);
        g.down = -(target_pose.pose.position.z - current_uav_pose_.pose.position.z);
        g.horizontal = std::hypot(g.north, g.east);
        g.distance = std::sqrt(g.horizontal * g.horizontal + g.down * g.down);
        return g;
    }

    StrikeTargetGeometry strikeTargetGeometry() const {
        return strikeTargetGeometry(current_target_pose_);
    }

    //=======高度下降===========//
    StrikeTargetGeometry strikeDescentGeometry() const {
        return raw_target_pose_valid_
            ? strikeTargetGeometry(raw_target_pose_)
            : strikeTargetGeometry(current_target_pose_);
    }
    //=======高度下降===========//

    bool targetPoseFreshForThisStrike() const {
        if (!is_target_pose_received_ || last_target_pose_receive_time_.isZero()) return false;
        //========速度修改========
        // 允许接管前已经收到、但仍在 freshness 窗口内的当前锁定目标立即用于导引。
        // 原逻辑强制等待 enable 后的新一帧目标，低频目标话题会造成一段零速。
        // stale 目标仍由 age 门槛拒绝，不能无限沿用上一任务缓存。
        // if (!strike_guidance_enable_time_.isZero() &&
        //     last_target_pose_receive_time_ < strike_guidance_enable_time_) return false;
        //========速度修改========
        return strike_target_latched_valid_;
    }

    double terminalHorizontalSpeed(double requested_speed, double horizontal_distance) const {
        if (horizontal_distance <= strike_horizontal_stop_radius_m_) return 0.0;
        if (horizontal_distance >= strike_slowdown_radius_m_) return requested_speed;
        const double scale =
            (horizontal_distance - strike_horizontal_stop_radius_m_) /
            (strike_slowdown_radius_m_ - strike_horizontal_stop_radius_m_);
        return requested_speed * clampValue(scale, 0.0, 1.0);
    }

    double terminalYawRate(double desired_yaw, double horizontal_distance) const {
        const double yaw_error = std::atan2(std::sin(desired_yaw - currentYawNed()),
                                            std::cos(desired_yaw - currentYawNed()));
        double yaw_rate = clampValue(image_yaw_kp_ * yaw_error,
                                     -image_yaw_rate_max_, image_yaw_rate_max_);
        if (strike_yaw_freeze_radius_m_ > 1e-3) {
            yaw_rate *= clampValue(horizontal_distance / strike_yaw_freeze_radius_m_, 0.0, 1.0);
        }
        return yaw_rate;
    }

    //=======高度下降===========//
    // === 2026-08-19: STRIKE 闭环下降 — 水平跟随实际垂直能力 ===
    //   原实现直接返回 requested_speed,导致 commanded_vz 和 actual_vz 不一致时
    //   (PX4 MPC_Z_VEL_MAX_DN 限速、电机饱和、目标在下方但 UAV 还没低头 等) 水平
    //   先到、垂直慢 10s 才落地,体验"下降速率不够"。
    //   修法:用实际下降率 (actual_descent_rate_ema_) 反推水平速度,确保水平/垂直同步
    //   到达命中圈。actual_vz ≥ commanded_vz 时不影响,直接走 requested_speed。
    //   关键:不被 commanded vz 假设绑架,所以 PX4 内部限多少都不影响闭环正确性。
    double feasibleHorizontalSpeedForDescent(const StrikeTargetGeometry& g,
                                             double requested_speed) const {
        // 目标在上方 / 已进命中圈 / 实际下降率还没收敛:不走闭环,保持原 requested_speed
        if (g.down <= 0.0 || g.horizontal <= strike_distance_threshold_ ||
            actual_descent_rate_ema_ < 0.05) {
            return requested_speed;
        }
        // 水平剩余距离 (扣掉命中圈)
        const double horizontal_to_hit = std::max(
            0.0, g.horizontal - strike_distance_threshold_);
        // 实际需要多久才能落到目标高度
        const double time_to_land = g.down / actual_descent_rate_ema_;
        // 闭环水平速度 = 剩余水平 / (落地时间 + 安全留余)
        // 确保水平在垂直落地后 (lag 秒) 关闭,实现水平/垂直同步到达
        const double closed_loop_h_speed = horizontal_to_hit /
            std::max(0.1, time_to_land + strike_vertical_lag_);
        return std::min(requested_speed, closed_loop_h_speed);
    }

    // === 2026-08-19: 实际下降率 EMA 跟踪器 ===
    //   每次 STRIKE tick 调一次,基于 current_uav_pose_.pose.position.z (NWU)
    //   与上一 tick 的差分算 actual_vz = -(dz/dt)。EMA 平滑避免单 tick 噪声。
    //   在 entering_strike 时 altitude_filter_initialized_ 清掉,Strike 头两秒
    //   还没收敛 (actual_descent_rate_ema_ < 0.05) 时 feasibleHorizontalSpeedForDescent
    //   走 fallback 用 requested_speed,不会因 EMA 接近 0 算成 horizontal≈0 卡死。
    void updateActualDescentRate(const ros::Time& now) {
        const double current_z = current_uav_pose_.pose.position.z;
        if (!altitude_filter_initialized_) {
            last_altitude_nwu_ = current_z;
            last_altitude_time_ = now;
            altitude_filter_initialized_ = true;
            return;
        }
        const double dt = (now - last_altitude_time_).toSec();
        if (dt < 0.005 || dt > 1.0) {
            // 异常 dt:不更新 EMA,只推进参考点
            last_altitude_nwu_ = current_z;
            last_altitude_time_ = now;
            return;
        }
        // NWU: z=Up,下降时 z 减小 → altitude_drop > 0
        const double altitude_drop = last_altitude_nwu_ - current_z;
        const double actual_vz = std::max(0.0, altitude_drop / dt);
        if (actual_descent_rate_ema_ < 0.01) {
            actual_descent_rate_ema_ = actual_vz;
        } else {
            // 0.6 / 0.4 EMA:对新 vz 略敏感,飞行控制器限速时会快速反映
            actual_descent_rate_ema_ = 0.6 * actual_descent_rate_ema_ +
                                       0.4 * actual_vz;
        }
        last_altitude_nwu_ = current_z;
        last_altitude_time_ = now;
    }

    void resetActualDescentRate() {
        altitude_filter_initialized_ = false;
        actual_descent_rate_ema_ = 0.0;
    }
    //=======高度下降===========//

    //===================制导修改====================
    double computeSlantDescentSpeed(const StrikeTargetGeometry& g,
                                    double horizontal_speed) const {
        // 旧逻辑是只按高度差限幅，远距离时也固定下降1m/s，水平先到后只能垂直打击：
        // return clampValue(g.down, -image_max_vertical_speed_,
        //                            image_max_vertical_speed_);
        if (!strike_slant_descent_enabled_) {
            return clampValue(g.down, -strike_descent_rate_,
                                      strike_descent_rate_);
        }

        // 目标在上方时仍保留原来的爬升能力；下面的保护只约束向下打击。
        if (g.down <= 0.0) {
            return clampValue(g.down, -strike_descent_rate_, 0.0);
        }

        // 尚未进入二维命中圈：按“到达2m命中圈边界”的剩余时间安排下降，
        // 并额外保留 strike_vertical_lag_ 秒。末端水平减速时，下降也会同步减小，
        // 不再因为固定3m提交区而提前耗尽最后高度。
        if (g.horizontal > strike_distance_threshold_) {
            const double closing_speed = std::max(horizontal_speed, 0.1);
            const double horizontal_to_hit =
                g.horizontal - strike_distance_threshold_;
            const double time_to_hit = horizontal_to_hit / closing_speed;
            const double desired_vz = g.down /
                (time_to_hit + strike_vertical_lag_);
            return clampValue(desired_vz, 0.0, strike_descent_rate_);
        }

        // 水平已进入2m命中圈，才允许完成剩余下降；MissionManager 仍按原三维2m
        // 条件结束 guidance，不改变既有命中定义。
        const double terminal_limit = 0.6 * strike_descent_rate_;
        return clampValue(g.down, 0.0, terminal_limit);
    }

    //=======高度下降===========//
    // IMAGE/GEO 源切换混合和指令斜率限制可能保留上一阶段的下降速度，因此仅在
    // computeSlantDescentSpeed() 内返回 0 还不足以守住提交高度。此保护必须放在
    // 所有混合/斜率限制之后：提交区外按剩余高度逐渐刹停，到保护高度后立即禁止
    // 继续向下。它只覆盖下降分量，不阻止为修正扰动而爬升。
    void enforcePreCommitAltitudeGuard(geometry_msgs::Twist& cmd,
                                       const StrikeTargetGeometry& g,
                                       double closing_speed) const {
        if (!strike_slant_descent_enabled_ ||
            g.horizontal <= strike_distance_threshold_ || cmd.linear.z <= 0.0) {
            return;
        }

        // === 2026-08-19: closing_speed 必须是"非钳位的预期水平速度",不能用 cmd.linear.x/y ===
        //   uav0_strike_2026-08-19-12-58-47_0.bag: PX4 MPC_Z_VEL_MAX_DN ≈ 1.5 → 我的
        //   feasibleHorizontalSpeedForDescent 把 horizontal 钳到 0.16,guard 用 cmd.linear.x/y
        //   当 closing_speed → time_to_hit = 169s → max_guarded_descent = 0.10 → vz 锁 0.10
        //   → strike 永远到不了。caller 必须传 scheduled_speed(终端减速调度)或
        //   track_max_speed_(PN 路径)这类"想要的速度",不是闭环后的实测值。
        closing_speed = std::max(closing_speed, 0.1);
        const double time_to_hit =
            (g.horizontal - strike_distance_threshold_) / closing_speed;
        const double max_guarded_descent = std::min(
            strike_descent_rate_,
            std::max(0.0, g.down) / (time_to_hit + strike_vertical_lag_));
        if (cmd.linear.z > max_guarded_descent) {
            ROS_WARN_THROTTLE(1.0,
                "[Guidance] Hit-circle descent brake: horizontal=%.2fm > hit=%.2fm, "
                "height=%.2fm t_hit=%.2fs, descent %.2f->%.2fm/s",
                g.horizontal, strike_distance_threshold_, g.down, time_to_hit,
                cmd.linear.z,
                max_guarded_descent);
            cmd.linear.z = max_guarded_descent;
        }
    }
    //=======高度下降===========//
    //===================制导修改====================

    void computeGeoFallbackVelocity(geometry_msgs::Twist& cmd) {
        //=======高度下降===========//
        // 水平末端减速、方向和下降必须使用同一实时目标几何，避免滤波滞后让
        // 水平提前停住、垂直却继续下降。
        const StrikeTargetGeometry g = strikeDescentGeometry();
        //=======高度下降===========//
        //========速度修改========
        // track_max_speed_ 由 guidance/guidance_speed 更新，来源是 Attack task_speed。
        // 默认不再受 image_max_speed、邻近目标减速和 1m 水平零速限制。
        const double requested_speed = track_max_speed_;
        const double scheduled_speed = strike_terminal_speed_schedule_enabled_
            ? terminalHorizontalSpeed(requested_speed, g.horizontal)
            : requested_speed;
        //=======高度下降===========//
        const double speed = feasibleHorizontalSpeedForDescent(g, scheduled_speed);
        //=======高度下降===========//
        //========速度修改========
        if (g.horizontal > 0.1 && speed > 0.0) {
            cmd.linear.x = speed * g.north / g.horizontal;
            cmd.linear.y = speed * g.east / g.horizontal;
        }
        //===================制导修改====================
        // 原垂直控制保留：
        // cmd.linear.z = clampValue(g.down, -image_max_vertical_speed_,
        //                                    image_max_vertical_speed_);
        // === 2026-08-19: slant 公式必须用 scheduled_speed,不用被钳位的 `speed` ===
        //   computeSlantDescentSpeed 把 horizontal_speed 当 closing_speed 反推 vz:
        //     closing_speed 小 → time_to_hit 大 → desired_vz 小 → PX4 跟小 →
        //   actual_vz 小 → 我的 feasibleHorizontalSpeedForDescent 用 actual_vz
        //   进一步把 horizontal 钳小 → 闭环稳定在 ~0.1 m/s,strike 永远到不了
        //   (uav0_strike_2026-08-19-12-40-57_0.bag)。scheduled_speed 是基于
        //   终端距离调度的,不被 h_speed 钳位影响,slant 算出来的 vz 不再自反馈死锁。
        cmd.linear.z = computeSlantDescentSpeed(g, scheduled_speed);
        //===================制导修改====================
        if (g.horizontal > 0.1) {
            cmd.angular.z = terminalYawRate(std::atan2(g.east, g.north), g.horizontal);
        }
    }

    // === 2026-08-20: 图像居中终端冲刺 helper ===
//   共享于 computeImageVelocity (image_geo_fallback) 和 pn 分支 (pn) 两条 strike 路径.
//   输入: 目标几何 g (用于 in_terminal_zone 判定)
//   输出: chase_active 时把 cmd 设成 ray*track_max_speed_ 3D 全速, 并设 yaw rate → 返回 true
//         否则 cmd 保持原值,terminal_image_chase_active_ 清零 → 返回 false
//   副作用: 维护 terminal_image_chase_active_ + terminal_image_chase_enter_time_
//           (sticky latch), caller 用 terminal_image_chase_active_ 决定是否绕过
//           enforcePreCommitAltitudeGuard
bool maybeApplyTerminalImageChase(geometry_msgs::Twist& cmd,
                                  const StrikeTargetGeometry& g) {
        Eigen::Vector3d ray = filtered_image_ray_nwu_;
        if (ray.norm() < 1e-6) {
            terminal_image_chase_active_ = false;
            return false;
        }
        ray.normalize();

        const uint64_t now_us = ros::Time::now().toNSec() / 1000ULL;
        const bool stamp_valid = image_target_received_ &&
            current_image_target_.stamp_us > 0 &&
            now_us >= current_image_target_.stamp_us;
        const double image_age = stamp_valid
            ? (now_us - current_image_target_.stamp_us) * 1e-6
            : std::numeric_limits<double>::infinity();
        const bool image_fresh = stamp_valid &&
            current_image_target_.visible &&
            image_age <= image_visible_timeout_sec_ &&
            image_ray_filter_valid_;
        const bool image_centered =
            std::fabs(current_image_target_.center_x - 0.5) <= strike_terminal_image_chase_center_band_ &&
            std::fabs(current_image_target_.center_y - 0.5) <= strike_terminal_image_chase_center_band_;
        const bool in_terminal_zone =
            g.horizontal > 0.1 &&
            g.horizontal <= strike_terminal_image_chase_min_horiz_m_;
        const bool should_chase = strike_terminal_image_chase_enabled_ &&
                                  image_fresh && image_centered && in_terminal_zone;

        if (should_chase) {
            terminal_image_chase_enter_time_ = ros::Time::now();
        }
        const bool in_latch_period =
            terminal_image_chase_active_ &&
            !terminal_image_chase_enter_time_.isZero() &&
            (ros::Time::now() - terminal_image_chase_enter_time_).toSec() <
                strike_terminal_image_chase_latch_sec_;
        const bool chase_active = should_chase ||
            (terminal_image_chase_active_ && in_latch_period && image_fresh);
        terminal_image_chase_active_ = chase_active;

        // === 状态切换日志 (2026-08-20):便于下次跑 bag 直接确认 chase 是否起作用 ===
        if (chase_active && !last_chase_log_active_) {
            ROS_WARN("[Guidance] CHASE ENTERED: image_centered=%.3f,%.3f horiz=%.2fm "
                     "(min=%.1fm, latch=%.1fs, image_age=%.3fs)",
                     current_image_target_.center_x, current_image_target_.center_y,
                     g.horizontal, strike_terminal_image_chase_min_horiz_m_,
                     strike_terminal_image_chase_latch_sec_, image_age);
        } else if (!chase_active && last_chase_log_active_) {
            ROS_WARN("[Guidance] CHASE EXITED: image_fresh=%d image_centered=%d in_zone=%d "
                     "(horiz=%.2fm image_age=%.3fs)",
                     image_fresh, image_centered, in_terminal_zone,
                     g.horizontal, image_age);
        }
        last_chase_log_active_ = chase_active;

        if (!chase_active) return false;

        // 沿相机射线 3D 全速冲向. NWU ray → NED velocity:
        //   v_north =  speed * ray.x
        //   v_east  = -speed * ray.y    (NWU +y=西, NED +y=东)
        //   v_down  = -speed * ray.z    (NWU +z=上, NED +z=下)
        // ray 已 normalize, |v|=speed. PX4 SITL 物理限速仍由 MPC_Z_VEL_* 兜底,
        // 但不主动钳位 (用户要求"不要减速冲过去")。
        const double chase_speed = track_max_speed_;
        cmd.linear.x =  chase_speed * ray.x();
        cmd.linear.y = -chase_speed * ray.y();
        cmd.linear.z = -chase_speed * ray.z();
        cmd.angular.z = terminalYawRate(
            std::atan2(ray.y(), ray.x()), g.horizontal);
        return true;
    }

    void computeImageVelocity(geometry_msgs::Twist& cmd) {
        //=======高度下降===========//
        // 图像负责中远距视线方向；末端距离调度和世界坐标方向与命中判定统一。
        const StrikeTargetGeometry g = strikeDescentGeometry();
        //=======高度下降===========//
        Eigen::Vector3d ray = filtered_image_ray_nwu_;
        if (ray.norm() < 1e-6) {
            computeGeoFallbackVelocity(cmd);
            terminal_image_chase_active_ = false;
            return;
        }
        ray.normalize();

        // === 2026-08-20: 图像居中终端冲刺 (image-terminal chase) ===
        //   helper 同时被 image_geo_fallback 和 pn 两条 strike 分支共享,
        //   chase 中 vx/vy/vz 完全沿 filtered_image_ray_nwu_,speed = track_max_speed_,
        //   绕过 terminalHorizontalSpeed / feasibleHorizontalSpeedForDescent /
        //   computeSlantDescentSpeed / enforcePreCommitAltitudeGuard 全部前馈钳位.
        //   "图像丢了" 判定 = !image_ready (与现有 IMAGE_GUIDANCE→GEO_FALLBACK 同源),
        //   但 chase 自身 sticky_latch_sec 秒: 进入 chase 后最少保持 latch 时长,
        //   容忍 1-2 帧 bbox 抖动 (visible=false) 不在 chase/GEO 间反复切。
        if (maybeApplyTerminalImageChase(cmd, g)) {
            return;  // chase 分支已完整设置 cmd
        }

        //========速度修改========
        // 原逻辑保留：bbox 中心只用于速度调度，偏离中心时 alignment 会把速度
        // 降至 Attack speed 的 25%。图像射线方向计算仍在 primaryImageTargetCallback 中保留。
        // double ex = current_image_target_.center_x - 0.5;
        // double ey = current_image_target_.center_y - 0.5;
        // if (std::fabs(ex) < image_center_deadband_) ex = 0.0;
        // if (std::fabs(ey) < image_center_deadband_) ey = 0.0;
        // const double alignment = clampValue(
        //     1.0 - std::hypot(ex, ey) / 0.5, 0.25, 1.0);
        // const double requested_speed =
        //     std::min(track_max_speed_, image_max_speed_) * alignment;
        // 图像偏差只决定方向，不再改变速度大小；水平速度直接接入 Attack task_speed。
        const double requested_speed = track_max_speed_;
        const double scheduled_speed = strike_terminal_speed_schedule_enabled_
            ? terminalHorizontalSpeed(requested_speed, g.horizontal)
            : requested_speed;
        //=======高度下降===========//
        const double speed = feasibleHorizontalSpeedForDescent(g, scheduled_speed);
        //=======高度下降===========//
        //========速度修改========

        // NWU ray → NED ray，一次转换；发布前仍统一走 convertVelNedToEnu。
        double north = ray.x();
        double east = -ray.y();
        //===================制导修改====================
        // 原图像射线垂直分量保留；新斜线下降统一使用目标三维几何，避免两套高度源跳变。
        // const double down = -ray.z();
        //===================制导修改====================
        // === 2026-08-19: slant 公式用 scheduled_speed,不用被钳的 speed,治自反馈死锁 ===
        //   (详见 computeGeoFallbackVelocity 中的同主题注释)
        cmd.linear.z = computeSlantDescentSpeed(g, scheduled_speed);
        //===================制导修改====================
        double horizontal = std::hypot(north, east);

        // bag中8m内图像射线趋近垂直并多次换边。末端仅保留图像可见性和速度调度，
        // 水平方向改用稳定的cluster世界坐标，避免越顶后瞬间反向满速修正。
        if (g.horizontal <= strike_terminal_world_direction_radius_m_ && g.horizontal > 0.1) {
            north = g.north;
            east = g.east;
            horizontal = g.horizontal;
        }
        if (horizontal > 0.1 && speed > 0.0) {
            cmd.linear.x = speed * north / horizontal;
            cmd.linear.y = speed * east / horizontal;
        }
        //===================制导修改====================
        // 原垂直控制保留：
        // const double descent_sign_source =
        //     g.distance > 1e-3 ? g.down : (down >= 0.0 ? 1.0 : -1.0);
        // cmd.linear.z = clampValue(descent_sign_source,
        //                           -image_max_vertical_speed_, image_max_vertical_speed_);
        // (cmd.linear.z 已在 L1495 用 scheduled_speed 赋值,这里不再重复,避免双赋值)
        //===================制导修改====================

        if (horizontal > 0.1) {
            cmd.angular.z = terminalYawRate(std::atan2(east, north), g.horizontal);
        }
    }

    void applyStrikeCommandSlewLimit(geometry_msgs::Twist& cmd) {
        const ros::Time now = ros::Time::now();
        double dt = last_strike_cmd_time_.isZero()
            ? (1.0 / 50.0) : (now - last_strike_cmd_time_).toSec();
        dt = clampValue(dt, 1e-3, 0.1);
        last_strike_cmd_time_ = now;

        const geometry_msgs::Twist previous = last_strike_cmd_valid_
            ? last_strike_cmd_ned_ : geometry_msgs::Twist();
        double dx = cmd.linear.x - previous.linear.x;
        double dy = cmd.linear.y - previous.linear.y;
        const double delta_horizontal = std::hypot(dx, dy);
        const double max_horizontal_delta = strike_max_horizontal_accel_ * dt;
        if (delta_horizontal > max_horizontal_delta && delta_horizontal > 1e-9) {
            const double scale = max_horizontal_delta / delta_horizontal;
            dx *= scale;
            dy *= scale;
        }
        cmd.linear.x = previous.linear.x + dx;
        cmd.linear.y = previous.linear.y + dy;
        cmd.linear.z = previous.linear.z + clampValue(
            cmd.linear.z - previous.linear.z,
            -strike_max_vertical_accel_ * dt,
             strike_max_vertical_accel_ * dt);
        cmd.angular.z = previous.angular.z + clampValue(
            cmd.angular.z - previous.angular.z,
            -strike_max_yaw_accel_ * dt,
             strike_max_yaw_accel_ * dt);
    }

    void computeImageGeoStrikeVelocity(geometry_msgs::Twist& cmd) {
        // === 2026-08-19: 闭环下降 — 每 tick 更新实际下降率 EMA ===
        //   feasibleHorizontalSpeedForDescent 据此反推水平速度,确保水平/垂直同步到达
        updateActualDescentRate(ros::Time::now());

        const uint64_t now_us = ros::Time::now().toNSec() / 1000ULL;
        const bool stamp_valid = image_target_received_ &&
                                 current_image_target_.stamp_us > 0 &&
                                 now_us >= current_image_target_.stamp_us;
        const double image_age = stamp_valid
            ? (now_us - current_image_target_.stamp_us) * 1e-6
            : std::numeric_limits<double>::infinity();

        // 新一轮guidance启用后，必须等本轮新发布的target_estimated_pose。
        // 等待期间输出零速，由50Hz发布维持OFFBOARD，不再沿用上一任务缓存目标。
        if (!targetPoseFreshForThisStrike()) {
            ROS_WARN_THROTTLE(1.0,
                "[Guidance] Waiting fresh target pose for this strike; publishing zero setpoint");
            return;
        }
        const bool image_ready = stamp_valid && current_image_target_.visible &&
                                 image_age <= image_visible_timeout_sec_ &&
                                 image_confirm_count_ >= image_confirm_frames_ &&
                                 image_ray_filter_valid_;
        const StrikeVisionState desired_state = image_ready
            ? StrikeVisionState::IMAGE_GUIDANCE
            : StrikeVisionState::GEO_FALLBACK;

        if (desired_state != strike_vision_state_) {
            strike_blend_from_ned_ = last_strike_cmd_valid_
                ? last_strike_cmd_ned_ : geometry_msgs::Twist();
            strike_vision_state_ = desired_state;
            strike_vision_switch_time_ = ros::Time::now();
            if (desired_state == StrikeVisionState::GEO_FALLBACK) {
                image_confirm_count_ = 0;
                image_ray_filter_valid_ = false;
            }
            ROS_WARN("[Guidance] strike source -> %s (image_age=%.3fs confirm=%d/%d)",
                     desired_state == StrikeVisionState::IMAGE_GUIDANCE
                         ? "IMAGE_GUIDANCE" : "GEO_FALLBACK",
                     image_age, image_confirm_count_, image_confirm_frames_);
        }

        geometry_msgs::Twist desired;
        if (strike_vision_state_ == StrikeVisionState::IMAGE_GUIDANCE) {
            computeImageVelocity(desired);
        } else {
            computeGeoFallbackVelocity(desired);
        }

        const double elapsed = strike_vision_switch_time_.isZero()
            ? image_reacquire_blend_sec_
            : (ros::Time::now() - strike_vision_switch_time_).toSec();
        const double alpha = image_reacquire_blend_sec_ <= 1e-3
            ? 1.0 : clampValue(elapsed / image_reacquire_blend_sec_, 0.0, 1.0);
        cmd.linear.x = (1.0 - alpha) * strike_blend_from_ned_.linear.x + alpha * desired.linear.x;
        cmd.linear.y = (1.0 - alpha) * strike_blend_from_ned_.linear.y + alpha * desired.linear.y;
        cmd.linear.z = (1.0 - alpha) * strike_blend_from_ned_.linear.z + alpha * desired.linear.z;
        cmd.angular.z = (1.0 - alpha) * strike_blend_from_ned_.angular.z + alpha * desired.angular.z;
        applyStrikeCommandSlewLimit(cmd);
        //=======高度下降===========//
        // 必须在 source blend 和 slew limit 之后执行，防止旧下降速度穿透保护高度。
        // closing_speed 用 scheduled_speed (终端减速调度结果),不用 cmd.linear.x/y,
        // 否则会被 feasibleHorizontalSpeedForDescent 钳位后的 h_speed 反向锁死 vz
        // (uav0_strike_2026-08-19-12-58-47_0.bag 复盘)。
        // === 2026-08-20: 图像居中终端冲刺时跳过此 guard ===
        //   chase 中 vx/vy/vz 完全沿相机射线, 几何上天然同步,不靠时间预算算 vz。
        //   guard 会把 ray 的 vz 钳到 time_to_hit 公式,破坏 chase 闭环。
        if (!terminal_image_chase_active_) {
            const StrikeTargetGeometry g_for_guard = strikeDescentGeometry();
            const double closing_speed_for_guard = strike_terminal_speed_schedule_enabled_
                ? terminalHorizontalSpeed(track_max_speed_, g_for_guard.horizontal)
                : track_max_speed_;
            enforcePreCommitAltitudeGuard(cmd, g_for_guard, closing_speed_for_guard);
        }
        //=======高度下降===========//
        last_strike_cmd_ned_ = cmd;
        last_strike_cmd_valid_ = true;

        const StrikeTargetGeometry g = strikeTargetGeometry();
        ROS_INFO_THROTTLE(1.0,
            "[Guidance] %s age=%.2fs horiz=%.2fm dist=%.2fm bbox=(%.3f,%.3f) "
            "cmd_ned=(%.2f,%.2f,%.2f yaw_rate=%.2f)",
            strike_vision_state_ == StrikeVisionState::IMAGE_GUIDANCE ? "IMAGE" : "GEO",
            image_age, g.horizontal, g.distance,
            current_image_target_.center_x, current_image_target_.center_y,
            cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }
    //===================制导修改====================

    void guidanceTimerCallback(const ros::TimerEvent&) {
        // === 2026-08-20: WP2 fallback stale timeout ===
        //   mission_manager 停 publish (cluster 接管 / guidance disable / disarm) 后,
        //   这里清 wp2_fallback_valid_ 防止 stale 数据继续喂 PN。
        if (wp2_fallback_valid_ &&
            !wp2_fallback_last_seen_.isZero() &&
            (ros::Time::now() - wp2_fallback_last_seen_).toSec() >
                wp2_fallback_stale_timeout_sec_) {
            wp2_fallback_valid_ = false;
            ROS_WARN_THROTTLE(2.0, "[Guidance] WP2 fallback stale (>%.2fs); cleared",
                              wp2_fallback_stale_timeout_sec_);
        }

        //=========setpoint====//
        // Guidance velocity 只允许在明确的跟踪/打击阶段发布。TAKING_OFF、
        // HOVERING、GROUND_IDLE 等阶段连零速度也不发，避免与 position 混流。
        //========不要fail safe============//
        // mission_state 仅1Hz发布。Attack交接时 waypoint_executor 会立即停止，
        // 但guidance可能仍缓存上一帧WAYPOINT_FOLLOW，导致整个COM_OF_LOSS_T窗口
        // 没有任何速度setpoint，PX4随即从OFFBOARD进入AUTO.LAND。
        // guidance/enable=true且mode已明确为track/strike，就是任务管理器完成控制权
        // 交接的实时信号；允许它在mission_state下一帧到达前先接管保活。
        const bool mission_phase_allows_velocity =
            mission_state_received_ &&
            (current_mission_phase_ == 4 || current_mission_phase_ == 5);
        const bool guidance_handoff_allows_velocity =
            is_enabled_ && (current_mode_ == "track" || current_mode_ == "strike");
        if ((!mission_phase_allows_velocity && !guidance_handoff_allows_velocity) ||
            (!use_sim_ &&
             (!mavros_state_received_ || !mavros_connected_ || !mavros_armed_))) {
            ROS_DEBUG_THROTTLE(2.0,
                "[Guidance] Velocity publish blocked: FCU not armed or phase is not GUIDANCE");
            return;
        }
        //========不要fail safe============//
        //=========setpoint====//

        // SEARCH_ONLY 模式下 guidance 不工作
        if (!is_enabled_) {
            return;
        }

        if (!is_uav_pose_received_ || !is_target_pose_received_) {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Waiting for UAV pose or target data...");
            // ===== 关键：即使没收到目标，也要持续发零速度维持 PX4 OFFBOARD 心跳 =====
            // PX4 OFFBOARD 模式要求 setpoint 频率 > 2Hz，否则触发 failsafe 自动降落。
            // 起飞后切到 SEARCH_TRACK/STRIKE 时，目标通常还没出现，此时若 guidance
            // 不发任何 setpoint，PX4 就会断流 → 降落。必须用零速度悬停来保活。
            if (is_uav_pose_received_) {
                geometry_msgs::Twist hover_cmd;
                hover_cmd.linear.x = 0.0;
                hover_cmd.linear.y = 0.0;
                hover_cmd.linear.z = 0.0;
                hover_cmd.angular.x = 0.0;
                hover_cmd.angular.y = 0.0;
                hover_cmd.angular.z = 0.0;
                if (!use_sim_) {
                    // NED -> ENU 转换（0 还是 0，不影响）
                    convertVelNedToEnu(hover_cmd);
                }
                publishGuidanceVelocity(hover_cmd);
            }
            return;
        }

        // 注:INTERCEPT 制导为纯追踪(定速飞向目标当前位置),不使用 target_twist。
        // 聚类只产生 2D 位置、无速度,故此处不再门槛拦截 twist。
        // 若将来对动目标启用提前量拦截(见 InterceptGuidance::computeInterceptPoint 的
        // target_vel 预测项),需恢复对 target_estimated_twist 的依赖并在此重新加门槛。

        // Compute guidance command based on strategy type
        switch (current_strategy_type_) {
            case multi_uav_strike::GuidanceStrategyType::INTERCEPT: {

                // Set flight mode to velocity (仅仿真模式发布；PX4 SITL 由 mission_manager 通过 mavros/set_mode 切换)
                if (use_sim_) {
                    std_msgs::Int16 mode_msg;
                    mode_msg.data = flight_mode_velocity_;
                    flight_mode_pub_.publish(mode_msg);
                }

                geometry_msgs::Twist vel_cmd;
                if (current_mode_ == "track") {
                    // TRACK模式：独立PI控制，不做STRIKE评估
                    // 固定高度 + PI控制水平位置
                    computeTrackVelocity(vel_cmd);
                    // 所有策略统一先在 NED 滤波，再在发布前做 NED→ENU；debug 因此始终是 NED。
                    filterVelocityCmd(vel_cmd);
                    if (!use_sim_) {
                        convertVelNedToEnu(vel_cmd);
                    }
                    publishGuidanceVelocity(vel_cmd);
                } else {
                    // STRIKE模式
                    //===================制导修改====================
                    if (strike_guidance_logic_ == "image_geo_fallback") {
                        geometry_msgs::Twist vel_cmd;
                        computeImageGeoStrikeVelocity(vel_cmd);
                        evaluateStrike();
                        filterVelocityCmd(vel_cmd);
                        if (!use_sim_) {
                            convertVelNedToEnu(vel_cmd);
                        }
                        publishGuidanceVelocity(vel_cmd);
                    } else if (strike_guidance_logic_ == "pn") {
                    // 原 PN STRIKE 分支完整保留，并融合main的目标LPF、stale软降速和handover。
                    //===================制导修改====================
                    const bool pn_target_from_this_strike = pn_filtered_target_valid_ &&
                        !pn_last_target_update_time_.isZero() &&
                        (strike_guidance_enable_time_.isZero() ||
                         pn_last_target_update_time_ >= strike_guidance_enable_time_);
                    if (!pn_target_from_this_strike) {
                        ROS_WARN_THROTTLE(1.0,
                            "[Guidance] PN waiting fresh filtered target; publishing zero setpoint");
                        geometry_msgs::Twist hover_cmd;
                        if (!use_sim_) convertVelNedToEnu(hover_cmd);
                        publishGuidanceVelocity(hover_cmd);
                        break;
                    }

                    const bool pn_cluster_stale = !pn_last_target_update_time_.isZero() &&
                        (ros::Time::now() - pn_last_target_update_time_).toSec() >
                            pn_cluster_stale_timeout_sec_;
                    if (pn_cluster_stale && !pn_cluster_stale_latch_) {
                        ROS_WARN("[Guidance] PN target stale > %.2fs; speed scale=%.2f",
                                 pn_cluster_stale_timeout_sec_,
                                 pn_cluster_stale_speed_scale_);
                    }
                    pn_cluster_stale_latch_ = pn_cluster_stale;

                    auto* strategy = static_cast<multi_uav_strike::InterceptGuidance*>(guidance_strategy_.get());
                    auto cmd = strategy->computeCommand(
                        current_uav_pose_,
                        pn_filtered_target_pose_,
                        current_target_twist_);
                    if (pn_cluster_stale) {
                        cmd.velocity *= pn_cluster_stale_speed_scale_;
                    }
                    // InterceptGuidance 输出已经是 NED；这里不能再按 NWU 翻转 Y/Z。
                    // 旧链路的二次翻转会让 East 反向，并把目标在下方时的 D+ 下降量钳成 0。
                    vel_cmd.linear.x = cmd.velocity.x();
                    vel_cmd.linear.y = cmd.velocity.y();

                    // 基于当前 primary target 的几何 LOS pitch 做下降门控。
                    // 使用 cmd.los_pitch 而不是 gimbal 的全局主目标 LOS，避免多目标时门控错对象。
                    // const double pitch = cmd.los_pitch;                           // 下负上正
                    // const double pitch_threshold = 0.20;                          // rad (2026-08-02: 0.5 → 0.20 ≈ 11.5°)
                    //   远距俯冲段 los_pitch 永远 < 28.6°, 旧阈值会把 PN 的 vz 前馈
                    //   压死, UAV 在目标上方水平飞过. 0.20 rad 让 11.5° 以上的俯冲段
                    //   即开始产生下降分量, 短距浅俯冲不再被门控卡住.
                    // const double transition_width = 0.1;                          // rad
                    // 原PN垂直门控参数保留：
                    // const double max_descent_when_level = 0.0;                    // m/s, NED D+
                    // const double smooth_weight = 0.5 * (std::tanh(
                    //     (std::fabs(pitch) - pitch_threshold) / transition_width) + 1.0);
                    // double desired_z = cmd.velocity.z();                          // NED: >0 下沉
                    // if (desired_z < max_descent_when_level) desired_z = max_descent_when_level;
                    //===================制导修改====================
                    // 原main PN垂直门控保留：
                    // vel_cmd.linear.z = smooth_weight * desired_z;
                    // 融合后两种导引共用斜线下降；PN仅保留自己的水平比例导引和偏航。
                    const StrikeTargetGeometry pn_geometry =
                        strikeTargetGeometry(pn_filtered_target_pose_);
                    const StrikeTargetGeometry pn_descent_geometry =
                        strikeDescentGeometry();
                    const double pn_horizontal_speed = std::hypot(
                        vel_cmd.linear.x, vel_cmd.linear.y);
                    // === 2026-08-19: slant 公式用非钳位的 track_max_speed_,治自反馈死锁 ===
                    //   (详见 computeGeoFallbackVelocity 中的同主题注释)
                    vel_cmd.linear.z = computeSlantDescentSpeed(
                        pn_descent_geometry, track_max_speed_);
                    //===================制导修改====================
                    vel_cmd.angular.x = 0.0;
                    vel_cmd.angular.y = 0.0;
                    // === 2026-08-02: STRIKE 偏航跟踪 ===
                    //   现象: 之前 vel_cmd.angular.z 强制 0, 机头冻结在进入 STRIKE 时的航向;
                    //         PN 在世界系推速度, 但机体朝别的方向, PX4 离轴速度激起 0.9-1.4 Hz 姿态抖.
                    //   修复: 复用 TRACK 模式 [computeTrackVelocity] 同样的 yaw rate P 控制,
                    //         desired = cmd.desired_yaw (= los_yaw, PN 已算好).
                    //   关键 frame 修复 (uav0_strike_2026-08-02-10-53-30 bag 复盘):
                    //     (1) current_uav_pose_ 在 use_sim_=false 分支下被 [uavPoseCallback]
                    //         ENU→NWU 变换 (line ~480-491), 所以 atan2(...) 给的是 NWU 偏航.
                    //     (2) cmd.desired_yaw 来自 PN 内部, 走的是 NED 系 (los_yaw 左负右正,
                    //         0=北, +π/2=东).
                    //     (3) NWU→NED: 同一物理指向 yaw_ned = -yaw_nwu (NWU 把西当 +π/2,
                    //         NED 把东当 +π/2, 差一个负号).
                    //     (4) err 公式: positive = "目标在 current 左侧" → 应让 PX4 CCW 转
                    //         (NWU heading 减小, NED heading 减小), 所以 err = current - target.
                    //     (5) PX4 NED: ang.z > 0 → heading 增大 (CW from above).
                    //         经 convertVelNedToEnu (line 1224) 取反后 mavros 发 FLU
                    //         (mavros 不再翻转), PX4 FRD 收到 -rate → heading 减小 (CCW).
                    //         所以 err>0 时直接传 +rate 即可.
                    //   旧版同时有 frame 错 (1) 和符号错 (4): 物理 20° 误差被放大成
                    //     100+° 饱和到 +0.6, 09:43 bag 看到 yaw 90°→116° 反向旋转.
                    const double uav_qx = current_uav_pose_.pose.orientation.x;
                    const double uav_qy = current_uav_pose_.pose.orientation.y;
                    const double uav_qz = current_uav_pose_.pose.orientation.z;
                    const double uav_qw = current_uav_pose_.pose.orientation.w;
                    const double current_yaw_nwu = std::atan2(
                        2.0 * (uav_qw * uav_qz + uav_qx * uav_qy),
                        1.0 - 2.0 * (uav_qy * uav_qy + uav_qz * uav_qz));
                    const double current_yaw_ned = -current_yaw_nwu;                // NWU → NED
                    const double target_bearing_ned = cmd.desired_yaw;             // = los_yaw (PN 内部 NED)
                    // === 2026-08-02: STRIKE 偏航 err 公式修正 ===
                    //   旧版: yaw_err_ctrl = wrap(current_ned - target_ned), vel_cmd.angular.z = +Kp*err
                    //   旧版物理流向(实测 2026-08-02-15-00 bag, t_rel=-0.78):
                    //     NED current=-67.9° (SE), target=+56.6° (NE, 目标在 RIGHT)
                    //     err = -124.5°(负) → yaw_rate = +Kp*(-124.5°) = 负 (NED)
                    //     经 convertVelNedToEnu 取反: published = 正 = CCW from above
                    //     实际需要 CW (目标在右),但发了 CCW → 飞机背对目标兜了一圈
                    //     (yaw_nwu 从 +67.9° 涨到 +195°, CCW 转了 128°,即用户描述"偏左90°再180°")
                    //   根因:旧公式 +Kp*(current - target) 在 NED 下方向反.
                    //   NED 下 +yaw_rate = CW from above (Z down 右手法则),目标在 RIGHT 时
                    //     需要 yaw_rate 为正,所以 err 必须是 (target - current) 才给出正数.
                    //   修法:err = target_bearing_ned - current_yaw_ned,与 TRACK 模式
                    //     [computeTrackVelocity] 保持同一符号约定 (TRACK 用 target-current,
                    //     输出再 -yaw_rate 取反,本质一致).
                    const double yaw_err_ctrl = std::atan2(
                        std::sin(target_bearing_ned - current_yaw_ned),
                        std::cos(target_bearing_ned - current_yaw_ned));
                    // 2026-08-02: STRIKE 用更保守的 kp / max_rate, 避免 87° 大误差下
                    //   卡到 los_max_rate_ 限幅引发的 yaw spin (实测 09:18 bag: yaw std=54°,
                    //   body_rate r 饱和 ±1.4 rad/s). 0.5 / 0.6 不影响 TRACK 模式.
                    // === 2026-08-19 Fix A2: 提速但仍保守 ===
                    //   背景:0.6 rad/s (34°/s) 太慢,146° 偏差需要 4.3s 才能追上,
                    //   期间 heading 在目标方位外 → user 观察"航向背对目标"。
                    //   现在 startGuidanceApproach (Fix A1) 已经先把 yaw 锁到目标方位
                    //   再 enable,残留 yaw_diff ≤ 30°,0.8 / 1.0 足够 0.4s 收敛。
                    //   保守提速:PX4 物理允许 ~3 rad/s,我们留余量到 1.0。
                    const double strike_los_kp_yaw    = 0.8;
                    const double strike_los_max_rate  = 1.0;
                    double yaw_rate = strike_los_kp_yaw * yaw_err_ctrl;
                    yaw_rate = std::max(-strike_los_max_rate, std::min(strike_los_max_rate, yaw_rate));
                    vel_cmd.angular.z = yaw_rate;

                    // === 2026-08-20: 图像居中终端冲刺 (覆盖 PN 输出) ===
                    //   当 bbox 在 deadband 内 + g.horizontal ≤ 阈值时, 直接沿相机射线 3D
                    //   全速冲向, 完全覆盖 PN 输出的 vx/vy/vz/yaw_rate。
                    //   PN 的世界坐标追击在近距会因 cluster z=0 + 图像未用 → 砸地,
                    //   切 chase 后 3D 速度由相机射线主导, 几何天然同步, 不刹停。
                    //   chase 中仍跑 filterVelocityCmd (指令斜率限制), 不跑 pre-commit guard。
                    const StrikeTargetGeometry pn_chase_geometry = strikeDescentGeometry();
                    maybeApplyTerminalImageChase(vel_cmd, pn_chase_geometry);

                    // 保存 pitch 门控后的 NED 指令；One Euro 也统一在 NED 下运行。
                    const geometry_msgs::Vector3 gated_velocity_ned = vel_cmd.linear;
                    evaluateStrike();
                    filterVelocityCmd(vel_cmd);
                    //=======高度下降===========//
                    // PN 滤波器也可能残留旧下降量，使用同一提交高度硬保护。
                    // closing_speed 用 track_max_speed_,不用被 PN 钳位后的 vel_cmd.linear.x/y,
                    // 否则会被自反馈死锁 (uav0_strike_2026-08-19-12-58-47_0.bag 复盘)。
                    // === 2026-08-20: chase 中跳过 pre-commit guard (同上 image_geo_fallback) ===
                    if (!terminal_image_chase_active_) {
                        enforcePreCommitAltitudeGuard(vel_cmd, pn_descent_geometry, track_max_speed_);
                    }
                    //=======高度下降===========//

                    multi_uav_strike::GuidancePnDebug pn_dbg;
                    pn_dbg.header.stamp = ros::Time::now();
                    pn_dbg.header.frame_id = "nwu_position_ned_velocity";
                    pn_dbg.uav_position_nwu = current_uav_pose_.pose.position;
                    pn_dbg.target_position_nwu = pn_filtered_target_pose_.pose.position;
                    pn_dbg.strategy_velocity_ned.x = cmd.velocity.x();
                    pn_dbg.strategy_velocity_ned.y = cmd.velocity.y();
                    pn_dbg.strategy_velocity_ned.z = cmd.velocity.z();
                    pn_dbg.gated_velocity_ned = gated_velocity_ned;
                    pn_dbg.filtered_velocity_ned = vel_cmd.linear;
                    const double rel_x = pn_filtered_target_pose_.pose.position.x -
                                         current_uav_pose_.pose.position.x;
                    const double rel_y = pn_filtered_target_pose_.pose.position.y -
                                         current_uav_pose_.pose.position.y;
                    const double rel_z = pn_filtered_target_pose_.pose.position.z -
                                         current_uav_pose_.pose.position.z;
                    pn_dbg.horizontal_distance = std::hypot(rel_x, rel_y);
                    pn_dbg.distance = std::sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z);
                    pn_dbg.los_yaw = cmd.los_yaw;
                    pn_dbg.los_pitch = cmd.los_pitch;
                    pn_dbg.los_yaw_rate = cmd.los_yaw_rate;
                    pn_dbg.los_pitch_rate = cmd.los_pitch_rate;
                    pn_dbg.desired_yaw = cmd.desired_yaw;
                    //===================制导修改====================
                    // 原俯仰门控调试量保留：
                    // pn_dbg.pitch_gate_weight = smooth_weight;
                    // 当前PN垂直速度不再经过俯仰门控，固定报告完全放行。
                    pn_dbg.pitch_gate_weight = 1.0;
                    //===================制导修改====================

                    if (!use_sim_) {
                        convertVelNedToEnu(vel_cmd);
                    }
                    publishGuidanceVelocity(vel_cmd);
                    pn_debug_pub_.publish(pn_dbg);
                    publishInterceptPointMarker(cmd.intercept_point_nwu);
                    //===================制导修改====================
                    }
                    //===================制导修改====================
                }
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::MINSNAP: {
                auto* strategy = static_cast<multi_uav_strike::MinSnapGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_);

                publishAttitudeThrust(cmd);
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::LOS: {
                if (!is_los_received_) {
                    ROS_WARN_THROTTLE(5.0, "[Guidance] LOS angle not available for LOS guidance");
                    return;
                }
                auto* strategy = static_cast<multi_uav_strike::LosGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_,
                    current_los_angle_);

                publishAttitudeThrust(cmd);
                break;
            }
        }
    }

    // === 2026-07-16: 速度指令 One Euro Filter 统一入口 ===
    //   TRACK/STRIKE 两个分支在 publish 前都过这里一次,共用 dt。
    //   目标位置突变(>5m)检测已搬到 targetPoseCallback 内:在那条路径上
    //   resetLosFilter() 会在下一次 PN computeCommand 之前同步生效,
    //   旧目标的 LOS 角速度不会套到新目标首帧。
    //   这里只负责 dt 计算 + 四通道 One Euro 平滑 + 滤波前后 debug 上报。
    //   注意:这里读的是 publish 前的 vel_cmd(NED 系),但滤波状态本身与坐标无关,
    //   同 dt 同样的物理量,坐标转换不影响平滑效果。
    void filterVelocityCmd(geometry_msgs::Twist& vel_cmd) {
        // ---- 1) dt ----
        ros::Time now = ros::Time::now();
        double te = vel_filter_last_time_.isZero()
                        ? (1.0 / 50.0)
                        : (now - vel_filter_last_time_).toSec();
        vel_filter_last_time_ = now;
        if (te <= 0.0) te = 1e-3;
        if (te > 1.0)  te = 1.0;  // 长时间没调用(断流/暂停)→ 限幅,避免 filter 一帧暴走

        // ---- 2) 四通道 One Euro ----
        // 先 snapshot raw(NED),filter 之后再 publish 一起发到 debug topic
        const double raw_vx = vel_cmd.linear.x;
        const double raw_vy = vel_cmd.linear.y;
        const double raw_vz = vel_cmd.linear.z;
        vel_cmd.linear.x  = vel_filter_x_.filter(vel_cmd.linear.x,  te);
        vel_cmd.linear.y  = vel_filter_y_.filter(vel_cmd.linear.y,  te);
        vel_cmd.linear.z  = vel_filter_z_.filter(vel_cmd.linear.z,  te);
        vel_cmd.angular.z = vel_filter_yaw_.filter(vel_cmd.angular.z, te);

        // === 2026-07-16: Debug — raw vs filtered 速度 ===
        //   用 rqt_plot 看 vel_raw - vel_filtered 的 residual,
        //   若 residual 平稳但 UAV 还抖 → 不是 filter 的问题,可能是 PX4 控制环增益
        //   若 residual 跟 UAV 抖同步 → One Euro 不够强,降 min_cutoff 或升 beta
        multi_uav_strike::TargetFilterDebug dbg;
        dbg.header.stamp    = now;
        dbg.header.frame_id = "ned";
        // 位置维度本节点不直接产生(由 mission_manager 计算),填 0
        dbg.pos_raw = dbg.pos_filtered = dbg.pos_delta = geometry_msgs::Point();
        // 注:raw 是 NED 系(进入 filterVelocityCmd 之前),filtered 同 NED;
        //   想跟 PX4 期望对比就直接 echo /guidance/target_filter_debug
        dbg.vel_raw.x = raw_vx;  dbg.vel_raw.y = raw_vy;  dbg.vel_raw.z = raw_vz;
        dbg.vel_filtered.x = vel_cmd.linear.x;
        dbg.vel_filtered.y = vel_cmd.linear.y;
        dbg.vel_filtered.z = vel_cmd.linear.z;
        dbg.vel_delta.x = raw_vx - vel_cmd.linear.x;
        dbg.vel_delta.y = raw_vy - vel_cmd.linear.y;
        dbg.vel_delta.z = raw_vz - vel_cmd.linear.z;
        dbg.cluster_id = 0;  // 本节点不感知 cluster_id,留给 mission_manager 的 msg
        dbg.is_locked  = is_target_pose_received_;
        dbg.source     = "velocity";
        target_filter_debug_pub_.publish(dbg);
    }

    // TRACK模式专用：基于视线角俯仰角的 stand-off 跟踪
    //
    // 设计目标（替代旧的 PI 飞向目标）：
    //   1) 识别到目标后保持在 stand-off 圆周上，使目标 LOS 俯仰角稳定在 desired_los_pitch_deg_
    //      （默认 30°）。这样摄像头（默认 45° 向下）持续看到目标，且为后续 strike 留出俯冲空间。
    //   2) 航向始终指向目标（target_bearing）。
    //   3) 不允许后退飞行：任何会让机体坐标系前向速度 < 0 的指令都归零。
    //      若目标偏离航向 > yaw_los_threshold_deg_（默认 45°），优先转向，前进速度大幅衰减，
    //      避免大角度侧冲或后退撞障碍物。
    //
    // 几何关系：
    //   tan(pitch) = alt_diff / horiz_dist  →  horiz_dist = alt_diff / tan(pitch)
    //   alt_diff 为 UAV 相对目标的高度差（正值 = UAV 高于目标）。
    void computeTrackVelocity(geometry_msgs::Twist& vel_cmd) {
        // ====== 调试节流计数器：每 DBG_PERIOD 次循环打一次（约 10Hz @ 50Hz 主循环） ======
        // static int dbg_tick = 0;
        // const int DBG_PERIOD = 5;
        // const bool dbg_this = (++dbg_tick % DBG_PERIOD) == 0;

        const double uav_x = current_uav_pose_.pose.position.x;
        const double uav_y = current_uav_pose_.pose.position.y;
        const double uav_z = current_uav_pose_.pose.position.z;

        const double tgt_x = current_target_pose_.pose.position.x;
        const double tgt_y = current_target_pose_.pose.position.y;
        const double tgt_z = current_target_pose_.pose.position.z;

        // 目标相对 UAV（NWU）
        const double dx = tgt_x - uav_x;
        const double dy = tgt_y - uav_y;
        const double dz = tgt_z - uav_z;

        const double horiz_dist = std::sqrt(dx*dx + dy*dy);
        const double alt_diff = uav_z - tgt_z;  // > 0 表示 UAV 高于目标

        // 当前航向（NWU：左转为正，从上方看是 CCW，0=North）
        const double current_yaw = atan2(
            2.0 * (current_uav_pose_.pose.orientation.w * current_uav_pose_.pose.orientation.z +
                   current_uav_pose_.pose.orientation.x * current_uav_pose_.pose.orientation.y),
            1.0 - 2.0 * (current_uav_pose_.pose.orientation.y * current_uav_pose_.pose.orientation.y +
                         current_uav_pose_.pose.orientation.z * current_uav_pose_.pose.orientation.z));

        // 目标方位角（NWU：0=North, +π/2=West）
        const double target_bearing = std::atan2(dy, dx);

        // 航向偏差：航向 → 目标（归一化到 [-π, π]）
        double yaw_error = target_bearing - current_yaw;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
        const double yaw_error_abs_deg = std::fabs(yaw_error) * 180.0 / M_PI;

        // ====== 原始 LOS 输入 + 几何对照 ======
        // gimbal 发的是 NED 系: los.x = -desired_yaw, los.y = -desired_pitch
        const double los_pitch_in_ned = is_los_received_ ? current_los_angle_.y : 0.0;  // NED, 通常为负(看向下)
        const double los_pitch_in_nwu_deg = -los_pitch_in_ned * 180.0 / M_PI;             // NWU, 正=在水平线下
        const double los_yaw_in_nwu_deg   = (is_los_received_ ? -current_los_angle_.x : 0.0) * 180.0 / M_PI;
        // 我们自己几何算出的实际俯仰角（NED 系，对照 gimbal 的 los.y）
        const double geom_pitch_ned_rad = (horiz_dist > 1e-3) ? std::atan2(alt_diff, horiz_dist) : 0.0;
        // 两者越接近 = LOS 越可信、估计的位置和云台视野一致
        const double los_geom_pitch_diff_deg = (los_pitch_in_ned - geom_pitch_ned_rad) * 180.0 / M_PI;

        // =====================================================================
        // 1) 计算期望 stand-off 水平距离（基于期望 LOS 俯仰角）
        // =====================================================================
        const double desired_los_pitch_rad = desired_los_pitch_deg_ * M_PI / 180.0;
        double desired_horiz_dist;
        if (alt_diff > 1.0) {
            desired_horiz_dist = alt_diff / std::tan(desired_los_pitch_rad);
        } else {
            // UAV 太低 / 与目标同高度，几何退化：用固定安全距离兜底
            desired_horiz_dist = track_horiz_dist_;
        }
        // 限幅，避免距离过近（目标在脚下）或过远（脱离跟踪）
        desired_horiz_dist = std::max(min_horiz_dist_, std::min(max_horiz_dist_, desired_horiz_dist));

        // =====================================================================
        // 2) 径向 P 控制：水平方向只关心 stand-off 距离，不让 UAV 飞到目标正上方
        //    horiz_dist > desired → 朝目标径向运动（径向速度为负 = 内移）
        //    horiz_dist < desired → 远离目标径向运动（径向速度为正 = 外移）
        // =====================================================================
        const double horiz_dist_error = horiz_dist - desired_horiz_dist;
        const double radial_speed_raw = track_k_approach_ * horiz_dist_error;  // P 控制（未限幅）
        double radial_speed = radial_speed_raw;
        // 限幅到 ±track_max_speed_
        const bool radial_speed_clamped = (radial_speed_raw != std::max(-track_max_speed_, std::min(track_max_speed_, radial_speed_raw)));
        radial_speed = std::max(-track_max_speed_, std::min(track_max_speed_, radial_speed));

        // 径向单位向量（UAV → target 在水平面上的投影）
        const double radial_x = (horiz_dist > 1e-3) ? (dx / horiz_dist) : 1.0;
        const double radial_y = (horiz_dist > 1e-3) ? (dy / horiz_dist) : 0.0;

        // 世界坐标系下的期望水平速度（NWU）-- 还未转机体
        double cmd_x = radial_x * radial_speed;
        double cmd_y = radial_y * radial_speed;
        const double cmd_world_pre_norm = std::sqrt(cmd_x*cmd_x + cmd_y*cmd_y);

        // =====================================================================
        // 3) 转机体坐标系，应用"不后退"约束
        //    NWU heading 向量 = (cos(yaw), sin(yaw))
        //    左向量 = (-sin(yaw), cos(yaw)) （CCW 旋转 90°）
        // =====================================================================
        const double cos_yaw = std::cos(current_yaw);
        const double sin_yaw = std::sin(current_yaw);
        const double cmd_fwd_pre  =  cmd_x * cos_yaw + cmd_y * sin_yaw;   // 前向（机头方向，转机体前）
        const double cmd_left_pre = -cmd_x * sin_yaw + cmd_y * cos_yaw;   // 左向（转机体前）
        double cmd_fwd  = cmd_fwd_pre;
        double cmd_left = cmd_left_pre;

        // 硬约束：不允许任何后退（cmd_fwd < 0 全部归零）
        const bool clamped_no_backward = (cmd_fwd_pre < 0.0);
        if (cmd_fwd < 0.0) {
            cmd_fwd = 0.0;
        }

        // 横向速度限幅（避免大角度侧冲）
        const double max_lat_speed = track_max_speed_ * 0.5;
        const bool clamped_lat = (std::fabs(cmd_left_pre) > max_lat_speed);
        cmd_left = std::max(-max_lat_speed, std::min(max_lat_speed, cmd_left));

        // 目标偏离航向 > 阈值时优先转向：前进大幅衰减，让 yaw rate 把机头先转过来
        const bool yaw_threshold_active = (yaw_error_abs_deg > yaw_los_threshold_deg_);
        if (yaw_threshold_active) {
            cmd_fwd *= 0.2;    // 大幅降低前进
            cmd_left *= 0.3;   // 横向也收紧
        }

        // 速度限幅（防数值问题）
        double speed_xy = std::sqrt(cmd_fwd*cmd_fwd + cmd_left*cmd_left);
        const bool body_speed_clamped = (speed_xy > track_max_speed_);
        if (speed_xy > track_max_speed_) {
            double s = track_max_speed_ / speed_xy;
            cmd_fwd *= s;
            cmd_left *= s;
        }

        // 转回世界坐标系
        cmd_x = cmd_fwd * cos_yaw - cmd_left * sin_yaw;
        cmd_y = cmd_fwd * sin_yaw + cmd_left * cos_yaw;
        const double cmd_world_post_norm = std::sqrt(cmd_x*cmd_x + cmd_y*cmd_y);

        // =====================================================================
        // 4) 高度控制：固定 track_altitude_（P 控制）
        // =====================================================================
        const double alt_error = uav_z - track_altitude_;
        double cmd_z = std::max(-1.0, std::min(1.0, -0.5 * alt_error));  // NED: z<0=上升
        const double alt_clamped = (alt_error > 2.0 || alt_error < -2.0);

        // =====================================================================
        // 5) 机间避障（在世界系叠加斥力）
        // =====================================================================
        const double cmd_x_pre_avoid = cmd_x;
        const double cmd_y_pre_avoid = cmd_y;
        const double cmd_z_pre_avoid = cmd_z;
        applyInterUavAvoidance(cmd_x, cmd_y, cmd_z);

        // =====================================================================
        // 6) 输出（NWU → NED 转换）
        // =====================================================================
        vel_cmd.linear.x = cmd_x;
        vel_cmd.linear.y = -cmd_y;  // NED E = -NWU W
        vel_cmd.linear.z = -cmd_z;   // NED D (z<0=up)
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;

        // =====================================================================
        // 7) 偏航角速度控制：航向始终指向目标（target_bearing）
        //    当目标偏离航向 > yaw_los_threshold_deg_ 时，yaw_rate 会迅速把机头转过去，
        //    一旦目标进入前方扇区，前向速度恢复，全速接近 stand-off。
        // =====================================================================
        const double yaw_err_ctrl = std::atan2(std::sin(target_bearing - current_yaw),
                                                std::cos(target_bearing - current_yaw));
        const double yaw_rate_raw = los_kp_yaw_ * yaw_err_ctrl;
        double yaw_rate = yaw_rate_raw;
        const bool yaw_rate_clamped = (std::fabs(yaw_rate_raw) > los_max_rate_);
        yaw_rate = std::max(-los_max_rate_, std::min(los_max_rate_, yaw_rate));
        // NWU -> NED
        vel_cmd.angular.z = -yaw_rate;

        // =====================================================================
        // 8) 综合调试打印：每 DBG_PERIOD 次循环打一次（约 10Hz）
        //    一次性把每一步的中间量都列出来，方便定位"为何没追上 / 姿态乱跳"
        // =====================================================================
        // if (dbg_this) {
        //     ROS_WARN(
        //         "\n========== [TRACK DEBUG tick=%d] =========="
        //         "\n[INPUT]"
        //         "\n  uav_pos[NWU]   = (%.2f, %.2f, %.2f)   alt_asl=%.2f"
        //         "\n  tgt_pos[NWU]   = (%.2f, %.2f, %.2f)"
        //         "\n  uav_quat(wxyz) = (%.3f, %.3f, %.3f, %.3f)"
        //         "\n  los_received  = %d"
        //         "\n  los_in[NED]   (yaw,pitch,conf) = (%.3f rad, %.3f rad, %.3f)"
        //         "\n  los_in[NWU deg] (yaw,pitch)   = (%.1f, %.1f)   <-- gimbal 直接发出来的"
        //         "\n[GEOMETRY]"
        //         "\n  rel_pos (dx,dy,dz) = (%.2f, %.2f, %.2f)   (target - uav in NWU)"
        //         "\n  horiz_dist      = %.2f m"
        //         "\n  alt_diff (uav-tgt) = %.2f m"
        //         "\n  geom_pitch[nwu deg] = %.1f   (= atan2(alt_diff, horiz_dist))"
        //         "\n  los_pitch[nwu deg]  = %.1f   (来自 los_angle.y)"
        //         "\n  diff(los - geom) = %+.1f deg   <-- 偏离 0 = LOS 与几何一致，越大说明 LOS 已经不指向真目标"
        //         "\n[YAW]"
        //         "\n  current_yaw   = %.1f deg   (NWU: 0=N, +π/2=W)"
        //         "\n  target_bearing= %.1f deg   (NWU)"
        //         "\n  yaw_error     = %+.1f deg (|err|=%.1f, threshold=%.1f, attenuation_active=%s)"
        //         "\n  los_yaw_in    = %.1f deg   (gimbal 发的，对照 current_yaw 看 gimbal 实际转向)"
        //         "\n[DESIRED STAND-OFF]"
        //         "\n  desired_los_pitch_deg = %.1f   (rad=%.3f, tan=%.3f)"
        //         "\n  desired_horiz_dist    = %.2f  (raw=horiz_pitch+clamp [%.1f, %.1f], fallback_for_low_alt=%.2f)"
        //         "\n[RADIAL P CONTROL]"
        //         "\n  horiz_dist_error       = horiz - desired = %.2f"
        //         "\n  radial_speed raw       = -k * err = -%.2f * %.2f = %.3f"
        //         "\n  radial_speed after cap = %.3f   (clamped_by_max_speed=%s, max=%.2f)"
        //         "\n  radial_unit (radial_x, radial_y) = (%.3f, %.3f)"
        //         "\n[WORLD VEL BEFORE BODY CONVERSION]"
        //         "\n  cmd_world_pre = (%.3f, %.3f)   |.|= %.3f"
        //         "\n[BODY CONVERSION  (cos=%.3f, sin=%.3f of current_yaw)]"
        //         "\n  pre-clamp : cmd_fwd_pre = %.3f   cmd_left_pre = %.3f   |body_pre|=%.3f"
        //         "\n  step1 no-backward clamp active=%s   (would have sent fwd=%.3f backward, now 0)"
        //         "\n  step2 lat-lim active=%s   (cap=±%.3f)"
        //         "\n  step3 yaw-threshold attenuation active=%s   (fwd x0.2, left x0.3)"
        //         "\n  step4 body-norm cap active=%s"
        //         "\n  post-clamp: cmd_fwd_final = %.3f   cmd_left_final = %.3f   |body_final|=%.3f"
        //         "\n[WORLD VEL AFTER BODY->WORLD  (before avoidance)]"
        //         "\n  cmd_world_pre_avoid = (%.3f, %.3f)   |.|= %.3f"
        //         "\n[ALTITUDE]"
        //         "\n  uav_z=%.2f  track_altitude=%.2f  err=%.2f  -> cmd_z=%.3f (NED: <0=up)  saturated=%s"
        //         "\n[INTER-UAV AVOIDANCE]"
        //         "\n  neighbors=%lu"
        //         "\n  pre  avoid: (%.3f, %.3f, %.3f)"
        //         "\n  post avoid: (%.3f, %.3f, %.3f)"
        //         "\n  delta      : (%.3f, %.3f, %.3f)"
        //         "\n[YAW RATE]"
        //         "\n  yaw_err_ctrl    = %.3f rad  (= %+.1f deg)"
        //         "\n  yaw_rate raw    = kp*err = %.2f * %.3f = %.3f"
        //         "\n  yaw_rate after clamp to ±%.2f  =>  %.3f   (clamped=%s)"
        //         "\n[PARAMS]"
        //         "\n  track_k_approach=%.2f  track_max_speed=%.2f  max_lat_speed=track_max*0.5=%.3f"
        //         "\n  los_kp_yaw=%.2f  los_max_rate=%.2f  yaw_los_threshold=%.1f"
        //         "\n[FINAL OUTPUT  (NED, 发送给飞控)]"
        //         "\n  vel_cmd.linear  = (%.3f, %.3f, %.3f)"
        //         "\n  vel_cmd.angular.z (yaw_rate, NED) = %.3f"
        //         "\n==========",
        //         dbg_tick,
        //         uav_x, uav_y, uav_z, uav_z,
        //         tgt_x, tgt_y, tgt_z,
        //         current_uav_pose_.pose.orientation.w, current_uav_pose_.pose.orientation.x,
        //         current_uav_pose_.pose.orientation.y, current_uav_pose_.pose.orientation.z,
        //         is_los_received_ ? 1 : 0,
        //         current_los_angle_.x, current_los_angle_.y, current_los_angle_.z,
        //         los_yaw_in_nwu_deg, los_pitch_in_nwu_deg,
        //         dx, dy, dz,
        //         horiz_dist,
        //         alt_diff,
        //         -geom_pitch_ned_rad * 180.0 / M_PI,
        //         los_pitch_in_nwu_deg,
        //         los_geom_pitch_diff_deg,
        //         current_yaw * 180.0 / M_PI,
        //         target_bearing * 180.0 / M_PI,
        //         yaw_error * 180.0 / M_PI, yaw_error_abs_deg, yaw_los_threshold_deg_,
        //         yaw_threshold_active ? "YES" : "no",
        //         los_yaw_in_nwu_deg,
        //         desired_los_pitch_deg_, desired_los_pitch_rad, std::tan(desired_los_pitch_rad),
        //         desired_horiz_dist, min_horiz_dist_, max_horiz_dist_, track_horiz_dist_,
        //         horiz_dist_error,
        //         track_k_approach_, horiz_dist_error, radial_speed_raw,
        //         radial_speed,
        //         radial_speed_clamped ? "YES" : "no", track_max_speed_,
        //         radial_x, radial_y,
        //         cmd_x, cmd_y, cmd_world_pre_norm,
        //         cos_yaw, sin_yaw,
        //         cmd_fwd_pre, cmd_left_pre,
        //         std::sqrt(cmd_fwd_pre*cmd_fwd_pre + cmd_left_pre*cmd_left_pre),
        //         clamped_no_backward ? "YES" : "no", cmd_fwd_pre,
        //         clamped_lat ? "YES" : "no", max_lat_speed,
        //         yaw_threshold_active ? "YES" : "no",
        //         body_speed_clamped ? "YES" : "no",
        //         cmd_fwd, cmd_left, std::sqrt(cmd_fwd*cmd_fwd + cmd_left*cmd_left),
        //         cmd_x_pre_avoid, cmd_y_pre_avoid, cmd_world_post_norm,
        //         uav_z, track_altitude_, alt_error, cmd_z, alt_clamped ? "YES" : "no",
        //         neighbors_.size(),
        //         cmd_x_pre_avoid, cmd_y_pre_avoid, cmd_z_pre_avoid,
        //         cmd_x, cmd_y, cmd_z,
        //         cmd_x - cmd_x_pre_avoid, cmd_y - cmd_y_pre_avoid, cmd_z - cmd_z_pre_avoid,
        //         yaw_err_ctrl, yaw_err_ctrl * 180.0 / M_PI,
        //         los_kp_yaw_, yaw_err_ctrl, yaw_rate_raw,
        //         los_max_rate_, yaw_rate, yaw_rate_clamped ? "YES" : "no",
        //         track_k_approach_, track_max_speed_, max_lat_speed,
        //         los_kp_yaw_, los_max_rate_, yaw_los_threshold_deg_,
        //         vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
        //         vel_cmd.angular.z
        //     );
        // }
    }

    /**
     * 将 NED 速度指令转换为 ENU（用于 PX4 SITL）
     * NED: x=North, y=East, z=Down
     * ENU: x=East, y=North, z=Up
     */
    void convertVelNedToEnu(geometry_msgs::Twist& vel_cmd) {
        double ned_vx = vel_cmd.linear.x;
        double ned_vy = vel_cmd.linear.y;
        double ned_vz = vel_cmd.linear.z;
        double ned_yaw = vel_cmd.angular.z;

        // NED -> ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        vel_cmd.linear.x = ned_vy;       // ENU East = NED East
        vel_cmd.linear.y = ned_vx;       // ENU North = NED North
        vel_cmd.linear.z = -ned_vz;      // ENU Up = -NED Down

        // Yaw: NED heading -> ENU heading
        // NED: 0=North, PI/2=East; ENU: 0=East, PI/2=North
        // yaw_enu = yaw_ned - PI/2, 但 PX4 期望的是 body frame rate
        // 这里简单取负值，与位置转换保持一致
        vel_cmd.angular.z = -ned_yaw;
    }

    void publishAttitudeThrust(const multi_uav_strike::AttitudeThrustCommand& cmd) {
        // Publish attitude (NWU → NED转换: w,x不变, y,z取反)
        geometry_msgs::PoseStamped att_cmd;
        att_cmd.header.stamp = ros::Time::now();
        att_cmd.header.frame_id = "map";
        att_cmd.pose.position.x = 0.0;
        att_cmd.pose.position.y = 0.0;
        att_cmd.pose.position.z = 0.0;
        att_cmd.pose.orientation.w = cmd.attitude.w();
        att_cmd.pose.orientation.x = cmd.attitude.x();
        att_cmd.pose.orientation.y = -cmd.attitude.y();  // NWU -> NED
        att_cmd.pose.orientation.z = -cmd.attitude.z();  // NWU -> NED
        attitude_cmd_pub_.publish(att_cmd);

        // Publish thrust(mavros setpoint_attitude/thrust 要求 mavros_msgs::Thrust)
        mavros_msgs::Thrust thrust_cmd;
        thrust_cmd.header.stamp = ros::Time::now();
        thrust_cmd.thrust = cmd.thrust;
        thrust_cmd_pub_.publish(thrust_cmd);

        // Set flight mode to attitude (仅仿真模式发布；PX4 SITL 由 mission_manager 通过 mavros/set_mode 切换)
        if (use_sim_) {
            std_msgs::Int16 mode_msg;
            mode_msg.data = flight_mode_attitude_;
            flight_mode_pub_.publish(mode_msg);
        }

        // Publish angular velocity (yaw_rate) via setpoint_attitude/cmd_vel
        geometry_msgs::TwistStamped vel_cmd;
        vel_cmd.header.stamp = ros::Time::now();
        vel_cmd.header.frame_id = "map";
        vel_cmd.twist.linear.x = 0.0;
        vel_cmd.twist.linear.y = 0.0;
        vel_cmd.twist.linear.z = 0.0;
        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = cmd.yaw_rate;  // 偏航角速率
        attitude_rates_pub_.publish(vel_cmd);

        // 发布推力方向可视化（用于RViz显示）
        publishThrustDirectionMarker(cmd);
    }

    // 发布推力方向可视化 marker
    void publishThrustDirectionMarker(const multi_uav_strike::AttitudeThrustCommand& cmd) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = viz_frame_;
        marker.ns = "thrust_direction";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 箭头起点：无人机位置
        marker.points.resize(2);
        marker.points[0].x = current_uav_pose_.pose.position.x;
        marker.points[0].y = current_uav_pose_.pose.position.y;
        marker.points[0].z = current_uav_pose_.pose.position.z;

        // 箭头终点：根据姿态和推力计算方向
        Eigen::Quaterniond quat(cmd.attitude.w(), cmd.attitude.x(), cmd.attitude.y(), cmd.attitude.z());
        Eigen::Vector3d thrust_dir = quat * Eigen::Vector3d(0, 0, 1);  // 机体系Z轴为推力方向
        double arrow_length = 3.0;  // 箭头长度3米
        marker.points[1].x = marker.points[0].x + thrust_dir.x() * arrow_length;
        marker.points[1].y = marker.points[0].y + thrust_dir.y() * arrow_length;
        marker.points[1].z = marker.points[0].z + thrust_dir.z() * arrow_length;

        // 推力越大箭头越粗
        marker.scale.x = 0.1 + cmd.thrust * 0.3;  // 直径0.1~0.4m
        marker.scale.y = 0.2 + cmd.thrust * 0.6;   // 头部宽度0.2~0.8m
        marker.scale.z = 0.0;

        // 颜色：推力越大越红
        marker.color.r = 0.5 + cmd.thrust * 0.5;
        marker.color.g = 0.5 - cmd.thrust * 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.1);
        thrust_dir_pub_.publish(marker);
    }

    // 发布拦截点可视化 marker
    void publishInterceptPointMarker(const Eigen::Vector3d& intercept_point) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = viz_frame_;
        marker.ns = "intercept_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = intercept_point.x();
        marker.pose.position.y = intercept_point.y();
        marker.pose.position.z = intercept_point.z();
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.scale.x = 1.0;  // 球直径1米
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.1);
        intercept_point_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "guidance_control_node");
    GuidanceControlNode node;
    ros::spin();
    return 0;
}
