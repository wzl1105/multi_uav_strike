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
#include <visualization_msgs/Marker.h>
#include <limits>
#include <cmath>
#include <Eigen/Eigen>

#include "multi_uav_strike/guidance_strategies.h"
#include "multi_uav_strike/GuidancePnDebug.h"   // PN 坐标/LOS/速度链路 rosbag 诊断
#include "multi_uav_strike/TargetFilterDebug.h"  // 滤波前后对比 debug msg (2026-07-16)
#include "multi_uav_strike/one_euro_filter.h"  // 速度指令平滑(2026-07-16),消除聚类抖动→PD→UAV 晃
#include "multi_uav_strike/InterUavStatus.h"  // /inter_uav/uav_status 心跳(替代 other_uav_poses)

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
    ros::Subscriber mode_sub_;        // 模式控制："strike" 或 "track"
    ros::Subscriber uav_speed_sub_;   // 拦截速度(来自 mission_manager, 下发到 InterceptGuidance)
    ros::Subscriber heartbeat_sub_;        // 邻居心跳 /inter_uav/uav_status(替代原 other_uav_poses)
    ros::Subscriber home_position_sub_;    // PX4 home,锁一次用于 GPS→NED 转换

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
    std::string viz_frame_;              // RViz 帧名 "<ns>/map"(多机分离由 static TF 偏移)

    // Timer for guidance computation
    ros::Timer guidance_timer_;

    // Current state
    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::PoseStamped current_target_pose_;
    geometry_msgs::TwistStamped current_target_twist_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::Point real_target_pos_;  // 真实目标位置

    bool is_uav_pose_received_ = false;
    bool is_target_pose_received_ = false;
    bool is_target_twist_received_ = false;
    bool is_los_received_ = false;
    bool is_real_target_received_ = false;

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
    // === 2026-08-04: STRIKE 反馈环解耦 (Layer 5) — PN 输入 LPF ===
    //   target_estimated_pose 在 10Hz 发布,带 0.2m std 噪声(target_estimator EMA α=0.1 后
    //   还会有 ~0.05m 残余)。PN 50Hz 调用时在帧边界阶跃,los_yaw_rate 单帧最大 ~0.04 rad/s。
    //   对 target 输入再加一层 1Hz OneEuro 滤波,代价 ~0.1s 滞后,STRIKE 时间窗够用。
    multi_uav_strike::OneEuroFilter1D target_filter_x_;
    multi_uav_strike::OneEuroFilter1D target_filter_y_;
    multi_uav_strike::OneEuroFilter1D target_filter_z_;
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
    geometry_msgs::Vector3 last_known_uav_velocity_ned_;  // pose-delta 计算
    geometry_msgs::Point   prev_uav_pos_ned_;
    ros::Time               prev_uav_pos_time_;
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
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);  // 2米内视为击中
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
        // === 2026-08-04: 再降 min_cutoff + 升 beta, 进一步压 vz/vx 中频抖动 ===
        //   背景:target_estimated_pose 10Hz + 0.2m std → PN 输出 0.144/0.184 m/s 单帧跳变。
        //   新默认值 0.25Hz + beta=0.02:
        //     - 0.25Hz 截止频率把 0.5-1Hz 中频成分再削 6dB
        //     - beta=0.02 让突然加速(d|v|/dt 大)时截止频率升到 ~1-2Hz,仍能跟上
        //       目标切换 / STRIKE 启动初期的大速度变化
        //   d_cutoff 1.0→0.5:导数估计更平滑,beta 放大截止频率时不会"毛刺"瞬时打开。
        double vel_min_cutoff, vel_beta, vel_d_cutoff;
        nh_private_.param<double>("vel_smooth_min_cutoff_hz", vel_min_cutoff, 0.25);
        nh_private_.param<double>("vel_smooth_beta",          vel_beta,       0.02);
        nh_private_.param<double>("vel_smooth_d_cutoff_hz",   vel_d_cutoff,   0.5);
        vel_filter_x_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_y_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_z_   = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        vel_filter_yaw_ = multi_uav_strike::OneEuroFilter1D(vel_min_cutoff, vel_beta, vel_d_cutoff);
        // === 2026-08-04: target 输入 OneEuro (Layer 5) ===
        //   比速度通道更激进(1Hz 截止,β=0.005) — target 是被动态调度的源,不是输出,
        //   过度平滑会让"目标切换/位置突变"延迟;但 STRIKE 中目标相对静止,
        //   把 0.05-0.1m 残余抖到稳态后,后续 0.5s 内基本无变化,极强 LPF 即可。
        target_filter_x_ = multi_uav_strike::OneEuroFilter1D(1.0, 0.005, 1.0);
        target_filter_y_ = multi_uav_strike::OneEuroFilter1D(1.0, 0.005, 1.0);
        target_filter_z_ = multi_uav_strike::OneEuroFilter1D(1.0, 0.005, 1.0);
        vel_filter_last_time_ = ros::Time();
        ROS_INFO("[Guidance] Vel cmd OneEuro: min_cutoff=%.2fHz beta=%.3f d_cutoff=%.2fHz",
                 vel_min_cutoff, vel_beta, vel_d_cutoff);

        // STRIKE模式专用参数
        nh_private_.param<double>("strike_min_altitude", strike_min_altitude_, 20.0);  // 开始下降的最小高度阈值

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

        // === 2026-08-04: STRIKE 反馈环解耦 (Layer 5) — PN 输入 OneEuro LPF ===
        //   target_estimator EMA α=0.1 后残余噪声 ~0.05m,10Hz 发布在 50Hz PN 调用
        //   帧边界产生 0.04 rad/s los_rate 阶跃 → v_lateral 单帧 ~6m/s 跳变(被 nav_ratio 限幅)。
        //   对 nx/ny/nz 加 1Hz OneEuro,稳态 std 砍 ~70%,首跳 ≤0.1s 滞后可接受。
        const double fx = target_filter_x_.filter(nx, msg->header.stamp.toSec());
        const double fy = target_filter_y_.filter(ny, msg->header.stamp.toSec());
        const double fz = target_filter_z_.filter(nz, msg->header.stamp.toSec());

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
                target_filter_x_.reset();
                target_filter_y_.reset();
                target_filter_z_.reset();
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
        // === 2026-08-04: Layer 6 — 记录最后更新时刻 ===
        last_target_update_time_ = ros::Time::now();
    }

    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_target_twist_ = *msg;
        is_target_twist_received_ = true;
    }

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

        // === 2026-08-05: 用 pose delta 估算 UAV 速度(NED),Handover 修复用 ===
        //   mavros 没订阅 velocity_local,用相邻两帧 pose 差分计算,精度够 Handover 初始化用
        if (has_prev_uav_pos_) {
            const double dt = (msg->header.stamp - prev_uav_pos_time_).toSec();
            if (dt > 0.001 && dt < 0.5) {
                last_known_uav_velocity_ned_.x = (current_uav_pose_.pose.position.x - prev_uav_pos_ned_.x) / dt;
                last_known_uav_velocity_ned_.y = (current_uav_pose_.pose.position.y - prev_uav_pos_ned_.y) / dt;
                last_known_uav_velocity_ned_.z = (current_uav_pose_.pose.position.z - prev_uav_pos_ned_.z) / dt;
            }
        }
        prev_uav_pos_ned_ = current_uav_pose_.pose.position;
        prev_uav_pos_time_ = msg->header.stamp;
        has_prev_uav_pos_ = true;
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
            ROS_WARN_THROTTLE(5.0, "[Guidance] Guidance DISABLED (SEARCH_ONLY mode)");
        }
        was_enabled_prev_ = is_enabled_;
    }

    void modeCallback(const std_msgs::String::ConstPtr& msg) {
        current_mode_ = msg->data;
        ROS_WARN_THROTTLE(5.0, "[Guidance] Mode changed to: %s", current_mode_.c_str());
    }

    void uavSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
        const double speed = static_cast<double>(msg->data);
        if (speed <= 0.0) {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Ignoring non-positive uav_speed: %.2f", speed);
            return;
        }
        // 更新本节点的速度上限(用于 applyInterUavAvoidance 和 vel cmd 限速)
        track_max_speed_ = speed;
        // 下发到当前激活的策略(目前只有 Intercept 用到)
        if (guidance_strategy_) {
            guidance_strategy_->setUavSpeed(speed);
        }
        ROS_INFO("[Guidance] uav_speed updated to %.2f m/s", speed);
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

    void guidanceTimerCallback(const ros::TimerEvent&) {
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
                vel_cmd_pub_.publish(hover_cmd);
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
                    vel_cmd_pub_.publish(vel_cmd);
                } else {
                    // STRIKE模式
                    // === 2026-08-04: STRIKE 反馈环解耦 (Layer 6) — cluster stale 软兜底 ===
                    //   target_estimated_pose 0.5s 没更新 → cluster 已 stale
                    //   强行用 last_known 位置 chase,会让 PN 输出残影速度,反馈到 body 噪声。
                    //   此处降速到 cruise 的 1/3,避免在 stale 期间把任何微小位置变化
                    //   当成新信号放大成大速度指令(已经在 Layer 3-5 平滑,但 stale 阶段
                    //   仍可能因为新一次 EMA α=0.1 后又来新位置触发完整 chase)。
                    bool cluster_stale = (last_target_pos_valid_ &&
                        (ros::Time::now() - last_target_update_time_).toSec() >
                         kClusterStaleThresholdSec);
                    if (cluster_stale && !cluster_stale_latch_) {
                        ROS_WARN_THROTTLE(1.0, "[Guidance] STRIKE cluster stale for >%.1fs, "
                                              "entering soft-fallback (reduced speed, dead-reckoning)",
                                              kClusterStaleThresholdSec);
                    }
                    cluster_stale_latch_ = cluster_stale;

                    auto* strategy = static_cast<multi_uav_strike::InterceptGuidance*>(guidance_strategy_.get());
                    auto cmd = strategy->computeCommand(
                        current_uav_pose_,
                        current_target_pose_,
                        current_target_twist_);
                    // === Layer 6: stale 期间把 radial_speed/lateral 限速到 1/3 ===
                    //   cmd.velocity 是 3D 矢量,直接按比例缩放最干净;导航方向不变,
                    //   只是"信心"降低 → 慢点飞,让真实 cluster 有时间回来或 mission_manager
                    //   走 TARGET_LOST_HOVER 接管。
                    if (cluster_stale) {
                        cmd.velocity.x() *= 0.33;
                        cmd.velocity.y() *= 0.33;
                        cmd.velocity.z() *= 0.33;
                    }
                    // InterceptGuidance 输出已经是 NED；这里不能再按 NWU 翻转 Y/Z。
                    // 旧链路的二次翻转会让 East 反向，并把目标在下方时的 D+ 下降量钳成 0。
                    vel_cmd.linear.x = cmd.velocity.x();
                    vel_cmd.linear.y = cmd.velocity.y();

                    // 基于当前 primary target 的几何 LOS pitch 做下降门控。
                    // 使用 cmd.los_pitch 而不是 gimbal 的全局主目标 LOS，避免多目标时门控错对象。
                    const double pitch = cmd.los_pitch;                              // 下负上正
                    const double pitch_threshold = 0.20;                             // rad (2026-08-02: 0.5 → 0.20 ≈ 11.5°)
                    //   远距俯冲段 los_pitch 永远 < 28.6°, 旧阈值会把 PN 的 vz 前馈
                    //   压死, UAV 在目标上方水平飞过. 0.20 rad 让 11.5° 以上的俯冲段
                    //   即开始产生下降分量, 短距浅俯冲不再被门控卡住.
                    const double transition_width = 0.1;                             // rad
                    const double max_descent_when_level = 0.0;                       // m/s, NED D+
                    const double smooth_weight = 0.5 * (std::tanh(
                        (std::fabs(pitch) - pitch_threshold) / transition_width) + 1.0);
                    double desired_z = cmd.velocity.z();                             // NED: >0 下沉
                    if (desired_z < max_descent_when_level) desired_z = max_descent_when_level;
                    vel_cmd.linear.z = smooth_weight * desired_z;
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
                    const double strike_los_kp_yaw    = 0.5;
                    const double strike_los_max_rate  = 0.6;
                    double yaw_rate = strike_los_kp_yaw * yaw_err_ctrl;
                    yaw_rate = std::max(-strike_los_max_rate, std::min(strike_los_max_rate, yaw_rate));
                    vel_cmd.angular.z = yaw_rate;

                    // 保存 pitch 门控后的 NED 指令；One Euro 也统一在 NED 下运行。
                    const geometry_msgs::Vector3 gated_velocity_ned = vel_cmd.linear;
                    evaluateStrike();
                    filterVelocityCmd(vel_cmd);

                    multi_uav_strike::GuidancePnDebug pn_dbg;
                    pn_dbg.header.stamp = ros::Time::now();
                    pn_dbg.header.frame_id = "nwu_position_ned_velocity";
                    pn_dbg.uav_position_nwu = current_uav_pose_.pose.position;
                    pn_dbg.target_position_nwu = current_target_pose_.pose.position;
                    pn_dbg.strategy_velocity_ned.x = cmd.velocity.x();
                    pn_dbg.strategy_velocity_ned.y = cmd.velocity.y();
                    pn_dbg.strategy_velocity_ned.z = cmd.velocity.z();
                    pn_dbg.gated_velocity_ned = gated_velocity_ned;
                    pn_dbg.filtered_velocity_ned = vel_cmd.linear;
                    const double rel_x = current_target_pose_.pose.position.x -
                                         current_uav_pose_.pose.position.x;
                    const double rel_y = current_target_pose_.pose.position.y -
                                         current_uav_pose_.pose.position.y;
                    const double rel_z = current_target_pose_.pose.position.z -
                                         current_uav_pose_.pose.position.z;
                    pn_dbg.horizontal_distance = std::hypot(rel_x, rel_y);
                    pn_dbg.distance = std::sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z);
                    pn_dbg.los_yaw = cmd.los_yaw;
                    pn_dbg.los_pitch = cmd.los_pitch;
                    pn_dbg.los_yaw_rate = cmd.los_yaw_rate;
                    pn_dbg.los_pitch_rate = cmd.los_pitch_rate;
                    pn_dbg.desired_yaw = cmd.desired_yaw;
                    pn_dbg.pitch_gate_weight = smooth_weight;

                    if (!use_sim_) {
                        convertVelNedToEnu(vel_cmd);
                    }
                    vel_cmd_pub_.publish(vel_cmd);
                    pn_debug_pub_.publish(pn_dbg);
                    publishInterceptPointMarker(cmd.intercept_point_nwu);
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