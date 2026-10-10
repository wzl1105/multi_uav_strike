#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <cmath>
//=============打击保护bbox=========//
#include <cctype>
//=============打击保护bbox=========//
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
    //===================manual smoke trigger====================
    ros::Subscriber rc_in_manual_smoke_sub_;     // mavros/rc/in 杆量入口（拉烟手动触发）
    ros::Subscriber manual_smoke_topic_sub_;     // /yx/cmd/trigger_smoke 话题入口（边沿触发）
    //===================manual smoke trigger====================

    // Publishers
    ros::Publisher vel_cmd_pub_;          // setpoint_velocity/cmd_vel_unstamped
    //=============打击保护bbox=========//
    ros::Publisher strike_test_position_hold_pub_;  // bbox 保护锁存后的位置设定点
    ros::ServiceClient strike_test_smoke_client_;   // 稳定悬停后一次性调用 YX 拉烟服务
    //=============打击保护bbox=========//
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
    ros::Timer manual_smoke_watchdog_timer_;  // 1Hz,只跑 manual smoke RC watchdog

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
    // === 2026-08-25: GS attack_cmd action=0 (打) 接收标志, 末段 chase gate ===
    //   missionStateCallback 同步自 mission_manager;helper 顶部 guard,
    //   TRACK 阶段 GS 未发攻击指令时不下沉 (用户口径"接收到 attackCmd 之后才要")
    bool attack_cmd_received_ = false;

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
    //=============打击保护bbox=========//
    int flight_mode_position_ = 2;
    //=============打击保护bbox=========//

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;
    std::string vel_cmd_topic_;
    //=============打击保护bbox=========//
    std::string strike_test_position_hold_topic_;
    //=============打击保护bbox=========//
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
    // === 2026-08-23: 动目标 lead-bearing 参数 ===
    //   horiz < track_horiz_min_for_yaw_track_m_ 时 bearing 用 last_valid_target_bearing_nwu_
    //   兜底,避免 atan2(0,0)=0 让 yaw 突然跳北
    double track_horiz_min_for_yaw_track_m_ = 5.0;
    double last_valid_target_bearing_nwu_   = 0.0;
    ros::Time last_valid_target_bearing_stamp_{0, 0};

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

    // === 2026-08-24: chase 重构 — v_cmd = filtered_target_vel + K_p * (lead - uav) ===
    //   旧版(径向 P-control + body-frame 钳位 + no_backward + yaw_threshold)在大
    //   yaw_err (>90°) 时 cmd_fwd → 0,chase 输出衰减到 ~3 m/s,动目标永远追不上。
    //   新版直接在 NWU 世界坐标系下做 P-control,模长 clamp 到 track_max_speed_,
    //   不切 body-frame → 大 yaw_err 时速度输出不会被刹车钳位吃掉。
    double chase_K_p_;                              // 位置 P 控制增益
    double chase_lead_min_;                         // lead_t 下限 (秒)
    double chase_lead_max_;                         // lead_t 上限 (秒)
    double chase_yaw_horiz_min_m_;                  // horiz < 此值时 yaw_rate=0 防 atan2(0,0) 跳变
    double chase_vel_filter_min_cutoff_hz_;         // 目标速度 OneEuro min cutoff (chase 函数专用)
    double chase_vel_filter_beta_;                  // 目标速度 OneEuro beta
    double chase_vel_filter_d_cutoff_hz_;           // 目标速度 OneEuro d_cutoff
    double track_strike_angle_rad_;                 // chase 打击锥半角 (从 yaml 读,默认 30°)
    double cot_strike_angle_;                       // cos/sin 缓存,锥半径 = altitude × cot(strike_angle)

    // === 2026-07-16: 速度指令平滑(One Euro Filter) ===
    // 背景:即使 mission_manager 已经平滑了 target_estimated_pose,这里 P 控制
    //   v = K_p * (tgt - uav) 还是会把任何位置残留抖动乘以增益变成速度抖动,
    //   PX4 高刚度控制下就让 UAV 上下左右晃。本节点再过一次 One Euro Filter
    //   压平速度指令,从根本上消除"速度量调"造成的视觉抖动。
    multi_uav_strike::OneEuroFilter1D vel_filter_x_;
    multi_uav_strike::OneEuroFilter1D vel_filter_y_;
    multi_uav_strike::OneEuroFilter1D vel_filter_z_;
    multi_uav_strike::OneEuroFilter1D vel_filter_yaw_;
    // === 2026-08-24: chase 重构 — 滤波目标速度 (OneEuro LPF) ===
    //   KF 输出 target_estimated_twist (vx/vy) 直接用作 chase 速度基线,
    //   再过一次 OneEuro 消除 KF 状态在 DYNAMIC/STATIC 切换和 cluster 跳变时的
    //   速度跳变(v3.x → v0 / v0 → v3.x 切换会让 UAV 突然加减速)。
    //   **解耦**:滤波只针对 chase 函数,不传给 STRIKE PN(PN 已有自己的 pn_target_filter)。
    multi_uav_strike::OneEuroFilter1D chase_target_vel_filter_x_;
    multi_uav_strike::OneEuroFilter1D chase_target_vel_filter_y_;
    // === 2026-08-24: chase OneEuro dt 缓存 — 之前传 now().toSec() 当 dt,filter 冻结 ===
    ros::Time chase_filter_last_time_;
    //===================制导修改====================
    // main PN增强使用独立目标滤波状态；不得覆盖IMAGE/GEO使用的原始目标坐标。
    multi_uav_strike::OneEuroFilter1D pn_target_filter_x_;
    multi_uav_strike::OneEuroFilter1D pn_target_filter_y_;
    multi_uav_strike::OneEuroFilter1D pn_target_filter_z_;
    // === 2026-08-25: PN OneEuro dt 缓存 — 之前传 msg->header.stamp.toSec() (~1.7e9) 当 dt,
    //   filter 内部 α = 1/(1+te/τ) ≈ 1e-10,filter 完全冻在 first input。
    //   模板照搬 chase_filter_last_time_ (L191),callback 顶部算一次 dt 共用 6 次 filter call。
    ros::Time pn_filter_last_time_;
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
    // === 2026-08-27: 末段水平速度缩放下限 (用户口径"水平也要按距离衰减") ===
    //   feasibleHorizontalSpeedForDescent 在 horiz≤strike_terminal_world_direction_radius_
    //   (30m) 时按 horiz/30 缩放水平速度,floor=此值 * requested_speed,腾电机余量给
    //   垂直下降。0.4 = 末段最多用 40% 电机预算给水平,留 60% 给 vz。
    //   命中圈 (≤strike_distance_threshold_) 已在函数顶部 early-return,这里不再叠加。
    double strike_terminal_horizontal_scale_floor_  = 0.4;
    // === 2026-08-28: 删除 track_terminal_image_chase_* 四套独立参数 ===
    //   用户口径"tracking 和 strike 用同样参数",统一用 strike_terminal_image_chase_*。
    //   mode-aware 选择器 (helper L1764+) 现在也直接选 strike_* 这一套,
    //   未来要再独立调参再加回。代码里 track_* 字段全部清掉。
    // (track_terminal_image_chase_{enabled/min_horiz_m/latch_sec/center_band} 已删除)
    bool   terminal_image_chase_active_              = false;
    ros::Time terminal_image_chase_enter_time_;
    // chase enter/exit 边沿日志 (2026-08-20),last=true/false 翻转时打 WARN
    bool   last_chase_log_active_                    = false;
    //=====没有图像不打击=====//
    // 末段进入图像导引区后，如果当前没有新鲜图像，则锁存当前位置悬停。
    // 该保护与 bbox/高度打击保护完全独立，绝不触发拉烟服务。
    bool terminal_no_image_hold_enabled_ = true;
    bool terminal_no_image_hold_latched_ = false;
    geometry_msgs::PoseStamped terminal_no_image_hold_pose_;
    //=====没有图像不打击=====//
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

    //=============打击保护bbox=========//
    // 测试保护使用归一化 bbox 等效边长 sqrt(width * height)，与图像分辨率无关。
    // bbox 连续达到 hover 阈值，或本地高度低于安全阈值时，直接锁存位置悬停。
    bool strike_test_guard_enabled_ = false;
    double strike_test_guard_arm_distance_m_ = 15.0;
    double strike_test_bbox_hover_threshold_ = 0.18;
    double strike_test_bbox_person_threshold_ = 0.11;
    double strike_test_bbox_vehicle_threshold_ = 0.30;
    double strike_test_bbox_trigger_max_altitude_m_ = 10.0;
    std::string strike_test_bbox_active_class_ = "default";
    double strike_test_bbox_active_threshold_ = 0.18;
    bool strike_test_altitude_guard_enabled_ = true;
    double strike_test_min_altitude_m_ = 4.0;
    double strike_test_bbox_center_band_ = 0.25;
    double strike_test_bbox_min_confidence_ = 0.70;
    int strike_test_bbox_confirm_frames_ = 5;
    double strike_test_bbox_timeout_sec_ = 0.4;
    double strike_test_bbox_filter_alpha_ = 0.4;
    double strike_test_bbox_metric_ = 0.0;
    bool strike_test_bbox_metric_valid_ = false;
    int strike_test_bbox_hover_confirm_count_ = 0;
    bool strike_test_bbox_guard_armed_ = false;
    bool strike_test_bbox_hover_latched_ = false;
    ros::Time strike_test_bbox_last_update_time_;
    geometry_msgs::PoseStamped strike_test_position_hold_pose_;
    bool strike_test_position_hold_latched_ = false;
    geometry_msgs::Point strike_test_hold_position_nwu_;
    bool strike_test_smoke_enabled_ = false;
    std::string strike_test_smoke_service_ = "/trigger_smoke";
    bool strike_test_smoke_request_attempted_ = false;
    //=============打击保护bbox=========//
    //=============manual smoke trigger (rc_in + topic)================
    bool manual_smoke_rc_enabled_ = false;
    int manual_smoke_rc_channel_ = 9;
    int manual_smoke_rc_on_pwm_ = 1600;
    int manual_smoke_rc_off_pwm_ = 1700;
    double manual_smoke_rc_timeout_sec_ = 1.0;
    bool manual_smoke_topic_enabled_ = true;
    std::string manual_smoke_topic_ = "/yx/cmd/trigger_smoke";
    // === 2026-10-10: 地面手动测试用,默认 false (安全),现场测试时 launch 临时设 true ===
    // 现场需要先卸保险再地面手动测拉烟 (disarm 状态),bypass armed gate 让 topic/RC
    // 在 disarm 时也能触发。注意:设了 true 之后地面开机就要小心,任何 True 边沿都
    // 会直接向 YX 板发 0x4E,测完务必改回 false。
    bool manual_smoke_bypass_armed_gate_ = false;
    // 边沿 latch:上一次 topic 触发时间,用于去抖,以及最近一次成功 service call
    ros::Time last_manual_smoke_topic_time_;
    ros::Time last_manual_smoke_call_time_;
    ros::Time last_manual_smoke_rc_in_time_;
    // 是否已"激活"——topic 模式下,只对每个 True 边沿触发一次
    bool manual_smoke_topic_active_ = false;
    //=============manual smoke trigger (rc_in + topic)================

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

        // 手动拉烟 watchdog:独立 1Hz timer,不挂在 50Hz 控制循环里
        manual_smoke_watchdog_timer_ = nh_.createTimer(
            ros::Duration(1.0),
            &GuidanceControlNode::manualSmokeWatchdogCallback,
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
            //=============打击保护bbox=========//
            strike_test_position_hold_topic_ = "quad/setpoint_position/local";
            //=============打击保护bbox=========//
            attitude_cmd_topic_ = "quad/setpoint_attitude/attitude";
            attitude_rates_topic_ = "quad/setpoint_attitude/cmd_vel";
            thrust_cmd_topic_ = "quad/thrust";
        } else {
            pose_topic_ = "mavros/local_position/pose";
            vel_cmd_topic_ = "mavros/setpoint_velocity/cmd_vel_unstamped";
            //=============打击保护bbox=========//
            strike_test_position_hold_topic_ = "mavros/setpoint_position/local";
            //=============打击保护bbox=========//
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
        //=============打击保护bbox=========//
        nh_private_.param<int>("flight_mode_position", flight_mode_position_, 2);
        nh_private_.param<std::string>("strike_test_position_hold_topic",
                                      strike_test_position_hold_topic_,
                                      strike_test_position_hold_topic_);
        //=============打击保护bbox=========//
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<bool>("enable_strike_evaluation",  enable_strike_evaluation_,  false);  // strike_eval 总开关(默认关闭,见字段注释)

        // TRACK模式专用参数
        nh_private_.param<double>("track_altitude", track_altitude_, 30.0);       // 固定跟踪高度
        nh_private_.param<double>("track_horiz_dist", track_horiz_dist_, 8.0);    // 水平安全距离（仅低空 fallback）
        nh_private_.param<double>("track_P", track_P_, 1.0);                     // P控制增益
        nh_private_.param<double>("track_I", track_I_, 0.1);                     // I控制增益
        // === 2026-08-23: 0.5→20.0,动目标 chase 需要 UAV 速度能追上 (≤17 m/s + 余量) ===
        //   **这是 chase 模式最大速度上限 (CAP),不是 chase 速度本身**。
        //   实际 chase 速度由 computeTrackVelocity 的 P-control + KF lead_pose 自然计算:
        //     - 目标动 → KF lead_pose 让 horiz_dist 偏大 → chase 加速
        //     - 目标静 → horiz_dist 趋向 stand-off → chase 减速到悬停
        //   设 20 m/s 是给 17 m/s 动目标留 18% 余量。
        nh_private_.param<double>("uav_speed", track_max_speed_, 20.0);     // 最大速度限制 (m/s)
        // 距离→速度的连续斜率（消除内/外边界的硬切换：每超出 stand-off 1m 速度增加多少 m/s）
        nh_private_.param<double>("track_k_approach", track_k_approach_, 0.5);
        // === 2026-08-23: 动目标 chase lead-bearing 兜底阈值 ===
        //   horiz_dist < 此值时 bearing 用 last_valid_target_bearing_nwu_ (sticky),
        //   避免 atan2(0,0)=0 让 yaw 突然跳北
        nh_private_.param<double>("track_horiz_min_for_yaw_track_m",
                                  track_horiz_min_for_yaw_track_m_, 5.0);

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

        // === 2026-08-24: chase 重构 — 目标速度 OneEuro + chase K_p / lead / yaw 参数 ===
        //   chase_target_vel_filter 与 vel_filter 分开,因为 chase 函数需要独立调参
        //   (默认 1Hz/0.005/1Hz 比 vel_filter 更跟手,因为 chase 速度基线要响应 KF 变化)。
        double chase_vel_min_cutoff, chase_vel_beta, chase_vel_d_cutoff;
        nh_private_.param<double>("chase_vel_filter_min_cutoff_hz",
                                  chase_vel_min_cutoff, 1.0);
        nh_private_.param<double>("chase_vel_filter_beta",
                                  chase_vel_beta, 0.005);
        nh_private_.param<double>("chase_vel_filter_d_cutoff_hz",
                                  chase_vel_d_cutoff, 1.0);
        chase_target_vel_filter_x_ = multi_uav_strike::OneEuroFilter1D(
            chase_vel_min_cutoff, chase_vel_beta, chase_vel_d_cutoff);
        chase_target_vel_filter_y_ = multi_uav_strike::OneEuroFilter1D(
            chase_vel_min_cutoff, chase_vel_beta, chase_vel_d_cutoff);
        chase_vel_filter_min_cutoff_hz_ = chase_vel_min_cutoff;
        chase_vel_filter_beta_           = chase_vel_beta;
        chase_vel_filter_d_cutoff_hz_    = chase_vel_d_cutoff;
        nh_private_.param<double>("chase_K_p", chase_K_p_, 0.5);
        nh_private_.param<double>("chase_lead_min", chase_lead_min_, 0.2);
        nh_private_.param<double>("chase_lead_max", chase_lead_max_, 1.5);
        // === 2026-08-24: chase 打击锥半角 — 锥外 P-control + 锥内纯速度 ===
        //   锥半径 = altitude × cot(strike_angle),30m 高 + 30° 角 → 半径 52m。
        //   UAV 在锥内不再被 P-control 拉到目标正上方,只跟目标速度;锥外才
        //   朝目标方向收 excess = horiz - tolerance。
        double strike_angle_deg;
        nh_private_.param<double>("track_strike_angle_deg", strike_angle_deg, 30.0);
        track_strike_angle_rad_ = strike_angle_deg * M_PI / 180.0;
        cot_strike_angle_ = std::cos(track_strike_angle_rad_) /
                            std::sin(track_strike_angle_rad_);
        nh_private_.param<double>("chase_yaw_horiz_min_m",
                                  chase_yaw_horiz_min_m_, 1.0);
        chase_K_p_             = std::max(0.0, chase_K_p_);
        chase_lead_min_        = std::max(0.0, chase_lead_min_);
        chase_lead_max_        = std::max(chase_lead_min_ + 0.05, chase_lead_max_);
        chase_yaw_horiz_min_m_ = std::max(0.0, chase_yaw_horiz_min_m_);
        ROS_INFO("[Guidance] Chase vel LPF: min_cutoff=%.2fHz beta=%.3f d_cutoff=%.2fHz | "
                 "K_p=%.2f lead=[%.2f,%.2f]s yaw_horiz_min=%.1fm",
                 chase_vel_min_cutoff, chase_vel_beta, chase_vel_d_cutoff,
                 chase_K_p_, chase_lead_min_, chase_lead_max_, chase_yaw_horiz_min_m_);

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
        //=====没有图像不打击=====//
        nh_private_.param<bool>("strike_terminal_no_image_hold_enabled",
                                terminal_no_image_hold_enabled_, true);
        //=====没有图像不打击=====//
        strike_terminal_image_chase_min_horiz_m_ =
            std::max(strike_terminal_image_chase_min_horiz_m_, 0.0);
        strike_terminal_image_chase_latch_sec_ =
            std::max(strike_terminal_image_chase_latch_sec_, 0.0);
        strike_terminal_image_chase_center_band_ =
            std::max(strike_terminal_image_chase_center_band_, image_center_deadband_);

        //=============打击保护bbox=========//
        nh_private_.param<bool>("strike_test_guard_enabled",
                                strike_test_guard_enabled_, false);
        nh_private_.param<double>("strike_test_guard_arm_distance_m",
                                  strike_test_guard_arm_distance_m_, 15.0);
        nh_private_.param<double>("strike_test_bbox_hover_threshold",
                                  strike_test_bbox_hover_threshold_, 0.18);
        nh_private_.param<double>("strike_test_bbox_person_threshold",
                                  strike_test_bbox_person_threshold_, 0.11);
        nh_private_.param<double>("strike_test_bbox_vehicle_threshold",
                                  strike_test_bbox_vehicle_threshold_, 0.30);
        nh_private_.param<double>("strike_test_bbox_trigger_max_altitude_m",
                                  strike_test_bbox_trigger_max_altitude_m_, 10.0);
        nh_private_.param<bool>("strike_test_altitude_guard_enabled",
                                strike_test_altitude_guard_enabled_, true);
        nh_private_.param<double>("strike_test_min_altitude_m",
                                  strike_test_min_altitude_m_, 4.0);
        nh_private_.param<bool>("strike_test_smoke_enabled",
                                strike_test_smoke_enabled_, false);
        nh_private_.param<std::string>("strike_test_smoke_service",
                                       strike_test_smoke_service_, "/trigger_smoke");
        // === 手动拉烟(rc_in 杆量 + topic 双入口,与 strike_test_smoke 自动流程并行)===
        // 现场反馈:目前 strike_test_smoke 偶尔不触发(可能是稳定条件过严或硬件),
        //   加手动兜底,任意 arm 状态下都可由用户主动调一次。
        // 杆量迟滞阈值与 manual_recognition 同一套路,topic 边沿触发,两路可独立开关。
        nh_private_.param<bool>("manual_smoke_rc_enabled",
                                manual_smoke_rc_enabled_, false);
        nh_private_.param<int>("manual_smoke_rc_channel",
                               manual_smoke_rc_channel_, 9);
        nh_private_.param<int>("manual_smoke_rc_on_pwm",
                               manual_smoke_rc_on_pwm_, 1600);
        nh_private_.param<int>("manual_smoke_rc_off_pwm",
                               manual_smoke_rc_off_pwm_, 1700);
        nh_private_.param<double>("manual_smoke_rc_timeout_sec",
                                  manual_smoke_rc_timeout_sec_, 1.0);
        nh_private_.param<bool>("manual_smoke_topic_enabled",
                                manual_smoke_topic_enabled_, true);
        nh_private_.param<std::string>("manual_smoke_topic",
                                       manual_smoke_topic_, "/yx/cmd/trigger_smoke");
        nh_private_.param<bool>("manual_smoke_bypass_armed_gate",
                                manual_smoke_bypass_armed_gate_, false);
        nh_private_.param<double>("strike_test_bbox_center_band",
                                  strike_test_bbox_center_band_, 0.25);
        nh_private_.param<double>("strike_test_bbox_min_confidence",
                                  strike_test_bbox_min_confidence_, 0.70);
        nh_private_.param<int>("strike_test_bbox_confirm_frames",
                               strike_test_bbox_confirm_frames_, 5);
        nh_private_.param<double>("strike_test_bbox_timeout_sec",
                                  strike_test_bbox_timeout_sec_, 0.4);
        nh_private_.param<double>("strike_test_bbox_filter_alpha",
                                  strike_test_bbox_filter_alpha_, 0.4);
        strike_test_guard_arm_distance_m_ = std::max(
            0.1, strike_test_guard_arm_distance_m_);
        strike_test_bbox_hover_threshold_ = clampValue(
            strike_test_bbox_hover_threshold_, 0.001, 1.0);
        strike_test_bbox_person_threshold_ = clampValue(
            strike_test_bbox_person_threshold_, 0.001, 1.0);
        strike_test_bbox_vehicle_threshold_ = clampValue(
            strike_test_bbox_vehicle_threshold_, 0.001, 1.0);
        strike_test_bbox_trigger_max_altitude_m_ = std::max(
            strike_test_min_altitude_m_, strike_test_bbox_trigger_max_altitude_m_);
        strike_test_bbox_active_threshold_ = strike_test_bbox_hover_threshold_;
        strike_test_min_altitude_m_ = std::max(0.0, strike_test_min_altitude_m_);
        strike_test_bbox_confirm_frames_ = std::max(1, strike_test_bbox_confirm_frames_);
        strike_test_bbox_timeout_sec_ = std::max(0.05, strike_test_bbox_timeout_sec_);
        strike_test_bbox_filter_alpha_ = clampValue(
            strike_test_bbox_filter_alpha_, 0.01, 1.0);
        strike_test_bbox_center_band_ = clampValue(
            strike_test_bbox_center_band_, 0.0, 0.5);
        strike_test_bbox_min_confidence_ = clampValue(
            strike_test_bbox_min_confidence_, 0.0, 1.0);
        if (strike_test_guard_enabled_) {
            ROS_WARN("[STRIKE TEST GUARD] ENABLED: arm_distance=%.1fm "
                     "bbox person=%.3f vehicle=%.3f default=%.3f trigger_altitude<=%.2fm "
                     "center=+/-%.2f confidence>=%.2f "
                     "altitude_guard=%d local_altitude<=%.2fm "
                     "confirm=%d timeout=%.2fs smoke=%d service=%s; real impact is inhibited",
                     strike_test_guard_arm_distance_m_,
                     strike_test_bbox_person_threshold_,
                     strike_test_bbox_vehicle_threshold_,
                     strike_test_bbox_hover_threshold_,
                     strike_test_bbox_trigger_max_altitude_m_,
                     strike_test_bbox_center_band_,
                     strike_test_bbox_min_confidence_,
                     static_cast<int>(strike_test_altitude_guard_enabled_),
                     strike_test_min_altitude_m_,
                     strike_test_bbox_confirm_frames_,
                     strike_test_bbox_timeout_sec_,
                     static_cast<int>(strike_test_smoke_enabled_),
                     strike_test_smoke_service_.c_str());
        } else {
            ROS_WARN("[STRIKE TEST GUARD] DISABLED: original strike command path is unchanged");
        }
        //=============打击保护bbox=========//
        // === 手动拉烟启动回显(两路入口 + 服务名)===
        ROS_WARN("[MANUAL SMOKE] rc_enabled=%d ch=%d on<=%d off>=%d timeout=%.1fs | "
                 "topic_enabled=%d topic=%s | service=%s (armed-only, see tryManualSmokeTrigger)",
                 static_cast<int>(manual_smoke_rc_enabled_),
                 manual_smoke_rc_channel_,
                 manual_smoke_rc_on_pwm_, manual_smoke_rc_off_pwm_,
                 manual_smoke_rc_timeout_sec_,
                 static_cast<int>(manual_smoke_topic_enabled_),
                 manual_smoke_topic_.c_str(),
                 strike_test_smoke_service_.c_str());

        // === 2026-08-27: 末段水平速度缩放下限 (腾电机余量给垂直) ===
        nh_private_.param<double>("strike_terminal_horizontal_scale_floor",
                                  strike_terminal_horizontal_scale_floor_, 0.4);
        strike_terminal_horizontal_scale_floor_ =
            clampValue(strike_terminal_horizontal_scale_floor_, 0.05, 1.0);

        // === 2026-08-28: 删除 track_terminal_image_chase_* 参数加载 ===
        //   TRACK/STRIKE 共享 strike_* 一组参数,这里不再 load 4 个 track_* 字段。
        // (2026-08-25 plan v7 引入的 4 个独立参数已合并到 strike_*)
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

            // 手动拉烟两路入口
            //   - rc_in 杆量(低电平触发,迟滞阈值 yaml 可调)
            //   - /yx/cmd/trigger_smoke 边沿触发(std_msgs/Bool, true 即触发)
            //  两条入口互不影响,可独立开关。
            rc_in_manual_smoke_sub_ = nh_.subscribe(
                "mavros/rc/in", 10,
                &GuidanceControlNode::rcInManualSmokeCallback, this);
            manual_smoke_topic_sub_ = nh_.subscribe(
                manual_smoke_topic_, 10,
                &GuidanceControlNode::manualSmokeTopicCallback, this);
        } else {
            // 仿真也接 topic 入口,方便 SIL/HITL 联调
            manual_smoke_topic_sub_ = nh_.subscribe(
                manual_smoke_topic_, 10,
                &GuidanceControlNode::manualSmokeTopicCallback, this);
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
        //=============打击保护bbox=========//
        strike_test_position_hold_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            strike_test_position_hold_topic_, 10);
        strike_test_smoke_client_ = nh_.serviceClient<std_srvs::Trigger>(
            strike_test_smoke_service_);
        //=============打击保护bbox=========//
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
        // === 2026-08-25: dt 修复 — 之前传 msg->header.stamp.toSec() (~1.7e9) 当 dt,
        //   filter 内部 α = 1/(1+te/τ) ≈ 1e-10 完全冻结。改用 callback 顶部算的 dt。
        //   callback 顶部一次算 dt,后面 6 次 filter call 共用 (L813-815 + L878-880 + L952-957)。
        const ros::Time pn_filter_now = ros::Time::now();
        double pn_filter_dt = pn_filter_last_time_.isZero()
                                  ? (1.0 / 10.0)  // target_estimated_pose 10Hz
                                  : (pn_filter_now - pn_filter_last_time_).toSec();
        pn_filter_last_time_ = pn_filter_now;
        if (pn_filter_dt <= 0.0) pn_filter_dt = 1e-3;
        if (pn_filter_dt > 1.0)  pn_filter_dt = 1.0;
        const double fx = pn_target_filter_x_.filter(nx, pn_filter_dt);
        const double fy = pn_target_filter_y_.filter(ny, pn_filter_dt);
        const double fz = pn_target_filter_z_.filter(nz, pn_filter_dt);

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
        // === 2026-08-25: dt 修复 — 与 L813-815 共用 callback 顶部 pn_filter_dt ===
        pn_filtered_target_pose_.pose.position.x = pn_target_filter_x_.filter(nx, pn_filter_dt);
        pn_filtered_target_pose_.pose.position.y = pn_target_filter_y_.filter(ny, pn_filter_dt);
        pn_filtered_target_pose_.pose.position.z = pn_target_filter_z_.filter(nz, pn_filter_dt);
        pn_filtered_target_valid_ = true;
        pn_last_target_update_time_ = ros::Time::now();
        //===================制导修改====================
    }

    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_target_twist_ = *msg;
        is_target_twist_received_ = true;
    }

    //=============打击保护bbox=========//
    std::string strikeTestBboxClass(const std::string& label) const {
        std::string normalized = label;
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (normalized == "person" || normalized == "pedestrian") {
            return "person";
        }
        if (normalized == "car" || normalized == "truck" ||
            normalized == "bus" || normalized == "vehicle" ||
            normalized == "van" || normalized == "suv") {
            return "vehicle";
        }
        return "default";
    }

    double strikeTestBboxThreshold(const std::string& target_class) const {
        if (target_class == "person") {
            return strike_test_bbox_person_threshold_;
        }
        if (target_class == "vehicle") {
            return strike_test_bbox_vehicle_threshold_;
        }
        return strike_test_bbox_hover_threshold_;
    }
    //=============打击保护bbox=========//

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

            //=============打击保护bbox=========//
            const bool bbox_centered =
                std::fabs(static_cast<double>(msg->center_x) - 0.5) <=
                    strike_test_bbox_center_band_ &&
                std::fabs(static_cast<double>(msg->center_y) - 0.5) <=
                    strike_test_bbox_center_band_;
            const bool bbox_qualified = strike_test_bbox_guard_armed_ &&
                msg->visible && bbox_centered &&
                msg->confidence >= strike_test_bbox_min_confidence_;
            if (strike_test_guard_enabled_ && bbox_qualified &&
                std::isfinite(msg->width) && std::isfinite(msg->height) &&
                msg->width > 0.0 && msg->height > 0.0) {
                const std::string target_class = strikeTestBboxClass(msg->label);
                if (target_class != strike_test_bbox_active_class_) {
                    strike_test_bbox_active_class_ = target_class;
                    strike_test_bbox_active_threshold_ =
                        strikeTestBboxThreshold(target_class);
                    // 类别发生变化后重新滤波并重新累计，禁止低阈值跨类别继承计数。
                    strike_test_bbox_metric_ = 0.0;
                    strike_test_bbox_metric_valid_ = false;
                    strike_test_bbox_hover_confirm_count_ = 0;
                    ROS_WARN("[STRIKE TEST GUARD] bbox class=%s label=%s threshold=%.3f",
                             strike_test_bbox_active_class_.c_str(), msg->label.c_str(),
                             strike_test_bbox_active_threshold_);
                }
                const double bbox_metric = std::sqrt(
                    static_cast<double>(msg->width) *
                    static_cast<double>(msg->height));
                if (!strike_test_bbox_metric_valid_) {
                    strike_test_bbox_metric_ = bbox_metric;
                    strike_test_bbox_metric_valid_ = true;
                } else {
                    strike_test_bbox_metric_ =
                        strike_test_bbox_filter_alpha_ * bbox_metric +
                        (1.0 - strike_test_bbox_filter_alpha_) *
                            strike_test_bbox_metric_;
                }
                strike_test_bbox_last_update_time_ = ros::Time::now();
                const bool trigger_altitude_ok = is_uav_pose_received_ &&
                    std::isfinite(current_uav_pose_.pose.position.z) &&
                    current_uav_pose_.pose.position.z <=
                        strike_test_bbox_trigger_max_altitude_m_;
                if (trigger_altitude_ok &&
                    strike_test_bbox_metric_ >= strike_test_bbox_active_threshold_) {
                    strike_test_bbox_hover_confirm_count_ = std::min(
                        strike_test_bbox_hover_confirm_count_ + 1,
                        strike_test_bbox_confirm_frames_);
                } else {
                    strike_test_bbox_hover_confirm_count_ = 0;
                }
            } else if (strike_test_guard_enabled_ && strike_test_bbox_guard_armed_) {
                strike_test_bbox_hover_confirm_count_ = 0;
            }
            //=============打击保护bbox=========//

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

        // === 2026-08-25: dt 修复 — 之前传 absolute timestamp,filter 冻结。
        //   wp2_fallback_pose 是 mission_manager Attack(105) WP2 fallback 路径,
        //   publish rate 不固定,fallback 用 0.1s (≈ 10Hz target_estimated_pose) 兜底。
        const ros::Time pn_filter_now = ros::Time::now();
        double pn_filter_dt = pn_filter_last_time_.isZero()
                                  ? (1.0 / 10.0)
                                  : (pn_filter_now - pn_filter_last_time_).toSec();
        pn_filter_last_time_ = pn_filter_now;
        if (pn_filter_dt <= 0.0) pn_filter_dt = 1e-3;
        if (pn_filter_dt > 1.0)  pn_filter_dt = 1.0;

        // === PN 路径:经 OneEuro LPF (避免 cold filter 首帧 0, A1/A5) ===
        pn_filtered_target_pose_ = *msg;
        pn_filtered_target_pose_.pose.position.x =
            pn_target_filter_x_.filter(msg->pose.position.x, pn_filter_dt);
        pn_filtered_target_pose_.pose.position.y =
            pn_target_filter_y_.filter(msg->pose.position.y, pn_filter_dt);
        pn_filtered_target_pose_.pose.position.z =
            pn_target_filter_z_.filter(msg->pose.position.z, pn_filter_dt);
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
            //=====没有图像不打击=====//
            terminal_no_image_hold_latched_ = false;
            terminal_no_image_hold_pose_ = geometry_msgs::PoseStamped();
            //=====没有图像不打击=====//

            //=============打击保护bbox=========//
            strike_test_bbox_metric_ = 0.0;
            strike_test_bbox_metric_valid_ = false;
            strike_test_bbox_hover_confirm_count_ = 0;
            strike_test_bbox_guard_armed_ = false;
            strike_test_bbox_hover_latched_ = false;
            strike_test_bbox_last_update_time_ = ros::Time();
            strike_test_position_hold_latched_ = false;
            strike_test_position_hold_pose_ = geometry_msgs::PoseStamped();


            strike_test_smoke_request_attempted_ = false;
            //=============打击保护bbox=========//
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
            //=====没有图像不打击=====//
            terminal_no_image_hold_latched_ = false;
            terminal_no_image_hold_pose_ = geometry_msgs::PoseStamped();
            //=====没有图像不打击=====//

            //=============打击保护bbox=========//
            strike_test_bbox_metric_ = 0.0;
            strike_test_bbox_metric_valid_ = false;
            strike_test_bbox_hover_confirm_count_ = 0;
            strike_test_bbox_guard_armed_ = false;
            strike_test_bbox_hover_latched_ = false;
            strike_test_bbox_last_update_time_ = ros::Time();
            strike_test_position_hold_latched_ = false;
            strike_test_position_hold_pose_ = geometry_msgs::PoseStamped();


            strike_test_smoke_request_attempted_ = false;
            //=============打击保护bbox=========//
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
            //=====没有图像不打击=====//
            terminal_no_image_hold_latched_ = false;
            terminal_no_image_hold_pose_ = geometry_msgs::PoseStamped();
            //=====没有图像不打击=====//

            //=============打击保护bbox=========//
            strike_test_bbox_metric_ = 0.0;
            strike_test_bbox_metric_valid_ = false;
            strike_test_bbox_hover_confirm_count_ = 0;
            strike_test_bbox_guard_armed_ = false;
            strike_test_bbox_hover_latched_ = false;
            strike_test_bbox_last_update_time_ = ros::Time();
            strike_test_position_hold_latched_ = false;
            strike_test_position_hold_pose_ = geometry_msgs::PoseStamped();


            strike_test_smoke_request_attempted_ = false;
            //=============打击保护bbox=========//
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

    //===================manual smoke trigger (rc_in + topic)================
    /**
     * rc_in 杆量入口:低电平(<= on_pwm)触发一次拉烟,高电平(>= off_pwm)允许再次触发。
     * 迟滞避免 PWM 抖动反复开/关。RC 消息丢失超过 timeout_sec 不再触发。
     * 与 mission_manager manual_recognition 同一套路,但武装门由本节点 mavros_armed_ 把控。
     */
    void rcInManualSmokeCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        if (!manual_smoke_rc_enabled_) return;
        last_manual_smoke_rc_in_time_ = ros::Time::now();
        const size_t index = static_cast<size_t>(manual_smoke_rc_channel_ - 1);
        if (index >= msg->channels.size()) {
            ROS_ERROR_THROTTLE(2.0,
                "[MANUAL SMOKE] RC channel CH%d unavailable: RCIn has %zu channels",
                manual_smoke_rc_channel_, msg->channels.size());
            return;
        }
        const int pwm = static_cast<int>(msg->channels[index]);
        if (pwm <= manual_smoke_rc_on_pwm_) {
            tryManualSmokeTrigger("RC low", pwm);
        } else if (pwm >= manual_smoke_rc_off_pwm_) {
            // 显式归零 latch:无 latch 状态机,只要再次拉低就再触发一次
            ROS_INFO_THROTTLE(5.0, "[MANUAL SMOKE] RC high pwm=%d, ready for next trigger", pwm);
        }
    }

    /**
     * /yx/cmd/trigger_smoke 边沿触发:每个 True 边沿触发一次拉烟。
     * 持续 True 不会反复触发(用 manual_smoke_topic_active_ latch)。
     * 可用于 GS 主动调触发按钮,或后续接外部硬件。
     */
    void manualSmokeTopicCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!manual_smoke_topic_enabled_) return;
        if (msg->data && !manual_smoke_topic_active_) {
            tryManualSmokeTrigger("topic edge", -1);
        }
        manual_smoke_topic_active_ = msg->data;
    }

    /**
     * 统一触发入口:调 /trigger_smoke 之前做"已武装"门控(防止地面误碰),
     * 加 200ms 最小间隔防连发,失败 200ms 后重试最多 3 次(覆盖 RS-485 SENDING
     * 短窗 + GPIO 瞬时未就绪等一次性失败)。
     * 与自动 strike_test_smoke_ 互不影响(各自走自己的 _attempted_ 状态)。
     */
    void tryManualSmokeTrigger(const char* reason, int pwm) {
        // === 门 1:必须 armed(mavros state 还没收到时也算不通过) ===
        //   现场地面手动测时需要 disarm 也能触发,设 manual_smoke_bypass_armed_gate=true 绕过。
        //   测完务必改回 false (yaml 默认)。
        if (!manual_smoke_bypass_armed_gate_ &&
            (!mavros_state_received_ || !mavros_armed_)) {
            ROS_WARN_THROTTLE(2.0,
                "[MANUAL SMOKE] REJECT (%s pwm=%d): UAV not armed (mavros_state_received=%d armed=%d bypass=%d). "
                "Manual trigger is in-flight only; arm the UAV first "
                "(or set manual_smoke_bypass_armed_gate=true for ground test).",
                reason, pwm,
                static_cast<int>(mavros_state_received_),
                static_cast<int>(mavros_armed_),
                static_cast<int>(manual_smoke_bypass_armed_gate_));
            return;
        }
        // === 门 2:200ms 最小间隔(防 RC 抖动 + topic 边沿重发) ===
        const ros::Time now = ros::Time::now();
        if (!last_manual_smoke_call_time_.isZero() &&
            (now - last_manual_smoke_call_time_).toSec() < 0.2) {
            ROS_WARN_THROTTLE(2.0,
                "[MANUAL SMOKE] REJECT (%s pwm=%d): within 200ms of last call (%.2fs ago)",
                reason, pwm,
                (now - last_manual_smoke_call_time_).toSec());
            return;
        }
        // === service 可达性 ===
        if (!strike_test_smoke_client_.exists() &&
            !strike_test_smoke_client_.waitForExistence(ros::Duration(0.2))) {
            ROS_ERROR_THROTTLE(2.0,
                "[MANUAL SMOKE] FAIL (%s pwm=%d): service %s not available",
                reason, pwm, strike_test_smoke_service_.c_str());
            return;
        }

        // === 失败重试:最多 3 次,每次间隔 200ms ===
        //   - 成功:立刻返回,更新 last_manual_smoke_call_time_
        //   - 失败:ROS_WARN 提示,200ms 后再调一次;超过 3 次彻底放弃
        // 一次性的 RS-485 SENDING 短窗 / GPIO 瞬时未就绪 会被这次重试覆盖掉
        constexpr int kMaxRetries = 3;            // 总共 3 次
        constexpr double kRetryIntervalSec = 0.2; // 间隔 200ms
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            std_srvs::Trigger srv;
            const bool rpc_ok = strike_test_smoke_client_.call(srv);
            last_manual_smoke_call_time_ = ros::Time::now();
            if (rpc_ok && srv.response.success) {
                ROS_WARN("[MANUAL SMOKE] >>>> TRIGGERED (%s pwm=%d) attempt=%d: %s",
                         reason, pwm, attempt, srv.response.message.c_str());
                return;
            }
            ROS_WARN("[MANUAL SMOKE] attempt %d/%d %s (%s pwm=%d): %s",
                     attempt, kMaxRetries,
                     rpc_ok ? "service rejected" : "RPC error",
                     reason, pwm,
                     rpc_ok ? srv.response.message.c_str() : "n/a");
            if (attempt < kMaxRetries) {
                ros::Duration(kRetryIntervalSec).sleep();
            }
        }
        ROS_ERROR("[MANUAL SMOKE] GIVE UP (%s pwm=%d): %d attempts failed in %.2fs",
                  reason, pwm, kMaxRetries, kMaxRetries * kRetryIntervalSec);
    }

    /**
     * RC 输入 watchdog:超 1.0s 没消息 + 杆量拉低过,自动清 latch(虽然无 latch 状态机,
     * 但用户期望"长时间无 RC 输入就别响应",保留这套便于将来扩 latch)。
     * 实际触发只看每次 callback 的瞬时 PWM,不会误触发。
     * 由 manual_smoke_watchdog_timer_ 1Hz 调度。
     */
    void manualSmokeWatchdogCallback(const ros::TimerEvent&) {
        if (!manual_smoke_rc_enabled_) return;
        if (last_manual_smoke_rc_in_time_.isZero()) return;
        if ((ros::Time::now() - last_manual_smoke_rc_in_time_).toSec() >
            manual_smoke_rc_timeout_sec_) {
            // 不再清任何 latch,仅 throttled 提示
            ROS_WARN_THROTTLE(5.0,
                "[MANUAL SMOKE] RC input stale (>%.1fs), manual RC trigger ignored",
                manual_smoke_rc_timeout_sec_);
        }
    }
    //===================manual smoke trigger (rc_in + topic)================

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
        // === 2026-08-25: 同步 GS attack_cmd 接收标志 (末段 chase gate) ===
        //   用户口径"接收到 attackCmd 之后才要末段 chase" → helper 顶部 guard
        attack_cmd_received_ = (msg->attack_cmd_received != 0);
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

    //=============打击保护bbox=========//
    // 必须在 NED 指令的所有制导、terminal chase 和滤波之后调用。
    // 返回 true 表示指令已被测试保护修改；开关关闭时立即返回，不影响原路径。
    void latchStrikeTestPositionHold() {
        if (strike_test_position_hold_latched_ || !is_uav_pose_received_) {
            return;
        }

        const auto& q = current_uav_pose_.pose.orientation;
        const double yaw_nwu = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        strike_test_position_hold_pose_ = geometry_msgs::PoseStamped();
        strike_test_position_hold_pose_.header.frame_id = "map";
        double output_yaw = 0.0;
        if (use_sim_) {
            // 内部 NWU 恢复为纯仿真 quad 接口使用的 NED。
            strike_test_position_hold_pose_.pose.position.x =
                current_uav_pose_.pose.position.x;
            strike_test_position_hold_pose_.pose.position.y =
                -current_uav_pose_.pose.position.y;
            strike_test_position_hold_pose_.pose.position.z =
                -current_uav_pose_.pose.position.z;
            output_yaw = -yaw_nwu;
        } else {
            // 内部 NWU 转 MAVROS setpoint_position/local 使用的 ENU。
            strike_test_position_hold_pose_.pose.position.x =
                -current_uav_pose_.pose.position.y;
            strike_test_position_hold_pose_.pose.position.y =
                current_uav_pose_.pose.position.x;
            strike_test_position_hold_pose_.pose.position.z =
                current_uav_pose_.pose.position.z;
            output_yaw = M_PI_2 + yaw_nwu;
        }
        output_yaw = std::atan2(std::sin(output_yaw), std::cos(output_yaw));
        strike_test_position_hold_pose_.pose.orientation.z =
            std::sin(0.5 * output_yaw);
        strike_test_position_hold_pose_.pose.orientation.w =
            std::cos(0.5 * output_yaw);
        strike_test_hold_position_nwu_ = current_uav_pose_.pose.position;
        strike_test_position_hold_latched_ = true;
        ROS_ERROR("[STRIKE TEST GUARD] POSITION HOLD LATCHED: "
                  "topic=%s position=(%.2f,%.2f,%.2f) yaw=%.1fdeg",
                  strike_test_position_hold_topic_.c_str(),
                  strike_test_position_hold_pose_.pose.position.x,
                  strike_test_position_hold_pose_.pose.position.y,
                  strike_test_position_hold_pose_.pose.position.z,
                  output_yaw * 180.0 / M_PI);
    }

    void maybeTriggerStrikeTestSmoke() {
        // === 2026-10-10: 简化触发条件,位置锁存就发,不再等 0.2s 速度/位置稳定 ===
        //   原 6 个 gate 删后 3 个 (min_hold_sec / max_pos_err+max_xy/z_speed /
        //   stable_confirm_sec)。位置锁存本身已经是"已命中"语义
        //   (高度硬保护 ≤4m 或 bbox hover 持续 N 帧),不应再被 0.2s 速度等待卡住
        //   (UAV 还在减速过程 → 速度误差超阈 → 永远不触发,实测"有时不触发"主因之一)。
        if (!strike_test_smoke_enabled_ || strike_test_smoke_request_attempted_ ||
            !strike_test_guard_enabled_ || current_mode_ != "strike" ||
            !strike_test_position_hold_latched_ || !has_prev_uav_position_) {
            return;
        }

        // 2026-10-10: 失败重试 3 次,200ms 间隔,治"有时不触发"
        //   旧逻辑: request_attempted_ 置位 + service 失败 → 本 strike 静默放弃
        //   新逻辑: 第一次失败不锁 latch,允许接下来 200ms×3 轮重试;只有 3 次都失败
        //           才锁 latch 避免 50Hz timer 死磕 YX 板
        if (!(strike_test_smoke_client_.exists() ||
              strike_test_smoke_client_.waitForExistence(ros::Duration(0.2)))) {
            ROS_ERROR("[STRIKE TEST GUARD] smoke service unavailable: %s; "
                      "request will not be retried in this strike",
                      strike_test_smoke_service_.c_str());
            // service 完全不存在 = 永久失败,这次直接锁 latch 避免死磕
            strike_test_smoke_request_attempted_ = true;
            return;
        }

        constexpr int kMaxRetries = 3;
        constexpr double kRetryIntervalSec = 0.2;
        bool final_ok = false;
        std::string final_msg = "n/a";
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            std_srvs::Trigger srv;
            const bool rpc_ok = strike_test_smoke_client_.call(srv);
            if (rpc_ok && srv.response.success) {
                ROS_WARN("[STRIKE TEST GUARD] smoke service ACCEPTED attempt=%d: %s",
                         attempt, srv.response.message.c_str());
                final_ok = true;
                break;
            }
            final_msg = rpc_ok ? srv.response.message.c_str() : "RPC error";
            ROS_WARN("[STRIKE TEST GUARD] smoke service attempt %d/%d %s: %s",
                     attempt, kMaxRetries,
                     rpc_ok ? "rejected" : "RPC error",
                     final_msg.c_str());
            if (attempt < kMaxRetries) {
                ros::Duration(kRetryIntervalSec).sleep();
            }
        }
        // 全部失败或成功 都锁 latch — 重试是给"瞬时失败"机会,3 次都失败就不再继续
        strike_test_smoke_request_attempted_ = true;
        if (!final_ok) {
            ROS_ERROR("[STRIKE TEST GUARD] smoke GIVE UP after %d attempts: %s",
                      kMaxRetries, final_msg.c_str());
        }
    }

    void publishStrikeTestPositionHold() {
        if (!strike_test_position_hold_latched_) {
            return;
        }
        strike_test_position_hold_pose_.header.stamp = ros::Time::now();
        strike_test_position_hold_pub_.publish(strike_test_position_hold_pose_);
        if (use_sim_) {
            std_msgs::Int16 mode_msg;
            mode_msg.data = flight_mode_position_;
            flight_mode_pub_.publish(mode_msg);
        }
        // 沿用现有接管确认，确保 waypoint/mission position 源继续静默。
        std_msgs::Bool active;
        active.data = true;
        velocity_active_pub_.publish(active);
        maybeTriggerStrikeTestSmoke();
        ROS_WARN_THROTTLE(1.0,
            "[STRIKE TEST GUARD] publishing latched position hold (%.2f,%.2f,%.2f)",
            strike_test_position_hold_pose_.pose.position.x,
            strike_test_position_hold_pose_.pose.position.y,
            strike_test_position_hold_pose_.pose.position.z);
    }

    //=====没有图像不打击=====//
    bool terminalImageFresh(double* image_age_out = nullptr) const {
        const uint64_t now_us = ros::Time::now().toNSec() / 1000ULL;
        const bool stamp_valid = image_target_received_ &&
            current_image_target_.stamp_us > 0 &&
            now_us >= current_image_target_.stamp_us;
        const double image_age = stamp_valid
            ? (now_us - current_image_target_.stamp_us) * 1e-6
            : std::numeric_limits<double>::infinity();
        if (image_age_out) *image_age_out = image_age;
        return stamp_valid && current_image_target_.visible &&
               image_age <= image_visible_timeout_sec_ &&
               image_ray_filter_valid_;
    }

    bool maybeLatchTerminalNoImageHold(double horizontal_distance_m) {
        if (!terminal_no_image_hold_enabled_ || current_mode_ != "strike" ||
            terminal_no_image_hold_latched_ || !is_uav_pose_received_ ||
            !strike_terminal_image_chase_enabled_ ||
            !(horizontal_distance_m > 0.1 &&
              horizontal_distance_m <= strike_terminal_image_chase_min_horiz_m_)) {
            return terminal_no_image_hold_latched_;
        }

        double image_age = std::numeric_limits<double>::infinity();
        if (terminalImageFresh(&image_age)) {
            return false;
        }

        const auto& q = current_uav_pose_.pose.orientation;
        const double yaw_nwu = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        terminal_no_image_hold_pose_ = geometry_msgs::PoseStamped();
        terminal_no_image_hold_pose_.header.frame_id = "map";
        double output_yaw = 0.0;
        if (use_sim_) {
            terminal_no_image_hold_pose_.pose.position.x = current_uav_pose_.pose.position.x;
            terminal_no_image_hold_pose_.pose.position.y = -current_uav_pose_.pose.position.y;
            terminal_no_image_hold_pose_.pose.position.z = -current_uav_pose_.pose.position.z;
            output_yaw = -yaw_nwu;
        } else {
            terminal_no_image_hold_pose_.pose.position.x = -current_uav_pose_.pose.position.y;
            terminal_no_image_hold_pose_.pose.position.y = current_uav_pose_.pose.position.x;
            terminal_no_image_hold_pose_.pose.position.z = current_uav_pose_.pose.position.z;
            output_yaw = M_PI_2 + yaw_nwu;
        }
        output_yaw = std::atan2(std::sin(output_yaw), std::cos(output_yaw));
        terminal_no_image_hold_pose_.pose.orientation.z = std::sin(0.5 * output_yaw);
        terminal_no_image_hold_pose_.pose.orientation.w = std::cos(0.5 * output_yaw);
        terminal_no_image_hold_latched_ = true;
        terminal_image_chase_active_ = false;
        ROS_ERROR("[TERMINAL NO-IMAGE GUARD] POSITION HOLD LATCHED: "
                  "horiz=%.2fm threshold=%.2fm image_age=%.3fs; smoke disabled for this hold",
                  horizontal_distance_m, strike_terminal_image_chase_min_horiz_m_, image_age);
        return true;
    }

    void publishTerminalNoImageHold() {
        if (!terminal_no_image_hold_latched_) return;
        terminal_no_image_hold_pose_.header.stamp = ros::Time::now();
        strike_test_position_hold_pub_.publish(terminal_no_image_hold_pose_);
        if (use_sim_) {
            std_msgs::Int16 mode_msg;
            mode_msg.data = flight_mode_position_;
            flight_mode_pub_.publish(mode_msg);
        }
        std_msgs::Bool active;
        active.data = true;
        velocity_active_pub_.publish(active);
        // 这里故意不调用 maybeTriggerStrikeTestSmoke()：无图悬停不是命中保护。
        ROS_WARN_THROTTLE(1.0,
            "[TERMINAL NO-IMAGE GUARD] publishing latched position hold");
    }
    //=====没有图像不打击=====//

    bool maybeLatchStrikeTestAltitudeGuard() {
        if (!strike_test_guard_enabled_ || current_mode_ != "strike" ||
            strike_test_position_hold_latched_ ||
            !strike_test_altitude_guard_enabled_ || !is_uav_pose_received_) {
            return false;
        }

        // 高度是内部 NWU 本地坐标的 z（向上为正）。高度硬保护不依赖
        // bbox 、目标距离或近区布防状态，低于阈值时立即锁存当前位置。
        const double local_altitude_m = current_uav_pose_.pose.position.z;
        if (!std::isfinite(local_altitude_m) ||
            local_altitude_m > strike_test_min_altitude_m_) {
            return false;
        }
        strike_test_bbox_hover_latched_ = true;
        ROS_ERROR("[STRIKE TEST GUARD] ALTITUDE HARD LIMIT: "
                  "local altitude=%.2fm <= %.2fm; latching position hold",
                  local_altitude_m, strike_test_min_altitude_m_);
        latchStrikeTestPositionHold();
        return strike_test_position_hold_latched_;
    }

    bool applyStrikeTestGuard(geometry_msgs::Twist& cmd_ned,
                              double horizontal_distance_m) {
        if (!strike_test_guard_enabled_ || current_mode_ != "strike") {
            return false;
        }

        if (maybeLatchStrikeTestAltitudeGuard()) {
            cmd_ned = geometry_msgs::Twist();
            last_strike_cmd_ned_ = cmd_ned;
            last_strike_cmd_valid_ = true;
            return true;
        }

        if (!strike_test_bbox_guard_armed_) {
            if (!std::isfinite(horizontal_distance_m) ||
                horizontal_distance_m > strike_test_guard_arm_distance_m_) {
                ROS_INFO_THROTTLE(2.0,
                    "[STRIKE TEST GUARD] waiting near zone: distance=%.2fm arm<=%.2fm",
                    horizontal_distance_m, strike_test_guard_arm_distance_m_);
                return false;
            }
            strike_test_bbox_guard_armed_ = true;
            // 丢弃远距离阶段积累的 bbox，必须等待进入近区后的新图像帧。
            strike_test_bbox_metric_ = 0.0;
            strike_test_bbox_metric_valid_ = false;
            strike_test_bbox_hover_confirm_count_ = 0;
            strike_test_bbox_last_update_time_ = ros::Time();
            strike_test_bbox_active_class_ = "default";
            strike_test_bbox_active_threshold_ = strike_test_bbox_hover_threshold_;
            ROS_WARN("[STRIKE TEST GUARD] bbox guard ARMED at horizontal distance %.2fm",
                     horizontal_distance_m);
            return false;
        }

        const ros::Time now = ros::Time::now();
        const bool bbox_fresh = strike_test_bbox_metric_valid_ &&
            !strike_test_bbox_last_update_time_.isZero() &&
            (now - strike_test_bbox_last_update_time_).toSec() <=
                strike_test_bbox_timeout_sec_;

        if (!strike_test_bbox_hover_latched_ && bbox_fresh &&
            strike_test_bbox_hover_confirm_count_ >=
                strike_test_bbox_confirm_frames_) {
            strike_test_bbox_hover_latched_ = true;
            ROS_ERROR("[STRIKE TEST GUARD] HOVER LATCHED: class=%s "
                      "bbox metric=%.3f >= %.3f altitude=%.2fm",
                      strike_test_bbox_active_class_.c_str(),
                      strike_test_bbox_metric_, strike_test_bbox_active_threshold_,
                      current_uav_pose_.pose.position.z);
        }

        if (strike_test_bbox_hover_latched_) {
            latchStrikeTestPositionHold();
            cmd_ned = geometry_msgs::Twist();
            last_strike_cmd_ned_ = cmd_ned;
            last_strike_cmd_valid_ = true;
            ROS_WARN_THROTTLE(1.0,
                "[STRIKE TEST GUARD] position hold active; bbox metric=%.3f fresh=%d",
                strike_test_bbox_metric_, static_cast<int>(bbox_fresh));
            return true;
        }

        // 未达到悬停条件时不再缩放速度，保持原制导指令。
        return false;
    }
    //=============打击保护bbox=========//

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
        // === 2026-08-27: 末段水平距离二次减速 (用户口径"水平也要按实际距离限幅") ===
        //   uav0_strike_2026-08-27-17-19.bag:STRIKE task_speed=8.5 m/s,末段 horiz=11.93m
        //   alt=9.9m 时 chase 仍按 8.5 m/s + descent 5.0 m/s 命令,但电机饱和,实际
        //   垂直只给 2 m/s → "无人机降不下来"。根因:水平吃满了电机拉力预算,腾不出
        //   余量给垂直下降。
        //   修法:horiz ≤ strike_terminal_world_direction_radius_(30m) 时,水平速度按
        //   horiz/30 线性衰减,floor=0.4*requested_speed 兜底。floor=0.4 保证 UAV 还能
        //   维持 chase 不丢失,但只允许用 40% 电机拉力预算,腾 60% 给 vz。horiz>30m
        //   不缩 (中段全速冲,与 chase_active 路径分工)。命中圈 (≤ strike_distance_threshold_)
        //   已经在上方 early-return,这里不再叠加,避免命中后还卡水平缩。
        //   不影响 terminal_image_chase_active_ 路径 (那段在 caller 提前 return,
        //   没机会走到 feasibleHorizontalSpeedForDescent)。
        const double horiz_scale_radius = strike_terminal_world_direction_radius_m_;
        double horiz_scaled = requested_speed;
        if (horiz_scale_radius > 1e-3 &&
            g.horizontal <= horiz_scale_radius && g.horizontal > strike_distance_threshold_) {
            const double scale = clampValue(g.horizontal / horiz_scale_radius,
                                            strike_terminal_horizontal_scale_floor_, 1.0);
            horiz_scaled = requested_speed * scale;
        }
        return std::min({requested_speed, closed_loop_h_speed, horiz_scaled});
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
        // === 2026-08-28: TRACK/STRIKE 共享 strike_terminal_image_chase_* 参数 (用户口径) ===
        //   2026-08-25 引入的 4 个 track_* 独立参数已删除,helper 顶部直接选 strike_*
        //   (见下面 4 个 local var)。用户口径"tracking 和 strike 用同样参数"。
        //   攻击意图 gate 移到下一段 (L1764+,attack_cmd 只挡 TRACK)。
        // === 2026-08-25: attack_cmd gate (TRACK only, 2026-08-28 修订) ===
        //   原口径"接收到 attackCmd 之后才要末段 chase" → 只对 TRACK 适用,
        //   TRACK 阶段 GS 未发 attack_cmd action=0 时不下沉,持续 stand-off 跟着。
        //   STRIKE 路径已由 GS 发 skill 105 触发了打击意图,默认就要开 chase,
        //   不再等 attack_cmd 协议 (用户口径 2026-08-28:"GS 在发 105 技能的时候
        //   默认就是要开打击,attackCmd 只是跟踪阶段的打击指令")。
        const bool is_track_chase_for_gate = (current_mode_ == "track");
        if (is_track_chase_for_gate && !attack_cmd_received_) {
            return false;
        }
        // === 2026-08-28: TRACK/STRIKE 共享 strike_terminal_image_chase_* 一组参数 ===
        //   用户口径"tracking 和 strike 用同样参数",删除 track_* 四套,只保留
        //   strike_*。mode-aware 选择那一层变量保留作为未来扩展点 (默认全用 strike)。
        const bool  terminal_chase_enabled = strike_terminal_image_chase_enabled_;
        const double min_horiz_m_chase = strike_terminal_image_chase_min_horiz_m_;
        const double center_band_chase = strike_terminal_image_chase_center_band_;
        const double latch_sec_chase = strike_terminal_image_chase_latch_sec_;
        const bool image_centered =
            std::fabs(current_image_target_.center_x - 0.5) <= center_band_chase &&
            std::fabs(current_image_target_.center_y - 0.5) <= center_band_chase;
        const bool in_terminal_zone =
            g.horizontal > 0.1 &&
            g.horizontal <= min_horiz_m_chase;
        const bool should_chase = terminal_chase_enabled &&
                                  image_fresh && image_centered && in_terminal_zone;

        if (should_chase) {
            terminal_image_chase_enter_time_ = ros::Time::now();
        }
        const bool in_latch_period =
            terminal_image_chase_active_ &&
            !terminal_image_chase_enter_time_.isZero() &&
            (ros::Time::now() - terminal_image_chase_enter_time_).toSec() <
                latch_sec_chase;
        const bool chase_active = should_chase ||
            (terminal_image_chase_active_ && in_latch_period && image_fresh);
        terminal_image_chase_active_ = chase_active;

        // === 状态切换日志 (2026-08-20):便于下次跑 bag 直接确认 chase 是否起作用 ===
        if (chase_active && !last_chase_log_active_) {
            ROS_WARN("[Guidance] CHASE ENTERED (mode=%s): image_centered=%.3f,%.3f horiz=%.2fm "
                     "(min=%.1fm, latch=%.1fs, image_age=%.3fs)",
                     current_mode_.c_str(),
                     current_image_target_.center_x, current_image_target_.center_y,
                     g.horizontal, min_horiz_m_chase,
                     latch_sec_chase, image_age);
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

        //=====没有图像不打击=====//
        // 末段无图保护锁存后由 position setpoint 独占，不再发布速度指令。
        if (terminal_no_image_hold_latched_ && current_mode_ == "strike") {
            publishTerminalNoImageHold();
            return;
        }
        //=====没有图像不打击=====//

        //=============打击保护bbox=========//
        // 高度硬保护在目标有效性检查之前执行，目标突然丢失也不会绕过 4m 保护。
        maybeLatchStrikeTestAltitudeGuard();
        // 一旦位置保持锁存，目标后续丢失或跳变都不能重新进入速度制导。
        // 这里只发布位置 setpoint，避免与 velocity setpoint 混流。
        if (strike_test_guard_enabled_ && strike_test_position_hold_latched_ &&
            current_mode_ == "strike") {
            publishStrikeTestPositionHold();
            return;
        }
        //=============打击保护bbox=========//

        // === 2026-08-24: TRACK 诊断日志 (1Hz) ===
        //   用户报告: 跟踪上目标后突然停滞,实际 chase 输出 vel_raw 衰减到 ~0。
        //   一次性打印关键路径状态 + 控制量,确认到底是哪个分支 / 哪个限幅把速度扣光。
        static ros::Time last_track_dbg_time;
        const ros::Time now_track_dbg = ros::Time::now();
        if ((now_track_dbg - last_track_dbg_time).toSec() > 1.0) {
            ROS_WARN("[TRACK-DBG] is_enabled=%d is_uav_pose=%d is_tgt_pose=%d mode=%s "
                     "strategy=%d target_twist_recv=%d",
                     (int)is_enabled_, (int)is_uav_pose_received_,
                     (int)is_target_pose_received_, current_mode_.c_str(),
                     (int)current_strategy_type_, (int)is_target_twist_received_);
            last_track_dbg_time = now_track_dbg;
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
                    //=============打击保护bbox=========//
                    mode_msg.data = strike_test_position_hold_latched_
                        ? flight_mode_position_ : flight_mode_velocity_;
                    //=============打击保护bbox=========//
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
                        //=====没有图像不打击=====//
                        if (maybeLatchTerminalNoImageHold(
                                strikeDescentGeometry().horizontal)) {
                            publishTerminalNoImageHold();
                            break;
                        }
                        //=====没有图像不打击=====//
                        evaluateStrike();
                        filterVelocityCmd(vel_cmd);
                        //=============打击保护bbox=========//
                        const double guard_dx = current_target_pose_.pose.position.x -
                                                current_uav_pose_.pose.position.x;
                        const double guard_dy = current_target_pose_.pose.position.y -
                                                current_uav_pose_.pose.position.y;
                        applyStrikeTestGuard(vel_cmd, std::hypot(guard_dx, guard_dy));
                        if (strike_test_position_hold_latched_) {
                            publishStrikeTestPositionHold();
                        } else {
                            if (!use_sim_) {
                                convertVelNedToEnu(vel_cmd);
                            }
                            publishGuidanceVelocity(vel_cmd);
                        }
                        //=============打击保护bbox=========//
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
                    //=====没有图像不打击=====//
                    if (maybeLatchTerminalNoImageHold(pn_chase_geometry.horizontal)) {
                        publishTerminalNoImageHold();
                        break;
                    }
                    //=====没有图像不打击=====//
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

                    //=============打击保护bbox=========//
                    applyStrikeTestGuard(vel_cmd, pn_geometry.horizontal);
                    //=============打击保护bbox=========//

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

                    //=============打击保护bbox=========//
                    if (strike_test_position_hold_latched_) {
                        publishStrikeTestPositionHold();
                    } else {
                        if (!use_sim_) {
                            convertVelNedToEnu(vel_cmd);
                        }
                        publishGuidanceVelocity(vel_cmd);
                    }
                    //=============打击保护bbox=========//
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

        // =====================================================================
        // === 2026-08-24: chase 重构 — v_cmd = filtered_target_vel + K_p * (lead - uav) ===
        //   旧版(径向 P-control + body-frame 钳位 + no_backward + yaw_threshold)在
        //   大 yaw_err (>90°) 时 cmd_fwd → 0,chase 输出衰减到 ~3 m/s,动目标永远追不上
        //   (uav0_strike_2026-08-24-13-48-28_0.bag / 01-37-13_0.bag 都中招)。
        //   新版:直接在 NWU 世界坐标系下做 P-control,模长 clamp 到 track_max_speed_,
        //     不切 body-frame → 大 yaw_err 时速度输出不会再被刹车钳位吃掉。
        //   两个速度源:
        //     1) tv_filt: KF 输出 (vx, vy) 经 OneEuro LPF,作为"目标移动基线"叠加到 UAV
        //     2) chase_K_p_ * (lead - uav): 位置误差 P-control,让 UAV 朝 lead pose 收敛
        //   lead_pose = target_pose + tv_filt × lead_t,lead_t = clamp(horiz/(track_max+1), [lead_min, lead_max])
        //   y轴:铅直向上速度 = K_p * (track_altitude_ - uav_z),机间避障叠加。
        // =====================================================================

        // 1) 滤波目标速度 (OneEuro LPF) — KF 抖动/模式切换/cluster 跳变时的速度尖峰
        const double tvx_raw = is_target_twist_received_ ? current_target_twist_.twist.linear.x : 0.0;
        const double tvy_raw = is_target_twist_received_ ? current_target_twist_.twist.linear.y : 0.0;
        // === 2026-08-24: OneEuro 需要的是 dt,不是绝对时间戳 ===
        //   原 chase_filter_time = ros::Time::now().toSec() (~1.7e9),filter 内部
        //   α = 1/(1+te/τ) ≈ 1e-10,filter 完全冻在 first input 值 (uav0_strike_2026-08-24-17-25-56_0.bag
        //   tv_filt 永远 = (0.24, 3.62) 即使 KF 早已翻转到 -4.90) → chase 跟着 stale 速度走错方向
        //   → tvy_f 长期主导 chase_cmd,锥内纯速度模式直接被反向的 tvy_f 抵消 P-control → chase 趋零。
        //   filterVelocityCmd (line ~2280) 早就用 (now - last).toSec() 写法,这里对齐。
        const ros::Time chase_filter_now = ros::Time::now();
        double chase_filter_dt = chase_filter_last_time_.isZero()
                                     ? (1.0 / 50.0)
                                     : (chase_filter_now - chase_filter_last_time_).toSec();
        chase_filter_last_time_ = chase_filter_now;
        if (chase_filter_dt <= 0.0) chase_filter_dt = 1e-3;  // 防御同帧多次调用 / dt=0
        if (chase_filter_dt > 1.0)  chase_filter_dt = 1.0;   // 长时间断流 → 限幅,防 filter 一帧暴走
        const double tvx_f = chase_target_vel_filter_x_.filter(tvx_raw, chase_filter_dt);
        const double tvy_f = chase_target_vel_filter_y_.filter(tvy_raw, chase_filter_dt);

        // 2) lead pose (NWU) — target 当前位置 + 滤波速度 × lead_t
        //    lead_t = horiz / (track_max_speed_ + 1.0) 钳到 [lead_min, lead_max]s
        //    +1.0 兜底:UAV 接近 stand-off 时 horiz 很小,lead_t 自然收缩
        double lead_t = 0.0;
        if (horiz_dist > 1e-3) {
            lead_t = std::max(chase_lead_min_,
                              std::min(chase_lead_max_,
                                       horiz_dist / (track_max_speed_ + 1.0)));
        }
        const double lead_x = tgt_x + tvx_f * lead_t;
        const double lead_y = tgt_y + tvy_f * lead_t;
        const double lead_dx = lead_x - uav_x;
        const double lead_dy = lead_y - uav_y;
        const double lead_horiz_dist = std::sqrt(lead_dx*lead_dx + lead_dy*lead_dy);

        // === 2026-08-24: chase 锥外 P-control + 锥内纯速度 ===
        //   打击锥 (以目标为中心,半径 = altitude × cot(strike_angle)):
        //     - 锥外:horiz_err = horiz - tolerance > 0,朝目标方向加 K_p × excess
        //     - 锥内:horiz_err = 0,只跟 tv_filt (目标速度基线,不收位置)
        //   不再收敛到目标正上方 → 避免撞目标、避免 heading atan2(0,0) 死锁
        //   strike_angle=30° → cot=1.732 → 30m 高 → 锥半径 52m。
        //   horiz_tolerance 下限 2m 防极低高度除零/异常。
        const double horiz_tolerance = std::max(2.0, uav_z * cot_strike_angle_);
        const double horiz_err = std::max(0.0, lead_horiz_dist - horiz_tolerance);
        const double dir_x = (lead_horiz_dist > 1e-3) ? (lead_dx / lead_horiz_dist) : 0.0;
        const double dir_y = (lead_horiz_dist > 1e-3) ? (lead_dy / lead_horiz_dist) : 0.0;

        // 3) chase 速度指令 (NWU 世界系) — 锥外 K_p × excess × dir,锥内只 tv_filt
        //    锥内: chase_cmd = tv_filt (跟目标平移,不收位置)
        //    锥外: chase_cmd = tv_filt + K_p × horiz_err × unit_dir (朝锥边界收)
        double chase_cmd_x = tvx_f + chase_K_p_ * horiz_err * dir_x;
        double chase_cmd_y = tvy_f + chase_K_p_ * horiz_err * dir_y;

        // 4) 总模长 clamp — 不切 body-frame,大 yaw_err 时不再刹车
        //    === 2026-08-25: 加二次减速区 (close_horiz=30m) ===
        //    horiz<30m 时按比例缩放速度: horiz=12m → 0.4 × 17 = 6.8 m/s 压位
        //    floor 0.4 防最低速度保护,允许 chase 在 horiz=0 时仍 6.8 m/s 跟速度
        //    修前 bug: 原 clamp 只用 track_max_speed_(17m/s),horiz=0 时仍按 17m/s 飞,
        //    target 移动时 chase 跟着平移却不会压位,稳态在目标上空 17m/s 盘旋过冲。
        const double close_horiz = 30.0;
        const double close_speed_floor = 0.4;
        double close_speed_scale = 1.0;
        if (lead_horiz_dist < close_horiz) {
            close_speed_scale = std::max(close_speed_floor,
                                          lead_horiz_dist / close_horiz);
        }
        const double final_speed = track_max_speed_ * close_speed_scale;
        double chase_speed = std::sqrt(chase_cmd_x*chase_cmd_x + chase_cmd_y*chase_cmd_y);
        const bool chase_speed_clamped = (chase_speed > final_speed);
        if (chase_speed > final_speed) {
            const double s = final_speed / chase_speed;
            chase_cmd_x *= s;
            chase_cmd_y *= s;
            chase_speed  = final_speed;
        }

        // 5) 高度控制：固定 track_altitude_（P 控制）— 与旧版一致
        const double alt_error = uav_z - track_altitude_;
        double cmd_z = std::max(-1.0, std::min(1.0, -0.5 * alt_error));  // NED: z<0=上升
        const bool alt_clamped = (std::fabs(alt_error) > 2.0);

        // 6) 机间避障（在世界系叠加斥力）— 与旧版一致
        const double cmd_x_pre_avoid = chase_cmd_x;
        const double cmd_y_pre_avoid = chase_cmd_y;
        const double cmd_z_pre_avoid = cmd_z;
        applyInterUavAvoidance(chase_cmd_x, chase_cmd_y, cmd_z);

        // 7) 输出: chase 内 dx,dy,chase_cmd_x,chase_cmd_y 全是 NWU 分量
        //   (dx=NWU North 向位移, dy=NWU West 向位移)。
        //   uavPoseCallback use_sim_=false 分支 (L996-1015) 把 ENU msg 转 NWU,
        //   targetPoseCallback 不转 (target_estimator 已发 NWU)。
        //   publish 路径会再走 convertVelNedToEnu (L2032) 把 NED 翻回 ENU 给 mavros,
        //   所以这里把 NWU 翻成 NED 喂入,让 convertVelNedToEnu 翻回 = chase_cmd 原值。
        //   NWU→NED: x 同名(North), y 翻号(West→East), z 翻号(Up→Down)。
        //   修前 bug: 注释误把 chase_cmd 当 ENU,linear.x/y 互换 → 镜像 90+°
        //   (uav0_strike_2026-08-25-02-47-16_0.bag t=25.8: 目标在 UAV ESE,
        //    chase 内部 NWU 意图 ESE (-12.7 N, -120.4 W),输出错成 WSW)。
        vel_cmd.linear.x =  chase_cmd_x;   // NED North = NWU North (同号)
        vel_cmd.linear.y = -chase_cmd_y;   // NED East  = -NWU West (翻号)
        vel_cmd.linear.z = -cmd_z;         // NED Down  = -NWU Up (翻号)
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;

        // === 2026-08-24: TRACK chase 详细诊断日志 (1Hz) ===
        //   打印 chase 重构后的输入/输出,便于核对 v_cmd 是否还在 clamp、tv_filt 是否合理
        static ros::Time last_chase_dbg_time;
        const ros::Time now_chase_dbg = ros::Time::now();
        if ((now_chase_dbg - last_chase_dbg_time).toSec() > 1.0) {
            ROS_WARN("[TRACK-CHASE] horiz=%.1f lead=%.1f tol=%.1f err=%.1f lead_t=%.2fs | "
                     "tv_raw=(%.2f,%.2f) tv_filt=(%.2f,%.2f) | "
                     "Kp=%.2f cone=(in=%d) dir=(%.2f,%.2f) | "
                     "chase_cmd=(%.2f,%.2f,%.2f) |v|=%.2f clamp=%d",
                     horiz_dist, lead_horiz_dist, horiz_tolerance, horiz_err, lead_t,
                     tvx_raw, tvy_raw, tvx_f, tvy_f,
                     chase_K_p_, (int)(horiz_err <= 0.0), dir_x, dir_y,
                     vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
                     chase_speed, (int)chase_speed_clamped);
            last_chase_dbg_time = now_chase_dbg;
        }

        // 8) 偏航角速度: 机头指向 lead pose;lead_horiz 太小 (< chase_yaw_horiz_min_m_)
        //    时 yaw_rate=0 防 atan2(0,0) 跳变。不锁定 desired_yaw,UAV 滑过目标下方时
        //    yaw 不会卡死(冻结 yaw 不好)。
        double yaw_rate = 0.0;
        const bool yaw_holdoff = (lead_horiz_dist <= chase_yaw_horiz_min_m_);
        if (!yaw_holdoff) {
            // === 2026-08-25: chase 内 dx,dy 是 NWU 分量 (dx=NWU North, dy=NWU West)。
            //   current_yaw (L2360-2365 quaternion) 经 uavPoseCallback 转 NWU 也是 NWU 系 yaw (0=North, CCW+)。
            //   NWU bearing (从 UAV 看 target) = atan2(W, N) = atan2(lead_dy, lead_dx)。
            //   修前 bug: 注释说"ENU bearing",用 atan2(dx, dy) 把 NWU N 当 ENU E → 镜像 90+°。
            //   uav0_strike_2026-08-25-02-47-16_0.bag t=25.8: lead_dx=-12.7, lead_dy=-120.4
            //   旧 atan2(-12.7, -120.4) = -174° (NWU 三象限, 当 ENU bearing = South 偏西 6°, 错)
            //   新 atan2(-120.4, -12.7) = -97° (NWU bearing, target 在 UAV South 偏东 6°, 对)
            const double lead_bearing = std::atan2(lead_dy, lead_dx);
            const double yaw_err_ctrl = std::atan2(std::sin(lead_bearing - current_yaw),
                                                    std::cos(lead_bearing - current_yaw));
            const double yaw_rate_raw = los_kp_yaw_ * yaw_err_ctrl;
            yaw_rate = std::max(-los_max_rate_,
                                std::min(los_max_rate_, yaw_rate_raw));
        }
        // === 2026-08-25: yaw_rate 链路 3 次翻号自洽 (不动)
        //   chase 算 yaw_rate (NWU angular.z, CCW+) →
        //   vel_cmd.angular.z = -yaw_rate (NWU→NED 翻号) →
        //   convertVelNedToEnu line 2537 angular.z = -ned_yaw (NED→ENU 翻号,得 +yaw_rate) →
        //   mavros setpoint_velocity.cpp L97-103 transform_frame_ned_enu((0,0,+yaw_rate))
        //     = (0,0,-yaw_rate) → PX4 NED yaw_rate = -yaw_rate (CCW+, 与 chase 内部 NWU CCW+ 同方向)
        //   物理方向 chase→PX4 保留正号 → UAV 左转/右转跟 chase 算的方向一致。✓
        //   修前错误诊断: 曾误以为此链需要少翻一次号,改成 +yaw_rate → 反而破坏自洽。
        vel_cmd.angular.z = -yaw_rate;

        // === 2026-08-25: TRACK 末段图像导引 (与 STRIKE chase 同源 helper) ===
        //   TRACK 不发 strike,UAV 在 horiz≤22m + bbox 在中心时沿相机射线全速压位,
        //   等 GS 发 105 后进入 STRIKE 命中圈。YOLO 不足/不在中心时降级回 chase 几何。
        //   必须在 vel_cmd.angular.z = -yaw_rate (上方) 之后,这样 chase_active=true 时
        //   helper 覆盖 angular.z = terminalYawRate(ray_bearing, horiz),chase_active=false
        //   时 helper return false 不修改,几何 chase 输出保留 (= 降级路径)。
        //   路径与 STRIKE 路径 (L2184-2185 在 computeCommand 之后) 完全对齐。
        const StrikeTargetGeometry track_geometry = strikeTargetGeometry();
        maybeApplyTerminalImageChase(vel_cmd, track_geometry);

        // =====================================================================

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
