// ============================================================================
// target_estimator_node — 多实例目标聚类 (label + 2D 位置, bbox 反投影)
//
// 设计变更 (2026-07 多目标改造):
//   - 订阅 detection/yolo_results (YoloDetections):每帧可能多条检测,
//     每条独立做 per-bbox 反投影 → 地面点。原先共享 LOS (current_los_) 的反投影
//     会把所有检测塌缩到同一地面点,必须改成 bbox 中心 + camera 几何的反投影。
//   - 实例聚类:删除 label→cluster_id 单索引(label_to_id_)。改为遍历 clusters_,
//     找 label 相同 + 2D 距离 ≤ merge_radius_m_ 的最近 cluster 合并;
//     无匹配 → 新建 cluster(同类别多实例可共存)。
//   - 主目标输出:订阅 mission/primary_target (UInt64)。statesTimerCb 里
//     target_estimated_pose / gps 选择 primary_cluster_id_ 对应 cluster;
//     0 或找不到 → 回退最高置信度 cluster。
//   - 相机几何参数 (image_width/height/fov_h/fov_v) 与 gimbal 同步,
//     订阅 gimbal_pose 拿当前 camera world quat,做射线-地面求交。
//
// 输入:
//   detection/yolo_results    (YoloDetections, batch)        ← 新增
//   gimbal_pose               (PoseStamped, NWU, body_cam quat) ← 新增
//   /<uav_pose_topic>         (PoseStamped, NWU)
//   /mavros/global_position/global (NavSatFix)
//   mission/primary_target    (UInt64, cluster_id)            ← 新增
//
// 输出:
//   cluster_event             (ClusterEvent)— 新 cluster 唯一事件
//   cluster_states            (ClusterState, 10Hz)— 所有 alive cluster 快照
//   target_estimated_marker   (MarkerArray, 10Hz)— RViz 可视化
//   target_estimated_gps      (NavSatFix, 10Hz)— primary cluster 的 GPS
//   target_estimated_pose     (PoseStamped, 10Hz)— primary cluster 的 NWU 坐标
// ============================================================================
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>  // 2026-08-23: 动目标 KF 速度输出
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>      // === 2026-08-24: /guidance/mode 订阅 ===
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <multi_uav_strike/YoloDetection.h>
#include <multi_uav_strike/YoloDetections.h>
#include <multi_uav_strike/ClusterEvent.h>
#include <multi_uav_strike/ClusterTarget.h>
#include <multi_uav_strike/ClusterState.h>
#include <multi_uav_strike/MissionState.h>
#include <multi_uav_strike/WaypointStatus.h>
//===================制导修改====================
#include <multi_uav_strike/PrimaryImageTarget.h>
//===================制导修改====================

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

// === 2026-08-23: 动目标 KF (4-state CV) ===
//   替换 2D EMA 让 chase 在 ≤17 m/s 动目标下不滞后。
#include <multi_uav_strike/target_motion_kf.hpp>

#include <cmath>
#include <cctype>
#include <deque>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>

class TargetClusterer {
public:
    // === 2026-08-23: 动目标 KF 模式分类 ===
    //   提到 class 作用域外 (但仍在 class 内), 便于 TargetCluster 字段直接引用。
    enum class MotionClass { UNKNOWN, STATIC, DYNAMIC };

    // ===== 单个目标集群 =====
    struct TargetCluster {
        uint64_t  id              = 0;
        std::string label;                  // 原始 label,string equality 区分
        uint32_t  label_hash      = 0;
        double    ned_x           = 0.0;     // 中心 NWU x (米)
        double    ned_y           = 0.0;     // 中心 NWU y (米)
        double    ned_alt         = 0.0;     // 中心 z (米)
        float     best_confidence = 0.0f;
        uint64_t  first_seen_us   = 0;
        uint64_t  last_seen_us    = 0;
        // 最新一帧 YoloDetection.img_data(可为 JPEG 字节,空 = 当前没图)
        std::vector<uint8_t> img_data;
        uint8_t              img_format = 0;  // 1 = JPEG(与 DetectTarget.img_format 对齐)

        // === 2026-07-16: 世界空间 EMA (Plan B) ===
        //   关键改动: 把 EMA 从像素域换到世界域 (cand_tx/cand_ty),绕开"像素域 EMA
        //   在相机机动时被甩" 的几何不一致 bug。
        //   - 像素域 EMA 假设: 同一像素 → 同一世界点 — 但相机一转,旧像素投影到新相机姿态下
        //     的世界点已经偏离真实目标,lag = (1-alpha)/alpha × camera_turn_rate,
        //     10Hz ema_alpha=0.5 下仍能偏移数米至几十米。
        //   - 世界域 EMA 直接对 cand_tx/cand_ty 做滤波 (像素仅用于跨帧匹配 cluster):
        //     真实目标位置应稳定在原地,相机的任意机动都收敛到真值,无几何滞后。
        //   - 代价: 1px 像素噪声 → ~9m@30m 距离 的世界噪声不再被像素域"吸收",
        //     但 ema_alpha_=0.3 (10Hz 时间常数 ~0.23s) 足以把噪声压到 ~5m 平滑后,
        //     且 UAV 机动期间零滞后。
        bool      world_ema_valid = false;   // 首帧用 cand_* 直接初始化,之后做 EMA

        // === 2026-07-16: 多次命中才晋升为正式 cluster ===
        //   hit_count: 累计被 detection 命中的次数(含本次)
        //   is_promoted: 达到 min_hit_count_ 后才置 true,之后才:
        //                - 发 cluster_event(让 mission_manager 决定上报/跟踪)
        //                - 进 cluster_states 10Hz 快照(RViz 显示)
        //                - 用 cluster_ttl_sec_ 长 TTL(确认存在的目标)
        //   未晋升的 tentative 用更短的 tentative_ttl_sec_ 过期丢弃。
        uint32_t  hit_count       = 0;
        bool      is_promoted     = false;
        //===================制导修改====================
        // 保存与该 cluster 最近一次关联的原始图像观测。世界坐标只用于聚类，
        // 图像导引直接使用 bbox 中心对应的相机视线，不再对 LOS 角速度求导。
        uint32_t  image_frame_seq = 0;
        float     image_center_x  = 0.5f;
        float     image_center_y  = 0.5f;
        float     image_width     = 0.0f;
        float     image_height    = 0.0f;
        Eigen::Vector3d image_world_ray_nwu{0.0, 0.0, -1.0};
        //===================制导修改====================

        // === 2026-08-23: 动目标 KF + 模式分类 (替代 2D EMA) ===
        //   静目标 (Q_vel→0): KF 速度衰减到 0, 等效位置 EMA, 行为零回归
        //   动目标 (Q_vel=2.0): KF 跟踪速度, lead pose 让 chase 不滞后
        multi_uav_strike::TargetMotionKF kf_;
        ros::Time                       kf_last_time_;          // 上次 YOLO update 时刻
        ros::Time                       kf_init_time_;          // 聚类创建时刻, 用于 cold-start grace
        // === 2026-08-24: 高频 KF 预测 (50Hz), YOLO 检测之间持续推进 ===
        //   KF 不能只在 10Hz YOLO 检测时 predict, 否则 FOV 丢失瞬间 KF 冻结,
        //   target 出视野后 cluster.ned_x/y 停在最后位置, chase 完全失去预测能力。
        ros::Time                       kf_last_predict_time_;  // 上次 predict 时刻 (timer/YOLO 同步)
        MotionClass                     motion_class_ = MotionClass::UNKNOWN;
        int                             motion_dynamic_evidence_ = 0;  // 连续 v > enter 帧数
        int                             motion_static_evidence_  = 0;  // 连续 v < exit 帧数
    };

    struct CameraPoseSample {
        ros::Time stamp;
        Eigen::Vector3d position_nwu{0.0, 0.0, 0.0};
        Eigen::Quaterniond orientation_world_camera{1.0, 0.0, 0.0, 0.0};
    };

    struct YawRateSample {
        ros::Time stamp;
        double yaw_rate_rad_s = 0.0;
    };

    TargetClusterer() {
        ros::NodeHandle p("~");
        // 仿真/真机切换
        p.param<bool>("use_sim", use_sim_, true);
        p.param<std::string>("uav_pose_topic", uav_pose_topic_,
                             use_sim_ ? "quad/pose" : "mavros/local_position/pose");
        // 聚类参数
        p.param<double>("merge_radius_m", merge_radius_m_, 10.0);
        p.param<double>("cluster_ttl_sec", cluster_ttl_sec_, 60.0);
        // === 2026-07-16: 多次命中门禁 (兼筛选) ===
        //   同一 cluster (label + merge_radius_m 范围内) 被检测到 hit_count_ ≥
        //   min_hit_count_ 才晋升为正式目标。同位置多次命中 = 真目标的强信号;
        //   单次或零星命中 = YOLO 误检 / 飞过飞回的残影,直接被门禁过滤掉。
        //   经验值:10Hz yolo → 20 hits ≈ 2s 持续命中,误检基本进不来;
        //          真目标在视野停留 3~5s 时稳定晋升。
        p.param<int>("min_hit_count", min_hit_count_, 20);
        p.param<double>("tentative_ttl_sec", tentative_ttl_sec_, 1.0);
        // === 2026-07-31: Attack(105) 阶段专享聚类门槛 ===
        //   搜索阶段保持 min_hit_count=20/tentative_ttl_sec=1.0 抗误检;
        //   Attack 阶段 (skill_type==105 && phase GUIDANCE_TRACK/STRIKE)
        //   切到 attack_min_hit_count/attack_tentative_ttl_sec,避免飞行过程中
        //   短停留目标被 1s TTL + 20 hit 门槛组合提早 DROP。
        p.param<int>   ("attack_min_hit_count",     attack_min_hit_count_,     6);
        p.param<double>("attack_tentative_ttl_sec", attack_tentative_ttl_sec_, 2.0);
        // === 2026-07-16: Plan B — 世界空间 EMA ===
        //   现在 EMA 直接作用于 cand_tx/cand_ty(米),不再有相机机动滞后问题。
        //   - alpha=0.3 + 10Hz → 时间常数 ≈ 0.23s,稳态世界噪声衰减 ~70%
        //     (像素噪声 2px @ 30m 距离 → ~18m 原始 → ~5m 平滑后,仍能容忍)
        //   - alpha 越大 (0.5+) 越接近原始,平滑弱;越小 (0.1~0.2) 越平滑但响应慢。
        //   - 静止相机基本无 alpha 影响(噪声=0 时 raw 和 EMA 同值);
        //     真正起作用的是相机机动 / 目标快速移动 / 像素噪声三个场景。
        p.param<double>("ema_alpha",       ema_alpha_,       0.3);
        p.param<double>("target_z_prior",  target_z_prior_,  0.0);
        // === 2026-08-13: iter_03 — 相机光轴水平/朝天时的 fallback 投影距离 ===
        //   几何上无法与 target_z_prior_ plane 求交时,沿 world_ray 方向走这个距离当 cluster 位置。
        //   30m = 典型 car 视野远端,既能保留 cluster 又不因为视距太远导致 merge_radius 失配。
        //   防御性兜底(实飞也保留),真实场景云台物理朝下,几乎用不到。
        p.param<double>("fallback_proj_distance", fallback_proj_distance_, 30.0);
        // === 2026-08-23: 动目标 KF + 模式分类参数 ===
        //   cold-start grace: 聚类创建后 N 秒内强制 STATIC, 防初始噪声触发 DYNAMIC
        p.param<double>("motion_cold_start_grace_sec", motion_cold_start_grace_sec_, 1.0);
        //   迟滞门槛: 进入 DYNAMIC 需 |v| > motion_speed_enter 连续 motion_enter_frames 帧
        p.param<double>("motion_speed_enter", motion_speed_enter_, 2.0);
        p.param<double>("motion_speed_exit",  motion_speed_exit_,  0.3);
        p.param<int>   ("motion_enter_frames", motion_enter_frames_, 3);
        p.param<int>   ("motion_exit_frames",  motion_exit_frames_,  30);
        //   前导时间上下限 (lead_pose = KF + v*lead_time)
        p.param<double>("motion_lead_min", motion_lead_min_, 0.2);
        p.param<double>("motion_lead_max", motion_lead_max_, 1.5);
        //   KF 速度发散保护: |v| > motion_max_speed_ 时 reset velocity 强制 STATIC
        p.param<double>("motion_max_speed", motion_max_speed_, 30.0);
        //   KF 噪声参数
        p.param<double>("kf_q_pos",          kf_q_pos_,          1.0);
        p.param<double>("kf_q_vel_static",   kf_q_vel_static_,   0.01);
        p.param<double>("kf_q_vel_dynamic",  kf_q_vel_dynamic_,  2.0);
        p.param<double>("kf_r_meas",         kf_r_meas_,         2.0);
        //   速度缓存 LPF (computeClosingSpeed 用)
        p.param<double>("uav_velocity_lpf_alpha", uav_velocity_lpf_alpha_, 0.5);
        // Detection timestamps are matched to the nearest full camera pose.
        // The warning threshold is diagnostic only: a large time delta never
        // rejects an observation as long as the pose buffer is non-empty.
        p.param<double>("pose_buffer_duration_sec", pose_buffer_duration_sec_, 5.0);
        p.param<double>("pose_sync_warn_threshold_sec", pose_sync_warn_threshold_sec_, 0.1);
        p.param<double>("max_cluster_yaw_rate_rad_s", max_cluster_yaw_rate_rad_s_, 0.2);
        p.param<double>("yaw_rate_sample_max_dt_sec", yaw_rate_sample_max_dt_sec_, 0.5);
        pose_buffer_duration_sec_ = std::max(0.1, pose_buffer_duration_sec_);
        pose_sync_warn_threshold_sec_ = std::max(0.0, pose_sync_warn_threshold_sec_);
        max_cluster_yaw_rate_rad_s_ = std::max(0.0, max_cluster_yaw_rate_rad_s_);
        yaw_rate_sample_max_dt_sec_ = std::max(0.001, yaw_rate_sample_max_dt_sec_);
        // 发布周期
        p.param<double>("states_publish_freq", states_publish_freq_, 10.0);
        p.param<double>("cleanup_freq",        cleanup_freq_,        1.0);
        // === 2026-08-24: KF 高频预测频率 ===
        //   KF 持续 predict 频率 (Hz), 应显著高于 YOLO 频率 (10Hz)。
        //   50Hz = YOLO 5x 频率, 每次 YOLO 间隔 (0.1s) 内 5 次 predict,
        //   既保证动目标出 FOV 后 cluster.ned_x/y 跟随 KF 速度连续推进,
        //   又不浪费 CPU 在 STATIC 目标上。
        p.param<double>("kf_predict_freq", kf_predict_freq_, 50.0);
        // === 2026-08-24: stale_freeze 阈值 (cluster 长时间无观测视为丢失) ===
        p.param<double>("cluster_stale_freeze_sec", cluster_stale_freeze_sec_, 3.0);
        // marker 颜色
        p.param<double>("est_marker_r", est_marker_r_, 1.0);
        p.param<double>("est_marker_g", est_marker_g_, 1.0);
        p.param<double>("est_marker_b", est_marker_b_, 0.0);
        p.param<double>("est_marker_a", est_marker_a_, 1.0);

        // 相机几何参数(必须与 gimbal_simulator 同步)
        p.param<int>("image_width",  image_width_,  1920);
        p.param<int>("image_height", image_height_, 1080);
        p.param<double>("fov_h_deg", fov_h_deg_, 68.0);
        p.param<double>("fov_v_deg", fov_v_deg_, 45.6);
        computeFocalLengths();

        // 订阅
        yolo_batch_sub_     = nh_.subscribe("detection/yolo_results", 10,
                                            &TargetClusterer::yoloBatchCallback, this);
        gimbal_pose_sub_    = nh_.subscribe("gimbal_pose", 10,
                                            &TargetClusterer::gimbalPoseCallback, this);
        // === 2026-07-16: 改为订阅稳像 camera/pose 作为反投影唯一真源 ===
        // gimbal_simulator_node.cpp::publishCameraPose 现在发布的是稳像世界 quat,
        // 与 forward projection 完全一致,这里直接拿来用。
        camera_pose_sub_    = nh_.subscribe("camera/pose", 10,
                                            &TargetClusterer::cameraPoseCallback, this);
        uav_pose_sub_       = nh_.subscribe(uav_pose_topic_, 10,
                                            &TargetClusterer::uavPoseCallback, this);
        self_gps_sub_       = nh_.subscribe("mavros/global_position/global", 10,
                                            &TargetClusterer::selfGpsCallback, this);
        // === 2026-07-28: 锁 PX4 home (启动后 stable 不更新),用于 GPS 输出的绝对参考 ===
        //   原版用 current_gps_ + (target - uav) 偏移, UAV 飞行过程中 ground_station 收到的
        //   GPS 会随 UAV 当前位置漂移,目标不再原地。改成 home 一次性锁定 + target 绝对 NED。
        home_position_sub_  = nh_.subscribe("mavros/home_position/home", 10,
                                            &TargetClusterer::homePositionCallback, this);
        primary_target_sub_ = nh_.subscribe("mission/primary_target", 10,
                                            &TargetClusterer::primaryTargetCallback, this);
        // === 2026-07-31: 订阅 mission_state 触发 Attack 模式降门槛 ===
        //   mission_manager 已在 1Hz 发布 mission/mission_state(skill_type + phase)
        //   这里只读不写,无新 publisher / timer / launch 改动
        mission_state_sub_   = nh_.subscribe("mission/mission_state", 10,
                                            &TargetClusterer::missionStateCallback, this);
        waypoint_status_sub_ = nh_.subscribe("waypoint_executor/status", 10,
                                            &TargetClusterer::waypointStatusCallback, this);
        // === 2026-08-20: 订阅 Attack(105) 原 WP2 GPS,主目标 fallback 用 ===
        //   mission_manager 在 Attack(105) push 时一次性 latched publish。
        attack_wp2_sub_     = nh_.subscribe("attack_wp2_gps", 1,
                                            &TargetClusterer::attackWp2Callback, this);
        // === 2026-08-24: 订阅 /guidance/mode — TRACK 模式禁用 YOLO yaw_rate 门控 ===
        //   跟踪动目标时 chase yaw_rate 持续偏高(0.27~0.46 rad/s),如果保留
        //   max_cluster_yaw_rate_rad_s=0.2 门控,YOLO batch 会被反复 skip,
        //   cluster 持续 stale → primary 切换 → chase 输出衰减 → 跟丢。
        //   修法:TRACK 模式 (mode=="track") 直接绕过 yaw_rate 检查,
        //   让 YOLO/cluster 在跟踪过程中保持开启。
        //   waypoint / SEARCH 阶段 mode=="strike" 或 guidance 未 enable 时门控照常生效。
        guidance_mode_sub_  = nh_.subscribe("guidance/mode", 10,
                                            &TargetClusterer::guidanceModeCallback, this);
        // === 2026-08-24: 订阅 /guidance/enable — 区分 tracked target (橙) vs primary (紫) ===
        //   guidance enable=true && mode in {track,strike} → primary 是当前 UAV 正在
        //   chase/strike 的目标,显示橙色大球;enable=false 时 primary 只是候选,显示紫色。
        //   waypoint / SEARCH 阶段 enable=false,候选 primary 显示紫色等 GS 接管。
        guidance_enable_sub_ = nh_.subscribe("guidance/enable", 10,
                                             &TargetClusterer::guidanceEnableCallback, this);

        // 发布
        // RViz 帧名:去掉命名空间前导 '/' 后拼 "/map"(如 /uav0 → uav0/map);无命名空间 → map
        {
            std::string vns = nh_.getNamespace();
            if (!vns.empty() && vns[0] == '/') vns = vns.substr(1);
            viz_frame_ = vns.empty() ? "map" : (vns + "/map");
        }
        cluster_event_pub_      = nh_.advertise<multi_uav_strike::ClusterEvent>(
                                    "cluster_event", 10);
        cluster_states_pub_     = nh_.advertise<multi_uav_strike::ClusterState>(
                                    "cluster_states", 10);
        target_est_pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(
                                    "target_estimated_pose", 10);
        // === 2026-08-23: 激活死 topic — 主目标 KF 速度 ===
        target_est_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
                                    "target_estimated_twist", 10);
        target_est_marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>(
                                    "target_estimated_marker", 10);
        // === 2026-07-16: 原始单帧识别标记(红色小点,瞬时显示)===
        raw_det_marker_pub_     = nh_.advertise<visualization_msgs::MarkerArray>(
                                    "raw_detection_marker", 10);
        target_est_gps_pub_     = nh_.advertise<sensor_msgs::NavSatFix>(
                                    "target_estimated_gps", 10);
        //===================制导修改====================
        primary_image_target_pub_ = nh_.advertise<multi_uav_strike::PrimaryImageTarget>(
                                    "guidance/primary_image_target", 10);
        //===================制导修改====================

        // 周期任务
        states_timer_ = nh_.createTimer(ros::Duration(1.0 / states_publish_freq_),
                                        &TargetClusterer::statesTimerCb, this);
        cleanup_timer_= nh_.createTimer(ros::Duration(1.0 / cleanup_freq_),
                                        &TargetClusterer::cleanupTimerCb, this);
        // === 2026-08-24: KF 高频 predict (默认 50Hz) ===
        //   在 YOLO 检测之间持续推进 KF 状态, 让 c.ned_x/y 跟随 KF 速度连续外推,
        //   解决动目标出 FOV 后 cluster 位置冻结的根因。
        kf_predict_timer_ = nh_.createTimer(ros::Duration(1.0 / kf_predict_freq_),
                                            &TargetClusterer::kfPredictTimerCb, this);

        ROS_INFO("[TargetClusterer] initialized. merge_radius=%.1fm ttl=%.1fs ema_alpha=%.2f "
                 "z_prior=%.1f image=%dx%d fov=(%.1f,%.1f)deg focal=(%.1f,%.1f)",
                 merge_radius_m_, cluster_ttl_sec_, ema_alpha_, target_z_prior_,
                 image_width_, image_height_, fov_h_deg_,
                 (fov_v_deg_ > 0.0 ? fov_v_deg_ : fov_v_rad_ * 180.0 / M_PI),
                 focal_x_, focal_y_);
    }

    // ============ 回调 ============

    // 每帧 batch YoloDetections:每条 detection 独立反投影 + 聚类
    void yoloBatchCallback(const multi_uav_strike::YoloDetections::ConstPtr& msg) {
        if (msg->detections.empty()) return;

        //=============一定打击=========//
        // Clustering starts only after the first waypoint of the Search(102)
        // area has been reached.  The latch is deliberately retained when the
        // mission subsequently enters Attack(105), so perception does not stop
        // or restart at the Attack path's first waypoint.
        if (!isFirstWaypointGateOpen()) return;
        //=============一定打击=========//

        const auto& frame = msg->detections.front();
        YawRateSample matched_yaw_rate;
        if (!findNearestYawRate(frame, matched_yaw_rate)) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Skip YOLO batch: no valid yaw-rate sample "
                "frame_seq=%u det=%.6f",
                frame.frame_seq, detectionStamp(frame).toSec());
            return;
        }
        // === 2026-08-24: TRACK 模式绕过 yaw_rate 门控 ===
        //   跟踪动目标时 chase yaw_rate 持续 0.27~0.46 rad/s,默认 0.2 阈值会被
        //   频繁触发,YOLO batch 反复 skip → cluster stale → primary 切换 →
        //   chase 输出衰减 → 跟丢。TRACK 模式直接放行,waypoint/SEARCH 阶段
        //   保留门控(防 yaw 突变 bbox 抖被聚类)。
        if (!guidance_mode_is_track_ &&
            std::fabs(matched_yaw_rate.yaw_rate_rad_s) >
            max_cluster_yaw_rate_rad_s_ + 1e-9) {
            ROS_WARN_THROTTLE(1.0,
                "[TargetClusterer] Skip YOLO batch: yaw rate too high "
                "frame_seq=%u det=%.6f yaw_sample=%.6f rate=%.3frad/s limit=%.3f",
                frame.frame_seq, detectionStamp(frame).toSec(),
                matched_yaw_rate.stamp.toSec(), matched_yaw_rate.yaw_rate_rad_s,
                max_cluster_yaw_rate_rad_s_);
            return;
        }

        // camera/pose carries both the camera origin and the stabilized world
        // orientation. Each detection selects its nearest timestamped sample.
        if (!is_camera_pose_received_) return;
        for (const auto& det : msg->detections) {
            processSingleDetection(det);
        }
    }

    // 单条 detection → 反投影地面点 → 命中已有 cluster (label + 2D 距离) 或新建
    void processSingleDetection(const multi_uav_strike::YoloDetection& det) {
        if (!det.is_in_fov) return;
        if (!is_camera_pose_received_) return;

        // === 2026-07-16: Plan A 像素空间 EMA ===
        //   先取原始 bbox 中心像素,后面再做 EMA → 投影
        const double raw_cx = (det.x_min + det.x_max) * 0.5 * image_width_;
        const double raw_cy = (det.y_min + det.y_max) * 0.5 * image_height_;

        CameraPoseSample matched_camera_pose;
        if (!findNearestCameraPose(det, matched_camera_pose)) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Drop detection frame_seq=%u: camera pose buffer is empty",
                det.frame_seq);
            return;
        }
        const uint64_t observation_stamp_us = det.stamp_us != 0
            ? det.stamp_us
            : matched_camera_pose.stamp.toNSec() / 1000ULL;
        // === 2026-08-23: KF predict 需要 dt, 用 camera_pose 时间戳 (与 GPS 同源) ===
        const ros::Time observation_time = matched_camera_pose.stamp;

        std::lock_guard<std::mutex> g(clusters_mtx_);
        // === 2026-07-31: 归一化 label (YOLO 数字串 → 语义串) ===
        //   同一份代码同时服务真机 YOLO ("1"/"2",gimbal_node.cpp:1461 +1 偏移后)
        //   和仿真云台 ("person"/"car" 语义串)。
        //   聚类匹配、新建 cluster、raw marker text 全部用归一化后的 label
        const std::string norm_label = normalizeYoloLabel(det.label);
        const std::string& label = norm_label;

        // 实例聚类:把当前帧的 raw 像素先投影成世界点(临时),与已有 cluster 的
        //   上一次稳定位置 c.ned_x/ned_y 算 2D 距离 → 选最近且 <merge_radius_m_ 的。
        //   注:c.ned_x/ned_y 是上一帧像素 EMA 投影后的稳定值,用做匹配参考点不会
        //       跟随单帧噪声漂移。
        double cand_tx = 0.0, cand_ty = 0.0, cand_tz = 0.0;
        const bool cand_valid = projectPixelToGround(
            raw_cx, raw_cy, matched_camera_pose, cand_tx, cand_ty, cand_tz);
        // === 2026-07-16: 原始单帧识别投影(红色点)===
        //   不参与聚类/EMA,完全反映当帧探测器说的瞬时位置,0.3s 后自动消失
        if (cand_valid) {
            // === 2026-07-16: 原始单帧识别投影(红色点)===
            //   不参与聚类/EMA,完全反映当帧探测器说的瞬时位置,0.3s 后自动消失
            publishRawDetectionMarker(cand_tx, cand_ty, cand_tz, label);
        }
        const double r2 = merge_radius_m_ * merge_radius_m_;
        uint64_t nearest_id = 0;
        double   best_dist_sq = r2;
        if (cand_valid) {
            for (const auto& kv : clusters_) {
                const TargetCluster& c = kv.second;
                if (c.label != label) continue;  // 不同 label 不合并
                double dx = cand_tx - c.ned_x;
                double dy = cand_ty - c.ned_y;
                double d2 = dx*dx + dy*dy;
                if (d2 < best_dist_sq) {
                    best_dist_sq = d2;
                    nearest_id = c.id;
                }
            }
        }

        // === 2026-07-16: 集群级反推诊断 ===
        // 同 label 的已晋升 cluster 距离应该 << merge_radius_m_, 如果距离 > 2× radius
        // (说明应该在合并但没合并) 或者距离 > 50m 但不与任何 cluster 匹配, 都非常可疑
        if (cand_valid) {
            double min_dist = 1e18;
            uint64_t min_id = 0;
            for (const auto& kv : clusters_) {
                const TargetCluster& c = kv.second;
                if (c.label != label) continue;
                double d = std::hypot(cand_tx - c.ned_x, cand_ty - c.ned_y);
                if (d < min_dist) { min_dist = d; min_id = c.id; }
            }
            if (min_id != 0 && min_dist > std::max(50.0, merge_radius_m_ * 2.0)) {
                ROS_ERROR_THROTTLE(2.0,
                    "[ProjDiag] STEP6 raw projection far from same-label cluster (%.1fm, threshold %.1fm) | "
                    "label=%s cand=(%.1f,%.1f) nearest_cluster_id=%lu pos=(%.1f,%.1f) | "
                    "Possible: R/target_z_prior_/cam_pos issue, but each step is self-consistent -> error accumulates",
                    min_dist, std::max(50.0, merge_radius_m_ * 2.0),
                    label.c_str(), cand_tx, cand_ty, min_id,
                    clusters_[min_id].ned_x, clusters_[min_id].ned_y);
            }
        }

        // 投影失败 (射线水平 / 朝后) — 既无法匹配已有 cluster 也无法新建,直接丢弃该 detection
        if (!cand_valid) return;

        if (nearest_id != 0) {
            // === 2026-08-23: 替换 2D EMA 为 4-state KF (Plan B + KF) ===
            //   静目标 (Q_vel→0): KF 速度衰减到 0, 等效位置 EMA, 行为零回归
            //   动目标 (Q_vel=2.0): KF 跟踪速度, lead pose 让 chase 不滞后
            TargetCluster& c = clusters_[nearest_id];
            updateClusterKF(c, cand_tx, cand_ty, observation_time);
            // 把 KF 输出写回 cluster 字段 (下游继续用 c.ned_x/y)
            const auto state = c.kf_.getState();
            c.ned_x = state.x;
            c.ned_y = state.y;
            c.ned_alt = cand_tz;
            c.best_confidence = std::max(c.best_confidence, det.confidence);
            c.last_seen_us = observation_stamp_us;
            //===================制导修改====================
            updateClusterImageObservation(c, det, raw_cx, raw_cy, matched_camera_pose);
            //===================制导修改====================
            // === 2026-07-16: 命中计数 + 晋升门禁 ===
            // === 2026-07-31: Attack 阶段降低 hit 门槛 ===
            //   is_attack_active_ 由 mission_state_cb 维护 (skill_type==105 && phase 4/5)
            //   搜索阶段仍走原 min_hit_count_ 抗误检
            c.hit_count++;
            const int cur_min_hit = is_attack_active_ ? attack_min_hit_count_ : min_hit_count_;
            if (!c.is_promoted && static_cast<int>(c.hit_count) >= cur_min_hit) {
                c.is_promoted = true;
                emitClusterEvent(c);
            }
            // === 2026-08-20: 仅在新检测置信度高于历史最佳时覆盖 img_data ===
            //   聚类过程中目标可能在视野边缘 → 半身子被 YOLO 识别 → 置信度偏低,
            //   之前的全身子高分图被覆盖丢失,GS 显示的就是半拉身子。
            //   改成只在 det.confidence > c.best_confidence 时覆盖,聚类结束后
            //   c.img_data 自然停留在整个生命周期内看到过的最清晰那张。
            //   GS 上行靠 mission_manager 的 reported_to_gs_ dedup 防止重复刷图。
            if (!det.img_data.empty() && det.confidence > c.best_confidence) {
                c.img_data   = det.img_data;
                c.img_format = 1;  // JPEG(与 gimbal_simulator 输出一致)
                ROS_INFO_THROTTLE(2.0,
                    "[TargetClusterer] Cluster[%lu] label=%s image upgraded "
                    "(conf %.2f -> %.2f, img_bytes=%zu)",
                    c.id, c.label.c_str(),
                    c.best_confidence, det.confidence, det.img_data.size());
            }
            return;
        }

        // 新建 cluster:cand 是同一帧的原始投影,已经有效,直接用
        TargetCluster nc;
        nc.id              = ++next_cluster_id_;
        nc.label           = label;
        nc.label_hash      = hashLabel(label);
        nc.ned_x           = cand_tx;
        nc.ned_y           = cand_ty;
        nc.ned_alt         = cand_tz;
        nc.world_ema_valid = true;
        // === 2026-08-23: KF 首帧 init (替代 world_ema_valid=true) ===
        nc.kf_.init(cand_tx, cand_ty);
        nc.kf_last_time_         = observation_time;
        nc.kf_last_predict_time_ = observation_time;  // === 2026-08-24: 高频 KF predict 同步时间戳 ===
        nc.kf_init_time_         = observation_time;
        nc.motion_class_   = MotionClass::UNKNOWN;  // grace 期内强制 STATIC
        nc.best_confidence = det.confidence;
        nc.first_seen_us   = observation_stamp_us;
        nc.last_seen_us    = observation_stamp_us;
        //===================制导修改====================
        updateClusterImageObservation(nc, det, raw_cx, raw_cy, matched_camera_pose);
        //===================制导修改====================
        nc.img_data        = det.img_data;
        nc.img_format      = det.img_data.empty() ? 0 : 1;
        // === 2026-07-16: 首次进入 tentative 态 ===
        nc.hit_count       = 1;
        nc.is_promoted     = false;
        clusters_[nc.id]   = nc;

        ROS_INFO_THROTTLE(1.0, "[TargetClusterer] NEW tentative id=%lu label=%s "
                              "(nwu=%.1f,%.1f,%.1f conf=%.2f hit=%u/%d, total=%zu)",
                           nc.id, nc.label.c_str(), nc.ned_x, nc.ned_y, nc.ned_alt,
                           nc.best_confidence, nc.hit_count, min_hit_count_,
                           clusters_.size());

        // 若 min_hit_count_ ≤ 1 (关闭门禁),立即晋升并发 event
        //   === 2026-07-31: Attack 阶段门槛也参与判断 ===
        const int cur_min_hit_new = is_attack_active_ ? attack_min_hit_count_ : min_hit_count_;
        if (cur_min_hit_new <= 1) {
            clusters_[nc.id].is_promoted = true;
            emitClusterEvent(clusters_[nc.id]);
        }
    }

    // ClusterEvent stores the promoted target position in NWU coordinates.
    // Each cluster emits this event once, when it is first promoted.
    void emitClusterEvent(const TargetCluster& c) {
        multi_uav_strike::ClusterEvent ev;
        ev.cluster_id    = c.id;
        ev.label_hash    = c.label_hash;
        ev.label         = c.label;
        ev.confidence    = c.best_confidence;
        ev.first_seen_us = c.first_seen_us;
        ev.nwu_x         = c.ned_x;
        ev.nwu_y         = c.ned_y;
        ev.nwu_alt       = c.ned_alt;
        ev.gps_valid     = clusterToGps(
            c, ev.gps_latitude, ev.gps_longitude, ev.gps_altitude);
        cluster_event_pub_.publish(ev);

        ROS_INFO("[TargetClusterer] PROMOTE cluster id=%lu label=%s "
                 "(nwu=%.1f,%.1f,%.1f gps=%s[%.7f,%.7f,%.1f] "
                 "conf=%.2f hit=%u)",
                 c.id, c.label.c_str(), c.ned_x, c.ned_y, c.ned_alt,
                 ev.gps_valid ? "valid" : "invalid",
                 ev.gps_latitude, ev.gps_longitude, ev.gps_altitude,
                 c.best_confidence, c.hit_count);
    }

    // 从 gimbal_pose 提取当前云台俯仰(q = setRPY(0, pitch, 0))
    void gimbalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 注意: gimbal_pose 的 orientation 已经是 uav_quat * body_cam_quat 形式
        // (见 gimbal_simulator_node.cpp 中 publishGimbalPose 的实现: 直接用
        //  setRPY(0, current_gimbal_pitch_, 0) 写入 orientation, 与 publishCameraPose
        //  不同; 此处采用与 forward projection 一致的 +pitch 约定)。
        // 我们需要从 orientation 反解 pitch 用于构造 R_world_cam。
        tf::Quaternion q(msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z,
                         msg->pose.orientation.w);
        double r, p, y;
        tf::Matrix3x3(q).getRPY(r, p, y);
        current_gimbal_pitch_ = p;
        is_gimbal_pose_received_ = true;
    }

    // === 2026-07-16: 接收稳像 camera/pose 作为反投影唯一真源 ===
    // 不再做 uav_quat * setRPY(pitch) 重建,直接用 gimbal 算好的稳像世界 quat,
    // 保证前向/反向投影用同一个 R_world_cam,且完全不依赖 UAV roll/机体 pitch。
    void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        CameraPoseSample sample;
        sample.stamp = msg->header.stamp;
        if (sample.stamp.isZero()) {
            sample.stamp = ros::Time::now();
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] camera/pose has zero timestamp; using receive time");
        }
        sample.position_nwu = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);
        sample.orientation_world_camera = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        if (sample.orientation_world_camera.norm() < 1e-9) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] camera/pose has invalid quaternion; sample ignored");
            return;
        }
        sample.orientation_world_camera.normalize();

        {
            std::lock_guard<std::mutex> lock(camera_pose_buffer_mtx_);
            if (camera_pose_buffer_.empty() ||
                sample.stamp >= camera_pose_buffer_.back().stamp) {
                camera_pose_buffer_.push_back(sample);
            } else {
                const auto pos = std::lower_bound(
                    camera_pose_buffer_.begin(), camera_pose_buffer_.end(), sample.stamp,
                    [](const CameraPoseSample& lhs, const ros::Time& rhs) {
                        return lhs.stamp < rhs;
                    });
                camera_pose_buffer_.insert(pos, sample);
            }

            const ros::Time newest_stamp = camera_pose_buffer_.back().stamp;
            while (camera_pose_buffer_.size() > 1 &&
                   (newest_stamp - camera_pose_buffer_.front().stamp).toSec() >
                       pose_buffer_duration_sec_) {
                camera_pose_buffer_.pop_front();
            }
        }
        is_camera_pose_received_ = true;
    }

    // Select the camera pose with the smallest absolute timestamp delta.
    // Equal deltas intentionally choose the earlier sample. Time delta is
    // diagnostic only and never rejects a detection.
    bool findNearestCameraPose(const multi_uav_strike::YoloDetection& det,
                               CameraPoseSample& selected) {
        std::lock_guard<std::mutex> lock(camera_pose_buffer_mtx_);
        if (camera_pose_buffer_.empty()) return false;

        ros::Time detection_stamp;
        if (det.stamp_us == 0) {
            selected = camera_pose_buffer_.back();
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Detection frame_seq=%u has zero timestamp; "
                "using latest camera pose t=%.6f",
                det.frame_seq, selected.stamp.toSec());
            return true;
        }
        detection_stamp.fromNSec(det.stamp_us * 1000ULL);

        const auto next = std::lower_bound(
            camera_pose_buffer_.begin(), camera_pose_buffer_.end(), detection_stamp,
            [](const CameraPoseSample& lhs, const ros::Time& rhs) {
                return lhs.stamp < rhs;
            });

        if (next == camera_pose_buffer_.begin()) {
            selected = *next;
        } else if (next == camera_pose_buffer_.end()) {
            selected = camera_pose_buffer_.back();
        } else {
            const auto prev = std::prev(next);
            const double prev_delta = std::fabs((detection_stamp - prev->stamp).toSec());
            const double next_delta = std::fabs((next->stamp - detection_stamp).toSec());
            selected = (prev_delta <= next_delta) ? *prev : *next;
        }

        const double selected_delta =
            std::fabs((selected.stamp - detection_stamp).toSec());
        if (selected_delta > pose_sync_warn_threshold_sec_) {
            ROS_WARN_THROTTLE(1.0,
                "[TargetClusterer] Large detection/pose time delta: frame_seq=%u "
                "det=%.6f selected=%.6f delta=%.3fs buffer=[%.6f, %.6f] "
                "(observation still accepted)",
                det.frame_seq, detection_stamp.toSec(), selected.stamp.toSec(),
                selected_delta, camera_pose_buffer_.front().stamp.toSec(),
                camera_pose_buffer_.back().stamp.toSec());
        }
        return true;
    }

    static ros::Time detectionStamp(
        const multi_uav_strike::YoloDetection& det) {
        ros::Time stamp;
        if (det.stamp_us != 0) stamp.fromNSec(det.stamp_us * 1000ULL);
        return stamp;
    }

    //=============一定打击=========//
    void updateFirstWaypointLatch() {
        if (search_area_first_waypoint_reached_ || !current_skill_executing_ ||
            (current_skill_type_ != 102 && current_skill_type_ != 105) ||
            !waypoint_status_received_) {
            return;
        }
        if (latest_waypoint_status_.skill_id != current_skill_id_) return;
        if (latest_waypoint_status_.skill_total > 0 &&
            latest_waypoint_status_.skill_idx >= 1) {
            search_area_first_waypoint_reached_ = true;
            cluster_skill_active_ = true;
            // skill 102:搜索区第一个航点到达 (开始允许聚类)
            // skill 105:WP1 到达 (skill_idx 0=WP1, 1=WP2;idx>=1 表示已过 WP1)
            //   起飞直接发 105 场景下,没有 102 提前开 gate,
            //   必须靠 WP1 到达来开,否则 guidance 全程没 cluster 输入。
            const char* tag = (current_skill_type_ == 102) ? "Search" : "Attack";
            ROS_INFO("[TargetClusterer] %s-area first-waypoint clustering gate opened: "
                     "skill_id=%s skill_idx=%u/%u",
                     tag,
                     current_skill_id_.c_str(),
                     latest_waypoint_status_.skill_idx,
                     latest_waypoint_status_.skill_total);
        }
    }

    bool isFirstWaypointGateOpen() const {
        if (!mission_state_received_ || !cluster_skill_active_) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Skip YOLO batch: search-area gate closed");
            return false;
        }
        if (!search_area_first_waypoint_reached_) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Skip YOLO batch: Search(102) area first waypoint not reached");
            return false;
        }
        return true;
    }
    //=============一定打击=========//

    void resetYawRateBaseline(const ros::Time& stamp, double yaw) {
        previous_yaw_stamp_ = stamp;
        previous_yaw_rad_ = yaw;
        yaw_baseline_valid_ = !stamp.isZero();
        std::lock_guard<std::mutex> lock(yaw_rate_buffer_mtx_);
        yaw_rate_buffer_.clear();
    }

    void updateYawRateSample() {
        ros::Time stamp = current_uav_pose_.header.stamp;
        const auto& q_msg = current_uav_pose_.pose.orientation;
        Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        if (stamp.isZero() || !std::isfinite(q.norm()) || q.norm() < 1e-9) {
            yaw_baseline_valid_ = false;
            std::lock_guard<std::mutex> lock(yaw_rate_buffer_mtx_);
            yaw_rate_buffer_.clear();
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Reset yaw-rate baseline: invalid UAV pose stamp/quaternion");
            return;
        }
        q.normalize();
        const double yaw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        if (!yaw_baseline_valid_) {
            resetYawRateBaseline(stamp, yaw);
            return;
        }

        const double dt = (stamp - previous_yaw_stamp_).toSec();
        if (!std::isfinite(dt) || dt <= 0.0 ||
            dt > yaw_rate_sample_max_dt_sec_) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Reset yaw-rate baseline: invalid pose dt=%.3fs",
                dt);
            resetYawRateBaseline(stamp, yaw);
            return;
        }

        // atan2(sin, cos) gives the shortest angular difference across +/-pi.
        const double raw_delta = yaw - previous_yaw_rad_;
        const double yaw_delta = std::atan2(std::sin(raw_delta), std::cos(raw_delta));
        YawRateSample sample;
        sample.stamp = stamp;
        sample.yaw_rate_rad_s = yaw_delta / dt;
        previous_yaw_stamp_ = stamp;
        previous_yaw_rad_ = yaw;

        std::lock_guard<std::mutex> lock(yaw_rate_buffer_mtx_);
        yaw_rate_buffer_.push_back(sample);
        while (yaw_rate_buffer_.size() > 1 &&
               (sample.stamp - yaw_rate_buffer_.front().stamp).toSec() >
                   pose_buffer_duration_sec_) {
            yaw_rate_buffer_.pop_front();
        }
    }

    bool findNearestYawRate(const multi_uav_strike::YoloDetection& det,
                            YawRateSample& selected) {
        std::lock_guard<std::mutex> lock(yaw_rate_buffer_mtx_);
        if (yaw_rate_buffer_.empty()) return false;
        if (det.stamp_us == 0) {
            selected = yaw_rate_buffer_.back();
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Detection frame_seq=%u has zero timestamp; "
                "using latest yaw-rate sample t=%.6f",
                det.frame_seq, selected.stamp.toSec());
            return true;
        }

        const ros::Time detection_stamp = detectionStamp(det);
        const auto next = std::lower_bound(
            yaw_rate_buffer_.begin(), yaw_rate_buffer_.end(), detection_stamp,
            [](const YawRateSample& lhs, const ros::Time& rhs) {
                return lhs.stamp < rhs;
            });
        if (next == yaw_rate_buffer_.begin()) {
            selected = *next;
        } else if (next == yaw_rate_buffer_.end()) {
            selected = yaw_rate_buffer_.back();
        } else {
            const auto prev = std::prev(next);
            const double prev_delta =
                std::fabs((detection_stamp - prev->stamp).toSec());
            const double next_delta =
                std::fabs((next->stamp - detection_stamp).toSec());
            selected = (prev_delta <= next_delta) ? *prev : *next;
        }
        return true;
    }

    void primaryTargetCallback(const std_msgs::UInt64::ConstPtr& msg) {
        primary_cluster_id_ = msg->data;
        ROS_INFO_THROTTLE(2.0, "[TargetClusterer] primary_target → cluster_id=%lu",
                          primary_cluster_id_);
    }

    // === 2026-07-31: 订阅 mission/mission_state 维护 Attack 模式位 ===
    //   判定条件(2026-07-31 修正):
    //     skill_type == 105 && state == 1 (EXECUTING)
    //   === 2026-07-31 修正原因 ===
    //   旧版用 phase == GUIDANCE_TRACK(4) / GUIDANCE_STRIKE(5) 触发, 但整个
    //   WP1→WP2 飞行段 mission_manager 报的 phase 一直是 3 (WAYPOINT_FOLLOW),
    //   只在 strike 真正触发后才进 4/5 — 时序错位, Attack 阶段降门槛从未生效。
    //   改用 state==1 (EXECUTING) 后, IN_TASK 整个段 (phase 2/3/4/5 都算) 都激活降门槛,
    //   Cluster 能在 0.6s 内 promote, 而不是等 2s 攒到 20 hit 时已经 stale。
    void missionStateCallback(const multi_uav_strike::MissionState::ConstPtr& msg) {
        mission_state_received_ = true;
        current_skill_flow_id_ = msg->skill_flow_id;
        current_skill_id_ = msg->skill_id;
        current_skill_type_ = msg->skill_type;
        current_skill_executing_ = (msg->state == 1 /* EXECUTING */);
        //=============一定打击=========//
        // Only a newly activated Search(102) defines a new clustering session.
        // Attack(105) consumes and keeps updating that session; it must neither
        // clear clusters nor impose its own first-waypoint gate.
        const bool search_executing = current_skill_executing_ &&
            current_skill_type_ == 102;
        cluster_skill_active_ = search_executing ||
            (search_area_first_waypoint_reached_ &&
             current_skill_executing_ && current_skill_type_ == 105);

        // Clear once when a new Search key first becomes active. A pause/resume
        // of the same key keeps the current search area's clusters.
        // next_cluster_id_ remains monotonic so cleared IDs are never reused.
        const bool new_skill_activation = search_executing &&
            (!activated_skill_key_valid_ ||
             current_skill_id_ != activated_skill_id_ ||
             current_skill_flow_id_ != activated_skill_flow_id_);
        if (new_skill_activation) {
            activated_skill_key_valid_ = true;
            activated_skill_id_ = current_skill_id_;
            activated_skill_flow_id_ = current_skill_flow_id_;
            search_area_first_waypoint_reached_ = false;
            cluster_skill_active_ = true;
            size_t cleared_count = 0;
            {
                std::lock_guard<std::mutex> lock(clusters_mtx_);
                cleared_count = clusters_.size();
                clusters_.clear();
                primary_cluster_id_ = 0;
            }
            multi_uav_strike::ClusterState empty_state;
            cluster_states_pub_.publish(empty_state);
            publishEmptyGps();
            ROS_INFO("[TargetClusterer] New Search area activated: "
                     "flow_id=%s skill_id=%s type=%d; cleared %zu clusters",
                     current_skill_flow_id_.c_str(), current_skill_id_.c_str(),
                     current_skill_type_, cleared_count);
        }
        //=============一定打击=========//

        updateFirstWaypointLatch();
        const bool is_105       = (msg->skill_type == 105);
        const bool is_executing = (msg->state == 1 /*EXECUTING*/);
        is_attack_active_ = is_105 && is_executing;
        ROS_INFO_THROTTLE(5.0,
            "[TargetClusterer] mission_state: skill_type=%d state=%d phase=%d → is_attack_active=%s",
            msg->skill_type, msg->state, msg->phase, is_attack_active_ ? "true" : "false");
    }

    // === 2026-08-20: 接收 Attack(105) 原 WP2 GPS (latched 一次) ===
    //   用于主目标 fallback: 没显式 primary_target 时,选 cluster 中离这个 GPS 最近的。
    void attackWp2Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(2.0, "[TargetClusterer] attack_wp2_gps: NavSatFix status < FIX, ignore");
            return;
        }
        attack_wp2_gps_      = *msg;
        attack_wp2_received_ = true;
        if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude)) {
            attack_wp2_gps_valid_ = true;
            ROS_WARN_THROTTLE(5.0,
                "[TargetClusterer] attack_wp2_gps received: lat=%.7f lon=%.7f alt=%.2f "
                "(will pick closest cluster for primary fallback)",
                msg->latitude, msg->longitude, msg->altitude);
        } else {
            attack_wp2_gps_valid_ = false;
            ROS_WARN_THROTTLE(2.0, "[TargetClusterer] attack_wp2_gps: non-finite lat/lon, ignore");
        }
    }

    // === 2026-08-24: 订阅 /guidance/mode — TRACK 模式禁用 YOLO yaw_rate 门控 ===
    //   跟踪动目标时 chase yaw_rate 持续偏高 (0.27~0.46 rad/s),超过默认
    //   max_cluster_yaw_rate_rad_s=0.2 → YOLO batch 持续被丢 → cluster stale →
    //   primary 切换 → chase 输出衰减 → 跟丢。
    //   修法:mode=="track" 时 guidance_mode_is_track_=true,yoloBatchCallback
    //   跳过 yaw_rate 检查。waypoint/SEARCH/strike 阶段 mode="strike" 或
    //   未启动时,门控照常生效,避免 yaw 突变带来的 bbox 抖动被聚类。
    void guidanceModeCallback(const std_msgs::String::ConstPtr& msg) {
        const bool now_track = (msg->data == "track");
        if (now_track != guidance_mode_is_track_) {
            ROS_WARN("[TargetClusterer] guidance_mode -> '%s' → YOLO yaw_rate gate %s",
                     msg->data.c_str(),
                     now_track ? "BYPASSED (track mode)" : "ACTIVE (waypoint/search)");
            guidance_mode_is_track_ = now_track;
        }
        current_guidance_mode_ = msg->data;
    }

    // === 2026-08-24: 订阅 /guidance/enable — 用于 marker 颜色分级 ===
    //   enable=true && mode in {track,strike} → primary 是 UAV 当前 chase/strike 的目标
    //   → 显示橙色大球;否则 primary 只是 GS 候选,显示紫色。
    void guidanceEnableCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data != is_guidance_enable_) {
            ROS_WARN("[TargetClusterer] guidance/enable -> %s → primary marker %s",
                     msg->data ? "TRUE" : "FALSE",
                     msg->data ? "ORANGE (tracked)" : "PURPLE (candidate)");
            is_guidance_enable_ = msg->data;
        }
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            current_uav_pose_.pose.position.x =  msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;
            current_uav_pose_.pose.orientation.w =  msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x =  msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // 位置: ENU → NWU
            current_uav_pose_.pose.position.x =  msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z =  msg->pose.position.z;
            // 四元数: mavros(ENU 世界 + FRD 机体) → 内部(NWU 世界 + FLU 机体)
            //   q_internal = q_W · q_mavros · q_B
            //   q_W = (√2/2, 0, 0, -√2/2)  ENU→NWU: 绕世界 Z 旋转 -90°
            //   q_B = (0, 1, 0, 0)          FRD→FLU: 绕机体 X 旋转 180°
            // 与 gimbal_simulator_node.cpp::uavPoseCallback 完全一致,确保 target_estimator
            // 反投影用的 R_world_cam 与 gimbal forward projection 用的 R_world_cam 一致。
            // 旧版简单取反 (x, y, z, w)→(x, -y, -z, w) 是绕世界 X 转 180°,不是 ENU→NWU,
            // 会让 projectBboxToGround 的射线方向错误,导致 cluster 永远建不出来。
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_uav_pose_.pose.orientation.w = -kSqrtHalf * (qe.x + qe.y);
            current_uav_pose_.pose.orientation.x =  kSqrtHalf * (qe.w + qe.z);
            current_uav_pose_.pose.orientation.y =  kSqrtHalf * (qe.z - qe.w);
            current_uav_pose_.pose.orientation.z =  kSqrtHalf * (qe.x - qe.y);
        }
        current_uav_pose_.header.stamp = msg->header.stamp;
        is_uav_pose_received_ = true;
        updateYawRateSample();

        // === 2026-08-23: 缓存 UAV 速度 (pose-delta + 一阶 IIR 平滑) ===
        //   computeClosingSpeed 用了这个,给 lead_pose 算 horiz/closing_speed。
        //   一阶 IIR (uav_velocity_lpf_alpha_) 把位姿 jitter 滤掉;
        //   50Hz mavros pose → 0.5 alpha 时间常数 ≈ 0.02s,响应够用且不抖。
        if (last_uav_velocity_stamp_.isValid() &&
            last_uav_velocity_stamp_ != msg->header.stamp) {
            const double dt = (msg->header.stamp - last_uav_velocity_stamp_).toSec();
            if (dt > 1e-4 && dt < 1.0) {
                const Eigen::Vector3d pos(
                    current_uav_pose_.pose.position.x,
                    current_uav_pose_.pose.position.y,
                    current_uav_pose_.pose.position.z);
                const Eigen::Vector3d inst_v = (pos - last_uav_velocity_pos_) / dt;
                const double a = uav_velocity_lpf_alpha_;
                current_uav_velocity_nwu_ = a * inst_v + (1.0 - a) * current_uav_velocity_nwu_;
            }
        }
        last_uav_velocity_pos_ = Eigen::Vector3d(
            current_uav_pose_.pose.position.x,
            current_uav_pose_.pose.position.y,
            current_uav_pose_.pose.position.z);
        last_uav_velocity_stamp_ = msg->header.stamp;
    }

    void selfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            current_gps_ = *msg;
            is_gps_received_ = true;
        }
    }

    // === 2026-07-28: PX4 home GPS 锁一次,作为 cluster 绝对 GPS 输出的参考原点 ===
    //   PX4 的 LOCAL_POSITION_NED 在启动时由 EKF2 锁定 origin,后续 home 更新不会重置 NED,
    //   此处只锁一次避免和后续 ground_station 地图使用的 home 原点不一致。
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (is_home_locked_) return;
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            return;  // PX4 home 未稳定前发 0/0,忽略
        }
        home_gps_.latitude  = msg->geo.latitude;
        home_gps_.longitude = msg->geo.longitude;
        home_gps_.altitude  = msg->geo.altitude;
        home_gps_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
        home_gps_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        is_home_locked_ = true;
        ROS_WARN_THROTTLE(5.0, "[TargetClusterer] PX4 home GPS locked: (%.7f, %.7f, %.2f)",
                          home_gps_.latitude, home_gps_.longitude, home_gps_.altitude);
    }

    // ============ 周期任务 ============

    // 10Hz:publish 所有 alive cluster 快照 + 可视化 + primary GPS / pose
    void statesTimerCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> g(clusters_mtx_);
        if (clusters_.empty()) {
            multi_uav_strike::ClusterState empty_state;
            cluster_states_pub_.publish(empty_state);
            publishEmptyGps();
            return;
        }

        multi_uav_strike::ClusterState state;
        visualization_msgs::MarkerArray markers;
        state.targets.reserve(clusters_.size());
        markers.markers.reserve(clusters_.size());

        // === 2026-08-23: classifyMotion 先跑一遍 (mutex 已锁, mutate 安全) ===
        const ros::Time now_stamp = ros::Time::now();
        for (auto& kv : clusters_) {
            classifyMotion(kv.second, now_stamp);
        }

        // === 2026-07-16: state.targets 仍只发 promoted (下游 API) ===
        //   markers 同时画 promoted + tentative (RViz 可视化区分)
        for (const auto& kv : clusters_) {
            const TargetCluster& c = kv.second;

            // === 2026-08-23: KF 集成后清理 (合并重复 if,原版双层 if (c.is_promoted) 是死代码) ===
            //   原版 promoted/tentative 各一个 if + else,经 iter_07 编辑丢失 } 闭合,这里合成单层。
            if (c.is_promoted) {
                multi_uav_strike::ClusterTarget t;
                t.cluster_id    = c.id;
                t.label         = c.label;
                t.label_hash    = c.label_hash;
                t.ned_x         = c.ned_x;
                t.ned_y         = c.ned_y;
                t.ned_alt       = c.ned_alt;
                t.confidence    = c.best_confidence;
                t.last_seen_us  = c.last_seen_us;
                t.img_data      = c.img_data;
                t.img_format    = c.img_format;
                state.targets.push_back(t);
            }

            // 标记: tentative (黄色扁环) + promoted (绿色实心球) 都画
            markers.markers.push_back(buildMarker(c));

            // promoted 额外画一根 5m 立柱(ns 不同, id 同 c.id),任何视角都显眼
            if (c.is_promoted) {
                markers.markers.push_back(buildPillar(c));
            }
        }
        cluster_states_pub_.publish(state);
        target_est_marker_pub_.publish(markers);

        // primary 选择:
//   1) 优先 primary_cluster_id_ (GS 或 mission_manager 显式指定)
//   2) fallback 在 Attack 阶段:用 attack_wp2 GPS (latched) 选最近 cluster
//      (替代旧的"最高置信度" — bag 02-11-07 实测 3 cluster 时挑错,
//       因为最高置信度可能不是 GS 推的 WP2 处那个)
//   3) 其他情况:最高置信度 (旧行为)
        const TargetCluster* primary = nullptr;
        if (primary_cluster_id_ != 0) {
            auto it = clusters_.find(primary_cluster_id_);
            if (it != clusters_.end() && it->second.is_promoted) {
                // === 2026-08-24: stale_freeze 配套 — primary cluster stale 时不锁 ===
                //   lsu_age > cluster_stale_freeze_sec_ 的 cluster KF 已 reset velocity,
                //   位置冻结在最后观测点,再 chase 只会追历史位置 → 鬼影。
                //   这里把 primary_cluster_id_ 清空, 让下面 fallback 重新选最高置信度。
                if (it->second.last_seen_us != 0) {
                    const double lsu_age = (now_stamp.toNSec() / 1000ULL -
                                            it->second.last_seen_us) * 1e-6;
                    if (lsu_age > cluster_stale_freeze_sec_) {
                        ROS_WARN_THROTTLE(2.0,
                            "[TargetClusterer] Primary cluster %lu stale (age=%.1fs > %.1fs), "
                            "release lock and fallback to highest confidence",
                            it->second.id, lsu_age, cluster_stale_freeze_sec_);
                        primary_cluster_id_ = 0;  // 清掉, 让 GS 重新指定或 fallback
                    } else {
                        primary = &it->second;
                    }
                } else {
                    primary = &it->second;
                }
            }
        }
        if (primary == nullptr) {
            bool used_wp2_fallback = false;
            if (is_attack_active_ && attack_wp2_gps_valid_ && is_home_locked_) {
                // WP2 GPS → NWU(相对 home,与 cluster.ned_x/ned_y 同系)
                //   c.ned_y 是 NWU y = west_m(正值=西)
                //   GPS lon delta → east_m(正值=东),转换:wp2_nwu_y = -east_m
                //   bag 02-25-41 复盘:符号反了导致 cluster 3 (N130 E100, WP2 处) 距离
                //   被算成 200m,所有 cluster 越界 sanity range,fallback 回最高置信度。
                constexpr double DEG_PER_M_LAT = 1.0 / 111320.0;
                const double cos_lat = std::cos(home_gps_.latitude * M_PI / 180.0);
                if (std::isfinite(cos_lat) && std::fabs(cos_lat) > 1e-9) {
                    const double deg_per_m_lon = DEG_PER_M_LAT / cos_lat;
                    const double wp2_n = (attack_wp2_gps_.latitude - home_gps_.latitude) /
                                          DEG_PER_M_LAT;
                    const double wp2_east_m = (attack_wp2_gps_.longitude - home_gps_.longitude) /
                                              deg_per_m_lon;
                    const double wp2_nwu_y = -wp2_east_m;  // NWU y = west_m
                    constexpr double kWp2SanityRange = 100.0;  // m
                    const TargetCluster* closest = nullptr;
                    double closest_dist_sq = std::numeric_limits<double>::infinity();
                    for (const auto& kv : clusters_) {
                        const TargetCluster& c = kv.second;
                        if (!c.is_promoted) continue;
                        // === 2026-08-24: stale promoted cluster 不参与 fallback 候选 ===
                        //   uav0_strike_2026-08-24-17-04-42_0.bag 复盘:primary 9 stale 后
                        //   primary_cluster_id_ 清零,fallback 仍按"最高 confidence"选 stale
                        //   cluster,chase 拿去追 KF 冻结位置 → 鬼影。这里跟 line 1123 一致
                        //   用 cluster_stale_freeze_sec_ 阈值过滤掉 stale 候选。
                        if (c.last_seen_us != 0) {
                            const double lsu_age = (now_stamp.toNSec() / 1000ULL -
                                                    c.last_seen_us) * 1e-6;
                            if (lsu_age > cluster_stale_freeze_sec_) continue;
                        }
                        const double dx = c.ned_x - wp2_n;
                        const double dy = c.ned_y - wp2_nwu_y;
                        const double dist_sq = dx*dx + dy*dy;
                        if (dist_sq < closest_dist_sq) {
                            closest_dist_sq = dist_sq;
                            closest = &c;
                        }
                    }
                    if (closest != nullptr &&
                        closest_dist_sq < kWp2SanityRange * kWp2SanityRange) {
                        primary = closest;
                        used_wp2_fallback = true;
                        ROS_INFO_THROTTLE(2.0,
                            "[TargetClusterer] Primary fallback: closest-to-WP2 cluster_id=%lu "
                            "(dist=%.1fm, WP2 lat=%.7f lon=%.7f)",
                            closest->id, std::sqrt(closest_dist_sq),
                            attack_wp2_gps_.latitude, attack_wp2_gps_.longitude);
                    } else {
                        ROS_WARN_THROTTLE(5.0,
                            "[TargetClusterer] No cluster within %.0fm of Attack WP2 GPS, "
                            "falling back to highest-confidence",
                            kWp2SanityRange);
                    }
                }
            }
            if (primary == nullptr) {
                for (const auto& kv : clusters_) {
                    const TargetCluster& c = kv.second;
                    if (!c.is_promoted) continue;
                    // === 2026-08-24: stale promoted cluster 不参与 fallback 候选 ===
                    //   同 WP2 fallback:stale cluster KF velocity 已 reset,位置冻结,
                    //   当 fallback primary 会被 chase 当活动目标追 → 鬼影。
                    if (c.last_seen_us != 0) {
                        const double lsu_age = (now_stamp.toNSec() / 1000ULL -
                                                c.last_seen_us) * 1e-6;
                        if (lsu_age > cluster_stale_freeze_sec_) continue;
                    }
                    if (primary == nullptr || c.best_confidence > primary->best_confidence) {
                        primary = &c;
                    }
                }
            }
        }
        if (primary != nullptr) {
            publishPrimaryGps(*primary);
            publishPrimaryPose(*primary);
            // === 2026-08-23: KF 速度 (动目标 lead pose 配套) ===
            publishPrimaryTwist(*primary);
            //===================制导修改====================
            publishPrimaryImageTarget(*primary);
            //===================制导修改====================
        }
    }

    // 1Hz:清理超 TTL 的 cluster
    void cleanupTimerCb(const ros::TimerEvent&) {
        ros::Time now = ros::Time::now();
        std::lock_guard<std::mutex> g(clusters_mtx_);
        for (auto it = clusters_.begin(); it != clusters_.end(); ) {
            const auto& c = it->second;
            ros::Time last_seen;
            last_seen.fromNSec(c.last_seen_us * 1000ULL);
            double age = (now - last_seen).toSec();
            // === 2026-07-16: tentative 用更短 TTL,promoted 用 cluster_ttl_sec_ ===
            // === 2026-07-31: Attack 阶段 tentative 用更长 TTL,防短停留目标被提前 DROP ===
            double ttl;
            if (c.is_promoted) {
                ttl = cluster_ttl_sec_;
            } else {
                ttl = is_attack_active_ ? attack_tentative_ttl_sec_ : tentative_ttl_sec_;
            }
            if (age > ttl) {
                if (c.is_promoted) {
                    ROS_INFO("[TargetClusterer] EXPIRED cluster id=%lu label=%s "
                             "(alive %.1fs)", c.id, c.label.c_str(), age);
                } else {
                    ROS_INFO("[TargetClusterer] DROP tentative id=%lu label=%s "
                             "(hit=%u/%d, age=%.2fs)", c.id, c.label.c_str(),
                             c.hit_count, min_hit_count_, age);
                }
                it = clusters_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    // ============ 数据成员 ============
    ros::NodeHandle nh_;
    bool use_sim_ = true;
    std::string uav_pose_topic_;

    double merge_radius_m_   = 10.0;
    double cluster_ttl_sec_  = 60.0;
    int    min_hit_count_    = 20;      // 同 cluster 被检测到这么多次才晋升
    double tentative_ttl_sec_ = 1.0;    // 未晋升的 tentative 在这么久没刷新就丢弃
    // === 2026-07-31: Attack 阶段专享聚类门槛 ===
    int    attack_min_hit_count_     = 6;
    double attack_tentative_ttl_sec_ = 2.0;
    bool   is_attack_active_         = false;  // mission_state_cb 维护
    double ema_alpha_        = 0.3;
    double target_z_prior_   = 0.0;
    double fallback_proj_distance_ = 30.0;  // 2026-08-13: iter_03 — 水平相机 fallback 距离
    double states_publish_freq_ = 10.0;
    double cleanup_freq_     = 1.0;
    double est_marker_r_ = 1.0, est_marker_g_ = 1.0, est_marker_b_ = 0.0, est_marker_a_ = 1.0;

    // === 2026-08-23: 动目标 KF + 模式分类 yaml 参数 ===
    //   cold-start grace: 聚类创建后 N 秒内强制 STATIC, 防初始噪声
    double motion_cold_start_grace_sec_ = 1.0;
    //   迟滞门槛
    double motion_speed_enter_  = 2.0;   // |v| > enter 连续 enter_frames → DYNAMIC
    double motion_speed_exit_   = 0.3;   // |v| < exit  连续 exit_frames  → STATIC
    int    motion_enter_frames_ = 3;
    int    motion_exit_frames_  = 30;
    //   lead_time 上下限
    double motion_lead_min_ = 0.2;
    double motion_lead_max_ = 1.5;
    //   发散保护
    double motion_max_speed_ = 30.0;    // |v| > max → resetVelocity + STATIC
    //   KF 过程/测量噪声
    double kf_q_pos_         = 1.0;
    double kf_q_vel_static_  = 0.01;    // 静目标:速度衰减到 0 (零回归)
    double kf_q_vel_dynamic_ = 2.0;     // 动目标:跟踪真速度
    double kf_r_meas_        = 2.0;     // YOLO 世界投影噪声 std
    //   UAV 速度缓存 LPF (computeClosingSpeed 用)
    double uav_velocity_lpf_alpha_ = 0.5;
    //   KF predict 频率
    double kf_predict_freq_ = 50.0;
    //   stale_freeze 阈值 (lsu_age > sec → 强制 STATIC + reset velocity)
    double cluster_stale_freeze_sec_ = 3.0;

    // 相机几何
    int    image_width_  = 1920;
    int    image_height_ = 1080;
    double fov_h_deg_    = 68.0;
    double fov_v_deg_    = 45.6;
    double fov_h_rad_ = 0.0, fov_v_rad_ = 0.0;
    double focal_x_ = 0.0, focal_y_ = 0.0;

    ros::Subscriber yolo_batch_sub_;
    ros::Subscriber gimbal_pose_sub_;
    ros::Subscriber camera_pose_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber self_gps_sub_;
    ros::Subscriber home_position_sub_;
    ros::Subscriber primary_target_sub_;
    ros::Subscriber mission_state_sub_;
    ros::Subscriber waypoint_status_sub_;
    ros::Subscriber guidance_mode_sub_;             // /guidance/mode → TRACK 时绕过 yaw_rate 门控
    ros::Subscriber guidance_enable_sub_;            // /guidance/enable → 区分 tracked (橙) vs primary (紫)

    ros::Publisher cluster_event_pub_;
    ros::Publisher cluster_states_pub_;
    ros::Publisher target_est_pose_pub_;
    ros::Publisher target_est_marker_pub_;
    ros::Publisher raw_det_marker_pub_;          // 单帧识别 (红色点)
    ros::Publisher target_est_gps_pub_;
    ros::Subscriber attack_wp2_sub_;             // /attack_wp2_gps → Attack 阶段原 WP2 GPS (NavSatFix, latched)
    //===================制导修改====================
    ros::Publisher primary_image_target_pub_;
    //===================制导修改====================
    // === 2026-08-23: 激活死 topic — KF 速度 (动目标 lead-bearing 用) ===
    ros::Publisher target_est_twist_pub_;
    std::string viz_frame_;                      // RViz 帧名 "<ns>/map"(多机分离由 static TF 偏移)
    ros::Timer states_timer_;
    ros::Timer cleanup_timer_;
    // === 2026-08-24: KF 高频 predict (50Hz), 解决动目标出 FOV 位置冻结 ===
    ros::Timer kf_predict_timer_;

    geometry_msgs::PoseStamped  current_uav_pose_;
    // === 2026-08-23: UAV 速度缓存 (pose-delta + 一阶 IIR LPF), computeClosingSpeed 用 ===
    //   uavPoseCallback 每帧算 inst_v = (pos - last) / dt,再 LPF 平滑。
    //   50Hz mavros pose + alpha=0.5 → 时间常数 ≈ 0.02s,够跟手又不抖。
    Eigen::Vector3d             current_uav_velocity_nwu_{0.0, 0.0, 0.0};
    Eigen::Vector3d             last_uav_velocity_pos_{0.0, 0.0, 0.0};
    ros::Time                   last_uav_velocity_stamp_{0, 0};
    sensor_msgs::NavSatFix      current_gps_;
    sensor_msgs::NavSatFix      home_gps_;     // PX4 home, 锁一次作为 GPS 输出的参考原点
    bool                       is_home_locked_ = false;
    // === 2026-08-20: Attack 阶段原 WP2 GPS (GS 推的 cluster 位置) ===
    //   由 mission_manager 在 Attack(105) push 时一次性 latched publish,
    //   主目标选取 fallback 用:在 primary_target topic 没人推时,优先选 cluster
    //   中离 attack_wp2_gps_ 最近的(而不是"最高置信度")。
    sensor_msgs::NavSatFix      attack_wp2_gps_;
    bool                        attack_wp2_received_ = false;
    bool                        attack_wp2_gps_valid_ = false;
    double                      current_gimbal_pitch_ = 0.0;
    bool is_uav_pose_received_    = false;
    bool is_gps_received_         = false;
    bool is_gimbal_pose_received_ = false;
    bool is_camera_pose_received_ = false;

    double pose_buffer_duration_sec_ = 5.0;
    double pose_sync_warn_threshold_sec_ = 0.1;
    std::mutex camera_pose_buffer_mtx_;
    std::deque<CameraPoseSample> camera_pose_buffer_;

    double max_cluster_yaw_rate_rad_s_ = 0.2;
    double yaw_rate_sample_max_dt_sec_ = 0.5;
    std::mutex yaw_rate_buffer_mtx_;
    std::deque<YawRateSample> yaw_rate_buffer_;
    bool yaw_baseline_valid_ = false;
    ros::Time previous_yaw_stamp_;
    double previous_yaw_rad_ = 0.0;

    bool mission_state_received_ = false;
    bool waypoint_status_received_ = false;
    bool current_skill_executing_ = false;
    bool cluster_skill_active_ = false;
    // === 2026-08-24: TRACK 模式旁路 YOLO yaw_rate 门控 ===
    //   guidance/mode=="track" 时 chase yaw_rate 通常 0.27~0.46 rad/s,超过默认
    //   max_cluster_yaw_rate_rad_s=0.2 → YOLO batch 持续被丢 → cluster stale →
    //   primary 切换 → chase 输出衰减 → 跟丢。TRACK 时直接绕过门控,
    //   waypoint/SEARCH/未启用阶段保留门控(避免搜索阶段 yaw 突变带来的 bbox 抖动被聚类)。
    bool guidance_mode_is_track_ = false;
    // === 2026-08-24: marker 颜色分级依据 — 是否在 chase/strike ===
    //   guidance/enable=true && mode in {track,strike} → primary 是 UAV 当前 chase/strike 的目标
    //   → 显示橙色大球;否则 primary 只是 GS 候选,显示紫色中等球
    bool is_guidance_enable_      = false;
    std::string current_guidance_mode_ = "";
    //=============一定打击=========//
    bool search_area_first_waypoint_reached_ = false;
    //=============一定打击=========//
    int32_t current_skill_type_ = 0;
    std::string current_skill_flow_id_;
    std::string current_skill_id_;
    multi_uav_strike::WaypointStatus latest_waypoint_status_;
    bool activated_skill_key_valid_ = false;
    std::string activated_skill_flow_id_;
    std::string activated_skill_id_;

    uint64_t primary_cluster_id_ = 0;   // 0 = mission 未指定, 回退最高置信度

    std::mutex clusters_mtx_;
    std::unordered_map<uint64_t, TargetCluster> clusters_;
    uint64_t next_cluster_id_ = 0;
    int     raw_marker_seq_    = 0;   // raw_detection_marker id 计数器, MarkerArray 同 ns 不能重复

    //===================制导修改====================
    Eigen::Vector3d pixelWorldRay(double cx, double cy,
                                  const CameraPoseSample& camera_pose) const {
        const double u_ndc = (cx - image_width_  / 2.0) / focal_x_;
        const double v_ndc = (cy - image_height_ / 2.0) / focal_y_;
        Eigen::Vector3d cam_ray(-v_ndc, u_ndc, 1.0);
        cam_ray.normalize();
        Eigen::Vector3d ray =
            camera_pose.orientation_world_camera.toRotationMatrix() * cam_ray;
        if (ray.norm() > 1e-6) ray.normalize();
        return ray;
    }

    void waypointStatusCallback(
        const multi_uav_strike::WaypointStatus::ConstPtr& msg) {
        latest_waypoint_status_ = *msg;
        waypoint_status_received_ = true;
        updateFirstWaypointLatch();
    }

    void updateClusterImageObservation(TargetCluster& c,
                                       const multi_uav_strike::YoloDetection& det,
                                       double raw_cx, double raw_cy,
                                       const CameraPoseSample& camera_pose) {
        c.image_frame_seq = det.frame_seq;
        c.image_center_x = static_cast<float>(raw_cx / image_width_);
        c.image_center_y = static_cast<float>(raw_cy / image_height_);
        c.image_width = det.x_max - det.x_min;
        c.image_height = det.y_max - det.y_min;
        c.image_world_ray_nwu = pixelWorldRay(raw_cx, raw_cy, camera_pose);
    }

    void publishPrimaryImageTarget(const TargetCluster& c) {
        multi_uav_strike::PrimaryImageTarget msg;
        msg.header.stamp.fromNSec(c.last_seen_us * 1000ULL);
        msg.header.frame_id = viz_frame_;
        msg.frame_seq = c.image_frame_seq;
        msg.stamp_us = c.last_seen_us;
        msg.cluster_id = c.id;
        msg.label = c.label;
        msg.confidence = c.best_confidence;
        msg.center_x = c.image_center_x;
        msg.center_y = c.image_center_y;
        msg.width = c.image_width;
        msg.height = c.image_height;
        msg.world_ray_nwu.x = c.image_world_ray_nwu.x();
        msg.world_ray_nwu.y = c.image_world_ray_nwu.y();
        msg.world_ray_nwu.z = c.image_world_ray_nwu.z();
        const double age = (ros::Time::now().toNSec() / 1000ULL - c.last_seen_us) * 1e-6;
        msg.visible = c.is_promoted && age >= 0.0 && age <= 0.5;
        primary_image_target_pub_.publish(msg);
    }
    //===================制导修改====================

    // === 2026-08-23: KF predict + update 一站式包装 ===
    //   Q_vel 根据 motion_class_ 自适应:
    //     - STATIC  : Q_vel=0.01 → 速度衰减到 0, 等效 EMA, 零回归
    //     - DYNAMIC : Q_vel=2.0  → 跟踪真速度
    //     - UNKNOWN : 用 Q_vel_static 兜底(grace 期内强制 STATIC, 这里 UNKNOWN 实际上不会触发)
    void updateClusterKF(TargetCluster& c,
                         double meas_x, double meas_y,
                         const ros::Time& observation_time) {
        if (!c.kf_.isInitialized()) {
            c.kf_.init(meas_x, meas_y);
            c.kf_last_time_         = observation_time;
            c.kf_last_predict_time_ = observation_time;
            c.kf_init_time_         = observation_time;
            c.motion_class_         = MotionClass::UNKNOWN;
            return;
        }
        // === 2026-08-24: 桥接 timer predict → observation_time 的小间隔 ===
        //   timer 在 kf_last_predict_time_ → now 之间持续推进 KF;
        //   YOLO 到达时 KF 状态还停在 kf_last_predict_time_,
        //   补一个 dt = observation_time - kf_last_predict_time_ 的 predict
        //   让 update() 在最新状态上做 (避免 YOLO 帧边界上的 0~20ms 滞后)。
        const double bridge_dt = std::max(0.0,
            (observation_time - c.kf_last_predict_time_).toSec());
        if (bridge_dt > 0.0) {
            const double q_vel = (c.motion_class_ == MotionClass::DYNAMIC) ?
                                  kf_q_vel_dynamic_ : kf_q_vel_static_;
            c.kf_.predict(bridge_dt, kf_q_pos_, q_vel);
        }
        c.kf_.update(meas_x, meas_y, kf_r_meas_);
        c.kf_last_time_         = observation_time;
        c.kf_last_predict_time_ = observation_time;
    }

    // === 2026-08-24: KF 高频 predict timer (默认 50Hz) ===
    //   在 YOLO 检测之间持续推进 KF 状态, 让 c.ned_x/y 跟随 KF 速度连续外推。
    //   这是治本:动目标出 FOV 后 KF 不再冻结, chase 的 lead pose 反映真实预测位置,
    //   UAV 不会停在"最后看到的位置"傻等。
    //   STATIC cluster Q_vel=0.01 衰减速度, 预测对位置影响极小, 安全。
    //   DYNAMIC cluster Q_vel=2.0, KF 状态主动外推, 与 YOLO 再次检测时无缝衔接。
    //   写回 c.ned_x/y: 让 processSingleDetection 的 merge_radius 匹配也用最新位置。
    void kfPredictTimerCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> g(clusters_mtx_);
        const ros::Time now = ros::Time::now();
        for (auto& kv : clusters_) {
            TargetCluster& c = kv.second;
            if (!c.kf_.isInitialized()) continue;
            const double dt = (now - c.kf_last_predict_time_).toSec();
            // 跳过过小或负 dt (时钟回退 / timer 与 updateClusterKF 同步竞争)
            if (dt <= 0.001) continue;
            const double q_vel = (c.motion_class_ == MotionClass::DYNAMIC) ?
                                  kf_q_vel_dynamic_ : kf_q_vel_static_;
            c.kf_.predict(dt, kf_q_pos_, q_vel);
            c.kf_last_predict_time_ = now;
            // 把 KF 当前位置写回 cluster.ned_x/y — 这是 merge 匹配用的字段
            const auto state = c.kf_.getState();
            c.ned_x = state.x;
            c.ned_y = state.y;
        }
    }

    // === 2026-08-23: 模式分类 (迟滞 + cold-start grace + 发散保护) ===
    //   - cold-start grace: 聚类创建后 grace_sec 秒内强制 STATIC, 防初始噪声
    //   - |v| > enter 连续 enter_frames 帧 → DYNAMIC (向上单向)
    //   - |v| < exit  连续 exit_frames  帧 → STATIC  (向下单向, 不再升 DYNAMIC)
    //   - |v| > max_speed (发散保护) → reset velocity, 强制 STATIC
    // === 2026-08-24: stale_freeze ===
    //   - lsu_age > cluster_stale_freeze_sec_ → 强制 STATIC + resetVelocity
    //     KF 不再外推 stale 位置
    void classifyMotion(TargetCluster& c, const ros::Time& now) {
        if (!c.kf_.isInitialized()) {
            c.motion_class_ = MotionClass::UNKNOWN;
            return;
        }

        // === stale_freeze: 长时间无新观测视为 stale, KF 不再外推 ===
        if (c.last_seen_us != 0) {
            const double lsu_age =
                (now.toNSec() / 1000ULL - c.last_seen_us) * 1e-6;
            if (lsu_age > cluster_stale_freeze_sec_) {
                // 只有从非 STATIC → STATIC 时打日志, 避免每 100ms 重复打
                if (c.motion_class_ != MotionClass::STATIC) {
                    ROS_WARN_THROTTLE(2.0,
                        "[TargetClusterer] Cluster[%lu] stale (age=%.1fs > %.1fs), "
                        "force STATIC + reset velocity (chase 解锁)",
                        c.id, lsu_age, cluster_stale_freeze_sec_);
                }
                c.motion_class_           = MotionClass::STATIC;
                c.motion_dynamic_evidence_ = 0;
                c.motion_static_evidence_  = 0;
                c.kf_.resetVelocity();
                return;
            }
        }

        // cold-start grace: 1s 内强制 STATIC
        const double age = (now - c.kf_init_time_).toSec();
        if (age < motion_cold_start_grace_sec_) {
            c.motion_class_ = MotionClass::STATIC;
            c.motion_dynamic_evidence_ = 0;
            return;
        }

        const double v = c.kf_.getSpeed();

        // 发散保护: 速度超阈值时 reset velocity + 强制 STATIC
        if (v > motion_max_speed_) {
            ROS_WARN_THROTTLE(2.0,
                "[TargetClusterer] Cluster[%lu] KF speed %.1f > max %.1f, reset velocity",
                c.id, v, motion_max_speed_);
            c.kf_.resetVelocity();
            c.motion_class_ = MotionClass::STATIC;
            c.motion_dynamic_evidence_ = 0;
            return;
        }

        // 迟滞
        if (c.motion_class_ != MotionClass::DYNAMIC) {
            // STATIC / UNKNOWN → 检查是否升 DYNAMIC
            if (v > motion_speed_enter_) {
                c.motion_dynamic_evidence_++;
                if (c.motion_dynamic_evidence_ >= motion_enter_frames_) {
                    c.motion_class_        = MotionClass::DYNAMIC;
                    c.motion_static_evidence_ = 0;
                    ROS_INFO("[TargetClusterer] Cluster[%lu] → DYNAMIC (v=%.2f m/s)",
                             c.id, v);
                }
            } else {
                c.motion_dynamic_evidence_ = 0;
            }
        } else {
            // DYNAMIC → 检查是否降 STATIC (单向)
            if (v < motion_speed_exit_) {
                c.motion_static_evidence_++;
                if (c.motion_static_evidence_ >= motion_exit_frames_) {
                    c.motion_class_        = MotionClass::STATIC;
                    c.motion_dynamic_evidence_ = 0;
                    ROS_INFO("[TargetClusterer] Cluster[%lu] → STATIC (v=%.2f m/s)",
                             c.id, v);
                }
            } else {
                c.motion_static_evidence_ = 0;
            }
        }
    }

    // === 2026-08-23: 收拢速度 (UAV→target 方向上的投影) ===
    //   用于 lead_time = clamp(horiz / closing_speed, lead_min, lead_max)
    //   闭环:closing_speed 大 → lead 小 → UAV 不冲过头;closing_speed 小 → lead 大 → 提前补偿
    double computeClosingSpeed(const TargetCluster& c) const {
        if (!c.kf_.isInitialized()) return 1.0;
        const double dx = c.ned_x - current_uav_pose_.pose.position.x;
        const double dy = c.ned_y - current_uav_pose_.pose.position.y;
        const double horiz = std::hypot(dx, dy);
        if (horiz < 0.1) return 1.0;
        const double ux = dx / horiz, uy = dy / horiz;
        const auto st = c.kf_.getState();
        //   UAV 速度 - 目标速度 在 UAV→target 单位向量上的投影 = 正值 = 收拢
        const double uav_vx = current_uav_velocity_nwu_.x();
        const double uav_vy = current_uav_velocity_nwu_.y();
        const double rel_vx = uav_vx - st.vx;
        const double rel_vy = uav_vy - st.vy;
        const double closing = rel_vx * ux + rel_vy * uy;
        return std::max(0.5, closing);
    }

    // === 2026-08-23: KF 速度发布 (动目标 lead pose 配套, 激活死 topic) ===
    void publishPrimaryTwist(const TargetCluster& c) {
        if (!c.kf_.isInitialized()) return;
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = viz_frame_;
        const auto st = c.kf_.getState();
        msg.twist.linear.x  = st.vx;
        msg.twist.linear.y  = st.vy;
        msg.twist.linear.z  = 0.0;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;
        target_est_twist_pub_.publish(msg);
    }

    // ============ 工具 ============

    void computeFocalLengths() {
        fov_h_rad_ = fov_h_deg_ * M_PI / 180.0;
        if (fov_v_deg_ > 0.0) {
            fov_v_rad_ = fov_v_deg_ * M_PI / 180.0;
        } else {
            double aspect = static_cast<double>(image_width_) / image_height_;
            fov_v_rad_ = 2.0 * std::atan(std::tan(fov_h_rad_ / 2.0) / aspect);
        }
        focal_x_ = (image_width_  / 2.0) / std::tan(fov_h_rad_ / 2.0);
        focal_y_ = (image_height_ / 2.0) / std::tan(fov_v_rad_ / 2.0);
    }

    static uint32_t hashLabel(const std::string& s) {
        // FNV-1a 32-bit:足够防撞,非密码学场景
        uint32_t h = 2166136261u;
        for (unsigned char c : s) { h ^= c; h *= 16777619u; }
        return h;
    }

    // === 2026-08-19: 数字→语义 归一化 (对齐 gimbal_node.cpp:1461 +1 偏移协议) ===
    //   YOLO 真机 (starling_gimbal) C++ 端 class_id 映射 (gimbal_node.cpp L710):
    //     class_id=0 → person, class_id=1 → car, class_id=2 → other (过滤丢弃)
    //   publishYoloDetections (gimbal_node.cpp:1461) 推到 ROS 前做了 +1 偏移:
    //     class_id=0 (person) → "1", class_id=1 (car) → "2"
    //   仿真 gimbal_simulator 直接发语义字符串 ("person"/"car"/...)
    //   target_estimator 内部统一用语义字符串, 聚类匹配不再受部署源影响
    //   协议(与 gimbal_node.cpp +1 偏移对齐):
    //     "1"            → "person"
    //     "2"            → "car"
    //     "3" 及以上数字 → "car"  // 兜底: 防止 3+ 漏到下游被误判 unknown
    //   "0"             → 原样返回 (防御: 旧版/未来版本可能不再 +1, 直接是 class_id)
    //   其他字符串 → 原样返回(已语义,例如 "person"/"car"/"truck")
    static std::string normalizeYoloLabel(const std::string& raw) {
        if (raw == "1") return "person";
        if (raw == "2") return "car";
        // 3+ 数字串兜底为 car(防 YOLO 漏报 1/2 时被当 unknown 误过滤)
        if (raw.size() >= 1 && raw[0] >= '3' && raw[0] <= '9' &&
            std::all_of(raw.begin(), raw.end(),
                        [](unsigned char c){ return std::isdigit(c); })) {
            return "car";
        }
        // "0" 或语义串: 原样返回
        return raw;
    }

    // Per-detection bbox 反投影: 与 gimbal 的 forward projection 完全互逆。
    //   1) bbox 中心像素 → (u_ndc, v_ndc)
    //   2) 相机射线方向 (cam frame, +z = 光学轴):  t_cam ∝ (-v_ndc, u_ndc, 1)
    //   3) world 射线 = R_world_cam * t_cam (R_world_cam 来自 gimbal 的稳像 camera/pose)
    //   4) 射线与 z = target_z_prior_ 平面求交 → 目标世界 (NWU) 坐标
    bool projectBboxToGround(const multi_uav_strike::YoloDetection& det,
                             const CameraPoseSample& camera_pose,
                             double& tx, double& ty, double& tz) {
        double cx = (det.x_min + det.x_max) * 0.5 * image_width_;
        double cy = (det.y_min + det.y_max) * 0.5 * image_height_;
        return projectPixelToGround(cx, cy, camera_pose, tx, ty, tz);
    }

    // === 2026-07-16: 像素级 helper (Plan A 用) ===
    //   processSingleDetection 先在像素空间做 EMA,再调本函数投影到世界。
    //   几何部分与 projectBboxToGround 完全相同,只是输入换成已平滑的像素。
    bool projectPixelToGround(double cx, double cy,
                              const CameraPoseSample& camera_pose,
                              double& tx, double& ty, double& tz) {
        // ---- Step 1: 像素 → 相机帧射线 ----
        double u_ndc = (cx - image_width_  / 2.0) / focal_x_;
        double v_ndc = (cy - image_height_ / 2.0) / focal_y_;
        Eigen::Vector3d cam_ray(-v_ndc, u_ndc, 1.0);
        cam_ray.normalize();

        // ---- Step 2: 相机射线 → 世界射线 (用稳像 camera/pose) ----
        Eigen::Matrix3d R_world_cam =
            camera_pose.orientation_world_camera.toRotationMatrix();
        Eigen::Vector3d opt_axis = R_world_cam.col(2);  // 光轴 (cam +z) 世界方向
        Eigen::Vector3d world_ray = R_world_cam * cam_ray;

        // ---- Step 3: 与地面平面求交 ----
        // 相机刚性安装在 UAV 上,cam_pos 直接取 UAV 位置(ENU→NWU 已转换)。
        // camera_pose 仅用作 orientation_world_camera (光轴方向),不读 position。
        Eigen::Vector3d cam_pos(current_uav_pose_.pose.position.x,
                                current_uav_pose_.pose.position.y,
                                current_uav_pose_.pose.position.z);
        double dz = target_z_prior_ - cam_pos.z();

        bool proj_ok = true;
        double t = 0.0;
        Eigen::Vector3d target_world = cam_pos;
        if (std::fabs(world_ray.z()) < 1e-3) {
            // === 2026-08-13: iter_03 — 射线几乎水平(如云台没朝下),fallback 到固定距离
            //   旧版直接 proj_ok=false → cluster 落 (0,0,0) → 全部堆 home。
            //   真实场景里云台物理上必须朝下,光轴水平本身就是异常,但不应阻挡聚类
            //   (任务目标可能在视野里,只是 z 坐标落到 0 平面无解而已)。
            //   fallback 距离 = merge 的视野最远端 — 典型 car 30m 左右,选择 30m。
            //   投影到 ray 方向上,得到一个"30m 外沿射线方向"的 cluster 点;
            //   后续 merge / cluster_event 仍正常,只是 z 坐标不再是 target_z_prior_。
            proj_ok = false;
            t = fallback_proj_distance_;  // 沿射线方向走 fallback_proj_distance_ 米
            target_world = cam_pos + t * world_ray;
        } else {
            t = dz / world_ray.z();
            if (t < 0.0) {
                // 射线指向远离地面(z 朝向天空,与 target_z_prior_ plane 反向)
                // fallback 到固定距离 — 同水平处理
                proj_ok = false;
                t = fallback_proj_distance_;
                target_world = cam_pos + t * world_ray;
            } else {
                target_world = cam_pos + t * world_ray;
            }
        }

        if (proj_ok) {
            tx = target_world.x();
            ty = target_world.y();
            tz = target_z_prior_;
        } else {
            // === 2026-08-13: iter_03 — fallback 模式仍返回有效坐标 ===
            tx = target_world.x();
            ty = target_world.y();
            // tz = cam_pos.z() (射线没碰到地面,目标在 UAV 高度的 30m 外)
            tz = target_world.z();
        }

        // ---- Step 4: 反推诊断 ----
        diagnoseProjection(cx, cy, u_ndc, v_ndc, cam_ray,
                           R_world_cam, opt_axis, world_ray,
                           camera_pose.orientation_world_camera,
                           cam_pos, dz, t, target_world, proj_ok);

        // === 2026-08-13: iter_03 — 任何 camera 角度都给出一个 3D 坐标 (fallback 已写在
        //   tx/ty/tz),聚类 caller 应该接受。proj_ok 仅作诊断保留,不再用来过滤 detection。
        return true;
    }

    // === 2026-07-16: 投影反推诊断 ===
    // 检查顺序与投影链路一致: 像素→cam_ray→光轴→world_ray→cam_pos→target
    // 任一步骤几何不自洽就 ROS_ERROR_THROTTLE (1Hz) 打印, 红色字体终端可见。
    void diagnoseProjection(double cx, double cy,
                            double u_ndc, double v_ndc,
                            const Eigen::Vector3d& cam_ray,
                            const Eigen::Matrix3d& R_world_cam,
                            const Eigen::Vector3d& opt_axis,
                            const Eigen::Vector3d& world_ray,
                            const Eigen::Quaterniond& camera_quat,
                            const Eigen::Vector3d& cam_pos,
                            double dz, double t,
                            const Eigen::Vector3d& target_world,
                            bool proj_ok) {
        // (1) cam_ray.z 应 > 0.4: 射线方向接近光轴, 不然像素严重偏离或 u_ndc/v_ndc 量级错
        if (cam_ray.z() < 0.4) {
            ROS_ERROR_THROTTLE(1.0,
                "[ProjDiag] STEP1 cam_ray.z=%.3f (<0.4) | "
                "cx=%.0f cy=%.0f u_ndc=%.3f v_ndc=%.3f | "
                "Possible: pixel too far from optical axis, or u_ndc/v_ndc not normalized to tan(FOV/2) ~ %.3f",
                cam_ray.z(), cx, cy, u_ndc, v_ndc,
                std::tan(fov_h_rad_ / 2.0));
            return;
        }

        // (2) Optical axis direction: when target is below, axis z must be < 0
        if (opt_axis.z() > -0.05) {
            ROS_ERROR_THROTTLE(1.0,
                "[ProjDiag] STEP2 optical axis opt.z=%.3f (>=0) | "
                "Axis points UP! Check uav_yaw extraction / gimbal_pitch sign / R construction | "
                "cam quat (wxyz)=(%.3f,%.3f,%.3f,%.3f) | "
                "R.col(2)=(%.3f,%.3f,%.3f)",
                opt_axis.z(),
                camera_quat.w(), camera_quat.x(),
                camera_quat.y(), camera_quat.z(),
                opt_axis.x(), opt_axis.y(), opt_axis.z());
            return;
        }

        // (3) world_ray.z should be < 0 (ray hits ground)
        if (world_ray.z() > 0.0) {
            ROS_ERROR_THROTTLE(1.0,
                "[ProjDiag] STEP3 world_ray.z=%.3f > 0 (ray points UP) | "
                "Axis points DOWN but ray points UP, sign of cam_ray.x or v_ndc may be flipped | "
                "cam_ray=(%.3f,%.3f,%.3f) R=\n"
                "[%.3f %.3f %.3f\n %.3f %.3f %.3f\n %.3f %.3f %.3f]",
                world_ray.z(),
                cam_ray.x(), cam_ray.y(), cam_ray.z(),
                R_world_cam(0,0), R_world_cam(0,1), R_world_cam(0,2),
                R_world_cam(1,0), R_world_cam(1,1), R_world_cam(1,2),
                R_world_cam(2,0), R_world_cam(2,1), R_world_cam(2,2));
            return;
        }

        if (!proj_ok) {
            // world_ray.z near 0 or t < 0 already caught above; here proj_ok=false but ray points down -> extreme geometry
            ROS_ERROR_THROTTLE(1.0,
                "[ProjDiag] STEP3b projection degenerated (proj_ok=false) | "
                "world_ray.z=%.3f dz=%.2f cam_z=%.2f tgt_z_prior=%.2f",
                world_ray.z(), dz, cam_pos.z(), target_z_prior_);
            return;
        }

        // (4) Horizontal projection range sanity: for downward-looking camera, horizontal range should be < cam_z * 20 (~ 87 deg off-axis)
        double horiz = std::hypot(target_world.x() - cam_pos.x(),
                                  target_world.y() - cam_pos.y());
        if (horiz > cam_pos.z() * 20.0 && cam_pos.z() > 5.0) {
            ROS_ERROR_THROTTLE(1.0,
                "[ProjDiag] STEP4 horizontal projection %.1fm abnormally large (cam_z=%.1fm cap ~%.0fm) | "
                "cam=(%.1f,%.1f,%.1f) tgt=(%.1f,%.1f,%.1f) | "
                "t=%.1f dz=%.1f u_ndc=%.3f v_ndc=%.3f | "
                "Possible: one of target_z_prior_/cam_pos/world_ray is wrong",
                horiz, cam_pos.z(), cam_pos.z() * 20.0,
                cam_pos.x(), cam_pos.y(), cam_pos.z(),
                target_world.x(), target_world.y(), target_world.z(),
                t, dz, u_ndc, v_ndc);
            return;
        }

        // (5) Center pixel consistency: when cx/cy ~ image center, target should be on the bearing directly below cam
        //   |u_ndc| < 0.05 and |v_ndc| < 0.05 -> check ratio of horizontal range to cam_z
        if (std::fabs(u_ndc) < 0.05 && std::fabs(v_ndc) < 0.05 && cam_pos.z() > 5.0) {
            // Center-pixel ray ~ optical-axis direction, horizontal range should ~ cam_z * tan(elevation)
            // elevation = asin(-opt_axis.z()); for downward look: tan(elev) = -opt_axis.z / sqrt(1-opt_axis.z^2)
            double horiz_z = std::sqrt(std::max(0.0, 1.0 - opt_axis.z() * opt_axis.z()));
            double expected_horiz = cam_pos.z() * horiz_z / (-opt_axis.z());
            // 50% tolerance
            if (horiz > expected_horiz * 1.5 + 5.0) {
                ROS_ERROR_THROTTLE(2.0,
                    "[ProjDiag] STEP5 center-pixel consistency | "
                    "actual horizontal %.1fm vs expected %.1fm (delta %.1fm) | "
                    "cam=(%.1f,%.1f,%.1f) tgt=(%.1f,%.1f,%.1f) opt.z=%.3f",
                    horiz, expected_horiz, horiz - expected_horiz,
                    cam_pos.x(), cam_pos.y(), cam_pos.z(),
                    target_world.x(), target_world.y(), target_world.z(),
                    opt_axis.z());
                return;
            }
        }
        // 全部检查通过 -> 不打印
    }

    visualization_msgs::Marker buildMarker(const TargetCluster& c) {
        // === 2026-08-24: 四色四档显示,方便 RViz 一眼区分 ===
        //   1) tracked/attack target (橙): UAV 当前 chase/strike 的目标 → 大球 8m
        //      = primary_cluster_id && (enable && mode in {track,strike})
        //   2) primary candidate (紫): GS 指定或最高置信度,但 UAV 还没接管 → 中球 7m
        //      = primary_cluster_id && !tracked
        //   3) other promoted (绿): 候选但不是 primary → 球 6m
        //   4) tentative (黄): 还在聚类,未晋升 → 4m 扁盘
        // 同一 ns "clusters" + 同一 id (c.id),状态切换时下一帧自动覆盖。
        visualization_msgs::Marker m;
        m.header.frame_id = viz_frame_;
        m.header.stamp    = ros::Time::now();
        m.ns   = "clusters";
        m.id   = static_cast<int>(c.id);
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.ned_x;
        m.pose.position.y = c.ned_y;
        m.pose.position.z = c.ned_alt + 0.3;
        m.pose.orientation.w = 1.0;
        m.lifetime = ros::Duration(0.5);

        const bool is_primary_cluster =
            (primary_cluster_id_ != 0 && c.id == primary_cluster_id_);
        const bool is_tracked_now =
            is_primary_cluster && is_guidance_enable_ &&
            (current_guidance_mode_ == "track" ||
             current_guidance_mode_ == "strike");

        // === 2026-08-24: stale cluster 视觉降权 (alpha 衰减) ===
        //   uav0_strike_2026-08-24-17-04-42_0.bag 复盘:RViz 一堆绿球干扰判读。
        //   promoted 但 lsu_age > cluster_stale_freeze_sec_ 的 cluster KF velocity
        //   已 reset、位置冻结,不再是真目标,只在视觉上 faded 保留(确认它在哪儿)。
        //   阈值与 line 1123 / fallback (新增) 共用,语义一致。
        bool is_stale = false;
        if (c.last_seen_us != 0) {
            const double lsu_age = (ros::Time::now().toNSec() / 1000ULL -
                                    c.last_seen_us) * 1e-6;
            if (lsu_age > cluster_stale_freeze_sec_) is_stale = true;
        }
        const float alpha_fresh = 0.95f;
        const float alpha_stale = 0.30f;  // stale promoted → faded (不影响 chase,仅视觉)
        const float alpha = is_stale ? alpha_stale : alpha_fresh;

        if (c.is_promoted && is_tracked_now) {
            // === 1) tracked/attack target: 橙色大球 (直径 24m,远超橙紫区分阈值) ===
            //   用户口径: "把橙色和紫色球都放大,直径20以上";24m 让目标在 RViz
            //   远距也能一眼看到,不与其他绿/黄小球混淆。
            m.type = visualization_msgs::Marker::SPHERE;
            m.scale.x = 24.0; m.scale.y = 24.0; m.scale.z = 24.0;
            m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.0f; m.color.a = alpha;
        } else if (c.is_promoted && is_primary_cluster) {
            // === 2) primary candidate: 紫色大球 (直径 22m,稍小于橙以保持等级感) ===
            m.type = visualization_msgs::Marker::SPHERE;
            m.scale.x = 22.0; m.scale.y = 22.0; m.scale.z = 22.0;
            m.color.r = 0.6f; m.color.g = 0.2f; m.color.b = 0.8f; m.color.a = alpha;
        } else if (c.is_promoted) {
            // === 3) other promoted: 绿色球 (stale 后 alpha 0.30 faded) ===
            m.type = visualization_msgs::Marker::SPHERE;
            m.scale.x = 6.0; m.scale.y = 6.0; m.scale.z = 6.0;
            m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = alpha;
        } else {
            // === 4) tentative: 黄色扁盘 (CYLINDER + 小高度 = 地面圆环),半透明 ===
            m.type = visualization_msgs::Marker::CYLINDER;
            m.scale.x = 4.0; m.scale.y = 0.2; m.scale.z = 4.0;
            m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.55f;
        }
        return m;
    }

    // === 2026-08-24: 立柱同步四色,任何角度都能一眼看到 ===
    //   tracked/attack = 橙色 24m 立柱;primary = 紫色 22m;other promoted = 绿色 5m;tentative 不画。
    //   立柱高度跟球直径对齐,确保球被云台/角度挡住时仍能通过立柱识别。
    visualization_msgs::Marker buildPillar(const TargetCluster& c) {
        visualization_msgs::Marker m;
        m.header.frame_id = viz_frame_;
        m.header.stamp    = ros::Time::now();
        m.ns   = "cluster_pillars";
        m.id   = static_cast<int>(c.id);
        m.type = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.ned_x;
        m.pose.position.y = c.ned_y;
        m.pose.orientation.w = 1.0;
        m.lifetime = ros::Duration(0.5);

        const bool is_primary_cluster =
            (primary_cluster_id_ != 0 && c.id == primary_cluster_id_);
        const bool is_tracked_now =
            is_primary_cluster && is_guidance_enable_ &&
            (current_guidance_mode_ == "track" ||
             current_guidance_mode_ == "strike");

        // === 2026-08-24: stale cluster 立柱同步 faded,与 buildMarker 配色逻辑对齐 ===
        bool is_stale = false;
        if (c.last_seen_us != 0) {
            const double lsu_age = (ros::Time::now().toNSec() / 1000ULL -
                                    c.last_seen_us) * 1e-6;
            if (lsu_age > cluster_stale_freeze_sec_) is_stale = true;
        }
        const float alpha_fresh = 0.9f;
        const float alpha_stale = 0.25f;  // stale 立柱基本透明,让 fresh 立柱显眼
        const float alpha = is_stale ? alpha_stale : alpha_fresh;

        if (is_tracked_now) {
            // 橙色 24m 立柱 (球直径对齐)
            m.pose.position.z = c.ned_alt + 12.0;
            m.scale.x = 0.7; m.scale.y = 24.0; m.scale.z = 0.7;
            m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.0f; m.color.a = alpha;
        } else if (is_primary_cluster) {
            // 紫色 22m 立柱
            m.pose.position.z = c.ned_alt + 11.0;
            m.scale.x = 0.6; m.scale.y = 22.0; m.scale.z = 0.6;
            m.color.r = 0.6f; m.color.g = 0.2f; m.color.b = 0.8f; m.color.a = alpha;
        } else {
            // 其他 promoted: 绿色 5m 立柱(stale 后 alpha 0.25 faded)
            m.pose.position.z = c.ned_alt + 2.5;
            m.scale.x = 0.5; m.scale.y = 5.0; m.scale.z = 0.5;
            m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 0.0f; m.color.a = alpha;
        }
        return m;
    }

    // === 2026-07-16: 单帧识别投影标记 (红色小点,瞬时闪现)===
    //   每条 YoloDetection 都发一个,0.3s 后自动消失。
    //   直接拿原始像素反投影(cand_tx/ty),不参与 EMA/聚类,完全反映
    //   当帧探测器说"这里有个东西"的瞬时位置。
    void publishRawDetectionMarker(double x, double y, double z, const std::string& label) {
        visualization_msgs::MarkerArray arr;
        visualization_msgs::Marker m;
        m.header.frame_id = viz_frame_;
        m.header.stamp    = ros::Time::now();
        m.ns   = "raw_detections";
        m.id   = ++raw_marker_seq_;     // 全局唯一,MarkerArray 同 ns 下不能重复
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.x = 10; m.scale.y = 10; m.scale.z = 10;  // 巨大红色原始识别点(放大到 0.6m,跟 3.5m 球/5m 柱更搭)
        m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.9f;  // 红
        m.lifetime = ros::Duration(0.3);
        m.text = label;
        arr.markers.push_back(m);
        raw_det_marker_pub_.publish(arr);
    }

    void publishEmptyGps() {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "wgs84";
        msg.status.status  = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        target_est_gps_pub_.publish(msg);
    }

    bool clusterToGps(const TargetCluster& c,
                      double& latitude, double& longitude,
                      double& altitude) const {
        latitude = 0.0;
        longitude = 0.0;
        altitude = 0.0;
        if (!is_home_locked_) return false;

        // Cluster coordinates are NWU relative to the locked PX4 home.
        const double north_m = c.ned_x;
        const double east_m = -c.ned_y;
        const double up_m = c.ned_alt;
        const double home_lat_rad = home_gps_.latitude * M_PI / 180.0;
        const double cos_lat = std::cos(home_lat_rad);
        if (!std::isfinite(cos_lat) || std::fabs(cos_lat) < 1e-9) {
            return false;
        }

        constexpr double DEG_PER_M_LAT = 1.0 / 111320.0;
        const double deg_per_m_lon = 1.0 / (111320.0 * cos_lat);
        latitude = home_gps_.latitude + north_m * DEG_PER_M_LAT;
        longitude = home_gps_.longitude + east_m * deg_per_m_lon;
        altitude = home_gps_.altitude + up_m;
        return std::isfinite(latitude) && std::isfinite(longitude) &&
               std::isfinite(altitude);
    }

    void publishPrimaryGps(const TargetCluster& c) {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "wgs84";

        // === 2026-07-28: 必须用 PX4 home 作为参考,不是 UAV 当前 GPS ===
        //   c.ned_x/ned_y 已经是反投影到 map 系的绝对坐标(与 publishPrimaryPose 一致),
        //   原版减去 UAV 当前位置再叠加 current_gps_,会导致 UAV 飞行时地面站目标跟着飘。
        if (!clusterToGps(c, msg.latitude, msg.longitude, msg.altitude)) {
            publishEmptyGps();
            return;
        }

        msg.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        target_est_gps_pub_.publish(msg);
    }

    /**
     * Publish primary cluster 位置作为 geometry_msgs::PoseStamped (NWU, 米)
     * guidance_control_node 订阅 target_estimated_pose 用此驱动 UAV 飞向目标
     * 注:guidance 只用 position.x/y/z + orientation 做运算,frame_id 仅给 RViz 显示用,
     * 因此改为本机 viz_frame_(uav0/map / uav1/map / ...),多机 RViz 各自能解析。
     */
    void publishPrimaryPose(const TargetCluster& c) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = viz_frame_;  // 本机 local 帧,由 static TF 偏移到世界 map
        // === 2026-08-23: 动目标 lead pose (KF + v*lead_time) ===
        //   STATIC / UNKNOWN: 直接用 KF 输出 (c.ned_x/y), 等效旧版 EMA 行为
        //   DYNAMIC:          c.kf + 收拢速度反算的 lead_time, UAV 不从目标后面飞过
        // === 2026-08-25: 删 estimator lead_t1 (double-lead bug 治本) ===
        //   原 DYNAMIC 块算 lead_time (0.2-1.5s) 后调 c.kf_.getPositionWithLead
        //   把 KF 真值提前发布。guidance_control_node.cpp:2400 chase 自己也加
        //   lead_t2 (0.2-1.5s),两层叠加 lead ≈ 2.0-3.0s,17 m/s × 2s = 34m,
        //   稳态过冲 30m+ (uav0_strike_2026-08-25-14-34-08_0.bag 复盘)。
        //   修法:统一在 chase 层加 lead,estimator 只 publish KF 真值。
        //   chase 的 lead_t2 由 chase_lead_min/max (yaml:0.2/1.5s) 钳位,
        //   closing speed 用 chase_K_p × horiz 闭环算出,比 estimator 这套更紧。
        //   不冲突验证:3 个 subscriber (guidance:680 / mission_manager:1142 /
        //   comm:226) 不读 lead,改完下游透明。
        msg.pose.position.x = c.ned_x;
        msg.pose.position.y = c.ned_y;
        msg.pose.position.z = c.ned_alt;
        msg.pose.orientation.w = 1.0;  // 单位四元数(目标无朝向)
        target_est_pose_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_estimator_node");
    TargetClusterer node;
    ros::spin();
    return 0;
}
