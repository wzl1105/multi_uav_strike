/**
 * waypoint_executor_node.cpp
 * 航点执行器：将 GPS 航点列表转换为飞控可执行的 NED 速度指令
 *
 * 职责：
 * - 接收地面站发送的 GPS 航点（经纬高）
 * - GPS → NED 坐标转换（基准点：安阳）
 * - 计算速度指令，引导无人机到达航点
 * - 发布 NED 坐标系速度指令给飞控（或仿真器）
 * - 在 RViz 上显示航点（NWU 坐标系）
 *
 * 仿真/真机切换：
 * - use_sim=true: 发送到 /quad 命名空间（仿真）
 * - use_sim=false: 发送到 /mavros 命名空间（真机）
 *
 * 坐标系说明：
 * - 输入：GPS (lat, lon, alt)
 * - 内部：NED (North-East-Down)
 * - 输出：NED 速度指令给飞控
 * - 显示：NWU (用于 RViz)
 *
 * 订阅：
 * - /mission/waypoint_cmd           - GPS 航点命令（来自 mission_manager 或地面站）
 * - /quad/pose 或 /mavros/local_position/pose - 本机位置（NED）
 * - /<ns>/avoidance/cmd             - 避障指令（来自 uav_avoidance_node,带优先级+cmd类型）
 *
 * 发布：
 * - /quad/setpoint_velocity/cmd_vel_unstamped 或 /mavros/... - NED 速度指令
 * - /waypoint_executor/status       - 执行状态
 * - /waypoint_executor/waypoints_rviz - 航点显示（RViz）
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <sstream>
#include <algorithm>
#include <limits>

// === Phase 4: 类型化消息 ===
#include "multi_uav_strike/Skill.h"
#include "multi_uav_strike/WaypointStatus.h"
#include "multi_uav_strike/AvoidanceCmd.h"
#include "multi_uav_strike/LidarMeasurement.h"

// 安阳基准点（用于 GPS → NED 转换）
const double ANYANG_LAT = 36.096;      // 安阳纬度
const double ANYANG_LON = 114.392;     // 安阳经度
const double ANYANG_ALT = 100.0;       // 安阳海拔（米）

class WaypointExecutor {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber waypoint_cmd_sub_;       // GPS 航点命令
    ros::Subscriber self_pose_sub_;          // 本机位置（NED）
    ros::Subscriber control_sub_;            // 控制命令（来自 mission_manager）
    ros::Subscriber mission_mode_sub_;       // 工作模式（SEARCH_ONLY/SEARCH_TRACK/SEARCH_STRIKE/IDLE）
    ros::Subscriber lidar_measurement_sub_;  // 带命中类型的前向激光测量

    // ============== 发布 ==============
    ros::Publisher setpoint_velocity_pub_;    // NED 速度指令
    ros::Publisher flight_mode_pub_;         // 飞行模式
    ros::Publisher status_pub_;              // 执行状态
    ros::Publisher waypoints_rviz_pub_;     // 航点显示（RViz）
    ros::Publisher waypoint_path_pub_;      // 航点路径（RViz Path显示）
    std::string viz_frame_;                  // RViz 帧名 "<ns>/map"(多机分离由 static TF 偏移)
    // === Phase 4: 类型化发布 ===
    ros::Publisher waypoint_status_pub_;    // typed WaypointStatus(给 mission_manager 用)
    ros::Publisher current_target_pub_;     // 当前航段终点 PoseStamped(给 uav_avoidance_node 用)

    // === Phase 4: 类型化订阅 ===
    ros::Subscriber skill_sub_;             // /waypoint_executor/skill(mission_manager 下发)
    ros::Subscriber avoidance_cmd_sub_;     // /avoidance/cmd(uav_avoidance_node 下发)

    // ============== 定时器 ==============
    ros::Timer executor_timer_;

    // ============== 状态 ==============
    // 航点队列
    struct Waypoint {
        double lat;      // 纬度
        double lon;     // 经度
        double alt;     // 高度
        double ned_x;   // NED X (North)
        double ned_y;   // NED Y (East)
        double ned_z;   // NED Z (Down)
        double desired_yaw_ned;  // 期望航向角（NED，北偏东，rad，来自地面站）
        bool reached;
    };
    std::vector<Waypoint> waypoint_queue_;
    size_t current_waypoint_index_;
    bool is_waypoints_received_;
    bool is_executing_;

    // === Phase 4: 分段航点状态(必须放在 Waypoint 之后)===
    enum class SegmentPhase { IDLE, ARRIVE, SKILL_AREA, HOLD, COMPLETE };
    SegmentPhase segment_phase_;
    std::string  active_flow_id_;
    std::string  active_skill_id_;
    std::vector<Waypoint> arrive_queue_;    // arrive_path 转 Waypoint 队列
    std::vector<Waypoint> skill_area_queue_; // skill_area_path 转 Waypoint 队列
    size_t arrive_idx_;
    size_t skill_area_idx_;
    double seg_start_x_;                    // 当前段起点(对 idx=0 是进入段时锁存的飞机位置,后续不再刷新)
    double seg_start_y_;                    //   ——区分于 current_ned_x_/y_(每帧刷新,作为飞机当前位置)
    double cruise_speed_;                   // 进场巡航速度(覆盖 uav_speed_,ARRIVE 段用)
    double task_speed_;                     // 任务区速度(覆盖 uav_speed_,SKILL_AREA 段用)

    // 最新 AvoidanceCmd(默认值:全不约束)
    multi_uav_strike::AvoidanceCmd latest_avoidance_;
    bool has_latest_avoidance_;
    // 垂直避让 latch(保留 anti-oscillation):爬升时锁存高度,短暂保留(由 latch_release_delay_ 控制)
    //   防止 vbias 抖动导致的"被抬高 → 拉回 → 再抬高"震荡,但保留时间窗不会太长,
    //   过了这个时间窗无论距离是否退出 PROXIMITY 都释放,让下降早点开始。
    bool      vert_takeover_             = false;
    double    avoid_hold_z_              = 0.0;   // 锁存的目标高度(NED)
    ros::Time last_climb_active_time_;            // 上一次 vbias!=0 的时刻(每 tick 更新)
    double    latch_release_delay_       = 1.5;   // climb 结束后 latch 保留时长(s),过后释放

    enum class BuildingAvoidanceState { CRUISE, CLIMBING, HOLD_HIGH };
    BuildingAvoidanceState building_avoidance_state_;
    bool lidar_building_avoidance_enabled_;
    double lidar_climb_rate_;
    double lidar_clear_confirm_time_;
    double lidar_timeout_;
    double lidar_max_relative_altitude_;
    double lidar_slowdown_distance_;
    double lidar_climb_trigger_distance_;
    double lidar_stop_distance_;
    double lidar_altitude_offset_;
    double latest_lidar_range_;
    uint8_t latest_lidar_hit_type_;
    ros::Time latest_lidar_stamp_;
    ros::Time lidar_no_hit_since_;

// 本机状态（NED）
    double current_ned_x_;
    double current_ned_y_;
    double current_ned_z_;
    double current_yaw_ned_;  // 当前偏航角（NED，北偏东，rad）
    bool is_pose_received_;

    // 速度参数
    double uav_speed_;   // 飞行速度 (m/s)
    double uav_vertical_speed_; // Vertical speed
    double arrival_threshold_;  // 到达阈值 (m)
    double final_approach_threshold_;  // 终段直接逼近距离阈值 (m) — L1 在小 cross 时 v_lateral 太小
                                      // 进不了 arrival_threshold 会卡死,最后这段切直接朝航点飞

    // 航向控制(P-controller 输出角速度用)
    double yaw_max_rate_;       // 最大角速度 rad/s(由 ~yaw_max_rate 配置)
    double yaw_speed_gate_;     // 水平速度门控:m/s,低于此值时冻结 yaw 角速度输出
                                // (避免 CRITICAL stop+reverse 时 UAV 几乎不动但 yaw 仍追 bearing → 原地打转)

    // ============== 参数 ==============
    double executor_rate_;
    int flight_mode_velocity_;
    int flight_mode_position_;

    // ===== 加/减速度限幅(平滑起步、缓和刹车)=====
    double accel_limit_;   // m/s²,起步加速度上限 — 防止 v_mag 从 0 阶跃到 cur_speed
    double decel_limit_;   // m/s²,刹车减速度上限 — 物理刹停 v²≤2ad 用这个值
    double last_v_mag_;    // 上周期 v_mag (accel_limit 用,跨航点平滑过渡)

    // ===== L1 横向控制(压线)=====
    // L1 = max(L1_min_, cur_speed * L1_period_) — 决定飞机在段方向上的"超前看"距离(m)
    // L1 越大 → 横向纠偏越平滑,但弯道/扰动下超调越大("画龙"感越强)
    // L1 越小 → 横向响应越尖锐,贴线越紧,但 < 2m 易高频抖动
    double L1_min_;        // m,低速段最小 lookahead,默认 3.0(原硬编码 5.0 偏长→压线响应慢)
    double L1_period_;     // s,高速段 lookahead = cur_speed * L1_period_,默认 0.6(原硬编码 1.0)

    // ===== 航向 P-control(机头跟踪 L1 bearing)=====
    // yaw_rate = yaw_kp_ * yaw_error,饱和在 yaw_max_rate_
    // yaw_kp 越大 → 航向跟踪越紧,但过大会震荡
    double yaw_kp_;        // rad/s per rad,航向跟踪增益,默认 3.0(原硬编码 2.0 → 转弯期 yaw 跟不上速度方向)
    double yaw_deadband_;  // rad,航向误差死区,默认 0.05(~3°)避免小幅抖动

    // 基准点参数(由 PX4 /mavros/home_position/home 自动填充,不再硬编码)
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;
    bool ref_initialized_ = false;
    ros::Subscriber home_position_sub_;  // 一次订阅,拿 PX4 home
    ros::Subscriber mavros_state_sub_;   // /mavros/state(仅真机):接管感知观测点

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;
    std::string vel_topic_;

    // 航向控制选择：true=跟踪地面站发送的航点期望航向，false=计算航点指向
    bool use_desired_yaw_from_wp_;

    // 避障 reverse_allowed 生效时沿段方向后退的速度 (m/s)
    double reverse_speed_;

    // 无人机编号（用于区分颜色）
    int uav_id_;

    // 最后一个航点到位后是否已打印过日志（避免每周期刷屏）
    bool last_waypoint_reached_logged_;

    // 最后一个航点 P-controller 增益（用于位置保持，避免震荡）
    double hold_kp_;

    // 当前工作模式（SEARCH_ONLY/SEARCH_TRACK/SEARCH_STRIKE/IDLE）
    // 只有 SEARCH_ONLY 模式下航点执行器才发控制指令，其他模式交给 guidance_control
    std::string current_work_mode_;

    // === Attack(105) 专用 ===
    // 当前 skill_type(由 skillCallback 缓存)。executeSegmentFlight 用此判断:
    //   - SKILL_AREA 段 idx>=1 且 active_skill_type_==105 时:
    //     把 L1 bearing 覆盖为"指向 WP2",保证 UAV 机头朝向目标,方便 YOLO 锁定。
    uint8_t active_skill_type_ = 0;
    // Attack(105) WP2 地板高度(NED 米,负值=地上方米)
    //   GS 发的目标 MSL=0(地面)时,不直接撞过去,而是保持至少此高度悬停/接近。
    double   attack_min_altitude_ned_ = -20.0;

public:
    WaypointExecutor() : nh_private_("~"),
        current_waypoint_index_(0),
        is_waypoints_received_(false),
        is_executing_(false),
        is_pose_received_(false),
        uav_speed_(5.0),
        uav_vertical_speed_(3.0),
        arrival_threshold_(1.0),
        executor_rate_(50.0),
        flight_mode_velocity_(0),
        flight_mode_position_(2),
        ref_lat_(0.0),
        ref_lon_(0.0),
        ref_alt_(0.0),
        use_sim_(true),
        last_waypoint_reached_logged_(false),
        hold_kp_(0.8),
        yaw_max_rate_(1.0),
        accel_limit_(2.0),
        decel_limit_(2.0),
        last_v_mag_(0.0),
        L1_min_(3.0),
        L1_period_(0.6),
        yaw_kp_(3.0),
        yaw_deadband_(0.05),
        current_work_mode_("SEARCH_ONLY"),
        segment_phase_(SegmentPhase::IDLE),
        arrive_idx_(0),
        skill_area_idx_(0),
        seg_start_x_(0.0),
        seg_start_y_(0.0),
        cruise_speed_(5.0),
        task_speed_(5.0),
        has_latest_avoidance_(false),
        building_avoidance_state_(BuildingAvoidanceState::CRUISE),
        lidar_building_avoidance_enabled_(false),
        lidar_climb_rate_(1.0),
        lidar_clear_confirm_time_(3.0),
        lidar_timeout_(0.5),
        lidar_max_relative_altitude_(500.0),
        lidar_slowdown_distance_(300.0),
        lidar_climb_trigger_distance_(100.0),
        lidar_stop_distance_(20.0),
        lidar_altitude_offset_(0.0),
        latest_lidar_range_(std::numeric_limits<double>::infinity()),
        latest_lidar_hit_type_(multi_uav_strike::LidarMeasurement::HIT_NONE) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();

        ROS_INFO("[WaypointExecutor] Initialized:");
        ROS_INFO("[WaypointExecutor]   Mode: %s", use_sim_ ? "SIMULATION (/quad)" : "REAL (/mavros)");
        ROS_INFO("[WaypointExecutor]   Ref: lat=%.6f, lon=%.6f, alt=%.1f", ref_lat_, ref_lon_, ref_alt_);
        ROS_INFO("[WaypointExecutor]   Speed: %.1f m/s, arrival_thresh: %.1f m", uav_speed_, arrival_threshold_);
    }

    void initParams() {
        nh_private_.param<double>("executor_rate", executor_rate_, 50.0);
        nh_private_.param<double>("uav_speed", uav_speed_, 5.0);
        nh_private_.param<double>("uav_vertical_speed", uav_vertical_speed_, 2.0);
        nh_private_.param<double>("arrival_threshold", arrival_threshold_, 1.0);
        // 终段直接逼近阈值 — 当飞机距航点 < 此值时,从 L1 压航线切到直接朝航点飞
        // (默认 2.5m:大于 arrival_threshold,留出从 L1 切到直接逼近的过渡区)
        nh_private_.param<double>("final_approach_threshold", final_approach_threshold_, 2.5);
        nh_private_.param<int>("flight_mode_velocity", flight_mode_velocity_, 0);
        nh_private_.param<int>("flight_mode_position", flight_mode_position_, 2);
        // 避障 reverse_allowed 生效时沿段方向后退的速度 (m/s)
        nh_private_.param<double>("reverse_speed", reverse_speed_, 1.0);
        // 最后一个航点位置保持的 P-controller 增益（1m 误差 → kp m/s）
        nh_private_.param<double>("hold_kp", hold_kp_, 0.8);
        // L1 横向控制(压线)。
        //   L1_min: lookahead 距离下限(m),低速段使用。
        //     减小 → 横向响应更尖锐,但 < 2m 易高频抖动;
        //     增大 → 响应更平滑,但"画龙"超调更明显。
        //   L1_period: 高速段 lookahead = cur_speed * L1_period_(m)。
        //     减小 → 高速时贴线更紧;增大 → 高速时更平滑但超调大。
        nh_private_.param<double>("L1_min", L1_min_, 3.0);
        nh_private_.param<double>("L1_period", L1_period_, 0.6);
        // 航向角速度限幅(rad/s)——避免 P-controller 输出过大旋转指令
        nh_private_.param<double>("yaw_max_rate", yaw_max_rate_, 1.0);
        // 航向 P-control 增益 + 死区:
        //   yaw_kp: 转弯期 yaw 跟踪 L1 bearing 的紧度。
        //     增大 → 航向跟踪更紧(转弯期 yaw 更快对齐速度方向,减小侧滑);
        //     过大会震荡。
        //   yaw_deadband: 航向误差死区(rad),低于此值不旋转,避免小幅抖动。
        nh_private_.param<double>("yaw_kp", yaw_kp_, 3.0);
        nh_private_.param<double>("yaw_deadband", yaw_deadband_, 0.05);
        // 水平速度门控(m/s):低于此值冻结 yaw 角速度输出,避免 CRITICAL 避让
        //   时身体几乎不动但 yaw 仍死追 bearing → 原地打转。默认 0.5 m/s,
        //   - PROXIMITY 边界(40m): scale=1.0 → v≈5 m/s > 0.5 → 正常跟踪
        //   - PROXIMITY→COLLISION 边界(20m): scale=0.3 → v≈1.5 m/s > 0.5 → 仍跟踪
        //   - COLLISION: scale=0.2 → v≈1 m/s > 0.5 → 仍跟踪
        //   - CRITICAL: scale=0.0 → v≈0 < 0.5 → **冻结 yaw**
        nh_private_.param<double>("yaw_speed_gate", yaw_speed_gate_, 0.5);
        // Latch 保留时间(s):climb 结束后 latch 仍保留的时长,缓冲 vbias 抖动。
        //   超过此时间(典型:对方已离开较远但 scale 还在边缘)→ 释放,允许开始下降。
        //   - 0 → 等价于"完全不 latch"(快速下降,但 oscillation 风险最大)
        //   - 正大 → 等价于原行为(抗 oscillation,但延迟下降)
        //   - 默认 1.5s:抗短时抖动的同时仍能在对方离开后较快下降
        nh_private_.param<double>("latch_release_delay", latch_release_delay_, 1.5);
        // 加/减速度限幅(m/s²)——起步不阶跃、刹车不急刹
        nh_private_.param<double>("accel_limit", accel_limit_, 3.0);
        nh_private_.param<double>("decel_limit", decel_limit_, 2.0);
        nh_private_.param<bool>("lidar_building_avoidance_enabled",
                                lidar_building_avoidance_enabled_, false);
        nh_private_.param<double>("lidar_climb_rate", lidar_climb_rate_, 1.0);
        nh_private_.param<double>("lidar_clear_confirm_time", lidar_clear_confirm_time_, 3.0);
        nh_private_.param<double>("lidar_timeout", lidar_timeout_, 0.5);
        nh_private_.param<double>("lidar_max_relative_altitude",
                                  lidar_max_relative_altitude_, 500.0);
        nh_private_.param<double>("lidar_slowdown_distance", lidar_slowdown_distance_, 300.0);
        nh_private_.param<double>("lidar_climb_trigger_distance",
                                  lidar_climb_trigger_distance_, 100.0);
        nh_private_.param<double>("lidar_stop_distance", lidar_stop_distance_, 20.0);

        // 自动从命名空间获取 uav_id（如 ns="uav0" → id=0）
        std::string ns = ros::this_node::getNamespace();
        size_t underscore_pos = ns.find("uav");
        if (underscore_pos != std::string::npos) {
            std::string num_str = ns.substr(underscore_pos + 3);
            uav_id_ = atoi(num_str.c_str());
        } else {
            uav_id_ = 0;
        }

        // RViz 帧名:去掉命名空间前导 '/' 后拼 "/map"(如 /uav0 → uav0/map);无命名空间 → map
        {
            std::string vns = ns;
            if (!vns.empty() && vns[0] == '/') vns = vns.substr(1);
            viz_frame_ = vns.empty() ? "map" : (vns + "/map");
        }

        // 基准点参数: 不再硬编码,而由 homePositionCallback() 从 PX4 自动加载
        // 这里 param() 只在 fallback(没收到 PX4 home)时给个粗略初值
        nh_private_.param<double>("ref_lat", ref_lat_, ANYANG_LAT);
        nh_private_.param<double>("ref_lon", ref_lon_, ANYANG_LON);
        nh_private_.param<double>("ref_alt", ref_alt_, ANYANG_ALT);

        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, true);
        if (use_sim_) {
            pose_topic_ = "quad/pose";
            vel_topic_ = "quad/setpoint_velocity/cmd_vel_unstamped";
        } else {
            pose_topic_ = "mavros/local_position/pose";
            vel_topic_ = "mavros/setpoint_velocity/cmd_vel_unstamped";
        }

        // 航向控制选择：true=跟踪地面站发送的期望航向，false=计算航点指向
        nh_private_.param<bool>("use_desired_yaw_from_wp", use_desired_yaw_from_wp_, true);

        // === Attack(105) 专用 ===
        // WP2 地板高度(NED),默认 -10.0m = 相对本机 home 上方 10m,避免 GS 标的目标
        // MSL=0 时飞机直接撞地。skillCallback 在 Attack skill 收到时把 WP2 的
        // ned_z 钳到此值上限。
        nh_private_.param<double>("attack_min_altitude_ned", attack_min_altitude_ned_, -20.0);
    }

    void initSubscribers() {
        // GPS 航点命令
        waypoint_cmd_sub_ = nh_.subscribe(
            "mission/waypoint_cmd", 10,
            &WaypointExecutor::waypointCmdCallback, this);

        // 本机位置（NED，来自飞控或仿真器）
        self_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &WaypointExecutor::selfPoseCallback, this);

        // 邻居无人机位置由 uav_avoidance_node 通过 /avoidance/cmd 提供(已带优先级 + cmd 类型),
        //   本节点只消费 avoidance cmd,不再单独订阅 other_uav_poses。

        // 控制命令（来自 mission_manager）
        control_sub_ = nh_.subscribe(
            "waypoint_executor/control", 10,
            &WaypointExecutor::controlCallback, this);

        // 工作模式（来自 mission_manager 转发的地面站指令）
        // 只有 SEARCH_ONLY 模式下航点执行器才发控制指令
        mission_mode_sub_ = nh_.subscribe(
            "mission/mode", 10,
            &WaypointExecutor::missionModeCallback, this);

        if (lidar_building_avoidance_enabled_) {
            lidar_measurement_sub_ = nh_.subscribe(
                "lidar/measurement", 10, &WaypointExecutor::lidarMeasurementCallback, this);
        }

        // === Phase 4: Skill(分段航点) — mission_manager 下发 ===
        skill_sub_ = nh_.subscribe(
            "waypoint_executor/skill", 10,
            &WaypointExecutor::skillCallback, this);

        // === Phase 6: AvoidanceCmd — uav_avoidance_node 下发(横向严禁 + 高度 bias + 速度 scale)===
        avoidance_cmd_sub_ = nh_.subscribe(
            "avoidance/cmd", 10,
            &WaypointExecutor::avoidanceCmdCallback, this);

        // === PX4 home 自动加载(ref_lat/lon/alt 的唯一权威来源)===
        home_position_sub_ = nh_.subscribe(
            "mavros/home_position/home", 10,
            &WaypointExecutor::homePositionCallback, this);

        // === 2026-07-23: 飞手接管感知观测点(仅真机 use_sim_=false)===
        // 决策(用户口径):接管期间**不停 setpoint**,让 setpoint_velocity 继续流,切回 OFFBOARD 后
        //   立即被飞控接走(避免 500ms COM_OFFBOARD_LOSS_TIMEOUT 真空)。段状态
        //   (segment_phase_/arrive_idx_/seg_start_x_/y_)在 guidance 触发的 stop() 时已被天然保留,
        //   构成隐式快照。因此本回调只做 DEBUG 观测,不改任何执行状态——保留订阅点以备未来扩展。
        if (!use_sim_) {
            mavros_state_sub_ = nh_.subscribe(
                "mavros/state", 10,
                &WaypointExecutor::mavrosStateCallback, this);
        }
    }

    void initPublishers() {
        // NED 速度指令（仿真或真机）
        setpoint_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(
            vel_topic_, 10);

        // 飞行模式（仿真或真机）
        std::string flight_mode_topic = use_sim_ ? "quad/flight_mode" : "mavros/flight_mode";
        flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
            flight_mode_topic, 10);

        // 执行状态(legacy std_msgs::String,挪到独立 topic 不与 typed WaypointStatus 冲突)
        status_pub_ = nh_.advertise<std_msgs::String>(
            "waypoint_executor/legacy_status", 10);

        // 航点显示（RViz - NWU 坐标系）
        waypoints_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "waypoint_executor/waypoints_rviz", 10);

        // 航点路径（RViz Path显示）
        waypoint_path_pub_ = nh_.advertise<nav_msgs::Path>(
            "waypoint_executor/waypoint_path", 10);

        // === Phase 4: typed WaypointStatus(mission_manager 用作 Skill 状态机门控)===
        waypoint_status_pub_ = nh_.advertise<multi_uav_strike::WaypointStatus>(
            "waypoint_executor/status", 10);

        // === Phase 6: 当前航段终点(uav_avoidance_node 用于"我方当前线段终点"参考)===
        current_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "waypoint_executor/current_target", 10);

        // 默认 avoidance:全不约束
        latest_avoidance_.lateral_blocked = true;
        latest_avoidance_.lateral_max     = 0.0;
        latest_avoidance_.vertical_bias   = 0.0;
        latest_avoidance_.speed_scale     = 1.0;
        latest_avoidance_.reverse_allowed = false;
        has_latest_avoidance_ = false;
    }

    void initTimers() {
        executor_timer_ = nh_.createTimer(
            ros::Duration(1.0 / executor_rate_),
            &WaypointExecutor::executorTimerCallback, this);
    }

    // ============== GPS <-> NED 转换 ==============

    /**
     * GPS (WGS84) -> NED 坐标转换
     *
     * @param alt_is_msl  false(默认)=alt 已是相对 PX4 home 的米数(GS 给的"home 上方 N 米"),
     *                          适用于 Search/Gather/Return 等常规 skill。
     *                    true =alt 是 MSL 绝对高(海平面之上),用于 Attack(105):
     *                          跨机协同场景里探测机记录的是目标 MSL + 绝对 GPS,
     *                          GS 转发给接收机,接收机必须用本机 ref_alt_ 归算成本地 NED。
     *                          要求 ref_alt_ 已 locked(见 homePositionCallback / skillCallback 安全门)。
     */
    void gpsToNed(double lat, double lon, double alt,
                  double& ned_x, double& ned_y, double& ned_z,
                  bool alt_is_msl = false) {
        const double EARTH_R = 6378137.0;

        // 纬度差 -> N (北向)
        double d_lat = lat - ref_lat_;
        ned_x = d_lat * M_PI / 180.0 * EARTH_R;

        // 经度差 -> E (东向)
        double d_lon = lon - ref_lon_;
        ned_y = d_lon * M_PI / 180.0 * EARTH_R * cos(ref_lat_ * M_PI / 180.0);

        // 高度差 -> D (下向)
        //   alt_is_msl=true  : MSL 绝对高,减 ref_alt_ 归算成本机 NED
        //   alt_is_msl=false : GS 已发相对量,直接取负
        if (alt_is_msl) {
            ned_z = -(alt - ref_alt_);
        } else {
            ned_z = -alt;
        }
    }

    /**
     * PX4 home 回调 — 锁一次
     * PX4 的 LOCAL_POSITION_NED 参考系由 EKF2 在启动时锁定,
     * 后续 home 更新(MAV_CMD_DO_SET_HOME / disarm)不会重置 EKF2 origin,
     * 也不让 local_position 跳变。如果跟着 home 更新去重算 NED,
     * 反而让 setpoint 与 UAV 当前 local_position 不在同一 frame → 偏飞。
     *
     * 注意:ref_alt_ 是 Attack(105) MSL → 本地 NED 转换的唯一权威来源。
     * skillCallback 会在 ref_initialized_=false 时拒绝 Attack skill,
     * 保证只有 home 锁定后才接收需要 MSL→NED 的航点。
     */
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (ref_initialized_) return;  // 锁一次
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            return;  // PX4 home 未稳定前发 0/0,忽略
        }
        ROS_WARN("[WaypointExecutor] >>>> PX4 home locked: (%.7f, %.7f, %.2f)",
                 msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        ref_lat_ = msg->geo.latitude;
        ref_lon_ = msg->geo.longitude;
        ref_alt_ = msg->geo.altitude;
        ref_initialized_ = true;
    }

    // === 2026-07-23: 飞手接管观测(仅真机)===
    // 见 initSubscribers 处的决策说明:接管期间本节点**不改任何执行状态**,setpoint 继续流,
    // 切回 OFFBOARD 后被飞控立即接走。此回调仅打 DEBUG,保留订阅点以备未来扩展(如接管期
    // 主动降频 setpoint 或朝向丢失目标)。
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        ROS_DEBUG_THROTTLE(2.0, "[WaypointExecutor] mavros_state mode=%s armed=%d",
                           msg->mode.c_str(), msg->armed ? 1 : 0);
    }

    // ============== 回调函数 ==============

    void waypointCmdCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoint_queue_.clear();

        for (const auto& pose : msg->poses) {
            Waypoint wp;

            // 从 pose 中提取 GPS 信息
            // nav_msgs/Path 的 pose.position 存储 GPS: x=lat, y=lon, z=alt
            wp.lat = pose.pose.position.x;
            wp.lon = pose.pose.position.y;
            wp.alt = pose.pose.position.z;

            // GPS -> NED 转换
            gpsToNed(wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z);

            // 从 pose.orientation 提取期望航向角（四元数 -> yaw）
            // 如果是默认四元数 (w=1)，则期望航向为0
            double qx = pose.pose.orientation.x;
            double qy = pose.pose.orientation.y;
            double qz = pose.pose.orientation.z;
            double qw = pose.pose.orientation.w;

            // 检测是否是单位四元数（没有设置航向）
            if (fabs(qw - 1.0) < 0.01 && fabs(qx) < 0.01 && fabs(qy) < 0.01 && fabs(qz) < 0.01) {
                wp.desired_yaw_ned = 0.0;  // 默认朝向
            } else {
                // 从四元数提取 yaw
                wp.desired_yaw_ned = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            }

            wp.reached = false;
            waypoint_queue_.push_back(wp);

            ROS_INFO("[WaypointExecutor] GPS (%.6f, %.6f, %.1f) -> NED (%.2f, %.2f, %.2f) yaw=%.1fdeg",
                     wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z,
                     wp.desired_yaw_ned * 180.0 / M_PI);
        }

        current_waypoint_index_ = 0;
        is_waypoints_received_ = true;
        is_executing_ = true;
        last_waypoint_reached_logged_ = false;

        // 发布航点给 RViz 显示
        publishWaypointsForRviz();

        ROS_INFO("[WaypointExecutor] Received %lu GPS waypoints, starting execution",
                 waypoint_queue_.size());
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 注意：根据 use_sim 区分输入坐标系
        //   - use_sim_=true  (jMAVSim):  /quad/pose 直接是 NED
        //   - use_sim_=false (PX4 SITL): /mavros/local_position/pose 是 ENU (REP-103)
        //                                必须先 ENU -> NED，否则后续 NED 减法/算速度全错
        if (use_sim_) {
            current_ned_x_ = msg->pose.position.x;
            current_ned_y_ = msg->pose.position.y;
            current_ned_z_ = msg->pose.position.z;
        } else {
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            //   ENU: x=East,  y=North, z=Up
            //   NED: x=North, y=East, z=Down
            current_ned_x_ = msg->pose.position.y;   // ENU y (north) -> NED x
            current_ned_y_ = msg->pose.position.x;   // ENU x (east)  -> NED y
            current_ned_z_ = -msg->pose.position.z;  // ENU z (up)    -> NED z
        }

        // 从四元数提取偏航角（NED，北偏东）
        // NED坐标系下：yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        // 四元数 ENU/FLU -> NED/FRD:
        //   q_ned = q_T * q_enu * q_S
        //   q_T = (1/√2, 1/√2, 0, 0)  世界系 ENU→NED(绕(1,1,0)轴 180°)
        //   q_S = (1, 0, 0, 0)         机体系 FLU→FRD(绕 x 轴 180°)
        // 展开后:
        const double kSqrtHalf = 0.7071067811865475;
        const auto& qe = msg->pose.orientation;
        double qw = kSqrtHalf * (qe.w + qe.z);
        double qx = kSqrtHalf * (qe.x + qe.y);
        double qy = kSqrtHalf * (qe.x - qe.y);
        double qz = kSqrtHalf * (qe.w - qe.z);

        // NED坐标系下从四元数提取yaw（ZYX顺序）
        // yaw (heading) = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        // 但由于我们使用的是Z轴向上的表示法，实际应该是：
        current_yaw_ned_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        is_pose_received_ = true;
    }

    // ============== 定时器回调 ==============

    void executorTimerCallback(const ros::TimerEvent&) {
        if (!is_pose_received_) {
            return;
        }

        // === Phase 4: 分段航点执行(若 active skill)===
        if (segment_phase_ == SegmentPhase::ARRIVE ||
            segment_phase_ == SegmentPhase::SKILL_AREA) {
            // 衔接修复:attack/track handoff 时 mission_manager 发 "stop" → stop() 把
            //   is_executing_ 置 false(但保留 segment_phase_ 和航点队列以便 resume)。
            //   必须在这里拦截,否则 50Hz 定时器仍会调 executeSegmentFlight() 发非零速度,
            //   与 guidance_control 的 setpoint 冲突,导致 PX4 OFFBOARD 看到速度跳变。
            //   老路径(下方 SEARCH_ONLY 分支)本来就检查 is_executing_,但 Phase 4 漏了。
            if (!is_executing_) {
                //publishZeroVelocity();
                publishWaypointStatus();  // 仍上报(mission_manager 门控依赖)
                publishCurrentTarget();   // 仍上报(uav_avoidance 用)
                return;
            }
            executeSegmentFlight();
            // 持续发 WaypointStatus(mission_manager 状态机门控)
            publishWaypointStatus();
            // 持续发当前航段终点(uav_avoidance_node 用)
            publishCurrentTarget();
            return;
        }

        // 兼容老路径:只有 SEARCH_ONLY 模式下才发航点控制指令
        // 其他模式(SEARCH_TRACK/SEARCH_STRIKE/IDLE)由 guidance_control_node 接管
        if (current_work_mode_ != "SEARCH_ONLY") {
            return;
        }

        if (!is_executing_ || !is_waypoints_received_) {
            return;
        }

    }

    /**
     * 分段航点执行(Phase 4)
     * - 当前段(arrive_queue_ 或 skill_area_queue_)取当前 idx 航点
     * - 沿切向飞(压航线:不补横向偏差,只算段方向)
     * - 距离 ≤ arrival_threshold_ → idx++(段末则 advanceSegmentPhase)
     * - SKILL_AREA 段速度 = task_speed_ 而不是 uav_speed_
     * - 应用 AvoidanceCmd(lateral_blocked → 横向置零 / vertical_bias / speed_scale)
     */
    void executeSegmentFlight() {
        // 选当前队列
        std::vector<Waypoint>* cur_queue = nullptr;
        size_t* cur_idx = nullptr;
        double cur_speed = uav_speed_;
        if (segment_phase_ == SegmentPhase::ARRIVE) {
            cur_queue = &arrive_queue_;
            cur_idx   = &arrive_idx_;
            cur_speed = cruise_speed_ > 0.0 ? cruise_speed_ : uav_speed_;
        } else if (segment_phase_ == SegmentPhase::SKILL_AREA) {
            cur_queue = &skill_area_queue_;
            cur_idx   = &skill_area_idx_;
            cur_speed = task_speed_ > 0.0 ? task_speed_ : uav_speed_;
        }
        if (!cur_queue || cur_queue->empty()) {
            advanceSegmentPhase();
            return;
        }

        // idx 越界 → 完成
        if (*cur_idx >= cur_queue->size()) {
            advanceSegmentPhase();
            return;
        }

        // === 段方向(压航线:用前一个点(或锁存的段起点)→ 当前点)===
        // 首点:用锁存的 seg_start_(进入当前段时的飞机位置,不再刷新),保证段线固定,真正压线
        // 后续:用上一个航点 → 当前航点
        double seg_x0, seg_y0;
        if (*cur_idx == 0) {
            seg_x0 = seg_start_x_;
            seg_y0 = seg_start_y_;
        } else {
            const auto& prev = (*cur_queue)[*cur_idx - 1];
            seg_x0 = prev.ned_x;
            seg_y0 = prev.ned_y;
        }
        const Waypoint& target_wp = (*cur_queue)[*cur_idx];

        double dx_to_target = target_wp.ned_x - current_ned_x_;
        double dy_to_target = target_wp.ned_y - current_ned_y_;
        // === Attack(105) SKILL_AREA 飞行轨迹高度钳 ===
        //   不修改 WP2 坐标,只在飞行目标 z 上钳到 attack_min_altitude_ned_
        //   (默认 -20m = 20m AGL)。效果:
        //     - WP1→WP2 段:飞机沿 WP1→WP2 直线飞,目标 z 钳到 20m AGL
        //       → 用户口径:"沿 WP1→WP2 飞,到 AGL 20m 停"
        //     - 到达 WP2 水平位置(20m AGL)时,YOLO 命中 → strike 接管;
        //       YOLO 没命中 → 20m AGL hover 等
        //   NED 下向=正,ned_z 越大=越低;ned_z > attack_min_altitude_ned_ 表示低于 20m AGL,需要钳到上限
        double target_z_for_flight = target_wp.ned_z;
        if (active_skill_type_ == 105 &&
            segment_phase_ == SegmentPhase::SKILL_AREA &&
            target_z_for_flight > attack_min_altitude_ned_) {
            ROS_WARN_THROTTLE(5.0,
                "[WaypointExecutor] Attack(105) flight path altitude clamp: target_wp.ned_z=%.2f -> %.2f (20m AGL floor)",
                target_z_for_flight, attack_min_altitude_ned_);
            target_z_for_flight = attack_min_altitude_ned_;
        }
        const double effective_target_z = target_z_for_flight + lidar_altitude_offset_;
        double dz_to_target = effective_target_z - current_ned_z_;
        double dist_to_target = sqrt(dx_to_target*dx_to_target +
                                     dy_to_target*dy_to_target +
                                     dz_to_target*dz_to_target);

        // === 水平距离(终段切换判定用)===
        double dist_h = sqrt(dx_to_target*dx_to_target + dy_to_target*dy_to_target);

        double vx, vy;
        double bearing;

        // === 终段 P-control(水平面位置误差 → 速度,平滑收敛到 0)===
        // 参考 PX4 位置控制器外环: pos_error → vel_sp (P-control),内环 PID 再做 att_sp + thrust
        //   与现有 hold_kp_ 一致,行为可预测: v = hold_kp * dist_h
        //     dist_h = 2.5m → 2.0 m/s(平顺接管)
        //     dist_h = 1.0m → 0.8 m/s(慢速接近 arrival)
        //     dist_h = 0.0m → 0.0 m/s(自然悬停)
        // 与 L1 的对比:
        //   - L1 用段方向算横向补偿,小 cross 时 v_lateral ≈ 0 + 切向刹停 v_along ≈ 0 → 卡死
        //   - P-control 直接对位置误差反应,不依赖段方向,水平位置永远在收敛
        if (dist_h < final_approach_threshold_) {
            // P-control 速度(距离 → 速度,自然悬停)
            double v_p = hold_kp_ * dist_h;
            // 防御性限速(避免 hold_kp_ 配大时冲到 cur_speed 以上)
            if (v_p > cur_speed) v_p = cur_speed;

            if (dist_h > 0.01) {
                vx = (dx_to_target / dist_h) * v_p;
                vy = (dy_to_target / dist_h) * v_p;
            } else {
                // 距离极小(数值上几乎到点),直接悬停
                vx = 0.0;
                vy = 0.0;
            }
            // bearing 取直接朝航点(避免段方向偏差,支持在航点上 hover)
            bearing = atan2(dy_to_target, dx_to_target);
        } else {
            // === Lookahead-L1 控制(借鉴 PX4 mc_pos_control::PositionControl) ===
            // 参考 PX4 旋翼外环: 飞机朝 lookahead 点飞,速度幅值由 v² ≤ 2·a·d 限速
            //   L1 距离 = max(L1_min, v * L1_period),高速 lookahead 自然变长 → 平滑
            //   横向速度由几何(sin/cos)自然限制 = cur_speed,无需手动 cap
            //   当 UAV 过段端:target_along 夹到 seg_len → 直接朝目标点,brake 距离
            //   自动切到 dist_h → 平滑过渡到 P-control,无死区
            double seg_dx = target_wp.ned_x - seg_x0;
            double seg_dy = target_wp.ned_y - seg_y0;
            double seg_len = sqrt(seg_dx*seg_dx + seg_dy*seg_dy);

            if (seg_len < 0.1) {
                // 段退化为点(连续两航点重合 / 第一点已在到达半径内)
                vx = 0.0;
                vy = 0.0;
                bearing = atan2(dy_to_target, dx_to_target);
            } else {
                double ux = seg_dx / seg_len;
                double uy = seg_dy / seg_len;
                // 飞机沿段投影
                double rx = current_ned_x_ - seg_x0;
                double ry = current_ned_y_ - seg_y0;
                double along = rx * ux + ry * uy;

                // ===== Lookahead 距离(PX4 mc_pos_control 同款) =====
// 来自字段 L1_min_/L1_period_,可在 yaml 调:减小→贴线更紧;过大→"画龙"超调。
                const double L1_min = L1_min_;     // m,低速段最小 lookahead,避免抖动
                const double L1_period = L1_period_;  // s,高速段 lookahead 系数
                double L1 = std::max(L1_min, cur_speed * L1_period);
                // lookahead 投影点夹在 [0, seg_len](不过段端、不倒退)
                double target_along = along + L1;
                if (target_along < 0.0) target_along = 0.0;
                if (target_along > seg_len) target_along = seg_len;
                double la_x = seg_x0 + ux * target_along;
                double la_y = seg_y0 + uy * target_along;

                // ===== 物理刹停(v² ≤ 2·a·d) =====
                // 沿段剩余距离;过段端(< 0)改用 dist_h → 自动衔接 P-control
                double dist_along_to_target = seg_len - along;
                double brake_dist = (dist_along_to_target > 0.0)
                                        ? dist_along_to_target
                                        : dist_h;
                double v_mag = std::min(cur_speed,
                                        sqrt(2.0 * decel_limit_ * brake_dist));

                // ===== 加速度限幅:起步不阶跃,跨航点平滑过渡 =====
                // L1 算出的 v_mag 在静止起步时 = cur_speed(阶跃);
                // 用 last_v_mag_ + a*dt 上限钳制 → 起飞/换航点平滑
                double dt = 1.0 / executor_rate_;
                double v_max_accel = last_v_mag_ + accel_limit_ * dt;
                if (v_mag > v_max_accel) v_mag = v_max_accel;
                if (v_mag < 0.0) v_mag = 0.0;
                last_v_mag_ = v_mag;

                // ===== 朝 lookahead 飞 =====
                double dx_la = la_x - current_ned_x_;
                double dy_la = la_y - current_ned_y_;
                double dist_la = sqrt(dx_la*dx_la + dy_la*dy_la);
                if (dist_la > 0.01) {
                    vx = (dx_la / dist_la) * v_mag;
                    vy = (dy_la / dist_la) * v_mag;
                } else {
                    vx = 0.0;
                    vy = 0.0;
                }
                // bearing 跟 lookahead: 段内朝段前方;偏离段时航向也跟着纠
                bearing = atan2(dy_la, dx_la);
            }
        }

        // === Attack(105) SKILL_AREA 接近段(idx>=1):机头朝向 WP2(目标) ===
        //   WP1 → WP2 段,L1 沿段方向算 lookahead,机头可能不正对目标;
        //   对搜索/打击场景,YOLO 朝向必须朝目标,这里强制 bearing = 朝 WP2。
        //   仅在非终段(< final_approach_threshold_)时不覆盖,终段 P-control 已经用直指目标。
        if (active_skill_type_ == 105 &&
            segment_phase_ == SegmentPhase::SKILL_AREA &&
            skill_area_idx_ >= 1 &&
            dist_h >= final_approach_threshold_) {
            bearing = atan2(dy_to_target, dx_to_target);
        }
        // 打印当前位置、目标位置(原始 vs 钳后)、距离、段方位角(5s 一次,状态变化另有日志)
        ROS_INFO_THROTTLE(5.0, "[WaypointExecutor] SegmentPhase=%d, idx=%lu, cur=(%.2f,%.2f,%.2f), target=(%.2f,%.2f,%.2f), target_z_flight=%.2f, dist=%.2f, bearing=%.1fdeg",
                           static_cast<int>(segment_phase_), *cur_idx,
                           current_ned_x_, current_ned_y_, current_ned_z_,
                           target_wp.ned_x, target_wp.ned_y, target_wp.ned_z,
                           target_z_for_flight,
                           dist_to_target, bearing * 180.0 / M_PI);

        // === 垂直速度(参考 executeWaypointFlight 的限幅)===
        double vz;
        if (fabs(dz_to_target) > 1.0) {
            vz = (dz_to_target > 0 ? 1.0 : -1.0) * uav_vertical_speed_;
        } else {
            vz = hold_kp_ * dz_to_target;
        }
        // 限幅(防御性,虽然上面已经限到 ±uav_vertical_speed_)
        if (vz >  uav_vertical_speed_) vz =  uav_vertical_speed_;
        if (vz < -uav_vertical_speed_) vz = -uav_vertical_speed_;

        // === 应用 AvoidanceCmd(横向严禁需要段方向来分解速度,清掉横向补偿)===
        applyAvoidanceCmd(vx, vy, vz, bearing);
        // applyBuildingAvoidance 拿钳制后的 target_z_for_flight,避免 HOLD_HIGH 状态
        //   以为目标是 WP2 真实高(可能是地面)而把飞机绕开到错误高度。
        applyBuildingAvoidance(vx, vy, vz, target_z_for_flight);

        // === 距离判定 ===
        if (dist_to_target < arrival_threshold_) {
            (*cur_queue)[*cur_idx].reached = true;
            (*cur_idx)++;

            if (*cur_idx >= cur_queue->size()) {
                // 段末 → 切下一段
                advanceSegmentPhase();
                publishZeroVelocity();
                return;
            }

            // 段内下一航点:继续按新段方向飞
            publishZeroVelocity();
            publishWaypointStatus();
            return;
        }

        // === 发布速度+航向(沿段方向,不是目标方向)===
        publishVelocityCommandWithYaw(vx, vy, vz, bearing);
    }

    /**
     * 把 AvoidanceCmd 应用到速度指令
     * 横向严禁(lateral_blocked=true)→ 强制 vx,vy = 沿段方向分量(去掉横向偏差)
     * vertical_bias → 向上爬升速率(正=向上,负=向下),叠加到 vz
     * speed_scale → 乘到 vx,vy(速度调整)
     * reverse_allowed → critical 冲突时沿段方向反向后退,主动拉开水平距离
     *
     * @param seg_bearing_ned 段方位角(NED),用于横向严禁时把速度投影到段方向
     */
    void applyAvoidanceCmd(double& vx, double& vy, double& vz, double seg_bearing_ned) {
        if (!has_latest_avoidance_) return;

        double vbias   = static_cast<double>(latest_avoidance_.vertical_bias);
        double scale   = static_cast<double>(latest_avoidance_.speed_scale);
        bool   reverse = latest_avoidance_.reverse_allowed;

        // 是否处于"避让中"(默认 cmd 是 lateral_blocked=true/speed=1/bias=0/no-reverse,
        // 所以用 speed<1 || bias!=0 || reverse 判定真正在避让)
        bool avoiding = (scale < 0.999) || (std::fabs(vbias) > 1e-3) || reverse;

        // ===== 垂直:避让接管 + 时间衰减锁存 =====
        // vbias = "向上爬升速率"(正=上,负=下);executor 内 vz 是 NED(down 正),上=负。
        //   - active 爬升(vbias!=0):接管 vz 爬升/俯冲,刷新 last_climb_active_time_
        //   - 爬升刚结束(仍 avoidance 中):保留 latch 至 latch_release_delay_ 秒,
        //     抗短时 oscillation;超过窗口即释放,让下降早点开始
        //   - 完全不用避让:清 latch
        if (std::fabs(vbias) > 1e-3) {
            vz = -vbias;
            vert_takeover_ = true;
            avoid_hold_z_  = current_ned_z_;
            last_climb_active_time_ = ros::Time::now();
        } else if (vert_takeover_ && avoiding) {
            double since_climb = (ros::Time::now() - last_climb_active_time_).toSec();
            if (since_climb < latch_release_delay_) {
                // 仍在保留窗口内 — 锁存峰值(P-control),缓冲 vbias 抖动
                vz = hold_kp_ * (avoid_hold_z_ - current_ned_z_);
                if (vz >  uav_vertical_speed_) vz =  uav_vertical_speed_;
                if (vz < -uav_vertical_speed_) vz = -uav_vertical_speed_;
            } else {
                // 保留窗口已过(典型场景:对方已离开 PROXIMITY 区但 scale 仍在边缘)→ 释放
                vert_takeover_ = false;
            }
        } else {
            vert_takeover_ = false;
        }

        // ===== 水平:速度缩放 =====
        vx *= scale;
        vy *= scale;

        double cos_b = cos(seg_bearing_ned);
        double sin_b = sin(seg_bearing_ned);

        // 横向严禁 — 把速度投影到段方向上,只保留切向,清除横向偏差补偿
        if (latest_avoidance_.lateral_blocked) {
            double v_along = vx * cos_b + vy * sin_b;
            vx = cos_b * v_along;
            vy = sin_b * v_along;
            ROS_DEBUG_THROTTLE(5.0, "[WaypointExecutor] lateral_blocked=true (proj to segment, lateral cleared)");
        }

        // reverse_allowed(critical 冲突):沿段方向反向后退,主动拉开水平距离
        if (reverse) {
            vx += -cos_b * reverse_speed_;
            vy += -sin_b * reverse_speed_;
            ROS_WARN_THROTTLE(1.0, "[WaypointExecutor] reverse_allowed=true -> back along segment %.1f m/s", reverse_speed_);
        }
    }

    /**
     * 发布当前航段终点(给 uav_avoidance_node 用于"我方当前线段终点"参考)
     */
    void publishCurrentTarget() {
        geometry_msgs::PoseStamped cur;
        cur.header.stamp    = ros::Time::now();
        cur.header.frame_id = "map";
        std::vector<Waypoint>* q = nullptr;
        size_t* idx = nullptr;
        if (segment_phase_ == SegmentPhase::ARRIVE) {
            q = &arrive_queue_; idx = &arrive_idx_;
        } else if (segment_phase_ == SegmentPhase::SKILL_AREA) {
            q = &skill_area_queue_; idx = &skill_area_idx_;
        }
        if (q && !q->empty() && *idx < q->size()) {
            const auto& wp = (*q)[*idx];
            cur.pose.position.x = wp.ned_x;
            cur.pose.position.y = wp.ned_y;
            cur.pose.position.z = wp.ned_z;
            cur.pose.orientation.w = 1.0;
        } else {
            cur.pose.orientation.w = 1.0;
        }
        current_target_pub_.publish(cur);
    }

    // ============== 模式回调 ==============

    void missionModeCallback(const std_msgs::String::ConstPtr& msg) {
        std::string new_mode = msg->data;
        if (new_mode == current_work_mode_) {
            return;
        }
        ROS_WARN("[WaypointExecutor] Work mode changed: %s -> %s",
                 current_work_mode_.c_str(), new_mode.c_str());
        current_work_mode_ = new_mode;

        // 切出 SEARCH_ONLY 模式时：暂停执行并发零速，但**保留航点队列**，
        // 这样之后切回 SEARCH_ONLY 时能从 current_waypoint_index_ 继续执行。
        // 新航点可在任意时刻通过 /mission/waypoint_cmd 重新写入并 reset 队列。
        if (new_mode != "SEARCH_ONLY") {
            if (is_executing_) {
                ROS_WARN("[WaypointExecutor] Left SEARCH_ONLY, pausing waypoint control (queue preserved)");
            }
            is_executing_ = false;
            publishZeroVelocity();
        } else {
            // 切回 SEARCH_ONLY：根据航点队列状态决定行为
            if (waypoint_queue_.empty()) {
                // 队列为空：没航点可飞，保持悬停并报错，等待新的 /mission/waypoint_cmd
                ROS_ERROR("[WaypointExecutor] Mode is SEARCH_ONLY but waypoint queue is empty! "
                          "Hovering. Send a new waypoint list via /mission/waypoint_cmd to resume.");
                is_executing_ = false;
                is_waypoints_received_ = false;
            } else {
                // 队列非空：可能是之前残留的旧航点（resume），也可能是 TRACK 期间收到的新航点
                // （waypointCmdCallback 已 reset 队列到 index 0）。两种情况都直接开始执行。
                ROS_WARN("[WaypointExecutor] Back to SEARCH_ONLY, resuming %lu waypoints from index %zu",
                         waypoint_queue_.size(), current_waypoint_index_);
                is_executing_ = true;
                is_waypoints_received_ = true;
                last_waypoint_reached_logged_ = false;
            }
        }
    }

    /**
     * 发布速度指令（包含偏航角速率）
     * @param vx, vy, vz NED 速度
     * @param desired_yaw_ned 期望偏航角（NED，北偏东，rad）
     *
     * NED 坐标系下：
     * - atan2(dy, dx) 给出从正北方向顺时针旋转的角度
     * - 正值 = 右转（东向）
     * - angular.z 作为角速率控制偏航
     *
     * PX4 SITL: 需要将 NED 转换为 ENU
     */
    void publishVelocityCommandWithYaw(double vx, double vy, double vz, double desired_yaw_ned) {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = vx;
        vel_cmd.linear.y = vy;
        vel_cmd.linear.z = vz;

        // === 航向控制:P-controller ===
        // mavros setpoint_velocity.angular.z 是机体角速度(ENU:左转/CCW 为正,rad/s),
        // 不是绝对航向角。需要根据 current_yaw_ned_ 和 desired_yaw_ned_ 算误差 → 角速率。
        double yaw_error_ned = desired_yaw_ned - current_yaw_ned_;
        // 角度环绕到 [-π, π](避免 ±180° 边界时选错方向)
        while (yaw_error_ned >  M_PI) yaw_error_ned -= 2.0 * M_PI;
        while (yaw_error_ned < -M_PI) yaw_error_ned += 2.0 * M_PI;
        // P 控制 + 死区 + 速度门控 + 限幅
        // 来自字段 yaw_kp_/yaw_deadband_,可在 yaml 调:kp↑ → 航向跟踪更紧
        const double yaw_kp       = yaw_kp_;       // 比例增益(典型 1.0~3.0)
        const double yaw_deadband = yaw_deadband_; // ~3° 死区,避免小幅抖动
        double yaw_rate_ned = 0.0;
        // 水平速度门控:水平速度过低(典型场景:CRITICAL 避让中 vx≈vy≈0)时冻结 yaw,
        //   否则身体几乎不动但 yaw 仍死追 L1 bearing → 看起来"原地自由旋转"。
        const double v_horiz = sqrt(vx*vx + vy*vy);
        if (v_horiz > yaw_speed_gate_ && fabs(yaw_error_ned) > yaw_deadband) {
            yaw_rate_ned = yaw_kp * yaw_error_ned;
            if (yaw_rate_ned >  yaw_max_rate_) yaw_rate_ned =  yaw_max_rate_;
            if (yaw_rate_ned < -yaw_max_rate_) yaw_rate_ned = -yaw_max_rate_;
        }
        // NED → ENU 角速度(NED 是 CW+,ENU 是 CCW+,所以取反)
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = -yaw_rate_ned;

        // PX4 SITL: NED -> ENU 线性速度转换
        if (!use_sim_) {
            double ned_vx = vel_cmd.linear.x;
            double ned_vy = vel_cmd.linear.y;
            double ned_vz = vel_cmd.linear.z;
            // NED -> ENU 速度: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
            vel_cmd.linear.x = ned_vy;
            vel_cmd.linear.y = ned_vx;
            vel_cmd.linear.z = -ned_vz;
            // angular.z 已经在上面算成 ENU 角速度,这里不需要再转换
        }
        // 调试信息：5s 一次即可，详细字段非常多，没必要每帧刷
        ROS_INFO_THROTTLE(5.0, "[WaypointExecutor] Publishing velocity command: vx=%.2f, vy=%.2f, vz=%.2f, desired_yaw_ned=%.2fdeg, angular rate=%.2fdeg/s",
                 vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
                 desired_yaw_ned * 180.0 / M_PI,
                 vel_cmd.angular.z * 180.0 / M_PI);

        setpoint_velocity_pub_.publish(vel_cmd);

        std_msgs::Int16 mode_msg;
        mode_msg.data = flight_mode_velocity_;
        flight_mode_pub_.publish(mode_msg);

        // 实时打印调试信息（NWU 坐标系）
        double uav_nwu_x = current_ned_x_;
        double uav_nwu_y = -current_ned_y_;
        double uav_nwu_z = -current_ned_z_;

        double target_nwu_x = 0.0, target_nwu_y = 0.0, target_nwu_z = 0.0;
        double dist_to_target = 0.0;
        if (current_waypoint_index_ < waypoint_queue_.size()) {
            const auto& wp = waypoint_queue_[current_waypoint_index_];
            target_nwu_x = wp.ned_x;
            target_nwu_y = -wp.ned_y;
            target_nwu_z = -wp.ned_z;
            double dx = wp.ned_x - current_ned_x_;
            double dy = wp.ned_y - current_ned_y_;
            double dz = wp.ned_z - current_ned_z_;
            dist_to_target = sqrt(dx*dx + dy*dy + dz*dz);
        }

        double vel_nwu_x = vx;
        double vel_nwu_y = -vy;
        double vel_nwu_z = -vz;

        // ROS_INFO_THROTTLE(0.5,
        //     "[Flight] NWU (%.2f, %.2f, %.2f) | Target WP[%zu]: (%.2f, %.2f, %.2f) dist=%.2fm | Vel(%.2f, %.2f, %.2f) | Yaw: cur=%.0f des=%.0f",
        //     uav_nwu_x, uav_nwu_y, uav_nwu_z,
        //     current_waypoint_index_,
        //     target_nwu_x, target_nwu_y, target_nwu_z,
        //     dist_to_target,
        //     vel_nwu_x, vel_nwu_y, vel_nwu_z,
        //     current_yaw_ned_ * 180.0 / M_PI,
        //     desired_yaw_ned * 180.0 / M_PI);
    }

    void publishZeroVelocity() {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0.0;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = 0.0;  // 停止时偏航角速率也为0

        setpoint_velocity_pub_.publish(vel_cmd);

        ROS_DEBUG_THROTTLE(1.0, "[WaypointExecutor] Zero velocity published");
    }

    void publishStatus(const std::string& status) {
        std_msgs::String status_msg;
        status_msg.data = status;
        status_pub_.publish(status_msg);
    }

    void publishWaypointsForRviz() {
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = viz_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "waypoint_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.3;
        // LINE_STRIP 不用 orientation,但 RViz 仍会校验四元数,显式置 identity
        line_strip.pose.orientation.w = 1.0;
        // 颜色根据 uav_id 设置
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.3 + 0.2 * uav_id_;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = viz_frame_;
        path_msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < waypoint_queue_.size(); ++i) {
            const auto& wp = waypoint_queue_[i];

            // NED -> NWU for RViz
            geometry_msgs::Point p;
            p.x = wp.ned_x;
            p.y = -wp.ned_y;
            p.z = -wp.ned_z;
            line_strip.points.push_back(p);

            // Path for RViz
            geometry_msgs::PoseStamped path_pose;
            path_pose.header.frame_id = viz_frame_;
            path_pose.header.stamp = ros::Time::now();
            path_pose.pose.position.x = wp.ned_x;
            path_pose.pose.position.y = -wp.ned_y;
            path_pose.pose.position.z = -wp.ned_z;
            path_pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(path_pose);

            visualization_msgs::Marker marker;
            marker.header.frame_id = viz_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = i + 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = wp.ned_x;
            marker.pose.position.y = -wp.ned_y;
            marker.pose.position.z = -wp.ned_z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 2.0;
            marker.scale.y = 2.0;
            marker.scale.z = 2.0;
            // 颜色根据 uav_id 设置：uav0=绿色系，uav1=蓝色系，uav2=黄色系
            if (uav_id_ == 0) {
                marker.color.r = 0.2;
                marker.color.g = 0.8;
                marker.color.b = 0.2;
            } else if (uav_id_ == 1) {
                marker.color.r = 0.2;
                marker.color.g = 0.4;
                marker.color.b = 0.8;
            } else if (uav_id_ == 2) {
                marker.color.r = 1.0;
                marker.color.g = 0.6;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        line_strip.lifetime = ros::Duration(0);
        // LINE_STRIP 至少要 2 个点;takeoff 等 skill 队列空/单点时跳过,避免 RViz 警告
        if (line_strip.points.size() >= 2) {
            marker_array.markers.push_back(line_strip);
        }

        waypoints_rviz_pub_.publish(marker_array);
        waypoint_path_pub_.publish(path_msg);
    }

    // ============== 公共接口 ==============

    void controlCallback(const std_msgs::String::ConstPtr& msg) {
        std::string cmd = msg->data;
        ROS_WARN("[WaypointExecutor] >>>>> Received control command: %s", cmd.c_str());

        if (cmd == "stop") {
            stop();
        } else if (cmd == "pause") {
            pause();
        } else if (cmd == "resume") {
            resume();
        } else if (cmd.substr(0, 7) == "spiral:") {
            // 螺旋接近命令，暂不支持
            ROS_WARN("[WaypointExecutor] Spiral command not implemented yet: %s", cmd.c_str());
        }
    }

    void stop() {
        // 与 missionModeCallback 切出 SEARCH_ONLY 一致：暂停执行但**保留航点队列**，
        // 这样 mission_manager 在检测到目标发"stop"时不会破坏后续 resume 的能力。
        // 如果真的需要清空航点，发个新的 waypoint_cmd 即可（会 reset 队列）。
        is_executing_ = false;
        publishZeroVelocity();
        ROS_INFO("[WaypointExecutor] Stopped");
    }

    void pause() {
        is_executing_ = false;
        publishZeroVelocity();
        ROS_INFO("[WaypointExecutor] Paused");
    }

    void resume() {
        if (is_waypoints_received_) {
            is_executing_ = true;
            ROS_INFO("[WaypointExecutor] Resumed");
        }
    }

    // ============== Phase 4: Skill + AvoidanceCmd 回调 ==============

    /**
     * Skill 回调(mission_manager 下发)
     * 收到 Skill 后:
     *   1. 保存 active_flow_id_/active_skill_id_/task_speed_
     *   2. arrive_path 转 Waypoint 队列 → arrive_queue_
     *   3. skill_area_path 转 Waypoint 队列 → skill_area_queue_
     *   4. 重置 arrive_idx_=0 / skill_area_idx_=0
     *   5. segment_phase_ = ARRIVE(若 arrive_path 非空)或 SKILL_AREA(若 arrive_path 为空)
     *   6. 触发 RViz 显示
     */
    void skillCallback(const multi_uav_strike::Skill::ConstPtr& msg) {
        ROS_WARN("[WaypointExecutor] >>>> Skill received: id=%s type=%u arrive=%lu skill=%lu",
                 msg->skill_id.c_str(),
                 static_cast<unsigned>(msg->skill_type),
                 msg->arrive_path.poses.size(),
                 msg->skill_area_path.poses.size());

        // === 高度语义分流 ===
        // Attack(105) 跨机协同,GS 发的是探测机记录的 MSL+绝对 GPS,接收机要按本机
        // ref_alt_ 归算成本地 NED;其余 skill 由 GS 发相对 home 的米数。
        const bool alt_is_msl = (msg->skill_type == 105);
        if (alt_is_msl && !ref_initialized_) {
            ROS_ERROR("[WaypointExecutor] Attack(105) Skill received but PX4 home "
                      "(ref_alt_) not yet locked — refusing to convert MSL altitude. "
                      "Dropping skill_id=%s", msg->skill_id.c_str());
            return;
        }

        active_flow_id_  = "";  // TaskFlow 里的 flow_id 需要额外下发,这里先用 skill_id
        active_skill_id_ = msg->skill_id;
        active_skill_type_ = msg->skill_type;
        cruise_speed_    = msg->cruise_speed > 0.0 ? msg->cruise_speed : uav_speed_;
        task_speed_      = msg->task_speed > 0.0 ? msg->task_speed : uav_speed_;

        // arrive_path
        arrive_queue_.clear();
        for (const auto& pose : msg->arrive_path.poses) {
            arrive_queue_.push_back(poseToWaypoint(pose, alt_is_msl));
        }
        arrive_idx_ = 0;

        // skill_area_path
        skill_area_queue_.clear();
        for (const auto& pose : msg->skill_area_path.poses) {
            skill_area_queue_.push_back(poseToWaypoint(pose, alt_is_msl));
        }
        // === Attack(105) 高度钳制改为钳飞行轨迹 ===
        //   旧逻辑在这里改 skill_area_queue_.back().ned_z(改 WP2 坐标);
        //   新逻辑:不修改 WP2 坐标,在 executeSegmentFlight 中把 target_z 钳到
        //   attack_min_altitude_ned_(默认 -20m = 20m AGL)。
        //   效果:WP1→WP2 整段直线目标 z 都被钳到 20m AGL,飞机沿原航线飞到 20m AGL 时停,
        //   然后等 YOLO(用户口径:"沿 WP1→WP2 飞,到 AGL 20m 停")。
        //   WP2 真实高保持不变,后续 strike 由 guidance 接管。
        skill_area_idx_ = 0;

        // 段状态
        if (!arrive_queue_.empty()) {
            segment_phase_ = SegmentPhase::ARRIVE;
            // 锁存段起点(进入 ARRIVE 时的飞机位置),后续 idx=0 用此值算段线
            seg_start_x_ = current_ned_x_;
            seg_start_y_ = current_ned_y_;
        } else if (!skill_area_queue_.empty()) {
            segment_phase_ = SegmentPhase::SKILL_AREA;
            // 直接 SKILL_AREA(无 ARRIVE):同样锁存当前位置
            seg_start_x_ = current_ned_x_;
            seg_start_y_ = current_ned_y_;
        } else {
            segment_phase_ = SegmentPhase::IDLE;
            ROS_WARN("[WaypointExecutor] >>>> Skill has NO paths, segment_phase=IDLE");
        }

        is_waypoints_received_ = true;
        is_executing_          = true;
        last_waypoint_reached_logged_ = false;
        // 打印所有航点信息
        ROS_WARN("[WaypointExecutor] >>>> Skill waypoints: ARRIVE=%lu, SKILL_AREA=%lu, cruise_speed=%.2f, task_speed=%.2f",
                 arrive_queue_.size(), skill_area_queue_.size(), cruise_speed_, task_speed_);
        for (size_t i = 0; i < arrive_queue_.size(); ++i) {
            const auto& wp = arrive_queue_[i];
            ROS_WARN("[WaypointExecutor] ARRIVE[%zu]: lat=%.6f lon=%.6f alt=%.2f ned=(%.2f, %.2f, %.2f) yaw=%.1fdeg",
                     i, wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z, wp.desired_yaw_ned * 180.0 / M_PI);
        }
        for (size_t i = 0; i < skill_area_queue_.size(); ++i) {
            const auto& wp = skill_area_queue_[i];
            ROS_WARN("[WaypointExecutor] SKILL_AREA[%zu]: lat=%.6f lon=%.6f alt=%.2f ned=(%.2f, %.2f, %.2f) yaw=%.1fdeg",
                     i, wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z, wp.desired_yaw_ned * 180.0 / M_PI);
        }
        publishSegmentForRviz();
        publishWaypointStatus();
    }

    /** PoseStamped(GPS in position) → Waypoint
     *  @param alt_is_msl 透传给 gpsToNed;true 时 pose.position.z 解释为 MSL 绝对高(Attack 105 用) */
    Waypoint poseToWaypoint(const geometry_msgs::PoseStamped& pose,
                            bool alt_is_msl = false) {
        Waypoint wp;
        wp.lat = pose.pose.position.x;
        wp.lon = pose.pose.position.y;
        wp.alt = pose.pose.position.z;
        gpsToNed(wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z, alt_is_msl);

        // orientation (四元数 → yaw)
        double qx = pose.pose.orientation.x;
        double qy = pose.pose.orientation.y;
        double qz = pose.pose.orientation.z;
        double qw = pose.pose.orientation.w;
        if (fabs(qw - 1.0) < 0.01 && fabs(qx) < 0.01 && fabs(qy) < 0.01 && fabs(qz) < 0.01) {
            wp.desired_yaw_ned = 0.0;
        } else {
            wp.desired_yaw_ned = atan2(2.0 * (qw * qz + qx * qy),
                                       1.0 - 2.0 * (qy * qy + qz * qz));
        }
        wp.reached = false;
        return wp;
    }

    /**
     * AvoidanceCmd 回调(uav_avoidance_node 下发)
     * 仅缓存,executorTimerCallback() 读 latest_avoidance_ 应用
     */
    void avoidanceCmdCallback(const multi_uav_strike::AvoidanceCmd::ConstPtr& msg) {
        latest_avoidance_ = *msg;
        has_latest_avoidance_ = true;
    }

    void lidarMeasurementCallback(const multi_uav_strike::LidarMeasurement::ConstPtr& msg) {
        latest_lidar_stamp_ = ros::Time::now();
        latest_lidar_range_ = msg->range;
        latest_lidar_hit_type_ = msg->hit_type;
        const bool building_hit =
            msg->hit_type == multi_uav_strike::LidarMeasurement::HIT_BUILDING &&
            std::isfinite(msg->range) &&
            msg->range >= msg->min_range && msg->range <= msg->max_range;
        if (building_avoidance_state_ == BuildingAvoidanceState::CRUISE && building_hit &&
            msg->range <= lidar_climb_trigger_distance_) {
            building_avoidance_state_ = BuildingAvoidanceState::CLIMBING;
            lidar_no_hit_since_ = ros::Time();
            ROS_WARN("[WaypointExecutor] Building detected at %.1fm: start forward climb",
                     msg->range);
        } else if (building_avoidance_state_ == BuildingAvoidanceState::CLIMBING) {
            if (building_hit) {
                lidar_no_hit_since_ = ros::Time();
            } else if (lidar_no_hit_since_.isZero()) {
                lidar_no_hit_since_ = latest_lidar_stamp_;
            }
        }
    }

    void latchHighAltitude(double nominal_target_z, const char* reason) {
        lidar_altitude_offset_ = current_ned_z_ - nominal_target_z;
        building_avoidance_state_ = BuildingAvoidanceState::HOLD_HIGH;
        ROS_WARN("[WaypointExecutor] Building cleared (%s): hold relative altitude %.1fm, offset %.1fm",
                 reason, -current_ned_z_, lidar_altitude_offset_);
    }

    void applyBuildingAvoidance(double& vx, double& vy, double& vz,
                                double nominal_target_z) {
        if (!lidar_building_avoidance_enabled_) return;

        const ros::Time now = ros::Time::now();
        const bool lidar_fresh = !latest_lidar_stamp_.isZero() &&
                                 (now - latest_lidar_stamp_).toSec() <= lidar_timeout_;
        if (lidar_fresh &&
            latest_lidar_hit_type_ == multi_uav_strike::LidarMeasurement::HIT_BUILDING &&
            std::isfinite(latest_lidar_range_) &&
            latest_lidar_range_ < lidar_slowdown_distance_) {
            // 300m 外保持全速；300m→20m 之间线性减速；20m 内停止水平前进。
            const double denom = std::max(1.0, lidar_slowdown_distance_ - lidar_stop_distance_);
            const double scale = std::max(0.0, std::min(1.0,
                (latest_lidar_range_ - lidar_stop_distance_) / denom));
            vx *= scale;
            vy *= scale;
            ROS_WARN_THROTTLE(1.0,
                "[WaypointExecutor] BUILDING SLOWDOWN: range=%.1fm horizontal_scale=%.2f",
                latest_lidar_range_, scale);
        }

        if (building_avoidance_state_ == BuildingAvoidanceState::CRUISE) return;

        if (building_avoidance_state_ == BuildingAvoidanceState::CLIMBING) {
            if (!lidar_fresh) {
                latchHighAltitude(nominal_target_z, "lidar timeout");
            } else if (-current_ned_z_ >= lidar_max_relative_altitude_) {
                latchHighAltitude(nominal_target_z, "altitude limit");
                ROS_ERROR_THROTTLE(2.0,
                    "[WaypointExecutor] Lidar climb stopped at %.0fm relative limit",
                    lidar_max_relative_altitude_);
            } else if (!lidar_no_hit_since_.isZero() &&
                       (now - lidar_no_hit_since_).toSec() >= lidar_clear_confirm_time_) {
                latchHighAltitude(nominal_target_z, "no return after roof");
            } else {
                // NED z/velocity 向下为正，因此持续爬升使用负速度。
                vz = -std::fabs(lidar_climb_rate_);
                ROS_WARN_THROTTLE(1.0,
                    "[WaypointExecutor] BUILDING CLIMB: range=%.1fm alt=%.1fm vz=%.1fm/s",
                    latest_lidar_range_, -current_ned_z_, vz);
                return;
            }
        }

        if (building_avoidance_state_ == BuildingAvoidanceState::HOLD_HIGH) {
            const double hold_target_z = nominal_target_z + lidar_altitude_offset_;
            vz = hold_kp_ * (hold_target_z - current_ned_z_);
            vz = std::max(-uav_vertical_speed_, std::min(uav_vertical_speed_, vz));
        }
    }

    /**
     * typed WaypointStatus 发布 — mission_manager 用作 Skill 状态机门控
     */
    void publishWaypointStatus() {
        multi_uav_strike::WaypointStatus ws;
        ws.flow_id  = active_flow_id_;
        ws.skill_id = active_skill_id_;

        switch (segment_phase_) {
            case SegmentPhase::IDLE:        ws.phase = multi_uav_strike::WaypointStatus::PHASE_IDLE;        break;
            case SegmentPhase::ARRIVE:      ws.phase = multi_uav_strike::WaypointStatus::PHASE_ARRIVE;      break;
            case SegmentPhase::SKILL_AREA:  ws.phase = multi_uav_strike::WaypointStatus::PHASE_SKILL_AREA;  break;
            case SegmentPhase::HOLD:        ws.phase = multi_uav_strike::WaypointStatus::PHASE_HOLD;        break;
            case SegmentPhase::COMPLETE:    ws.phase = multi_uav_strike::WaypointStatus::PHASE_COMPLETE;    break;
        }
        ws.arrive_idx   = static_cast<uint32_t>(arrive_idx_);
        ws.arrive_total = static_cast<uint32_t>(arrive_queue_.size());
        ws.skill_idx    = static_cast<uint32_t>(skill_area_idx_);
        ws.skill_total  = static_cast<uint32_t>(skill_area_queue_.size());

        waypoint_status_pub_.publish(ws);
    }

    /** 发布当前航段队列给 RViz(到达段 / 任务段二选一) */
    void publishSegmentForRviz() {
        waypoint_queue_.clear();
        if (segment_phase_ == SegmentPhase::ARRIVE) {
            waypoint_queue_ = arrive_queue_;
            current_waypoint_index_ = arrive_idx_;
        } else if (segment_phase_ == SegmentPhase::SKILL_AREA) {
            waypoint_queue_ = skill_area_queue_;
            current_waypoint_index_ = skill_area_idx_;
        }
        publishWaypointsForRviz();
    }

    /**
     * 切到下一段(ARRIVE → SKILL_AREA → COMPLETE)
     */
    void advanceSegmentPhase() {
        if (segment_phase_ == SegmentPhase::ARRIVE) {
            if (!skill_area_queue_.empty()) {
                segment_phase_ = SegmentPhase::SKILL_AREA;
                skill_area_idx_ = 0;
                // 锁存段起点(ARRIVE→SKILL_AREA 切换时飞机位置,即最后一个 ARRIVE 航点附近)
                seg_start_x_ = current_ned_x_;
                seg_start_y_ = current_ned_y_;
                ROS_WARN("[WaypointExecutor] ===== ARRIVE to SKILL_AREA =====");
                publishSegmentForRviz();
            } else {
                segment_phase_ = SegmentPhase::COMPLETE;
                ROS_WARN("[WaypointExecutor] ===== ARRIVE to COMPLETE (no skill_area) =====");
            }
        } else if (segment_phase_ == SegmentPhase::SKILL_AREA) {
            segment_phase_ = SegmentPhase::COMPLETE;
            ROS_WARN("[WaypointExecutor] ===== SKILL_AREA to COMPLETE =====");
        }
        publishWaypointStatus();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_executor_node");
    WaypointExecutor executor;
    ros::spin();
    return 0;
}
