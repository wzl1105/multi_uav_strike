/**
 * comm_node.cpp
 * 通信节点：机间通信(地面站通信改由 typed TaskFlow 直达 mission_manager,不再走本节点)
 *
 * 职责：
 * - 广播本机位置 + 心跳给其他无人机
 * - 缓存邻居心跳供自适应调频使用
 * - 共享目标发现信息
 *
 * 订阅：
 * - /mavros/local_position/pose    - 本机位置（来自飞控）
 * - /inter_uav/uav_status          - 邻居 UAV 心跳(Phase 7)
 * - /detection/yolo_result         - YOLO检测结果（typed YoloDetection）
 * - /target_estimated_pose         - 目标估计位置（共享给队友）
 *
 * 发布：
 * - /inter_uav/self_pose           - 本机位置
 * - /inter_uav/target_info         - 目标信息
 * - /inter_uav/uav_status          - 本机状态心跳
 *
 * 注:曾经还订阅其他 UAV 的 /inter_uav/self_pose 汇总成 PoseArray 发布到
 *   /inter_uav/other_uav_poses,但全代码库已无该 topic 订阅者,已删除(2026-07-21)。
 *   邻居感知完全依赖 /inter_uav/uav_status 心跳。
 */

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

// 自定义消息：目标信息
// 后续可扩展为 custom_msgs/TargetInfo
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>

// === Phase 7: 类型化心跳 ===
#include "multi_uav_strike/InterUavStatus.h"
#include "multi_uav_strike/UavGatherStatus.h"
#include "multi_uav_strike/YoloDetection.h"
#include "multi_uav_strike/MissionState.h"   // 用于把当前 skill_type/skill_id 灌进 InterUavStatus
#include "multi_uav_strike/WorkMode.h"       // 用于把当前 work_mode 灌进 InterUavStatus

// 注:本节点原本用 NED + 每机 mavros/home_position/home 投影 GPS。
// 多机 SITL 时 PX4 各自的 HOME 可能不同(取决于 PX4 启动参数),会导致
// 算出的 GPS 各自偏移,机间心跳拿到的位置不在同一坐标系,距离算错。
// 现改为订阅 mavros/global_position/raw(HOME 无关的真实 GPS)直接用。
// world_ref_* 仅用于 RViz TF 视觉对齐,不做任何距离/坐标计算。

class CommNode {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber self_pose_sub_;          // 本机 NED 位姿(姿态用,以及 GPS 未到时 NED→GPS 兜底投影)
    ros::Subscriber gps_global_sub_;         // 本机 EKF2 GPS(mavros/global_position/global,HOME 依赖但必发)
    ros::Subscriber gps_raw_sub_;            // 本机真实 GPS(mavros/global_position/raw,不依赖 PX4 HOME)
    ros::Subscriber yolo_result_sub_;        // YOLO检测结果(typed YoloDetection)
    ros::Subscriber target_est_pose_sub_;    // 目标估计位置
    // === Phase 7: 其他 UAV 心跳 ===
    ros::Subscriber inter_uav_status_sub_;
    // === Phase 7: 本机 mission_state 订阅(把 skill_type/skill_id 灌进 InterUavStatus)===
    ros::Subscriber mission_state_sub_;
    ros::Subscriber work_mode_state_sub_;

    // ============== 发布 ==============
    ros::Publisher self_pose_pub_;           // 本机位置(发给其他无人机)
    ros::Publisher target_info_pub_;         // 目标信息
    // === Phase 7: 类型化心跳 ===
    ros::Publisher inter_uav_status_pub_;

    // ============== 状态 ==============
    geometry_msgs::PoseStamped current_self_pose_;
    bool is_self_pose_received_ = false;

    // 邻居超时时间(秒) — 仍用于心跳清理(见 interUavStatusTimerCallback)
    double neighbor_timeout_;

    // (legacy /gs/* 状态缓冲已移除,任务入口由 typed TaskFlow 直达 mission_manager)

    // 目标信息
    struct TargetInfo {
        geometry_msgs::PoseStamped pose;
        std::string target_type;  // "person", "vehicle"
        double confidence;
        ros::Time detection_time;
        bool is_shared;  // 是否已共享给队友
    };
    std::vector<TargetInfo> detected_targets_;
    double target_share_interval_;  // 目标共享间隔（秒）
    ros::Time last_target_share_time_;

    // === Phase 7: 心跳定时器(频率自适应 1/5/20Hz)===
    ros::Timer inter_uav_status_timer_;

    // === Phase 7: 心跳调频相关 ===
    double heartbeat_base_rate_;    // 基础频率 (Hz,默认 1)
    double heartbeat_near_rate_;    // 邻居 ≤40m 时频率 (Hz,默认 5)
    double heartbeat_close_rate_;   // 邻居 ≤20m 时频率 (Hz,默认 20)
    double heartbeat_near_thresh_;  // ≤此距离进入 near 频率 (m,默认 40)
    double heartbeat_close_thresh_; // ≤此距离进入 close 频率 (m,默认 20)
    double current_heartbeat_rate_; // 当前实际频率
    uint8_t current_typed_work_mode_;   // 来自 mission_manager 的 typed mode(占位)
    int32_t current_skill_type_;        // 当前 skill_type(占位)
    std::string current_skill_id_;      // 当前 skill_id(占位)
    uint8_t current_phase_;             // 来自 mission_manager 的 MissionState.phase(uav_avoidance 仲裁用)

    // 邻居心跳缓存(用于调频 + 集结判断)
    struct NeighborHeartbeat {
        std::string sn;
        multi_uav_strike::InterUavStatus::ConstPtr latest;
        ros::Time last_update;
    };
    std::vector<NeighborHeartbeat> neighbor_heartbeats_;

    // 参数
    std::string uav_name_;
    // UAV 序列号(命名约定 "<group>-<plane>" 如 "1-1");心跳 sn 用这个值,避免暴露 ROS namespace
    // 默认回落到 uav_name,保持向后兼容
    std::string uav_device_sn_;
    bool use_sim_;        // 仿真/真机切换

    // raw GPS 是否已收到(mavros/global_position/raw 回调里置 true)
    // 之前的 NED+HOME 投影逻辑已删除,这里只判断 raw 是否到
    bool is_gps_received_ = false;
    // raw vs global 跟踪,用于决定 gpsGlobalCallback 是否仍要覆盖
    bool is_raw_gps_received_ = false;
    // (2026-07-20) NED→GPS 兜底投影是否启用。
    // 当 raw/global 都还没到、但 self_pose 已到时,先发基于 NED 的"近似 GPS"心跳,
    // 让 uav_avoidance 至少有数据流;距离计算在多机 SITL 各自 HOME 相同时近似准。
    // 注:仅当 ROS 参数 ~ned_to_gps_fallback=true 时启用(默认 false,保持
    //   之前 "GPS 没到就不发心跳" 的保守行为)。如果上游 launch 设了共享 HOME
    //   (如 multi_uav_px4_sitl_test_3.launch 里 mavros 启动参数),可开 true。
    bool ned_to_gps_fallback_ = false;
    // NED→GPS 参考点(兜底投影用)。如 PX4 HOME 已在多机间对齐(launch 注入)用 HOME,
    // 否则用 launch 私有 ~ned_ref_lat/lon/alt(单位:度 / 米 AMSL)。
    double ned_ref_lat_  = 36.05801135;
    double ned_ref_lon_  = 114.5491114;
    double ned_ref_alt_  = 0.0;

public:
    CommNode() : nh_private_("~"),
        neighbor_timeout_(5.0),
        target_share_interval_(1.0) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();

        ROS_INFO("[Comm] CommNode initialized for UAV: %s (device_sn=%s)",
                 uav_name_.c_str(), uav_device_sn_.c_str());
    }

    void initParams() {
        nh_private_.param<std::string>("uav_name", uav_name_, "uav0");
        // device_sn 是心跳实际发的 sn;空 → 回落 uav_name,避免改了 launch 之后还得改所有地方
        nh_private_.param<std::string>("uav_device_sn", uav_device_sn_, uav_name_);
        nh_private_.param<double>("neighbor_timeout", neighbor_timeout_, 5.0);
        nh_private_.param<double>("target_share_interval", target_share_interval_, 1.0);
        nh_private_.param<bool>("use_sim", use_sim_, true);

        // === Phase 7: 心跳调频参数 ===
        nh_private_.param<double>("heartbeat_base_rate",  heartbeat_base_rate_,  1.0);
        nh_private_.param<double>("heartbeat_near_rate",  heartbeat_near_rate_,  5.0);
        nh_private_.param<double>("heartbeat_close_rate", heartbeat_close_rate_, 20.0);
        nh_private_.param<double>("heartbeat_near_thresh",  heartbeat_near_thresh_,  40.0);
        nh_private_.param<double>("heartbeat_close_thresh", heartbeat_close_thresh_, 20.0);
        current_heartbeat_rate_ = heartbeat_base_rate_;
        current_typed_work_mode_ = 0;
        current_skill_type_ = 0;
        current_phase_ = 0;  // GROUND_IDLE
        // GPS 改用 mavros/global_position/raw,不再需要 ref_lat/lon/alt 投影

        // === (2026-07-20) NED→GPS 兜底投影配置 ===
        //   默认关闭(0/false)。launch 显式传 ~ned_to_gps_fallback:=true 时启用。
        nh_private_.param<bool>("ned_to_gps_fallback", ned_to_gps_fallback_, false);
        nh_private_.param<double>("ned_ref_lat", ned_ref_lat_, 36.05801135);
        nh_private_.param<double>("ned_ref_lon", ned_ref_lon_, 114.5491114);
        nh_private_.param<double>("ned_ref_alt", ned_ref_alt_, 0.0);
    }

    void initSubscribers() {
        // 本机位置
        // 仿真：订阅 quad/pose；实机：订阅 mavros/local_position/pose
        if (use_sim_) {
            self_pose_sub_ = nh_.subscribe(
                "quad/pose", 10,
                &CommNode::selfPoseCallback, this);
        } else {
            self_pose_sub_ = nh_.subscribe(
                "mavros/local_position/pose", 10,
                &CommNode::selfPoseCallback, this);
        }

        // 旧 otherUavPosesCallback 已删除 — 不再订阅邻居 /inter_uav/self_pose,
        //   邻居感知完全依赖 /inter_uav/uav_status 心跳(见 interUavStatusTimerCallback)。

        // YOLO 检测结果（目标发现）
        yolo_result_sub_ = nh_.subscribe(
            "detection/yolo_result", 10,
            &CommNode::yoloResultCallback, this);

        // 目标估计位置（用于共享给队友）
        target_est_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &CommNode::targetEstPoseCallback, this);

        // === Phase 7: 其他 UAV 心跳(全局 topic)===
        inter_uav_status_sub_ = nh_.subscribe(
            "/inter_uav/uav_status", 50,
            &CommNode::interUavStatusCallback, this);

        // === Phase 7: 本机 mission_state(1Hz,来自 mission_manager)===
        //   用来把 current_skill_type_/skill_id_/phase_ 灌进 InterUavStatus,
        //   否则心跳里 skill_type 永远是 0(占位),多机 gather 同步无法判别伙伴当前阶段。
        mission_state_sub_ = nh_.subscribe(
            "mission/mission_state", 10,
            &CommNode::missionStateCallback, this);
        work_mode_state_sub_ = nh_.subscribe(
            "mission/current_work_mode", 10,
            &CommNode::workModeStateCallback, this);

        // === mavros GPS 双源(2026-07-20 改造)===
        //   /mavros/global_position/global = EKF2 估计,HOME 依赖,但 mavros 默认必发
        //   /mavros/global_position/raw    = 传感器原始,HOME 无关,但部分配置不发
        // 双订阅:/global 永远填进 current_self_pose_(保证邻居持续有数据流),
        //        /raw 一旦到达就覆盖,得到 HOME 无关的真值。
        // 注:之前只有 /raw,某些 SITL 配置下 raw 不发 → comm 不发心跳 → 邻居全空。
        gps_global_sub_ = nh_.subscribe(
            "mavros/global_position/global", 10,
            &CommNode::gpsGlobalCallback, this);
        gps_raw_sub_ = nh_.subscribe(
            "mavros/global_position/raw", 10,
            &CommNode::gpsRawCallback, this);
    }

    void initPublishers() {
        // 本机位置（发给其他无人机）- 用相对路径，自动加 namespace
        self_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/inter_uav/self_pose", 10);

        // 旧 other_uav_poses_pub_ 已删除 — PoseArray 汇总无订阅者,机间避让改由 uav_avoidance_node
        //   直接订阅心跳(见 /inter_uav/uav_status)。comm_node 不再汇总邻居位置列表。

        // 目标信息（共享给队友）
        target_info_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "inter_uav/target_info", 10);

        // === Phase 7: 类型化心跳(全局 topic)===
        inter_uav_status_pub_ = nh_.advertise<multi_uav_strike::InterUavStatus>(
            "/inter_uav/uav_status", 50);
    }

    void initTimers() {
        // === Phase 7: 心跳定时器(频率自适应)===
        inter_uav_status_timer_ = nh_.createTimer(
            ros::Duration(1.0 / current_heartbeat_rate_),
            &CommNode::interUavStatusTimerCallback, this);
    }

    // ============== 回调函数 ==============

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // NED 位置用作 GPS 兜底投影(当 ned_to_gps_fallback=true 且 /global + /raw 都没到)。
        // 姿态信息也保留,给需要朝向的下游。
        current_self_pose_.pose.orientation = msg->pose.orientation;
        current_self_pose_.header = msg->header;
        current_self_pose_.header.frame_id = uav_name_;
        // 如果 GPS 还没到,这里用 NED→GPS 投影兜底填一下,避免 self_pose 不发导致邻居全空
        // (2026-07-20 修复:之前这里条件 if(is_gps_received_) 直接 return,
        //  某些 SITL 配置 raw/global 都不发时邻居 = 0)
        if (!is_gps_received_ && ned_to_gps_fallback_) {
            double gx, gy, gz;
            nedToGpsFallback(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                             gx, gy, gz);
            current_self_pose_.pose.position.x = gx;
            current_self_pose_.pose.position.y = gy;
            current_self_pose_.pose.position.z = gz;
        }
        // GPS 由 gpsRawCallback / gpsGlobalCallback 写入。
        if (is_gps_received_ || (ned_to_gps_fallback_ && is_self_pose_received_)) {
            self_pose_pub_.publish(current_self_pose_);
        }
        is_self_pose_received_ = true;
    }

    /**
     * mavros/global_position/global 回调 — EKF2 估计,HOME 依赖,但 mavros 默认必发。
     * 作为 primary,保证 /inter_uav/self_pose 持续有数据流。
     * 一旦收到 /raw,这里就不再覆盖(避免 /global 把 /raw 真值覆盖回去)。
     */
    void gpsGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (is_raw_gps_received_) {
            return;  // /raw 真值已生效,不再被 /global 覆盖
        }
        current_self_pose_.pose.position.x = msg->latitude;
        current_self_pose_.pose.position.y = msg->longitude;
        current_self_pose_.pose.position.z = msg->altitude;
        current_self_pose_.header = msg->header;
        current_self_pose_.header.frame_id = uav_name_;
        if (!is_gps_received_) {
            is_gps_received_ = true;
            ROS_WARN("[Comm] >>>> First GPS received (/global, EKF2 HOME-dep): lat=%.7f lon=%.7f alt=%.2f",
                     msg->latitude, msg->longitude, msg->altitude);
        }
        is_self_pose_received_ = true;
        self_pose_pub_.publish(current_self_pose_);
    }

    /**
     * mavros/global_position/raw 回调 — 真实 GPS,不依赖 PX4 HOME。
     * 多机 SITL 各自 PX4 HOME 不同时,只有这个 topic 能保证所有 UAV 在同一坐标系。
     * 标记 is_raw_gps_received_=true,后续 /global 不再覆盖。
     */
    void gpsRawCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        current_self_pose_.pose.position.x = msg->latitude;
        current_self_pose_.pose.position.y = msg->longitude;
        current_self_pose_.pose.position.z = msg->altitude;
        current_self_pose_.header = msg->header;
        current_self_pose_.header.frame_id = uav_name_;
        if (!is_raw_gps_received_) {
            is_raw_gps_received_ = true;
            ROS_WARN("[Comm] >>>> First raw GPS received (HOME-indep, overrides /global): lat=%.7f lon=%.7f alt=%.2f",
                     msg->latitude, msg->longitude, msg->altitude);
        }
        if (!is_gps_received_) {
            is_gps_received_ = true;
        }
        is_self_pose_received_ = true;
        self_pose_pub_.publish(current_self_pose_);
    }

    /**
     * NED→GPS 兜底投影(本地 NED 用 ned_ref_* 转 WGS84)
     * 仅在 ned_to_gps_fallback=true 时由 selfPoseCallback 调用。
     * 精度依赖 ned_ref_* 与 PX4 HOME 的一致性;若各机 HOME 不同则不要启用。
     */
    void nedToGpsFallback(double ned_x, double ned_y, double ned_z,
                          double& lat, double& lon, double& alt) {
        const double EARTH_R = 6378137.0;
        const double DEG_PER_RAD = 180.0 / M_PI;
        lat = ned_ref_lat_ + (ned_x / EARTH_R) * DEG_PER_RAD;
        double cos_lat = std::cos(ned_ref_lat_ * M_PI / 180.0);
        if (cos_lat < 1e-6) cos_lat = 1e-6;
        lon = ned_ref_lon_ + (ned_y / (EARTH_R * cos_lat)) * DEG_PER_RAD;
        alt = ned_ref_alt_ - ned_z;  // NED z 下向 → AMSL 上向
    }

    /**
     * (deprecated) 原 NED -> GPS 投影 — 已删除,改用 gpsRawCallback。
     * 旧代码用每机 PX4 home 做参考,多机 SITL 时各自 HOME 不同会算错距离。
     * 如果未来需要回退,在 launch 显式传 ref_lat/lon/alt 即可重建此函数。
     */
    void nedToGps_DELETED(double ned_x, double ned_y, double ned_z,
                           double& lat, double& lon, double& alt) {
        (void)ned_x; (void)ned_y; (void)ned_z;
        (void)lat;   (void)lon;   (void)alt;
    }

    // 旧 otherUavPosesCallback / neighborCheckTimerCallback / publishOtherUavPoses 已删除 —
    //   原 PoseArray 汇总管线无订阅者(2026-07-21),机间感知全部走 /inter_uav/uav_status 心跳。

    void yoloResultCallback(const multi_uav_strike::YoloDetection::ConstPtr& msg) {
        // 收到 typed YoloDetection(multi_uav_strike/YoloDetection)
        // comm_node 当前仅做日志/转发占位,具体目标信息上报逻辑后续再做
        if (!msg->is_in_fov) {
            return;  // 矩形 FOV 外不处理
        }
        ROS_DEBUG("[Comm] YOLO detection: %s conf=%.2f bbox=[%.2f,%.2f,%.2f,%.2f]",
                  msg->label.c_str(), msg->confidence,
                  msg->x_min, msg->y_min, msg->x_max, msg->y_max);
    }

    void targetEstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 收到目标估计位置，共享给队友
        ros::Time now = ros::Time::now();

        // 限制目标共享频率
        if ((now - last_target_share_time_).toSec() < target_share_interval_) {
            return;
        }

        // 共享目标信息
        geometry_msgs::PoseStamped target_info = *msg;
        target_info.header.frame_id = uav_name_;  // 标记是哪个无人机发现的目标
        target_info_pub_.publish(target_info);

        last_target_share_time_ = now;
    }

    // ============== 定时器回调 ==============

    // 旧 neighborCheckTimerCallback / publishOtherUavPoses 已删除 — 见上方注释。

    // ============== Phase 7: 类型化心跳 ==============

    /**
     * 收到其他 UAV 的心跳 — 缓存 + 用于自适应调频
     */
    void interUavStatusCallback(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        // 自环过滤:心跳 sn 已切到 device_sn("1-1"格式),不能再用 ROS namespace 比
        if (msg->sn == uav_device_sn_) {
            return;  // 忽略自己的
        }
        bool found = false;
        for (auto& nh : neighbor_heartbeats_) {
            if (nh.sn == msg->sn) {
                nh.latest = msg;
                nh.last_update = ros::Time::now();
                found = true;
                break;
            }
        }
        if (!found) {
            NeighborHeartbeat n;
            n.sn = msg->sn;
            n.latest = msg;
            n.last_update = ros::Time::now();
            neighbor_heartbeats_.push_back(n);
            ROS_INFO("[Comm] New heartbeat neighbor: %s", msg->sn.c_str());
        }
    }

    /**
     * 心跳定时器回调:发布本机 InterUavStatus + 自适应调频
     */
    void interUavStatusTimerCallback(const ros::TimerEvent&) {
        if (!is_self_pose_received_) return;

        // 1. 发本机心跳
        multi_uav_strike::InterUavStatus hs;
        // 心跳 sn 用 device_sn("1-1" 格式)而不是 ROS namespace,保证 uav_avoidance
        // 在仿真/实机下拿到的都是同一种 id,uav_avoidance 可以靠字典序仲裁优先级。
        hs.sn            = uav_device_sn_;
        hs.work_mode     = current_typed_work_mode_;
        hs.skill_type    = current_skill_type_;
        hs.skill_id      = current_skill_id_;
        hs.phase         = current_phase_;
        hs.timestamp_us  = ros::Time::now().toNSec() / 1000;
        hs.lat           = current_self_pose_.pose.position.x;
        hs.lon           = current_self_pose_.pose.position.y;
        hs.alt           = current_self_pose_.pose.position.z;
        hs.yaw           = 0.0;  // 简化:暂不发 yaw
        inter_uav_status_pub_.publish(hs);

        // 2. 清理超时心跳
        ros::Time now = ros::Time::now();
        neighbor_heartbeats_.erase(
            std::remove_if(neighbor_heartbeats_.begin(), neighbor_heartbeats_.end(),
                           [&](const NeighborHeartbeat& n) {
                               return (now - n.last_update).toSec() > neighbor_timeout_;
                           }),
            neighbor_heartbeats_.end());

        // 3. 自适应调频:取最近邻居距离,据此选频率
        double min_dist = computeClosestNeighborDistance();
        double new_rate = heartbeat_base_rate_;
        if (min_dist < heartbeat_close_thresh_) {
            new_rate = heartbeat_close_rate_;
        } else if (min_dist < heartbeat_near_thresh_) {
            new_rate = heartbeat_near_rate_;
        }

        // 频率变化时才 setPeriod(避免每个 tick 重建 timer)
        if (std::fabs(new_rate - current_heartbeat_rate_) > 0.01) {
            current_heartbeat_rate_ = new_rate;
            inter_uav_status_timer_.setPeriod(ros::Duration(1.0 / current_heartbeat_rate_));
            ROS_INFO("[Comm] >>>> Heartbeat rate adjusted to %.1f Hz (closest UAV: %.1f m)",
                     current_heartbeat_rate_, min_dist);
        }
    }

    /**
     * mission_state 回调(mission_manager 1Hz 上报) — 把当前 skill_type/skill_id
     * 缓存下来,InterUavStatus 发布时携带。
     *   -1 表示无 active skill,心跳里照样发出 -1(下游应忽略)。
     */
    void missionStateCallback(const multi_uav_strike::MissionState::ConstPtr& msg) {
        current_skill_type_ = msg->skill_type;  // 可能是 -1(无 active skill)
        current_skill_id_   = msg->skill_id;
        // phase 字段也灌进心跳,uav_avoidance 用 (work_mode, phase) 联合判定优先级
        current_phase_      = msg->phase;
    }

    /**
     * work_mode_state 回调(mission_manager 1Hz 上报) — 把当前 work_mode 灌进心跳
     */
    void workModeStateCallback(const multi_uav_strike::WorkMode::ConstPtr& msg) {
        current_typed_work_mode_ = msg->mode;
    }

    /**
     * 计算最近邻居距离(米),无邻居返回 1e9
     * 用心跳缓存的 lat/lon/alt 做粗算(同 uav_avoidance_node 算法)
     */
    double computeClosestNeighborDistance() {
        if (neighbor_heartbeats_.empty()) return 1e9;

        double self_lat = current_self_pose_.pose.position.x;
        double self_lon = current_self_pose_.pose.position.y;
        double self_alt = current_self_pose_.pose.position.z;

        double min_dist = 1e9;
        for (const auto& n : neighbor_heartbeats_) {
            if (!n.latest) continue;
            double dlat = n.latest->lat - self_lat;
            double dlon = n.latest->lon - self_lon;
            double dalt = n.latest->alt - self_alt;
            double cos_lat = std::max(0.1, std::cos(self_lat * M_PI / 180.0));
            double dx = dlat * 111000.0;
            double dy = dlon * 111000.0 * cos_lat;
            double dist = std::sqrt(dx*dx + dy*dy + dalt*dalt);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "comm_node");
    CommNode comm_node;
    ros::spin();
    return 0;
}
