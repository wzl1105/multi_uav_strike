#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include "multi_uav_strike/YoloDetection.h"
#include "multi_uav_strike/YoloDetections.h"
#include "multi_uav_strike/SimTarget.h"
#include "multi_uav_strike/SimTargets.h"
#include "multi_uav_strike/LidarMeasurement.h"

using namespace std;

/**
 * 云台模拟器 + 检测模拟器 合并版 (2026-07-07)
 *
 * 设计要点:
 * 1. 云台只能控制俯仰, 偏航固定为 0
 * 2. 相机俯仰约定: 0 = 水平, -π/2 = 垂直下视
 *    也就是说 gimbal_pitch_ ∈ [-π/2, 0]
 * 3. 控制算法: 先判断目标在当前云台角度下是否在视野中
 *    - 在视野中 → 让云台指向目标(计算 desired_pitch)
 *    - 不在视野中 → 回到默认角度 (default_pitch_deg, 默认 -45)
 * 4. FOV 判定: 矩形视场(类 detection_simulator), 把目标投到相机系
 * 5. 替代 detection_simulator_node: 在 gimbal 内做检测,
 *    发布 /<ns>/detection/yolo_result 和 /<ns>/detection/target_in_view
 * 6. 保留原 /target_los_angle, /gimbal_pose, /camera/pose
 *
 * 与 detection_simulator 的关键差异:
 * - 矩形 FOV 中心是当前云台指向(不再是固定的 cam_pitch=-90)
 * - 目标"在不在 FOV 中"取决于云台当前角度, 而非机体固定相机角度
 */

template<typename T>
static inline T clampv(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

class GimbalSimulator {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // 订阅
    ros::Subscriber target_sub_;
    ros::Subscriber uav_pose_sub_;

    // 发布
    ros::Publisher los_angle_pub_;     // 目标 LOS (保持)
    ros::Publisher gimbal_pose_pub_;   // 云台姿态 (RViz 用, 保持)
    ros::Publisher camera_pose_pub_;   // 相机世界姿态 (保持)
    std::string viz_frame_;            // RViz 帧名 "<ns>/map"(多机分离由 static TF 偏移)
    ros::Publisher yolo_pub_;          // typed YoloDetection (单条,主目标,兼容 comm_node)
    ros::Publisher yolo_results_pub_;  // typed YoloDetections (多目标,供 target_estimator)
    ros::Publisher in_fov_pub_;        // Bool (合并自 detection_simulator)
    ros::Publisher lidar_range_pub_;
    ros::Publisher lidar_measurement_pub_;
    ros::Publisher lidar_beam_pub_;
    ros::Publisher building_marker_pub_;

    ros::Timer control_timer_;
    ros::Timer lidar_timer_;
    uint32_t frame_seq_ = 0;

    // === FOV / 图像参数 ===
    int image_width_;
    int image_height_;
    double fov_h_deg_;       // 水平 FOV (度)
    double fov_v_deg_;       // 垂直 FOV (度, 0 表示按方形像素从 fov_h 推导)
    double fov_h_rad_;
    double fov_v_rad_;
    double focal_x_, focal_y_;
    std::string label_;

    // === 控制参数 ===
    double gimbal_p_gain_;
    double max_pitch_rate_;
    double loop_freq_;          // 云台控制循环 Hz(默认 100)
    double yolo_pub_freq_;      // yolo_results / yolo (单条) 实际发布频率 Hz(默认 10)
    bool use_sim_;
    double image_noise_std_dev_;
    std::string uav_pose_topic_;
    double desired_pitch, desired_yaw;

    bool lidar_enabled_;
    double lidar_min_range_, lidar_max_range_, lidar_frequency_;
    double building_x_, building_y_, building_z_;
    double building_size_x_, building_size_y_, building_size_z_;
    double ground_z_;
    double ground_range_abs_tolerance_;
    double ground_range_rel_tolerance_;

    // 固定模拟 JPEG(测试用,真实 gimbal 接入后由相机抓拍帧替换)
    std::vector<uint8_t> fake_jpeg_;
    std::string fake_jpeg_path_;

    // === 云台参数 (新约定: pitch ∈ [-π/2, 0]) ===
    //   current_gimbal_pitch_ = 0      → 水平
    //   current_gimbal_pitch_ = -π/2   → 垂直下视
    //   default_pitch_deg_             → 目标丢失时回退到此角度(默认 -45)
    double current_gimbal_pitch_ = 0.0;
    double current_uav_yaw_      = 0.0;   // 仅保留机头朝向,用于稳像世界姿态构造
    double default_pitch_deg_;
    double default_pitch_rad_;

    // 噪声
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_dist_;

    // === 状态 ===
    std::vector<multi_uav_strike::SimTarget> targets_;   // 全部仿真目标(来自 /sim_targets)
    geometry_msgs::PoseStamped current_uav_pose_;   // NWU
    bool is_target_received_ = false;
    bool is_uav_pose_received_ = false;

    // === 每 loop tick 缓存的相机-目标投影(逐目标) ===
    //   由 recomputeAllProjections() 在循环开头一次性算出所有目标的投影,
    //   之后 FOV 判定 / 像素投影 / distance / camera pose / 逐目标 YoloDetection 都读缓存。
    struct TargetProj {
        uint32_t    id       = 0;
        std::string label;
        geometry_msgs::Point pos;      // 目标 NWU 世界坐标
        bool   visible   = false;      // 在 FOV 里
        double u_px      = 0.0;        // 像素 u
        double v_px      = 0.0;        // 像素 v
        double distance  = 0.0;        // 相机系距离
    };
    Eigen::Quaterniond cached_q_cam_world_;
    bool   cached_proj_valid_ = false;             // 数据齐了(有目标 + 有 pose)
    std::vector<TargetProj> projections_;          // 与 targets_ 对齐
    int    primary_idx_ = -1;                      // projections_ 内的主目标索引(FOV 内最近);-1=无

public:
    GimbalSimulator() {
        nh_private_ = ros::NodeHandle("~");

        // FOV / 图像
        nh_private_.param<int>("image_width", image_width_, 1920);
        nh_private_.param<int>("image_height", image_height_, 1080);
        nh_private_.param<double>("fov_h_deg", fov_h_deg_, 66.0);
        nh_private_.param<double>("fov_v_deg", fov_v_deg_, 52.0);   // 0 = 按方形像素从 fov_h 推导
        nh_private_.param<std::string>("label", label_, "general_target");

        // 控制
        nh_private_.param<double>("gimbal_p_gain", gimbal_p_gain_, 0.8);
        nh_private_.param<double>("max_pitch_rate", max_pitch_rate_, 1.2);
        nh_private_.param<double>("loop_freq", loop_freq_, 100.0);
        // === yolo 仿真发布频率 ===
        //   真实 YOLO 一般 10~30Hz。loop_freq_=100 让 yolo 也跟着 100Hz 太假了,
        //   会让 target_estimator 的 hit_count 语义完全错位(20 hits @ 100Hz = 0.2s)。
        //   这里独立节流:yolo_pub_freq_ 决定 yolo_results / 单条 yolo 的发布频率,
        //   云台控制仍按 loop_freq_ 跑。yolo_pub_freq_ 必须 ≤ loop_freq_。
        nh_private_.param<double>("yolo_pub_freq", yolo_pub_freq_, 10.0);
        if (yolo_pub_freq_ > loop_freq_) {
            ROS_WARN("[GimbalSim] yolo_pub_freq=%.1f > loop_freq=%.1f, clamp to loop_freq",
                     yolo_pub_freq_, loop_freq_);
            yolo_pub_freq_ = loop_freq_;
        }
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<double>("image_noise_std_dev", image_noise_std_dev_, 0.0);
        nh_private_.param<bool>("lidar_enabled", lidar_enabled_, false);
        nh_private_.param<double>("lidar_min_range", lidar_min_range_, 0.2);
        nh_private_.param<double>("lidar_max_range", lidar_max_range_, 300.0);
        nh_private_.param<double>("lidar_frequency", lidar_frequency_, 9.0);
        nh_private_.param<double>("building_x", building_x_, 320.0);
        nh_private_.param<double>("building_y", building_y_, 0.0);
        nh_private_.param<double>("building_z", building_z_, 150.0);
        nh_private_.param<double>("building_size_x", building_size_x_, 10.0);
        nh_private_.param<double>("building_size_y", building_size_y_, 20.0);
        nh_private_.param<double>("building_size_z", building_size_z_, 300.0);
        nh_private_.param<double>("ground_z", ground_z_, 0.0);
        nh_private_.param<double>("ground_range_abs_tolerance",
                                  ground_range_abs_tolerance_, 10);
        nh_private_.param<double>("ground_range_rel_tolerance",
                                  ground_range_rel_tolerance_, 0.2);

        // 加载固定模拟 JPEG:放到 YoloDetection.img_data 给下游(mission_manager →
        // starling_bridge → DEVICE_TARGETS 0x2001)组装上行报文;真实 gimbal 接入后
        // 此参数失效,由相机抓拍帧替换。
        nh_private_.param<std::string>("fake_jpeg_path", fake_jpeg_path_,
                                        ros::package::getPath("multi_uav_strike") +
                                        "/config/fake_target.jpg");
        loadFakeJpeg(fake_jpeg_path_);

        // 新约定下的回退角
        nh_private_.param<double>("default_pitch_deg", default_pitch_deg_, 45.0);

        if (use_sim_) {
            uav_pose_topic_ = "quad/pose";
        } else {
            uav_pose_topic_ = "mavros/local_position/pose";
        }

        // 焦点长度(focal length in pixels)
        // focal_x / focal_y 独立计算,各自对应 fov_h / fov_v 半角的 tan
        //   focal_x = (W/2) / tan(fov_h/2)
        //   focal_y = (H/2) / tan(fov_v/2)
        // 若 fov_v_deg=0 (方形像素),则按 W/H 比例从 fov_h 推 fov_v
        fov_h_rad_ = fov_h_deg_ * M_PI / 180.0;
        if (fov_v_deg_ > 0.0) {
            fov_v_rad_ = fov_v_deg_ * M_PI / 180.0;
        } else {
            double aspect = static_cast<double>(image_width_) / image_height_;
            fov_v_rad_ = 2.0 * std::atan(std::tan(fov_h_rad_ / 2.0) / aspect);
        }
        focal_x_ = (image_width_  / 2.0) / std::tan(fov_h_rad_ / 2.0);
        focal_y_ = (image_height_ / 2.0) / std::tan(fov_v_rad_ / 2.0);

        default_pitch_rad_ = clampv(default_pitch_deg_ * M_PI / 180.0, 0.0, M_PI/2.0);
        current_gimbal_pitch_ = default_pitch_rad_;

        // 噪声
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        noise_dist_ = std::normal_distribution<double>(0.0, image_noise_std_dev_);

        // 订阅
        target_sub_ = nh_.subscribe("/sim_targets", 10,
                                    &GimbalSimulator::targetsCallback, this);
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10,
                                      &GimbalSimulator::uavPoseCallback, this);

        // 发布(保持)
        los_angle_pub_   = nh_.advertise<geometry_msgs::Point>("target_los_angle", 10);
        gimbal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gimbal_pose", 10);
        camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera/pose", 10);

        // 发布(合并自 detection_simulator)
        std::string ns = nh_.getNamespace();
        // RViz 帧名:去掉命名空间前导 '/' 后拼 "/map"(如 /uav0 → uav0/map);无命名空间 → map
        {
            std::string vns = ns;
            if (!vns.empty() && vns[0] == '/') vns = vns.substr(1);
            viz_frame_ = vns.empty() ? "map" : (vns + "/map");
        }
        std::string yolo_t = "detection/yolo_result";
        if (!ns.empty() && ns != "/") yolo_t = ns + "/" + yolo_t;
        if (yolo_t.find("//") == 0) yolo_t = yolo_t.substr(1);
        yolo_pub_ = nh_.advertise<multi_uav_strike::YoloDetection>(yolo_t, 10);

        // 多目标批量检测(供 target_estimator)
        std::string yolos_t = "detection/yolo_results";
        if (!ns.empty() && ns != "/") yolos_t = ns + "/" + yolos_t;
        if (yolos_t.find("//") == 0) yolos_t = yolos_t.substr(1);
        yolo_results_pub_ = nh_.advertise<multi_uav_strike::YoloDetections>(yolos_t, 10);

        std::string fov_t = "detection/target_in_view";
        if (!ns.empty() && ns != "/") fov_t = ns + "/" + fov_t;
        if (fov_t.find("//") == 0) fov_t = fov_t.substr(1);
        in_fov_pub_ = nh_.advertise<std_msgs::Bool>(fov_t, 10);
        if (lidar_enabled_) {
            lidar_range_pub_ = nh_.advertise<sensor_msgs::Range>("lidar/range", 10);
            lidar_measurement_pub_ = nh_.advertise<multi_uav_strike::LidarMeasurement>(
                "lidar/measurement", 10);
            lidar_beam_pub_ = nh_.advertise<visualization_msgs::Marker>("lidar/beam_marker", 1, true);
            building_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_building_marker", 1, true);
        }

        // 100Hz 主循环
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_freq_),
                                         &GimbalSimulator::controlLoopCallback, this);
        if (lidar_enabled_ && lidar_frequency_ > 0.0) {
            lidar_timer_ = nh_.createTimer(ros::Duration(1.0 / lidar_frequency_),
                                           &GimbalSimulator::lidarTimerCallback, this);
        }

        ROS_INFO("[GimbalSim] image=%dx%d fov=(%.1fh x %.1fv deg) focal=(%.1f, %.1f)",
                 image_width_, image_height_, fov_h_deg_,
                 (fov_v_deg_ > 0.0 ? fov_v_deg_ : fov_v_rad_ * 180.0 / M_PI),
                 focal_x_, focal_y_);
        ROS_INFO("[GimbalSim] pitch conv: 0=horizontal, -90=down; default_pitch=%.1fdeg",
                 default_pitch_deg_);
        ROS_INFO("[GimbalSim] uav_pose=%s, publishing target_los_angle, gimbal_pose, "
                 "camera/pose, %s, %s",
                 uav_pose_topic_.c_str(), yolo_t.c_str(), fov_t.c_str());
    }

    // ====================================================================
    // 订阅回调
    // ====================================================================

    void targetsCallback(const multi_uav_strike::SimTargets::ConstPtr& msg) {
        targets_ = msg->targets;
        is_target_received_ = !targets_.empty();
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 与原版保持一致:NED/ENU → NWU
        if (use_sim_) {
            current_uav_pose_.pose.position.x =  msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;
            current_uav_pose_.pose.orientation.w =  msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x =  msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // ENU → NWU: x = y_enu, y = -x_enu, z = z_enu
            current_uav_pose_.pose.position.x =  msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z =  msg->pose.position.z;
            // 四元数 mavros(ENU 世界 + FRD 机体) -> 内部(NWU 世界 + FLU 机体):
            //   q_internal = q_W · q_mavros · q_B
            //   q_W = (√2/2, 0, 0, -√2/2)  ENU→NWU: 绕世界 Z 旋转 -90° (E→N, N→W)
            //   q_B = (0, 1, 0, 0)          FRD→FLU: 绕机体 X 旋转 180° (F→-F≈F, R→L, D→U)
            // 展开后 (Hamilton 约定, q_B = (0,1,0,0) 会交换 w↔x、y↔-z):
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_uav_pose_.pose.orientation.w = -kSqrtHalf * (qe.x + qe.y);
            current_uav_pose_.pose.orientation.x =  kSqrtHalf * (qe.w + qe.z);
            current_uav_pose_.pose.orientation.y =  kSqrtHalf * (qe.z - qe.w);
            current_uav_pose_.pose.orientation.z =  kSqrtHalf * (qe.x - qe.y);
        }
        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;

        // 提取机头朝向 (NWU 世界 + FLU 机体 下的 yaw)
        //   yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        // 用于稳像世界姿态构造,与 roll/pitch 解耦
        const auto& qn = current_uav_pose_.pose.orientation;
        current_uav_yaw_ = std::atan2(2.0 * (qn.w * qn.z + qn.x * qn.y),
                                      1.0 - 2.0 * (qn.y * qn.y + qn.z * qn.z));

        is_uav_pose_received_ = true;
    }

    // ====================================================================
    // 主循环 (100Hz)
    // ====================================================================

    void controlLoopCallback(const ros::TimerEvent&) {
        // === 1. 一次性算出所有目标的相机投影(后续都读缓存),并选主目标 ===
        recomputeAllProjections();

        // 数据没到齐:发"空帧"+保持当前云台角度
        if (!cached_proj_valid_) {
            ROS_WARN_THROTTLE(5.0, "[GimbalSim] waiting for targets/pose ...");
            publishYoloDetections();
            return;
        }

        // === 2. 决定 desired_pitch(指向主目标;无主目标则回退默认角) ===
        bool has_primary = (primary_idx_ >= 0);
        if (has_primary) {
            const TargetProj& p = projections_[primary_idx_];
            desired_pitch = computePitchAndYawToTarget(p.pos.x, p.pos.y);
            ROS_INFO_THROTTLE(5.0, "[GimbalSim] primary target id=%u label=%s in view: "
                                "u_px=%.1f v_px=%.1f distance=%.2fm desired_pitch=%.2fdeg "
                                "(total_visible=%zu)",
                              p.id, p.label.c_str(), p.u_px, p.v_px, p.distance,
                              desired_pitch * 180.0 / M_PI, countVisible());
        } else {
            desired_pitch = default_pitch_rad_;
        }

        // === 3. 云台比例控制 ===
        updateGimbalAngles(desired_pitch);

        // === 4. 发布 ===
        publishLOSAngle(desired_pitch, desired_yaw);       // target_los_angle(指向主目标)
        publishGimbalPose();                  // gimbal_pose (RViz)
        publishCameraPose();                  // camera/pose (用缓存的 q_cam_world_)
        publishYoloDetections();              // YoloDetections(全部可见目标) + 单条 YoloDetection(主目标)
    }

    double rayBuildingDistance(double ox, double oy, double oz,
                               double dx, double dy, double dz) const {
        const double mins[3] = {
            building_x_ - building_size_x_ * 0.5,
            building_y_ - building_size_y_ * 0.5,
            building_z_ - building_size_z_ * 0.5};
        const double maxs[3] = {
            building_x_ + building_size_x_ * 0.5,
            building_y_ + building_size_y_ * 0.5,
            building_z_ + building_size_z_ * 0.5};
        const double origin[3] = {ox, oy, oz};
        const double dir[3] = {dx, dy, dz};
        double t_min = 0.0;
        double t_max = lidar_max_range_;
        for (int axis = 0; axis < 3; ++axis) {
            if (std::fabs(dir[axis]) < 1e-9) {
                if (origin[axis] < mins[axis] || origin[axis] > maxs[axis]) {
                    return std::numeric_limits<double>::infinity();
                }
                continue;
            }
            double t1 = (mins[axis] - origin[axis]) / dir[axis];
            double t2 = (maxs[axis] - origin[axis]) / dir[axis];
            if (t1 > t2) std::swap(t1, t2);
            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);
            if (t_min > t_max) return std::numeric_limits<double>::infinity();
        }
        if (t_min < lidar_min_range_ || t_min > lidar_max_range_) {
            return std::numeric_limits<double>::infinity();
        }
        return t_min;
    }

    double rayGroundDistance(double oz, double dz) const {
        if (dz >= -1e-9 || oz <= ground_z_) {
            return std::numeric_limits<double>::infinity();
        }
        const double t = (ground_z_ - oz) / dz;
        if (t < lidar_min_range_ || t > lidar_max_range_) {
            return std::numeric_limits<double>::infinity();
        }
        return t;
    }

    void publishBuildingMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = viz_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "sim_building";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = building_x_;
        marker.pose.position.y = building_y_;
        marker.pose.position.z = building_z_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = building_size_x_;
        marker.scale.y = building_size_y_;
        marker.scale.z = building_size_z_;
        marker.color.r = 0.45f;
        marker.color.g = 0.48f;
        marker.color.b = 0.52f;
        marker.color.a = 0.9f;
        marker.lifetime = ros::Duration(0.0);
        building_marker_pub_.publish(marker);
    }

    void lidarTimerCallback(const ros::TimerEvent&) {
        if (!lidar_enabled_ || !is_uav_pose_received_) return;
        const ros::Time now = ros::Time::now();

        const geometry_msgs::Point& p = current_uav_pose_.pose.position;
        const auto& oq = current_uav_pose_.pose.orientation;
        Eigen::Quaterniond q_body_world(oq.w, oq.x, oq.y, oq.z);
        if (q_body_world.norm() < 1e-9) q_body_world = Eigen::Quaterniond::Identity();
        q_body_world.normalize();
        const Eigen::Vector3d ray_dir = q_body_world * Eigen::Vector3d::UnitX();
        // 阶段1：场景只生成一个原始距离，不把命中的物体类型传给分类器。
        const double building_range = rayBuildingDistance(
            p.x, p.y, p.z, ray_dir.x(), ray_dir.y(), ray_dir.z());
        const double ground_range = rayGroundDistance(p.z, ray_dir.z());
        const double range = std::min(building_range, ground_range);

        // 阶段2：只根据高度、射线方向和原始距离反算是否为地面回波。
        // HIT_BUILDING 在此表示“有限的非地面障碍回波”，不使用楼体真值分类。
        uint8_t hit_type = multi_uav_strike::LidarMeasurement::HIT_NONE;
        if (std::isfinite(range)) {
            const double tolerance = std::max(
                ground_range_abs_tolerance_,
                ground_range_rel_tolerance_ * ground_range);
            if (std::isfinite(ground_range) &&
                std::fabs(range - ground_range) <= tolerance) {
                hit_type = multi_uav_strike::LidarMeasurement::HIT_GROUND;
            } else {
                hit_type = multi_uav_strike::LidarMeasurement::HIT_BUILDING;
            }
        }

        sensor_msgs::Range msg;
        msg.header.stamp = now;
        msg.header.frame_id = viz_frame_;
        msg.radiation_type = sensor_msgs::Range::INFRARED;
        msg.field_of_view = 0.01;
        msg.min_range = lidar_min_range_;
        msg.max_range = lidar_max_range_;
        msg.range = range;
        lidar_range_pub_.publish(msg);

        multi_uav_strike::LidarMeasurement detailed;
        detailed.header = msg.header;
        detailed.range = range;
        detailed.min_range = lidar_min_range_;
        detailed.max_range = lidar_max_range_;
        detailed.hit_type = hit_type;
        if (std::isfinite(range)) {
            detailed.hit_point.x = p.x + ray_dir.x() * range;
            detailed.hit_point.y = p.y + ray_dir.y() * range;
            detailed.hit_point.z = p.z + ray_dir.z() * range;
        } else {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            detailed.hit_point.x = nan;
            detailed.hit_point.y = nan;
            detailed.hit_point.z = nan;
        }
        lidar_measurement_pub_.publish(detailed);

        const double draw_range = std::isfinite(range) ? range : lidar_max_range_;
        visualization_msgs::Marker beam;
        beam.header = msg.header;
        beam.ns = "forward_lidar";
        beam.id = 0;
        beam.type = visualization_msgs::Marker::LINE_LIST;
        beam.action = visualization_msgs::Marker::ADD;
        beam.scale.x = 0.25;
        beam.color.r = hit_type == multi_uav_strike::LidarMeasurement::HIT_BUILDING ? 1.0f : 0.1f;
        beam.color.g = hit_type == multi_uav_strike::LidarMeasurement::HIT_NONE ? 0.8f : 0.1f;
        beam.color.b = hit_type == multi_uav_strike::LidarMeasurement::HIT_GROUND ? 1.0f : 0.1f;
        beam.color.a = 0.9f;
        geometry_msgs::Point end;
        end.x = p.x + ray_dir.x() * draw_range;
        end.y = p.y + ray_dir.y() * draw_range;
        end.z = p.z + ray_dir.z() * draw_range;
        beam.points.push_back(p);
        beam.points.push_back(end);
        beam.lifetime = ros::Duration(2.0 / lidar_frequency_);
        lidar_beam_pub_.publish(beam);
        publishBuildingMarker();
    }

    size_t countVisible() const {
        size_t n = 0;
        for (const auto& p : projections_) if (p.visible) ++n;
        return n;
    }

    // ====================================================================
    // 核心算法: 计算 desired_gimbal_angle
    // ====================================================================
    //
    // 关键改动: 先在"当前云台角度"下做 FOV 判定
    //   - 目标在视野中 → desired_pitch = atan2(高度,水平距) 指向目标
    //   - 目标不在视野中 → desired_pitch = default_pitch_rad_ (回退)
    // 偏航固定 0, 不参与计算(desired_yaw 一直为 0)
    //
    // 计算目标相对无人机的"几何"俯仰角(忽略云台当前状态)
    // 新约定: 0=水平, -π/2=垂直下视
    //   uav 高于 target 时, target 在视野下方, desired_pitch 为负
    double computePitchAndYawToTarget(double target_x, double target_y) {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_alt = current_uav_pose_.pose.position.z;  // NWU: z=高度

        if (image_noise_std_dev_ > 0.0) {
            target_x += noise_dist_(rng_);
            target_y += noise_dist_(rng_);
        }

        double dx = target_x - uav_x;
        double dy = target_y - uav_y;
        double horiz = std::sqrt(dx*dx + dy*dy);
        double drop = uav_alt - 0.0;  // 目标假设在地面, 高度 0

        desired_yaw = atan2(dy, dx);
        desired_yaw = atan2(sin(desired_yaw), cos(desired_yaw));

        // atan2(drop, horiz) ∈ (0, π/2)  (目标在下方)
        // 新约定"看向下"为负, 所以取负号
        double pitch = std::atan2(drop, horiz);
        return clampv(pitch, 0.0, M_PI/2.0);
    }

    // ====================================================================
    // 单次投影: 对所有目标算出相机-目标几何, 缓存到 projections_, 并选主目标
    // ====================================================================
    //
    // 每 tick 在 controlLoopCallback 开头调一次, 后续全部读缓存:
    //   - projections_[i].visible/u_px/v_px/distance
    //   - cached_q_cam_world_   (camera/pose 发布)
    //   - primary_idx_          (FOV 内距 UAV 最近的目标; -1=无)
    //
    // 相机世界姿态 = uav_quat(NWU) * R(0, gimbal_pitch, 0)
    // 相机系约定: x=前, y=右, z=下
    // 目标在前方 (t_cam.z > 0) 且投影在矩形内 → visible
    void recomputeAllProjections() {
        cached_proj_valid_ = is_target_received_ && is_uav_pose_received_;
        projections_.clear();
        primary_idx_ = -1;

        if (!cached_proj_valid_) return;

        // 相机世界姿态(所有目标共用)
        cached_q_cam_world_ = currentCameraWorldQuat();
        Eigen::Matrix3d R_world_cam = cached_q_cam_world_.toRotationMatrix();

        double best_dist = 1e18;
        projections_.reserve(targets_.size());
        for (const auto& t : targets_) {
            TargetProj pr;
            pr.id    = t.id;
            pr.label = t.label;
            pr.pos   = t.position;

            Eigen::Vector3d world_diff(
                t.position.x - current_uav_pose_.pose.position.x,
                t.position.y - current_uav_pose_.pose.position.y,
                t.position.z - current_uav_pose_.pose.position.z);
            Eigen::Vector3d t_cam = R_world_cam.transpose() * world_diff;

            if (t_cam.z() > 0.05) {
                double u_ndc = t_cam.y() / t_cam.z();
                double v_ndc = -t_cam.x() / t_cam.z();
                pr.u_px = u_ndc * focal_x_ + image_width_  / 2.0;
                pr.v_px = v_ndc * focal_y_ + image_height_ / 2.0;
                bool u_ok = (pr.u_px >= 0.0 && pr.u_px < image_width_);
                bool v_ok = (pr.v_px >= 0.0 && pr.v_px < image_height_);
                pr.visible = u_ok && v_ok;
                pr.distance = t_cam.norm();
            }

            if (pr.visible && pr.distance < best_dist) {
                best_dist = pr.distance;
                primary_idx_ = static_cast<int>(projections_.size());
            }
            projections_.push_back(pr);
        }
    }

    // 计算相机世界姿态(纯函数, 不改状态)
    // 注意: 不写缓存, 调用方要么自己存、要么用 recomputeCameraTargetProjection() 缓存版
    //
    // === 2026-07-16: 稳像世界姿态 (Plan B) ===
    // 老版本 world_cam = uav_quat * setRPY(0, pitch, 0) 把 UAV 的 roll/pitch 全部
    // 叠进了相机姿态,导致任何机体抖动都被斜距放大成目标估计横漂。
    // 新版本: 仅用 (uav_yaw, gimbal_pitch) 直接构造 R_world_cam,
    //   光轴在 NWU 世界系 = (cos p·cos y, cos p·sin y, -sin p)
    //   图像上方向 = world_up 在光轴平面的投影
    //   图像右方向 = 光轴 × 上方向
    // 完全不依赖 UAV roll/机体 pitch,UAV 一滚转 R_world_cam 不变。
    Eigen::Quaterniond currentCameraWorldQuat() const {
        const double cy = std::cos(current_uav_yaw_);
        const double sy = std::sin(current_uav_yaw_);
        const double cp = std::cos(current_gimbal_pitch_);
        const double sp = std::sin(current_gimbal_pitch_);

        // 光轴 (cam +z) → 世界方向
        Eigen::Vector3d opt(cp * cy, cp * sy, -sp);
        // 图像上 (cam +x) → world_up 在光轴平面的投影
        Eigen::Vector3d world_up(0.0, 0.0, 1.0);
        Eigen::Vector3d up_proj = world_up - opt * world_up.dot(opt);
        if (up_proj.norm() < 1e-3) {
            // 极限: 相机光轴正上/正下 → 用机头方向作 "上"
            up_proj = Eigen::Vector3d(cy, sy, 0.0);
        }
        up_proj.normalize();
        // 图像右 (cam +y) → 光轴 × 上方向
        Eigen::Vector3d right = opt.cross(up_proj);
        if (right.norm() < 1e-6) {
            right = Eigen::Vector3d::UnitY();
        } else {
            right.normalize();
        }
        // 重新正交化 up
        Eigen::Vector3d up_orth = right.cross(opt);

        Eigen::Matrix3d R;
        R.col(0) = up_orth;
        R.col(1) = right;
        R.col(2) = opt;
        return Eigen::Quaterniond(R).normalized();
    }

    // ====================================================================
    // 控制: P-control (只动 pitch)
    // ====================================================================
    void updateGimbalAngles(double desired_pitch) {
        double dt = 1.0 / loop_freq_;
        double pitch_error = desired_pitch - current_gimbal_pitch_;
        double rate = gimbal_p_gain_ * pitch_error;
        rate = clampv(rate, -max_pitch_rate_, max_pitch_rate_);
        current_gimbal_pitch_ += rate * dt;
        current_gimbal_pitch_ = clampv(current_gimbal_pitch_, 0.0, M_PI/2.0);
    }

    // ====================================================================
    // 发布(全部读 cached_*, 不再做几何计算)
    // ====================================================================

    // target_los_angle: 目标 LOS (Point.x=yaw, .y=pitch, .z=tracking_accuracy)
    // 维持原语义:发的是 desired (目标的几何 LOS),不是当前相机
    // 发布NED下的角度
    void publishLOSAngle(double desired_pitch, double desired_yaw) {
        geometry_msgs::Point msg;
        msg.x = -desired_yaw;  // 偏航
        msg.y = -desired_pitch;
        // tracking_accuracy: 当前 pitch 与 desired 的接近度(指数衰减)
        double err = std::fabs(desired_pitch - current_gimbal_pitch_);
        double acc = std::exp(-err);
        msg.z = std::isnan(acc) ? 0.5 : acc;
        los_angle_pub_.publish(msg);
    }

    // gimbal_pose: 当前云台在 NWU 下的姿态 (RViz 用)
    void publishGimbalPose() {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = viz_frame_;
        msg.pose.position = current_uav_pose_.pose.position;

        tf::Quaternion q;
        // 新约定: pitch 直接传给 setRPY (NWU 下负 pitch = 低头)
        q.setRPY(0.0, current_gimbal_pitch_, 0.0);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        gimbal_pose_pub_.publish(msg);
    }

    // camera/pose: 相机世界姿态 (稳像, 与 forward projection 共用 currentCameraWorldQuat)
    void publishCameraPose() {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = viz_frame_;
        msg.pose.position = current_uav_pose_.pose.position;

        // === 2026-07-16: 改为稳像世界姿态 ===
        // 与 forward projection 完全一致,UAV 一滚转 camera/pose 不抖。
        Eigen::Quaterniond q = currentCameraWorldQuat();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        camera_pose_pub_.publish(msg);
    }

    // 从一个 TargetProj 构造一条 YoloDetection(仅在 visible 时填 bbox/conf/img)
    multi_uav_strike::YoloDetection buildDetection(const TargetProj& pr, uint32_t frame_seq,
                                                   uint64_t stamp_us) {
        multi_uav_strike::YoloDetection out;
        out.frame_seq = frame_seq;
        out.stamp_us = stamp_us;
        out.label = pr.label;
        out.confidence = 0.0f;
        out.x_min = out.y_min = out.x_max = out.y_max = 0.0f;
        out.is_in_fov = false;
        if (pr.visible) {
            // 简化的 bbox: size ∝ 1/distance
            double size_px = std::min(image_width_, image_height_) * 0.05 +
                             1000.0 / (pr.distance + 1.0);
            size_px = std::min(size_px, std::min(image_width_, image_height_) * 0.4);
            double half = size_px / 2.0;
            out.x_min = (float)std::max(0.0, (pr.u_px - half) / image_width_);
            out.y_min = (float)std::max(0.0, (pr.v_px - half) / image_height_);
            out.x_max = (float)std::min(1.0, (pr.u_px + half) / image_width_);
            out.y_max = (float)std::min(1.0, (pr.v_px + half) / image_height_);
            out.confidence = (float)std::max(0.4, 0.95 - 0.001 * pr.distance);
            out.is_in_fov = true;
            // 可见帧才携带模拟 JPEG;真实 gimbal 接入后此分支替换为相机抓拍帧
            out.img_data = fake_jpeg_;
        }
        return out;
    }

    // 发布:YoloDetections(全部可见目标,供 target_estimator)
    //       + 单条 YoloDetection(主目标,兼容 comm_node)
    //       + Bool(是否有任一目标在 FOV)
    void publishYoloDetections() {
        // === 2026-07-16: yolo_pub_freq 节流 (与云台 loop_freq 解耦) ===
        //   frame_seq_ 每 tick 自增,作为 loop tick 计数器。
        //   period = loop_freq / yolo_pub_freq:loop=100Hz / yolo=10Hz → 每 10 tick 发一次。
        //   仅 YoloDetection(单条/批量) 被节流,in_fov_pub_ 保持原频率
        //   (gimbal FOV 状态对 comm_node 不是 detection,不需要节流)。
        const uint32_t period = std::max(1u,
            static_cast<uint32_t>(std::round(loop_freq_ / yolo_pub_freq_)));
        const bool emit_yolo = (frame_seq_ % period) == 0;
        uint32_t seq = ++frame_seq_;
        uint64_t stamp_us = ros::Time::now().toNSec() / 1000;

        bool any_visible = false;
        if (emit_yolo) {
            multi_uav_strike::YoloDetections batch;
            for (const auto& pr : projections_) {
                if (!pr.visible) continue;   // 只把在 FOV 内的目标放入批量(与原单目标语义一致)
                batch.detections.push_back(buildDetection(pr, seq, stamp_us));
                any_visible = true;
            }
            yolo_results_pub_.publish(batch);

            // 单条 = 主目标(无主目标则空帧 is_in_fov=false)
            multi_uav_strike::YoloDetection primary;
            if (primary_idx_ >= 0) {
                primary = buildDetection(projections_[primary_idx_], seq, stamp_us);
            } else {
                primary.frame_seq = seq;
                primary.stamp_us = stamp_us;
                primary.label = "";
                primary.confidence = 0.0f;
                primary.x_min = primary.y_min = primary.x_max = primary.y_max = 0.0f;
                primary.is_in_fov = false;
            }
            yolo_pub_.publish(primary);
        } else {
            // 即使本 tick 不发 yolo,也更新 any_visible(in_fov 仍反映当前 FOV 状态)
            for (const auto& pr : projections_) if (pr.visible) { any_visible = true; break; }
        }

        std_msgs::Bool in_fov_flag;
        in_fov_flag.data = any_visible;
        in_fov_pub_.publish(in_fov_flag);
    }

    // 从磁盘加载固定 JPEG 字节流;加载失败时 fake_jpeg_ 留空(下游 img_data 也空)
    void loadFakeJpeg(const std::string& path) {
        std::ifstream ifs(path, std::ios::binary | std::ios::ate);
        if (!ifs) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' open failed, img_data will be empty",
                     path.c_str());
            return;
        }
        const std::streamsize sz = ifs.tellg();
        if (sz <= 0) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' is empty", path.c_str());
            return;
        }
        ifs.seekg(0, std::ios::beg);
        fake_jpeg_.resize(static_cast<size_t>(sz));
        if (!ifs.read(reinterpret_cast<char*>(fake_jpeg_.data()), sz)) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' read failed", path.c_str());
            fake_jpeg_.clear();
            return;
        }
        ROS_INFO("[GimbalSim] loaded fake JPEG: path=%s size=%zu bytes",
                 path.c_str(), fake_jpeg_.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_simulator_node");
    GimbalSimulator gimbal_sim;
    ros::spin();
    return 0;
}
