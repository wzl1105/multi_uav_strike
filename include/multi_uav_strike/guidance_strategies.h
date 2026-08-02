#ifndef GUIDANCE_STRATEGIES_H
#define GUIDANCE_STRATEGIES_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <memory>
// === 2026-07-31: 比例导引(PN)落地前置依赖, 老 strategy 暂未引用, 仅占位
//   等 PN InterceptGuidance::computeCommand 替换老实现时一并接入 ===
#include "multi_uav_strike/low_pass_filter_2p.h"
#include "multi_uav_strike/velocity_vector_limiter.h"

namespace multi_uav_strike {

// Enum for strategy selection
enum class GuidanceStrategyType {
    INTERCEPT = 0,
    MINSNAP = 1,
    LOS = 2
};

// Velocity command output (for Intercept strategy)
// === 2026-07-31: 与 PN 头(guidance_strategies_PN.h)字段对齐 ===
//   - intercept_point → intercept_point_nwu (语义不变, 改名避免与点变量重名)
//   - 新增 5 个字段(desired_yaw/los_*)留给 PN 可视化调试, 老代码默认 0
struct VelocityCommand {
    // InterceptGuidance 的控制计算与输出统一使用 NED；调用方只在发布 MAVROS 前转 ENU。
    Eigen::Vector3d velocity;
    Eigen::Vector3d intercept_point_nwu;           // 拦截点位置 (NWU)
    double desired_yaw       = 0.0;                // 期望机头指向 (NED)
    double los_yaw           = 0.0;                // 当前 LOS 方位角 (NED)
    double los_pitch         = 0.0;                // 当前 LOS 俯仰角 (NED)
    double los_yaw_rate      = 0.0;                 // LOS 方位角速度 (滤波后)
    double los_pitch_rate    = 0.0;                 // LOS 俯仰角速度 (滤波后)
};

// Attitude + Thrust command output (for MinSnap and LOS strategies)
struct AttitudeThrustCommand {
    Eigen::Quaterniond attitude;
    double thrust;
    double yaw_rate;  // 偏航角速率（用于LOS控制）

    AttitudeThrustCommand() : thrust(0.0), yaw_rate(0.0) {}
};

// Base class for all guidance strategies
class GuidanceStrategy {
public:
    virtual ~GuidanceStrategy() = default;

    // Get strategy name
    const std::string& getName() const { return name_; }

    // Get strategy type
    virtual GuidanceStrategyType getType() const = 0;

    // 允许 mission_manager 运行时下发拦截速度(目前仅 Intercept 关心,默认 no-op)
    virtual void setUavSpeed(double /*speed*/) {}

protected:
    GuidanceStrategy(ros::NodeHandle& nh, const std::string& name)
        : nh_(nh), name_(name) {}

    ros::NodeHandle& nh_;
    std::string name_;
};

// Intercept Guidance: LOS-rate based proportional navigation with lead angle constraint
// === 2026-07-31: 改为比例导引(PN), 原"指向目标匀速"算法删除 ===
//   来源: guidance_strategies_PN.h (其他老师贡献)
//   几何: 机头指向 LOS, 横向加速度正比于 LOS 角速度 (nav_ratio * los_rate * distance)
//   约束: 前置角 ≤ 30°, 三维速度联合限幅
//   多机扩展点: setDesiredLosYawRate 由 LosSpacingController 注入
class InterceptGuidance : public GuidanceStrategy {
public:
    InterceptGuidance(ros::NodeHandle& nh);

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::INTERCEPT; }

    // mission_manager 运行时下发拦截速度, 同时映射到水平速度上限
    void setUavSpeed(double speed) override {
        radial_speed_         = speed;
        max_horizontal_speed_ = speed;
    }

    // 多机分散层 (LosSpacingController) 注入期望 LOS 方位角速度 (rad/s)
    void setDesiredLosYawRate(double rate) { desired_los_yaw_rate_ = rate; }

    VelocityCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist);

    // 重置 LOS 角滤波器状态 (目标切换 / 解锁 / 跨长时间断流时调用)
    void resetLosFilter();

private:
    double radial_speed_;             // 协同到达给出的 LOS 径向前向分量 (m/s)
    double nav_ratio_;                // 导航比 N (默认 1.5)
    double max_horizontal_speed_;     // STRIKE 最大水平速度 (默认 20 m/s)
    double max_vertical_speed_;       // STRIKE 最大升降速度 (默认 3 m/s)
    double max_lead_angle_rad_;       // 最大前置角约束 (默认 30°)
    double los_filter_cutoff_hz_;     // LOS 角低通截止频率 (默认 2 Hz)
    double desired_los_yaw_rate_;     // 多机分散层给出的期望 LOS 方位角速度 (rad/s)

    // LOS 角状态 (上一拍滤波值)
    double prev_los_yaw_filtered_;
    double prev_los_pitch_filtered_;
    bool   los_filter_initialized_;

    LowPassFilter2p los_yaw_filter_;
    LowPassFilter2p los_pitch_filter_;

    ros::Time last_update_time_;
    double    dt_;                    // 控制周期 (s)
    double    los_yaw_unwrapped_prev_;  // 上一拍解包裹后的原始 LOS yaw
    bool      los_yaw_unwrap_initialized_;

    // 角度 wrap 到 [-π, π]
    static double wrap_pi(double angle) {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

// Min-Snap Trajectory Guidance: Generates minimum-snap trajectories
class MinSnapGuidance : public GuidanceStrategy {
public:
    MinSnapGuidance(ros::NodeHandle& nh);
    ~MinSnapGuidance();

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::MINSNAP; }

    AttitudeThrustCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist);

    void updateTrajectory();

private:
    // Trajectory generation parameters
    double v_max_;
    double a_max_;
    int polynomial_order_;
    double sampling_interval_;
    double trajectory_lookahead_;

    // Tracking controller params
    double pos_p_gain_;
    double vel_p_gain_;
    double thrust_weight_;

    // Current trajectory state
    double trajectory_start_time_;
    bool trajectory_valid_;

    // Helper: compute attitude + thrust from acceleration
    AttitudeThrustCommand computeAttitudeThrust(
        const Eigen::Vector3d& desired_acceleration,
        const Eigen::Vector3d& position_error,
        const Eigen::Vector3d& velocity_error);

    // Generate trajectory to target
    bool generateTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& goal_pos,
        const Eigen::Vector3d& goal_vel);

    // Sample current trajectory
    bool sampleTrajectory(double time_since_start,
                          Eigen::Vector3d& pos,
                          Eigen::Vector3d& vel,
                          Eigen::Vector3d& acc);
};

// LOS Guidance: Uses line-of-sight angle error for guidance
class LosGuidance : public GuidanceStrategy {
public:
    LosGuidance(ros::NodeHandle& nh);

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::LOS; }

    AttitudeThrustCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist,
        const geometry_msgs::Point& los_angle);

private:
    // LOS control params
    double los_kp_yaw_;
    double los_kp_pitch_;
    double los_max_rate_;
    double thrust_weight_;

    // Integrator state for yaw/pitch
    double integrated_yaw_error_;
    double integrated_pitch_error_;

    // Helper: compute attitude + thrust
    AttitudeThrustCommand computeAttitudeThrust(
        double desired_roll,
        double desired_pitch,
        double desired_yaw);
};

// Factory function to create strategies
std::unique_ptr<GuidanceStrategy> createGuidanceStrategy(
    GuidanceStrategyType type,
    ros::NodeHandle& nh);

} // namespace multi_uav_strike

#endif // GUIDANCE_STRATEGIES_H