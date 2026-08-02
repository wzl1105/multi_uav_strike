#include "multi_uav_strike/guidance_strategies.h"
#include <cmath>
#include <algorithm>

namespace multi_uav_strike {

//==============================================================================
// InterceptGuidance Implementation
//==============================================================================

InterceptGuidance::InterceptGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "Intercept"),
      radial_speed_(10.0),
      nav_ratio_(1.5),
      max_horizontal_speed_(20.0),
      max_vertical_speed_(3.0),
      max_lead_angle_rad_(30.0 * M_PI / 180.0),
      los_filter_cutoff_hz_(2.0),
      desired_los_yaw_rate_(0.0),
      prev_los_yaw_filtered_(0.0),
      prev_los_pitch_filtered_(0.0),
      los_filter_initialized_(false),
      los_yaw_unwrapped_prev_(0.0),
      los_yaw_unwrap_initialized_(false),
      dt_(0.02) {  // 50Hz default
    
    nh.param<double>("strike_max_speed", radial_speed_, 20.0);
    nh.param<double>("nav_ratio", nav_ratio_, 1.5);
    nh.param<double>("strike_max_speed", max_horizontal_speed_, 20.0);
    nh.param<double>("strike_max_vertical_speed", max_vertical_speed_, 3.0);
    nh.param<double>("max_lead_angle_deg", max_lead_angle_rad_, 30.0);
    max_lead_angle_rad_ = max_lead_angle_rad_ * M_PI / 180.0;  // deg → rad
    nh.param<double>("los_filter_cutoff_hz", los_filter_cutoff_hz_, 2.0);
    
    // 初始化 LOS 角低通滤波器
    los_yaw_filter_.configure(1.0 / dt_, los_filter_cutoff_hz_);
    los_pitch_filter_.configure(1.0 / dt_, los_filter_cutoff_hz_);
    
    last_update_time_ = ros::Time::now();
    
    ROS_INFO("[Intercept] N=%.2f, max_lead=%.1f deg, los_filter=%.1f Hz, speed=%.2f m/s",
             nav_ratio_, max_lead_angle_rad_ * 180.0 / M_PI,
             los_filter_cutoff_hz_, radial_speed_);
}

void InterceptGuidance::resetLosFilter() {
    los_yaw_filter_.reset();
    los_pitch_filter_.reset();
    los_filter_initialized_ = false;
    los_yaw_unwrap_initialized_ = false;
    ROS_INFO("[Intercept] LOS filter reset");
}

VelocityCommand InterceptGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist) {

    VelocityCommand cmd;

    // ===== 1. 计算 LOS 角（NED 地理系）=====
    // 节点内部约定 NWU（uavPoseCallback 已转换），先转回 NED 再算 LOS：
    //   NED N = NWU N, NED E = -NWU W, NED D = -NWU U
    // los_yaw:   左负右正, 0=北, +π/2=东
    // los_pitch: 下负上正, 0=水平, +π/2=目标在上方

    double dx = (target_pose.pose.position.x - uav_pose.pose.position.x);   // NED N 差
    double dy = -(target_pose.pose.position.y - uav_pose.pose.position.y);  // NED E = -NWU W
    double dz = -(target_pose.pose.position.z - uav_pose.pose.position.z);  // NED D = -NWU U

    double horiz = sqrt(dx*dx + dy*dy);
    double los_yaw = atan2(dy, dx);              // 左负右正
    double los_pitch = atan2(-dz, horiz);        // 下负上正（-dz: NED D+ → 上方为正）
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    // ===== 2. LOS 角低通滤波（方案 A：先滤波角度，再差分）=====
    ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    if (dt <= 0.0 || dt > 1.0) dt = dt_;  // 防御
    last_update_time_ = now;
    
    // 先把 yaw 解包裹成连续角，再低通。否则 +179°→-179° 会被普通线性
    // 滤波错误地拉向 0°，制造一个并不存在的大 LOS 角速度。
    double los_yaw_unwrapped = los_yaw;
    if (los_yaw_unwrap_initialized_) {
        los_yaw_unwrapped = los_yaw_unwrapped_prev_ +
                            wrap_pi(los_yaw - wrap_pi(los_yaw_unwrapped_prev_));
    }
    los_yaw_unwrapped_prev_ = los_yaw_unwrapped;
    los_yaw_unwrap_initialized_ = true;

    double los_yaw_filtered = los_yaw_filter_.apply(los_yaw_unwrapped);
    double los_pitch_filtered = los_pitch_filter_.apply(los_pitch);
    
    // ===== 3. LOS 角速度（差分）=====
    double los_yaw_rate = 0.0;
    double los_pitch_rate = 0.0;
    
    if (los_filter_initialized_) {
        double dyaw = wrap_pi(los_yaw_filtered - prev_los_yaw_filtered_);
        double dpitch = los_pitch_filtered - prev_los_pitch_filtered_;
        
        los_yaw_rate = dyaw / dt;
        los_pitch_rate = dpitch / dt;
    }
    
    prev_los_yaw_filtered_ = los_yaw_filtered;
    prev_los_pitch_filtered_ = los_pitch_filtered;
    los_filter_initialized_ = true;
    
    // 保存到 cmd（用于可视化和调试）
    cmd.los_yaw = los_yaw;
    cmd.los_pitch = los_pitch;
    cmd.los_yaw_rate = los_yaw_rate;
    cmd.los_pitch_rate = los_pitch_rate;
    
    // ===== 4. P 控制：让 LOS 角速度 → 0 =====
    // 期望：UAV 速度方向应该消除 LOS 旋转
    // los_yaw_rate > 0（LOS 向右转）→ 飞机向右飞（vy > 0）
    // los_pitch_rate > 0（LOS 向上转）→ 飞机向上飞（vz < 0，NED）
    
    // LOS 方向单位向量（NED）
    double cos_yaw = cos(los_yaw);
    double sin_yaw = sin(los_yaw);
    double sin_pitch = sin(los_pitch);
    
    // 水平面：LOS 水平方向
    // 水平径向分量按“水平 LOS”定义，单位长度必须为 1；若再乘 cos(pitch)，
    // 则实际水平径向投影只有 v_fwd*cos(pitch)，会破坏按水平距离做的同步。
    double los_xy_x = cos_yaw;  // NED North
    double los_xy_y = sin_yaw;  // NED East
    
    // 垂直于 LOS 水平方向（左旋 90°，向右）
    double los_xy_perp_x = -los_xy_y;  // 向右为正
    double los_xy_perp_y = los_xy_x;
    
    // 横向速度：消除 yaw_rate，向右为正
    // los_yaw_rate > 0 → 需要向右飞 → v_lateral > 0
    const double los_yaw_rate_error = los_yaw_rate - desired_los_yaw_rate_;
    double v_lateral = nav_ratio_ * los_yaw_rate_error * horiz;
    
    // 垂直速度：消除 pitch_rate，向上为负（NED）
    // los_pitch_rate > 0 → LOS 向上转 → 飞机向上飞 → vz < 0
    double v_vertical = -nav_ratio_ * los_pitch_rate * distance;
    
    // ===== 5. 前向/横向速度分配 =====
    // 协同到达模式下：radial_speed_ 是同步算出的 LOS 径向前向分量，直接使用。
    // 横向修正可以使合速度大于该前向分量；否则整体归一化会破坏 d/tmax 约束。
    // 横向约束：|v_lateral| ≤ 0.4 · radial_speed_
    double max_lateral = 0.4 * radial_speed_;
    v_lateral = std::max(-max_lateral, std::min(max_lateral, v_lateral));
    double v_forward = radial_speed_;
    
    // ===== 6. 合成速度矢量 =====
    // 水平面：前进 + 横向
    double vx = los_xy_x * v_forward + los_xy_perp_x * v_lateral;
    double vy = los_xy_y * v_forward + los_xy_perp_y * v_lateral;
    
    // 垂直面：前进分量 + 垂直修正
    // los_pitch 约定“下负上正”，而 NED z 是“下正”，所以沿 LOS 的 NED
    // 垂直分量必须是 -v_forward*sin(los_pitch)。
    double vz = -v_forward * sin_pitch + v_vertical;
    
    // ===== 7. 前置角约束（只约束水平面）=====
    // 计算当前速度方向与 LOS 水平方向的夹角
    double vel_yaw = atan2(vy, vx);
    double lead_angle = wrap_pi(vel_yaw - los_yaw);
    
    if (fabs(lead_angle) > max_lead_angle_rad_) {
        // 把速度方向往 LOS 方向拉回来
        double clamped_yaw = los_yaw + 
            (lead_angle > 0 ? max_lead_angle_rad_ : -max_lead_angle_rad_);
        double speed = sqrt(vx*vx + vy*vy);
        vx = speed * cos(clamped_yaw);
        vy = speed * sin(clamped_yaw);
        
        ROS_DEBUG_THROTTLE(0.5, "[Intercept] Lead angle clamped: %.1f → %.1f deg",
                          lead_angle * 180.0 / M_PI,
                          max_lead_angle_rad_ * 180.0 / M_PI);
    }

    // 水平/垂直上限共同约束整个三维速度矢量，保持合成方向不变。
    // 不能只截断 vz，否则会使水平通道提前到达、垂直通道来不及下降。
    limitVelocityVector(vx, vy, vz,
                        max_horizontal_speed_, max_vertical_speed_);
    
    // ===== 8. 机头指向 =====
    // 期望 yaw = los_yaw（机头指向目标）
    cmd.desired_yaw = los_yaw;
    
    // ===== 9. 输出 =====
    cmd.velocity.x() = vx;   // NED North
    cmd.velocity.y() = vy;   // NED East
    cmd.velocity.z() = vz;   // NED Down
    cmd.intercept_point_nwu = Eigen::Vector3d(
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z);
    
    ROS_DEBUG_THROTTLE(1.0,
        "[Intercept] dist=%.1f, los=(%.1f, %.1f) deg, rate=(%.1f, %.1f) deg/s, "
        "vel=(%.2f, %.2f, %.2f), lead=%.1f deg",
        distance,
        los_yaw * 180.0 / M_PI, los_pitch * 180.0 / M_PI,
        los_yaw_rate * 180.0 / M_PI, los_pitch_rate * 180.0 / M_PI,
        vx, vy, vz,
        lead_angle * 180.0 / M_PI);
    
    return cmd;
}

//==============================================================================
// MinSnapGuidance Implementation
//==============================================================================

MinSnapGuidance::MinSnapGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "MinSnap"),
      trajectory_valid_(false),
      trajectory_start_time_(0.0) {

    // Trajectory params
    nh.param<double>("v_max", v_max_, 5.0);
    nh.param<double>("a_max", a_max_, 10.0);
    nh.param<int>("polynomial_order", polynomial_order_, 10);
    nh.param<double>("sampling_interval", sampling_interval_, 0.01);
    nh.param<double>("trajectory_lookahead", trajectory_lookahead_, 0.5);

    // Tracking controller params
    nh.param<double>("pos_p_gain", pos_p_gain_, 1.0);
    nh.param<double>("vel_p_gain", vel_p_gain_, 0.5);
    nh.param<double>("thrust_weight", thrust_weight_, 0.5);

    ROS_INFO("[MinSnap] v_max=%.2f, a_max=%.2f, pos_p=%.2f, vel_p=%.2f, thrust_w=%.2f",
             v_max_, a_max_, pos_p_gain_, vel_p_gain_, thrust_weight_);
}

MinSnapGuidance::~MinSnapGuidance() {}

bool MinSnapGuidance::generateTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& goal_pos,
    const Eigen::Vector3d& goal_vel) {

    // Use mav_trajectory_generation library
    // This is a simplified version - full implementation would use
    // PolynomialOptimization and Vertex classes

    // For now, we'll store the trajectory parameters and use simple sampling
    // A full implementation would integrate with mav_trajectory_generation

    trajectory_valid_ = false;
    return false;  // Placeholder - full trajectory generation requires more setup
}

bool MinSnapGuidance::sampleTrajectory(double time_since_start,
                                       Eigen::Vector3d& pos,
                                       Eigen::Vector3d& vel,
                                       Eigen::Vector3d& acc) {
    if (!trajectory_valid_) {
        return false;
    }
    // Placeholder - full implementation would sample from trajectory
    return false;
}

AttitudeThrustCommand MinSnapGuidance::computeAttitudeThrust(
    const Eigen::Vector3d& desired_acceleration,
    const Eigen::Vector3d& position_error,
    const Eigen::Vector3d& velocity_error) {

    AttitudeThrustCommand cmd;

    // Total acceleration = desired + P*pos_error + D*vel_error
    Eigen::Vector3d total_acc = desired_acceleration
                                 + pos_p_gain_ * position_error
                                 + vel_p_gain_ * velocity_error;

    // Add gravity compensation (in ENU frame, z is up)
    total_acc.z() += 9.81;

    // Compute attitude from acceleration direction
    // z_B = total_acc / |total_acc| (body z-axis points in acceleration direction)
    Eigen::Vector3d z_B = total_acc.normalized();

    // Assume yaw is maintained (use current yaw or default to 0)
    double yaw = 0.0;

    // Compute x_C (NED-derived, in world frame)
    Eigen::Vector3d x_C(cos(yaw), sin(yaw), 0);

    // Compute y_B = z_B x x_C
    Eigen::Vector3d y_B = z_B.cross(x_C).normalized();

    // Compute x_B = y_B x z_B
    Eigen::Vector3d x_B = y_B.cross(z_B).normalized();

    // Build rotation matrix [x_B, y_B, z_B]
    Eigen::Matrix3d R;
    R.col(0) = x_B;
    R.col(1) = y_B;
    R.col(2) = z_B;

    cmd.attitude = Eigen::Quaterniond(R);
    cmd.thrust = total_acc.norm() * thrust_weight_;

    return cmd;
}

AttitudeThrustCommand MinSnapGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist) {

    AttitudeThrustCommand cmd;

    // Extract UAV state
    Eigen::Vector3d uav_pos(uav_pose.pose.position.x,
                             uav_pose.pose.position.y,
                             uav_pose.pose.position.z);

    // Extract target state
    Eigen::Vector3d target_pos(target_pose.pose.position.x,
                                 target_pose.pose.position.y,
                                 target_pose.pose.position.z);
    Eigen::Vector3d target_vel(target_twist.twist.linear.x,
                                 target_twist.twist.linear.y,
                                 target_twist.twist.linear.z);

    // Position error (UAV should be near target for MinSnap)
    Eigen::Vector3d pos_error = target_pos - uav_pos;
    Eigen::Vector3d vel_error = -target_vel;  // Assume UAV velocity is small

    // For MinSnap, we want to follow the target
    // Use target velocity as desired velocity, target acceleration as desired
    Eigen::Vector3d desired_vel = target_vel;
    Eigen::Vector3d desired_acc = Eigen::Vector3d::Zero();  // Assume constant velocity

    // Simple PD control to target
    cmd = computeAttitudeThrust(desired_acc, pos_error, vel_error);

    ROS_DEBUG_THROTTLE(1.0, "[MinSnap] pos_error=(%.2f, %.2f, %.2f), thrust=%.2f",
                       pos_error.x(), pos_error.y(), pos_error.z(), cmd.thrust);

    return cmd;
}

void MinSnapGuidance::updateTrajectory() {
    // Placeholder for trajectory regeneration logic
}

//==============================================================================
// LosGuidance Implementation
//==============================================================================

LosGuidance::LosGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "LOS"),
      integrated_yaw_error_(0.0),
      integrated_pitch_error_(0.0) {

    nh.param<double>("los_kp_yaw", los_kp_yaw_, 1.5);
    nh.param<double>("los_kp_pitch", los_kp_pitch_, 1.5);
    nh.param<double>("los_max_rate", los_max_rate_, 1.2);
    nh.param<double>("thrust_weight", thrust_weight_, 0.5);

    ROS_INFO("[LOS] kp_yaw=%.2f, kp_pitch=%.2f, max_rate=%.2f, thrust_w=%.2f",
             los_kp_yaw_, los_kp_pitch_, los_max_rate_, thrust_weight_);
}

AttitudeThrustCommand LosGuidance::computeAttitudeThrust(
    double desired_roll,
    double desired_pitch,
    double desired_yaw) {

    AttitudeThrustCommand cmd;

    // Convert roll/pitch/yaw to quaternion
    // Using Euler angles to quaternion conversion (ZYX order: yaw, pitch, roll)
    double cy = cos(desired_yaw * 0.5);
    double sy = sin(desired_yaw * 0.5);
    double cp = cos(desired_pitch * 0.5);
    double sp = sin(desired_pitch * 0.5);
    double cr = cos(desired_roll * 0.5);
    double sr = sin(desired_roll * 0.5);
    std::cout << "desired_roll: " << desired_roll << ", desired_pitch: " << desired_pitch << ", desired_yaw: " << desired_yaw << std::endl;

    cmd.attitude.w() = cr * cp * cy + sr * sp * sy;
    cmd.attitude.x() = sr * cp * cy - cr * sp * sy;
    cmd.attitude.y() = cr * sp * cy + sr * cp * sy;
    cmd.attitude.z() = cr * cp * sy - sr * sp * cy;

    // 推力控制：使用固定的悬停推力
    // 姿态变化不应该大幅改变推力，否则飞机会上下起伏
    // 如果需要高度控制，应该通过调整desired_pitch来实现
    cmd.thrust = 0.313;  // 悬停油门固定值

    return cmd;
}

AttitudeThrustCommand LosGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist,
    const geometry_msgs::Point& los_angle) {

    AttitudeThrustCommand cmd;

    // Get current yaw from无人机自身姿态（而不是gimbal的LOS角度）
    Eigen::Quaterniond q(uav_pose.pose.orientation.w,
                         uav_pose.pose.orientation.x,
                         uav_pose.pose.orientation.y,
                         uav_pose.pose.orientation.z);
    // Yaw (Z轴旋转)
    double current_yaw = atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                               1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
    double current_pitch = los_angle.y;  // 俯仰仍用LOS

    // Compute desired LOS angles to target
    Eigen::Vector3d uav_pos(uav_pose.pose.position.x,
                             uav_pose.pose.position.y,
                             uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(target_pose.pose.position.x,
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);

    Eigen::Vector3d rel_pos = target_pos - uav_pos;
    double horizontal_dist = sqrt(rel_pos.x()*rel_pos.x() + rel_pos.y()*rel_pos.y());

    double desired_yaw = atan2(rel_pos.y(), rel_pos.x());
    double desired_pitch = atan2(-rel_pos.z(), horizontal_dist);

    // Compute errors
    double yaw_error = desired_yaw - current_yaw;
    yaw_error = atan2(sin(yaw_error), cos(yaw_error));  // Normalize

    double pitch_error = desired_pitch - current_pitch;
    pitch_error = atan2(sin(pitch_error), cos(pitch_error));

    // P control on angular errors to get angular rate commands
    double desired_yaw_rate = los_kp_yaw_ * yaw_error;
    double desired_pitch_rate = los_kp_pitch_ * pitch_error;

    // Clamp to max rates
    double yaw_rate_mag = std::abs(desired_yaw_rate);
    double pitch_rate_mag = std::abs(desired_pitch_rate);
    double max_rate = los_max_rate_;

    if (yaw_rate_mag > max_rate) {
        desired_yaw_rate *= max_rate / yaw_rate_mag;
    }
    if (pitch_rate_mag > max_rate) {
        desired_pitch_rate *= max_rate / pitch_rate_mag;
    }

    // 计算期望的roll和pitch
    // Roll: 用于协调转弯，基于yaw_error和横向位置
    // NWU坐标系: rel_pos.y() > 0 表示目标在左侧(西)，左转需要负roll
    //            rel_pos.y() < 0 表示目标在右侧(东)，右转需要正roll
    double roll_sign = (rel_pos.y() > 0) ? -1.0 : 1.0;
    double desired_roll = roll_sign * std::abs(yaw_error) * 2.0;  // yaw_error越大，roll越大
    double desired_pitch_for_cmd = desired_pitch;  // 俯仰控制高度

    // Clamp roll to reasonable range
    desired_roll = std::max(-0.5, std::min(0.5, desired_roll));

    // 使用绝对偏航角（不需要积分）
    cmd = computeAttitudeThrust(desired_roll, desired_pitch_for_cmd, desired_yaw);
    cmd.yaw_rate = -desired_yaw_rate;  // 偏航角速率用于cmd_vel

    ROS_DEBUG_THROTTLE(1.0, "[LOS] yaw_err=%.2f, pitch_err=%.2f, thrust=%.2f",
                       yaw_error, pitch_error, cmd.thrust);

    return cmd;
}

//==============================================================================
// Factory Function
//==============================================================================

std::unique_ptr<GuidanceStrategy> createGuidanceStrategy(
    GuidanceStrategyType type,
    ros::NodeHandle& nh) {

    switch (type) {
        case GuidanceStrategyType::INTERCEPT:
            return std::unique_ptr<InterceptGuidance>(new InterceptGuidance(nh));
        case GuidanceStrategyType::MINSNAP:
            return std::unique_ptr<MinSnapGuidance>(new MinSnapGuidance(nh));
        case GuidanceStrategyType::LOS:
            return std::unique_ptr<LosGuidance>(new LosGuidance(nh));
        default:
            ROS_WARN("[Guidance] Unknown strategy type, defaulting to Intercept");
            return std::unique_ptr<InterceptGuidance>(new InterceptGuidance(nh));
    }
}

} // namespace multi_uav_strike