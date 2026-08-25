// ============================================================================
// TargetMotionKF — 4-state 恒速 (CV) 卡尔曼滤波器, header-only
//
// 设计动机 (2026-08-23, 动目标跟踪):
//   原版 cluster 跟踪用 2D EMA (ema_alpha=0.3, τ≈0.23s), 只估计 [x, y],
//   没有速度状态。位置滞后 = v × τ, 5 m/s 是 1.17m, 10 m/s 2.33m,
//   20 m/s 4.66m — chase 路径严重偏航。
//
//   改用 4-state CV 模型 [x, y, vx, vy]:
//     - 静目标: Q_vel → 0 让速度衰减到 0, 等效位置 EMA, 零回归
//     - 动目标: Q_vel 充分 跟踪真速度, lead pose 让 chase 不滞后
//   z 轴单独由 target_z_prior_ 处理, 不进 KF
//
// 使用模式:
//   1. 聚类首帧: kf_.init(z_x, z_y)
//   2. 后续每帧:
//        kf_.predict(dt, q_pos, q_vel);
//        kf_.update(z_x, z_y, r_meas);
//   3. 状态输出:
//        state = kf_.getState();
//        lead_x = state.x + state.vx * lead_time;
//        speed = hypot(state.vx, state.vy);
//
// 线程安全: 不是线程安全。每个 cluster 一份实例, 调用方需要保证 KF 调用
//   来自同一 ROS callback (processSingleDetection) 或同一 timer (statesTimerCb)。
//   本工程在 processSingleDetection 里持有 clusters_mtx_, 在 statesTimerCb 里
//   也持有同一把锁, KF 操作串行化, 无并发问题。
// ============================================================================
#ifndef MULTI_UAV_STRIKE_TARGET_MOTION_KF_HPP
#define MULTI_UAV_STRIKE_TARGET_MOTION_KF_HPP

#include <Eigen/Dense>
#include <cmath>

namespace multi_uav_strike {

class TargetMotionKF {
public:
    struct State {
        double x;
        double y;
        double vx;
        double vy;
        State() : x(0.0), y(0.0), vx(0.0), vy(0.0) {}
        State(double xx, double yy, double vvx, double vvy)
            : x(xx), y(yy), vx(vvx), vy(vvy) {}
    };

    TargetMotionKF() = default;

    // 初始化 (首帧调用): 速度置 0, 协方差给大值让后续 update 快速收敛
    void init(double x, double y) {
        state_ = State(x, y, 0.0, 0.0);
        P_ = Eigen::Matrix4d::Identity() * 10.0;
        initialized_ = true;
    }

    bool isInitialized() const { return initialized_; }

    // 预测 (时间 dt, 单位秒):
    //   x ← x + vx*dt
    //   y ← y + vy*dt
    //   P ← F*P*F^T + Q
    // q_pos / q_vel 是过程噪声 std, 不是方差 (内部平方成 Q 项)
    void predict(double dt, double q_pos, double q_vel) {
        if (!initialized_ || dt <= 0.0) return;

        // 状态外推
        state_.x += state_.vx * dt;
        state_.y += state_.vy * dt;

        // 状态转移矩阵 (CV 模型, dt 小)
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        // 过程噪声 (离散 CV 模型, 噪声作用于加速度)
        //   位置 Q 与 dt^2 成正比 (白加速度积分效应), 速度 Q 与 dt 成正比
        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
        Q(0, 0) = q_pos * q_pos * dt * dt;
        Q(1, 1) = q_pos * q_pos * dt * dt;
        Q(2, 2) = q_vel * q_vel * dt * dt;
        Q(3, 3) = q_vel * q_vel * dt * dt;

        P_ = F * P_ * F.transpose() + Q;
    }

    // 更新 (测量 z = (meas_x, meas_y), 测量噪声 std = r_meas):
    //   K = P*H^T*(H*P*H^T + R)^-1
    //   x ← x + K*(z - H*x)
    //   P ← (I - K*H)*P
    void update(double meas_x, double meas_y, double r_meas) {
        if (!initialized_) return;

        // 测量矩阵 H = [[1,0,0,0],[0,1,0,0]] (位置观测量)
        Eigen::Matrix<double, 2, 4> H;
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (r_meas * r_meas);

        const Eigen::Matrix<double, 4, 2> PHt = P_ * H.transpose();
        const Eigen::Matrix2d S = H * PHt + R;
        const Eigen::Matrix<double, 4, 2> K = PHt * S.inverse();

        Eigen::Vector2d innov;
        innov << meas_x - state_.x, meas_y - state_.y;

        Eigen::Vector4d ns;
        ns << state_.x, state_.y, state_.vx, state_.vy;
        ns += K * innov;
        state_.x  = ns(0);
        state_.y  = ns(1);
        state_.vx = ns(2);
        state_.vy = ns(3);

        P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
    }

    // 直接读写状态 (公开给外部 helper, 如 classifyMotion)
    State getState() const { return state_; }

    // 当前估计速度幅值 (m/s)
    double getSpeed() const {
        return std::hypot(state_.vx, state_.vy);
    }

    // 带 lead_time 的预测位置:  (x + vx*lead_time, y + vy*lead_time)
    // chase 用这个让 UAV 不从目标后面飞过
    void getPositionWithLead(double lead_time, double& px, double& py) const {
        px = state_.x + state_.vx * lead_time;
        py = state_.y + state_.vy * lead_time;
    }

    // 速度异常 (发散) 时调用, 保留位置, 速度归零
    //   配合 motion_class_=STATIC 强制收敛, 等 KF 重新观察到速度再升级 DYNAMIC
    void resetVelocity() {
        state_.vx = 0.0;
        state_.vy = 0.0;
    }

    // 调试用: 协方差 trace (发散检测, 后续可加 P_trace > 阈值 时强制 reset)
    double getCovarianceTrace() const {
        return P_(0, 0) + P_(1, 1) + P_(2, 2) + P_(3, 3);
    }

private:
    State state_;
    Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();
    bool initialized_ = false;
};

}  // namespace multi_uav_strike

#endif  // MULTI_UAV_STRIKE_TARGET_MOTION_KF_HPP