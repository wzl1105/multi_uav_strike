#ifndef MULTI_UAV_STRIKE_VELOCITY_VECTOR_LIMITER_H
#define MULTI_UAV_STRIKE_VELOCITY_VECTOR_LIMITER_H

#include <algorithm>
#include <cmath>

namespace multi_uav_strike {

// 保持三维速度方向不变，用水平/垂直速度上限共同约束整个速度矢量。
inline void limitVelocityVector(double& north, double& east, double& down,
                                double max_horizontal_speed,
                                double max_vertical_speed) {
    double scale = 1.0;
    const double horizontal_speed = std::hypot(north, east);
    if (horizontal_speed > max_horizontal_speed && horizontal_speed > 1e-9) {
        scale = std::min(scale, max_horizontal_speed / horizontal_speed);
    }
    if (std::fabs(down) > max_vertical_speed && std::fabs(down) > 1e-9) {
        scale = std::min(scale, max_vertical_speed / std::fabs(down));
    }
    north *= scale;
    east *= scale;
    down *= scale;
}

}  // namespace multi_uav_strike

#endif  // MULTI_UAV_STRIKE_VELOCITY_VECTOR_LIMITER_H
