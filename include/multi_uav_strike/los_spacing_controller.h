#ifndef MULTI_UAV_STRIKE_LOS_SPACING_CONTROLLER_H
#define MULTI_UAV_STRIKE_LOS_SPACING_CONTROLLER_H

#include <cmath>
#include <vector>

namespace multi_uav_strike {

class LosSpacingController {
public:
    LosSpacingController(double threshold_rad, double angle_gain,
                         double disable_distance)
        : threshold_rad_(threshold_rad),
          angle_gain_(angle_gain),
          disable_distance_(disable_distance) {}

    double desiredRate(double own_los,
                       const std::vector<double>& neighbor_los,
                       double horizontal_distance) const {
        if (horizontal_distance < disable_distance_ || neighbor_los.empty()) {
            return 0.0;
        }

        double rate_sum = 0.0;
        std::size_t active_count = 0;
        for (double los : neighbor_los) {
            const double difference = wrapPi(own_los - los);
            if (std::fabs(difference) > threshold_rad_) {
                continue;
            }
            const double side = difference >= 0.0 ? 1.0 : -1.0;
            const double desired_los = los + side * threshold_rad_;
            rate_sum += angle_gain_ * wrapPi(desired_los - own_los);
            ++active_count;
        }
        return active_count == 0 ? 0.0 : rate_sum / active_count;
    }

    static double wrapPi(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

private:
    double threshold_rad_;
    double angle_gain_;
    double disable_distance_;
};

}  // namespace multi_uav_strike

#endif  // MULTI_UAV_STRIKE_LOS_SPACING_CONTROLLER_H
