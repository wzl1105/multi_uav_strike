#ifndef MULTI_UAV_STRIKE_LOW_PASS_FILTER_2P_H
#define MULTI_UAV_STRIKE_LOW_PASS_FILTER_2P_H

#include <algorithm>
#include <cmath>

namespace multi_uav_strike {

// Second-order Butterworth low-pass (bilinear transform), following the same
// Direct Form II structure used by the Plane-4.6.3-LS filter utilities.
class LowPassFilter2p {
public:
    LowPassFilter2p()
        : a1_(0.0), a2_(0.0), b0_(1.0), b1_(0.0), b2_(0.0),
          delay1_(0.0), delay2_(0.0), initialized_(false) {}

    LowPassFilter2p(double sample_hz, double cutoff_hz) : LowPassFilter2p() {
        configure(sample_hz, cutoff_hz);
    }

    void configure(double sample_hz, double cutoff_hz) {
        if (sample_hz <= 0.0 || cutoff_hz <= 0.0) {
            a1_ = a2_ = b1_ = b2_ = 0.0;
            b0_ = 1.0;
            return;
        }
        cutoff_hz = std::min(cutoff_hz, 0.4 * sample_hz);
        const double omega = std::tan(M_PI * cutoff_hz / sample_hz);
        const double normalizer =
            1.0 + 2.0 * std::cos(M_PI / 4.0) * omega + omega * omega;
        b0_ = omega * omega / normalizer;
        b1_ = 2.0 * b0_;
        b2_ = b0_;
        a1_ = 2.0 * (omega * omega - 1.0) / normalizer;
        a2_ = (1.0 - 2.0 * std::cos(M_PI / 4.0) * omega + omega * omega) /
              normalizer;
    }

    double apply(double sample) {
        if (!initialized_) {
            reset(sample);
        }
        const double delay0 = sample - delay1_ * a1_ - delay2_ * a2_;
        const double output = delay0 * b0_ + delay1_ * b1_ + delay2_ * b2_;
        delay2_ = delay1_;
        delay1_ = delay0;
        return output;
    }

    void reset() { initialized_ = false; }

    void reset(double value) {
        const double denominator = 1.0 + a1_ + a2_;
        delay1_ = delay2_ = std::fabs(denominator) > 1e-12
            ? value / denominator : value;
        initialized_ = true;
    }

private:
    double a1_, a2_, b0_, b1_, b2_;
    double delay1_, delay2_;
    bool initialized_;
};

}  // namespace multi_uav_strike

#endif  // MULTI_UAV_STRIKE_LOW_PASS_FILTER_2P_H
