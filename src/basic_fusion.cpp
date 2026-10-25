#include "fusion_selection.h"
#include "basic_fusion.h"
#include "orientation_math.h"

#if SENSOR_FUSION_MODE == FUSION_BASIC

#include <cmath>

namespace quadcopter {

BasicFusion::BasicFusion(const ImuConfig& config)
    : config_(config), angles_{}, accelerometer_pitch_deg_(0), accelerometer_roll_deg_(0),
      healthy_(false)
{
}

bool BasicFusion::begin(const ImuSample& sample)
{
    angles_ = FusionAngles{};
    healthy_ = orientation_math::accelerometerAngles(
        sample.acceleration_g, config_.degrees_per_radian, angles_);
    if (healthy_) {
        accelerometer_pitch_deg_ = angles_.pitch_deg;
        accelerometer_roll_deg_ = angles_.roll_deg;
    }
    return healthy_;
}

void BasicFusion::update(const ImuSample& sample, float cycle_seconds)
{
    const Vector3& rates = sample.angular_velocity_dps;
    const Vector3& acceleration = sample.acceleration_g;
    if (!std::isfinite(cycle_seconds) || cycle_seconds <= 0.0f ||
        !std::isfinite(rates.x) || !std::isfinite(rates.y) || !std::isfinite(rates.z) ||
        !std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z)) {
        healthy_ = false;
        return;
    }
    angles_.roll_deg += rates.x * cycle_seconds;
    angles_.pitch_deg += rates.y * cycle_seconds;
    angles_.yaw_deg += rates.z * cycle_seconds;
    angles_.yaw_deg = orientation_math::wrapYaw(angles_.yaw_deg);

    FusionAngles measured = {};
    if (orientation_math::accelerometerAngles(
            acceleration, config_.degrees_per_radian, measured)) {
        accelerometer_pitch_deg_ = measured.pitch_deg;
        accelerometer_roll_deg_ = measured.roll_deg;
        angles_.pitch_deg = angles_.pitch_deg * (1 - config_.pitch_correction) +
                              accelerometer_pitch_deg_ * config_.pitch_correction;
        angles_.roll_deg = angles_.roll_deg * (1 - config_.roll_correction) +
                             accelerometer_roll_deg_ * config_.roll_correction;
    }
    healthy_ = std::isfinite(angles_.pitch_deg) && std::isfinite(angles_.roll_deg) &&
               std::isfinite(angles_.yaw_deg);
}

bool BasicFusion::synchronize()
{
    return healthy_;
}

bool BasicFusion::healthy() const
{
    return healthy_;
}

const FusionAngles& BasicFusion::angles() const
{
    return angles_;
}

const char* BasicFusion::diagnostic() const
{
    return healthy_ ? "Basic complementary filter ready" : "Invalid Basic fusion input";
}

} // namespace quadcopter

#endif
