#include "fusion_selection.h"
#include "madgwick_fusion.h"

#if SENSOR_FUSION_MODE == FUSION_MADGWICK

#include "orientation_math.h"

#include <cmath>

namespace quadcopter {

MadgwickFusion::MadgwickFusion(const ImuConfig& imu, const FusionConfig& config)
    : imu_config_(imu), filter_(config.madgwick_gyro_error_dps), angles_{},
      initial_yaw_deg_(0), healthy_(false)
{
}

bool MadgwickFusion::begin(const ImuSample& sample)
{
    Quaternion initial = {};
    initial_yaw_deg_ = 0;
    healthy_ = orientation_math::fromAcceleration(sample.acceleration_g, initial) &&
               filter_.setQuaternion(initial) && readAngles();
    if (healthy_) {
        initial_yaw_deg_ = angles_.yaw_deg;
        angles_.yaw_deg = 0;
    }
    return healthy_;
}

bool MadgwickFusion::synchronize()
{
    return healthy_;
}

void MadgwickFusion::update(const ImuSample& sample, float cycle_seconds)
{
    const Vector3& acceleration = sample.acceleration_g;
    const Vector3& rates = sample.angular_velocity_dps;
    if (!std::isfinite(cycle_seconds) || cycle_seconds <= 0.0f ||
        !std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) || !std::isfinite(rates.x) ||
        !std::isfinite(rates.y) || !std::isfinite(rates.z)) {
        healthy_ = false;
        return;
    }

    const float radians_per_degree = 0.017453292519943295f;
    // Work directly in body FRD. The filter's +Z reference is gravity down,
    // so negate the accelerometer's specific force, not the angular rates.
    filter_.update(-acceleration.x, -acceleration.y, -acceleration.z,
                   rates.x * radians_per_degree, rates.y * radians_per_degree,
                   rates.z * radians_per_degree, cycle_seconds);
    healthy_ = readAngles();
}

bool MadgwickFusion::readAngles()
{
    float values[4] = {};
    filter_.quaternion(values);
    const Quaternion quaternion = {values[0], values[1], values[2], values[3]};
    FusionAngles candidate = {};
    if (!orientation_math::toFlightAngles(quaternion, imu_config_.degrees_per_radian, candidate)) {
        return false;
    }
    candidate.yaw_deg = orientation_math::wrapYaw(candidate.yaw_deg - initial_yaw_deg_);
    angles_ = candidate;
    return true;
}

bool MadgwickFusion::healthy() const
{
    return healthy_;
}

const FusionAngles& MadgwickFusion::angles() const
{
    return angles_;
}

const char* MadgwickFusion::diagnostic() const
{
    return healthy_ ? "Madgwick filter ready" : "Madgwick initialization or numerical input failed";
}

} // namespace quadcopter

#endif
