#include "attitude_estimator.h"

namespace quadcopter {

AttitudeEstimator::AttitudeEstimator(Mpu6050& sensor, const ImuConfig& imu,
                                     const FusionConfig& fusion)
    : config_(imu), attitude_{},
#if SENSOR_FUSION_MODE == FUSION_BASIC
      fusion_(imu)
#elif SENSOR_FUSION_MODE == FUSION_MADGWICK
      fusion_(imu, fusion)
#elif SENSOR_FUSION_MODE == FUSION_DMP
      fusion_(sensor, imu, fusion)
#endif
{
#if SENSOR_FUSION_MODE != FUSION_DMP
    (void)sensor;
#endif
#if SENSOR_FUSION_MODE == FUSION_BASIC
    (void)fusion;
#endif
}

bool AttitudeEstimator::initialize(const ImuSample& sample)
{
    const bool ready = fusion_.begin(sample);
    copyAngles();
    attitude_.integrated_pitch_deg = attitude_.pitch_deg;
    attitude_.integrated_roll_deg = attitude_.roll_deg;
    return ready;
}

bool AttitudeEstimator::synchronize()
{
    const bool ready = fusion_.synchronize();
    copyAngles();
    return ready;
}

void AttitudeEstimator::update(const ImuSample& sample, float cycle_seconds)
{
    const Vector3& rates = sample.angular_velocity_dps;
    Vector3& filtered = attitude_.filtered_rates_dps;
    filtered.x = filtered.x * config_.rate_filter_memory + rates.x * config_.rate_filter_input;
    filtered.y = filtered.y * config_.rate_filter_memory + rates.y * config_.rate_filter_input;
    filtered.z = filtered.z * config_.rate_filter_memory + rates.z * config_.rate_filter_input;

    fusion_.update(sample, cycle_seconds);
    copyAngles();
    attitude_.integrated_roll_deg += rates.x * cycle_seconds;
    attitude_.integrated_pitch_deg += rates.y * cycle_seconds;
}

void AttitudeEstimator::copyAngles()
{
    const FusionAngles& angles = fusion_.angles();
    attitude_.pitch_deg = angles.pitch_deg;
    attitude_.roll_deg = angles.roll_deg;
    attitude_.yaw_deg = angles.yaw_deg;
}

void AttitudeEstimator::applyYawRateDeadband(float threshold_dps)
{
    // Preserve the deadband in the stored rate-filter state for every backend.
    if (attitude_.filtered_rates_dps.z < threshold_dps &&
        attitude_.filtered_rates_dps.z > -threshold_dps) {
        attitude_.filtered_rates_dps.z = 0;
    }
}

const Attitude& AttitudeEstimator::attitude() const
{
    return attitude_;
}

bool AttitudeEstimator::healthy() const
{
    return fusion_.healthy();
}

const char* AttitudeEstimator::name() const
{
    return SENSOR_FUSION_NAME;
}

const char* AttitudeEstimator::diagnostic() const
{
    return fusion_.diagnostic();
}

} // namespace quadcopter
