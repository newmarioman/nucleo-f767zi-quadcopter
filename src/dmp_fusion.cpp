#include "fusion_selection.h"
#include "dmp_fusion.h"

#if SENSOR_FUSION_MODE == FUSION_DMP

#include "orientation_math.h"

namespace quadcopter {

DmpFusion::DmpFusion(Mpu6050& sensor, const ImuConfig& imu, const FusionConfig& config)
    : imu_config_(imu), dmp_(sensor, config), angles_{}, initial_yaw_deg_(0), valid_angles_(false)
{
}

bool DmpFusion::begin(const ImuSample& sample)
{
    (void)sample;
    initial_yaw_deg_ = 0;
    valid_angles_ = dmp_.begin() && readAngles(dmp_.quaternion());
    if (valid_angles_) {
        initial_yaw_deg_ = angles_.yaw_deg;
        angles_.yaw_deg = 0;
    }
    return healthy();
}

bool DmpFusion::synchronize()
{
    // Clear packets collected during the blocking ESC startup and establish startup yaw.
    initial_yaw_deg_ = 0;
    valid_angles_ = dmp_.synchronize() && readAngles(dmp_.quaternion());
    if (valid_angles_) {
        initial_yaw_deg_ = angles_.yaw_deg;
        angles_.yaw_deg = 0;
    }
    return healthy();
}

void DmpFusion::update(const ImuSample& sample, float cycle_seconds)
{
    (void)sample;
    (void)cycle_seconds;
    Quaternion quaternion = {};
    if (dmp_.readLatest(quaternion)) {
        valid_angles_ = readAngles(quaternion);
    }
}

bool DmpFusion::readAngles(const Quaternion& quaternion)
{
    Quaternion body_quaternion = {};
    FusionAngles candidate = {};
    if (!orientation_math::sensorToBodyAttitude(quaternion, body_quaternion) ||
        !orientation_math::toFlightAngles(body_quaternion,
                                         imu_config_.degrees_per_radian, candidate)) {
        return false;
    }
    candidate.yaw_deg = orientation_math::wrapYaw(candidate.yaw_deg - initial_yaw_deg_);
    angles_ = candidate;
    return true;
}

bool DmpFusion::healthy() const
{
    return valid_angles_ && dmp_.healthy();
}

const FusionAngles& DmpFusion::angles() const
{
    return angles_;
}

const char* DmpFusion::diagnostic() const
{
    return dmp_.diagnostic();
}

} // namespace quadcopter

#endif
