#ifndef QUADCOPTER_ATTITUDE_ESTIMATOR_H
#define QUADCOPTER_ATTITUDE_ESTIMATOR_H

#include "fusion_selection.h"
#include "flight_config.h"
#include "flight_types.h"

#if SENSOR_FUSION_MODE == FUSION_BASIC
#include "basic_fusion.h"
#elif SENSOR_FUSION_MODE == FUSION_MADGWICK
#include "madgwick_fusion.h"
#elif SENSOR_FUSION_MODE == FUSION_DMP
#include "dmp_fusion.h"
#endif

namespace quadcopter {

class Mpu6050;

class AttitudeEstimator {
public:
    AttitudeEstimator(Mpu6050& sensor, const ImuConfig& imu, const FusionConfig& fusion);

    bool initialize(const ImuSample& sample);
    bool synchronize();
    void update(const ImuSample& sample, float cycle_seconds);
    void applyYawRateDeadband(float threshold_dps);
    const Attitude& attitude() const;
    bool healthy() const;
    const char* name() const;
    const char* diagnostic() const;

private:
    void copyAngles();

    ImuConfig config_;
    Attitude attitude_;
#if SENSOR_FUSION_MODE == FUSION_BASIC
    BasicFusion fusion_;
#elif SENSOR_FUSION_MODE == FUSION_MADGWICK
    MadgwickFusion fusion_;
#elif SENSOR_FUSION_MODE == FUSION_DMP
    DmpFusion fusion_;
#endif
};

} // namespace quadcopter

#endif
