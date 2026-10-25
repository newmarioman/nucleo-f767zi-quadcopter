#ifndef QUADCOPTER_MADGWICK_FUSION_H
#define QUADCOPTER_MADGWICK_FUSION_H

#include "flight_config.h"
#include "flight_types.h"
#include "madgwick_filter.h"

namespace quadcopter {

class MadgwickFusion {
public:
    MadgwickFusion(const ImuConfig& imu, const FusionConfig& config);
    bool begin(const ImuSample& sample);
    bool synchronize();
    void update(const ImuSample& sample, float cycle_seconds);
    bool healthy() const;
    const FusionAngles& angles() const;
    const char* diagnostic() const;

private:
    bool readAngles();

    ImuConfig imu_config_;
    MadgwickFilter filter_;
    FusionAngles angles_;
    float initial_yaw_deg_;
    bool healthy_;
};

} // namespace quadcopter

#endif
