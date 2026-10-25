#ifndef QUADCOPTER_DMP_FUSION_H
#define QUADCOPTER_DMP_FUSION_H

#include "flight_config.h"
#include "flight_types.h"
#include "mpu6050_dmp.h"

namespace quadcopter {

class DmpFusion {
public:
    DmpFusion(Mpu6050& sensor, const ImuConfig& imu, const FusionConfig& config);
    bool begin(const ImuSample& sample);
    bool synchronize();
    void update(const ImuSample& sample, float cycle_seconds);
    bool healthy() const;
    const FusionAngles& angles() const;
    const char* diagnostic() const;

private:
    bool readAngles(const Quaternion& quaternion);

    ImuConfig imu_config_;
    Mpu6050Dmp dmp_;
    FusionAngles angles_;
    float initial_yaw_deg_;
    bool valid_angles_;
};

} // namespace quadcopter

#endif
