#ifndef QUADCOPTER_BASIC_FUSION_H
#define QUADCOPTER_BASIC_FUSION_H

#include "flight_config.h"
#include "flight_types.h"

namespace quadcopter {

class BasicFusion {
public:
    explicit BasicFusion(const ImuConfig& config);
    bool begin(const ImuSample& sample);
    bool synchronize();
    void update(const ImuSample& sample, float cycle_seconds);
    bool healthy() const;
    const FusionAngles& angles() const;
    const char* diagnostic() const;

private:
    ImuConfig config_;
    FusionAngles angles_;
    float accelerometer_pitch_deg_;
    float accelerometer_roll_deg_;
    bool healthy_;
};

} // namespace quadcopter

#endif
