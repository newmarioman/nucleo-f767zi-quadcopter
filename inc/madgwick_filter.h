#ifndef QUADCOPTER_MADGWICK_FILTER_H
#define QUADCOPTER_MADGWICK_FILTER_H

#include "flight_types.h"

namespace quadcopter {

// Six-axis quaternion filter used by the Madgwick fusion backend.
class MadgwickFilter {
public:
    explicit MadgwickFilter(float gyro_error_dps = 60.0f);
    // Angular rates are in radians per second. Acceleration is normalized internally.
    void update(float ax, float ay, float az, float gx, float gy, float gz,
                float cycle_seconds);
    void quaternion(float (&destination)[4]) const;
    void reset();
    bool setQuaternion(const Quaternion& quaternion);

private:
    float quaternion_[4];
    float beta_;
};

} // namespace quadcopter

#endif
