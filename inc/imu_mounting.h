#ifndef QUADCOPTER_IMU_MOUNTING_H
#define QUADCOPTER_IMU_MOUNTING_H

#include "flight_types.h"

namespace quadcopter {
namespace imu_mounting {

// MPU6050 mounting: native +X FRONT, +Y LEFT, +Z UP (FLU).
// Unit quaternion rotating native vectors into body FRD (180 degrees about X):
// body X = sensor X, body Y = -sensor Y, body Z = -sensor Z.
// Acceleration, software gyro bias/rates and DMP orientation share this rotation.
// Change only this rotation if the physical sensor mounting changes.
constexpr Quaternion sensor_to_body = {0.0f, 1.0f, 0.0f, 0.0f};

} // namespace imu_mounting
} // namespace quadcopter

#endif
