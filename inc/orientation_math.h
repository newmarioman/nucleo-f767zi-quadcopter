#ifndef QUADCOPTER_ORIENTATION_MATH_H
#define QUADCOPTER_ORIENTATION_MATH_H

#include "flight_types.h"

namespace quadcopter {
namespace orientation_math {

bool normalize(Quaternion& quaternion);
Vector3 sensorToBody(const Vector3& sensor_vector);
// Convert a DMP sensor-to-world-up quaternion into body-to-world-NED attitude.
bool sensorToBodyAttitude(const Quaternion& sensor_quaternion, Quaternion& body_quaternion);
// Acceleration is measured specific force in body FRD, not the gravity vector.
bool accelerometerAngles(const Vector3& acceleration, double degrees_per_radian,
                         FusionAngles& angles);
bool fromAcceleration(const Vector3& acceleration, Quaternion& quaternion);
// Input is a body-to-world-NED quaternion; output uses ZYX Euler angles.
bool toFlightAngles(const Quaternion& quaternion, double degrees_per_radian,
                    FusionAngles& angles);
float wrapYaw(float yaw_deg);

} // namespace orientation_math
} // namespace quadcopter

#endif
