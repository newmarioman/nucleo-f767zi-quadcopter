#include "orientation_math.h"
#include "imu_mounting.h"

#include <cmath>

namespace quadcopter {
namespace orientation_math {

namespace {

float unitClamp(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

Quaternion multiply(const Quaternion& left, const Quaternion& right)
{
    return Quaternion{
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
    };
}

bool accelerationTilt(const Vector3& acceleration, float& roll, float& pitch)
{
    const float squared_norm = acceleration.x * acceleration.x +
                               acceleration.y * acceleration.y +
                               acceleration.z * acceleration.z;
    if (!std::isfinite(squared_norm) || squared_norm <= 0.0f) {
        return false;
    }
    // A stationary accelerometer measures the opposite of gravity in FRD.
    roll = std::atan2(-acceleration.y, -acceleration.z);
    pitch = std::atan2(acceleration.x,
                       std::sqrt(acceleration.y * acceleration.y +
                                 acceleration.z * acceleration.z));
    return true;
}

} // namespace

bool normalize(Quaternion& quaternion)
{
    const float squared_norm = quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                               quaternion.y * quaternion.y + quaternion.z * quaternion.z;
    if (!std::isfinite(squared_norm) || squared_norm <= 0.0f) {
        return false;
    }
    const float inverse_norm = 1.0f / std::sqrt(squared_norm);
    quaternion.w *= inverse_norm;
    quaternion.x *= inverse_norm;
    quaternion.y *= inverse_norm;
    quaternion.z *= inverse_norm;
    return true;
}

Vector3 sensorToBody(const Vector3& sensor_vector)
{
    Quaternion rotation = imu_mounting::sensor_to_body;
    if (!normalize(rotation)) {
        return Vector3{};
    }
    const Quaternion vector = {0.0f, sensor_vector.x, sensor_vector.y, sensor_vector.z};
    const Quaternion inverse = {rotation.w, -rotation.x, -rotation.y, -rotation.z};
    const Quaternion rotated = multiply(multiply(rotation, vector), inverse);
    return Vector3{rotated.x, rotated.y, rotated.z};
}

bool sensorToBodyAttitude(const Quaternion& sensor_quaternion, Quaternion& body_quaternion)
{
    Quaternion rotation = imu_mounting::sensor_to_body;
    Quaternion native = sensor_quaternion;
    if (!normalize(rotation) || !normalize(native)) {
        return false;
    }
    const Quaternion body_to_sensor = {rotation.w, -rotation.x, -rotation.y, -rotation.z};
    // Reference FLU -> NED: keep reference X, reverse Y/Z (180 degrees about X).
    // This is separate from physical mounting, and yaw is zeroed by DmpFusion.
    // With the FLU mounting, the result is (w, x, -y, -z).
    const Quaternion world_up_to_ned = {0.0f, 1.0f, 0.0f, 0.0f};
    body_quaternion = multiply(multiply(world_up_to_ned, native), body_to_sensor);
    return normalize(body_quaternion);
}

bool accelerometerAngles(const Vector3& acceleration, double degrees_per_radian,
                         FusionAngles& angles)
{
    float roll = 0.0f;
    float pitch = 0.0f;
    if (!accelerationTilt(acceleration, roll, pitch)) {
        return false;
    }
    angles.roll_deg = roll * degrees_per_radian;
    angles.pitch_deg = pitch * degrees_per_radian;
    return std::isfinite(angles.roll_deg) && std::isfinite(angles.pitch_deg);
}

bool fromAcceleration(const Vector3& acceleration, Quaternion& quaternion)
{
    float roll = 0.0f;
    float pitch = 0.0f;
    if (!accelerationTilt(acceleration, roll, pitch)) {
        return false;
    }
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    // ZYX attitude with zero heading and the measured roll/pitch.
    quaternion = Quaternion{cr * cp, sr * cp, cr * sp, -sr * sp};
    return normalize(quaternion);
}

bool toFlightAngles(const Quaternion& quaternion, double degrees_per_radian,
                    FusionAngles& angles)
{
    Quaternion normalized = quaternion;
    if (!normalize(normalized)) {
        return false;
    }
    angles.roll_deg = std::atan2(
        2.0f * (normalized.w * normalized.x + normalized.y * normalized.z),
        1.0f - 2.0f * (normalized.x * normalized.x + normalized.y * normalized.y)) *
        degrees_per_radian;
    angles.pitch_deg = std::asin(unitClamp(
        2.0f * (normalized.w * normalized.y - normalized.z * normalized.x))) *
        degrees_per_radian;
    const float yaw = std::atan2(
        2.0f * (normalized.w * normalized.z + normalized.x * normalized.y),
        1.0f - 2.0f * (normalized.y * normalized.y + normalized.z * normalized.z));
    angles.yaw_deg = wrapYaw(yaw * degrees_per_radian);
    return std::isfinite(angles.pitch_deg) && std::isfinite(angles.roll_deg) &&
           std::isfinite(angles.yaw_deg);
}

float wrapYaw(float yaw_deg)
{
    float wrapped = std::fmod(yaw_deg, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}

} // namespace orientation_math
} // namespace quadcopter
