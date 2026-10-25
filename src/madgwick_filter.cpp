#include "madgwick_filter.h"
#include "orientation_math.h"

#include <cmath>

namespace quadcopter {

MadgwickFilter::MadgwickFilter(float gyro_error_dps)
    : quaternion_{1.0f, 0.0f, 0.0f, 0.0f}, beta_(0)
{
    const float pi = 3.14159265358979323846f;
    const float gyro_measurement_error = pi * (gyro_error_dps / 180.0f);
    beta_ = std::sqrt(3.0f / 4.0f) * gyro_measurement_error;
}

void MadgwickFilter::reset()
{
    quaternion_[0] = 1.0f;
    quaternion_[1] = quaternion_[2] = quaternion_[3] = 0.0f;
}

bool MadgwickFilter::setQuaternion(const Quaternion& quaternion)
{
    Quaternion normalized = quaternion;
    if (!orientation_math::normalize(normalized)) {
        return false;
    }
    quaternion_[0] = normalized.w;
    quaternion_[1] = normalized.x;
    quaternion_[2] = normalized.y;
    quaternion_[3] = normalized.z;
    return true;
}

void MadgwickFilter::quaternion(float (&destination)[4]) const
{
    for (unsigned int index = 0; index < 4; ++index) {
        destination[index] = quaternion_[index];
    }
}

// Based on the Sebastian Madgwick implementation included in the original driver.
// Reference retained from that driver: http://www.x-io.co.uk/category/open-source/
void MadgwickFilter::update(float ax, float ay, float az, float gx, float gy, float gz,
                            float cycle_seconds)
{
    float q1 = quaternion_[0], q2 = quaternion_[1], q3 = quaternion_[2], q4 = quaternion_[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;

    hatDot1 = hatDot2 = hatDot3 = hatDot4 = 0.0f;
    // Normalise accelerometer measurement
    norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Compute the objective function and Jacobian
        f1 = _2q2 * q4 - _2q1 * q3 - ax;
        f2 = _2q1 * q2 + _2q3 * q4 - ay;
        f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
        J_11or24 = _2q3;
        J_12or23 = _2q4;
        J_13or22 = _2q1;
        J_14or21 = _2q2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;

        // Compute the gradient (matrix multiplication)
        hatDot1 = J_14or21 * f2 - J_11or24 * f1;
        hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
        hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
        hatDot4 = J_14or21 * f1 + J_11or24 * f2;

        // Normalize the gradient
        norm = std::sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
        if (norm > 0.0f) {
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
        } else {
            hatDot1 = hatDot2 = hatDot3 = hatDot4 = 0.0f;
        }
    }

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta_ * hatDot1)) * cycle_seconds;
    q2 += (qDot2 -(beta_ * hatDot2)) * cycle_seconds;
    q3 += (qDot3 -(beta_ * hatDot3)) * cycle_seconds;
    q4 += (qDot4 -(beta_ * hatDot4)) * cycle_seconds;

    // Normalize the quaternion
    norm = std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // Normalize the quaternion.
    if (norm == 0.0f) {
        return;
    }
    norm = 1.0f/norm;
    quaternion_[0] = q1 * norm;
    quaternion_[1] = q2 * norm;
    quaternion_[2] = q3 * norm;
    quaternion_[3] = q4 * norm;

}

} // namespace quadcopter
