#ifndef QUADCOPTER_FLIGHT_TYPES_H
#define QUADCOPTER_FLIGHT_TYPES_H

#include <stdint.h>

namespace quadcopter {

struct Vector3 {
    float x;
    float y;
    float z;
};

struct ImuSample {
    // Both vectors use body FRD: X forward, Y right, Z down.
    // Specific force is approximately (0, 0, -1) g when level and stationary.
    Vector3 acceleration_g;
    // X = roll rate p, Y = pitch rate q, Z = yaw rate r (right-hand rule).
    Vector3 angular_velocity_dps;
};

struct Attitude {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float integrated_pitch_deg;
    float integrated_roll_deg;
    Vector3 filtered_rates_dps; // Body FRD: X roll, Y pitch, Z yaw.
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct FusionAngles {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
};

struct RcFrame {
    enum Channel {
        Synchronization = 0,
        Roll = 1,
        Pitch = 2,
        Throttle = 3,
        Yaw = 4,
        Auxiliary1 = 5,
        Auxiliary2 = 6,
        MotorInhibit = 7,
        Auxiliary4 = 8,
        ChannelCount = 9
    };

    int32_t pulse_us[ChannelCount];
};

struct RcCommand {
    int32_t roll_us;
    int32_t pitch_us;
    int32_t yaw_us;
    float throttle_us;
};

struct ControlOutput {
    // Positive outputs request positive body FRD torques.
    float roll;
    float pitch;
    float yaw;
};

struct MotorPulses {
    double front_left_us;
    double front_right_us;
    double rear_right_us;
    double rear_left_us;
};

} // namespace quadcopter

#endif
