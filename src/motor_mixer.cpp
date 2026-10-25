#include "motor_mixer.h"

namespace quadcopter {

MotorMixer::MotorMixer(int minimum_pulse_us, int maximum_pulse_us)
    : minimum_pulse_us_(minimum_pulse_us), maximum_pulse_us_(maximum_pulse_us)
{
}

MotorPulses MotorMixer::mix(float throttle_us, const ControlOutput& control) const
{
    MotorPulses pulses = {};
    // FRD positive roll: right side down; positive pitch: nose up.
    // Per diagram-FRD.png, M1/M3 spin CW and M2/M4 CCW (viewed from above).
    // Increasing CCW propeller torque produces positive (CW) body yaw.
    pulses.front_left_us = control.pitch + control.roll - control.yaw + throttle_us;
    pulses.front_right_us = control.pitch - control.roll + control.yaw + throttle_us;
    pulses.rear_right_us = -control.pitch - control.roll - control.yaw + throttle_us;
    pulses.rear_left_us = -control.pitch + control.roll + control.yaw + throttle_us;
    pulses.front_left_us = clamp(pulses.front_left_us);
    pulses.front_right_us = clamp(pulses.front_right_us);
    pulses.rear_right_us = clamp(pulses.rear_right_us);
    pulses.rear_left_us = clamp(pulses.rear_left_us);
    return pulses;
}

double MotorMixer::clamp(double pulse_us) const
{
    if (pulse_us < minimum_pulse_us_) {
        pulse_us = minimum_pulse_us_;
    }
    if (pulse_us > maximum_pulse_us_) {
        pulse_us = maximum_pulse_us_;
    }
    return pulse_us;
}

} // namespace quadcopter
