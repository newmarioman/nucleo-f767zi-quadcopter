#ifndef QUADCOPTER_MOTOR_MIXER_H
#define QUADCOPTER_MOTOR_MIXER_H

#include "flight_types.h"

namespace quadcopter {

class MotorMixer {
public:
    MotorMixer(int minimum_pulse_us, int maximum_pulse_us);
    MotorPulses mix(float throttle_us, const ControlOutput& control) const;

private:
    double clamp(double pulse_us) const;

    int minimum_pulse_us_;
    int maximum_pulse_us_;
};

} // namespace quadcopter

#endif
