#ifndef QUADCOPTER_SERVO_CONTROLLER_H
#define QUADCOPTER_SERVO_CONTROLLER_H

#include "mbed.h"
#include "flight_config.h"
#include "flight_types.h"

namespace quadcopter {

class ServoController {
public:
    enum Channel { First = 0, Second = 1, ChannelCount = 2 };

    ServoController(PinName first_pin, PinName second_pin, const ServoConfig& config);
    void enable();
    void disable();
    bool enabled() const;
    void center();
    void setAngleDeg(Channel channel, float angle_deg);
    void setPulseWidthUs(Channel channel, int pulse_us);
    int pulseWidthUs(Channel channel) const;
    void updateFromReceiver(const RcFrame& frame, const RadioConfig& radio);

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;

private:
    void write(Channel channel);
    void updateChannel(Channel channel, int raw_pulse_us, const RadioConfig& radio);

    PwmOut first_;
    PwmOut second_;
    ServoConfig config_;
    int pulses_us_[ChannelCount];
    bool enabled_;
};

} // namespace quadcopter

#endif
