#include "servo_controller.h"
#include <cmath>

namespace quadcopter {

ServoController::ServoController(PinName first_pin, PinName second_pin, const ServoConfig& config)
    : first_(first_pin), second_(second_pin), config_(config),
      pulses_us_{config.neutral_pulse_us, config.neutral_pulse_us}, enabled_(false)
{
    // Both pins share TIM2, so they must use the same period.
    first_.period_us(config_.pwm_period_us);
    second_.period_us(config_.pwm_period_us);
    disable();
}

void ServoController::enable()
{
    enabled_ = true;
    write(First);
    write(Second);
}

void ServoController::disable()
{
    enabled_ = false;
    first_.pulsewidth_us(0);
    second_.pulsewidth_us(0);
}

bool ServoController::enabled() const
{
    return enabled_;
}

void ServoController::center()
{
    setPulseWidthUs(First, config_.neutral_pulse_us);
    setPulseWidthUs(Second, config_.neutral_pulse_us);
}

void ServoController::setAngleDeg(Channel channel, float angle_deg)
{
    if (!std::isfinite(angle_deg)) {
        return;
    }
    if (angle_deg < 0.0f) {
        angle_deg = 0.0f;
    } else if (angle_deg > 180.0f) {
        angle_deg = 180.0f;
    }
    const float pulse_us = config_.minimum_pulse_us +
        (config_.maximum_pulse_us - config_.minimum_pulse_us) * (angle_deg / 180.0f);
    setPulseWidthUs(channel, static_cast<int>(pulse_us + 0.5f));
}

void ServoController::setPulseWidthUs(Channel channel, int pulse_us)
{
    if (channel != First && channel != Second) {
        return;
    }
    if (pulse_us < config_.minimum_pulse_us) {
        pulse_us = config_.minimum_pulse_us;
    } else if (pulse_us > config_.maximum_pulse_us) {
        pulse_us = config_.maximum_pulse_us;
    }
    pulses_us_[channel] = pulse_us;
    if (enabled_) {
        write(channel);
    }
}

int ServoController::pulseWidthUs(Channel channel) const
{
    return (channel == First || channel == Second) ? pulses_us_[channel] : 0;
}

void ServoController::write(Channel channel)
{
    if (channel == First) {
        first_.pulsewidth_us(pulses_us_[First]);
    } else {
        second_.pulsewidth_us(pulses_us_[Second]);
    }
}

void ServoController::updateChannel(Channel channel, int raw_pulse_us, const RadioConfig& radio)
{
    if (raw_pulse_us <= 0 || raw_pulse_us > radio.channel_max_us ||
        radio.axis_max_us <= radio.axis_min_us) {
        setPulseWidthUs(channel, config_.neutral_pulse_us);
        return;
    }
    const float position = static_cast<float>(raw_pulse_us + radio.command_offset_us -
                                              radio.axis_min_us) /
                           (radio.axis_max_us - radio.axis_min_us);
    setAngleDeg(channel, position * 180.0f);
}

void ServoController::updateFromReceiver(const RcFrame& frame, const RadioConfig& radio)
{
    updateChannel(First, frame.pulse_us[RcFrame::Auxiliary1], radio);
    updateChannel(Second, frame.pulse_us[RcFrame::Auxiliary2], radio);
}

} // namespace quadcopter
