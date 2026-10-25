#include "rc.h"

namespace quadcopter {

RcReceiver::RcReceiver(PinName pin, const RadioConfig& config)
    : input_(pin), config_(config), pulses_us_{}, next_channel_(0),
      synchronized_(false)
{
}

RcReceiver::~RcReceiver()
{
    input_.rise(NULL);
    input_.fall(NULL);
}

void RcReceiver::begin()
{
    input_.rise(callback(this, &RcReceiver::onRise));
    input_.fall(callback(this, &RcReceiver::onFall));
}

void RcReceiver::onRise()
{
    pulse_timer_.start();
}

void RcReceiver::onFall()
{
    pulse_timer_.stop();
    const int32_t width_us = pulse_timer_.read_us();

    if ((width_us > config_.synchronization_min_us &&
         width_us < config_.synchronization_max_us) || synchronized_) {
        synchronized_ = true;
        pulses_us_[next_channel_] = width_us;
        pulse_timer_.reset();
        ++next_channel_;
        if (next_channel_ == RcFrame::ChannelCount) {
            next_channel_ = 0;
        }

        for (unsigned int channel = 1; channel < RcFrame::ChannelCount; ++channel) {
            if (pulses_us_[channel] > config_.channel_max_us) {
                synchronized_ = false;
                next_channel_ = 0;
                clearFrame();
                break;
            }
        }
    } else {
        pulse_timer_.reset();
    }
}

void RcReceiver::clearFrame()
{
    for (unsigned int channel = 0; channel < RcFrame::ChannelCount; ++channel) {
        pulses_us_[channel] = 0;
    }
}

RcFrame RcReceiver::snapshot() const
{
    RcFrame frame = {};
    // Preserve the caller's interrupt mask while copying the ISR-owned values.
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    for (unsigned int channel = 0; channel < RcFrame::ChannelCount; ++channel) {
        frame.pulse_us[channel] = pulses_us_[channel];
    }
    __set_PRIMASK(interrupt_mask);
    return frame;
}

RcCommand RcReceiver::command(const RcFrame& frame) const
{
    RcCommand result = {};
    result.roll_us = clampAxis(frame.pulse_us[RcFrame::Roll] + config_.command_offset_us);
    result.pitch_us = clampAxis(frame.pulse_us[RcFrame::Pitch] + config_.command_offset_us);
    result.yaw_us = clampAxis(frame.pulse_us[RcFrame::Yaw] + config_.command_offset_us);
    result.throttle_us = frame.pulse_us[RcFrame::Throttle] + config_.command_offset_us;
    return result;
}

int32_t RcReceiver::clampAxis(int32_t value) const
{
    if (value > config_.axis_max_us) {
        value = config_.axis_max_us;
    }
    if (value < config_.axis_min_us) {
        value = config_.axis_min_us;
    }
    return value;
}

} // namespace quadcopter
