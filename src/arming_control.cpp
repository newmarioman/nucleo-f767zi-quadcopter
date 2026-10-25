#include "arming_control.h"

namespace quadcopter {

ArmingControl::ArmingControl(const MotorConfig& config)
    : config_(config), armed_(false)
{
}

void ArmingControl::update(const RcFrame& frame, float throttle_us, bool ground_allows_gesture)
{
    if (!ground_allows_gesture) {
        return;
    }

    if (!armed_) {
        if (frame.pulse_us[RcFrame::Yaw] < config_.gesture_low_us &&
            frame.pulse_us[RcFrame::Throttle] < config_.gesture_low_us &&
            frame.pulse_us[RcFrame::Pitch] < config_.gesture_low_us &&
            frame.pulse_us[RcFrame::Roll] > config_.gesture_high_us) {
            armed_ = true;
        }
    }

    if (armed_ && throttle_us < config_.disarm_throttle_us) {
        if (frame.pulse_us[RcFrame::Yaw] > config_.gesture_high_us &&
            frame.pulse_us[RcFrame::Throttle] < config_.gesture_low_us &&
            frame.pulse_us[RcFrame::Pitch] < config_.gesture_low_us &&
            frame.pulse_us[RcFrame::Roll] < config_.gesture_low_us) {
            armed_ = false;
        }
    }
}

bool ArmingControl::armed() const
{
    return armed_;
}

void ArmingControl::disarm()
{
    armed_ = false;
}

bool ArmingControl::outputsEnabled(const RcFrame& frame) const
{
    return frame.pulse_us[RcFrame::MotorInhibit] < config_.inhibit_threshold_us && armed_;
}

} // namespace quadcopter
