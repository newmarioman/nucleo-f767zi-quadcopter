#include "motor_controller.h"

namespace quadcopter {

MotorController::MotorController(const BoardConfig& board, const MotorConfig& config)
    : front_left_(board.motor_front_left), front_right_(board.motor_front_right),
      rear_right_(board.motor_rear_right), rear_left_(board.motor_rear_left),
      config_(config), mixer_(config.minimum_pulse_us, config.maximum_pulse_us),
      arming_(config), mixed_pulses_{}, outputs_configured_(false)
{
}

void MotorController::configureOutputs()
{
    if (outputs_configured_) {
        return;
    }
    front_left_.period_us(config_.pwm_period_us);
    front_right_.period_us(config_.pwm_period_us);
    rear_right_.period_us(config_.pwm_period_us);
    rear_left_.period_us(config_.pwm_period_us);
    outputs_configured_ = true;
}

void MotorController::initializeEscs()
{
    configureOutputs();
    writeAll(config_.maximum_pulse_us);
    wait_ms(config_.startup_high_ms);
    stop();
    wait_ms(config_.startup_low_ms);
}

void MotorController::update(const RcFrame& frame, float throttle_us, const ControlOutput& control,
                             bool ground_allows_gesture)
{
    if (throttle_us > config_.maximum_throttle_us) {
        throttle_us = config_.maximum_throttle_us;
    }
    mixed_pulses_ = mixer_.mix(throttle_us, control);
    arming_.update(frame, throttle_us, ground_allows_gesture);
    if (arming_.outputsEnabled(frame)) {
        write(mixed_pulses_);
    } else {
        stop();
    }
}

void MotorController::write(const MotorPulses& pulses)
{
    front_left_ = pulses.front_left_us / config_.pwm_period_us;
    front_right_ = pulses.front_right_us / config_.pwm_period_us;
    rear_right_ = pulses.rear_right_us / config_.pwm_period_us;
    rear_left_ = pulses.rear_left_us / config_.pwm_period_us;
}

void MotorController::writeAll(double pulse_us)
{
    const MotorPulses pulses = {pulse_us, pulse_us, pulse_us, pulse_us};
    write(pulses);
}

void MotorController::stop()
{
    configureOutputs();
    writeAll(config_.minimum_pulse_us);
}

bool MotorController::armed() const
{
    return arming_.armed();
}

void MotorController::disarm()
{
    arming_.disarm();
    stop();
}

const MotorPulses& MotorController::mixedPulses() const
{
    return mixed_pulses_;
}

} // namespace quadcopter
