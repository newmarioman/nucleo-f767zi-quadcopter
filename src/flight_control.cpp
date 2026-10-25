#include "flight_control.h"

namespace quadcopter {

FlightControl::FlightControl(const ControlConfig& config)
    : config_(config), pitch_pid_(config.roll_pitch_rate), roll_pid_(config.roll_pitch_rate)
{
}

float FlightControl::stickDeviation(int32_t pulse_us) const
{
    if (pulse_us > config_.stick_deadband_high_us) {
        return pulse_us - config_.stick_deadband_high_us;
    }
    if (pulse_us < config_.stick_deadband_low_us) {
        return pulse_us - config_.stick_deadband_low_us;
    }
    return 0;
}

float FlightControl::updateAxis(int32_t command_us, float angle_deg, float rate_dps,
                                PidController& pid)
{
    const float angle_setpoint = stickDeviation(command_us) / config_.roll_pitch_stick_divisor;
    float rate_setpoint = (angle_setpoint - angle_deg) * config_.roll_pitch_angle_gain;
    if (rate_setpoint > config_.roll_pitch_rate_limit) {
        rate_setpoint = config_.roll_pitch_rate_limit;
    } else if (rate_setpoint < -config_.roll_pitch_rate_limit) {
        rate_setpoint = -config_.roll_pitch_rate_limit;
    }
    return pid.update(rate_setpoint - rate_dps);
}

ControlOutput FlightControl::update(const RcCommand& command, const Attitude& attitude)
{
    ControlOutput output = {};
    output.roll = updateAxis(command.roll_us, attitude.roll_deg,
                             attitude.filtered_rates_dps.x, roll_pid_);
    output.pitch = updateAxis(command.pitch_us, attitude.pitch_deg,
                              attitude.filtered_rates_dps.y, pitch_pid_);

    float yaw_rate_setpoint = stickDeviation(command.yaw_us);
    yaw_rate_setpoint /= config_.yaw_stick_divisor;
    const float yaw_error = yaw_rate_setpoint - attitude.filtered_rates_dps.z;
    output.yaw = config_.yaw_gain * yaw_error;
    if (output.yaw > config_.yaw_output_limit) {
        output.yaw = config_.yaw_output_limit;
    } else if (output.yaw < -config_.yaw_output_limit) {
        output.yaw = -config_.yaw_output_limit;
    }
    return output;
}

void FlightControl::reset()
{
    pitch_pid_.reset();
    roll_pid_.reset();
}

} // namespace quadcopter
