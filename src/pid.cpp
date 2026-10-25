#include "pid.h"

namespace quadcopter {

PidController::PidController(const PidConfig& config)
    : config_(config), integral_(0), previous_error_(0), previous_error_sign_(0)
{
}

float PidController::update(float error)
{
    int error_sign = 0;
    if (error > 0) {
        error_sign = 1;
    } else if (error < 0) {
        error_sign = 2;
    } else {
        integral_ = 0;
    }

    const float proportional = config_.kp * error;
    // The integral and derivative are per iteration, without dt scaling.
    float candidate_integral = integral_;
    if (error_sign == previous_error_sign_) {
        candidate_integral += config_.ki * error;
        if (candidate_integral > config_.integral_limit) {
            candidate_integral = config_.integral_limit;
        } else if (candidate_integral < -config_.integral_limit) {
            candidate_integral = -config_.integral_limit;
        }
    } else {
        integral_ = 0;
        candidate_integral = 0;
    }

    const float derivative = config_.kd * (error - previous_error_);
    const float candidate_output = proportional + candidate_integral + derivative;
    // Conditional integration: do not accumulate in the direction that pushes
    // an already saturated PID output further into saturation. Allow unwinding.
    const bool worsens_saturation =
        (candidate_output > config_.output_limit && candidate_integral > integral_) ||
        (candidate_output < -config_.output_limit && candidate_integral < integral_);
    if (!worsens_saturation) {
        integral_ = candidate_integral;
    }
    float output = proportional + integral_ + derivative;
    if (output > config_.output_limit) {
        output = config_.output_limit;
    } else if (output < -config_.output_limit) {
        output = -config_.output_limit;
    }

    previous_error_ = error;
    previous_error_sign_ = error_sign;
    return output;
}

void PidController::reset()
{
    integral_ = 0;
    previous_error_ = 0;
    previous_error_sign_ = 0;
}

} // namespace quadcopter
