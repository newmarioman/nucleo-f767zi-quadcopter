#ifndef QUADCOPTER_FLIGHT_CONTROL_H
#define QUADCOPTER_FLIGHT_CONTROL_H

#include "flight_config.h"
#include "flight_types.h"
#include "pid.h"

namespace quadcopter {

class FlightControl {
public:
    explicit FlightControl(const ControlConfig& config);
    ControlOutput update(const RcCommand& command, const Attitude& attitude);
    void reset();

private:
    float stickDeviation(int32_t pulse_us) const;
    float updateAxis(int32_t command_us, float angle_deg, float rate_dps, PidController& pid);

    ControlConfig config_;
    PidController pitch_pid_;
    PidController roll_pid_;
};

} // namespace quadcopter

#endif
