#ifndef QUADCOPTER_ARMING_CONTROL_H
#define QUADCOPTER_ARMING_CONTROL_H

#include "flight_config.h"
#include "flight_types.h"

namespace quadcopter {

class ArmingControl {
public:
    explicit ArmingControl(const MotorConfig& config);
    void update(const RcFrame& frame, float throttle_us, bool ground_allows_gesture);
    void disarm();
    bool armed() const;
    bool outputsEnabled(const RcFrame& frame) const;

private:
    MotorConfig config_;
    bool armed_;
};

} // namespace quadcopter

#endif
