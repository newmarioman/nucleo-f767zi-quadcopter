#ifndef QUADCOPTER_PID_H
#define QUADCOPTER_PID_H

#include "flight_config.h"

namespace quadcopter {

class PidController {
public:
    explicit PidController(const PidConfig& config);
    float update(float error);
    void reset();

private:
    PidConfig config_;
    float integral_;
    float previous_error_;
    int previous_error_sign_;
};

} // namespace quadcopter

#endif
