#ifndef QUADCOPTER_STATUS_INDICATORS_H
#define QUADCOPTER_STATUS_INDICATORS_H

#include "mbed.h"
#include <stdint.h>

namespace quadcopter {

class StatusIndicators {
public:
    StatusIndicators(DigitalOut& green, DigitalOut& blue);
    void update(bool armed, bool battery_low, uint32_t now_us);

private:
    static void updateBlink(DigitalOut& led, bool active, bool& was_active,
                            uint32_t& last_toggle_us, uint32_t half_period_us,
                            uint32_t now_us);

    DigitalOut& green_;
    DigitalOut& blue_;
    bool green_active_;
    bool blue_active_;
    uint32_t green_last_toggle_us_;
    uint32_t blue_last_toggle_us_;
};

} // namespace quadcopter

#endif
