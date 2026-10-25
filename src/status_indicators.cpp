#include "status_indicators.h"

namespace quadcopter {

StatusIndicators::StatusIndicators(DigitalOut& green, DigitalOut& blue)
    : green_(green), blue_(blue), green_active_(false), blue_active_(false),
      green_last_toggle_us_(0), blue_last_toggle_us_(0)
{
    green_ = 0;
    blue_ = 0;
}

void StatusIndicators::update(bool armed, bool battery_low, uint32_t now_us)
{
    // Half periods: 250 ms gives 2 Hz; 1000 ms gives 0.5 Hz (50% duty cycle).
    updateBlink(green_, armed, green_active_, green_last_toggle_us_, 250000u, now_us);
    updateBlink(blue_, battery_low, blue_active_, blue_last_toggle_us_, 1000000u, now_us);
}

void StatusIndicators::updateBlink(DigitalOut& led, bool active, bool& was_active,
                                    uint32_t& last_toggle_us, uint32_t half_period_us,
                                    uint32_t now_us)
{
    if (!active) {
        led = 0;
    } else if (!was_active) {
        led = 1;
        last_toggle_us = now_us;
    } else {
        const uint32_t elapsed_periods = (now_us - last_toggle_us) / half_period_us;
        if (elapsed_periods != 0) {
            if ((elapsed_periods & 1u) != 0) {
                led = !led;
            }
            last_toggle_us += elapsed_periods * half_period_us;
        }
    }
    was_active = active;
}

} // namespace quadcopter
