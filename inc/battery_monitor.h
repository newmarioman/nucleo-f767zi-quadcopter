#ifndef QUADCOPTER_BATTERY_MONITOR_H
#define QUADCOPTER_BATTERY_MONITOR_H

#include "mbed.h"
#include "flight_config.h"

namespace quadcopter {

class BatteryMonitor {
public:
    BatteryMonitor(PinName pin, const BatteryConfig& config);
    void update(uint32_t now_us);
    float voltage() const;
    bool low() const;

    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

private:
    AnalogIn input_;
    BatteryConfig config_;
    float voltage_;
    uint32_t last_sample_us_;
    bool sampled_;
};

} // namespace quadcopter

#endif
