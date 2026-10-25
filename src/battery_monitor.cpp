#include "battery_monitor.h"

namespace quadcopter {

BatteryMonitor::BatteryMonitor(PinName pin, const BatteryConfig& config)
    : input_(pin), config_(config), voltage_(0.0f), last_sample_us_(0), sampled_(false)
{
}

void BatteryMonitor::update(uint32_t now_us)
{
    if (sampled_ && now_us - last_sample_us_ < config_.sample_period_us) {
        return;
    }
    // AnalogIn::read() is normalized to 0..1. The 10k/1k divider has a 1/11 ratio.
    voltage_ = input_.read() * config_.adc_reference_voltage / config_.divider_ratio;
    last_sample_us_ = now_us;
    sampled_ = true;
}

float BatteryMonitor::voltage() const
{
    return voltage_;
}

bool BatteryMonitor::low() const
{
    return sampled_ && voltage_ < config_.low_voltage;
}

} // namespace quadcopter
