#ifndef QUADCOPTER_RC_H
#define QUADCOPTER_RC_H

#include "mbed.h"
#include "flight_config.h"
#include "flight_types.h"

namespace quadcopter {

class RcReceiver {
public:
    RcReceiver(PinName pin, const RadioConfig& config);
    ~RcReceiver();

    void begin();
    RcFrame snapshot() const;
    RcCommand command(const RcFrame& frame) const;

    RcReceiver(const RcReceiver&) = delete;
    RcReceiver& operator=(const RcReceiver&) = delete;

private:
    void onRise();
    void onFall();
    void clearFrame();
    int32_t clampAxis(int32_t value) const;

    InterruptIn input_;
    Timer pulse_timer_;
    RadioConfig config_;
    volatile int32_t pulses_us_[RcFrame::ChannelCount];
    unsigned int next_channel_;
    bool synchronized_;
};

} // namespace quadcopter

#endif
