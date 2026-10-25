/* Copyright (c) 2013 Prabhu Desai
 * pdtechworld@gmail.com
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * For more details on the sensor :
 * http://www.elecfreaks.com/store/hcsr04-ultrasonic-sensor-distance-measuring-module-p-91.html?zenid=pgm8pgnvaodbe36dibq5s1soi3
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "range_finder.h"
#include "hal/us_ticker_api.h"

namespace quadcopter {

namespace {
const uint32_t measurement_period_us = 60000u;
const uint32_t echo_timeout_us = 30000u;
const uint32_t maximum_reading_age_us = 250000u;
const uint32_t minimum_echo_us = 100u;
}

RangeFinder::RangeFinder(PinName trigger_pin, PinName echo_pin)
    : trigger_(trigger_pin, 0), echo_(echo_pin), pulse_us_(0), distance_cm_(0),
      last_trigger_us_(0), echo_rise_us_(0), last_reading_us_(0),
      waiting_for_echo_(false), echo_high_(false), reading_valid_(false), triggered_(false)
{
    echo_.mode(PullDown);
    echo_.rise(callback(this, &RangeFinder::onRise));
    echo_.fall(callback(this, &RangeFinder::onFall));
    trigger_ = 0;
}

RangeFinder::~RangeFinder()
{
    echo_.rise(NULL);
    echo_.fall(NULL);
}

void RangeFinder::startMeasurement()
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_us = us_ticker_read();
    const uint32_t elapsed_us = now_us - last_trigger_us_;
    if (waiting_for_echo_ && elapsed_us >= echo_timeout_us) {
        waiting_for_echo_ = false;
        echo_high_ = false;
        reading_valid_ = false;
    }
    if (waiting_for_echo_ || (triggered_ && elapsed_us < measurement_period_us)) {
        __set_PRIMASK(interrupt_mask);
        return;
    }
    last_trigger_us_ = now_us;
    waiting_for_echo_ = true;
    echo_high_ = false;
    triggered_ = true;
    __set_PRIMASK(interrupt_mask);

    trigger_ = 1;
    wait_us(10);
    trigger_ = 0;
}

unsigned int RangeFinder::sample()
{
    startMeasurement();
    // Echo processing is asynchronous; return the latest completed measurement.
    return distanceCm();
}

void RangeFinder::onRise()
{
    const uint32_t now_us = us_ticker_read();
    if (waiting_for_echo_ && !echo_high_ && now_us - last_trigger_us_ < echo_timeout_us) {
        echo_rise_us_ = now_us;
        echo_high_ = true;
    }
}

void RangeFinder::onFall()
{
    if (!waiting_for_echo_ || !echo_high_) {
        return;
    }
    const uint32_t now_us = us_ticker_read();
    const uint32_t width_us = now_us - echo_rise_us_;
    waiting_for_echo_ = false;
    echo_high_ = false;
    reading_valid_ = width_us >= minimum_echo_us && width_us < echo_timeout_us &&
                     now_us - last_trigger_us_ < echo_timeout_us;
    if (reading_valid_) {
        pulse_us_ = width_us;
        distance_cm_ = (width_us * 343u) / 20000u;
        last_reading_us_ = now_us;
    }
}

unsigned int RangeFinder::distanceCm() const
{
    return distance_cm_;
}

unsigned int RangeFinder::pulseWidthUs() const
{
    return pulse_us_;
}

bool RangeFinder::hasValidReading() const
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const bool valid = reading_valid_ &&
        us_ticker_read() - last_reading_us_ < maximum_reading_age_us;
    __set_PRIMASK(interrupt_mask);
    return valid;
}

bool RangeFinder::isBelowCm(unsigned int limit_cm) const
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    // Compare the pulse before integer-cm truncation, using one coherent ISR snapshot.
    const bool below = reading_valid_ &&
        us_ticker_read() - last_reading_us_ < maximum_reading_age_us &&
        static_cast<uint64_t>(pulse_us_) * 343u < static_cast<uint64_t>(limit_cm) * 20000u;
    __set_PRIMASK(interrupt_mask);
    return below;
}

void RangeFinder::rise(void (*handler)())
{
    echo_.rise(handler);
}

void RangeFinder::fall(void (*handler)())
{
    echo_.fall(handler);
}

} // namespace quadcopter
