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

#ifndef QUADCOPTER_RANGE_FINDER_H
#define QUADCOPTER_RANGE_FINDER_H

#include "mbed.h"

namespace quadcopter {

class RangeFinder {
public:
    RangeFinder(PinName trigger_pin, PinName echo_pin);
    ~RangeFinder();

    void startMeasurement();
    unsigned int sample();
    unsigned int distanceCm() const;
    unsigned int pulseWidthUs() const;
    bool hasValidReading() const;
    bool isBelowCm(unsigned int limit_cm) const;

    void rise(void (*handler)());
    void fall(void (*handler)());

    RangeFinder(const RangeFinder&) = delete;
    RangeFinder& operator=(const RangeFinder&) = delete;

private:
    void onRise();
    void onFall();

    DigitalOut trigger_;
    InterruptIn echo_;
    volatile unsigned int pulse_us_;
    volatile unsigned int distance_cm_;
    volatile uint32_t last_trigger_us_;
    volatile uint32_t echo_rise_us_;
    volatile uint32_t last_reading_us_;
    volatile bool waiting_for_echo_;
    volatile bool echo_high_;
    volatile bool reading_valid_;
    bool triggered_;
};

} // namespace quadcopter

#endif
