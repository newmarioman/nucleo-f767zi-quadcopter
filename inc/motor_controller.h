#ifndef QUADCOPTER_MOTOR_CONTROLLER_H
#define QUADCOPTER_MOTOR_CONTROLLER_H

#include "mbed.h"
#include "arming_control.h"
#include "board_config.h"
#include "flight_config.h"
#include "flight_types.h"
#include "motor_mixer.h"

namespace quadcopter {

class MotorController {
public:
    MotorController(const BoardConfig& board, const MotorConfig& config);

    void initializeEscs();
    void update(const RcFrame& frame, float throttle_us, const ControlOutput& control,
                bool ground_allows_gesture);
    void stop();
    void disarm();
    bool armed() const;
    const MotorPulses& mixedPulses() const;

    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

private:
    void configureOutputs();
    void write(const MotorPulses& pulses);
    void writeAll(double pulse_us);

    PwmOut front_left_;
    PwmOut front_right_;
    PwmOut rear_right_;
    PwmOut rear_left_;
    MotorConfig config_;
    MotorMixer mixer_;
    ArmingControl arming_;
    MotorPulses mixed_pulses_;
    bool outputs_configured_;
};

} // namespace quadcopter

#endif
