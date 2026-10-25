#ifndef QUADCOPTER_FLIGHT_APP_H
#define QUADCOPTER_FLIGHT_APP_H

#include "mbed.h"
#include "attitude_estimator.h"
#include "board_config.h"
#include "battery_monitor.h"
#include "flight_config.h"
#include "flight_control.h"
#include "imu.h"
#include "motor_controller.h"
#include "mpu6050.h"
#include "range_finder.h"
#include "rc.h"
#include "servo_selection.h"
#include "status_indicators.h"
#if ENABLE_SG90_SERVOS
#include "servo_controller.h"
#endif

namespace quadcopter {

class FlightApp {
public:
    FlightApp(const BoardConfig& board, const FlightConfig& config);
    void run();

    FlightApp(const FlightApp&) = delete;
    FlightApp& operator=(const FlightApp&) = delete;

private:
    void initialize();
    void update();

    FlightConfig config_;
    DigitalOut green_led_;
    DigitalOut blue_led_;
    DigitalOut red_led_;
    StatusIndicators indicators_;
    BatteryMonitor battery_;
    Serial console_;
    I2C i2c_;
    RcReceiver receiver_;
    Mpu6050 sensor_;
    Imu imu_;
    AttitudeEstimator estimator_;
    RangeFinder range_finder_;
    FlightControl flight_control_;
    MotorController motors_;
#if ENABLE_SG90_SERVOS
    ServoController servos_;
#endif
    Timer cycle_timer_;
    float previous_time_seconds_;
    float cycle_seconds_;
    unsigned int distance_cm_;
    bool fusion_initialized_;
    bool fusion_fault_reported_;
};

} // namespace quadcopter

#endif
