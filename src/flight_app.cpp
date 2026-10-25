#include "flight_app.h"
#include "hal/us_ticker_api.h"

namespace quadcopter {

FlightApp::FlightApp(const BoardConfig& board, const FlightConfig& config)
    : config_(config), green_led_(board.green_led, 0), blue_led_(board.blue_led, 0),
      red_led_(board.red_led, 0), indicators_(green_led_, blue_led_),
      battery_(board.battery_lvl, config.battery), console_(board.console_tx, board.console_rx),
      i2c_(board.i2c_sda, board.i2c_scl), receiver_(board.receiver, config.radio),
      sensor_(i2c_), imu_(sensor_, config.imu), estimator_(sensor_, config.imu, config.fusion),
      range_finder_(board.range_trigger, board.range_echo),
      flight_control_(config.control), motors_(board, config.motors),
#if ENABLE_SG90_SERVOS
      servos_(board.servo_first, board.servo_second, config.servos),
#endif
      previous_time_seconds_(0), cycle_seconds_(config.initial_cycle_seconds),
      distance_cm_(0), fusion_initialized_(false), fusion_fault_reported_(false)
{
}

void FlightApp::initialize()
{
    receiver_.begin();
    i2c_.frequency(config_.i2c_frequency_hz);
    console_.baud(config_.serial_baud);
    wait_ms(config_.startup_delay_ms);

    const ImuCalibrationStatus& calibration = imu_.calibrate(console_, blue_led_);
    console_.printf("Sensor fusion: %s\n\r", estimator_.name());
    fusion_initialized_ = calibration.configuration_succeeded &&
                          estimator_.initialize(imu_.sample());

    if (fusion_initialized_) {
        motors_.initializeEscs();
    } else {
        motors_.disarm();
    }
    cycle_timer_.start();
    wait_ms(config_.loop_warmup_ms);
    if (fusion_initialized_) {
        fusion_initialized_ = estimator_.synchronize();
    }
    if (!fusion_initialized_) {
        console_.printf("Sensor fusion startup failed: %s\n\r", estimator_.diagnostic());
        fusion_fault_reported_ = true;
        motors_.disarm();
    }
#if ENABLE_SG90_SERVOS
    servos_.center();
    servos_.enable();
#endif
    const uint32_t now_us = us_ticker_read();
    battery_.update(now_us);
    // Startup calibration may use blue; runtime indication belongs to the battery.
    indicators_.update(motors_.armed(), battery_.low(), now_us);
    previous_time_seconds_ = cycle_timer_.read();
}

void FlightApp::update()
{
    estimator_.update(imu_.read(), cycle_seconds_);
    distance_cm_ = range_finder_.sample();
    const bool ground_allows_gesture =
        range_finder_.isBelowCm(config_.motors.gesture_max_distance_cm);

    const RcFrame frame = receiver_.snapshot();
    const RcCommand command = receiver_.command(frame);
#if ENABLE_SG90_SERVOS
    if (config_.servos.follow_receiver) {
        servos_.updateFromReceiver(frame, config_.radio);
    }
#endif
    estimator_.applyYawRateDeadband(config_.control.yaw_rate_deadband_dps);
    if (fusion_initialized_ && estimator_.healthy()) {
        if (fusion_fault_reported_) {
            console_.printf("Sensor fusion recovered; an arming gesture is required\n\r");
            fusion_fault_reported_ = false;
        }
        const ControlOutput control = flight_control_.update(command, estimator_.attitude());
        motors_.update(frame, command.throttle_us, control, ground_allows_gesture);
    } else {
        motors_.disarm();
        flight_control_.reset();
        if (!fusion_fault_reported_) {
            console_.printf("Sensor fusion unavailable: %s\n\r", estimator_.diagnostic());
            fusion_fault_reported_ = true;
        }
    }

    const uint32_t now_us = us_ticker_read();
    battery_.update(now_us);
    indicators_.update(motors_.armed(), battery_.low(), now_us);
    red_led_ = !red_led_;
    // Preserve use of the previous iteration's measured duration and both timer reads.
    cycle_seconds_ = cycle_timer_.read() - previous_time_seconds_;
    previous_time_seconds_ = cycle_timer_.read();
}

void FlightApp::run()
{
    initialize();
    while (true) {
        update();
    }
}

} // namespace quadcopter
