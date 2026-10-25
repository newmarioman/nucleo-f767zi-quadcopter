#ifndef QUADCOPTER_FLIGHT_CONFIG_H
#define QUADCOPTER_FLIGHT_CONFIG_H

#include <stdint.h>

namespace quadcopter {

struct RadioConfig {
    int synchronization_min_us;
    int synchronization_max_us;
    int channel_max_us;
    int command_offset_us;
    int axis_min_us;
    int axis_max_us;
};

struct PidConfig {
    float kp;
    float ki;
    float kd;
    int integral_limit;
    int output_limit;
};

struct ControlConfig {
    // A single configuration, with separate controller state for each axis.
    // Kp, Ki and Kd are distinct values shared by roll and pitch respectively.
    PidConfig roll_pitch_rate;
    float roll_pitch_angle_gain;
    double roll_pitch_stick_divisor;
    int roll_pitch_rate_limit;
    float yaw_gain;
    int yaw_output_limit;
    double yaw_stick_divisor;
    float yaw_rate_deadband_dps;
    int stick_deadband_low_us;
    int stick_deadband_high_us;
};

struct ImuConfig {
    double degrees_per_radian;
    double rate_filter_memory;
    double rate_filter_input;
    float pitch_correction;
    float roll_correction;
    int gyro_bias_samples;
    int gyro_bias_delay_ms;
    float self_test_upper_limit;
};

struct MotorConfig {
    int pwm_period_us;
    int minimum_pulse_us;
    int maximum_pulse_us;
    float maximum_throttle_us;
    int gesture_low_us;
    int gesture_high_us;
    float disarm_throttle_us;
    int inhibit_threshold_us;
    int startup_high_ms;
    int startup_low_ms;
    unsigned int gesture_max_distance_cm;
};

struct BatteryConfig {
    float adc_reference_voltage;
    float divider_ratio;
    float low_voltage;
    uint32_t sample_period_us;
};

struct ServoConfig {
    int pwm_period_us;
    int minimum_pulse_us;
    int maximum_pulse_us;
    int neutral_pulse_us;
    bool follow_receiver;
};

struct FusionConfig {
    float madgwick_gyro_error_dps;
    int dmp_startup_timeout_ms;
    int dmp_stale_timeout_ms;
};

struct FlightConfig {
    FlightConfig();

    RadioConfig radio;
    ControlConfig control;
    ImuConfig imu;
    MotorConfig motors;
    FusionConfig fusion;
    BatteryConfig battery;
    ServoConfig servos;
    int i2c_frequency_hz;
    int serial_baud;
    int startup_delay_ms;
    int loop_warmup_ms;
    float initial_cycle_seconds;
};

} // namespace quadcopter

#endif
