#ifndef QUADCOPTER_IMU_H
#define QUADCOPTER_IMU_H

#include "mbed.h"
#include "flight_config.h"
#include "flight_types.h"
#include "mpu6050.h"

namespace quadcopter {

struct ImuCalibrationStatus {
    uint8_t device_identity;
    bool identity_ok;
    bool self_test_passed;
    bool fifo_calibration_succeeded;
    bool configuration_succeeded;
    bool initial_sample_available;
    float self_test[6];
    float hardware_gyro_bias_dps[3]; // Native MPU6050 axes.
    float accelerometer_bias_g[3]; // Native MPU6050 axes; reported only.
    Vector3 software_gyro_bias_dps; // Body FRD axes, like ImuSample rates.
};

class Imu {
public:
    Imu(Mpu6050& sensor, const ImuConfig& config);

    const ImuCalibrationStatus& calibrate(Serial& console, DigitalOut& error_led);
    const ImuSample& read();
    const ImuSample& sample() const;
    const ImuCalibrationStatus& calibrationStatus() const;

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

private:
    void calibrateSoftwareGyroBias();

    Mpu6050& sensor_;
    ImuConfig config_;
    ImuCalibrationStatus calibration_;
    ImuSample sample_;
};

} // namespace quadcopter

#endif
