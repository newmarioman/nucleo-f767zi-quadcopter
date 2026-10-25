#include "imu.h"
#include "orientation_math.h"

namespace quadcopter {

Imu::Imu(Mpu6050& sensor, const ImuConfig& config)
    : sensor_(sensor), config_(config), calibration_{}, sample_{}
{
}

const ImuCalibrationStatus& Imu::calibrate(Serial& console, DigitalOut& error_led)
{
    calibration_ = ImuCalibrationStatus{};
    calibration_.device_identity = sensor_.identity();
    calibration_.identity_ok = calibration_.device_identity == 0x68;
    console.printf("Address: 0x%x\n\r", calibration_.device_identity);

    if (calibration_.identity_ok) {
        console.printf("MPU6050 online");
        sensor_.selfTest(calibration_.self_test);
        wait(0.5);
        calibration_.self_test_passed = true;
        for (unsigned int index = 0; index < 6; ++index) {
            if (!(calibration_.self_test[index] < config_.self_test_upper_limit)) {
                calibration_.self_test_passed = false;
            }
        }

        if (calibration_.self_test_passed) {
            sensor_.reset();
            calibration_.fifo_calibration_succeeded = sensor_.calibrate(
                calibration_.hardware_gyro_bias_dps, calibration_.accelerometer_bias_g);
            if (!calibration_.fifo_calibration_succeeded) {
                console.printf("MPU6050 calibration FIFO is empty\n\r");
                error_led = !error_led;
            }
            calibration_.configuration_succeeded = sensor_.initialize();
            if (!calibration_.configuration_succeeded) {
                console.printf("MPU6050 full-function configuration failed\n\r");
                error_led = !error_led;
                return calibration_;
            }
            console.printf("MPU6050 full-function: PLL, all axes, raw 1 kHz, gyro DLPF 188 Hz\n\r");
            wait(0.5);
        } else {
            console.printf("Sensor self-test failed\n\r");
            error_led = !error_led;
        }
    } else {
        console.printf("Sensor communication failed\n\r");
        console.printf("%#x \n", calibration_.device_identity);
        error_led = !error_led;
    }

    // One-time data-ready gate and startup continuation.
    if (calibration_.configuration_succeeded && sensor_.dataReady()) {
        calibrateSoftwareGyroBias();
        read();
        calibration_.initial_sample_available = true;
        console.printf("%f;%f;%f;%f;%f;%f\n\r",
                       sample_.acceleration_g.x, sample_.acceleration_g.y,
                       sample_.acceleration_g.z, sample_.angular_velocity_dps.x,
                       sample_.angular_velocity_dps.y, sample_.angular_velocity_dps.z);
    }
    return calibration_;
}

void Imu::calibrateSoftwareGyroBias()
{
    Vector3& bias = calibration_.software_gyro_bias_dps;
    const float gyro_scale = sensor_.gyroscopeResolution();
    for (int index = 0; index < config_.gyro_bias_samples; ++index) {
        int16_t raw[3] = {};
        sensor_.readGyroscope(raw);
        const Vector3 native_rates = {
            static_cast<float>(raw[0]) * gyro_scale,
            static_cast<float>(raw[1]) * gyro_scale,
            static_cast<float>(raw[2]) * gyro_scale
        };
        const Vector3 body_rates = orientation_math::sensorToBody(native_rates);
        bias.x += body_rates.x;
        bias.y += body_rates.y;
        bias.z += body_rates.z;
        wait_ms(config_.gyro_bias_delay_ms);
    }
    if (config_.gyro_bias_samples > 0) {
        bias.x /= config_.gyro_bias_samples;
        bias.y /= config_.gyro_bias_samples;
        bias.z /= config_.gyro_bias_samples;
    }
}

const ImuSample& Imu::read()
{
    int16_t acceleration[3] = {};
    int16_t gyroscope[3] = {};
    sensor_.readMotion(acceleration, gyroscope);

    // All backends follow the actual configured ranges, including DMP range changes.
    const float acceleration_scale = sensor_.accelerometerResolution();
    const float gyro_scale = sensor_.gyroscopeResolution();

    const Vector3 native_acceleration = {
        static_cast<float>(acceleration[0]) * acceleration_scale,
        static_cast<float>(acceleration[1]) * acceleration_scale,
        static_cast<float>(acceleration[2]) * acceleration_scale
    };
    const Vector3 native_rates = {
        static_cast<float>(gyroscope[0]) * gyro_scale,
        static_cast<float>(gyroscope[1]) * gyro_scale,
        static_cast<float>(gyroscope[2]) * gyro_scale
    };
    sample_.acceleration_g = orientation_math::sensorToBody(native_acceleration);
    sample_.angular_velocity_dps = orientation_math::sensorToBody(native_rates);
    sample_.angular_velocity_dps.x -= calibration_.software_gyro_bias_dps.x;
    sample_.angular_velocity_dps.y -= calibration_.software_gyro_bias_dps.y;
    sample_.angular_velocity_dps.z -= calibration_.software_gyro_bias_dps.z;
    return sample_;
}

const ImuSample& Imu::sample() const
{
    return sample_;
}

const ImuCalibrationStatus& Imu::calibrationStatus() const
{
    return calibration_;
}

} // namespace quadcopter
