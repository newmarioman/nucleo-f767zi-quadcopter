#ifndef QUADCOPTER_MPU6050_H
#define QUADCOPTER_MPU6050_H

#include "mbed.h"
#include <stdint.h>

namespace quadcopter {

class Mpu6050 {
public:
    enum class AccelerometerScale : uint8_t { G2 = 0, G4, G8, G16 };
    enum class GyroscopeScale : uint8_t { Dps250 = 0, Dps500, Dps1000, Dps2000 };

    explicit Mpu6050(I2C& bus, uint8_t address = (0x68 << 1),
                     GyroscopeScale gyro_scale = GyroscopeScale::Dps2000,
                     AccelerometerScale accel_scale = AccelerometerScale::G8);

    uint8_t identity();
    bool dataReady();
    void reset();
    bool initialize();
    // Continuous six-axis operation, PLL clock, temperature enabled, 188 Hz gyro DLPF.
    // Divider 0 gives 1 kHz raw data; MotionApps 6.12 requires divider 4 (200 Hz).
    bool configureFullFunction(uint8_t sample_rate_divider = 0);
    bool calibrate(float* gyro_bias, float* accel_bias);
    void selfTest(float* destination);
    void readAcceleration(int16_t* destination);
    void readGyroscope(int16_t* destination);
    void readMotion(int16_t* acceleration, int16_t* gyroscope);
    int16_t readTemperatureRaw();
    float gyroscopeResolution() const;
    float accelerometerResolution() const;

    void writeByte(uint8_t address, uint8_t sub_address, uint8_t value);
    uint8_t readByte(uint8_t address, uint8_t sub_address);
    void readBytes(uint8_t address, uint8_t sub_address, uint8_t count, uint8_t* destination);

    // Checked operations use this instance's address and report Mbed I2C failures.
    bool readRegister(uint8_t sub_address, uint8_t& value);
    bool writeRegister(uint8_t sub_address, uint8_t value);
    bool readRegisterBlock(uint8_t sub_address, uint8_t* destination, uint8_t count);
    // Writes are limited to 16 payload bytes, matching the DMP transfer chunks.
    bool writeRegisterBlock(uint8_t sub_address, const uint8_t* source, uint8_t count);
    bool configureRanges(AccelerometerScale accel_scale, GyroscopeScale gyro_scale);

    Mpu6050(const Mpu6050&) = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;

private:
    void readVector(uint8_t start_register, int16_t* destination);

    I2C& bus_;
    uint8_t address_;
    GyroscopeScale gyro_scale_;
    AccelerometerScale accel_scale_;
};

} // namespace quadcopter

#endif
