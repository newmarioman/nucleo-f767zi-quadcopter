#include "mpu6050.h"
#include "mpu6050_registers.h"

#include <cmath>

namespace quadcopter {

using namespace mpu6050_registers;

namespace {

int16_t signedWord(uint8_t high, uint8_t low)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

} // namespace

Mpu6050::Mpu6050(I2C& bus, uint8_t address, GyroscopeScale gyro_scale,
                 AccelerometerScale accel_scale)
    : bus_(bus), address_(address), gyro_scale_(gyro_scale), accel_scale_(accel_scale)
{
}

uint8_t Mpu6050::identity()
{
    return readByte(address_, WHO_AM_I_MPU6050);
}

bool Mpu6050::dataReady()
{
    return (readByte(address_, INT_STATUS) & 0x01) != 0;
}

void Mpu6050::writeByte(uint8_t address, uint8_t sub_address, uint8_t value)
{
    const char data[2] = {static_cast<char>(sub_address), static_cast<char>(value)};
    bus_.write(address, data, 2, false);
}

uint8_t Mpu6050::readByte(uint8_t address, uint8_t sub_address)
{
    char value = 0;
    const char command = static_cast<char>(sub_address);
    bus_.write(address, &command, 1, true);
    bus_.read(address, &value, 1, false);
    return static_cast<uint8_t>(value);
}

void Mpu6050::readBytes(uint8_t address, uint8_t sub_address, uint8_t count,
                       uint8_t* destination)
{
    const char command = static_cast<char>(sub_address);
    for (uint8_t index = 0; index < count; ++index) {
        destination[index] = 0;
    }
    bus_.write(address, &command, 1, true);
    // Read into the caller's buffer, without the original 14-byte temporary limit.
    bus_.read(address, reinterpret_cast<char*>(destination), count, false);
}

float Mpu6050::gyroscopeResolution() const
{
    switch (gyro_scale_) {
    case GyroscopeScale::Dps250:  return 250.0 / 32768.0;
    case GyroscopeScale::Dps500:  return 500.0 / 32768.0;
    case GyroscopeScale::Dps1000: return 1000.0 / 32768.0;
    case GyroscopeScale::Dps2000: return 2000.0 / 32768.0;
    }
    return 0;
}

float Mpu6050::accelerometerResolution() const
{
    switch (accel_scale_) {
    case AccelerometerScale::G2:  return 2.0 / 32768.0;
    case AccelerometerScale::G4:  return 4.0 / 32768.0;
    case AccelerometerScale::G8:  return 8.0 / 32768.0;
    case AccelerometerScale::G16: return 16.0 / 32768.0;
    }
    return 0;
}

void Mpu6050::readVector(uint8_t start_register, int16_t* destination)
{
    uint8_t raw[6] = {};
    readBytes(address_, start_register, 6, raw);
    for (unsigned int axis = 0; axis < 3; ++axis) {
        destination[axis] = signedWord(raw[axis * 2], raw[axis * 2 + 1]);
    }
}

void Mpu6050::readAcceleration(int16_t* destination)
{
    readVector(ACCEL_XOUT_H, destination);
}

void Mpu6050::readGyroscope(int16_t* destination)
{
    readVector(GYRO_XOUT_H, destination);
}

void Mpu6050::readMotion(int16_t* acceleration, int16_t* gyroscope)
{
    // One burst captures a coherent accel/temperature/gyro register snapshot.
    // Two separate six-byte reads could straddle a sensor update at 1 kHz.
    uint8_t raw[14] = {};
    readBytes(address_, ACCEL_XOUT_H, sizeof(raw), raw);
    for (unsigned int axis = 0; axis < 3; ++axis) {
        acceleration[axis] = signedWord(raw[axis * 2], raw[axis * 2 + 1]);
        gyroscope[axis] = signedWord(raw[axis * 2 + 8], raw[axis * 2 + 9]);
    }
}

int16_t Mpu6050::readTemperatureRaw()
{
    uint8_t raw[2] = {};
    readBytes(address_, TEMP_OUT_H, 2, raw);
    return signedWord(raw[0], raw[1]);
}

void Mpu6050::reset()
{
    writeByte(address_, PWR_MGMT_1, 0x80);
    wait(0.1);
}

bool Mpu6050::configureFullFunction(uint8_t sample_rate_divider)
{
    // SLEEP=0, CYCLE=0, TEMP_DIS=0, CLKSEL=1 (X-gyro PLL).
    // Clear every accelerometer/gyro standby bit and the low-power wake rate.
    if (!writeRegister(PWR_MGMT_1, 0x01) || !writeRegister(PWR_MGMT_2, 0x00)) {
        return false;
    }
    wait_ms(100); // Allow gyro startup and PLL settling before acquiring data.
    // DLPF_CFG=1: gyro 188 Hz / 1.9 ms, accel 184 Hz / 2.0 ms; FSYNC off.
    if (!writeRegister(CONFIG, 0x01) || !writeRegister(SMPLRT_DIV, sample_rate_divider)) {
        return false;
    }

    uint8_t power[2] = {};
    uint8_t timing[2] = {};
    return readRegisterBlock(PWR_MGMT_1, power, 2) && power[0] == 0x01 && power[1] == 0x00 &&
           readRegisterBlock(SMPLRT_DIV, timing, 2) &&
           timing[0] == sample_rate_divider && timing[1] == 0x01;
}

bool Mpu6050::initialize()
{
    // Leave neither the calibration FIFO nor a previously enabled DMP running.
    return writeRegister(INT_ENABLE, 0x00) && writeRegister(FIFO_EN, 0x00) &&
           writeRegister(USER_CTRL, 0x00) && writeRegister(USER_CTRL, 0x04) &&
           configureFullFunction() && configureRanges(accel_scale_, gyro_scale_) &&
           writeRegister(INT_PIN_CFG, 0x22) && writeRegister(INT_ENABLE, 0x01);
}

bool Mpu6050::calibrate(float* gyro_bias_dps, float* accel_bias_g)
{
    uint8_t data[12] = {};
    int32_t gyro_bias[3] = {};
    int32_t accel_bias[3] = {};

    writeByte(address_, PWR_MGMT_1, 0x80);
    wait(0.1);
    writeByte(address_, PWR_MGMT_1, 0x01);
    writeByte(address_, PWR_MGMT_2, 0x00);
    wait(0.2);

    writeByte(address_, INT_ENABLE, 0x00);
    writeByte(address_, FIFO_EN, 0x00);
    writeByte(address_, PWR_MGMT_1, 0x01); // Keep the gyro PLL during bias calibration.
    writeByte(address_, I2C_MST_CTRL, 0x00);
    writeByte(address_, USER_CTRL, 0x00);
    writeByte(address_, USER_CTRL, 0x0C);
    wait(0.015);

    writeByte(address_, CONFIG, 0x01);
    writeByte(address_, SMPLRT_DIV, 0x00);
    writeByte(address_, GYRO_CONFIG, 0x00);
    writeByte(address_, ACCEL_CONFIG, 0x00);
    const uint16_t gyro_sensitivity = 131;
    const uint16_t accel_sensitivity = 16384;

    writeByte(address_, USER_CTRL, 0x40);
    writeByte(address_, FIFO_EN, 0x78);
    wait(0.08);
    writeByte(address_, FIFO_EN, 0x00);
    readBytes(address_, FIFO_COUNTH, 2, data);
    const uint16_t fifo_count = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t packet_count = fifo_count / 12;

    // An empty FIFO has no samples to average; avoid division by zero.
    if (packet_count == 0) {
        for (unsigned int axis = 0; axis < 3; ++axis) {
            gyro_bias_dps[axis] = 0;
            accel_bias_g[axis] = 0;
        }
        return false;
    }

    for (uint16_t packet = 0; packet < packet_count; ++packet) {
        readBytes(address_, FIFO_R_W, 12, data);
        for (unsigned int axis = 0; axis < 3; ++axis) {
            accel_bias[axis] += signedWord(data[axis * 2], data[axis * 2 + 1]);
            gyro_bias[axis] += signedWord(data[axis * 2 + 6], data[axis * 2 + 7]);
        }
    }
    for (unsigned int axis = 0; axis < 3; ++axis) {
        accel_bias[axis] /= packet_count;
        gyro_bias[axis] /= packet_count;
    }
    if (accel_bias[2] > 0) {
        accel_bias[2] -= accel_sensitivity;
    } else {
        accel_bias[2] += accel_sensitivity;
    }

    // Hardware gyro offsets use the original sign and divide-by-four scaling.
    for (unsigned int axis = 0; axis < 3; ++axis) {
        data[axis * 2] = ((-gyro_bias[axis] / 4) >> 8) & 0xFF;
        data[axis * 2 + 1] = (-gyro_bias[axis] / 4) & 0xFF;
    }
    writeByte(address_, XG_OFFS_USRH, data[0]);
    writeByte(address_, XG_OFFS_USRL, data[1]);
    writeByte(address_, YG_OFFS_USRH, data[2]);
    writeByte(address_, YG_OFFS_USRL, data[3]);
    writeByte(address_, ZG_OFFS_USRH, data[4]);
    writeByte(address_, ZG_OFFS_USRL, data[5]);
    for (unsigned int axis = 0; axis < 3; ++axis) {
        gyro_bias_dps[axis] = static_cast<float>(gyro_bias[axis]) / gyro_sensitivity;
    }

    // Preserve the factory-trim reads and temperature-bit calculations.
    int32_t factory_trim[3] = {};
    readBytes(address_, XA_OFFSET_H, 2, data);
    factory_trim[0] = signedWord(data[0], data[1]);
    readBytes(address_, YA_OFFSET_H, 2, data);
    factory_trim[1] = signedWord(data[0], data[1]);
    readBytes(address_, ZA_OFFSET_H, 2, data);
    factory_trim[2] = signedWord(data[0], data[1]);
    for (unsigned int axis = 0; axis < 3; ++axis) {
        const uint8_t temperature_bit = factory_trim[axis] & 0x01;
        factory_trim[axis] -= accel_bias[axis] / 8;
        data[axis * 2] = (factory_trim[axis] >> 8) & 0xFF;
        data[axis * 2 + 1] = (factory_trim[axis] & 0xFF) | temperature_bit;
        accel_bias_g[axis] = static_cast<float>(accel_bias[axis]) / accel_sensitivity;
    }
    // Accelerometer hardware-offset writes remain disabled.
    return true;
}

void Mpu6050::selfTest(float* destination)
{
    uint8_t raw[4] = {};
    uint8_t self_test[6] = {};
    float factory_trim[6] = {};

    writeByte(address_, ACCEL_CONFIG, 0xF0);
    writeByte(address_, GYRO_CONFIG, 0xE0);
    wait(0.25);
    raw[0] = readByte(address_, SELF_TEST_X);
    raw[1] = readByte(address_, SELF_TEST_Y);
    raw[2] = readByte(address_, SELF_TEST_Z);
    raw[3] = readByte(address_, SELF_TEST_A);

    // Retain the supplied bit extraction and result formula without retuning.
    self_test[0] = (raw[0] >> 3) | ((raw[3] & 0x30) >> 4);
    self_test[1] = (raw[1] >> 3) | ((raw[3] & 0x0C) >> 4);
    self_test[2] = (raw[2] >> 3) | ((raw[3] & 0x03) >> 4);
    self_test[3] = raw[0] & 0x1F;
    self_test[4] = raw[1] & 0x1F;
    self_test[5] = raw[2] & 0x1F;

    for (unsigned int axis = 0; axis < 3; ++axis) {
        factory_trim[axis] = (4096.0f * 0.34f) *
            std::pow(0.92f / 0.34f, (self_test[axis] - 1.0f) / 30.0f);
    }
    factory_trim[3] = (25.0f * 131.0f) * std::pow(1.046f, self_test[3] - 1.0f);
    factory_trim[4] = (-25.0f * 131.0f) * std::pow(1.046f, self_test[4] - 1.0f);
    factory_trim[5] = (25.0f * 131.0f) * std::pow(1.046f, self_test[5] - 1.0f);
    for (unsigned int index = 0; index < 6; ++index) {
        destination[index] = 100.0f + 100.0f *
            (self_test[index] - factory_trim[index]) / factory_trim[index];
    }
}

bool Mpu6050::readRegister(uint8_t sub_address, uint8_t& value)
{
    return readRegisterBlock(sub_address, &value, 1);
}

bool Mpu6050::writeRegister(uint8_t sub_address, uint8_t value)
{
    return writeRegisterBlock(sub_address, &value, 1);
}

bool Mpu6050::readRegisterBlock(uint8_t sub_address, uint8_t* destination, uint8_t count)
{
    if (destination == NULL || count == 0) {
        return false;
    }
    for (uint8_t index = 0; index < count; ++index) {
        destination[index] = 0;
    }
    const char command = static_cast<char>(sub_address);
    if (bus_.write(address_, &command, 1, true) != 0) {
        return false;
    }
    return bus_.read(address_, reinterpret_cast<char*>(destination), count, false) == 0;
}

bool Mpu6050::writeRegisterBlock(uint8_t sub_address, const uint8_t* source, uint8_t count)
{
    if (source == NULL || count == 0 || count > 16) {
        return false;
    }
    char data[17] = {};
    data[0] = static_cast<char>(sub_address);
    for (uint8_t index = 0; index < count; ++index) {
        data[index + 1] = static_cast<char>(source[index]);
    }
    return bus_.write(address_, data, count + 1, false) == 0;
}

bool Mpu6050::configureRanges(AccelerometerScale accel_scale, GyroscopeScale gyro_scale)
{
    // Explicit writes clear self-test and the legacy accel high-pass setting.
    // The old repeated writes OR-ed the requested range into stale register bits.
    const uint8_t accel_config = static_cast<uint8_t>(accel_scale) << 3;
    const uint8_t gyro_config = static_cast<uint8_t>(gyro_scale) << 3;
    if (!writeRegister(ACCEL_CONFIG, accel_config) || !writeRegister(GYRO_CONFIG, gyro_config)) {
        return false;
    }
    uint8_t readback[2] = {};
    if (!readRegisterBlock(GYRO_CONFIG, readback, 2) ||
        readback[0] != gyro_config || readback[1] != accel_config) {
        return false;
    }
    accel_scale_ = accel_scale;
    gyro_scale_ = gyro_scale;
    return true;
}

} // namespace quadcopter
