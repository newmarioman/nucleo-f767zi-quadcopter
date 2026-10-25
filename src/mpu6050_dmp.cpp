#include "fusion_selection.h"
#include "mpu6050_dmp.h"

#if SENSOR_FUSION_MODE == FUSION_DMP

#include "mpu6050_dmp_firmware.h"
#include "mpu6050_registers.h"
#include "orientation_math.h"

#include <cmath>
#include <cstring>

// Initialization and memory image adapted from Jeff Rowberg's i2cdevlib.
// See THIRD_PARTY_NOTICES.md for the pinned source and MIT license.
namespace quadcopter {

using namespace mpu6050_registers;

namespace {

float quaternionWord(const uint8_t* source)
{
    const uint32_t bits = (static_cast<uint32_t>(source[0]) << 24) |
                          (static_cast<uint32_t>(source[1]) << 16) |
                          (static_cast<uint32_t>(source[2]) << 8) | source[3];
    return static_cast<int32_t>(bits) / 1073741824.0f;
}

} // namespace

Mpu6050Dmp::Mpu6050Dmp(Mpu6050& sensor, const FusionConfig& config)
    : sensor_(sensor), config_(config), quaternion_{1.0f, 0.0f, 0.0f, 0.0f},
      status_(Status::Uninitialized), initialized_(false), have_packet_(false),
      overflows_(0), packets_(0)
{
}

bool Mpu6050Dmp::fail(Status status)
{
    status_ = status;
    have_packet_ = false;
    return false;
}

bool Mpu6050Dmp::begin()
{
    initialized_ = false;
    have_packet_ = false;
    status_ = Status::Uninitialized;
    if (config_.dmp_startup_timeout_ms <= 0 || config_.dmp_stale_timeout_ms <= 0) {
        return fail(Status::InvalidConfiguration);
    }

    uint8_t identity = 0;
    uint8_t gyro_offsets[6] = {};
    if (!sensor_.readRegister(WHO_AM_I_MPU6050, identity)) {
        return fail(Status::IoError);
    }
    if (identity != 0x68) {
        return fail(Status::WrongIdentity);
    }
    // A device reset must not discard the gyro offsets produced by startup calibration.
    if (!sensor_.readRegisterBlock(XG_OFFS_USRH, gyro_offsets, 6) ||
        !sensor_.writeRegister(PWR_MGMT_1, 0x80)) {
        return fail(Status::IoError);
    }
    wait_ms(100);
    if (!sensor_.writeRegister(USER_CTRL, 0x07)) {
        return fail(Status::IoError);
    }
    wait_ms(100);

    // Use the same continuous full-function power/PLL/filter policy as raw fusion.
    // Keep MotionApps 6.12's 200 Hz input and firmware-specific ranges: changing
    // its input to 1 kHz without matching firmware changes invalidates its timing.
    if (!sensor_.configureFullFunction(0x04) ||
        !sensor_.writeRegister(INT_ENABLE, 0x00) ||
        !sensor_.writeRegister(FIFO_EN, 0x00) ||
        !sensor_.writeRegister(ACCEL_CONFIG, 0x00) ||
        !sensor_.writeRegister(INT_PIN_CFG, 0x80)) {
        return fail(Status::IoError);
    }
    if (!loadFirmware()) {
        return false;
    }

    const uint8_t start_address[2] = {0x04, 0x00};
    if (!sensor_.writeRegisterBlock(DMP_REG_1, start_address, 2) ||
        !sensor_.configureRanges(Mpu6050::AccelerometerScale::G2,
                                 Mpu6050::GyroscopeScale::Dps2000) ||
        !sensor_.writeRegisterBlock(XG_OFFS_USRH, gyro_offsets, 6) ||
        !sensor_.writeRegister(USER_CTRL, 0xC0) ||
        !sensor_.writeRegister(INT_ENABLE, 0x12) ||
        !sensor_.writeRegister(USER_CTRL, 0xC4) ||
        !sensor_.writeRegister(USER_CTRL, 0x40)) {
        return fail(Status::IoError);
    }

    initialized_ = true;
    if (!synchronize()) {
        initialized_ = false;
        sensor_.writeRegister(USER_CTRL, 0x00);
        return false;
    }
    return true;
}

bool Mpu6050Dmp::loadFirmware()
{
    const unsigned int chunk_size = 16;
    uint8_t readback[chunk_size] = {};
    for (unsigned int offset = 0; offset < mpu6050_dmp_firmware::kImageSize;) {
        const uint8_t bank = offset / 256;
        const uint8_t address = offset % 256;
        unsigned int count = mpu6050_dmp_firmware::kImageSize - offset;
        if (count > chunk_size) {
            count = chunk_size;
        }
        if (count > 256u - address) {
            count = 256u - address;
        }
        const uint8_t length = static_cast<uint8_t>(count);
        const uint8_t* source = &mpu6050_dmp_firmware::kImage[offset];
        if (!sensor_.writeRegister(DMP_BANK, bank) ||
            !sensor_.writeRegister(DMP_RW_PNT, address) ||
            !sensor_.writeRegisterBlock(DMP_REG, source, length) ||
            !sensor_.writeRegister(DMP_BANK, bank) ||
            !sensor_.writeRegister(DMP_RW_PNT, address) ||
            !sensor_.readRegisterBlock(DMP_REG, readback, length)) {
            return fail(Status::IoError);
        }
        if (std::memcmp(source, readback, count) != 0) {
            return fail(Status::FirmwareVerificationFailed);
        }
        offset += count;
    }
    return true;
}

bool Mpu6050Dmp::resetFifo()
{
    // Keep DMP/FIFO enabled while pulsing the FIFO-reset bit, as upstream does.
    if (!sensor_.writeRegister(USER_CTRL, 0xC4)) {
        return fail(Status::IoError);
    }
    uint8_t status = 0;
    if (!sensor_.readRegister(INT_STATUS, status)) {
        return fail(Status::IoError);
    }
    return true;
}

bool Mpu6050Dmp::synchronize()
{
    if (!initialized_) {
        return false;
    }
    have_packet_ = false;
    if (!resetFifo()) {
        return false;
    }
    packet_age_.stop();
    packet_age_.reset();
    packet_age_.start();
    status_ = Status::WaitingForPacket;
    return waitForPacket();
}

bool Mpu6050Dmp::waitForPacket()
{
    Timer deadline;
    deadline.start();
    Quaternion candidate = {};
    while (deadline.read_ms() < config_.dmp_startup_timeout_ms) {
        if (readLatest(candidate)) {
            return true;
        }
        if (status_ == Status::IoError || status_ == Status::InvalidQuaternion) {
            return false;
        }
        wait_ms(1);
    }
    return fail(Status::StartupTimeout);
}

bool Mpu6050Dmp::readLatest(Quaternion& quaternion)
{
    if (!initialized_) {
        return false;
    }
    uint8_t interrupt_status = 0;
    uint8_t fifo_bytes[2] = {};
    if (!sensor_.readRegister(INT_STATUS, interrupt_status) ||
        !sensor_.readRegisterBlock(FIFO_COUNTH, fifo_bytes, 2)) {
        return fail(Status::IoError);
    }
    const unsigned int fifo_count = (static_cast<unsigned int>(fifo_bytes[0]) << 8) | fifo_bytes[1];
    if ((interrupt_status & 0x10) != 0 || fifo_count >= 1024) {
        ++overflows_;
        if (!resetFifo()) {
            return false;
        }
        return fail(Status::FifoOverflow);
    }
    const unsigned int complete_packets = fifo_count / mpu6050_dmp_firmware::kPacketSize;
    if (complete_packets == 0) {
        if (have_packet_ && packet_age_.read_ms() > config_.dmp_stale_timeout_ms) {
            return fail(Status::DataExpired);
        }
        if (status_ != Status::DataExpired) {
            status_ = Status::WaitingForPacket;
        }
        return false;
    }

    // Drain only the complete packets from this count snapshot; work is bounded by FIFO size.
    uint8_t packet[mpu6050_dmp_firmware::kPacketSize] = {};
    for (unsigned int index = 0; index < complete_packets; ++index) {
        if (!sensor_.readRegisterBlock(FIFO_R_W, packet, sizeof(packet))) {
            // A partial I2C transfer may have consumed an unknown FIFO byte count.
            sensor_.writeRegister(USER_CTRL, 0xC4);
            return fail(Status::IoError);
        }
    }
    Quaternion candidate = {
        quaternionWord(&packet[0]), quaternionWord(&packet[4]),
        quaternionWord(&packet[8]), quaternionWord(&packet[12])
    };
    const float squared_norm = candidate.w * candidate.w + candidate.x * candidate.x +
                               candidate.y * candidate.y + candidate.z * candidate.z;
    if (!std::isfinite(squared_norm) || squared_norm < 0.5f || squared_norm > 1.5f ||
        !orientation_math::normalize(candidate)) {
        if (!resetFifo()) {
            return false;
        }
        return fail(Status::InvalidQuaternion);
    }
    quaternion_ = candidate;
    quaternion = candidate;
    packets_ += complete_packets;
    have_packet_ = true;
    packet_age_.reset();
    status_ = Status::Ready;
    return true;
}

bool Mpu6050Dmp::healthy() const
{
    const bool usable_status = status_ == Status::Ready || status_ == Status::WaitingForPacket;
    return initialized_ && have_packet_ && usable_status &&
           packet_age_.read_ms() >= 0 && packet_age_.read_ms() <= config_.dmp_stale_timeout_ms;
}

const Quaternion& Mpu6050Dmp::quaternion() const
{
    return quaternion_;
}

Mpu6050Dmp::Status Mpu6050Dmp::status() const
{
    return status_;
}

const char* Mpu6050Dmp::diagnostic() const
{
    if (initialized_ && have_packet_ &&
        packet_age_.read_ms() > config_.dmp_stale_timeout_ms) {
        return "DMP quaternion data expired";
    }
    switch (status_) {
    case Status::Uninitialized: return "DMP not initialized";
    case Status::Ready: return "DMP ready";
    case Status::WaitingForPacket: return "DMP waiting for a complete FIFO packet";
    case Status::InvalidConfiguration: return "Invalid DMP timeout configuration";
    case Status::WrongIdentity: return "Unexpected MPU6050 identity";
    case Status::IoError: return "DMP I2C operation failed";
    case Status::FirmwareVerificationFailed: return "DMP firmware readback mismatch";
    case Status::FifoOverflow: return "DMP FIFO overflow; FIFO reset";
    case Status::InvalidQuaternion: return "Invalid DMP quaternion; FIFO reset";
    case Status::DataExpired: return "DMP quaternion data expired";
    case Status::StartupTimeout: return "DMP startup packet timeout";
    }
    return "Unknown DMP status";
}

uint32_t Mpu6050Dmp::overflowCount() const
{
    return overflows_;
}

uint32_t Mpu6050Dmp::packetCount() const
{
    return packets_;
}

} // namespace quadcopter

#endif
