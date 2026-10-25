#ifndef QUADCOPTER_MPU6050_DMP_H
#define QUADCOPTER_MPU6050_DMP_H

#include "mbed.h"
#include "flight_config.h"
#include "flight_types.h"
#include "mpu6050.h"

namespace quadcopter {

class Mpu6050Dmp {
public:
    enum class Status {
        Uninitialized,
        Ready,
        WaitingForPacket,
        InvalidConfiguration,
        WrongIdentity,
        IoError,
        FirmwareVerificationFailed,
        FifoOverflow,
        InvalidQuaternion,
        DataExpired,
        StartupTimeout
    };

    Mpu6050Dmp(Mpu6050& sensor, const FusionConfig& config);
    bool begin();
    bool synchronize();
    bool readLatest(Quaternion& quaternion);
    bool healthy() const;
    const Quaternion& quaternion() const;
    Status status() const;
    const char* diagnostic() const;
    uint32_t overflowCount() const;
    uint32_t packetCount() const;

    Mpu6050Dmp(const Mpu6050Dmp&) = delete;
    Mpu6050Dmp& operator=(const Mpu6050Dmp&) = delete;

private:
    bool loadFirmware();
    bool resetFifo();
    bool waitForPacket();
    bool fail(Status status);

    Mpu6050& sensor_;
    FusionConfig config_;
    mutable Timer packet_age_;
    Quaternion quaternion_;
    Status status_;
    bool initialized_;
    bool have_packet_;
    uint32_t overflows_;
    uint32_t packets_;
};

} // namespace quadcopter

#endif
