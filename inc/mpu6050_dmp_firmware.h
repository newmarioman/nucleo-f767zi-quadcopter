#ifndef QUADCOPTER_MPU6050_DMP_FIRMWARE_H
#define QUADCOPTER_MPU6050_DMP_FIRMWARE_H

#include <stdint.h>

namespace quadcopter {
namespace mpu6050_dmp_firmware {

constexpr unsigned int kImageSize = 3062;
constexpr unsigned int kPacketSize = 28;
extern const uint8_t kImage[kImageSize];

} // namespace mpu6050_dmp_firmware
} // namespace quadcopter

#endif
