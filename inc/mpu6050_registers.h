#ifndef QUADCOPTER_MPU6050_REGISTERS_H
#define QUADCOPTER_MPU6050_REGISTERS_H

#include <stdint.h>

// Register definitions retained from the supplied MPU6050 driver.
namespace quadcopter {
namespace mpu6050_registers {

constexpr uint8_t XGOFFS_TC = 0x00; // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
constexpr uint8_t YGOFFS_TC = 0x01;
constexpr uint8_t ZGOFFS_TC = 0x02;
constexpr uint8_t X_FINE_GAIN = 0x03; // [7:0] fine gain
constexpr uint8_t Y_FINE_GAIN = 0x04;
constexpr uint8_t Z_FINE_GAIN = 0x05;
constexpr uint8_t XA_OFFSET_H = 0x06; // User-defined trim values for accelerometer
constexpr uint8_t XA_OFFSET_L_TC = 0x07;
constexpr uint8_t YA_OFFSET_H = 0x08;
constexpr uint8_t YA_OFFSET_L_TC = 0x09;
constexpr uint8_t ZA_OFFSET_H = 0x0A;
constexpr uint8_t ZA_OFFSET_L_TC = 0x0B;
constexpr uint8_t SELF_TEST_X = 0x0D;
constexpr uint8_t SELF_TEST_Y = 0x0E;
constexpr uint8_t SELF_TEST_Z = 0x0F;
constexpr uint8_t SELF_TEST_A = 0x10;
constexpr uint8_t XG_OFFS_USRH = 0x13;  // User-defined trim values for gyroscope; supported in MPU-6050?
constexpr uint8_t XG_OFFS_USRL = 0x14;
constexpr uint8_t YG_OFFS_USRH = 0x15;
constexpr uint8_t YG_OFFS_USRL = 0x16;
constexpr uint8_t ZG_OFFS_USRH = 0x17;
constexpr uint8_t ZG_OFFS_USRL = 0x18;
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t FF_THR = 0x1D;  // Free-fall
constexpr uint8_t FF_DUR = 0x1E;  // Free-fall
constexpr uint8_t MOT_THR = 0x1F;  // Motion detection threshold bits [7:0]
constexpr uint8_t MOT_DUR = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
constexpr uint8_t ZMOT_THR = 0x21;  // Zero-motion detection threshold bits [7:0]
constexpr uint8_t ZRMOT_DUR = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
constexpr uint8_t FIFO_EN = 0x23;
constexpr uint8_t I2C_MST_CTRL = 0x24;
constexpr uint8_t I2C_SLV0_ADDR = 0x25;
constexpr uint8_t I2C_SLV0_REG = 0x26;
constexpr uint8_t I2C_SLV0_CTRL = 0x27;
constexpr uint8_t I2C_SLV1_ADDR = 0x28;
constexpr uint8_t I2C_SLV1_REG = 0x29;
constexpr uint8_t I2C_SLV1_CTRL = 0x2A;
constexpr uint8_t I2C_SLV2_ADDR = 0x2B;
constexpr uint8_t I2C_SLV2_REG = 0x2C;
constexpr uint8_t I2C_SLV2_CTRL = 0x2D;
constexpr uint8_t I2C_SLV3_ADDR = 0x2E;
constexpr uint8_t I2C_SLV3_REG = 0x2F;
constexpr uint8_t I2C_SLV3_CTRL = 0x30;
constexpr uint8_t I2C_SLV4_ADDR = 0x31;
constexpr uint8_t I2C_SLV4_REG = 0x32;
constexpr uint8_t I2C_SLV4_DO = 0x33;
constexpr uint8_t I2C_SLV4_CTRL = 0x34;
constexpr uint8_t I2C_SLV4_DI = 0x35;
constexpr uint8_t I2C_MST_STATUS = 0x36;
constexpr uint8_t INT_PIN_CFG = 0x37;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t DMP_INT_STATUS = 0x39;  // Check DMP interrupt
constexpr uint8_t INT_STATUS = 0x3A;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t ACCEL_XOUT_L = 0x3C;
constexpr uint8_t ACCEL_YOUT_H = 0x3D;
constexpr uint8_t ACCEL_YOUT_L = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L = 0x40;
constexpr uint8_t TEMP_OUT_H = 0x41;
constexpr uint8_t TEMP_OUT_L = 0x42;
constexpr uint8_t GYRO_XOUT_H = 0x43;
constexpr uint8_t GYRO_XOUT_L = 0x44;
constexpr uint8_t GYRO_YOUT_H = 0x45;
constexpr uint8_t GYRO_YOUT_L = 0x46;
constexpr uint8_t GYRO_ZOUT_H = 0x47;
constexpr uint8_t GYRO_ZOUT_L = 0x48;
constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
constexpr uint8_t EXT_SENS_DATA_01 = 0x4A;
constexpr uint8_t EXT_SENS_DATA_02 = 0x4B;
constexpr uint8_t EXT_SENS_DATA_03 = 0x4C;
constexpr uint8_t EXT_SENS_DATA_04 = 0x4D;
constexpr uint8_t EXT_SENS_DATA_05 = 0x4E;
constexpr uint8_t EXT_SENS_DATA_06 = 0x4F;
constexpr uint8_t EXT_SENS_DATA_07 = 0x50;
constexpr uint8_t EXT_SENS_DATA_08 = 0x51;
constexpr uint8_t EXT_SENS_DATA_09 = 0x52;
constexpr uint8_t EXT_SENS_DATA_10 = 0x53;
constexpr uint8_t EXT_SENS_DATA_11 = 0x54;
constexpr uint8_t EXT_SENS_DATA_12 = 0x55;
constexpr uint8_t EXT_SENS_DATA_13 = 0x56;
constexpr uint8_t EXT_SENS_DATA_14 = 0x57;
constexpr uint8_t EXT_SENS_DATA_15 = 0x58;
constexpr uint8_t EXT_SENS_DATA_16 = 0x59;
constexpr uint8_t EXT_SENS_DATA_17 = 0x5A;
constexpr uint8_t EXT_SENS_DATA_18 = 0x5B;
constexpr uint8_t EXT_SENS_DATA_19 = 0x5C;
constexpr uint8_t EXT_SENS_DATA_20 = 0x5D;
constexpr uint8_t EXT_SENS_DATA_21 = 0x5E;
constexpr uint8_t EXT_SENS_DATA_22 = 0x5F;
constexpr uint8_t EXT_SENS_DATA_23 = 0x60;
constexpr uint8_t MOT_DETECT_STATUS = 0x61;
constexpr uint8_t I2C_SLV0_DO = 0x63;
constexpr uint8_t I2C_SLV1_DO = 0x64;
constexpr uint8_t I2C_SLV2_DO = 0x65;
constexpr uint8_t I2C_SLV3_DO = 0x66;
constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
constexpr uint8_t SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t MOT_DETECT_CTRL = 0x69;
constexpr uint8_t USER_CTRL = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
constexpr uint8_t PWR_MGMT_1 = 0x6B; // Device defaults to the SLEEP mode
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t DMP_BANK = 0x6D;  // Activates a specific bank in the DMP
constexpr uint8_t DMP_RW_PNT = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
constexpr uint8_t DMP_REG = 0x6F;  // Register in DMP from which to read or to which to write
constexpr uint8_t DMP_REG_1 = 0x70;
constexpr uint8_t DMP_REG_2 = 0x71;
constexpr uint8_t FIFO_COUNTH = 0x72;
constexpr uint8_t FIFO_COUNTL = 0x73;
constexpr uint8_t FIFO_R_W = 0x74;
constexpr uint8_t WHO_AM_I_MPU6050 = 0x75; // Should return 0x68

} // namespace mpu6050_registers
} // namespace quadcopter

#endif
