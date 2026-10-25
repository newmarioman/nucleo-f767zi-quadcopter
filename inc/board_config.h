#ifndef QUADCOPTER_BOARD_CONFIG_H
#define QUADCOPTER_BOARD_CONFIG_H

#include "mbed.h"

namespace quadcopter {

struct BoardConfig {
    BoardConfig();

    PinName receiver;
    PinName i2c_sda;
    PinName i2c_scl;
    PinName range_trigger;
    PinName range_echo;
    PinName motor_front_left;
    PinName motor_front_right;
    PinName motor_rear_right;
    PinName motor_rear_left;
    PinName console_tx;
    PinName console_rx;
    PinName battery_lvl;
    PinName servo_first;
    PinName servo_second;
    PinName green_led;
    PinName blue_led;
    PinName red_led;
};

} // namespace quadcopter

#endif
