#include "board_config.h"

namespace quadcopter {

BoardConfig::BoardConfig()
    : receiver(PC_7),
      i2c_sda(PF_0),
      i2c_scl(PF_1),
      range_trigger(PB_8),
      range_echo(PB_9),
      motor_front_left(PE_9),
      motor_front_right(PE_11),
      motor_rear_right(PE_14),
      motor_rear_left(PE_13),
      console_tx(USBTX),
      console_rx(USBRX),
      battery_lvl(PA_3),
      servo_first(PB_10),
      servo_second(PB_11),
      green_led(LED1),
      blue_led(LED2),
      red_led(LED3)
{
}

} // namespace quadcopter
