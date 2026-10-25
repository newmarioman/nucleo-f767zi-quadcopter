#include "flight_config.h"

namespace quadcopter {

FlightConfig::FlightConfig()
    : radio{3400, 11000, 1650, 400, 1000, 2000},
      control{{1.5f, 0.004f, 18.0f, 80, 200},
              5.5f, 16.6, 180, 4.0f, 50, 3.0, 1.0f,
              1492, 1508},
      imu{57.296, 0.7, 0.3,
          0.001f, 0.001f, 20, 1, 1.0f},
      motors{20000, 1000, 2000, 1700.0f, 700, 1500, 1050.0f,
             1100, 3000, 1000, 40},
      fusion{60.0f, 1000, 100},
      battery{3.3f, 1.0f / 11.0f, 11.0f, 100000u},
      servos{20000, 1000, 2000, 1500, true},
      i2c_frequency_hz(400000),
      serial_baud(921600),
      startup_delay_ms(500),
      loop_warmup_ms(1000),
      initial_cycle_seconds(0.0025f)
{
}

} // namespace quadcopter
