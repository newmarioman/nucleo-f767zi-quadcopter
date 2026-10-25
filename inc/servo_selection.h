#ifndef QUADCOPTER_SERVO_SELECTION_H
#define QUADCOPTER_SERVO_SELECTION_H

// Set to 1 here, or define ENABLE_SG90_SERVOS=1 for the entire build.
#ifndef ENABLE_SG90_SERVOS
#define ENABLE_SG90_SERVOS 0
#endif

#if ENABLE_SG90_SERVOS != 0 && ENABLE_SG90_SERVOS != 1
#error "ENABLE_SG90_SERVOS must be 0 or 1"
#endif

#endif
