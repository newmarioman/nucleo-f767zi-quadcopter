#ifndef QUADCOPTER_FUSION_SELECTION_H
#define QUADCOPTER_FUSION_SELECTION_H

#define FUSION_BASIC 1
#define FUSION_MADGWICK 2
#define FUSION_DMP 3

// Select one backend here, or define SENSOR_FUSION_MODE for the entire build.

// ⚠️ WARNING: FUSION_DMP ⚠️   
// The DMP's +/-2 g input can clip during strong acceleration or vibration. 
// Additionally, the ODR is only 200 Hz compared to the 1000 Hz of the other 
// two modes. Therefore, this mode has been implemented but not validated; 
// it is completely experimental and may compromise PID operation in some motor
// and chassis combinations.

#ifndef SENSOR_FUSION_MODE
#define SENSOR_FUSION_MODE FUSION_BASIC
#endif

#if SENSOR_FUSION_MODE == FUSION_BASIC
#define SENSOR_FUSION_NAME "Basic complementary filter"
#elif SENSOR_FUSION_MODE == FUSION_MADGWICK
#define SENSOR_FUSION_NAME "Madgwick IMU filter"
#elif SENSOR_FUSION_MODE == FUSION_DMP
#define SENSOR_FUSION_NAME "MPU6050 DMP MotionApps 6.12"
#else
#error "SENSOR_FUSION_MODE must be FUSION_BASIC, FUSION_MADGWICK, or FUSION_DMP"
#endif

#endif
