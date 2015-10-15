#ifndef _SENS_TYPE_H_
#define _SENS_TYPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SENS_TYPE_INVALID         0
#define SENS_TYPE_ACCEL           1
#define SENS_TYPE_ANY_MOTION      2 //provided by ACCEL, nondiscardable edge trigger
#define SENS_TYPE_NO_MOTION       3 //provided by ACCEL, nondiscardable edge trigger
#define SENS_TYPE_SIG_MOTION      4 //provided by ACCEL, nondiscardable edge trigger
#define SENS_TYPE_FLAT            5
#define SENS_TYPE_GYRO            6
#define SENS_TYPE_GYRO_UNCAL      7
#define SENS_TYPE_MAG             8
#define SENS_TYPE_MAG_UNCAL       9
#define SENS_TYPE_BARO            10
#define SENS_TYPE_TEMP            11
#define SENS_TYPE_ALS             12
#define SENS_TYPE_PROX            13
#define SENS_TYPE_ORIENTATION     14
#define SENS_TYPE_HEARTRATE_ECG   15
#define SENS_TYPE_HEARTRATE_PPG   16
#define SENS_TYPE_GRAVITY         17
#define SENS_TYPE_LINEAR_ACCEL    18
#define SENS_TYPE_ROTATION_VECTOR 19
#define SENS_TYPE_GEO_MAG_ROT_VEC 20
#define SENS_TYPE_GAME_ROT_VECTOR 21
#define SENS_TYPE_STEP_COUNT      22
#define SENS_TYPE_STEP_DETECT     23
#define SENS_TYPE_GESTURE         24
#define SENS_TYPE_TILT            25
#define SENS_TYPE_DOUBLE_TWIST    26
#define SENS_TYPE_DOUBLE_TAP      27
#define SENS_TYPE_WIN_ORIENTATION 28
#define SENS_TYPE_HALL            29
#define SENS_TYPE_ACTIVITY        30
#define SENS_TYPE_FIRST_USER      64

#ifdef __cplusplus
}
#endif

#endif
