#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct accelerometer_data_t
{
    int timestamp;
    float x;
    float y;
    float z;
    int8_t status;
} accelerometer_data_t;

#ifdef __cplusplus
}
#endif

#endif

