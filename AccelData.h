#ifndef ACCEL_DATA_H
#define ACCEL_DATA_H

#include <stdint.h>

/// raw 16 bit accelerometer counts
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelRaw;

/// scaled values in g units
typedef struct {
    float x;
    float y;
    float z;
} AccelScaled;

#endif // ACCEL_DATA_H
