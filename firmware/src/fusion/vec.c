#include <fusion/vec.h>

#include <nanohub_math.h>


void findOrthogonalVector( float inX, float inY, float inZ, float *outX, float *outY, float *outZ) {

    float x, y, z;

    // discard the one with the smallest absolute value
    if (fabsf(inX) <= fabsf(inY) && fabsf(inX) <= fabsf(inZ)) {
        x = 0.0f;
        y = inZ;
        z = -inY;
    } else if (fabsf(inY) <= fabsf(inZ)) {
        x = inZ;
        y = 0.0f;
        z = -inX;
    } else {
        x = inY;
        y = -inX;
        z = 0.0f;
    }

    float magSquared = x * x + y * y + z * z;
    float invMag = 1.0f / sqrtf(magSquared);

    *outX = x * invMag;
    *outY = y * invMag;
    *outZ = z * invMag;
}
