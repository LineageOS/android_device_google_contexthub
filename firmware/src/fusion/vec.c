#include <fusion/vec.h>

#include <nanohub_math.h>

void initVec3(struct Vec3 *v, float x, float y, float z) {
    v->x = x;
    v->y = y;
    v->z = z;
}

void vec3Add(struct Vec3 *v, const struct Vec3 *w) {
    v->x += w->x;
    v->y += w->y;
    v->z += w->z;
}

void vec3Sub(struct Vec3 *v, const struct Vec3 *w) {
    v->x -= w->x;
    v->y -= w->y;
    v->z -= w->z;
}

void vec3ScalarMul(struct Vec3 *v, float c) {
    v->x *= c;
    v->y *= c;
    v->z *= c;
}

float vec3Dot(const struct Vec3 *v, const struct Vec3 *w) {
    return v->x * w->x + v->y * w->y + v->z * w->z;
}

float vec3NormSquared(const struct Vec3 *v) {
    return vec3Dot(v, v);
}

float vec3Norm(const struct Vec3 *v) {
    return sqrtf(vec3NormSquared(v));
}

void vec3Normalize(struct Vec3 *v) {
    float invNorm = 1.0f / vec3Norm(v);
    v->x *= invNorm;
    v->y *= invNorm;
    v->z *= invNorm;
}

void vec3Cross(struct Vec3 *u, const struct Vec3 *v, const struct Vec3 *w) {
    u->x = v->y * w->z - v->z * w->y;
    u->y = v->z * w->x - v->x * w->z;
    u->z = v->x * w->y - v->y * w->x;
}

void initVec4(struct Vec4 *v, float x, float y, float z, float w) {
    v->x = x;
    v->y = y;
    v->z = z;
    v->w = w;
}

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
