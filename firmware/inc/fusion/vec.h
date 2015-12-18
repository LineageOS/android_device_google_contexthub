#ifndef VEC_H_
#define VEC_H_

#include <nanohub_math.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Vec3 {
    float x, y, z;
};

struct Vec4 {
    float x, y, z, w;
};


static inline void initVec3(struct Vec3 *v, float x, float y, float z) {
    v->x = x;
    v->y = y;
    v->z = z;
}

static inline void vec3Add(struct Vec3 *v, const struct Vec3 *w) {
    v->x += w->x;
    v->y += w->y;
    v->z += w->z;
}

static inline void vec3Sub(struct Vec3 *v, const struct Vec3 *w) {
    v->x -= w->x;
    v->y -= w->y;
    v->z -= w->z;
}

static inline void vec3ScalarMul(struct Vec3 *v, float c) {
    v->x *= c;
    v->y *= c;
    v->z *= c;
}

static inline float vec3Dot(const struct Vec3 *v, const struct Vec3 *w) {
    return v->x * w->x + v->y * w->y + v->z * w->z;
}

static inline float vec3NormSquared(const struct Vec3 *v) {
    return vec3Dot(v, v);
}

static inline float vec3Norm(const struct Vec3 *v) {
    return sqrtf(vec3NormSquared(v));
}

static inline void vec3Normalize(struct Vec3 *v) {
    float invNorm = 1.0f / vec3Norm(v);
    v->x *= invNorm;
    v->y *= invNorm;
    v->z *= invNorm;
}

static inline void vec3Cross(struct Vec3 *u, const struct Vec3 *v, const struct Vec3 *w) {
    u->x = v->y * w->z - v->z * w->y;
    u->y = v->z * w->x - v->x * w->z;
    u->z = v->x * w->y - v->y * w->x;
}

static inline void initVec4(struct Vec4 *v, float x, float y, float z, float w) {
    v->x = x;
    v->y = y;
    v->z = z;
    v->w = w;
}


void findOrthogonalVector( float inX, float inY, float inZ,
        float *outX, float *outY, float *outZ);


#ifdef __cplusplus
}
#endif

#endif  // VEC_H_
