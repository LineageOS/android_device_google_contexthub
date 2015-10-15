#ifndef VEC_H_

#define VEC_H_

#ifdef __cplusplus
extern "C" {
#endif

struct Vec3 {
    float x, y, z;
};

struct Vec4 {
    float x, y, z, w;
};

void initVec3(struct Vec3 *v, float x, float y, float z);
void vec3Add(struct Vec3 *v, const struct Vec3 *w);
void vec3Sub(struct Vec3 *v, const struct Vec3 *w);
void vec3ScalarMul(struct Vec3 *v, float c);
float vec3Dot(const struct Vec3 *v, const struct Vec3 *w);
float vec3NormSquared(const struct Vec3 *v);
float vec3Norm(const struct Vec3 *v);
void vec3Normalize(struct Vec3 *v);
void vec3Cross(struct Vec3 *u, const struct Vec3 *v, const struct Vec3 *w);
void findOrthogonalVector( float inX, float inY, float inZ,
        float *outX, float *outY, float *outZ);

void initVec4(struct Vec4 *v, float x, float y, float z, float w);

#ifdef __cplusplus
}
#endif

#endif  // VEC_H_
