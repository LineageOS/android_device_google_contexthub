#ifndef QUAT_H_

#define QUAT_H_

#include "mat.h"
#include "vec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec4 Quat;

void initQuat(Quat *q, const struct Mat33 *R);
void quatToMatrix(struct Mat33 *R, const Quat *q);
void quatNormalize(Quat *q);

#ifdef __cplusplus
}
#endif

#endif  // QUAT_H_
