/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fusion/mat.h>

#include <assert.h>
#include <nanohub_math.h>
#include <sys/types.h>
#include <float.h>
#include <string.h>

#define kEps 1E-5

void initZeroMatrix(struct Mat33 *A) {
    memset(A->elem, 0.0f, sizeof(A->elem));
}

void initDiagonalMatrix(struct Mat33 *A, float x) {
    initZeroMatrix(A);

    size_t i;
    for (i = 0; i < 3; ++i) {
        A->elem[i][i] = x;
    }
}

void initMatrixColumns(
        struct Mat33 *A, const struct Vec3 *v1, const struct Vec3 *v2, const struct Vec3 *v3) {
    A->elem[0][0] = v1->x;
    A->elem[0][1] = v2->x;
    A->elem[0][2] = v3->x;

    A->elem[1][0] = v1->y;
    A->elem[1][1] = v2->y;
    A->elem[1][2] = v3->y;

    A->elem[2][0] = v1->z;
    A->elem[2][1] = v2->z;
    A->elem[2][2] = v3->z;
}

void mat33Apply(struct Vec3 *out, const struct Mat33 *A, const struct Vec3 *v) {
    // assert(out != v);
    out->x = A->elem[0][0] * v->x + A->elem[0][1] * v->y + A->elem[0][2] * v->z;
    out->y = A->elem[1][0] * v->x + A->elem[1][1] * v->y + A->elem[1][2] * v->z;
    out->z = A->elem[2][0] * v->x + A->elem[2][1] * v->y + A->elem[2][2] * v->z;
}

void mat33Multiply(struct Mat33 *out, const struct Mat33 *A, const struct Mat33 *B) {
    // assert(out != A);
    // assert(out != B);

    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            size_t k;
            float sum = 0.0f;
            for (k = 0; k < 3; ++k) {
                sum += A->elem[i][k] * B->elem[k][j];
            }

            out->elem[i][j] = sum;
        }
    }
}

void mat33ScalarMul(struct Mat33 *A, float c) {
    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            A->elem[i][j] *= c;
        }
    }
}

void mat33Add(struct Mat33 *out, const struct Mat33 *A) {
    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            out->elem[i][j] += A->elem[i][j];
        }
    }
}

void mat33Sub(struct Mat33 *out, const struct Mat33 *A) {
    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            out->elem[i][j] -= A->elem[i][j];
        }
    }
}

int mat33IsPositiveSemidefinite(const struct Mat33 *A, float tolerance) {
    size_t i;
    for (i = 0; i < 3; ++i) {
        if (A->elem[i][i] < 0.0f) {
            return 0;
        }
    }

    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = i + 1; j < 3; ++j) {
            if (fabsf(A->elem[i][j] - A->elem[j][i]) > tolerance) {
                return 0;
            }
        }
    }

    return 1;
}

void mat33MultiplyTransposed(
        struct Mat33 *out, const struct Mat33 *A, const struct Mat33 *B) {
    // assert(out != A);
    // assert(out != B);

    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            size_t k;
            float sum = 0.0f;
            for (k = 0; k < 3; ++k) {
                sum += A->elem[k][i] * B->elem[k][j];
            }

            out->elem[i][j] = sum;
        }
    }
}

void mat33MultiplyTransposed2(
        struct Mat33 *out, const struct Mat33 *A, const struct Mat33 *B) {
    // assert(out != A);
    // assert(out != B);

    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            size_t k;
            float sum = 0.0f;
            for (k = 0; k < 3; ++k) {
                sum += A->elem[i][k] * B->elem[j][k];
            }

            out->elem[i][j] = sum;
        }
    }
}

void mat33Invert(struct Mat33 *out, const struct Mat33 *A) {
    float t;
    initDiagonalMatrix(out, 1.0f);

    struct Mat33 tmp = *A;

    size_t i, k;
    for (i = 0; i < 3; ++i) {
        size_t swap = i;
        size_t j;
        for (j = i + 1; j < 3; ++j) {
            if (fabsf(tmp.elem[j][i]) > fabsf(tmp.elem[i][i])) {
                swap = j;
            }
        }

        if (swap != i) {
            for (k = 0; k < 3; ++k) {
                t = tmp.elem[i][k];
                tmp.elem[i][k] = tmp.elem[swap][k];
                tmp.elem[swap][k] = t;

                t = out->elem[i][k];
                out->elem[i][k] = out->elem[swap][k];
                out->elem[swap][k] = t;
            }
        }

        t = 1.0f / tmp.elem[i][i];
        for (k = 0; k < 3; ++k) {
            tmp.elem[i][k] *= t;
            out->elem[i][k] *= t;
        }

        for (j = 0; j < 3; ++j) {
            if (j != i) {
                t = tmp.elem[j][i];
                for (k = 0; k < 3; ++k) {
                    tmp.elem[j][k] -= tmp.elem[i][k] * t;
                    out->elem[j][k] -= out->elem[i][k] * t;
                }
            }
        }
    }
}

void mat33Transpose(struct Mat33 *out, const struct Mat33 *A) {
    size_t i;
    for (i = 0; i < 3; ++i) {
        size_t j;
        for (j = 0; j < 3; ++j) {
            out->elem[i][j] = A->elem[j][i];
        }
    }
}

void mat33DecomposeLup(struct Mat33 *LU, struct Size3 *pivot) {

    const size_t N = 3;
    size_t i, j, k;

    for (k = 0; k < N; ++k) {
        pivot->elem[k] = k;
        float max = fabsf(LU->elem[k][k]);
        for (j = k + 1; j < N; ++j) {
            if (max < fabsf(LU->elem[j][k])) {
                max = fabsf(LU->elem[j][k]);
                pivot->elem[k] = j;
            }
        }

        if (pivot->elem[k] != k) {
            mat33SwapRows(LU, k, pivot->elem[k]);
        }

        if (fabsf(LU->elem[k][k]) < kEps) {
            continue;
        }

        for (j = k + 1; j < N; ++j) {
            LU->elem[k][j] /= LU->elem[k][k];
        }

        for (i = k + 1; i < N; ++i) {
            for (j = k + 1; j < 3; ++j) {
                LU->elem[i][j] -= LU->elem[i][k] * LU->elem[k][j];
            }
        }
    }
}

void mat33SwapRows(struct Mat33 *A, const size_t i, const size_t j) {

    const size_t N = 3;
    size_t k;

    if (i == j) {
        return;
    }

    for (k = 0; k < N; ++k) {
        float tmp = A->elem[i][k];
        A->elem[i][k] = A->elem[j][k];
        A->elem[j][k] = tmp;
    }
}

void mat33Solve(const struct Mat33 *A, struct Vec3 *x, const struct Vec3 *b, const struct Size3 *pivot) {

    const size_t N = 3;
    size_t i, k;

    float bCopy[N];
    bCopy[0] = b->x;
    bCopy[1] = b->y;
    bCopy[2] = b->z;

    float _x[N];
    for (k = 0; k < N; ++k) {
        if (pivot->elem[k] != k) {
            float tmp = bCopy[k];
            bCopy[k] = bCopy[pivot->elem[k]];
            bCopy[pivot->elem[k]] = tmp;
        }

        _x[k] = bCopy[k];
        for (i = 0; i < k; ++i) {
            _x[k] -= _x[i] * A->elem[k][i];
        }
        _x[k] /= A->elem[k][k];
    }

    for (k = N; k-- > 0;) {
        for (i = k + 1; i < N; ++i) {
            _x[k] -= _x[i] * A->elem[k][i];
        }
    }

    initVec3(x, _x[0], _x[1], _x[2]);
}

/* Returns the eigenvalues and corresponding eigenvectors of the _symmetric_ matrix.
 The i-th eigenvalue corresponds to the eigenvector in the i-th _row_ of "eigenvecs".
 */

void mat33GetEigenbasis(struct Mat33 *S, struct Vec3 *eigenvals, struct Mat33 *eigenvecs) {

    const size_t N = 3;
    size_t i, j, k, l, m;

    float _eigenvals[N];

    size_t ind[N];
    for (k = 0; k < N; ++k) {
        ind[k] = mat33Maxind(S, k);
        _eigenvals[k] = S->elem[k][k];
    }

    initDiagonalMatrix(eigenvecs, 1.0f);

    for (;;) {
        m = 0;
        for (k = 1; k + 1 < N; ++k) {
            if (fabsf(S->elem[k][ind[k]]) > fabsf(S->elem[m][ind[m]])) {
                m = k;
            }
        }

        k = m;
        l = ind[m];
        float p = S->elem[k][l];

        if (fabsf(p) < kEps) {
            break;
        }

        float y = (_eigenvals[l] - _eigenvals[k]) * 0.5f;

        float t = fabsf(y) + sqrtf(p * p + y * y);
        float s = sqrtf(p * p + t * t);
        float c = t / s;
        s = p / s;
        t = p * p / t;

        if (y < 0.0f) {
            s = -s;
            t = -t;
        }

        S->elem[k][l] = 0.0f;

        _eigenvals[k] -= t;
        _eigenvals[l] += t;

        for (i = 0; i < k; ++i) {
            mat33Rotate(S, c, s, i, k, i, l);
        }

        for (i = k + 1; i < l; ++i) {
            mat33Rotate(S, c, s, k, i, i, l);
        }

        for (i = l + 1; i < N; ++i) {
            mat33Rotate(S, c, s, k, i, l, i);
        }

        for (i = 0; i < N; ++i) {
            float tmp = c * eigenvecs->elem[k][i] - s * eigenvecs->elem[l][i];
            eigenvecs->elem[l][i] = s * eigenvecs->elem[k][i] + c * eigenvecs->elem[l][i];
            eigenvecs->elem[k][i] = tmp;
        }

        ind[k] = mat33Maxind(S, k);
        ind[l] = mat33Maxind(S, l);

        float sum = 0.0f;
        for (i = 0; i < N; ++i) {
            for (j = i + 1; j < N; ++j) {
                sum += fabsf(S->elem[i][j]);
            }
        }

        if (sum < kEps) {
            break;
        }
    }

    for (k = 0; k < N; ++k) {
        m = k;
        for (l = k + 1; l < N; ++l) {
            if (_eigenvals[l] > _eigenvals[m]) {
                m = l;
            }
        }

        if (k != m) {
            float tmp = _eigenvals[k];
            _eigenvals[k] = _eigenvals[m];
            _eigenvals[m] = tmp;

            mat33SwapRows(eigenvecs, k, m);
        }
    }

    initVec3(eigenvals, _eigenvals[0], _eigenvals[1], _eigenvals[2]);
}

// index of largest off-diagonal element in row k
size_t mat33Maxind(const struct Mat33 *A, size_t k) {

    const size_t N = 3;

    size_t m = k + 1;
    size_t i;

    for (i = k + 2; i < N; ++i) {
        if (fabsf(A->elem[k][i]) > fabsf(A->elem[k][m])) {
            m = i;
        }
    }

    return m;
}

void mat33Rotate(struct Mat33 *A, float c, float s, size_t k, size_t l, size_t i, size_t j) {
    float tmp = c * A->elem[k][l] - s * A->elem[i][j];
    A->elem[i][j] = s * A->elem[k][l] + c * A->elem[i][j];
    A->elem[k][l] = tmp;
}

void mat44Apply(struct Vec4 *out, const struct Mat44 *A, const struct Vec4 *v) {
    // assert(out != v);

    out->x =
        A->elem[0][0] * v->x
            + A->elem[0][1] * v->y
            + A->elem[0][2] * v->z
            + A->elem[0][3] * v->w;

    out->y =
        A->elem[1][0] * v->x
            + A->elem[1][1] * v->y
            + A->elem[1][2] * v->z
            + A->elem[1][3] * v->w;

    out->z =
        A->elem[2][0] * v->x
            + A->elem[2][1] * v->y
            + A->elem[2][2] * v->z
            + A->elem[2][3] * v->w;

    out->w =
        A->elem[3][0] * v->x
            + A->elem[3][1] * v->y
            + A->elem[3][2] * v->z
            + A->elem[3][3] * v->w;
}

void mat44DecomposeLup(struct Mat44 *LU, struct Size4 *pivot) {

    const size_t N = 4;
    size_t i, j, k;

    for (k = 0; k < N; ++k) {
        pivot->elem[k] = k;
        float max = fabsf(LU->elem[k][k]);
        for (j = k + 1; j < N; ++j) {
            if (max < fabsf(LU->elem[j][k])) {
                max = fabsf(LU->elem[j][k]);
                pivot->elem[k] = j;
            }
        }

        if (pivot->elem[k] != k) {
            mat44SwapRows(LU, k, pivot->elem[k]);
        }

        if (fabsf(LU->elem[k][k]) < kEps) {
            continue;
        }

        for (j = k + 1; j < N; ++j) {
            LU->elem[k][j] /= LU->elem[k][k];
        }

        for (i = k + 1; i < N; ++i) {
            for (j = k + 1; j < N; ++j) {
                LU->elem[i][j] -= LU->elem[i][k] * LU->elem[k][j];
            }
        }
    }
}

void mat44SwapRows(struct Mat44 *A, const size_t i, const size_t j) {

    const size_t N = 4;
    size_t k;

    if (i == j) {
        return;
    }

    for (k = 0; k < N; ++k) {
        float tmp = A->elem[i][k];
        A->elem[i][k] = A->elem[j][k];
        A->elem[j][k] = tmp;
    }
}

void mat44Solve(const struct Mat44 *A, struct Vec4 *x, const struct Vec4 *b, const struct Size4 *pivot) {

    const size_t N = 4;
    size_t i, k;

    float bCopy[N];
    bCopy[0] = b->x;
    bCopy[1] = b->y;
    bCopy[2] = b->z;
    bCopy[3] = b->w;

    float _x[N];
    for (k = 0; k < N; ++k) {
        if (pivot->elem[k] != k) {
            float tmp = bCopy[k];
            bCopy[k] = bCopy[pivot->elem[k]];
            bCopy[pivot->elem[k]] = tmp;
        }

        _x[k] = bCopy[k];
        for (i = 0; i < k; ++i) {
            _x[k] -= _x[i] * A->elem[k][i];
        }
        _x[k] /= A->elem[k][k];
    }

    for (k = N; k-- > 0;) {
        for (i = k + 1; i < N; ++i) {
            _x[k] -= _x[i] * A->elem[k][i];
        }
    }

    initVec4(x, _x[0], _x[1], _x[2], _x[3]);
}
