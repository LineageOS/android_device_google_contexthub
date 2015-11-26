#include "mag_cal.h"
#include <fusion/mat.h>

#include <errno.h>
#include <nanohub_math.h>
#include <string.h>

#define MAX_EIGEN_RATIO     25.0f
#define MAX_EIGEN_MAG       80.0f   // uT
#define MIN_EIGEN_MAG       10.0f   // uT

#define MAX_FIT_MAG         80.0f
#define MIN_FIT_MAG         10.0f

#define MIN_BATCH_WINDOW    1.0e9   // 1 sec
#define MAX_BATCH_WINDOW    15.0e9  // 15 sec
#define MIN_BATCH_SIZE      25      // samples

#define BMM150_REG_REPXY          0x51
#define BMM150_REG_REPZ           0x52
#define BMM150_REG_DIG_X1         0x5d
#define BMM150_REG_DIG_Y1         0x5e
#define BMM150_REG_DIG_Z4_LSB     0x62
#define BMM150_REG_DIG_Z4_MSB     0x63
#define BMM150_REG_DIG_X2         0x64
#define BMM150_REG_DIG_Y2         0x65
#define BMM150_REG_DIG_Z2_LSB     0x68
#define BMM150_REG_DIG_Z2_MSB     0x69
#define BMM150_REG_DIG_Z1_LSB     0x6a
#define BMM150_REG_DIG_Z1_MSB     0x6b
#define BMM150_REG_DIG_XYZ1_LSB   0x6c
#define BMM150_REG_DIG_XYZ1_MSB   0x6d
#define BMM150_REG_DIG_Z3_LSB     0x6e
#define BMM150_REG_DIG_Z3_MSB     0x6f
#define BMM150_REG_DIG_XY2        0x70
#define BMM150_REG_DIG_XY1        0x71

#define BMI160_MAG_FLIP_OVERFLOW_ADCVAL     ((int16_t)-4096)
#define BMI160_MAG_HALL_OVERFLOW_ADCVAL     ((int16_t)-16384)
#define BMI160_MAG_OVERFLOW_OUTPUT          ((int16_t)-32768)
#define BMM150_CALIB_HEX_LACKS              0x100000
#define BMI160_MAG_OVERFLOW_OUTPUT_S32      ((int32_t)(-2147483647-1))

// eigen value magnitude and ratio test
static int moc_eigen_test(struct MagCal *moc)
{
    // covariance matrix
    struct Mat33 S;
    S.elem[0][0] = moc->acc_xx - moc->acc_x * moc->acc_x;
    S.elem[0][1] = S.elem[1][0] = moc->acc_xy - moc->acc_x * moc->acc_y;
    S.elem[0][2] = S.elem[2][0] = moc->acc_xz - moc->acc_x * moc->acc_z;
    S.elem[1][1] = moc->acc_yy - moc->acc_y * moc->acc_y;
    S.elem[1][2] = S.elem[2][1] = moc->acc_yz - moc->acc_y * moc->acc_z;
    S.elem[2][2] = moc->acc_zz - moc->acc_z * moc->acc_z;

    struct Vec3 eigenvals;
    struct Mat33 eigenvecs;
    mat33GetEigenbasis(&S, &eigenvals, &eigenvecs);

    float evmax = (eigenvals.x > eigenvals.y) ? eigenvals.x : eigenvals.y;
    evmax = (eigenvals.z > evmax) ? eigenvals.z : evmax;

    float evmin = (eigenvals.x < eigenvals.y) ? eigenvals.x : eigenvals.y;
    evmin = (eigenvals.z < evmin) ? eigenvals.z : evmin;

    float evmag = sqrtf(eigenvals.x + eigenvals.y + eigenvals.z);

    int eigen_pass = (evmin * MAX_EIGEN_RATIO > evmax)
                        && (evmag > MIN_EIGEN_MAG)
                        && (evmag < MAX_EIGEN_MAG);

    return eigen_pass;
}

//Kasa sphere fitting with normal equation
static int moc_fit(struct MagCal *moc, struct Vec3 *bias, float *radius)
{
    //    A    *   out   =    b
    // (4 x 4)   (4 x 1)   (4 x 1)
    struct Mat44 A;
    A.elem[0][0] = moc->acc_xx; A.elem[0][1] = moc->acc_xy;
    A.elem[0][2] = moc->acc_xz; A.elem[0][3] = moc->acc_x;
    A.elem[1][0] = moc->acc_xy; A.elem[1][1] = moc->acc_yy;
    A.elem[1][2] = moc->acc_yz; A.elem[1][3] = moc->acc_y;
    A.elem[2][0] = moc->acc_xz; A.elem[2][1] = moc->acc_yz;
    A.elem[2][2] = moc->acc_zz; A.elem[2][3] = moc->acc_z;
    A.elem[3][0] = moc->acc_x;  A.elem[3][1] = moc->acc_y;
    A.elem[3][2] = moc->acc_z;  A.elem[3][3] = 1.0f;

    struct Vec4 b;
    initVec4(&b, -moc->acc_xw, -moc->acc_yw, -moc->acc_zw, -moc->acc_w);

    struct Size4 pivot;
    mat44DecomposeLup(&A, &pivot);

    struct Vec4 out;
    mat44Solve(&A, &out, &b, &pivot);

    // sphere: (x - xc)^2 + (y - yc)^2 + (z - zc)^2 = r^2
    //
    // xc = -out[0] / 2, yc = -out[1] / 2, zc = -out[2] / 2
    // r = sqrt(xc^2 + yc^2 + zc^2 - out[3])

    struct Vec3 v;
    initVec3(&v, out.x, out.y, out.z);
    vec3ScalarMul(&v, -0.5f);

    float r = sqrtf(vec3Dot(&v, &v) - out.w);

    initVec3(bias, v.x, v.y, v.z);
    *radius = r;

    int success = 0;
    if (r > MIN_FIT_MAG && r < MAX_FIT_MAG) {
        success = 1;
    }

    return success;
}

static void moc_reset(struct MagCal *moc)
{
    moc->acc_x = moc->acc_y = moc->acc_z = moc->acc_w = 0.0f;
    moc->acc_xx = moc->acc_xy = moc->acc_xz = moc->acc_xw = 0.0f;
    moc->acc_yy = moc->acc_yz = moc->acc_yw = 0.0f;
    moc->acc_zz = moc->acc_zw = 0.0f;

    moc->nsamples = 0;
    moc->start_time = 0;
}

static int moc_batch_complete(struct MagCal *moc, uint64_t sample_time_ns)
{
    int complete = 0;

    if ((sample_time_ns - moc->start_time > MIN_BATCH_WINDOW)
        && (moc->nsamples > MIN_BATCH_SIZE)) {

        complete = 1;

    } else if (sample_time_ns - moc->start_time > MAX_BATCH_WINDOW) {
        // not enough samples collected in MAX_BATCH_WINDOW
        moc_reset(moc);
    }

    return complete;
}

void initMagCal(struct MagCal *moc,
                  float x_bias, float y_bias, float z_bias,
                  float c00, float c01, float c02,
                  float c10, float c11, float c12,
                  float c20, float c21, float c22)
{
    moc_reset(moc);
    moc->update_time = 0;
    moc->radius = 0.0f;

    moc->x_bias = x_bias;
    moc->y_bias = y_bias;
    moc->z_bias = z_bias;

    moc->c00 = c00; moc->c01 = c01; moc->c02 = c02;
    moc->c10 = c10; moc->c11 = c11; moc->c12 = c12;
    moc->c20 = c20; moc->c21 = c21; moc->c22 = c22;
}

void destroy_mag_cal(struct MagCal *moc)
{
    (void)moc;
}

bool magCalUpdate(struct MagCal *moc, uint64_t sample_time_ns,
                   float x, float y, float z)
{
    bool new_bias = false;

    // 1. run accumulators
    float w = x * x + y * y + z * z;

    moc->acc_x += x;
    moc->acc_y += y;
    moc->acc_z += z;
    moc->acc_w += w;

    moc->acc_xx += x * x;
    moc->acc_xy += x * y;
    moc->acc_xz += x * z;
    moc->acc_xw += x * w;

    moc->acc_yy += y * y;
    moc->acc_yz += y * z;
    moc->acc_yw += y * w;

    moc->acc_zz += z * z;
    moc->acc_zw += z * w;

    if (++moc->nsamples == 1) {
        moc->start_time = sample_time_ns;
    }

    // 2. batch has enough samples?
    if (moc_batch_complete(moc, sample_time_ns)) {

        float inv = 1.0f / moc->nsamples;

        moc->acc_x *= inv;
        moc->acc_y *= inv;
        moc->acc_z *= inv;
        moc->acc_w *= inv;

        moc->acc_xx *= inv;
        moc->acc_xy *= inv;
        moc->acc_xz *= inv;
        moc->acc_xw *= inv;

        moc->acc_yy *= inv;
        moc->acc_yz *= inv;
        moc->acc_yw *= inv;

        moc->acc_zz *= inv;
        moc->acc_zw *= inv;

        // 3. eigen test
        if (moc_eigen_test(moc)) {

            struct Vec3 bias;
            float radius;

            // 4. Kasa sphere fitting
            if (moc_fit(moc, &bias, &radius)) {

                moc->x_bias = bias.x;
                moc->y_bias = bias.y;
                moc->z_bias = bias.z;

                moc->radius = radius;
                moc->update_time = sample_time_ns;

                new_bias = true;
            }
        }

        // 5. reset for next batch
        moc_reset(moc);
    }

    return new_bias;
}

void magCalGetBias(struct MagCal *moc, float *x, float *y, float *z)
{
    *x = moc->x_bias;
    *y = moc->y_bias;
    *z = moc->z_bias;
}

void magCalRemoveBias(struct MagCal *moc, float xi, float yi, float zi,
                         float *xo, float *yo, float *zo)
{
    *xo = xi - moc->x_bias;
    *yo = yi - moc->y_bias;
    *zo = zi - moc->z_bias;
}

void magCalSetSoftiron(struct MagCal *moc,
                          float c00, float c01, float c02,
                          float c10, float c11, float c12,
                          float c20, float c21, float c22)
{
    moc->c00 = c00; moc->c01 = c01; moc->c02 = c02;
    moc->c10 = c10; moc->c11 = c11; moc->c12 = c12;
    moc->c20 = c20; moc->c21 = c21; moc->c22 = c22;
}

void magCalRemoveSoftiron(struct MagCal *moc, float xi, float yi, float zi,
                             float *xo, float *yo, float *zo)
{
    *xo = moc->c00 * xi + moc->c01 * yi + moc->c02 * zi;
    *yo = moc->c10 * xi + moc->c11 * yi + moc->c12 * zi;
    *zo = moc->c20 * xi + moc->c21 * yi + moc->c22 * zi;
}

int32_t bmm150TempCompensateX(
       struct MagCal *moc, int16_t mag_x, uint16_t rhall)
{
    int32_t inter_retval = 0;

    // some temp var to made the long calculation easier to read
    int32_t temp_1, temp_2, temp_3, temp_4;

    // no overflow
    if (mag_x != BMI160_MAG_FLIP_OVERFLOW_ADCVAL) {
        if ((rhall != 0) && (moc->dig_xyz1 != 0)) {

            inter_retval = ((int32_t)(((uint16_t) ((((int32_t)moc->dig_xyz1) << 14)
                / (rhall != 0 ?  rhall : moc->dig_xyz1))) - ((uint16_t)0x4000)));

        } else {
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
            return inter_retval;
        }

        temp_1 = ((int32_t)moc->dig_xy2) * ((((int32_t)inter_retval) * ((int32_t)inter_retval)) >> 7);
        temp_2 = ((int32_t)inter_retval) * ((int32_t)(((int16_t)moc->dig_xy1) << 7));
        temp_3 = ((temp_1 + temp_2) >> 9) + ((int32_t)BMM150_CALIB_HEX_LACKS);
        temp_4 = ((int32_t)mag_x) * ((temp_3 * ((int32_t)(((int16_t)moc->dig_x2) + ((int16_t)0xa0)))) >> 12);

        inter_retval = ((int32_t)(temp_4 >> 13)) + (((int16_t)moc->dig_x1) << 3);

        // check the overflow output
        if (inter_retval == (int32_t)BMI160_MAG_OVERFLOW_OUTPUT)
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
    } else {
        // overflow
        inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
    }
    return inter_retval;
}

int32_t bmm150TempCompensateY(struct MagCal *moc, int16_t mag_y, uint16_t rhall)
{
    int32_t inter_retval = 0;

    // some temp var to made the long calculation easier to read
    int32_t temp_1, temp_2, temp_3, temp_4;

    // no overflow
    if (mag_y != BMI160_MAG_FLIP_OVERFLOW_ADCVAL) {
        if ((rhall != 0) && (moc->dig_xyz1 != 0)) {

            inter_retval = ((int32_t)(((uint16_t)((( (int32_t)moc->dig_xyz1) << 14)
                / (rhall != 0 ?  rhall : moc->dig_xyz1))) - ((uint16_t)0x4000)));

        } else {
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
            return inter_retval;
        }

        temp_1 = ((int32_t)moc->dig_xy2) * ((((int32_t) inter_retval) * ((int32_t)inter_retval)) >> 7);
        temp_2 = ((int32_t)inter_retval) * ((int32_t)(((int16_t)moc->dig_xy1) << 7));
        temp_3 = ((temp_1 + temp_2) >> 9) + ((int32_t)BMM150_CALIB_HEX_LACKS);
        temp_4 = ((int32_t)mag_y) * ((temp_3 * ((int32_t)(((int16_t)moc->dig_y2) + ((int16_t)0xa0)))) >> 12);

        inter_retval = ((int32_t)(temp_4 >> 13)) + (((int16_t)moc->dig_y1) << 3);

        // check the overflow output
        if (inter_retval == (int32_t)BMI160_MAG_OVERFLOW_OUTPUT)
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
    } else {
        // overflow
        inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
    }
    return inter_retval;
}

int32_t bmm150TempCompensateZ(struct MagCal *moc, int16_t mag_z, uint16_t rhall)
{
    int32_t retval = 0;
    if (mag_z != BMI160_MAG_HALL_OVERFLOW_ADCVAL) {
        if ((rhall != 0) && (moc->dig_z2 != 0) && (moc->dig_z1 != 0)) {

            retval = ((((int32_t)(mag_z - moc->dig_z4)) << 15)
                    - ((((int32_t)moc->dig_z3) * ((int32_t)(((int16_t)rhall) - ((int16_t)moc->dig_xyz1)))) >> 2));

            retval /= (moc->dig_z2
                    + ((int16_t)(((((int32_t)moc->dig_z1) * ((((int16_t)rhall) << 1))) + (1 << 15)) >> 16)));
        }
    } else {
        retval = BMI160_MAG_OVERFLOW_OUTPUT;
    }
    return retval;
}

uint16_t U16_AT(uint8_t *ptr)
{
    return (uint32_t)ptr[0] | ((uint32_t)ptr[1] << 8);
}

int16_t S16_AT(uint8_t *ptr)
{
    return (int32_t)ptr[0] | ((int32_t)ptr[1] << 8);
}


void saveDigData(struct MagCal *moc, uint8_t *data, size_t offset)
{
    // magnetometer temperature calibration data is read in 3 bursts of 8 byte
    // length each.
    memcpy(&moc->raw_dig_data[offset], data, 8);

    if (offset == 16) {
        // we have all the raw data.

        static const size_t first_reg = BMM150_REG_DIG_X1;
        moc->dig_x1 = moc->raw_dig_data[BMM150_REG_DIG_X1 - first_reg];
        moc->dig_y1 = moc->raw_dig_data[BMM150_REG_DIG_Y1 - first_reg];
        moc->dig_x2 = moc->raw_dig_data[BMM150_REG_DIG_X2 - first_reg];
        moc->dig_y2 = moc->raw_dig_data[BMM150_REG_DIG_Y2 - first_reg];
        moc->dig_xy2 = moc->raw_dig_data[BMM150_REG_DIG_XY2 - first_reg];
        moc->dig_xy1 = moc->raw_dig_data[BMM150_REG_DIG_XY1 - first_reg];

        moc->dig_z1 = U16_AT(&moc->raw_dig_data[BMM150_REG_DIG_Z1_LSB - first_reg]);
        moc->dig_z2 = S16_AT(&moc->raw_dig_data[BMM150_REG_DIG_Z2_LSB - first_reg]);
        moc->dig_z3 = S16_AT(&moc->raw_dig_data[BMM150_REG_DIG_Z3_LSB - first_reg]);
        moc->dig_z4 = S16_AT(&moc->raw_dig_data[BMM150_REG_DIG_Z4_LSB - first_reg]);

        moc->dig_xyz1 = U16_AT(&moc->raw_dig_data[BMM150_REG_DIG_XYZ1_LSB - first_reg]);
    }
}

