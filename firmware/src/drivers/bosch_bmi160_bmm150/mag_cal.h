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

#ifndef MAG_CAL_H_

#define MAG_CAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct MagCal {
    uint64_t start_time;
    uint64_t update_time;

    float acc_x, acc_y, acc_z, acc_w;
    float acc_xx, acc_xy, acc_xz, acc_xw;
    float acc_yy, acc_yz, acc_yw, acc_zz, acc_zw;

    float x_bias, y_bias, z_bias;
    float radius;

    float c00, c01, c02, c10, c11, c12, c20, c21, c22;

    size_t nsamples;

    uint16_t dig_z1;
    int16_t dig_z2, dig_z3, dig_z4;
    uint16_t dig_xyz1;
    uint8_t raw_dig_data[24];
    int8_t dig_x1, dig_y1, dig_x2, dig_y2;
    uint8_t dig_xy1;
    int8_t dig_xy2;
};

void initMagCal(struct MagCal *moc,
                  float x_bias, float y_bias, float z_bias,
                  float c00, float c01, float c02,
                  float c10, float c11, float c12,
                  float c20, float c21, float c22);

void destroy_mag_cal(struct MagCal *moc);

bool magCalUpdate(struct MagCal *moc, uint64_t sample_time_us,
                   float x, float y, float z);

void magCalGetBias(struct MagCal *moc, float *x, float *y, float *z);

void magCalAddBias(struct MagCal *moc, float x, float y, float z);

void magCalRemoveBias(struct MagCal *moc, float xi, float yi, float zi,
                         float *xo, float *yo, float *zo);

void magCalSetSoftiron(struct MagCal *moc,
                          float c00, float c01, float c02,
                          float c10, float c11, float c12,
                          float c20, float c21, float c22);

void magCalRemoveSoftiron(struct MagCal *moc, float xi, float yi, float zi,
                             float *xo, float *yo, float *zo);

int32_t bmm150TempCompensateX(struct MagCal *moc, int16_t mag_x, uint16_t rhall);
int32_t bmm150TempCompensateY(struct MagCal *moc, int16_t mag_y, uint16_t rhall);
int32_t bmm150TempCompensateZ(struct MagCal *moc, int16_t mag_z, uint16_t rhall);

uint16_t U16_AT(uint8_t *ptr);
int16_t S16_AT(uint8_t *ptr);

void saveDigData(struct MagCal *moc, uint8_t *data, size_t offset);

#ifdef __cplusplus
}
#endif

#endif  // MAG_CAL_H_
