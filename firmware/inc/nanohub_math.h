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


#define sinf   arm_sin_f32
#define cosf   arm_cos_f32

#define sqrt    __ieee754_sqrt
#define acos    __ieee754_acos
#define acosh   __ieee754_acosh
#define log     __ieee754_log
#define atanh   __ieee754_atanh
#define asin    __ieee754_asin
#define atan2   __ieee754_atan2
#define exp     __ieee754_exp
#define cosh    __ieee754_cosh
#define fmod    __ieee754_fmod
#define pow     __ieee754_pow
#define lgamma  __ieee754_lgamma
#define gamma   __ieee754_gamma
#define gamma_r __ieee754_gamma_r
#define log10   __ieee754_log10
#define sinh    __ieee754_sinh
#define hypot   __ieee754_hypot
#define scalb   __ieee754_scalb
#define sqrtf   __ieee754_sqrtf
#define acosf   __ieee754_acosf
#define acoshf  __ieee754_acoshf
#define logf    __ieee754_logf
#define atanhf  __ieee754_atanhf
#define asinf   __ieee754_asinf
#define atan2f  __ieee754_atan2f
#define expf    __ieee754_expf
#define coshf   __ieee754_coshf
#define fmodf   __ieee754_fmodf
#define powf    __ieee754_powf
#define lgammaf __ieee754_lgammaf
#define gammaf  __ieee754_gammaf
#define log10f  __ieee754_log10f
#define sinhf   __ieee754_sinhf
#define hypotf  __ieee754_hypotf
#define scalbf  __ieee754_scalbf
#define lgamma_r    __ieee754_lgamma_r
#define remainder   __ieee754_remainder
#define lgammaf_r   __ieee754_lgammaf_r
#define gammaf_r    __ieee754_gammaf_r
#define remainderf  __ieee754_remainderf

#include <math.h>
