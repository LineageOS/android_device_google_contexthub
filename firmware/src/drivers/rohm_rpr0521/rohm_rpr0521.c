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

#include <stdlib.h>
#include <string.h>
#include <float.h>

#include <seos.h>
#include <i2c.h>
#include <timer.h>
#include <sensors.h>
#include <heap.h>
#include <hostIntf.h>
#include <nanohubPacket.h>
#include <eventnums.h>

#define I2C_BUS_ID 0
#define I2C_SPEED 400000
#define I2C_ADDR 0x38

#define ROHM_RPR0521_REG_ID                     0x92
#define ROHM_RPR0521_REG_SYSTEM_CONTROL         0x40
#define ROHM_RPR0521_REG_MODE_CONTROL           0x41
#define ROHM_RPR0521_REG_ALS_PS_CONTROL         0x42
#define ROHM_RPR0521_REG_PS_CONTROL             0x43
#define ROHM_RPR0521_REG_PS_DATA_LSB            0x44
#define ROHM_RPR0521_REG_ALS_DATA0_LSB          0x46
#define ROHM_RPR0521_REG_ALS_DATA1_LSB          0x48
#define ROHM_RPR0521_REG_INTERRUPT              0x4a
#define ROHM_RPR0521_REG_PS_TH_LSB              0x4b
#define ROHM_RPR0521_REG_PS_TH_MSB              0x4c
#define ROHM_RPR0521_REG_PS_TL_LSB              0x4d
#define ROHM_RPR0521_REG_PS_TL_MSB              0x4e
#define ROHM_RPR0521_REG_ALS_DATA0_TH_LSB       0x4f
#define ROHM_RPR0521_REG_ALS_DATA0_TL_LSB       0x51
#define ROHM_RPR0521_REG_PS_OFFSET_LSB          0x53
#define ROHM_RPR0521_REG_PS_OFFSET_MSB          0x54

#define ROHM_RPR0521_ID                         0xe0

enum {
    ALS_GAIN_X1         = 0,
    ALS_GAIN_X2         = 1,
    ALS_GAIN_X64        = 2,
    ALS_GAIN_X128       = 3,
};
#define ROHM_RPR0521_GAIN_ALS0          ALS_GAIN_X1
#define ROHM_RPR0521_GAIN_ALS1          ALS_GAIN_X1

enum {
    PS_GAIN_X1          = 0,
    PS_GAIN_X2          = 1,
    PS_GAIN_X4          = 2,
};
#define ROHM_RPR0521_GAIN_PS            PS_GAIN_X1

#define ROHM_RPR0521_LED_CURRENT_100MA          2

#define ROHM_RPR0521_CAL_DEFAULT_OFFSET         20

/* ROHM_RPR0521_REG_SYSTEM_CONTROL */
#define SW_RESET_BIT                            (1 << 7)

/* ROHM_RPR0521_REG_MODE_CONTROL */
#define ALS_EN_BIT                              (1 << 7)
#define PS_EN_BIT                               (1 << 6)

#define ROHM_RPR0521_REPORT_NEAR_VALUE          0.0f // centimeters
#define ROHM_RPR0521_REPORT_FAR_VALUE           5.0f // centimeters
#define ROHM_RPR0521_THRESHOLD_ASSERT_NEAR      12   // value in PS_DATA
#define ROHM_RPR0521_THRESHOLD_DEASSERT_NEAR    7    // value in PS_DATA

/* Private driver events */
enum SensorEvents
{
    EVT_SENSOR_I2C = EVT_APP_START + 1,
    EVT_SENSOR_ALS_TIMER,
    EVT_SENSOR_PROX_TIMER,
};

/* I2C state machine */
enum SensorState
{
    SENSOR_STATE_RESET,
    SENSOR_STATE_VERIFY_ID,
    SENSOR_STATE_INIT,
    SENSOR_STATE_ENABLING_ALS,
    SENSOR_STATE_ENABLING_PROX,
    SENSOR_STATE_DISABLING_ALS,
    SENSOR_STATE_DISABLING_PROX,
    SENSOR_STATE_IDLE,
    SENSOR_STATE_SAMPLING,
};

enum ProxState
{
    PROX_STATE_INIT,
    PROX_STATE_NEAR,
    PROX_STATE_FAR,
};

enum MeasurementTime {
    MEASUREMENT_TIME_ALS_STANDBY_PS_STANDBY     = 0,
    MEASUREMENT_TIME_ALS_STANDBY_PS_10          = 1,
    MEASUREMENT_TIME_ALS_STANDBY_PS_40          = 2,
    MEASUREMENT_TIME_ALS_STANDBY_PS_100         = 3,
    MEASUREMENT_TIME_ALS_STANDBY_PS_400         = 4,
    MEASUREMENT_TIME_ALS_100_PS_50              = 5,
    MEASUREMENT_TIME_ALS_100_PS_100             = 6,
    MEASUREMENT_TIME_ALS_100_PS_400             = 7,
    MEASUREMENT_TIME_ALS_400_PS_50              = 8,
    MEASUREMENT_TIME_ALS_400_PS_100             = 9,
    MEASUREMENT_TIME_ALS_400_PS_STANDBY         = 10,
    MEASUREMENT_TIME_ALS_400_PS_400             = 11,
    MEASUREMENT_TIME_ALS_50_PS_50               = 12,
};

struct SensorData
{
    union {
        uint8_t bytes[16];
        struct {
            uint16_t prox;
            uint16_t als[2];
        } sample;
    } txrxBuf;

    uint32_t tid;

    uint32_t alsHandle;
    uint32_t proxHandle;
    uint32_t alsTimerHandle;
    uint32_t proxTimerHandle;

    union EmbeddedDataPoint lastAlsSample;

    uint8_t proxState; // enum ProxState

    bool alsOn;
    bool alsReading;
    bool proxOn;
    bool proxReading;
} data;

static const uint32_t supportedRates[] =
{
    SENSOR_HZ(0.1),
    SENSOR_HZ(1),
    SENSOR_HZ(4),
    SENSOR_HZ(5),
    SENSOR_HZ(10),
    SENSOR_HZ(25),
    0,
};

static const uint64_t rateTimerVals[] = //should match "supported rates in length" and be the timer length for that rate in nanosecs
{
    10 * 1000000000ULL,
     1 * 1000000000ULL,
    1000000000ULL / 4,
    1000000000ULL / 5,
    1000000000ULL / 10,
    1000000000ULL / 25,
};

/*
 * Helper functions
 */

static void i2cCallback(void *cookie, size_t tx, size_t rx, int err)
{
    if (err == 0)
        osEnqueuePrivateEvt(EVT_SENSOR_I2C, cookie, NULL, data.tid);
    else
        osLog(LOG_INFO, "ROHM: i2c error (%d)\n", err);
}

static void alsTimerCallback(uint32_t timerId, void *cookie)
{
    osEnqueuePrivateEvt(EVT_SENSOR_ALS_TIMER, cookie, NULL, data.tid);
}

static void proxTimerCallback(uint32_t timerId, void *cookie)
{
    osEnqueuePrivateEvt(EVT_SENSOR_PROX_TIMER, cookie, NULL, data.tid);
}

static inline float getLuxFromAlsData(uint16_t als0, uint16_t als1)
{
    static const float invGain[] = {1.0f, 0.5f, 1.0f / 64.0f, 1.0f / 128.0f};
    float d0 = (float)als0 * invGain[ROHM_RPR0521_GAIN_ALS0];
    float d1 = (float)als1 * invGain[ROHM_RPR0521_GAIN_ALS1];
    float ratio = d1 / d0;
    float c1;
    float c2;

    if (ratio < 1.221f) {
        c1 = 6.323f;
        c2 = -3.917f;
    } else if (ratio < 1.432f) {
        c1 = 5.350f;
        c2 = -3.121f;
    } else if (ratio < 1.710f) {
        c1 = 2.449f;
        c2 = -1.096f;
    } else if (ratio < 3.393f) {
        c1 = 1.155f;
        c2 = -0.340f;
    } else {
        c1 = c2 = 0.0f;
    }

    return c1 * d0 + c2 * d1;
}

static void setMode(bool alsOn, bool proxOn, void *cookie)
{
    static const uint8_t measurementTime[] = {
        MEASUREMENT_TIME_ALS_STANDBY_PS_STANDBY, /* als disabled, prox disabled */
        MEASUREMENT_TIME_ALS_100_PS_400,         /* als enabled, prox disabled (unsupported in HW, so keep PS on) */
        MEASUREMENT_TIME_ALS_STANDBY_PS_100,     /* als disabled, prox enabled  */
        MEASUREMENT_TIME_ALS_100_PS_100,         /* als enabled, prox enabled */
    };

    data.txrxBuf.bytes[0] = ROHM_RPR0521_REG_MODE_CONTROL;
    data.txrxBuf.bytes[1] = measurementTime[alsOn ? 1 : 0 + proxOn ? 2 : 0] |
                      (alsOn ? ALS_EN_BIT : 0) |
                      (proxOn ? PS_EN_BIT : 0);
    i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                &i2cCallback, cookie);
}

static bool sensorPowerAls(bool on, void *cookie)
{
    osLog(LOG_INFO, "ROHM: sensorPowerAls: %d\n", on);

    if (data.alsTimerHandle) {
        timTimerCancel(data.alsTimerHandle);
        data.alsTimerHandle = 0;
        data.alsReading = false;
    }

    data.alsOn = on;
    setMode(on, data.proxOn, (void *)(on ? SENSOR_STATE_ENABLING_ALS : SENSOR_STATE_DISABLING_ALS));

    return true;
}

static bool sensorFirmwareAls(void *cookie)
{
    sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateAls(uint32_t rate, uint64_t latency, void *cookie)
{
    osLog(LOG_INFO, "ROHM: sensorRateAls: rate=%ld Hz latency=%lld ns\n",
          rate / 1024, latency);

    if (data.alsTimerHandle)
        timTimerCancel(data.alsTimerHandle);
    data.alsTimerHandle = timTimerSet(sensorTimerLookupCommon(supportedRates, rateTimerVals, rate), 0, 50, alsTimerCallback, NULL, false);
    data.lastAlsSample.fdata = -FLT_MAX;
    osEnqueuePrivateEvt(EVT_SENSOR_ALS_TIMER, NULL, NULL, data.tid);
    sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);

    return true;
}

static bool sensorFlushAls(void *cookie)
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_ALS), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool sensorPowerProx(bool on, void *cookie)
{
    osLog(LOG_INFO, "ROHM: sensorPowerProx: %d\n", on);

    if (data.proxTimerHandle) {
        timTimerCancel(data.proxTimerHandle);
        data.proxTimerHandle = 0;
        data.proxReading = false;
    }

    data.proxOn = on;
    setMode(data.alsOn, on, (void *)(on ? SENSOR_STATE_ENABLING_PROX : SENSOR_STATE_DISABLING_PROX));

    return true;
}

static bool sensorFirmwareProx(void *cookie)
{
    sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateProx(uint32_t rate, uint64_t latency, void *cookie)
{
    osLog(LOG_INFO, "ROHM: sensorRateProx: rate=%ld Hz latency=%lld ns\n",
          rate / 1024, latency);

    if (data.proxTimerHandle)
        timTimerCancel(data.proxTimerHandle);
    data.proxTimerHandle = timTimerSet(sensorTimerLookupCommon(supportedRates, rateTimerVals, rate), 0, 50, proxTimerCallback, NULL, false);
    data.proxState = PROX_STATE_INIT;
    osEnqueuePrivateEvt(EVT_SENSOR_PROX_TIMER, NULL, NULL, data.tid);
    sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);

    return true;
}

static bool sensorFlushProx(void *cookie)
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_PROX), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorInfo sensorInfoAls =
{
    "ALS",
    supportedRates,
    SENS_TYPE_ALS,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_NONWAKEUP,
    20
};

static const struct SensorOps sensorOpsAls =
{
    sensorPowerAls,
    sensorFirmwareAls,
    sensorRateAls,
    sensorFlushAls,
    NULL
};

static const struct SensorInfo sensorInfoProx =
{
    "Proximity",
    supportedRates,
    SENS_TYPE_PROX,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_WAKEUP,
    300
};

static const struct SensorOps sensorOpsProx =
{
    sensorPowerProx,
    sensorFirmwareProx,
    sensorRateProx,
    sensorFlushProx,
    NULL
};

/*
 * Sensor i2c state machine
 */

static void __attribute__((unused)) sensorAlsFree(void *ptr)
{
}

static void __attribute__((unused)) sensorProxFree(void *ptr)
{
}

static void handle_i2c_event(int state)
{
    union EmbeddedDataPoint sample;
    bool sendData;

    switch (state) {
    case SENSOR_STATE_RESET:
        data.txrxBuf.bytes[0] = ROHM_RPR0521_REG_ID;
        i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                        data.txrxBuf.bytes, 1, &i2cCallback,
                        (void *)SENSOR_STATE_VERIFY_ID);
        break;
    case SENSOR_STATE_VERIFY_ID:
        /* Check the sensor ID */
        if (data.txrxBuf.bytes[0] != ROHM_RPR0521_ID) {
            osLog(LOG_INFO, "ROHM: not detected\n");
            sensorUnregister(data.alsHandle);
            sensorUnregister(data.proxHandle);
            break;
        }

        /* ALS gain and LED current */
        data.txrxBuf.bytes[0] = (ROHM_RPR0521_GAIN_ALS0 << 4) |
                          (ROHM_RPR0521_GAIN_ALS1 << 2) |
                          ROHM_RPR0521_LED_CURRENT_100MA;
        /* PS gain */
        data.txrxBuf.bytes[1] = (ROHM_RPR0521_GAIN_PS << 4) | 0x1;

        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                          &i2cCallback, (void *)SENSOR_STATE_INIT);
        break;

    case SENSOR_STATE_INIT:
        /* Start register */
        data.txrxBuf.bytes[0] = ROHM_RPR0521_REG_PS_OFFSET_LSB;
        data.txrxBuf.bytes[1] = ROHM_RPR0521_CAL_DEFAULT_OFFSET & 0xff;
        data.txrxBuf.bytes[2] = (ROHM_RPR0521_CAL_DEFAULT_OFFSET >> 8) & 0x3;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 3,
                          &i2cCallback, (void *)SENSOR_STATE_IDLE);
        break;

    case SENSOR_STATE_IDLE:
        sensorRegisterInitComplete(data.alsHandle);
        sensorRegisterInitComplete(data.proxHandle);
        break;

    case SENSOR_STATE_ENABLING_ALS:
        sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
        break;

    case SENSOR_STATE_ENABLING_PROX:
        sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
        break;

    case SENSOR_STATE_DISABLING_ALS:
        sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);
        break;

    case SENSOR_STATE_DISABLING_PROX:
        sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);
        break;

    case SENSOR_STATE_SAMPLING:
        /* TEST: log collected data
        osLog(LOG_INFO, "ROHM: sample ready: prox=%u als0=%u als1=%u\n",
              data.txrxBuf.sample.prox, data.txrxBuf.sample.als[0], data.txrxBuf.sample.als[1]);
        */

        if (data.alsOn && data.alsReading) {
            /* Create event */
            sample.fdata = getLuxFromAlsData(data.txrxBuf.sample.als[0],
                                             data.txrxBuf.sample.als[1]);
            if (data.lastAlsSample.idata != sample.idata) {
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_ALS), sample.vptr, NULL);
                data.lastAlsSample.fdata = sample.fdata;
            }
        }

        if (data.proxOn && data.proxReading) {
            /* Create event */
            sendData = true;
            if (data.proxState == PROX_STATE_INIT) {
                if (data.txrxBuf.sample.prox > ROHM_RPR0521_THRESHOLD_ASSERT_NEAR) {
                    sample.fdata = ROHM_RPR0521_REPORT_NEAR_VALUE;
                    data.proxState = PROX_STATE_NEAR;
                } else {
                    sample.fdata = ROHM_RPR0521_REPORT_FAR_VALUE;
                    data.proxState = PROX_STATE_FAR;
                }
            } else {
                if (data.proxState == PROX_STATE_NEAR &&
                    data.txrxBuf.sample.prox < ROHM_RPR0521_THRESHOLD_DEASSERT_NEAR) {
                    sample.fdata = ROHM_RPR0521_REPORT_FAR_VALUE;
                    data.proxState = PROX_STATE_FAR;
                } else if (data.proxState == PROX_STATE_FAR &&
                    data.txrxBuf.sample.prox > ROHM_RPR0521_THRESHOLD_ASSERT_NEAR) {
                    sample.fdata = ROHM_RPR0521_REPORT_NEAR_VALUE;
                    data.proxState = PROX_STATE_NEAR;
                } else {
                    sendData = false;
                }
            }

            if (sendData)
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_PROX), sample.vptr, NULL);
        }

        data.alsReading = false;
        data.proxReading = false;
        break;
    }
}

/*
 * Main driver entry points
 */

static bool init_app(uint32_t myTid)
{
    osLog(LOG_INFO, "ROHM: task starting\n");

    /* Set up driver private data */
    data.tid = myTid;
    data.alsOn = false;
    data.alsReading = false;
    data.proxOn = false;
    data.proxReading = false;

    /* Register sensors */
    data.alsHandle = sensorRegister(&sensorInfoAls, &sensorOpsAls, NULL, false);
    data.proxHandle = sensorRegister(&sensorInfoProx, &sensorOpsProx, NULL, false);

    osEventSubscribe(myTid, EVT_APP_START);

    return true;
}

static void end_app(void)
{
    sensorUnregister(data.alsHandle);
    sensorUnregister(data.proxHandle);

    i2cMasterRelease(I2C_BUS_ID);
}

static void handle_event(uint32_t evtType, const void* evtData)
{
    switch (evtType) {
    case EVT_APP_START:
        i2cMasterRequest(I2C_BUS_ID, I2C_SPEED);

        /* Reset chip */
        data.txrxBuf.bytes[0] = ROHM_RPR0521_REG_SYSTEM_CONTROL;
        data.txrxBuf.bytes[1] = SW_RESET_BIT;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                              &i2cCallback, (void *)SENSOR_STATE_RESET);
        break;

    case EVT_SENSOR_I2C:
        handle_i2c_event((int)evtData);
        break;

    case EVT_SENSOR_ALS_TIMER:
    case EVT_SENSOR_PROX_TIMER:
        /* Start sampling for a value */
        if (!data.alsReading && !data.proxReading) {
            data.txrxBuf.bytes[0] = ROHM_RPR0521_REG_PS_DATA_LSB;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                                data.txrxBuf.bytes, 6, &i2cCallback,
                                (void *)SENSOR_STATE_SAMPLING);
        }

        if (evtType == EVT_SENSOR_ALS_TIMER)
            data.alsReading = true;
        else
            data.proxReading = true;
        break;
    }
}

INTERNAL_APP_INIT(APP_ID_MAKE(APP_ID_VENDOR_GOOGLE, 10), 0, init_app, end_app, handle_event);

