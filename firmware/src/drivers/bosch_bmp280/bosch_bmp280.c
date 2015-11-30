#include <stdlib.h>
#include <string.h>
#include <float.h>

#include <eventnums.h>
#include <heap.h>
#include <hostIntf.h>
#include <i2c.h>
#include <nanohubPacket.h>
#include <sensors.h>
#include <seos.h>
#include <timer.h>

#define I2C_BUS_ID                      0
#define I2C_SPEED                       400000
#define I2C_ADDR                        0x76

#define BOSCH_BMP280_ID                 0x58

#define BOSCH_BMP280_REG_RESET          0x60
#define BOSCH_BMP280_REG_DIG_T1         0x88
#define BOSCH_BMP280_REG_ID             0xd0
#define BOSCH_BMP280_REG_CTRL_MEAS      0xf4
#define BOSCH_BMP280_REG_CONFIG         0xf5
#define BOSCH_BMP280_REG_PRES_MSB       0xf7

enum BMP280SensorEvents
{
    EVT_SENSOR_I2C = EVT_APP_START + 1,
    EVT_SENSOR_BARO_TIMER,
    EVT_SENSOR_TEMP_TIMER,
};

enum BMP280TaskState
{
    STATE_RESET,
    STATE_VERIFY_ID,
    STATE_AWAITING_COMP_PARAMS,
    STATE_IDLE,
    STATE_ENABLING_BARO,
    STATE_ENABLING_TEMP,
    STATE_DISABLING_BARO,
    STATE_DISABLING_TEMP,
    STATE_SAMPLING,
};

struct BMP280CompParams
{
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} __attribute__((packed));

static struct BMP280Task
{
    struct BMP280CompParams comp;

    uint32_t id;
    uint32_t baroHandle;
    uint32_t tempHandle;
    uint32_t baroTimerHandle;
    uint32_t tempTimerHandle;

    uint8_t txrxBuf[24];

    bool baroOn;
    bool tempOn;
    bool baroReading;
    bool tempReading;
} mTask;

static const uint32_t tempSupportedRates[] =
{
    SENSOR_HZ(0.1),
    SENSOR_HZ(1),
    SENSOR_HZ(5),
    SENSOR_HZ(10),
    SENSOR_HZ(25),
    0,
};

static const uint32_t baroSupportedRates[] =
{
    SENSOR_HZ(0.1),
    SENSOR_HZ(1),
    SENSOR_HZ(5),
    SENSOR_HZ(10),
    0
};

/* sensor callbacks from nanohub */

static void i2cCallback(void *cookie, size_t tx, size_t rx, int err)
{
    if (err == 0)
        osEnqueuePrivateEvt(EVT_SENSOR_I2C, cookie, NULL, mTask.id);
    else
        osLog(LOG_INFO, "BMP280: i2c error (%d)\n", err);
}

static void baroTimerCallback(uint32_t timerId, void *cookie)
{
    osEnqueuePrivateEvt(EVT_SENSOR_BARO_TIMER, cookie, NULL, mTask.id);
}

static void tempTimerCallback(uint32_t timerId, void *cookie)
{
    osEnqueuePrivateEvt(EVT_SENSOR_TEMP_TIMER, cookie, NULL, mTask.id);
}

static void setMode(bool on, void *cookie)
{
    mTask.txrxBuf[0] = BOSCH_BMP280_REG_CTRL_MEAS;
    if (on) {
        // temp: 2x oversampling, baro: 16x oversampling, power: normal
        mTask.txrxBuf[1] = (2 << 5) | (5 << 2) | 3;
    } else {
        // temp: 2x oversampling, baro: 16x oversampling, power: sleep
        mTask.txrxBuf[2] = (2 << 5) | (5 << 2) | 0;
    }
    i2cMasterTx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 2, &i2cCallback,
                cookie);
}

// TODO: only turn on the timer when enabled
static bool sensorPowerBaro(bool on)
{
    if (!on && mTask.baroTimerHandle) {
        timTimerCancel(mTask.baroTimerHandle);
        mTask.baroTimerHandle = 0;
        mTask.baroReading = false;
    }

    if (on != mTask.baroOn) {
        setMode(on || mTask.tempOn,
                (void*)(on ? STATE_ENABLING_BARO : STATE_DISABLING_BARO));
    }

    mTask.baroOn = on;

    return true;
}

static bool sensorFirmwareBaro()
{
    sensorSignalInternalEvt(mTask.baroHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateBaro(uint32_t rate, uint64_t latency)
{
    if (mTask.baroTimerHandle)
        timTimerCancel(mTask.baroTimerHandle);
    mTask.baroTimerHandle = timTimerSet(1024000000000ULL / rate, 0, 50, baroTimerCallback, NULL, false);
    osEnqueuePrivateEvt(EVT_SENSOR_BARO_TIMER, NULL, NULL, mTask.id);
    sensorSignalInternalEvt(mTask.baroHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool sensorFlushBaro()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_BARO), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool sensorPowerTemp(bool on)
{
    if (!on && mTask.tempTimerHandle) {
        timTimerCancel(mTask.tempTimerHandle);
        mTask.tempTimerHandle = 0;
        mTask.tempReading = false;
    }

    if (on != mTask.tempOn) {
        setMode(on || mTask.baroOn,
                (void*)(on ? STATE_ENABLING_TEMP : STATE_DISABLING_TEMP));
    }

    mTask.tempOn = on;

    return true;
}

static bool sensorFirmwareTemp()
{
    sensorSignalInternalEvt(mTask.tempHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateTemp(uint32_t rate, uint64_t latency)
{
    if (mTask.tempTimerHandle)
        timTimerCancel(mTask.tempTimerHandle);
    mTask.tempTimerHandle = timTimerSet(1024000000000ULL / rate, 0, 50, tempTimerCallback, NULL, false);
    osEnqueuePrivateEvt(EVT_SENSOR_TEMP_TIMER, NULL, NULL, mTask.id);
    sensorSignalInternalEvt(mTask.tempHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool sensorFlushTemp()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_TEMP), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorInfo sensorInfoBaro =
{
    "Pressure",
    baroSupportedRates,
    SENS_TYPE_BARO,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_NONWAKEUP,
    300
};

static const struct SensorOps sensorOpsBaro =
{
    sensorPowerBaro,
    sensorFirmwareBaro,
    sensorRateBaro,
    sensorFlushBaro,
};

static const struct SensorInfo sensorInfoTemp =
{
    "Temperature",
    tempSupportedRates,
    SENS_TYPE_TEMP,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_NONWAKEUP,
    20
};

static const struct SensorOps sensorOpsTemp =
{
    sensorPowerTemp,
    sensorFirmwareTemp,
    sensorRateTemp,
    sensorFlushTemp,
};

// Returns temperature in units of 0.01 degrees celsius.
static int32_t compensateTemp( int32_t adc_T, int32_t *t_fine)
{
    int32_t var1 =
        (((adc_T >> 3) - ((int32_t)mTask.comp.dig_T1 << 1))
            * (int32_t)mTask.comp.dig_T2) >> 11;

    int32_t tmp = (adc_T >> 4) - (int32_t)mTask.comp.dig_T1;

    int32_t var2 = (((tmp * tmp) >> 12) * (int32_t)mTask.comp.dig_T3) >> 14;

    int32_t sum = var1 + var2;

    *t_fine = sum;

    return (sum * 5 + 128) >> 8;
}

// Returns pressure in Pa in Q24.8 format.
static uint32_t compensateBaro( int32_t t_fine, int32_t adc_P)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)mTask.comp.dig_P6;
    var2 += (var1 * (int64_t)mTask.comp.dig_P5) << 17;
    var2 += ((int64_t)mTask.comp.dig_P4) << 35;

    var1 = ((var1 * var1 * (int64_t)mTask.comp.dig_P3) >> 8)
            + ((var1 * (int64_t)mTask.comp.dig_P2) << 12);

    var1 = ((1ll << 47) + var1) * ((int64_t)mTask.comp.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)mTask.comp.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)mTask.comp.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)mTask.comp.dig_P7) << 4);

    return (uint32_t)p;
}

static void getTempAndBaro(const uint8_t *tmp, float *pressure_Pa,
                           float *temp_centigrade)
{
    int32_t pres_adc =
        ((int32_t)tmp[0] << 12) | ((int32_t)tmp[1] << 4) | (tmp[2] >> 4);

    int32_t temp_adc =
        ((int32_t)tmp[3] << 12) | ((int32_t)tmp[4] << 4) | (tmp[5] >> 4);

    int32_t T_fine;
    int32_t temp = compensateTemp(temp_adc, &T_fine);
    uint32_t pres = compensateBaro(T_fine, pres_adc);

    *temp_centigrade = (float)temp * 0.01f;
    *pressure_Pa = (float)pres / 256.0f;
}

static void handleI2cEvent(enum BMP280TaskState state)
{
    union EmbeddedDataPoint sample;

    switch (state) {
        case STATE_RESET: {
            mTask.txrxBuf[0] = BOSCH_BMP280_REG_ID;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 1,
                            mTask.txrxBuf, 1, &i2cCallback,
                            (void*)STATE_VERIFY_ID);
            break;
        }

        case STATE_VERIFY_ID: {
            /* Check the sensor ID */
            if (mTask.txrxBuf[0] != BOSCH_BMP280_ID) {
                osLog(LOG_INFO, "BMP280: not detected\n");
                break;
            }

            /* Get compensation parameters */
            mTask.txrxBuf[0] = BOSCH_BMP280_REG_DIG_T1;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 1,
                            (uint8_t*)&mTask.comp, 24, &i2cCallback,
                            (void*)STATE_AWAITING_COMP_PARAMS);

            break;
        }

        case STATE_AWAITING_COMP_PARAMS: {
            mTask.txrxBuf[0] = BOSCH_BMP280_REG_CTRL_MEAS;
            mTask.txrxBuf[1] = (2 << 5) | (5 << 2);
            i2cMasterTx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 2,
                              &i2cCallback, (void*)STATE_IDLE);
            break;
        }

        case STATE_ENABLING_BARO: {
            sensorSignalInternalEvt(mTask.baroHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
            break;
        }

        case STATE_ENABLING_TEMP: {
            sensorSignalInternalEvt(mTask.tempHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
            break;
        }

        case STATE_DISABLING_BARO: {
            sensorSignalInternalEvt(mTask.baroHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);
            break;
        }

        case STATE_DISABLING_TEMP: {
            sensorSignalInternalEvt(mTask.tempHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);
            break;
        }

        case STATE_IDLE: {
            sensorRegisterInitComplete(mTask.baroHandle);
            sensorRegisterInitComplete(mTask.tempHandle);
            osLog(LOG_INFO, "BMP280: idle\n");
            break;
        }

        case STATE_SAMPLING: {
            float pressure_Pa, temp_centigrade;
            getTempAndBaro(mTask.txrxBuf, &pressure_Pa, &temp_centigrade);

            if (mTask.baroOn && mTask.baroReading) {
                sample.fdata = pressure_Pa * 0.01f;
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_BARO), sample.vptr, NULL);
            }

            if (mTask.tempOn && mTask.tempReading) {
                sample.fdata = temp_centigrade;
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_TEMP), sample.vptr, NULL);
            }

            mTask.baroReading = false;
            mTask.tempReading = false;

            break;
        }
    }
}

static void handleEvent(uint32_t evtType, const void* evtData)
{
    switch (evtType) {
        case EVT_APP_START:
        {
            osEventUnsubscribe(mTask.id, EVT_APP_START);
            i2cMasterRequest(I2C_BUS_ID, I2C_SPEED);

            /* Reset chip */
            mTask.txrxBuf[0] = BOSCH_BMP280_REG_RESET;
            mTask.txrxBuf[1] = 0xB6;
            i2cMasterTx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 2,
                        &i2cCallback, (void*)STATE_RESET);
            break;
        }

        case EVT_SENSOR_I2C:
        {
            handleI2cEvent((enum BMP280TaskState)evtData);
            break;
        }

        case EVT_SENSOR_BARO_TIMER:
        {
            /* Start sampling for a value */
            if (!mTask.baroReading && !mTask.tempReading) {
                mTask.txrxBuf[0] = BOSCH_BMP280_REG_PRES_MSB;
                i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 1,
                              mTask.txrxBuf, 6, &i2cCallback,
                              (void*)STATE_SAMPLING);
            }

            mTask.baroReading = true;
            break;
        }

        case EVT_SENSOR_TEMP_TIMER:
        {
            /* Start sampling for a value */
            if (!mTask.baroReading && !mTask.tempReading) {
                mTask.txrxBuf[0] = BOSCH_BMP280_REG_PRES_MSB;
                i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, mTask.txrxBuf, 1,
                              mTask.txrxBuf, 6, &i2cCallback,
                              (void*)STATE_SAMPLING);

            }

            mTask.tempReading = true;
            break;
        }
    }
}

static bool startTask(uint32_t taskId)
{
    osLog(LOG_INFO, "BMP280: task starting\n");

    mTask.id = taskId;

    /* Register sensors */
    mTask.baroHandle = sensorRegister(&sensorInfoBaro, &sensorOpsBaro);
    mTask.tempHandle = sensorRegister(&sensorInfoTemp, &sensorOpsTemp);

    osEventSubscribe(taskId, EVT_APP_START);

    return true;
}

static void endTask(void)
{

}

INTERNAL_APP_INIT(0x0000000000000005ULL, startTask, endTask, handleEvent);
