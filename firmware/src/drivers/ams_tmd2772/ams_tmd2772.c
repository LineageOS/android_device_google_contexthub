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

#define DRIVER_NAME "tmd2772 "

#define I2C_BUS_ID 0
#define I2C_SPEED 400000
#define I2C_ADDR 0x39

#define AMS_TMD2772_ID                         0x39

#define AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT    0xa0

#define AMS_TMD2772_REG_ENABLE                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x00)
#define AMS_TMD2772_REG_ATIME                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x01)
#define AMS_TMD2772_REG_PTIME                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x02)
#define AMS_TMD2772_REG_WTIME                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x03)
#define AMS_TMD2772_REG_AILTL                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x04)
#define AMS_TMD2772_REG_AILTH                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x05)
#define AMS_TMD2772_REG_AIHTL                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x06)
#define AMS_TMD2772_REG_AIHTH                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x07)
#define AMS_TMD2772_REG_PILTL                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x08)
#define AMS_TMD2772_REG_PILTH                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x09)
#define AMS_TMD2772_REG_PIHTL                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0a)
#define AMS_TMD2772_REG_PIHTH                  (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0b)
#define AMS_TMD2772_REG_PERS                   (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0c)
#define AMS_TMD2772_REG_CONFIG                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0d)
#define AMS_TMD2772_REG_PPULSE                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0e)
#define AMS_TMD2772_REG_CONTROL                (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x0f)
#define AMS_TMD2772_REG_ID                     (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x12)
#define AMS_TMD2772_REG_STATUS                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x13)
#define AMS_TMD2772_REG_C0DATA                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x14)
#define AMS_TMD2772_REG_C0DATAH                (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x15)
#define AMS_TMD2772_REG_C1DATA                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x16)
#define AMS_TMD2772_REG_C1DATAH                (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x17)
#define AMS_TMD2772_REG_PDATAL                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x18)
#define AMS_TMD2772_REG_PDATAH                 (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x19)
#define AMS_TMD2772_REG_POFFSET                (AMS_TMD2772_CMD_TYPE_AUTO_INCREMENT | 0x1E)

#define AMS_TMD2772_ATIME_SETTING              0xed
#define AMS_TMD2772_ATIME_MS                   ((256 - AMS_TMD2772_ATIME_SETTING) * 2.73) // in milliseconds
#define AMS_TMD2772_PTIME_SETTING              0xff
#define AMS_TMD2772_PTIME_MS                   ((256 - AMS_TMD2772_PTIME_SETTING) * 2.73) // in milliseconds
#define AMS_TMD2772_WTIME_SETTING              0xee
#define AMS_TMD2772_WTIME_MS                   ((256 - AMS_TMD2772_WTIME_SETTING) * 2.73) // in milliseconds
#define AMS_TMD2772_PPULSE_SETTING             8

#define AMS_TMD2772_CAL_MAX_OFFSET             150

/* AMS_TMD2772_REG_ENABLE */
#define POWER_ON_BIT                           (1 << 0)
#define ALS_ENABLE_BIT                         (1 << 1)
#define PROX_ENABLE_BIT                        (1 << 2)
#define WAIT_ENABLE_BIT                        (1 << 3)

/* AMS_TMD2772_REG_STATUS */
#define PROX_INT_BIT                           (1 << 5)
#define ALS_INT_BIT                            (1 << 4)

// TODO: Tune for our parts
#define AMS_TMD2772_CAL_DEFAULT_OFFSET         0
#define AMS_TMD2772_CAL_MAX_OFFSET             150

#define AMS_TMD2772_REPORT_NEAR_VALUE          0.0f // centimeters
#define AMS_TMD2772_REPORT_FAR_VALUE           5.0f // centimeters

#define AMS_TMD2772_THRESHOLD_ASSERT_NEAR      700  // in PS units
#define AMS_TMD2772_THRESHOLD_DEASSERT_NEAR    400  // in PS units

#define AMS_TMD2772_TRANSMITTANCE_VISIBLE      10   // percent
#define AMS_TMD2772_TRANSMITTANCE_IR           75   // percent

#define MAX_FIFO_SIZE_ALS                      20
#define MAX_FIFO_SIZE_PROX                     300


/* Private driver events */
enum SensorEvents
{
    EVT_SENSOR_I2C = EVT_NO_FIRST_USER_EVENT,
    EVT_SENSOR_ALS_TIMER,
    EVT_SENSOR_PROX_TIMER,
};

/* I2C state machine */
enum SensorState
{
    SENSOR_STATE_VERIFY_ID,
    SENSOR_STATE_INIT,

    SENSOR_STATE_CALIBRATE_RESET,
    SENSOR_STATE_CALIBRATE_START,
    SENSOR_STATE_CALIBRATE_ENABLING,
    SENSOR_STATE_CALIBRATE_POLLING_STATUS,
    SENSOR_STATE_CALIBRATE_AWAITING_SAMPLE,
    SENSOR_STATE_CALIBRATE_DISABLING,

    SENSOR_STATE_ENABLING_ALS,
    SENSOR_STATE_ENABLING_PROX,
    SENSOR_STATE_DISABLING_ALS,
    SENSOR_STATE_DISABLING_PROX,

    SENSOR_STATE_IDLE,
    SENSOR_STATE_SAMPLING,
};

struct SensorSample
{
    uint64_t time;
    float sample;
} __attribute__((packed));

struct SensorData
{
    uint32_t tid;
    enum SensorState state;
    union {
        uint8_t bytes[16];
        struct {
            uint8_t status;
            uint16_t als[2];
            uint16_t prox;
        } sample;
        struct {
            uint16_t prox;
        } calibration;
    } txrxBuf;

    uint32_t count; /* number of samples acquired */
    uint64_t sampleTimeNs; /* time of current sample */

    uint32_t alsHandle;
    uint32_t proxHandle;
    uint32_t alsTimerHandle;
    uint32_t proxTimerHandle;

    bool alsOn;
    bool alsReading;
    bool proxOn;
    bool proxReading;

    uint16_t alsHead;
    uint16_t alsPrev;
    struct SensorSample *alsFIFO;
    uint16_t proxHead;
    uint16_t proxPrev;
    struct SensorSample *proxFIFO;

    struct SensorSample flush;

    uint8_t calibrationSampleCount;
    uint32_t calibrationSampleTotal;
} data;

/* TODO: check rates are supported */
static const uint32_t supportedRates[] =
{
    SENSOR_HZ(4),
    SENSOR_HZ(5),
    SENSOR_HZ(10),
    0,
};

/*
 * Helper functions
 */

static void i2cCallback(void *cookie, size_t tx, size_t rx, int err)
{
    if (err == 0)
        osEnqueuePrivateEvt(EVT_SENSOR_I2C, cookie, NULL, data.tid);
    else
        osLog(LOG_INFO, DRIVER_NAME "i2c error (%d)\n", err);
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
    float als0_boost = als0 * (100.0f / AMS_TMD2772_TRANSMITTANCE_VISIBLE);
    float als1_boost = als1 * (100.0f / AMS_TMD2772_TRANSMITTANCE_IR);

    float fudge_factor = 5;

    float cpl = AMS_TMD2772_ATIME_MS / 20.0f;
    float lux1 = (als0_boost - (1.75f * als1_boost)) / cpl;
    float lux2 = ((0.63f * als0_boost) - als1_boost) / cpl;
    if ((lux1 > lux2) && (lux1 > 0)) {
        return lux1 * fudge_factor;
    } else if (lux2 > 0) {
        return lux2 * fudge_factor;
    } else {
        return 0.0f;
    }
}

static void setMode(bool alsOn, bool proxOn)
{
    data.txrxBuf.bytes[0] = AMS_TMD2772_REG_ENABLE;
    data.txrxBuf.bytes[1] = POWER_ON_BIT | WAIT_ENABLE_BIT |
                            (alsOn ? ALS_ENABLE_BIT : 0) | (proxOn ? PROX_ENABLE_BIT : 0);
    i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                &i2cCallback, NULL);
}

static bool sensorPowerAls(bool on)
{
    osLog(LOG_INFO, DRIVER_NAME "als %s\n", on ? "on" : "off");

    if (data.alsOn != on) {
        setMode(on, data.proxOn);
        data.state = on ? SENSOR_STATE_ENABLING_ALS : SENSOR_STATE_DISABLING_ALS;
    }

    return true;
}

static bool sensorFirmwareAls()
{
    sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateAls(uint32_t rate, uint64_t latency)
{
    if (data.alsTimerHandle)
        timTimerCancel(data.alsTimerHandle);
    data.alsTimerHandle = timTimerSet(1024000000000ULL / rate, 0, 50, alsTimerCallback, NULL, false);
    data.alsFIFO[data.alsPrev].sample = -FLT_MAX;
    osEnqueuePrivateEvt(EVT_SENSOR_ALS_TIMER, NULL, NULL, data.tid);
    sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool sensorFlushAls()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_ALS), &data.flush, NULL);
}

static bool sensorPowerProx(bool on)
{
    osLog(LOG_INFO, DRIVER_NAME "prox %s\n", on ? "on" : "off");

    if (data.proxOn != on) {
        setMode(data.alsOn, on);
        data.state = on ? SENSOR_STATE_ENABLING_PROX : SENSOR_STATE_DISABLING_PROX;
    }

    return true;
}

static bool sensorFirmwareProx()
{
    sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool sensorRateProx(uint32_t rate, uint64_t latency)
{
    if (data.proxTimerHandle)
        timTimerCancel(data.proxTimerHandle);
    data.proxTimerHandle = timTimerSet(1024000000000ULL / rate, 0, 50, proxTimerCallback, NULL, false);
    data.proxFIFO[data.proxPrev].sample = -FLT_MAX;
    osEnqueuePrivateEvt(EVT_SENSOR_PROX_TIMER, NULL, NULL, data.tid);
    sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool sensorFlushProx()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_PROX), &data.flush, NULL);
}

static const struct SensorInfo sensorInfoAls =
{
    "ALS",
    supportedRates,
    SENS_TYPE_ALS
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
    SENS_TYPE_PROX
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

static void handle_calibration_event(void) {
    switch (data.state) {
    case SENSOR_STATE_CALIBRATE_RESET:
        data.calibrationSampleCount = 0;
        data.calibrationSampleTotal = 0;
        /* Intentional fall-through */

    case SENSOR_STATE_CALIBRATE_START:
        data.state = SENSOR_STATE_CALIBRATE_ENABLING;
        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_ENABLE;
        data.txrxBuf.bytes[1] = POWER_ON_BIT | PROX_ENABLE_BIT;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                    &i2cCallback, NULL);
        break;

    case SENSOR_STATE_CALIBRATE_ENABLING:
        data.state = SENSOR_STATE_CALIBRATE_POLLING_STATUS;
        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_STATUS;
        i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                      data.txrxBuf.bytes, 1, &i2cCallback, NULL);
        break;

    case SENSOR_STATE_CALIBRATE_POLLING_STATUS:
        if (data.txrxBuf.bytes[0] & PROX_INT_BIT) {
            /* Done */
            data.state = SENSOR_STATE_CALIBRATE_AWAITING_SAMPLE;
            data.txrxBuf.bytes[0] = AMS_TMD2772_REG_PDATAL;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                          data.txrxBuf.bytes, 2, &i2cCallback, NULL);
        } else {
            /* Poll again; go back to previous state */
            data.state = SENSOR_STATE_CALIBRATE_ENABLING;
            handle_calibration_event();
        }
        break;

    case SENSOR_STATE_CALIBRATE_AWAITING_SAMPLE:
        data.calibrationSampleCount++;
        data.calibrationSampleTotal += data.txrxBuf.calibration.prox;

        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_ENABLE;
        data.txrxBuf.bytes[1] = 0x00;
        data.state = SENSOR_STATE_CALIBRATE_DISABLING;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                    &i2cCallback, NULL);
        break;

    case SENSOR_STATE_CALIBRATE_DISABLING:
        if (data.calibrationSampleCount >= 20) {
            /* Done, calculate calibration */
            uint16_t average = data.calibrationSampleTotal / data.calibrationSampleCount;
            uint16_t crosstalk = (average > 0x7f) ? 0x7f : average;

            data.txrxBuf.bytes[0] = AMS_TMD2772_REG_POFFSET;
            data.txrxBuf.bytes[1] = crosstalk;
            data.state = SENSOR_STATE_IDLE;
            i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 2,
                        &i2cCallback, NULL);
        } else {
            /* Get another sample; go back to earlier state */
            data.state = SENSOR_STATE_CALIBRATE_START;
            handle_calibration_event();
        }
        break;

    default:
        break;
    }
}

static void handle_i2c_event(void)
{
    switch (data.state) {
    case SENSOR_STATE_VERIFY_ID:
        /* Check the sensor ID */
        if (data.txrxBuf.bytes[0] != AMS_TMD2772_ID) {
            osLog(LOG_INFO, DRIVER_NAME "not detected\n");
            break;
        }

        /* Start address */
        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_ENABLE;
        /* ENABLE */
        data.txrxBuf.bytes[1] = 0x00;
        /* ATIME */
        data.txrxBuf.bytes[2] = AMS_TMD2772_ATIME_SETTING;
        /* PTIME */
        data.txrxBuf.bytes[3] = AMS_TMD2772_PTIME_SETTING;
        /* WTIME */
        data.txrxBuf.bytes[4] = AMS_TMD2772_WTIME_SETTING;
        data.state = SENSOR_STATE_INIT;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 5,
                    &i2cCallback, NULL);
        break;

    case SENSOR_STATE_INIT:
        /* Start address */
        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_PERS;
        /* PERS */
        data.txrxBuf.bytes[1] = 0x00;
        /* CONFIG */
        data.txrxBuf.bytes[2] = 0x00;
        /* PPULSE */
        data.txrxBuf.bytes[3] = AMS_TMD2772_PPULSE_SETTING;
        /* CONTROL */
        data.txrxBuf.bytes[4] = 0x20;
        data.state = SENSOR_STATE_IDLE;
        i2cMasterTx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 5,
                    &i2cCallback, NULL);
        break;

    case SENSOR_STATE_ENABLING_ALS:
        data.alsOn = true;
        data.state = SENSOR_STATE_IDLE;

        /* Signal OS that the sensor was enabled */
        sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
        break;

    case SENSOR_STATE_ENABLING_PROX:
        data.proxOn = true;
        data.state = SENSOR_STATE_IDLE;

        /* Signal OS that the sensor was enabled */
        sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, true, 0);
        break;

    case SENSOR_STATE_DISABLING_ALS:
        /* Signal OS that the sensor was disabled */
        sensorSignalInternalEvt(data.alsHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);

        /* Cancel timer */
        if (data.alsTimerHandle) {
            timTimerCancel(data.alsTimerHandle);
            data.alsTimerHandle = 0;
        }

        data.alsOn = false;
        data.state = SENSOR_STATE_IDLE;
        break;

    case SENSOR_STATE_DISABLING_PROX:
        /* Signal OS that the sensor was disabled */
        sensorSignalInternalEvt(data.proxHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, false, 0);

        /* Cancel timer */
        if (data.proxTimerHandle) {
            timTimerCancel(data.proxTimerHandle);
            data.proxTimerHandle = 0;
        }

        data.proxOn = false;
        data.state = SENSOR_STATE_IDLE;
        break;

    case SENSOR_STATE_IDLE:
        osLog(LOG_INFO, DRIVER_NAME "idle\n");

        break;

    case SENSOR_STATE_SAMPLING:
        /* TEST: log collected data */
        osLog(LOG_INFO, DRIVER_NAME "sample ready: status=%02x prox=%u als0=%u als1=%u\n",
              data.txrxBuf.sample.status, data.txrxBuf.sample.prox,
              data.txrxBuf.sample.als[0], data.txrxBuf.sample.als[1]);

        data.sampleTimeNs = timGetTime();
        data.count++;

        if (data.alsOn && data.alsReading) {
            /* Create event */
            data.alsFIFO[data.alsHead].time = data.sampleTimeNs;
            data.alsFIFO[data.alsHead].sample = getLuxFromAlsData(data.txrxBuf.sample.als[0],
                                                               data.txrxBuf.sample.als[1]);
            if (memcmp(&data.alsFIFO[data.alsHead].sample, &data.alsFIFO[data.alsPrev].sample, sizeof(float)) != 0) {
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_ALS), &data.alsFIFO[data.alsHead], sensorAlsFree);
                data.alsPrev = data.alsHead;
                data.alsHead = (data.alsHead + 1) % MAX_FIFO_SIZE_ALS;
                hostIntfSetInterrupt(/*NANOHUB_INT_NONWAKEUP*/NANOHUB_INT_WAKEUP);
            }
        }

        if (data.proxOn && data.proxReading) {
            /* Create event */
            data.proxFIFO[data.proxHead].time = data.sampleTimeNs;
            if (data.txrxBuf.sample.prox > AMS_TMD2772_THRESHOLD_ASSERT_NEAR) {
                data.proxFIFO[data.proxHead].sample = AMS_TMD2772_REPORT_NEAR_VALUE;
            } else {
                data.proxFIFO[data.proxHead].sample = AMS_TMD2772_REPORT_FAR_VALUE;
            }
            if (memcmp(&data.proxFIFO[data.proxHead].sample, &data.proxFIFO[data.proxPrev].sample, sizeof(float)) != 0) {
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_PROX), &data.proxFIFO[data.proxHead], sensorProxFree);
                data.proxPrev = data.proxHead;
                data.proxHead = (data.proxHead + 1) % MAX_FIFO_SIZE_PROX;
                hostIntfSetInterrupt(NANOHUB_INT_WAKEUP);
            }
        }

        data.alsReading = false;
        data.proxReading = false;
        /* Go back to idle state because timer is repeating */
        data.state = SENSOR_STATE_IDLE;
        break;

    default:
        handle_calibration_event();
    }
}

/*
 * Main driver entry points
 */

static bool init_app(uint32_t myTid)
{
    osLog(LOG_INFO, DRIVER_NAME "task starting\n");

    /* Set up driver private data */
    data.tid = myTid;
    data.count = 0;
    data.alsOn = false;
    data.alsReading = false;
    data.proxOn = false;
    data.proxReading = false;

    /* Allocate FIFOs */
    data.alsHead = 0;
    data.alsPrev = MAX_FIFO_SIZE_ALS - 1;
    data.alsFIFO = heapAlloc(MAX_FIFO_SIZE_ALS * sizeof(struct SensorSample));
    data.alsFIFO[data.alsPrev].sample = -FLT_MAX;

    data.proxHead = 0;
    data.proxPrev = MAX_FIFO_SIZE_PROX - 1;
    data.proxFIFO = heapAlloc(MAX_FIFO_SIZE_PROX * sizeof(struct SensorSample));
    data.proxFIFO[data.proxPrev].sample = -FLT_MAX;

    memset(&data.flush, 0x00, sizeof(data.flush));

    /* Register sensors */
    data.alsHandle = sensorRegister(&sensorInfoAls, &sensorOpsAls);
    data.proxHandle = sensorRegister(&sensorInfoProx, &sensorOpsProx);

    osEventSubscribe(myTid, EVT_APP_START);

	return true;
}

static void end_app(void)
{
    sensorUnregister(data.alsHandle);
    sensorUnregister(data.proxHandle);

    heapFree(data.alsFIFO);
    heapFree(data.proxFIFO);

    i2cMasterRelease(I2C_BUS_ID);
}

static void handle_event(uint32_t evtType, const void* evtData)
{
    switch (evtType) {
    case EVT_APP_START:
        osEventUnsubscribe(data.tid, EVT_APP_START);
        i2cMasterRequest(I2C_BUS_ID, I2C_SPEED);

        /* TODO: reset chip first */

        data.txrxBuf.bytes[0] = AMS_TMD2772_REG_ID;
        data.state = SENSOR_STATE_VERIFY_ID;
        i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                        data.txrxBuf.bytes, 1, &i2cCallback, NULL);
        break;

    case EVT_SENSOR_I2C:
        handle_i2c_event();
        break;

    case EVT_SENSOR_ALS_TIMER:
        /* Start sampling for a value */
        if (!data.alsReading && !data.proxReading) {
            data.alsReading = true;
            /* TODO: can we avoid reading status so we can just read the appropriate sensor? */
            /* Start sampling for a value */
            data.txrxBuf.bytes[0] = AMS_TMD2772_REG_STATUS;
            data.state = SENSOR_STATE_SAMPLING;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                                data.txrxBuf.bytes, 7, &i2cCallback, NULL);
        }
        break;

    case EVT_SENSOR_PROX_TIMER:
        /* Start sampling for a value */
        if (!data.alsReading && !data.proxReading) {
            data.proxReading = true;
            /* TODO: can we avoid reading status so we can just read the appropriate sensor? */
            /* Start sampling for a value */
            data.txrxBuf.bytes[0] = AMS_TMD2772_REG_STATUS;
            data.state = SENSOR_STATE_SAMPLING;
            i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, data.txrxBuf.bytes, 1,
                                data.txrxBuf.bytes, 7, &i2cCallback, NULL);
        }
        break;
    }
}

INTERNAL_APP_INIT(0x60060001, init_app, end_app, handle_event);
