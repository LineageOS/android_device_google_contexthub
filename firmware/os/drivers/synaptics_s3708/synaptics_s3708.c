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

#include <errno.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

#include <eventnums.h>
#include <gpio.h>
#include <heap.h>
#include <hostIntf.h>
#include <isr.h>
#include <i2c.h>
#include <nanohubPacket.h>
#include <sensors.h>
#include <seos.h>
#include <timer.h>
#include <util.h>

#include <plat/exti.h>
#include <plat/gpio.h>
#include <plat/syscfg.h>

#define S3708_APP_ID                APP_ID_MAKE(NANOHUB_VENDOR_GOOGLE, 13)
#define S3708_APP_VERSION           1

#define I2C_BUS_ID                  0
#define I2C_SPEED                   400000
#define I2C_ADDR                    0x20

#define S3708_ID                    0x34

#define S3708_REG_PAGE_SELECT       0xFF

#define S3708_REG_F01_DATA_BASE     0x06
#define S3708_INT_STATUS_LPWG       0x04

#define S3708_REG_DATA_BASE         0x08
#define S3708_REG_DATA_4_OFFSET     0x02
#define S3708_INT_STATUS_DOUBLE_TAP 0x03

#define S3708_REG_F01_CTRL_BASE     0x14
#define S3708_NORMAL_MODE           0x00
#define S3708_SLEEP_MODE            0x01

#define S3708_REG_CTRL_BASE         0x1b
#define S3708_REG_CTRL_20_OFFSET    0x07
#define S3708_REPORT_MODE_CONT      0x00
#define S3708_REPORT_MODE_LPWG      0x02

#define S3708_REG_PDT_BASE          0xee

#define MAX_PENDING_I2C_REQUESTS    4
#define MAX_I2C_TRANSFER_SIZE       8
#define MAX_I2C_RETRY_DELAY         250000000ull // 250 milliseconds
#define MAX_I2C_RETRY_COUNT         (15000000000ull / MAX_I2C_RETRY_DELAY) // 15 seconds
#define HACK_RETRY_SKIP_COUNT       1

#define ENABLE_DEBUG 0

#define INFO_PRINT(fmt, ...) osLog(LOG_INFO, "[S3708] " fmt, ##__VA_ARGS__)
#define ERROR_PRINT(fmt, ...) osLog(LOG_ERROR, "[S3708] " fmt, ##__VA_ARGS__)
#if ENABLE_DEBUG
#define DEBUG_PRINT(fmt, ...) INFO_PRINT(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) ((void)0)
#endif


#ifndef TOUCH_PIN
#error "TOUCH_PIN is not defined; please define in variant.h"
#endif

#ifndef TOUCH_IRQ
#error "TOUCH_IRQ is not defined; please define in variant.h"
#endif

enum SensorEvents
{
    EVT_SENSOR_I2C = EVT_APP_START + 1,
    EVT_SENSOR_TOUCH_INTERRUPT,
    EVT_SENSOR_RETRY_TIMER,
};

enum TaskState
{
    STATE_ENABLE_0,
    STATE_ENABLE_1,
    STATE_ENABLE_2,
    STATE_DISABLE_0,
    STATE_INT_HANDLE_0,
    STATE_INT_HANDLE_1,
    STATE_IDLE,
    STATE_CANCELLED,
};

struct I2cTransfer
{
    uint8_t txrxBuf[MAX_I2C_TRANSFER_SIZE];
    uint8_t state;
    bool inUse;
};

static struct TaskStruct
{
    struct Gpio *pin;
    struct ChainedIsr isr;
    uint32_t id;
    uint32_t handle;
    uint32_t retryTimerHandle;
    uint32_t retryCnt;
    struct I2cTransfer transfers[MAX_PENDING_I2C_REQUESTS];
    bool on;
} mTask;

static inline void enableInterrupt(bool enable)
{
    if (enable) {
        extiEnableIntGpio(mTask.pin, EXTI_TRIGGER_FALLING);
        extiChainIsr(TOUCH_IRQ, &mTask.isr);
    } else {
        extiUnchainIsr(TOUCH_IRQ, &mTask.isr);
        extiDisableIntGpio(mTask.pin);
    }
}

static bool touchIsr(struct ChainedIsr *localIsr)
{
    struct TaskStruct *data = container_of(localIsr, struct TaskStruct, isr);

    if (!extiIsPendingGpio(data->pin)) {
        return false;
    }

    osEnqueuePrivateEvt(EVT_SENSOR_TOUCH_INTERRUPT, NULL, NULL, data->id);

    extiClearPendingGpio(data->pin);

    return true;
}

static void i2cCallback(void *cookie, size_t tx, size_t rx, int err)
{
    if (err == 0) {
        osEnqueuePrivateEvt(EVT_SENSOR_I2C, cookie, NULL, mTask.id);
    } else {
        ERROR_PRINT("I2C error (%d)", err);
    }
}

static void retryTimerCallback(uint32_t timerId, void *cookie)
{
    osEnqueuePrivateEvt(EVT_SENSOR_RETRY_TIMER, cookie, NULL, mTask.id);
}

// Allocate a buffer and mark it as in use with the given state, or return NULL
// if no buffers available. Must *not* be called from interrupt context.
static struct I2cTransfer *allocXfer(uint8_t state)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(mTask.transfers); i++) {
        if (!mTask.transfers[i].inUse) {
            mTask.transfers[i].inUse = true;
            mTask.transfers[i].state = state;
            memset(mTask.transfers[i].txrxBuf, 0x00, sizeof(mTask.transfers[i].txrxBuf));
            return &mTask.transfers[i];
        }
    }

    ERROR_PRINT("Ran out of I2C buffers!");
    return NULL;
}

// Helper function to initiate the I2C transfer. Returns true is the transaction
// was successfully register by I2C driver. Otherwise, returns false.
static bool performXfer(struct I2cTransfer *xfer, size_t txBytes, size_t rxBytes)
{
    int ret;

    if ((txBytes > MAX_I2C_TRANSFER_SIZE) || (rxBytes > MAX_I2C_TRANSFER_SIZE)) {
        ERROR_PRINT("txBytes and rxBytes must be less than %d", MAX_I2C_TRANSFER_SIZE);
        return false;
    }

    if (rxBytes) {
        ret = i2cMasterTxRx(I2C_BUS_ID, I2C_ADDR, xfer->txrxBuf, txBytes, xfer->txrxBuf, rxBytes, i2cCallback, xfer);
    } else {
        ret = i2cMasterTx(I2C_BUS_ID, I2C_ADDR, xfer->txrxBuf, txBytes, i2cCallback, xfer);
    }

    if (ret != 0) {
        ERROR_PRINT("I2C transfer was not successful (error %d)!", ret);
    }

    return (ret == 0);
}

// Helper function to write a one byte register. Returns true if we got a
// successful return value from i2cMasterTx().
static bool writeRegister(uint8_t reg, uint8_t value, uint8_t state)
{
    struct I2cTransfer *xfer = allocXfer(state);

    if (xfer != NULL) {
        xfer->txrxBuf[0] = reg;
        xfer->txrxBuf[1] = value;
        return performXfer(xfer, 2, 0);
    }

    return false;
}

static bool setSleepEnable(bool enable, uint8_t state)
{
    return writeRegister(S3708_REG_F01_CTRL_BASE, enable ? S3708_SLEEP_MODE : S3708_NORMAL_MODE, state);
}

static bool setReportingMode(uint8_t mode, uint8_t state)
{
    struct I2cTransfer *xfer;

    xfer = allocXfer(state);
    if (xfer != NULL) {
        xfer->txrxBuf[0] = S3708_REG_CTRL_BASE + S3708_REG_CTRL_20_OFFSET;
        xfer->txrxBuf[1] = 0x00;
        xfer->txrxBuf[2] = 0x00;
        xfer->txrxBuf[3] = mode;
        return performXfer(xfer, 4, 0);
    }

    return false;
}

static bool callbackPower(bool on, void *cookie)
{
    bool ret;
    size_t i;

    INFO_PRINT("enable %d", on);

    // Cancel any pending I2C transactions by changing the callback state
    for (i = 0; i < ARRAY_SIZE(mTask.transfers); i++) {
        if (mTask.transfers[i].inUse) {
            mTask.transfers[i].state = STATE_CANCELLED;
        }
    }

    if (on) {
        mTask.retryCnt = 0;

        // Set page number to 0x00
        ret = writeRegister(S3708_REG_PAGE_SELECT, 0x00, STATE_ENABLE_0);
    } else {
        // Cancel any pending retries
        if (mTask.retryTimerHandle) {
            timTimerCancel(mTask.retryTimerHandle);
            mTask.retryTimerHandle = 0;
        }

        // Reset to continuous reporting mode
        ret = setReportingMode(S3708_REPORT_MODE_CONT, STATE_DISABLE_0);
    }

    if (ret) {
        mTask.on = on;
        enableInterrupt(mTask.on);
        sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, mTask.on, 0);
    }

    return ret;
}

static bool callbackFirmwareUpload(void *cookie)
{
    return sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
}

static bool callbackSetRate(uint32_t rate, uint64_t latency, void *cookie)
{
    return sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
}

static bool callbackFlush(void *cookie)
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_DOUBLE_TOUCH), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorInfo mSensorInfo = {
    .sensorName = "Double Touch",
    .sensorType = SENS_TYPE_DOUBLE_TOUCH,
    .numAxis = NUM_AXIS_EMBEDDED,
    .interrupt = NANOHUB_INT_WAKEUP,
    .minSamples = 20,
    .flags1 = SENSOR_INFO_FLAGS1_LOCAL_ONLY
};

static const struct SensorOps mSensorOps =
{
    .sensorPower = callbackPower,
    .sensorFirmwareUpload = callbackFirmwareUpload,
    .sensorSetRate = callbackSetRate,
    .sensorFlush = callbackFlush,
};

static void handleI2cEvent(struct I2cTransfer *xfer)
{
    struct I2cTransfer *nextXfer;
    union EmbeddedDataPoint sample;

    switch (xfer->state) {
        case STATE_ENABLE_0:
            // Disable sleep
            setSleepEnable(false, STATE_ENABLE_1);
            break;

        case STATE_ENABLE_1:
            // Verify that we can talk to the chip
            nextXfer = allocXfer(STATE_ENABLE_2);
            if (nextXfer != NULL) {
                nextXfer->txrxBuf[0] = S3708_REG_PDT_BASE;
                performXfer(nextXfer, 1, 1);
            }
            break;

        case STATE_ENABLE_2:
            DEBUG_PRINT("ID: 0x%02x", xfer->txrxBuf[0]);

            // HACK: DozeService reactivates pickup gesture before the screen
            // comes on, so we need to wait for some time after enabling before
            // trying to talk to touch controller. We may see the touch
            // controller on the first few samples and then have communication
            // switched off. So, wait HACK_RETRY_SKIP_COUNT samples before we
            // consider the transaction.
            if ((mTask.retryCnt < HACK_RETRY_SKIP_COUNT) || (xfer->txrxBuf[0] != S3708_ID)) {
                mTask.retryCnt++;
                if (mTask.retryCnt < MAX_I2C_RETRY_COUNT) {
                    mTask.retryTimerHandle = timTimerSet(MAX_I2C_RETRY_DELAY, 0, 50, retryTimerCallback, NULL, true);
                    if (!mTask.retryTimerHandle) {
                        ERROR_PRINT("failed to allocate timer");
                    }
                } else {
                    ERROR_PRINT("could not communicate with touch controller");
                }
            } else {
                // Set reporting control to lpwg mode
                setReportingMode(S3708_REPORT_MODE_LPWG, STATE_IDLE);
            }
            break;

        case STATE_DISABLE_0:
            // Enable sleep
            setSleepEnable(true, STATE_IDLE);
            break;

        case STATE_INT_HANDLE_0:
            // If the interrupt was from the LPWG function, read the function interrupt status register
            if (xfer->txrxBuf[1] & S3708_INT_STATUS_LPWG) {
                nextXfer = allocXfer(STATE_INT_HANDLE_1);
                if (nextXfer != NULL) {
                    nextXfer->txrxBuf[0] = S3708_REG_DATA_BASE + S3708_REG_DATA_4_OFFSET;
                    performXfer(nextXfer, 1, 5);
                }
            }
            break;

        case STATE_INT_HANDLE_1:
            // Verify the LPWG interrupt status
            if (xfer->txrxBuf[0] & S3708_INT_STATUS_DOUBLE_TAP) {
                DEBUG_PRINT("Sending event");
                sample.idata = 1;
                osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_DOUBLE_TOUCH), sample.vptr, NULL);
            }
            break;

        default:
            break;
    }

    xfer->inUse = false;
}

static void handleEvent(uint32_t evtType, const void* evtData)
{
    struct I2cTransfer *xfer;
    int ret;

    switch (evtType) {
        case EVT_APP_START:
            osEventUnsubscribe(mTask.id, EVT_APP_START);
            ret = i2cMasterRequest(I2C_BUS_ID, I2C_SPEED);
            // Since the i2c bus can be shared with other drivers, it is
            // possible that one of the other drivers requested the bus first.
            // Therefore, either 0 or -EBUSY is an acceptable return.
            if ((ret < 0) && (ret != -EBUSY)) {
                ERROR_PRINT("i2cMasterRequest() failed!");
            }
            sensorRegisterInitComplete(mTask.handle);
            break;

        case EVT_SENSOR_I2C:
            handleI2cEvent((struct I2cTransfer *)evtData);
            break;

        case EVT_SENSOR_TOUCH_INTERRUPT:
            if (mTask.on) {
                // Read the interrupt status register
                xfer = allocXfer(STATE_INT_HANDLE_0);
                if (xfer != NULL) {
                    xfer->txrxBuf[0] = S3708_REG_F01_DATA_BASE;
                    performXfer(xfer, 1, 2);
                }
            }
            break;

        case EVT_SENSOR_RETRY_TIMER:
            if (mTask.on) {
                // Set page number to 0x00
                writeRegister(S3708_REG_PAGE_SELECT, 0x00, STATE_ENABLE_0);
            }
            break;
    }
}

static bool startTask(uint32_t taskId)
{
    mTask.id = taskId;
    mTask.handle = sensorRegister(&mSensorInfo, &mSensorOps, NULL, false);

    mTask.pin = gpioRequest(TOUCH_PIN);
    gpioConfigInput(mTask.pin, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(mTask.pin);
    mTask.isr.func = touchIsr;

    osEventSubscribe(taskId, EVT_APP_START);
    return true;
}

static void endTask(void)
{
    enableInterrupt(false);
    extiUnchainIsr(TOUCH_IRQ, &mTask.isr);
    extiClearPendingGpio(mTask.pin);
    gpioRelease(mTask.pin);

    i2cMasterRelease(I2C_BUS_ID);

    sensorUnregister(mTask.handle);
}

INTERNAL_APP_INIT(S3708_APP_ID, S3708_APP_VERSION, startTask, endTask, handleEvent);
