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

#include <eventnums.h>
#include <gpio.h>
#include <heap.h>
#include <hostIntf.h>
#include <isr.h>
#include <nanohubPacket.h>
#include <sensors.h>
#include <seos.h>
#include <timer.h>
#include <plat/inc/gpio.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>
#include <variant/inc/variant.h>

#define HALL_REPORT_OPENED_VALUE  0
#define HALL_REPORT_CLOSED_VALUE  1

#ifndef HALL_S_PIN
#error "HALL_S_PIN is not defined; please define in variant.h"
#endif
#ifndef HALL_S_IRQ
#error "HALL_S_IRQ is not defined; please define in variant.h"
#endif
#ifndef HALL_N_PIN
#error "HALL_N_PIN is not defined; please define in variant.h"
#endif
#ifndef HALL_N_IRQ
#error "HALL_N_IRQ is not defined; please define in variant.h"
#endif

static struct SensorTask
{
    struct Gpio *sPin;
    struct Gpio *nPin;
    struct ChainedIsr sIsr;
    struct ChainedIsr nIsr;

    uint32_t id;
    uint32_t sensorHandle;

    bool on;
} mTask;

static void hallReportState(bool on, bool sPinState, bool nPinState)
{
    union EmbeddedDataPoint sample;
    if (on) {
        sample.idata = (sPinState ? HALL_REPORT_OPENED_VALUE : HALL_REPORT_CLOSED_VALUE) +
            ((nPinState ? HALL_REPORT_OPENED_VALUE : HALL_REPORT_CLOSED_VALUE) << 1);
        osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_HALL), sample.vptr, NULL);
    }
}

static bool hallSouthIsr(struct ChainedIsr *localIsr)
{
    struct SensorTask *data = container_of(localIsr, struct SensorTask, sIsr);
    hallReportState(data->on, gpioGet(data->sPin), gpioGet(data->nPin));
    extiClearPendingGpio(data->sPin);
    return true;
}

static bool hallNorthIsr(struct ChainedIsr *localIsr)
{
    struct SensorTask *data = container_of(localIsr, struct SensorTask, nIsr);
    hallReportState(data->on, gpioGet(data->sPin), gpioGet(data->nPin));
    extiClearPendingGpio(data->nPin);
    return true;
}

static bool enableInterrupt(struct Gpio *pin, struct ChainedIsr *isr, IRQn_Type irqn)
{
    gpioConfigInput(pin, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(pin);
    extiEnableIntGpio(pin, EXTI_TRIGGER_BOTH);
    extiChainIsr(irqn, isr);
    return true;
}

static bool disableInterrupt(struct Gpio *pin, struct ChainedIsr *isr, IRQn_Type irqn)
{
    extiUnchainIsr(irqn, isr);
    extiDisableIntGpio(pin);
    return true;
}

static const struct SensorInfo mSensorInfo =
{
    .sensorName = "Hall",
    .sensorType = SENS_TYPE_HALL,
    .numAxis = NUM_AXIS_EMBEDDED,
    .interrupt = NANOHUB_INT_WAKEUP,
    .minSamples = 20
};

static bool hallPower(bool on, void *cookie)
{
    if (on) {
        extiClearPendingGpio(mTask.sPin);
        extiClearPendingGpio(mTask.nPin);
        enableInterrupt(mTask.sPin, &mTask.sIsr, HALL_S_IRQ);
        enableInterrupt(mTask.nPin, &mTask.nIsr, HALL_N_IRQ);
    } else {
        disableInterrupt(mTask.sPin, &mTask.sIsr, HALL_S_IRQ);
        disableInterrupt(mTask.nPin, &mTask.nIsr, HALL_N_IRQ);
        extiClearPendingGpio(mTask.sPin);
        extiClearPendingGpio(mTask.nPin);
    }

    mTask.on = on;
    sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    // report initial state of hall interrupt pin
    hallReportState(mTask.on, gpioGet(mTask.sPin), gpioGet(mTask.nPin));

    return true;
}

static bool hallFirmwareUpload(void *cookie)
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
}

static bool hallSetRate(uint32_t rate, uint64_t latency, void *cookie)
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
}

static bool hallFlush(void *cookie)
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_HALL), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorOps mSensorOps =
{
    .sensorPower = hallPower,
    .sensorFirmwareUpload = hallFirmwareUpload,
    .sensorSetRate = hallSetRate,
    .sensorFlush = hallFlush,
};

static void handleEvent(uint32_t evtType, const void* evtData)
{
}

static bool startTask(uint32_t taskId)
{
    osLog(LOG_INFO, "HALL: task starting\n");

    mTask.id = taskId;
    mTask.sensorHandle = sensorRegister(&mSensorInfo, &mSensorOps, NULL, true);
    mTask.sPin = gpioRequest(HALL_S_PIN);
    mTask.nPin = gpioRequest(HALL_N_PIN);
    mTask.sIsr.func = hallSouthIsr;
    mTask.nIsr.func = hallNorthIsr;

    return true;
}

static void endTask(void)
{
    disableInterrupt(mTask.sPin, &mTask.sIsr, HALL_S_IRQ);
    disableInterrupt(mTask.nPin, &mTask.nIsr, HALL_N_IRQ);
    extiUnchainIsr(HALL_S_IRQ, &mTask.sIsr);
    extiUnchainIsr(HALL_N_IRQ, &mTask.nIsr);
    extiClearPendingGpio(mTask.sPin);
    extiClearPendingGpio(mTask.nPin);
    gpioRelease(mTask.sPin);
    gpioRelease(mTask.nPin);
    sensorUnregister(mTask.sensorHandle);
    memset(&mTask, 0, sizeof(struct SensorTask));
}

INTERNAL_APP_INIT(APP_ID_MAKE(APP_ID_VENDOR_GOOGLE, 11), 0, startTask, endTask, handleEvent);
