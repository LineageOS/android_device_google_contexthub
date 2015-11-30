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

#ifndef HALL_PIN
#error "HALL_PIN is not defined; please define in variant.h"
#endif

#ifndef HALL_IRQ
#error "HALL_IRQ is not defined; please define in variant.h"
#endif

static struct SensorTask
{
    struct Gpio *pin;
    struct ChainedIsr isr;

    uint32_t id;
    uint32_t sensorHandle;

    bool on;
} mTask;

static bool hallIsr(struct ChainedIsr *localIsr)
{
    struct SensorTask *data = container_of(localIsr, struct SensorTask, isr);

    if (!extiIsPendingGpio(data->pin)) {
        return false;
    }

    if (data->on) {
        union EmbeddedDataPoint sample;
        bool pinState = gpioGet(data->pin);
        sample.idata = pinState ? HALL_REPORT_OPENED_VALUE :
            HALL_REPORT_CLOSED_VALUE;
        osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_HALL), sample.vptr, NULL);
    }

    extiClearPendingGpio(data->pin);
    return true;
}

static bool enableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    gpioConfigInput(pin, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(pin);
    extiEnableIntGpio(pin, EXTI_TRIGGER_BOTH);
    extiChainIsr(HALL_IRQ, isr);
    return true;
}

static bool disableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    extiUnchainIsr(HALL_IRQ, isr);
    extiDisableIntGpio(pin);
    return true;
}

static const struct SensorInfo mSensorInfo =
{
    "Hall",
    NULL,
    SENS_TYPE_HALL,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_WAKEUP,
    20
};

static bool hallPower(bool on)
{
    if (on) {
        extiClearPendingGpio(mTask.pin);
        enableInterrupt(mTask.pin, &mTask.isr);
    } else {
        disableInterrupt(mTask.pin, &mTask.isr);
        extiClearPendingGpio(mTask.pin);
    }

    mTask.on = on;
    sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    // report initial state of hall interrupt pin
    if (on) {
        union EmbeddedDataPoint sample;
        bool pinState = gpioGet(mTask.pin);
        sample.idata = pinState ? HALL_REPORT_OPENED_VALUE :
            HALL_REPORT_CLOSED_VALUE;
        osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_HALL), sample.vptr, NULL);
    }

    return true;
}

static bool hallFirmwareUpload()
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
}

static bool hallSetRate(uint32_t rate, uint64_t latency)
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
}

static bool hallFlush()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_HALL), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorOps mSensorOps =
{
    hallPower,
    hallFirmwareUpload,
    hallSetRate,
    hallFlush,
    NULL
};

static void handleEvent(uint32_t evtType, const void* evtData)
{
}

static bool startTask(uint32_t taskId)
{
    osLog(LOG_INFO, "HALL: task starting\n");

    mTask.id = taskId;
    mTask.sensorHandle = sensorRegister(&mSensorInfo, &mSensorOps);
    sensorRegisterInitComplete(mTask.sensorHandle);
    mTask.pin = gpioRequest(HALL_PIN);
    mTask.isr.func = hallIsr;

    return true;
}

static void endTask(void)
{
    disableInterrupt(mTask.pin, &mTask.isr);
    extiUnchainIsr(HALL_IRQ, &mTask.isr);
    extiClearPendingGpio(mTask.pin);
    gpioRelease(mTask.pin);
    sensorUnregister(mTask.sensorHandle);
    memset(&mTask, 0, sizeof(struct SensorTask));
}

INTERNAL_APP_INIT(0x0000000000000006ULL, startTask, endTask, handleEvent);
